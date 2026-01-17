#pragma once
/**
 * @file    steering.hpp
 * @brief   CRTP base class for path enumeration & integration.
 *
 * This version removes collision/workspace concerns from steering.
 * It exposes:
 *   • candidates(a,b)     – enumerate all geometric candidates (no constraints)
 *   • shortestPath(a,b)   – discretised path of the shortest geometric candidate
 *   • ensureStates(...)   – lazily integrate a specific candidate
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <optional>
#include <span>
#include <vector>
#include <utility>

#include <arcgen/core/control.hpp>
#include <arcgen/core/math.hpp>
#include <arcgen/core/numeric.hpp>
#include <arcgen/core/state.hpp>
#include <arcgen/steering/path.hpp>

namespace arcgen::steering
{
    using arcgen::core::Control;
    using arcgen::core::State;
    using arcgen::core::normalizeAngleSigned;
    using arcgen::core::computeArcEndpoint;
    using arcgen::core::computeLineEndpoint;

    template <class, std::size_t> class SteeringBase;

    /**
     * @brief Parameterised arc description used by steering policies.
     *
     * @tparam Segment  Segment enum/type used by the policy.
     * @tparam N        Maximum number of segments in the arc pattern.
     */
    template <class Segment, std::size_t N> struct Arc
    {
        const std::array<Segment, N> *type{nullptr}; ///< Points into a static pattern table.
        std::array<double, N> length{};              ///< Signed/unsigned segment parameters.

        /**
         * @brief Sum of absolute segment parameters.
         * @return Total length in the parameter domain (|s_i|).
         */
        [[nodiscard]] constexpr double total () const noexcept
        {
            return std::transform_reduce (length.begin (), length.end (), 0.0, std::plus<>{}, [] (double v) { return std::fabs (v); });
        }
    };

    /**
     * @brief Concept for steering policies (matching the CRTP hook).
     *
     * Kept separate from the base class so it doesn't force early instantiation.
     */
    template <class D, std::size_t N>
    concept SteeringPolicy = requires (const D &d, const State &a, const State &b) {
        { d.getArcs (a, b) } -> std::same_as<std::vector<Arc<typename SteeringBase<D, N>::Segment, N>>>;
    };

    /**
     * @brief CRTP base that converts policy arcs to concrete controls,
     *        computes per-segment AABBs, and integrates paths.
     *
     * @tparam Derived  Concrete steering policy (must provide `getArcs(State,State)`).
     * @tparam N        Maximum number of segments in a candidate.
     */
    template <class Derived, std::size_t N> class SteeringBase
    {
      public:
        using PathType = Path<N>;
        static constexpr std::size_t kSegments = N;

        /**
         * @brief Construct with geometric parameters.
         * @param rMin           Minimal turning radius (meters).
         * @param discretisation Integration step size (meters). Internally capped.
         */
        constexpr SteeringBase (double rMin, double discretisation) noexcept : rMin_ (rMin), ds_ (discretisation > 0.0 ? discretisation : 1e-2) {}

        /**
         * @brief Segment types used by policies.
         *
         * NOTE: values kept as-is for backwards compatibility with existing code.
         */
        enum class Segment : std::uint8_t
        {
            LEFT,
            RIGHT,
            STRAIGHT,
            NOP
        };

        /*────────── compile-time accessors ─────────────────────────────*/

        /// @brief Signed curvature magnitude 1 / rMin.
        [[nodiscard]] constexpr double getKappa () const noexcept { return 1.0 / rMin_; }

        /// @brief Minimal turning radius [m].
        [[nodiscard]] constexpr double getRadiusMin () const noexcept { return rMin_; }

        /// @brief Integration step size [m].
        [[nodiscard]] constexpr double getStep () const noexcept { return ds_; }

        /*────────── uniform public API (CRTP constrained) ─────────────*/

        /**
         * @brief Enumerate all geometric candidates (no constraints).
         * @param a Start state (only x/y/heading are used).
         * @param b Goal state  (only x/y/heading are used).
         * @return Vector of candidates; each contains concrete controls and per-segment AABBs.
         */
        [[nodiscard]] std::vector<Path<N>> candidates (State a, State b) const noexcept (noexcept (static_cast<const Derived &> (*this).getArcs (a, b)))
            requires SteeringPolicy<Derived, N>
        {
            a.heading = normalizeAngleSigned (a.heading);
            b.heading = normalizeAngleSigned (b.heading);

            std::vector<Path<N>> out;
            const auto arcs = d ().getArcs (a, b);
            if (arcs.empty ())
                return out;
            out.reserve (arcs.size ());

            for (const auto &arc : arcs)
            {
                if (!std::isfinite (arc.total ()))
                    continue;

                Path<N> cand;

                for (std::size_t i = 0; i < N; ++i)
                {
                    const Segment seg = (*arc.type)[i];
                    if (seg == Segment::NOP)
                        break;

                    Control ctl{};
                    switch (seg)
                    {
                        case Segment::LEFT:
                            ctl.curvature = getKappa ();
                            break;
                        case Segment::RIGHT:
                            ctl.curvature = -getKappa ();
                            break;
                        case Segment::STRAIGHT:
                            ctl.curvature = 0.0;
                            break;
                        case Segment::NOP:
                            continue;
                    }
                    ctl.arcLength = getRadiusMin () * arc.length[i];

                    cand.controls.push_back (ctl);
                }

                if (cand.controls.n > 0)
                    out.emplace_back (std::move (cand));
            }

            return out;
        }

        /**
         * @brief Discretised path of the shortest *geometric* candidate (no constraints).
         * @param a Start state.
         * @param b Goal state.
         * @return Candidate with lazily generated states filled in. Returns empty Path on failure.
         */
        [[nodiscard]] Path<N> shortestPath (State &a, State &b) const noexcept (noexcept (static_cast<const Derived &> (*this).getArcs (a, b)))
            requires SteeringPolicy<Derived, N>
        {
            auto all = candidates (a, b);
            if (all.empty ())
                return {};

            auto it = std::ranges::min_element (all, [] (const Path<N> &L, const Path<N> &R) { return L.length () < R.length (); });

            if (!it->states)
                it->states = integrate (a, it->controls.view ());

            return *it;
        }

        /**
         * @brief Ensure discretised states exist for the given candidate (lazy integration).
         * @param start  Start state used for integration.
         * @param cand   Candidate to augment (states are filled in if missing).
         */
        void ensureStates (const State &start, const Path<N> &cand) const
        {
            if (!cand.states)
                cand.states = integrate (start, cand.controls.view ());
        }

      protected:
        /*────────── helpers available to the policy ────────────────────*/

        /// @brief Downcast to the derived policy (non-const).
        constexpr decltype (auto) d () noexcept { return static_cast<Derived &> (*this); }
        /// @brief Downcast to the derived policy (const).
        constexpr decltype (auto) d () const noexcept { return static_cast<const Derived &> (*this); }

        /**
         * @brief One forward-Euler step of size @p ds along a control.
         * @param st   Starting state.
         * @param ctl  Control to advance along.
         * @param ds   Step length (meters), non-negative.
         * @return New state at distance @p ds along the control.
         */
        [[nodiscard]] State advance (const State &st, const Control &ctl, double ds) const
        {
            State nxt{};
            nxt.setDirectionFrom (ctl.arcLength);

            if (std::fabs (ctl.curvature) > arcgen::core::CURVATURE_TOL)
            {
                std::tie (nxt.x, nxt.y, nxt.heading) = computeArcEndpoint (st.x, st.y, st.heading, ctl.curvature, std::to_underlying (nxt.direction), ds);
            }
            else
            {
                std::tie (nxt.x, nxt.y) = computeLineEndpoint (st.x, st.y, st.heading, std::to_underlying (nxt.direction), ds);
                nxt.heading = st.heading;
            }
            // Carry curvature forward; callers usually set st.curvature = ctl.curvature beforehand.
            nxt.curvature = st.curvature;
            return nxt;
        }

        /**
         * @brief Integrate a whole sequence of controls into discretised states.
         * @param start  Starting state.
         * @param seq    Sequence of controls (span view).
         * @return Discretised states (including the state at the beginning of every segment
         *         and every subsequent step of size @ref getStep()).
         */
        [[nodiscard]] std::vector<State> integrate (const State &start, std::span<const Control> seq) const
        {
            std::vector<State> out;
            State cur = start;

            // Reservation heuristic.
            std::size_t reserve = 0;
            for (const Control &c : seq)
                reserve += static_cast<std::size_t> (std::ceil (std::fabs (c.arcLength) / static_cast<double> (getStep ())));
            out.reserve (reserve);

            for (const Control &ctl : seq)
            {
                double seg = 0.0;
                const double absLen = std::fabs (ctl.arcLength);

                cur.curvature = ctl.curvature;
                cur.setDirectionFrom (ctl.arcLength);
                out.push_back (cur);

                while (seg < absLen - arcgen::core::epsilon)
                {
                    const double h = std::min (static_cast<double> (getStep ()), absLen - seg);
                    cur = advance (cur, ctl, h);
                    seg += h;
                    out.push_back (cur);
                }
            }
            return out;
        }

      private:
        /*────────── data ───────────────────────────────────────────────*/
        double rMin_; ///< Minimal turning radius [m].
        double ds_;   ///< Path discretisation step [m].
    };

} // namespace arcgen::steering
