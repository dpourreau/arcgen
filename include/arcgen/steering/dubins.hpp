#pragma once
/**
 * @file   dubins.hpp
 * @brief  Dubins 3-segment shortest-path generator (CRTP policy).
 *
 * Produces the classic Dubins shortest paths between two SE(2) poses under
 * the constraint of forward-only motion and bounded curvature. This class
 * enumerates geometric candidates as parameterized arcs; the CRTP base
 * converts them into concrete controls and can integrate them to states.
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>
#include <vector>

#include <arcgen/core/control.hpp>
#include <arcgen/core/math.hpp>
#include <arcgen/core/state.hpp>
#include <arcgen/steering/steering.hpp>

namespace arcgen::steering
{
    /*───────────────────────────────────────────────────────────────────────*
     * Compile-time configuration
     *───────────────────────────────────────────────────────────────────────*/
    /// @brief Number of segments in a Dubins shortest path.
    inline constexpr std::size_t DUBINS_SEGS = 3;

    /**
     * @brief Dubins shortest-path policy (CRTP) over 3 segments.
     *
     * @details
     * This policy computes all feasible geometric patterns of the form
     * `C S C` and `C C C` where `C ∈ {L, R}` and `S` is a straight segment,
     * then returns them as parameterized arcs. The @ref SteeringBase converts
     * these arcs into concrete controls and (optionally) integrates them to
     * a discretised state sequence.
     */
    class Dubins : public SteeringBase<Dubins, DUBINS_SEGS>
    {
      public:
        using Base = SteeringBase<Dubins, DUBINS_SEGS>;
        using Segment = typename Base::Segment;
        using enum Segment;

        /**
         * @brief Construct a Dubins policy.
         * @param rMin Minimal turning radius (meters).
         * @param ds   State integration step (meters).
         */
        constexpr explicit Dubins (double rMin, double ds = 0.05) noexcept : SteeringBase<Dubins, DUBINS_SEGS> (rMin, ds) {}

        /*────────────── Public aliases ─────────────────────────────────*/

        /// @brief Fixed-size segment pattern (3 entries).
        using ThreeSeg = std::array<Segment, DUBINS_SEGS>;
        /// @brief Arc descriptor produced by this policy.
        using DubinsPath = Arc<Segment, DUBINS_SEGS>;

        /**
         * @brief Enumerate geometric Dubins candidates between two poses.
         * @param s1 Start pose (x, y, heading).
         * @param s2 Goal pose (x, y, heading).
         * @return Vector of parameterized arc candidates (no constraints applied).
         */
        [[nodiscard]] std::vector<DubinsPath> getArcs (const State &s1, const State &s2) const noexcept
        {
            std::vector<Dubins::DubinsPath> paths;

            const double dx = s2.x - s1.x;
            const double dy = s2.y - s1.y;
            const double th = std::atan2 (dy, dx);

            const double d = std::hypot (dx, dy) * this->getKappa (); // scaled
            const double a = normalizeAngle2Pi (s1.heading - th);
            const double b = normalizeAngle2Pi (s2.heading - th);

            /* easy zero-distance special-case */
            if (d < arcgen::core::dubins_tol && std::fabs (a - b) < arcgen::core::dubins_tol)
            {
                paths.push_back (makePath (PATH_TYPES[0], 0, 0, 0));
                return paths;
            }

            const double ca = std::cos (a), sa = std::sin (a);
            const double cb = std::cos (b), sb = std::sin (b);

            auto lsl = LSL (d, a, b, ca, sa, cb, sb);
            if (lsl)
                paths.push_back (*lsl);

            auto rsr = RSR (d, a, b, ca, sa, cb, sb);
            if (rsr)
                paths.push_back (*rsr);

            auto rsl = RSL (d, a, b, ca, sa, cb, sb);
            if (rsl)
                paths.push_back (*rsl);

            auto lsr = LSR (d, a, b, ca, sa, cb, sb);
            if (lsr)
                paths.push_back (*lsr);

            auto rlr = RLR (d, a, b, ca, sa, cb, sb);
            if (rlr)
                paths.push_back (*rlr);

            auto lrl = LRL (d, a, b, ca, sa, cb, sb);
            if (lrl)
                paths.push_back (*lrl);

            std::sort (paths.begin (), paths.end (), [this] (const auto &pathA, const auto &pathB) { return this->shorter (pathA, pathB); });
            return paths;
        }

      private:
        /*────────────── Static data — segment patterns ─────────────────*/

        /// @brief All Dubins segment patterns considered by this policy.
        static constexpr auto PATH_TYPES = std::to_array<ThreeSeg> (
            {{LEFT, STRAIGHT, LEFT}, {RIGHT, STRAIGHT, RIGHT}, {RIGHT, STRAIGHT, LEFT}, {LEFT, STRAIGHT, RIGHT}, {RIGHT, LEFT, RIGHT}, {LEFT, RIGHT, LEFT}});

        /**
         * @brief Small helper to create a parameterized path.
         * @param pattern One of the entries in @ref PATH_TYPES.
         * @param s0 First segment parameter (dimensionless; multiplied by rMin).
         * @param s1 Second segment parameter.
         * @param s2 Third segment parameter.
         * @return Arc descriptor bound to @p pattern with the given parameters.
         */
        [[nodiscard]] constexpr DubinsPath makePath (const ThreeSeg &pattern, double s0, double s1, double s2) const noexcept { return {&pattern, {s0, s1, s2}}; }

        /*──────────── Low-level primitive families ─────────────────────*
         * Each routine returns the shortest candidate for its family or
         * std::nullopt if the family is infeasible for the given geometry.
         * Segment parameters are expressed in units of 1/r (i.e., scaled by
         * the curvature magnitude); the base multiplies them by rMin later.
         *───────────────────────────────────────────────────────────────*/

        [[nodiscard]] constexpr std::optional<DubinsPath> LSL (const double &d, const double &a, const double &b, const double &ca, const double &sa, const double &cb,
                                                               const double &sb) const noexcept
        {
            const double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sa - sb));
            if (tmp >= core::dubins_zero_tol)
            {
                const double th = std::atan2 (cb - ca, d + sa - sb);
                const double t = normalizeAngle2Pi (-a + th);
                const double p = std::sqrt (std::max (tmp, 0.0));
                const double q = normalizeAngle2Pi (b - th);
                return makePath (PATH_TYPES[0], t, p, q);
            }
            return std::nullopt;
        }

        [[nodiscard]] constexpr std::optional<DubinsPath> RSR (const double &d, const double &a, const double &b, const double &ca, const double &sa, const double &cb,
                                                               const double &sb) const noexcept
        {
            const double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sb - sa));
            if (tmp >= arcgen::core::dubins_zero_tol)
            {
                const double th = std::atan2 (ca - cb, d - sa + sb);
                const double t = normalizeAngle2Pi (a - th);
                const double p = std::sqrt (std::max (tmp, 0.0));
                const double q = normalizeAngle2Pi (-b + th);
                return makePath (PATH_TYPES[1], t, p, q);
            }
            return std::nullopt;
        }

        [[nodiscard]] constexpr std::optional<DubinsPath> RSL (const double &d, const double &a, const double &b, const double &ca, const double &sa, const double &cb,
                                                               const double &sb) const noexcept
        {
            const double tmp = d * d - 2. + 2. * (ca * cb + sa * sb - d * (sa + sb));
            if (tmp >= arcgen::core::dubins_zero_tol)
            {
                const double p = std::sqrt (std::max (tmp, 0.0));
                const double th = std::atan2 (ca + cb, d - sa - sb) - std::atan2 (2.0, p);
                const double t = normalizeAngle2Pi (a - th);
                const double q = normalizeAngle2Pi (b - th);
                return makePath (PATH_TYPES[2], t, p, q);
            }
            return std::nullopt;
        }

        [[nodiscard]] constexpr std::optional<DubinsPath> LSR (const double &d, const double &a, const double &b, const double &ca, const double &sa, const double &cb,
                                                               const double &sb) const noexcept
        {
            const double tmp = -2. + d * d + 2. * (ca * cb + sa * sb + d * (sa + sb));
            if (tmp >= arcgen::core::dubins_zero_tol)
            {
                const double p = std::sqrt (std::max (tmp, 0.0));
                const double th = std::atan2 (-ca - cb, d + sa + sb) - std::atan2 (-2.0, p);
                const double t = normalizeAngle2Pi (-a + th);
                const double q = normalizeAngle2Pi (-b + th);
                return makePath (PATH_TYPES[3], t, p, q);
            }
            return std::nullopt;
        }

        [[nodiscard]] constexpr std::optional<DubinsPath> RLR (const double &d, const double &a, const double &b, const double &ca, const double &sa, const double &cb,
                                                               const double &sb) const noexcept
        {
            const double tmp = 0.125 * (6. - d * d + 2. * (ca * cb + sa * sb + d * (sa - sb)));
            if (std::fabs (tmp) < 1.0)
            {
                const double p = 2. * std::numbers::pi - std::acos (tmp);
                const double th = std::atan2 (ca - cb, d - sa + sb);
                const double t = normalizeAngle2Pi (a - th + 0.5 * p);
                const double q = normalizeAngle2Pi (a - b - t + p);
                return makePath (PATH_TYPES[4], t, p, q);
            }
            return std::nullopt;
        }

        [[nodiscard]] constexpr std::optional<DubinsPath> LRL (const double &d, const double &a, const double &b, const double &ca, const double &sa, const double &cb,
                                                               const double &sb) const noexcept
        {
            const double tmp = 0.125 * (6. - d * d + 2. * (ca * cb + sa * sb - d * (sa - sb)));
            if (std::fabs (tmp) < 1.0)
            {
                const double p = 2. * std::numbers::pi - std::acos (tmp);
                const double th = std::atan2 (-ca + cb, d + sa - sb);
                const double t = normalizeAngle2Pi (-a + th + 0.5 * p);
                const double q = normalizeAngle2Pi (b - a - t + p);
                return makePath (PATH_TYPES[5], t, p, q);
            }
            return std::nullopt;
        }

        /**
         * @brief Compare two candidates by total absolute length.
         * @param a First candidate.
         * @param b Second candidate.
         * @return True if @p a is strictly shorter than @p b.
         */
        [[nodiscard]] constexpr bool shorter (const DubinsPath &a, const DubinsPath &b) const noexcept { return a.total () < b.total (); }
    };

    // Validate the CRTP contract for Dubins against SteeringBase’s expectations.
    static_assert (SteeringPolicy<Dubins, DUBINS_SEGS>, "Dubins doesn’t satisfy SteeringPolicy");

} // namespace arcgen::steering
