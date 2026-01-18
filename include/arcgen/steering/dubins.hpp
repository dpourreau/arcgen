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
            const double a = arcgen::core::normalizeAngle2Pi (s1.heading - th);
            const double b = arcgen::core::normalizeAngle2Pi (s2.heading - th);

            if (d < arcgen::core::dubins_tol && std::fabs (a - b) < arcgen::core::dubins_tol)
            {
                paths.push_back (makePath (PATH_TYPES[0], 0, 0, 0));
                return paths;
            }

            const double ca = std::cos (a);
            const double sa = std::sin (a);
            const double cb = std::cos (b);
            const double sb = std::sin (b);

            if (auto p = lsl (d, a, b, ca, sa, cb, sb); p)
                paths.push_back (*p);

            if (auto p = rsr (d, a, b, ca, sa, cb, sb); p)
                paths.push_back (*p);

            if (auto p = rsl (d, a, b, ca, sa, cb, sb); p)
                paths.push_back (*p);

            if (auto p = lsr (d, a, b, ca, sa, cb, sb); p)
                paths.push_back (*p);

            if (auto p = rlr (d, a, b, ca, sa, cb, sb); p)
                paths.push_back (*p);

            if (auto p = lrl (d, a, b, ca, sa, cb, sb); p)
                paths.push_back (*p);

            std::ranges::sort (paths, [this] (const auto &pathA, const auto &pathB) { return this->shorter (pathA, pathB); });
            return paths;
        }

      private:
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

        [[nodiscard]] constexpr std::optional<DubinsPath> lsl (const double &d, const double &a, const double &b, const double &ca, const double &sa, const double &cb,
                                                               const double &sb) const noexcept
        {
            if (const double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sa - sb)); tmp >= arcgen::core::dubins_zero_tol)
            {
                const double th = std::atan2 (cb - ca, d + sa - sb);
                const double t = arcgen::core::normalizeAngle2Pi (-a + th);
                const double p = std::sqrt (std::max (tmp, 0.0));
                const double q = arcgen::core::normalizeAngle2Pi (b - th);
                return makePath (PATH_TYPES[0], t, p, q);
            }
            return std::nullopt;
        }

        [[nodiscard]] constexpr std::optional<DubinsPath> rsr (const double &d, const double &a, const double &b, const double &ca, const double &sa, const double &cb,
                                                               const double &sb) const noexcept
        {
            if (const double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sb - sa)); tmp >= arcgen::core::dubins_zero_tol)
            {
                const double th = std::atan2 (ca - cb, d - sa + sb);
                const double t = arcgen::core::normalizeAngle2Pi (a - th);
                const double p = std::sqrt (std::max (tmp, 0.0));
                const double q = arcgen::core::normalizeAngle2Pi (-b + th);
                return makePath (PATH_TYPES[1], t, p, q);
            }
            return std::nullopt;
        }

        [[nodiscard]] constexpr std::optional<DubinsPath> rsl (const double &d, const double &a, const double &b, const double &ca, const double &sa, const double &cb,
                                                               const double &sb) const noexcept
        {
            if (const double tmp = d * d - 2. + 2. * (ca * cb + sa * sb - d * (sa + sb)); tmp >= arcgen::core::dubins_zero_tol)
            {
                const double p = std::sqrt (std::max (tmp, 0.0));
                const double th = std::atan2 (ca + cb, d - sa - sb) - std::atan2 (2.0, p);
                const double t = arcgen::core::normalizeAngle2Pi (a - th);
                const double q = arcgen::core::normalizeAngle2Pi (b - th);
                return makePath (PATH_TYPES[2], t, p, q);
            }
            return std::nullopt;
        }

        [[nodiscard]] constexpr std::optional<DubinsPath> lsr (const double &d, const double &a, const double &b, const double &ca, const double &sa, const double &cb,
                                                               const double &sb) const noexcept
        {
            if (const double tmp = -2. + d * d + 2. * (ca * cb + sa * sb + d * (sa + sb)); tmp >= arcgen::core::dubins_zero_tol)
            {
                const double p = std::sqrt (std::max (tmp, 0.0));
                const double th = std::atan2 (-ca - cb, d + sa + sb) - std::atan2 (-2.0, p);
                const double t = arcgen::core::normalizeAngle2Pi (-a + th);
                const double q = arcgen::core::normalizeAngle2Pi (-b + th);
                return makePath (PATH_TYPES[3], t, p, q);
            }
            return std::nullopt;
        }

        [[nodiscard]] constexpr std::optional<DubinsPath> rlr (const double &d, const double &a, const double &b, const double &ca, const double &sa, const double &cb,
                                                               const double &sb) const noexcept
        {
            if (const double tmp = 0.125 * (6. - d * d + 2. * (ca * cb + sa * sb + d * (sa - sb))); std::fabs (tmp) < 1.0)
            {
                const double p = 2. * std::numbers::pi - std::acos (tmp);
                const double th = std::atan2 (ca - cb, d - sa + sb);
                const double t = arcgen::core::normalizeAngle2Pi (a - th + 0.5 * p);
                const double q = arcgen::core::normalizeAngle2Pi (a - b - t + p);
                return makePath (PATH_TYPES[4], t, p, q);
            }
            return std::nullopt;
        }

        [[nodiscard]] constexpr std::optional<DubinsPath> lrl (const double &d, const double &a, const double &b, const double &ca, const double &sa, const double &cb,
                                                               const double &sb) const noexcept
        {
            if (const double tmp = 0.125 * (6. - d * d + 2. * (ca * cb + sa * sb - d * (sa - sb))); std::fabs (tmp) < 1.0)
            {
                const double p = 2. * std::numbers::pi - std::acos (tmp);
                const double th = std::atan2 (-ca + cb, d + sa - sb);
                const double t = arcgen::core::normalizeAngle2Pi (-a + th + 0.5 * p);
                const double q = arcgen::core::normalizeAngle2Pi (b - a - t + p);
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
