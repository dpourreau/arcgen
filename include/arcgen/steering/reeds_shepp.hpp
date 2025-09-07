#pragma once
/**
 * @file   reeds_shepp.hpp
 * @brief  Reeds–Shepp 48-pattern shortest-path generator (CRTP policy).
 *
 * Produces shortest paths between two SE(2) poses for a car-like vehicle that
 * can drive both forward and reverse under bounded curvature. This class
 * enumerates geometric candidates as parameterized arcs; the CRTP base
 * converts them into concrete controls and can integrate them to states.
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
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
    /// @brief Number of segments in a Reeds–Shepp shortest path.
    inline constexpr std::size_t REEDS_SEGS = 5;

    /**
     * @brief Reeds–Shepp shortest-path policy (CRTP) over 5 segments.
     *
     * @details
     * This policy enumerates the classic Reeds–Shepp families (CSC, CCC, CCCC,
     * CCSC, CCSCC) using the 48 canonical patterns. It returns parameterized
     * arcs; @ref SteeringBase converts them to concrete controls and can
     * integrate them to a discretised state sequence on demand.
     */
    class ReedsShepp : public SteeringBase<ReedsShepp, REEDS_SEGS>
    {
      public:
        using Base = SteeringBase<ReedsShepp, REEDS_SEGS>;
        using Segment = typename Base::Segment;
        using enum Segment;

        /**
         * @brief Construct a Reeds–Shepp policy.
         * @param rMin Minimal turning radius (meters).
         * @param ds   State integration step (meters).
         */
        constexpr explicit ReedsShepp (double rMin, double ds = 0.05) noexcept : SteeringBase<ReedsShepp, REEDS_SEGS> (rMin, ds) {}

        /*────────────── Public aliases ─────────────────────────────────*/

        /// @brief Fixed-size segment pattern (5 entries).
        using FiveSeg = std::array<Segment, REEDS_SEGS>;
        /// @brief Arc descriptor produced by this policy.
        using ReedsSheppPath = Arc<Segment, REEDS_SEGS>;

        /**
         * @brief Enumerate geometric Reeds–Shepp candidates between two poses.
         * @param s1 Start pose (x, y, heading).
         * @param s2 Goal pose (x, y, heading).
         * @return Vector of parameterized arc candidates (no constraints applied).
         */
        [[nodiscard]] std::vector<ReedsSheppPath> getArcs (const State &s1, const State &s2) const noexcept
        {
            std::vector<ReedsShepp::ReedsSheppPath> paths;
            /* transform into the local frame of from and scale by κ */
            const double dx = s2.x - s1.x;
            const double dy = s2.y - s1.y;
            const double dth = s2.heading - s1.heading;

            const double c = std::cos (s1.heading);
            const double s = std::sin (s1.heading);

            const double x = c * dx + s * dy;
            const double y = -s * dx + c * dy;

            familyCsc (x * this->getKappa (), y * this->getKappa (), dth, paths);
            familyCcc (x * this->getKappa (), y * this->getKappa (), dth, paths);
            familyCccc (x * this->getKappa (), y * this->getKappa (), dth, paths);
            familyCcsc (x * this->getKappa (), y * this->getKappa (), dth, paths);
            familyCcscc (x * this->getKappa (), y * this->getKappa (), dth, paths);

            std::sort (paths.begin (), paths.end (), [this] (const auto &a, const auto &b) { return this->shorter (a, b); });
            return paths;
        }

      private:
        /*────────────── Static data — segment patterns ─────────────────*/

        /// @brief Canonical Reeds–Shepp segment patterns considered by this policy.
        static constexpr auto PATH_TYPES = std::to_array<FiveSeg> ({/*  0 */ {LEFT, RIGHT, LEFT, NOP, NOP},
                                                                    /*  1 */ {RIGHT, LEFT, RIGHT, NOP, NOP},
                                                                    /*  2 */ {LEFT, RIGHT, LEFT, RIGHT, NOP},
                                                                    /*  3 */ {RIGHT, LEFT, RIGHT, LEFT, NOP},
                                                                    /*  4 */ {LEFT, RIGHT, STRAIGHT, LEFT, NOP},
                                                                    /*  5 */ {RIGHT, LEFT, STRAIGHT, RIGHT, NOP},
                                                                    /*  6 */ {LEFT, STRAIGHT, RIGHT, LEFT, NOP},
                                                                    /*  7 */ {RIGHT, STRAIGHT, LEFT, RIGHT, NOP},
                                                                    /*  8 */ {LEFT, RIGHT, STRAIGHT, RIGHT, NOP},
                                                                    /*  9 */ {RIGHT, LEFT, STRAIGHT, LEFT, NOP},
                                                                    /* 10 */ {RIGHT, STRAIGHT, RIGHT, LEFT, NOP},
                                                                    /* 11 */ {LEFT, STRAIGHT, LEFT, RIGHT, NOP},
                                                                    /* 12 */ {LEFT, STRAIGHT, RIGHT, NOP, NOP},
                                                                    /* 13 */ {RIGHT, STRAIGHT, LEFT, NOP, NOP},
                                                                    /* 14 */ {LEFT, STRAIGHT, LEFT, NOP, NOP},
                                                                    /* 15 */ {RIGHT, STRAIGHT, RIGHT, NOP, NOP},
                                                                    /* 16 */ {LEFT, RIGHT, STRAIGHT, LEFT, RIGHT},
                                                                    /* 17 */ {RIGHT, LEFT, STRAIGHT, RIGHT, LEFT}});

        /**
         * @brief Small helper to create a parameterized path.
         * @param pattern One of the entries in @ref PATH_TYPES.
         * @param s0 First segment parameter (dimensionless; multiplied by rMin).
         * @param s1 Second segment parameter.
         * @param s2 Third segment parameter.
         * @return Arc descriptor bound to @p pattern with the given parameters.
         */
        [[nodiscard]] constexpr ReedsSheppPath makePath (const FiveSeg &pattern, double s0, double s1, double s2) const noexcept { return {&pattern, {s0, s1, s2}}; }

        /*──────────── Low-level primitive families ─────────────────────*
         * Each routine encodes the closed-form solution for a small set
         * of patterns. If feasible, it returns the shortest candidate in
         * its family; otherwise returns std::nullopt.
         *───────────────────────────────────────────────────────────────*/

        /// @brief CSC family primitive: L+ S L+.
        [[nodiscard]] constexpr bool LpSpLp (const double &x, const double &y, const double &phi, double &t, double &u, double &v) const noexcept
        {
            std::tie (u, t) = toPolar (x - std::sin (phi), y - 1.0 + std::cos (phi));
            if (t >= -arcgen::core::reeds_shepp_zero_tol)
            {
                v = normalizeAngleSigned (phi - t);
                return v >= -arcgen::core::reeds_shepp_zero_tol;
            }
            return false;
        }

        /// @brief CSC family primitive: L+ S R+.
        [[nodiscard]] constexpr bool LpSpRp (const double &x, const double &y, const double &phi, double &t, double &u, double &v) const noexcept
        {
            auto [rho, theta] = toPolar (x + std::sin (phi), y - 1.0 - std::cos (phi));
            rho *= rho; // ρ²
            if (rho >= 4.0)
            {
                u = std::sqrt (rho - 4.0);
                const double th = std::atan2 (2.0, u);
                t = normalizeAngleSigned (theta + th);
                v = normalizeAngleSigned (t - phi);
                return t >= -arcgen::core::reeds_shepp_zero_tol && v >= -arcgen::core::reeds_shepp_zero_tol;
            }
            return false;
        }

        /// @brief CCC family primitive: L+ R− L.
        [[nodiscard]] constexpr bool LpRmL (const double &x, const double &y, const double &phi, double &t, double &u, double &v) const noexcept
        {
            const double xi = x - std::sin (phi);
            const double eta = y - 1.0 + std::cos (phi);
            auto [rho, theta] = toPolar (xi, eta);
            if (rho <= 4.0)
            {
                u = -2.0 * std::asin (0.25 * rho);
                t = normalizeAngleSigned (theta + 0.5 * u + std::numbers::pi);
                v = normalizeAngleSigned (phi - t + u);
                return t >= -arcgen::core::reeds_shepp_zero_tol && u <= arcgen::core::reeds_shepp_zero_tol;
            }
            return false;
        }

        /// @brief CCCC family primitive variant.
        [[nodiscard]] constexpr bool LpRupLumRm (const double &x, const double &y, const double &phi, double &t, double &u, double &v) const noexcept
        {
            const double xi = x + std::sin (phi);
            const double eta = y - 1.0 - std::cos (phi);
            const double rho = 0.25 * (2.0 + std::hypot (xi, eta));
            if (rho <= 1.0)
            {
                u = std::acos (rho);
                std::tie (t, v) = tauOmega (u, -u, xi, eta, phi);
                return t >= -arcgen::core::reeds_shepp_zero_tol && v <= arcgen::core::reeds_shepp_zero_tol;
            }
            return false;
        }

        /// @brief CCCC family primitive variant.
        [[nodiscard]] constexpr bool LpRumLumRp (const double &x, const double &y, const double &phi, double &t, double &u, double &v) const noexcept
        {
            const double xi = x + std::sin (phi);
            const double eta = y - 1.0 - std::cos (phi);
            const double rho = (20.0 - xi * xi - eta * eta) / 16.0;
            if (rho >= 0.0 && rho <= 1.0)
            {
                u = -std::acos (rho);
                if (u >= -0.5 * std::numbers::pi)
                {
                    std::tie (t, v) = tauOmega (u, u, xi, eta, phi);
                    return t >= -arcgen::core::reeds_shepp_zero_tol && v >= -arcgen::core::reeds_shepp_zero_tol;
                }
            }
            return false;
        }

        /// @brief CCSC family primitive variant.
        [[nodiscard]] constexpr bool LpRmSmLm (const double &x, const double &y, const double &phi, double &t, double &u, double &v) const noexcept
        {
            const double xi = x - std::sin (phi);
            const double eta = y - 1.0 + std::cos (phi);
            auto [rho, theta] = toPolar (xi, eta);

            if (rho >= 2.0)
            {
                const double r = std::sqrt (rho * rho - 4.0);
                u = 2.0 - r;
                t = normalizeAngleSigned (theta + std::atan2 (r, -2.0));
                v = normalizeAngleSigned (phi - 0.5 * std::numbers::pi - t);
                return t >= -arcgen::core::reeds_shepp_zero_tol && u <= arcgen::core::reeds_shepp_zero_tol && v <= arcgen::core::reeds_shepp_zero_tol;
            }
            return false;
        }

        /// @brief CCSC family primitive variant.
        [[nodiscard]] constexpr bool LpRmSmRm (const double &x, const double &y, const double &phi, double &t, double &u, double &v) const noexcept
        {
            const double xi = x + std::sin (phi);
            const double eta = y - 1.0 - std::cos (phi);
            auto [rho, theta] = toPolar (-eta, xi);

            if (rho >= 2.0)
            {
                t = theta;
                u = 2.0 - rho;
                v = normalizeAngleSigned (t + 0.5 * std::numbers::pi - phi);
                return t >= -arcgen::core::reeds_shepp_zero_tol && u <= arcgen::core::reeds_shepp_zero_tol && v <= arcgen::core::reeds_shepp_zero_tol;
            }
            return false;
        }

        /// @brief CCSCC family primitive variant.
        [[nodiscard]] constexpr bool LpRmSLmRp (const double &x, const double &y, const double &phi, double &t, double &u, double &v) const noexcept
        {
            const double xi = x + std::sin (phi);
            const double eta = y - 1.0 - std::cos (phi);
            auto [rho, theta] = toPolar (xi, eta);

            if (rho >= 2.0)
            {
                u = 4.0 - std::sqrt (rho * rho - 4.0);
                if (u <= arcgen::core::reeds_shepp_zero_tol)
                {
                    t = normalizeAngleSigned (std::atan2 ((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
                    v = normalizeAngleSigned (t - phi);
                    return t >= -arcgen::core::reeds_shepp_zero_tol && v >= -arcgen::core::reeds_shepp_zero_tol;
                }
            }
            return false;
        }

        /**
         * @brief Auxiliary angles for several families.
         * @param x   Normalized goal x (scaled by 1/r).
         * @param y   Normalized goal y (scaled by 1/r).
         * @param phi Goal heading difference (radians).
         * @param cs  cos(phi).
         * @param sn  sin(phi).
         * @return Pair {tau, omega}.
         */
        [[nodiscard]] constexpr std::pair<double, double> tauOmega (const double &u, const double &v, const double &xi, const double &eta, const double &phi) const noexcept
        {
            const double delta = normalizeAngleSigned (u - v);
            const double A = std::sin (u) - std::sin (delta);
            const double B = std::cos (u) - std::cos (delta) - 1.0;

            const double t1 = std::atan2 (eta * A - xi * B, xi * A + eta * B);
            const double t2 = 2.0 * (std::cos (delta) - std::cos (v) - std::cos (u)) + 3.0;

            const double tau = (t2 < 0.0) ? normalizeAngleSigned (t1 + std::numbers::pi) : normalizeAngleSigned (t1);
            const double omega = normalizeAngleSigned (tau - u + v - phi);
            return {tau, omega};
        }

        /*──────────── Families (build candidate sets) ──────────────────*/

        /// @brief Build CSC-family candidates and append to @p paths.
        constexpr void familyCsc (const double &x, const double &y, const double &phi, std::vector<ReedsShepp::ReedsSheppPath> &paths) const noexcept
        {
            double t, u, v;

            /* L+S-L+,  mirror & sign variants */
            if (LpSpLp (x, y, phi, t, u, v))
                paths.push_back ({&PATH_TYPES[14], {t, u, v, 0, 0}});
            if (LpSpLp (-x, y, -phi, t, u, v))
                paths.push_back ({&PATH_TYPES[14], {-t, -u, -v, 0, 0}});
            if (LpSpLp (x, -y, -phi, t, u, v))
                paths.push_back ({&PATH_TYPES[15], {t, u, v, 0, 0}});
            if (LpSpLp (-x, -y, phi, t, u, v))
                paths.push_back ({&PATH_TYPES[15], {-t, -u, -v, 0, 0}});

            /* L+S-R+ */
            if (LpSpRp (x, y, phi, t, u, v))
                paths.push_back ({&PATH_TYPES[12], {t, u, v, 0, 0}});
            if (LpSpRp (-x, y, -phi, t, u, v))
                paths.push_back ({&PATH_TYPES[12], {-t, -u, -v, 0, 0}});
            if (LpSpRp (x, -y, -phi, t, u, v))
                paths.push_back ({&PATH_TYPES[13], {t, u, v, 0, 0}});
            if (LpSpRp (-x, -y, phi, t, u, v))
                paths.push_back ({&PATH_TYPES[13], {-t, -u, -v, 0, 0}});
        }

        /// @brief Build CCC-family candidates and append to @p paths.
        constexpr void familyCcc (const double &x, const double &y, const double &phi, std::vector<ReedsShepp::ReedsSheppPath> &paths) const noexcept
        {
            double t, u, v;

            if (LpRmL (x, y, phi, t, u, v))
                paths.push_back ({&PATH_TYPES[0], {t, u, v, 0, 0}});
            if (LpRmL (-x, y, -phi, t, u, v))
                paths.push_back ({&PATH_TYPES[0], {-t, -u, -v, 0, 0}});
            if (LpRmL (x, -y, -phi, t, u, v))
                paths.push_back ({&PATH_TYPES[1], {t, u, v, 0, 0}});
            if (LpRmL (-x, -y, phi, t, u, v))
                paths.push_back ({&PATH_TYPES[1], {-t, -u, -v, 0, 0}});

            /* mirrored “backwards” versions */
            const double cs = std::cos (phi), sn = std::sin (phi);
            const double xb = x * cs + y * sn;
            const double yb = x * sn - y * cs;

            if (LpRmL (xb, yb, phi, t, u, v))
                paths.push_back ({&PATH_TYPES[0], {v, u, t, 0, 0}});
            if (LpRmL (-xb, yb, -phi, t, u, v))
                paths.push_back ({&PATH_TYPES[0], {-v, -u, -t, 0, 0}});
            if (LpRmL (xb, -yb, -phi, t, u, v))
                paths.push_back ({&PATH_TYPES[1], {v, u, t, 0, 0}});
            if (LpRmL (-xb, -yb, phi, t, u, v))
                paths.push_back ({&PATH_TYPES[1], {-v, -u, -t, 0, 0}});
        }

        /// @brief Build CCCC-family candidates and append to @p paths.
        constexpr void familyCccc (const double &x, const double &y, const double &phi, std::vector<ReedsShepp::ReedsSheppPath> &paths) const noexcept
        {
            double t, u, v;

            /* L+R⁺L⁻R⁻ */
            if (LpRupLumRm (x, y, phi, t, u, v))
                paths.push_back ({&PATH_TYPES[2], {t, u, -u, v, 0}});
            if (LpRupLumRm (-x, y, -phi, t, u, v))
                paths.push_back ({&PATH_TYPES[2], {-t, -u, u, -v, 0}});
            if (LpRupLumRm (x, -y, -phi, t, u, v))
                paths.push_back ({&PATH_TYPES[3], {t, u, -u, v, 0}});
            if (LpRupLumRm (-x, -y, phi, t, u, v))
                paths.push_back ({&PATH_TYPES[3], {-t, -u, u, -v, 0}});

            /* L+R⁻L⁺R⁺ */
            if (LpRumLumRp (x, y, phi, t, u, v))
                paths.push_back ({&PATH_TYPES[2], {t, u, u, v, 0}});
            if (LpRumLumRp (-x, y, -phi, t, u, v))
                paths.push_back ({&PATH_TYPES[2], {-t, -u, -u, -v, 0}});
            if (LpRumLumRp (x, -y, -phi, t, u, v))
                paths.push_back ({&PATH_TYPES[3], {t, u, u, v, 0}});
            if (LpRumLumRp (-x, -y, phi, t, u, v))
                paths.push_back ({&PATH_TYPES[3], {-t, -u, -u, -v, 0}});
        }

        /// @brief Build CCSC-family candidates and append to @p paths.
        constexpr void familyCcsc (const double &x, const double &y, const double &phi, std::vector<ReedsShepp::ReedsSheppPath> &paths) const noexcept
        {
            double t, u, v;

            constexpr double H = -0.5 * std::numbers::pi; // half-turn in radians

            /* Table II (original paper) */
            if (LpRmSmLm (x, y, phi, t, u, v))
                paths.push_back ({&PATH_TYPES[4], {t, H, u, v, 0}});
            if (LpRmSmLm (-x, y, -phi, t, u, v))
                paths.push_back ({&PATH_TYPES[4], {-t, -H, -u, -v, 0}});
            if (LpRmSmLm (x, -y, -phi, t, u, v))
                paths.push_back ({&PATH_TYPES[5], {t, H, u, v, 0}});
            if (LpRmSmLm (-x, -y, phi, t, u, v))
                paths.push_back ({&PATH_TYPES[5], {-t, -H, -u, -v, 0}});

            if (LpRmSmRm (x, y, phi, t, u, v))
                paths.push_back ({&PATH_TYPES[8], {t, H, u, v, 0}});
            if (LpRmSmRm (-x, y, -phi, t, u, v))
                paths.push_back ({&PATH_TYPES[8], {-t, -H, -u, -v, 0}});
            if (LpRmSmRm (x, -y, -phi, t, u, v))
                paths.push_back ({&PATH_TYPES[9], {t, H, u, v, 0}});
            if (LpRmSmRm (-x, -y, phi, t, u, v))
                paths.push_back ({&PATH_TYPES[9], {-t, -H, -u, -v, 0}});

            const double cs = std::cos (phi), sn = std::sin (phi);
            const double xb = x * cs + y * sn;
            const double yb = x * sn - y * cs;

            if (LpRmSmLm (xb, yb, phi, t, u, v))
                paths.push_back ({&PATH_TYPES[6], {v, u, H, t, 0}});
            if (LpRmSmLm (-xb, yb, -phi, t, u, v))
                paths.push_back ({&PATH_TYPES[6], {-v, -u, -H, -t, 0}});
            if (LpRmSmLm (xb, -yb, -phi, t, u, v))
                paths.push_back ({&PATH_TYPES[7], {v, u, H, t, 0}});
            if (LpRmSmLm (-xb, -yb, phi, t, u, v))
                paths.push_back ({&PATH_TYPES[7], {-v, -u, -H, -t, 0}});

            if (LpRmSmRm (xb, yb, phi, t, u, v))
                paths.push_back ({&PATH_TYPES[10], {v, u, H, t, 0}});
            if (LpRmSmRm (-xb, yb, -phi, t, u, v))
                paths.push_back ({&PATH_TYPES[10], {-v, -u, -H, -t, 0}});
            if (LpRmSmRm (xb, -yb, -phi, t, u, v))
                paths.push_back ({&PATH_TYPES[11], {v, u, H, t, 0}});
            if (LpRmSmRm (-xb, -yb, phi, t, u, v))
                paths.push_back ({&PATH_TYPES[11], {-v, -u, -H, -t, 0}});
        }

        /// @brief Build CCSCC-family candidates and append to @p paths.
        constexpr void familyCcscc (const double &x, const double &y, const double &phi, std::vector<ReedsShepp::ReedsSheppPath> &paths) const noexcept
        {
            double t, u, v;

            constexpr double H = -0.5 * std::numbers::pi;

            if (LpRmSLmRp (x, y, phi, t, u, v))
                paths.push_back ({&PATH_TYPES[16], {t, H, u, H, v}});
            if (LpRmSLmRp (-x, y, -phi, t, u, v))
                paths.push_back ({&PATH_TYPES[16], {-t, -H, -u, -H, -v}});
            if (LpRmSLmRp (x, -y, -phi, t, u, v))
                paths.push_back ({&PATH_TYPES[17], {t, H, u, H, v}});
            if (LpRmSLmRp (-x, -y, phi, t, u, v))
                paths.push_back ({&PATH_TYPES[17], {-t, -H, -u, -H, -v}});
        }

        /*──────────── Tiny utilities ───────────────────────────────────*/

        /**
         * @brief Compare two candidates by total absolute length.
         * @param a First candidate.
         * @param b Second candidate.
         * @return True if @p a is strictly shorter than @p b.
         */
        [[nodiscard]] constexpr bool shorter (const ReedsSheppPath &a, const ReedsSheppPath &b) const noexcept { return a.total () < b.total (); }
    };

    // Validate the CRTP contract for ReedsShepp against SteeringBase’s expectations.
    static_assert (SteeringPolicy<ReedsShepp, REEDS_SEGS>, "ReedsShepp doesn’t satisfy SteeringPolicy");

} // namespace arcgen::steering
