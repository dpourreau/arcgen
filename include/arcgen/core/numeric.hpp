#pragma once
/**
 * @file   numeric.hpp
 * @brief  Project-wide numerical constants and tolerances.
 *
 * Keep *all* floating-point comparisons and common constants in one place.
 * Prefer using the ALL_CAPS constants; the lower-case aliases are provided
 * for backward compatibility with existing code.
 */

#include <limits>
#include <numbers>

namespace arcgen::core
{
    /**
     * @brief Machine epsilon for a given floating-point type.
     * @tparam T Floating-point type.
     *
     * Example: `EPSILON_V<double>` equals `std::numeric_limits<double>::epsilon()`.
     */
    template <typename T> inline constexpr T EPSILON_V = std::numeric_limits<T>::epsilon ();

    /**
     * @brief π specialized for a given floating-point type.
     * @tparam T Floating-point type.
     *
     * Example: `PI_V<double>` equals `std::numbers::pi_v<double>`.
     */
    template <typename T> inline constexpr T PI_V = std::numbers::pi_v<T>;

    /*────────────────── Common double-precision constants ──────────────────*/

    /// Absolute tolerance used across the project (double precision).
    inline constexpr double EPSILON = EPSILON_V<double>;

    /// π in double precision.
    inline constexpr double PI = PI_V<double>;

    /// 2π in double precision.
    inline constexpr double PI2 = 2.0 * PI;

    /// π/2 in double precision.
    inline constexpr double PI_OVER_TWO = PI / 2.0;

    /*────────────────── Planner-specific tolerances (double) ───────────────*/

    /// Reeds–Shepp numeric tolerance.
    inline constexpr double REEDS_SHEPP_TOL = 1e-6;

    /// Reeds–Shepp “zero” tolerance for near-zero checks.
    inline constexpr double REEDS_SHEPP_ZERO_TOL = 10 * std::numeric_limits<double>::epsilon ();

    /// Dubins numeric tolerance.
    inline constexpr double DUBINS_TOL = 1e-6;

    /// Dubins “zero” tolerance (signed; negative enforces strictness).
    inline constexpr double DUBINS_ZERO_TOL = -1e-9;

    /// Curvature numeric tolerance.
    inline constexpr double CURVATURE_TOL = 1e-6;

    /*────────────────── Backward-compatibility aliases ─────────────────────
     * Prefer the ALL_CAPS names above. These aliases keep older code working.
     */
    template <typename T> inline constexpr T epsilon_v = EPSILON_V<T>;
    template <typename T> inline constexpr T pi_v = PI_V<T>;

    inline constexpr double epsilon = EPSILON;
    inline constexpr double pi = PI;
    inline constexpr double two_pi = PI2;
    inline constexpr double pi_over_two = PI_OVER_TWO;

    inline constexpr double reeds_shepp_tol = REEDS_SHEPP_TOL;
    inline constexpr double reeds_shepp_zero_tol = REEDS_SHEPP_ZERO_TOL;
    inline constexpr double dubins_tol = DUBINS_TOL;
    inline constexpr double dubins_zero_tol = DUBINS_ZERO_TOL;

} // namespace arcgen::core
