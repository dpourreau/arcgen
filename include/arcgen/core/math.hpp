#pragma once
/**
 * @file   math.hpp
 * @brief  Tiny, constexpr math helpers.
 */

#include <arcgen/core/numeric.hpp>

#include <cmath>
#include <concepts>
#include <numbers>
#include <tuple>
#include <utility>

namespace arcgen::core
{
    /**
     * @brief Convert Cartesian coordinates to polar form ⟨r, θ⟩.
     * @tparam T Floating-point type.
     * @param x  X coordinate.
     * @param y  Y coordinate.
     * @return Pair { r, θ } with r = √(x² + y²) and θ = atan2(y, x).
     */
    template <std::floating_point T> [[nodiscard]] constexpr std::pair<T, T> toPolar (T x, T y) noexcept
    {
        const T r = std::hypot (x, y);
        const T theta = std::atan2 (y, x);
        return {r, theta};
    }

    /**
     * @brief Wrap an angle to the interval [0, 2π).
     * @tparam T     Floating-point type.
     * @param angle  Angle in radians.
     * @return Angle in [0, 2π).
     */
    template <std::floating_point T> [[nodiscard]] constexpr T normalizeAngle2Pi (T angle) noexcept
    {
        constexpr T pi2 = T{2} * std::numbers::pi_v<T>;
        if (angle >= T{0} && angle < pi2)
            return angle;
        if (angle >= -pi2 && angle < T{0})
            return angle + pi2;
        const T mod = std::fmod (angle, pi2);
        return (mod < T{0}) ? mod + pi2 : mod;
    }

    /**
     * @brief Wrap an angle to the interval (−π, π].
     * @tparam T     Floating-point type.
     * @param angle  Angle in radians.
     * @return Angle in (−π, π].
     */
    template <std::floating_point T> [[nodiscard]] constexpr T normalizeAngleSigned (T angle) noexcept
    {
        constexpr T pi2 = T{2} * std::numbers::pi_v<T>;
        return std::remainder (angle, pi2); // (−π, π]
    }

    /**
     * @brief Endpoint of a straight-line segment.
     * @tparam T         Floating-point type.
     * @param x0         Start X.
     * @param y0         Start Y.
     * @param theta      Heading (rad).
     * @param direction  +1 ⇒ forward, −1 ⇒ reverse, 0 ⇒ no motion.
     * @param length     Line length (m).
     * @return ⟨x₁, y₁⟩ of the endpoint.
     */
    template <std::floating_point T> [[nodiscard]] constexpr std::pair<T, T> computeLineEndpoint (T x0, T y0, T theta, int direction, T length) noexcept
    {
        const T dx = static_cast<T> (direction) * length * std::cos (theta);
        const T dy = static_cast<T> (direction) * length * std::sin (theta);
        return {x0 + dx, y0 + dy};
    }

    /**
     * @brief Endpoint pose of a circular arc.
     * @tparam T         Floating-point type.
     * @param x0         Start X.
     * @param y0         Start Y.
     * @param theta0     Start heading (rad).
     * @param curvature  κ = 1 / radius (rad·m⁻¹). If |κ| ≈ 0, a straight step is used.
     * @param direction  +1 ⇒ forward, −1 ⇒ reverse, 0 ⇒ no motion.
     * @param length     Arc length (m).
     * @return ⟨x₁, y₁, θ₁⟩ of the endpoint (θ wrapped to (−π, π]).
     */
    template <std::floating_point T> [[nodiscard]] constexpr std::tuple<T, T, T> computeArcEndpoint (T x0, T y0, T theta0, T curvature, int direction, T length) noexcept
    {
        // Robustness: treat tiny curvature as straight motion to avoid 1/0.
        if (std::fabs (curvature) <= static_cast<T> (core::CURVATURE_TOL))
        {
            const auto [x1, y1] = computeLineEndpoint (x0, y0, theta0, direction, length);
            return {x1, y1, normalizeAngleSigned (theta0)};
        }

        const T delta = static_cast<T> (direction) * curvature * length;
        const T theta1 = theta0 + delta;
        const T radius = T{1} / curvature;

        const T x1 = x0 + radius * (-std::sin (theta0) + std::sin (theta1));
        const T y1 = y0 + radius * (std::cos (theta0) - std::cos (theta1));

        return {x1, y1, normalizeAngleSigned (theta1)};
    }

} // namespace arcgen::core
