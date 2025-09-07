#pragma once
/**
 * @file   state.hpp
 * @brief  Pose, curvature, and driving-direction primitives for path generators.
 */

#include <arcgen/core/math.hpp>
#include <arcgen/core/numeric.hpp>
#include <type_traits>

namespace arcgen::core
{
    /**
     * @enum DrivingDirection
     * @brief Signed direction of motion for a car-like vehicle.
     */
    enum class DrivingDirection : int
    {
        Reverse = -1, ///< Driving backward.
        Neutral = 0,  ///< Standing still.
        Forward = 1   ///< Driving forward.
    };

    /**
     * @struct State
     * @brief  Vehicle pose and kinematic state in SE(2).
     *
     * Holds Cartesian position, heading (radians), path curvature, and
     * the inferred driving direction.
     */
    struct State
    {
        double x{};                                            ///< X position [m]
        double y{};                                            ///< Y position [m]
        double heading{};                                      ///< Heading angle [rad]
        double curvature{};                                    ///< Path curvature [m⁻¹]
        DrivingDirection direction{DrivingDirection::Neutral}; ///< Motion direction

        /**
         * @brief Set #direction from the sign of a scalar (e.g., arc length).
         * @param value A signed quantity; >0 ⇒ Forward, <0 ⇒ Reverse, 0 ⇒ Neutral.
         */
        inline constexpr void setDirectionFrom (double value) noexcept
        {
            direction = (value > 0.0) ? DrivingDirection::Forward : (value < 0.0) ? DrivingDirection::Reverse : DrivingDirection::Neutral;
        }
    };

    static_assert (std::is_trivially_copyable_v<State>, "State must remain trivially copyable");

} // namespace arcgen::core
