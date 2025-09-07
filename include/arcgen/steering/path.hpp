#pragma once
/**
 * @file   path.hpp
 * @brief  Containers for steering-law candidates and their per-segment bounds.
 */

#include <arcgen/core/control.hpp>
#include <arcgen/core/state.hpp>

#include <array>
#include <optional>
#include <vector>

namespace arcgen::steering
{
    using arcgen::core::ControlSeq;
    using arcgen::core::State;

    /**
     * @brief Full candidate produced by a steering-law enumeration.
     *
     * Holds the concrete constant-curvature controls, a per-segment AABB
     * (aligned to world axes), and—optionally—the lazily discretized states.
     *
     * @tparam N Maximum number of segments in the steering pattern.
     */
    template <std::size_t N> struct Path
    {
        ControlSeq<N> controls;                           ///< Concrete controls (curvature + length)
        mutable std::optional<std::vector<State>> states; ///< Discretized states along the path (lazy)

        /**
         * @brief Total absolute path length (mirrors @c controls.length ).
         * @return Sum of absolute arc lengths of all controls.
         */
        [[nodiscard]] double length () const noexcept { return controls.length; }
    };

} // namespace arcgen::steering
