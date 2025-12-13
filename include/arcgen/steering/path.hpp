#pragma once
/**
 * @file   path.hpp
 * @brief  Containers for steering-law candidates and their per-segment bounds.
 */

#include <arcgen/core/control.hpp>
#include <arcgen/core/state.hpp>

#include <array>
#include <cmath>
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
         * @brief Total absolute path length.
         *
         * If controls are present, returns their accumulated length.
         * Otherwise, calculates geometric length from discretized states.
         *
         * @return Path length in meters.
         */
        [[nodiscard]] double length () const noexcept
        {
            if (controls.n > 0)
                return controls.length;

            if (states && !states->empty ())
            {
                double len = 0.0;
                const auto &sts = *states;
                for (std::size_t i = 0; i + 1 < sts.size (); ++i)
                {
                    const double dx = sts[i + 1].x - sts[i].x;
                    const double dy = sts[i + 1].y - sts[i].y;
                    len += std::hypot (dx, dy);
                }
                return len;
            }

            return 0.0;
        }
    };

} // namespace arcgen::steering
