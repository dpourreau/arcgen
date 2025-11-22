#pragma once
/**
 * @file   control.hpp
 * @brief  Motion primitives: constant-curvature control and small fixed-size sequences.
 */

#include <arcgen/core/numeric.hpp>
#include <arcgen/core/state.hpp>

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <type_traits>

namespace arcgen::core
{
    /**
     * @brief  A single constant-curvature motion primitive.
     *
     * A control represents one path segment parameterized by curvature (κ) and
     * signed arc length (s). The sign of @ref arcLength encodes driving direction.
     */
    struct Control
    {
        double curvature{}; ///< Signed curvature κ in m⁻¹. κ = 0 ⇒ straight segment.
        double arcLength{}; ///< Signed arc length s in meters. Sign ⇒ driving direction.

        /**
         * @brief  Driving direction derived from the sign of @ref arcLength.
         * @return DrivingDirection::Forward if s>0, Reverse if s<0, otherwise Neutral.
         */
        [[nodiscard]] constexpr DrivingDirection direction () const noexcept
        {
            return (arcLength > 0.0) ? DrivingDirection::Forward : (arcLength < 0.0) ? DrivingDirection::Reverse : DrivingDirection::Neutral;
        }
    };

    /**
     * @brief  Small fixed-size sequence of @ref Control.
     *
     * @tparam N  Maximum number of stored controls.
     *
     * Accumulates **absolute** total length upon insertion.
     */
    template <std::size_t N> struct ControlSeq
    {
        static_assert (N <= std::numeric_limits<std::uint8_t>::max (), "ControlSeq<N>: N must be ≤ 255 because 'n' is uint8_t.");

        std::array<Control, N> buf{}; ///< Inline storage for up to N controls.
        std::uint8_t n{0};            ///< Current number of valid elements (≤ N).
        double length{0.0};           ///< Absolute total length: Σ |control.arcLength|.

        /// @brief Remove all elements and reset the accumulated length.
        constexpr void clear () noexcept
        {
            n = 0;
            length = 0.0;
        }

        /**
         * @brief Append a control.
         * @param c Control to append.
         * @return True if appended, false if the sequence was full.
         *
         * On success increments @ref n and increases @ref length by |c.arcLength|.
         */
        constexpr bool push_back (Control c) noexcept
        {
            assert (n < N && "ControlSeq overflow");
            if (n >= N)
                return false;
            buf[n++] = c;
            length += std::fabs (c.arcLength);
            return true;
        }

        /**
         * @brief  Read-only view over the stored controls.
         * @return Span of size @ref n into @ref buf.
         */
        [[nodiscard]] constexpr std::span<const Control> view () const noexcept { return {buf.data (), static_cast<std::size_t> (n)}; }
    };

    static_assert (std::is_trivially_copyable_v<Control>, "Control must remain trivially copyable");

} // namespace arcgen::core
