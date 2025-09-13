#pragma once
/**
 * @file    robot.hpp
 * @brief   Polygonal robot model in local frame with SE(2) transforms
 *          and swept-footprint utilities for collision checks.
 *
 * Conventions:
 * - The robot polygon is defined in its local/body frame with the
 *   reference point (e.g., GPS/center) at the origin (0,0) and the
 *   robot initially facing +X.
 * - Polygons are CCW and closed, matching arcgen's `Polygon` type.
 */

#include <arcgen/core/math.hpp>
#include <arcgen/core/state.hpp>
#include <arcgen/geometry/workspace.hpp>

#include <boost/geometry.hpp>

#include <cmath>
#include <utility>
#include <vector>

namespace arcgen::geometry
{
    namespace bg = boost::geometry;
    using arcgen::core::State;

    /**
     * @brief Rigid polygonal robot model.
     *
     * Stores a base polygon in the robot's local frame (origin at the
     * chosen reference point, heading along +X). Provides transforms to
     * world poses and utilities to compute swept footprints along paths.
     */
    class Robot
    {
      public:
        /// @brief Default constructor.
        Robot () = default;

        /**
         * @brief Construct from a body-frame polygon.
         * @param body CCW, closed polygon with reference point at (0,0).
         */
        explicit Robot (Polygon body) : body_ (std::move (body)) { bg::correct (body_); }

        /**
         * @brief Factory: axis-aligned rectangle with selectable reference point.
         * @param length Total length along +X (fore–aft), meters.
         * @param width  Total width along +Y, meters.
         * @param refX   Reference point inside the rectangle expressed in the
         *               rectangle's centered frame (origin at its geometric center),
         *               +X forward, +Y left. For example, `refX = -length/2, refY = 0`
         *               places the reference at the rear-center; `refX = 0, refY = 0`
         *               keeps it at the geometric center.
         * @param refY   Reference Y coordinate in the centered frame (meters).
         * @return Robot polygon whose chosen reference point is at (0,0) and heading +X.
         */
        static Robot rectangle (double length, double width, double refX, double refY)
        {
            const double halfLength = 0.5 * length;
            const double halfWidth = 0.5 * width;
            Polygon polygon;
            // Start with rectangle centered at origin (body frame), then shift so that
            // the requested reference (refX, refY) moves to (0,0).
            polygon.outer () = {{-halfLength - refX, -halfWidth - refY},
                                {halfLength - refX, -halfWidth - refY},
                                {halfLength - refX, halfWidth - refY},
                                {-halfLength - refX, halfWidth - refY},
                                {-halfLength - refX, -halfWidth - refY}}; // CCW
            bg::correct (polygon);
            return Robot{std::move (polygon)};
        }

        /**
         * @brief Convenience: axis-aligned rectangle centered at origin (ref at center).
         * @param length Total length along +X (fore–aft), meters.
         * @param width  Total width along +Y, meters.
         */
        static Robot rectangle (double length, double width) { return rectangle (length, width, 0.0, 0.0); }

        /// @brief Access the body-frame polygon (CCW, closed).
        [[nodiscard]] const Polygon &body () const noexcept { return body_; }

        /**
         * @brief Compute the world-frame polygon at a given pose.
         * @param x        World X position.
         * @param y        World Y position.
         * @param heading  Heading angle (rad).
         * @return Transformed polygon in world frame.
         */
        [[nodiscard]] Polygon at (double x, double y, double heading) const { return transform (body_, x, y, heading); }

        /**
         * @brief Compute the world-frame polygon at a given state.
         * @param s  State containing position and heading.
         * @return Transformed polygon in world frame.
         */
        [[nodiscard]] Polygon at (const State &s) const { return at (s.x, s.y, s.heading); }

      private:
        /**
         * @brief Apply rotation about origin then translation to a polygon.
         * @param src      Source polygon (body frame).
         * @param tx       Translation X (world).
         * @param ty       Translation Y (world).
         * @param theta    Heading angle (rad).
         * @return Transformed polygon in world frame.
         */
        static Polygon transform (const Polygon &src, double tx, double ty, double theta)
        {
            const double thetaNorm = arcgen::core::normalizeAngleSigned (theta);
            const double cosTheta = std::cos (thetaNorm);
            const double sinTheta = std::sin (thetaNorm);
            Polygon out;
            out.outer ().clear ();
            out.outer ().reserve (src.outer ().size ());
            for (const auto &pt : src.outer ())
            {
                const double x = pt.x ();
                const double y = pt.y ();
                const double xr = cosTheta * x - sinTheta * y + tx;
                const double yr = sinTheta * x + cosTheta * y + ty;
                out.outer ().push_back (Point{xr, yr});
            }
            bg::correct (out);
            return out;
        }

        Polygon body_{}; ///< Base polygon in body frame (CCW, closed; reference at origin).
    };

} // namespace arcgen::geometry
