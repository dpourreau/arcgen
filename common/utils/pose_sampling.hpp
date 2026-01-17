#pragma once
/**
 * @file   pose_sampling.hpp
 * @brief  Helpers to build oriented half-disks and test local reachability.
 */

#include <arcgen.hpp>

#include <boost/geometry.hpp>

#include <cmath>
#include <vector>

namespace arcgen::utils
{
    using arcgen::core::PI;

    namespace bg = boost::geometry;

    /**
     * @brief Oriented half-disk polygon (CCW, closed) centered at (cx,cy).
     *
     * The arc spans 180° around @p heading. If @p forward==true, the cap is
     * in front of the pose; otherwise it's behind (i.e., rotated by π).
     *
     * The closing segment is the diameter chord between the two arc endpoints.
     *
     * @param cx        Center X.
     * @param cy        Center Y.
     * @param heading   Pose heading (rad).
     * @param radius    Half-disk radius (m).
     * @param forward   True ⇒ forward cap; false ⇒ backward cap.
     * @param segments  Number of points along the semicircle (≥ 2).
     * @return CCW, closed polygon approximating the half-disk.
     */
    inline arcgen::planner::geometry::Polygon orientedHalfDisk (double cx, double cy, double heading, double radius, bool forward, int segments = 24)
    {
        arcgen::planner::geometry::Polygon poly;
        if (segments < 2)
            segments = 2;

        // Start/end angles: [θ - π/2, θ + π/2] for forward; add π for backward.
        const double base = heading + (forward ? 0.0 : PI);
        const double a0 = base - PI / 2.0;
        const double a1 = base + PI / 2.0;

        auto &ring = poly.outer ();
        ring.reserve (static_cast<std::size_t> (segments + 1));

        // Sample along the arc CCW from a0 to a1 inclusive.
        for (int i = 0; i <= segments; ++i)
        {
            const double t = static_cast<double> (i) / static_cast<double> (segments);
            const double a = std::lerp (a0, a1, t);
            ring.emplace_back (cx + radius * std::cos (a), cy + radius * std::sin (a));
        }

        // Close ring (Boost.Geometry requires closed polygons).
        ring.push_back (ring.front ());
        bg::correct (poly);
        return poly;
    }

    /**
     * @brief Test if an oriented half-disk around a pose is fully inside a workspace.
     * @param W        Workspace.
     * @param cx,cy    Pose position.
     * @param heading  Pose heading (rad).
     * @param radius   Half-disk radius (m).
     * @param forward  Which cap to test (true=forward, false=backward).
     * @return True if the half-disk area is covered by the valid region.
     */
    inline bool halfDiskInside (const arcgen::planner::geometry::Workspace &W, double cx, double cy, double heading, double radius, bool forward)
    {
        // Slight shrink to avoid grazing numerical issues on borders.
        const double r = std::max (0.0, radius - 1e-9);
        arcgen::planner::geometry::Polygon hd = orientedHalfDisk (cx, cy, heading, r, forward, 28);
        return W.coveredBy (hd);
    }
} // namespace arcgen::utils
