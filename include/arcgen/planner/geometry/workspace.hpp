#pragma once
/**
 * @file    workspace.hpp
 * @brief   Polygon-set wrapper that precomputes the valid region
 *          as (outer ⊖ ⋃obstacles) and provides fast spatial queries.
 *
 * - Both outer and obstacle polygons must already include any safety margin.
 * - All polygons are CCW and closed (arcgen::planner::geometry::Polygon = bg::polygon<Point, false, true>).

 */

#include <arcgen/core/state.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/index/adaptors/query.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace arcgen::planner::geometry
{
    using namespace arcgen::core;

    namespace bg = boost::geometry;
    namespace bgi = boost::geometry::index;

    /// 2D point (x,y) in double precision.
    using Point = bg::model::d2::point_xy<double>;
    /// CCW, closed polygon type used across the project.
    using Polygon = bg::model::polygon<Point, /*CCW=*/false, /*closed=*/true>;
    /// Collection of polygons (possibly with holes).
    using MultiPolygon = bg::model::multi_polygon<Polygon>;
    /// Axis-aligned bounding box.
    using BBox = bg::model::box<Point>;

    /**
     * @brief Workspace that stores the valid region and supports fast queries.
     *
     * Internally builds an R-tree of polygon envelopes so that point queries.
     */
    class Workspace final
    {
      public:
        /**
         * @brief Construct the workspace's valid region as: outer ⊖ (⋃ obstacles).
         * @param outer       Outer boundary polygon (CCW, closed).
         * @param obstacles   Optional list of obstacle polygons (CCW, closed).
         */
        explicit Workspace (Polygon outer, std::vector<Polygon> obstacles = {})
        {
            // 1) Normalize orientation & closure (cheap).
            bg::correct (outer);
            for (auto &p : obstacles)
                bg::correct (p);

            // 2) Union all obstacles.
            MultiPolygon obstaclesUnion;
            for (const auto &p : obstacles)
            {
                MultiPolygon tmp;
                bg::union_ (obstaclesUnion, p, tmp); // three-argument union_
                obstaclesUnion = std::move (tmp);
            }

            // 3) Subtract obstacles from the outer polygon to build the valid region.
            if (obstaclesUnion.empty ())
            {
                region_.push_back (std::move (outer)); // no obstacles
            }
            else
            {
                bg::difference (outer, obstaclesUnion, region_);
            }

            buildRTree ();
        }

        /**
         * @brief Test if a point is inside the valid region.
         * @param x X coordinate.
         * @param y Y coordinate.
         * @return True if (x,y) lies strictly inside the valid region.
         */
        [[nodiscard]] bool contains (double x, double y) const
        {
            const Point p{x, y};
            BBox box{p, p};

            // Query polygons whose envelopes intersect the point's zero-area box.
            for (const auto &entry : rtree_ | bgi::adaptors::queried (bgi::intersects (box)))
            {
                const auto idx = entry.second;
                if (bg::within (p, region_[idx]))
                    return true;
            }
            return false;
        }

        /**
         * @brief Test whether all states in a path are inside the valid region.
         * @param path Sequence of SE(2) states to test.
         * @return True if every state's (x,y) is inside the valid region.
         */
        [[nodiscard]] bool contains (const std::vector<State> &path) const
        {
            for (const auto &s : path)
                if (!contains (s.x, s.y))
                    return false;
            return true;
        }

        /**
         * @brief Optimized coverage test for a single polygon using the envelope R-tree.
         *
         * A connected polygon can only be fully covered if it lies entirely inside a
         * single connected component of the valid region. We leverage the R-tree to
         * filter candidate region polygons by envelope intersection and then run an
         * exact covered_by() test against each candidate.
         *
         * @param p Polygon to test.
         * @return True if @p p is fully covered by the valid region; false otherwise.
         */
        [[nodiscard]] bool coveredBy (const Polygon &p) const
        {
            if (region_.empty ())
                return false;

            BBox pb;
            bg::envelope (p, pb);

            for (const auto &entry : rtree_ | bgi::adaptors::queried (bgi::intersects (pb)))
            {
                const auto idx = entry.second;
                if (bg::covered_by (p, region_[idx]))
                    return true; // fully inside this connected component
            }

            // No candidate envelopes overlapped, or none fully covered the polygon
            return false;
        }

        /**
         * @brief Intersect the current valid region with an arbitrary polygon.
         * @param clip Polygon to clip with.
         * @return New workspace whose valid region is (this ∩ clip). If empty, the
         *         returned workspace will also be empty (i.e., contains() is false for any point).
         */
        [[nodiscard]] Workspace clippedTo (const Polygon &clip) const
        {
            MultiPolygon clipped;
            bg::intersection (region_, clip, clipped);
            return Workspace{std::move (clipped)};
        }

        /// @brief Quick test for an empty valid region.
        [[nodiscard]] bool empty () const noexcept { return region_.empty (); }

        /// @brief Access the precomputed valid region (multi-polygon).
        [[nodiscard]] const MultiPolygon &region () const noexcept { return region_; }

      private:
        /**
         * @brief Private constructor from an already-built multi-polygon.
         * @param mp Precomputed valid region.
         */
        explicit Workspace (MultiPolygon mp) : region_ (std::move (mp)) { buildRTree (); }

        /// @brief Rebuild the envelope R-tree for fast spatial queries.
        void buildRTree ()
        {
            rtree_.clear ();
            for (std::size_t i = 0; i < region_.size (); ++i)
            {
                BBox bb;
                bg::envelope (region_[i], bb);
                rtree_.insert (std::make_pair (bb, i));
            }
        }

        /* Spatial index: polygon envelope → polygon index. */
        using RTree = bgi::rtree<std::pair<BBox, std::size_t>, bgi::quadratic<16>>;
        RTree rtree_;

        /* The final valid region (possibly multiple disjoint polygons with holes). */
        MultiPolygon region_;
    };
} // namespace arcgen::planner::geometry
