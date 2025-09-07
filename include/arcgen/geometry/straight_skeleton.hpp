#pragma once
/**
 * @file   straight_skeleton.hpp
 * @brief  Interior straight-skeleton generator (CGAL) that builds a Boost graph.
 *
 * The produced graph contains only interior bisector edges of the valid region
 * (border vertices are filtered out). Each edge is weighted by its Euclidean length.
 */

#include <arcgen/geometry/skeleton.hpp>
#include <arcgen/geometry/workspace.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Straight_skeleton_2.h>
#include <CGAL/Unique_hash_map.h>
#include <CGAL/create_straight_skeleton_2.h>

#include <cmath>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

namespace arcgen::geometry
{
    /* ────────── CGAL / Boost aliases ────────────────────────────── */
    using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
    using CGALPoint = CGAL::Point_2<Kernel>;
    using CGALPolygon = CGAL::Polygon_2<Kernel>;
    using CGALSkeleton = CGAL::Straight_skeleton_2<Kernel>;

    /// @brief Output graph: Boost undirected graph with vertex = Point and edge weight = length.
    using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Point, boost::property<boost::edge_weight_t, double>>;

    /* ────────── generic helpers ─────────────────────────────────── */
    namespace detail
    {
        /**
         * @brief Convert a Boost.Geometry ring (closed, last = first) to a CGAL polygon.
         * @tparam Ring Ring type with `.size()`, `.x()`, `.y()` accessors.
         * @param ring Closed ring where the last point duplicates the first.
         * @return CGAL polygon with the duplicate last vertex dropped.
         */
        template <class Ring> inline CGALPolygon makeCgalPolygon (const Ring &ring)
        {
            CGALPolygon poly;
            const std::size_t n = ring.size () - 1; // skip duplicate last point
            poly.reserve (n);
            for (std::size_t i = 0; i < n; ++i)
                poly.push_back (CGALPoint (ring[i].x (), ring[i].y ()));
            return poly;
        }

        /// @brief Ensure polygon is counter-clockwise oriented (reverses in place if needed).
        inline void ensureCcw (CGALPolygon &p)
        {
            if (!p.is_counterclockwise_oriented ())
                p.reverse_orientation ();
        }

        /// @brief Ensure polygon is clockwise oriented (reverses in place if needed).
        inline void ensureCw (CGALPolygon &p)
        {
            if (!p.is_clockwise_oriented ())
                p.reverse_orientation ();
        }
    } // namespace detail

    /**
     * @brief Straight-skeleton generator that converts a @ref Workspace into a graph.
     *
     * The class filters out skeleton vertices that fall on the boundary of the
     * valid region (keeps only interior points) and assigns edge weights equal to
     * the Euclidean distance between endpoints.
     */
    class StraightSkeleton final : public SkeletonBase<StraightSkeleton, Workspace>
    {
      private:
        using VertexHandle = CGALSkeleton::Vertex_const_handle;

        /**
         * @brief Helper that maps CGAL skeleton vertices to Boost vertices with caching.
         */
        struct VertexMapper
        {
            Graph &graph_;                                                        ///< Target Boost graph.
            const Workspace &workspace_;                                          ///< Workspace used to filter interior vertices.
            CGAL::Unique_hash_map<VertexHandle, Graph::vertex_descriptor> cache_; ///< CGAL→Boost cache.

            /**
             * @brief Construct a mapper bound to a graph and workspace.
             * @param graph      Output Boost graph to receive vertices.
             * @param workspace  Valid region used to filter interior vertices.
             */
            explicit VertexMapper (Graph &graph, const Workspace &workspace) : graph_ (graph), workspace_ (workspace), cache_ (boost::graph_traits<Graph>::null_vertex ()) {}

            /**
             * @brief Get (or create and cache) a Boost vertex for a CGAL vertex.
             * @param vh CGAL vertex handle.
             * @return Optional Boost vertex descriptor (std::nullopt if on border).
             */
            std::optional<Graph::vertex_descriptor> get (VertexHandle vh)
            {
                const auto null_v = boost::graph_traits<Graph>::null_vertex ();
                if (cache_.is_defined (vh))
                {
                    const auto vd = cache_[vh];
                    return (vd == null_v) ? std::nullopt : std::optional{vd};
                }

                const auto &p = vh->point ();
                const double x = CGAL::to_double (p.x ());
                const double y = CGAL::to_double (p.y ());

                // Filter out border points; keep only interior vertices.
                if (!workspace_.contains (x, y))
                {
                    cache_[vh] = null_v; // cache the negative result too
                    return std::nullopt;
                }

                auto vd = add_vertex (Point{x, y}, graph_);
                cache_[vh] = vd;
                return vd;
            }
        };

      public:
        /**
         * @brief Generate a straight-skeleton graph for the given workspace.
         * @param workspace Workspace (valid region).
         * @return Boost graph of interior skeleton edges with Euclidean weights.
         */
        [[nodiscard]] Graph generateImpl (const Workspace &workspace) const
        {
            Graph graph;
            if (workspace.empty ())
                return graph;

            auto weightMap = get (boost::edge_weight, graph);

            // Build a skeleton for each connected component of the valid region.
            for (const auto &component : workspace.region ())
            {
                if (component.outer ().size () < 4)
                    continue; // degenerate

                /* ---- outer ring (CCW) ------------------------------------- */
                CGALPolygon outer = detail::makeCgalPolygon (component.outer ());
                detail::ensureCcw (outer);

                /* ---- holes (CW) ------------------------------------------- */
                std::vector<CGALPolygon> holes;
                holes.reserve (component.inners ().size ());
                for (const auto &h : component.inners ())
                {
                    CGALPolygon hole = detail::makeCgalPolygon (h);
                    detail::ensureCw (hole);
                    holes.push_back (std::move (hole));
                }

                /* ---- build interior straight skeleton -------------------- */
                auto skeleton = holes.empty () ? CGAL::create_interior_straight_skeleton_2 (outer, Kernel{})
                                               : CGAL::create_interior_straight_skeleton_2 (outer.vertices_begin (), outer.vertices_end (), holes.begin (), holes.end (), Kernel{});

                if (!skeleton)
                    continue; // numerical failure

                /* ---- map CGAL vertices → Boost vertices ------------------ */
                VertexMapper mapper{graph, workspace};

                /* ---- collect interior bisector edges ---------------------- */
                for (auto he = skeleton->halfedges_begin (); he != skeleton->halfedges_end (); ++he)
                {
                    if (!he->is_bisector ())
                        continue;

                    // Process each undirected bisector edge only once.
                    if (&*he > &*(he->opposite ()))
                        continue;

                    auto v0 = mapper.get (he->opposite ()->vertex ());
                    auto v1 = mapper.get (he->vertex ());
                    if (!v0 || !v1)
                        continue; // one endpoint on the border

                    const Point p0 = graph[*v0];
                    const Point p1 = graph[*v1];

                    const double dx = p1.x () - p0.x ();
                    const double dy = p1.y () - p0.y ();
                    const double dist = std::hypot (dx, dy);

                    auto [e, ok] = add_edge (*v0, *v1, graph);
                    (void)ok;
                    weightMap[e] = dist;
                }
            }
            return graph;
        }
    };

} // namespace arcgen::geometry
