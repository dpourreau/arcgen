#pragma once
/**
 * @file
 * @brief A* graph-search adaptor for Boost.Graph skeletons.
 *
 * Works with any Boost **undirected** graph whose vertex property provides
 * `x()` and `y()` accessors convertible to `double`, and whose edges expose a
 * weight through `boost::edge_weight_t`. Returns the *intermediate* states
 * (start and goal are omitted).
 */

#include <arcgen/core/state.hpp>
#include <arcgen/planner/graph/graph_search.hpp>

#include <boost/graph/astar_search.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/function_property_map.hpp>
#include <boost/range/iterator_range.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace arcgen::planner::graph
{
    using arcgen::core::State;

    /**
     * @brief A* adaptor over a Boost.Graph-like skeleton.
     *
     * @tparam Graph Undirected Boost graph whose **vertex property** exposes
     *               `x()` / `y()` methods convertible to `double` and that has
     *               edge weights accessible through `boost::edge_weight_t`.
     *
     * This class implements the CRTP hook expected by @ref GraphSearchBase and
     * provides a Euclidean straight-line heuristic.
     */
    template <typename Graph> class AStar : public GraphSearchBase<AStar<Graph>, Graph>
    {
        using Vertex = typename boost::graph_traits<Graph>::vertex_descriptor;

        /*───────────────────────────── helpers ─────────────────────────────*/

        /// @brief Internal exception used to break out of Boost's A* once the goal is reached.
        struct GoalFound
        {
        };

        /// @brief A* visitor that stops the search when the goal vertex is examined.
        struct ExitVisitor : boost::default_astar_visitor
        {
            Vertex goal_;
            explicit ExitVisitor (Vertex g) : goal_ (g) {}

            template <class GT> void examine_vertex (Vertex u, const GT &)
            {
                if (u == goal_)
                    throw GoalFound{};
            }
        };

        /// @brief Straight-line distance (Euclidean) heuristic.
        struct Heuristic
        {
            const Graph &graph_;
            Vertex goal_;

            /// @brief Evaluate the heuristic at a vertex.
            [[nodiscard]] double operator() (Vertex u) const
            {
                auto const &tgt = graph_[goal_];
                auto const &cur = graph_[u];
                return std::hypot (cur.x () - tgt.x (), cur.y () - tgt.y ());
            }
        };

      public:
        /**
         * @brief CRTP implementation entry point.
         * @param graph Graph to search.
         * @param start Start pose in world coordinates.
         * @param goal  Goal pose in world coordinates.
         * @return Intermediate states along the skeleton (start/goal omitted). Empty on failure.
         */
        [[nodiscard]] std::vector<State> searchImpl (const Graph &graph, const State &start, const State &goal) const
        {
            const std::size_t numVertices = num_vertices (graph);
            if (numVertices == 0)
                return {};

            // 1) Nearest graph vertex to an arbitrary (x, y).
            auto nearestVertex = [&] (const State &s) -> std::optional<Vertex>
            {
                double best = std::numeric_limits<double>::max ();
                std::optional<Vertex> pick{};

                for (Vertex v : boost::make_iterator_range (vertices (graph)))
                {
                    auto const &p = graph[v];
                    const double dx = p.x () - s.x;
                    const double dy = p.y () - s.y;
                    const double d2 = dx * dx + dy * dy;

                    if (d2 < best)
                    {
                        best = d2;
                        pick = v;
                    }
                }
                return pick;
            };

            auto vStartOpt = nearestVertex (start);
            auto vGoalOpt = nearestVertex (goal);
            if (!vStartOpt || !vGoalOpt)
                return {}; // empty graph or disconnected

            Vertex vStart = *vStartOpt;
            Vertex vGoal = *vGoalOpt;

            // 2) Storage for the search.
            std::vector<Vertex> predecessor (numVertices);
            std::vector<double> distance (numVertices, std::numeric_limits<double>::max ());
            std::vector<boost::default_color_type> color (numVertices);
            auto index = get (boost::vertex_index, graph);

            // 3) Run A*.
            try
            {
                boost::astar_search (graph, vStart, Heuristic{graph, vGoal},
                                     boost::weight_map (get (boost::edge_weight, graph))
                                         .predecessor_map (boost::make_iterator_property_map (predecessor.begin (), index))
                                         .distance_map (boost::make_iterator_property_map (distance.begin (), index))
                                         .color_map (boost::make_iterator_property_map (color.begin (), index))
                                         .visitor (ExitVisitor{vGoal}));
            }
            catch (const GoalFound &)
            {
                // Success: goal reached.
            }
            catch (const std::exception &)
            {
                return {};
            }

            if (distance[index[vGoal]] == std::numeric_limits<double>::max ())
                return {}; // goal unreachable

            // 4) Rebuild path (exclude start/goal).
            std::vector<State> coarse;
            for (Vertex v = vGoal; v != vStart; v = predecessor[index[v]])
            {
                if (v == vStart || v == vGoal)
                    continue;

                auto const &p = graph[v];
                coarse.push_back (State{p.x (), p.y ()});
            }
            std::ranges::reverse (coarse);
            return coarse;
        }
    };

} // namespace arcgen::planner::graph
