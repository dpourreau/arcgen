#pragma once
/**
 * @file search_engine.hpp
 * @brief High-level three-stage planner combining a steering-law, graph search, and a skeleton generator.
 *
 * Pipeline:
 *  - Stage 1: direct steering-law connection (enumerate + constraints).
 *  - Stage 2: local skeleton (AABB around start/goal + 2 * R_min margin).
 *  - Stage 3: global skeleton (built once, reused).
 *
 * Parallel highlight: connection evaluation can run in parallel over steering candidates
 * with a thread-safe argmin selection.
 */

#include <arcgen/core/state.hpp>
#include <arcgen/planner/connector/connector.hpp>
#include <arcgen/planner/connector/greedy_connector.hpp>
#include <arcgen/planner/constraints/collision.hpp>
#include <arcgen/planner/constraints/constraints.hpp>
#include <arcgen/planner/constraints/path_length.hpp>
#include <arcgen/planner/engine/evaluator.hpp>
#include <arcgen/planner/geometry/skeleton.hpp>
#include <arcgen/planner/geometry/workspace.hpp>
#include <arcgen/planner/graph/astar.hpp> // A* adaptor
#include <arcgen/planner/graph/graph_search.hpp>
#include <arcgen/steering/steering.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <concepts>
#include <memory>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

namespace bg = boost::geometry;

#include <expected>

namespace arcgen::planner::engine
{
    using namespace arcgen::core;
    using namespace arcgen::planner::geometry;
    using namespace arcgen::planner::connector;

    /**
     * @brief Error codes for the planning process.
     */
    enum class PlanningError
    {
        None,
        InvalidInput,  ///< Start/Goal invalid or in collision
        NoPathFound,   ///< No feasible path found
        InternalError, ///< Engine not initialized correctly
        Timeout        ///< (Reserved for future use)
    };

    /**
     * @brief Concepts to validate template arguments at compile-time.
     */
    namespace detail
    {
        template <class S>
        concept SteeringLike = requires (S s, const State &a, const State &b, typename S::PathType &p) {
            { S::kSegments } -> std::convertible_to<std::size_t>;
            { s.candidates (a, b) } -> std::same_as<std::vector<typename S::PathType>>;
            { s.ensureStates (a, p) };
            { s.getRadiusMin () } -> std::convertible_to<double>;
        };

        template <class Sk>
        concept SkeletonLike = requires (Sk sk, const Workspace &w) {
            { sk.generate (w) }; // concrete graph type checked via GraphSearchLike below
        };

        template <class GS, class Graph>
        concept GraphSearchLike = requires (GS gs, const Graph &g, const State &a, const State &b) {
            { gs.search (g, a, b) } -> std::same_as<std::vector<State>>;
        };
    } // namespace detail

    /**
     * @brief Three-stage search engine orchestrating steering enumeration, graph-search over a skeleton and stitching.
     *
     * @tparam Steering    Steering policy type (must satisfy @c detail::SteeringLike).
     * @tparam GraphSearch Graph search adaptor (must satisfy @c detail::GraphSearchLike on the skeleton graph type).
     * @tparam Skeleton    Skeleton generator (must satisfy @c detail::SkeletonLike).
     * @tparam Connector   Connector template that will be instantiated with (Steering, DebugInfo).
     */
    template <typename Steering, typename GraphSearch, typename Skeleton, template <class, class> class Connector = connector::GreedyConnector> class SearchEngine
    {
        using PathT = typename Steering::PathType;
        static constexpr std::size_t N = Steering::kSegments;

        using ConstraintSet = arcgen::planner::constraints::ConstraintSet<N>;

        /// Graph type produced by the skeleton generator.
        using GlobalGraph = std::decay_t<decltype (std::declval<Skeleton> ().generate (std::declval<const Workspace &> ()))>;

        // Compile-time interface checks (kept light-weight).
        static_assert (detail::SteeringLike<Steering>, "Steering does not satisfy SteeringLike.");
        static_assert (detail::SkeletonLike<Skeleton>, "Skeleton does not satisfy SkeletonLike.");
        static_assert (detail::GraphSearchLike<GraphSearch, GlobalGraph>, "GraphSearch does not satisfy GraphSearchLike for the skeleton graph type.");

      public:
        /**
         * @brief Lightweight debugging bundle filled during planning if requested.
         */
        struct DebugInfo
        {
            enum class Source
            {
                Direct,
                Local,
                Global
            };

            struct TrajectoryStats
            {
                double totalCost{0.0};
                std::unordered_map<std::string, double> softConstraints;
            };

            struct Step
            {
                std::string name;
                std::vector<State> path;
                std::vector<State> fixedAnchors;    // Purple/Big
                std::vector<State> resampledPoints; // Orange/Small
                TrajectoryStats stats;
                double computationTime{0.0};
            };

            /// Sequence of algorithmic steps (initial stitch, smoothing iterations, etc.)
            std::vector<Step> history;

            /// High-level component profiling (e.g., "Skeleton", "Graph Search").
            std::unordered_map<std::string, double> componentTimes;

            // Skeleton graph actually used (copied) when associated with a stage.
            std::optional<GlobalGraph> graph;

            std::optional<Source> source;
            int smoothingIterations{0};
        };

        using ConnectorType = Connector<Steering, DebugInfo>;
        using ConnectorPtr = std::shared_ptr<ConnectorType>;
        static_assert (connector::ConnectorLike<ConnectorType, Steering, DebugInfo>, "Connector does not satisfy ConnectorLike.");

        /**
         * @brief Construct the search engine.
         * @param steering   Steering-law instance.
         * @param graphSearch Graph search adaptor instance.
         * @param skeleton   Skeleton generator instance.
         * @param workspace  Workspace used for collision and skeleton generation.
         * @param connector  Connector responsible for stitching coarse paths (must not be null).
         */
        SearchEngine (std::shared_ptr<Steering> steering, std::shared_ptr<GraphSearch> graphSearch, std::shared_ptr<Skeleton> skeleton, std::shared_ptr<Workspace> workspace,
                      ConnectorPtr connector, bool enableLocalSearch = false, double localBBMargin = 10.0)
            : steering_ (std::move (steering)), graphSearch_ (std::move (graphSearch)), skeleton_ (std::move (skeleton)), workspace_ (std::move (workspace)),
              connector_ (std::move (connector)), enableLocalSearch_ (enableLocalSearch), localBBMargin_ (localBBMargin)
        {
        }

        /**
         * @brief Replace the entire constraint set (hard + weighted soft).
         * @param cset New constraint set.
         */
        void setConstraints (ConstraintSet cset) { constraints_ = std::move (cset); }

        /**
         * @brief Get a const reference to the current constraint set.
         * @return Constraint set (const reference).
         */
        [[nodiscard]] const ConstraintSet &getConstraints () const noexcept { return constraints_; }

        /**
         * @brief Clear all constraints (both hard and soft).
         */
        void clearConstraints ()
        {
            constraints_.hard.clear ();
            constraints_.soft.clear ();
        }

        /**
         * @brief Set the steering-law object.
         * @param steering Shared pointer to a steering implementation.
         */
        void setSteering (std::shared_ptr<Steering> steering) { steering_ = std::move (steering); }

        /**
         * @brief Get the steering-law object.
         * @return Shared pointer to the current steering implementation.
         */
        [[nodiscard]] const std::shared_ptr<Steering> &getSteering () const noexcept { return steering_; }

        /**
         * @brief Set the graph-search adaptor.
         * @param graphSearch Shared pointer to a graph-search adaptor.
         */
        void setGraphSearch (std::shared_ptr<GraphSearch> graphSearch) { graphSearch_ = std::move (graphSearch); }

        /**
         * @brief Get the graph-search adaptor.
         * @return Shared pointer to the current graph-search adaptor.
         */
        [[nodiscard]] const std::shared_ptr<GraphSearch> &getGraphSearch () const noexcept { return graphSearch_; }

        /**
         * @brief Set the skeleton generator. Invalidates any cached global graph.
         * @param skeleton Shared pointer to a skeleton generator.
         */
        void setSkeleton (std::shared_ptr<Skeleton> skeleton)
        {
            skeleton_ = std::move (skeleton);
            invalidateGlobalGraph ();
        }

        /**
         * @brief Get the skeleton generator.
         * @return Shared pointer to the current skeleton generator.
         */
        [[nodiscard]] const std::shared_ptr<Skeleton> &getSkeleton () const noexcept { return skeleton_; }

        /**
         * @brief Set the workspace. Invalidates any cached global graph.
         * @param workspace Shared pointer to a workspace (valid region).
         */
        void setWorkspace (std::shared_ptr<Workspace> workspace)
        {
            workspace_ = std::move (workspace);
            invalidateGlobalGraph ();
        }

        /**
         * @brief Get the workspace.
         * @return Shared pointer to the current workspace.
         */
        [[nodiscard]] const std::shared_ptr<Workspace> &getWorkspace () const noexcept { return workspace_; }

        /**
         * @brief Set the connector responsible for stitching coarse paths (must not be null).
         * @param connector Shared pointer to a connector implementation.
         */
        void setConnector (ConnectorPtr connector) { connector_ = std::move (connector); }

        /**
         * @brief Get the current connector.
         * @return Shared pointer to the connector.
         */
        [[nodiscard]] const ConnectorPtr &getConnector () const noexcept { return connector_; }

        /**
         * @brief Get a pointer to the cached global skeleton graph (if any).
         * @return Non-owning pointer to cached graph, or nullptr if not built.
         */
        [[nodiscard]] const GlobalGraph *getGlobalGraph () const noexcept { return globalGraph_ ? &(*globalGraph_) : nullptr; }

        /**
         * @brief Invalidate (clear) the cached global skeleton graph.
         */
        void invalidateGlobalGraph () noexcept { globalGraph_.reset (); }

        /**
         * @brief Plan without debug info.
         * @param start Start state (unchanged).
         * @param goal  Goal state (unchanged).
         * @return Discretized collision-free path or error code.
         */
        [[nodiscard]] std::expected<std::vector<State>, PlanningError> plan (State &start, State &goal) { return plan (start, goal, nullptr); }
        /**
         * @brief Plan with optional debug capture.
         * @param start Start state (unchanged).
         * @param goal  Goal state (unchanged).
         * @param dbg   Optional debug capture (can be nullptr).
         * @return Discretized collision-free path or error code.
         */
        [[nodiscard]] std::expected<std::vector<State>, PlanningError> plan (State &start, State &goal, DebugInfo *dbg)
        {
            if (dbg)
            {
                dbg->history.clear ();
                dbg->componentTimes.clear ();
                dbg->graph.reset ();
                dbg->source.reset ();
                dbg->smoothingIterations = 0;
            }

            if (!steering_ || !graphSearch_ || !skeleton_ || !workspace_ || !connector_)
                return std::unexpected (PlanningError::InternalError); // badly initialized

            // Check if start/goal are in valid region (basic check)
            if (!workspace_->contains (start.x, start.y) || !workspace_->contains (goal.x, goal.y))
                return std::unexpected (PlanningError::InvalidInput);

            Evaluator<Steering> evaluator (steering_.get (), &constraints_);

            // ── Stage 1: direct (enumerate + constraints)
            {
                // Measure time? Direct connection is usually negligible but we can add it if needed.
                if (auto best = evaluator.bestStatesBetween (start, goal))
                {
                    if (dbg)
                    {
                        // Record direct connection as a step
                        typename DebugInfo::Step step;
                        step.name = "Direct";
                        step.path = best->first;
                        step.stats.totalCost = best->second;
                        if (const auto *cset = evaluator.getConstraints ())
                        {
                            if (!cset->soft.empty ())
                            {
                                // Recalculate breakdown if needed or assumption: single segment cost is simple.
                                // NOTE: Detailed breakdown would require constraints->score to return breakdown.
                                // For now, we can just manually score soft constraints if we want breakdown.
                            }
                        }
                        dbg->history.push_back (std::move (step));
                        dbg->source = DebugInfo::Source::Direct;
                    }
                    return best->first;
                }
            }

            auto tryGraph = [&] (auto &&graph, std::string stageName) -> std::optional<std::vector<State>>
            {
                // Measure Graph Search Time
                auto t0 = std::chrono::steady_clock::now ();
                auto coarse = graphSearch_->search (graph, start, goal);
                auto t1 = std::chrono::steady_clock::now ();

                if (dbg)
                {
                    dbg->componentTimes["Graph Search (" + stageName + ")"] = std::chrono::duration<double> (t1 - t0).count ();
                }

                if (coarse.size () < 2)
                    return std::nullopt;

                if (dbg)
                {
                    dbg->graph = graph; // copy for visualization
                }

                auto path = connector_->connect (evaluator, start, goal, std::move (coarse), dbg);
                if (path.empty ())
                    return std::nullopt;

                if (dbg)
                {
                    if (stageName == "Local")
                        dbg->source = DebugInfo::Source::Local;
                    else if (stageName == "Global")
                        dbg->source = DebugInfo::Source::Global;
                }
                return path;
            };

            // ── Stage 2: local skeleton (axis-aligned rectangle around start/goal + margin)
            if (enableLocalSearch_)
            {
                const double margin = localBBMargin_;
                const double xMin = std::min (start.x, goal.x) - margin;
                const double yMin = std::min (start.y, goal.y) - margin;
                const double xMax = std::max (start.x, goal.x) + margin;
                const double yMax = std::max (start.y, goal.y) + margin;

                Polygon rect;
                rect.outer ().resize (5);
                rect.outer ()[0] = Point{xMin, yMin};
                rect.outer ()[1] = Point{xMax, yMin};
                rect.outer ()[2] = Point{xMax, yMax};
                rect.outer ()[3] = Point{xMin, yMax};
                rect.outer ()[4] = Point{xMin, yMin};
                bg::correct (rect);

                Workspace localValid = workspace_->clippedTo (rect);

                if (!localValid.empty ())
                {
                    auto t0 = std::chrono::steady_clock::now ();
                    auto localGraph = skeleton_->generate (localValid);
                    auto t1 = std::chrono::steady_clock::now ();
                    if (dbg)
                        dbg->componentTimes["Skeleton Generation (Local)"] = std::chrono::duration<double> (t1 - t0).count ();

                    if (auto path = tryGraph (localGraph, "Local"))
                        return *path;
                }
            }

            // ── Stage 3: global skeleton (lazy, cached; rebuild if needed).
            if (!globalGraph_)
            {
                if (skeleton_ && workspace_ && !workspace_->empty ())
                {
                    auto t0 = std::chrono::steady_clock::now ();
                    globalGraph_.emplace (skeleton_->generate (*workspace_));
                    auto t1 = std::chrono::steady_clock::now ();
                    if (dbg)
                        dbg->componentTimes["Skeleton Generation (Global)"] = std::chrono::duration<double> (t1 - t0).count ();
                }
            }

            if (!globalGraph_)
                return std::unexpected (PlanningError::NoPathFound); // workspace empty or skeleton generation failed

            if (auto path = tryGraph (*globalGraph_, "Global"))
                return *path;

            return std::unexpected (PlanningError::NoPathFound);
        }

      private:
        std::shared_ptr<Steering> steering_;       ///< Steering-law policy.
        std::shared_ptr<GraphSearch> graphSearch_; ///< Graph search adaptor.
        std::shared_ptr<Skeleton> skeleton_;       ///< Skeleton generator.
        std::shared_ptr<Workspace> workspace_;     ///< Workspace (valid region).
        std::optional<GlobalGraph> globalGraph_;   ///< Cached global skeleton (built lazily).
        ConnectorPtr connector_;                   ///< Stitching strategy.
        ConstraintSet constraints_;                ///< Active constraints (hard + soft).
        bool enableLocalSearch_;                   ///< Enable Stage 2 local search.
        double localBBMargin_;                     ///< Margin for local bounding box search.
    };

} // namespace arcgen::planner::engine
