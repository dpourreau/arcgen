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
 * Parallel highlight: pickBest() evaluates candidate feasibility + score across
 * all steering candidates in parallel, with a thread-safe argmin selection.
 */

#include <arcgen/core/math.hpp>
#include <arcgen/core/numeric.hpp>
#include <arcgen/geometry/skeleton.hpp>
#include <arcgen/geometry/workspace.hpp>
#include <arcgen/planning/constraints/collision.hpp>
#include <arcgen/planning/constraints/constraints.hpp>
#include <arcgen/planning/constraints/path_length.hpp>
#include <arcgen/planning/search/graph_search.hpp>
#include <arcgen/steering/steering.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <concepts>
#include <limits>
#include <memory>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

#if defined(_OPENMP)
    #include <omp.h>
#endif

namespace bg = boost::geometry;

namespace arcgen::planning::engine
{
    using namespace arcgen::core;
    using namespace arcgen::geometry;

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
     */
    template <typename Steering, typename GraphSearch, typename Skeleton> class SearchEngine
    {
        using PathT = typename Steering::PathType;
        static constexpr std::size_t N = Steering::kSegments;

        using ConstraintSet = arcgen::planning::constraints::ConstraintSet<N>;
        using EvalContext = arcgen::planning::constraints::EvalContext<N>;

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
            /// Which planning stage produced the result.
            enum class StageUsed : std::uint8_t
            {
                Direct,
                Local,
                Global
            };

            StageUsed stage{StageUsed::Direct};                             ///< Stage that succeeded.
            std::vector<State> coarse;                                      ///< Output of graph search (+ assigned headings)
            std::vector<State> waypoints;                                   ///< start + coarse + goal (what stitch() uses)
            std::vector<std::pair<std::size_t, std::size_t>> stitchedPairs; ///< (i, j) indices in @ref waypoints that were stitched

            // Skeleton graph actually used (copied) when stage is Local/Global; empty for Direct.
            std::optional<GlobalGraph> graph;
        };

        /**
         * @brief Construct the search engine.
         * @param steering   Steering-law instance.
         * @param graphSearch Graph search adaptor instance.
         * @param skeleton   Skeleton generator instance.
         * @param workspace  Workspace used for collision and skeleton generation.
         */
        SearchEngine (std::shared_ptr<Steering> steering, std::shared_ptr<GraphSearch> graphSearch, std::shared_ptr<Skeleton> skeleton, std::shared_ptr<Workspace> workspace)
            : steering_ (std::move (steering)), graphSearch_ (std::move (graphSearch)), skeleton_ (std::move (skeleton)), workspace_ (std::move (workspace))
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
         * @return Discretized collision-free path; empty on failure.
         */
        [[nodiscard]] std::vector<State> plan (State &start, State &goal) { return plan (start, goal, nullptr); }

        /**
         * @brief Plan with optional debug capture.
         * @param start Start state (unchanged).
         * @param goal  Goal state (unchanged).
         * @param dbg   Optional debug capture (can be nullptr).
         * @return Discretized collision-free path; empty on failure.
         */
        [[nodiscard]] std::vector<State> plan (State &start, State &goal, DebugInfo *dbg)
        {
            if (!steering_ || !graphSearch_ || !skeleton_)
                return {}; // badly initialized

            // ── Stage 1: direct (enumerate + constraints)
            {
                auto cand = steering_->candidates (start, goal);
                if (auto best = pickBest (*steering_, start, goal, cand, constraints_))
                {
                    if (dbg)
                    {
                        dbg->stage = DebugInfo::StageUsed::Direct;
                        dbg->coarse.clear ();
                        dbg->waypoints.clear ();
                        dbg->waypoints.push_back (start);
                        dbg->waypoints.push_back (goal);
                        dbg->stitchedPairs.clear ();
                        dbg->stitchedPairs.emplace_back (0, 1);
                        dbg->graph.reset ();
                    }
                    // states already ensured by pickBest
                    return *(best->states);
                }
            }

            // Small helper to run graph-search + stitching on an arbitrary graph
            auto tryGraph = [&] (auto &&graph, typename DebugInfo::StageUsed stage) -> std::vector<State>
            {
                auto coarse = graphSearch_->search (graph, start, goal);
                if (coarse.size () < 2)
                    return {};

                assignHeadings (coarse);
                if (dbg)
                    dbg->coarse = coarse;

                std::vector<State> waypoints;
                waypoints.reserve (coarse.size () + 2);
                waypoints.push_back (start);
                waypoints.insert (waypoints.end (), coarse.begin (), coarse.end ());
                waypoints.push_back (goal);

                if (dbg)
                {
                    dbg->waypoints = waypoints;
                    dbg->stage = stage;
                    dbg->graph = graph; // copy for visualization
                }

                return stitch (waypoints, dbg);
            };

            // ── Stage 2: local skeleton (axis-aligned rectangle around start/goal + margin)
            {
                const double margin = 2.0 * steering_->getRadiusMin ();
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
                    auto localGraph = skeleton_->generate (localValid);
                    if (auto path = tryGraph (localGraph, DebugInfo::StageUsed::Local); !path.empty ())
                        return path;
                }
            }

            // ── Stage 3: global skeleton (lazy, cached; rebuild if needed).
            if (!globalGraph_)
            {
                if (skeleton_ && workspace_ && !workspace_->empty ())
                    globalGraph_.emplace (skeleton_->generate (*workspace_));
            }

            if (!globalGraph_)
                return {}; // workspace empty or skeleton generation failed

            return tryGraph (*globalGraph_, DebugInfo::StageUsed::Global);
        }

      private:
        /**
         * @brief Pick the feasible candidate with the best score (soft constraints), falling back to geometric length.
         * @param s     Steering policy.
         * @param a     Start state.
         * @param b     Goal state.
         * @param cand  Candidate paths (modified only to ensure states lazily).
         * @param cset  Constraint set.
         * @return Best candidate if any; std::nullopt otherwise.
         */
        static std::optional<PathT> pickBest (const Steering &s, const State &a, const State &b, std::vector<PathT> &cand, const ConstraintSet &cset)
        {
            EvalContext ctx{a, b, [&] (PathT &p) { s.ensureStates (a, p); }};

            std::optional<std::size_t> argmin;
            double best = std::numeric_limits<double>::infinity ();

#if defined(_OPENMP)
            const std::size_t n = cand.size ();
    #pragma omp parallel if (static_cast<int> (n) > 1)
            {
                double bestLocal = std::numeric_limits<double>::infinity ();
                std::optional<std::size_t> argLocal;

    #pragma omp for nowait
                for (std::ptrdiff_t i = 0; i < static_cast<std::ptrdiff_t> (n); ++i)
                {
                    auto &c = cand[static_cast<std::size_t> (i)];
                    if (!cset.feasible (c, ctx))
                        continue;

                    const double score = cset.soft.empty () ? c.length () : cset.score (c, ctx);
                    if (score < bestLocal)
                    {
                        bestLocal = score;
                        argLocal = static_cast<std::size_t> (i);
                    }
                }

    #pragma omp critical
                {
                    if (argLocal && bestLocal < best)
                    {
                        best = bestLocal;
                        argmin = *argLocal;
                    }
                }
            }
#else
            for (std::size_t i = 0; i < cand.size (); ++i)
            {
                if (!cset.feasible (cand[i], ctx))
                    continue;

                const double score = cset.soft.empty () ? cand[i].length () : cset.score (cand[i], ctx);
                if (score < best)
                {
                    best = score;
                    argmin = i;
                }
            }
#endif

            if (!argmin)
                return std::nullopt;

            s.ensureStates (a, cand[*argmin]); // ensure discretised states before returning
            return cand[*argmin];
        }

        /**
         * @brief Assign forward headings for a polyline-like list of 2D points.
         * @param pts Points whose headings will be set in-place.
         */
        static void assignHeadings (std::vector<State> &pts)
        {
            const std::size_t n = pts.size ();
            if (n < 2)
                return;

            for (std::size_t i = 0; i + 1 < n; ++i)
            {
                const double dx = pts[i + 1].x - pts[i].x;
                const double dy = pts[i + 1].y - pts[i].y;

                if (std::fabs (dx) < arcgen::core::CURVATURE_TOL && std::fabs (dy) < arcgen::core::CURVATURE_TOL)
                {
                    if (i > 0)
                        pts[i].heading = pts[i - 1].heading;
                    continue;
                }

                pts[i].heading = normalizeAngleSigned (std::atan2 (dy, dx));
            }

            pts.back ().heading = pts[n - 2].heading;
        }

        /**
         * @brief Stitch a sequence of waypoints using the steering policy under constraints.
         *        Greedy farthest-reachable jumps are attempted first.
         * @param nodes Waypoints: start + (coarse) + goal.
         * @param dbg   Optional debug info sink.
         * @return Discretized states of the stitched path; empty if stitching fails.
         */
        std::vector<State> stitch (std::vector<State> &nodes, DebugInfo *dbg = nullptr) const
        {
            const std::size_t count = nodes.size ();
            if (count < 3)
                return {};

            std::vector<State> out;
            std::vector<bool> triedFail (count * count, false);

            auto reachable = [&] (std::size_t i, std::size_t j) -> bool
            {
                if (triedFail[i * count + j])
                    return false;

                auto cand = steering_->candidates (nodes[i], nodes[j]);
                if (auto best = pickBest (*steering_, nodes[i], nodes[j], cand, constraints_))
                {
                    // merge states; avoid duplicating the junction point
                    if (!out.empty () && best->states && !best->states->empty ())
                        best->states->erase (best->states->begin ());

                    out.insert (out.end (), best->states->begin (), best->states->end ());

                    if (dbg)
                        dbg->stitchedPairs.emplace_back (i, j);
                    return true;
                }
                triedFail[i * count + j] = true;
                return false;
            };

            std::size_t i = 0;
            while (i < count - 1)
            {
                std::size_t j = i + 1;

                // Try the farthest reachable node first, walking backward.
                for (std::size_t k = count - 1; k > i; --k)
                {
                    if (i == 0 && k == count - 1)
                        continue;
                    if (reachable (i, k))
                    {
                        j = k;
                        break;
                    }
                }

                // If the immediate neighbor was tried and failed, no path exists.
                if (j == i + 1 && triedFail[i * count + j])
                    return {};

                i = j;
            }
            return out;
        }

      private:
        std::shared_ptr<Steering> steering_;       ///< Steering-law policy.
        std::shared_ptr<GraphSearch> graphSearch_; ///< Graph search adaptor.
        std::shared_ptr<Skeleton> skeleton_;       ///< Skeleton generator.
        std::shared_ptr<Workspace> workspace_;     ///< Workspace (valid region).
        std::optional<GlobalGraph> globalGraph_;   ///< Cached global skeleton (built lazily).
        ConstraintSet constraints_;                ///< Active constraints (hard + soft).
    };

} // namespace arcgen::planning::engine
