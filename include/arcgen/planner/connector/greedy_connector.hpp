#pragma once
/**
 * @file greedy_connector.hpp
 * @brief Greedy farthest-reachable connector (default stitching strategy).
 */

#include <arcgen/planner/engine/evaluator.hpp>

#include <arcgen/core/math.hpp>
#include <arcgen/core/numeric.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <optional>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace arcgen::planner::connector
{
    using arcgen::planner::engine::Evaluator;

    /**
     * @brief Greedy connector that assigns headings to coarse waypoints and stitches them with farthest-reachable jumps.
     */
    template <class Steering, class DebugInfo = void> class GreedyConnector final
    {
        /// Counter for generating unique steering IDs within this connector instance.
        mutable int64_t nextSteeringId_{0};
        double resampleInterval_;
        unsigned int maxIterations_;
        unsigned int lookaheadMatches_;
        double minResampleInterval_;
        double costImprovementTol_;

      public:
        /**
         * @brief Constructs a GreedyConnector with configurable parameters.
         *
         * @param resampleInterval     The interval (in meters) at which the path is resampled for smoothing.
         *                             Must be greater than minResampleInterval. Default is 3.0.
         * @param maxIterations        The maximum number of smoothing iterations to perform.
         *                             Default is 3.
         * @param lookaheadMatches     The number of subsequent segments to search for greedy shortcuts.
         *                             Must be at least 1. Default is 3.
         * @param minResampleInterval  Minimum allowed resample interval (default: core::GREEDY_MIN_RESAMPLE_INTERVAL).
         * @param costImprovementTol   Minimum cost improvement to accept a shortcut (default: core::GREEDY_COST_IMPROVEMENT_TOL).
         * @throws std::invalid_argument if resampleInterval < minResampleInterval or lookaheadMatches < 1.
         */
        explicit GreedyConnector (double resampleInterval = 3, unsigned int maxIterations = 3, unsigned int lookaheadMatches = 3,
                                  double minResampleInterval = arcgen::core::GREEDY_MIN_RESAMPLE_INTERVAL, double costImprovementTol = arcgen::core::GREEDY_COST_IMPROVEMENT_TOL)
            : resampleInterval_ (resampleInterval), maxIterations_ (maxIterations), lookaheadMatches_ (lookaheadMatches), minResampleInterval_ (minResampleInterval),
              costImprovementTol_ (costImprovementTol)
        {
            if (resampleInterval_ < minResampleInterval_)
                throw std::invalid_argument ("GreedyConnector: resampleInterval must be >= minResampleInterval");
            // maxIterations is unsigned, so >= 0 is always true.
            if (lookaheadMatches_ < 1)
                throw std::invalid_argument ("GreedyConnector: lookaheadMatches must be >= 1");
        }

        /// @brief Get the resample interval.
        [[nodiscard]] double getResampleInterval () const noexcept { return resampleInterval_; }
        /// @brief Get the maximum number of smoothing iterations.
        [[nodiscard]] unsigned int getMaxIterations () const noexcept { return maxIterations_; }
        /// @brief Get the lookahead depth for greedy shortcuts.
        [[nodiscard]] unsigned int getLookaheadMatches () const noexcept { return lookaheadMatches_; }
        /// @brief Get the minimum allowed resample interval.
        [[nodiscard]] double getMinResampleInterval () const noexcept { return minResampleInterval_; }
        /// @brief Get the cost improvement tolerance.
        [[nodiscard]] double getCostImprovementTol () const noexcept { return costImprovementTol_; }

        /**
         * @brief Connects start and goal states via coarse waypoints, optimizing with a greedy strategy.
         * @param evaluator  Steering evaluator instance.
         * @param start      Start state.
         * @param goal       Goal state.
         * @param coarse     List of coarse waypoints.
         * @param dbg        Optional debug info pointer.
         * @return Dense path of states.
         */
        [[nodiscard]] std::vector<arcgen::core::State> connect (const Evaluator<Steering> &evaluator, const arcgen::core::State &start, const arcgen::core::State &goal,
                                                                std::vector<arcgen::core::State> coarse, DebugInfo *dbg = nullptr) const
        {
            auto startTotal = std::chrono::steady_clock::now ();

            assignHeadings (coarse);

            std::vector<arcgen::core::State> waypoints;
            waypoints.reserve (coarse.size () + 2);
            waypoints.push_back (start);
            waypoints.insert (waypoints.end (), coarse.begin (), coarse.end ());
            waypoints.push_back (goal);

            // Initial stitch
            auto t0 = std::chrono::steady_clock::now ();
            StitchResult currentResult = stitch (waypoints, evaluator, dbg);
            auto t1 = std::chrono::steady_clock::now ();

            if (currentResult.path.empty ())
                return {};

            logInitialStitch (dbg, currentResult, waypoints, evaluator, std::chrono::duration<double> (t1 - t0).count ());

            // Smoothing loop
            double currentScore = currentResult.totalScore;
            std::vector<arcgen::core::State> currentPath = std::move (currentResult.path);
            std::vector<int64_t> currentIds = std::move (currentResult.segmentIds);
            std::unordered_map<int64_t, double> currentCosts = std::move (currentResult.segmentCosts);

            // Time smoothing
            auto tSmoothingStart = std::chrono::steady_clock::now ();
            int successfulIterations = 0;

            for (unsigned int iter = 0; iter < maxIterations_; ++iter)
            {
                auto tIter0 = std::chrono::steady_clock::now ();
                auto indices = resampleIndices (currentPath, currentIds);

                auto newResult = stitchLocal (currentPath, currentIds, indices, evaluator, currentCosts, nullptr);
                auto tIter1 = std::chrono::steady_clock::now ();

                if (newResult.path.empty ())
                {
                    // Even if empty (failed), we might want to log? usually stitchLocal returns empty only if totally failed which shouldn't happen if path is valid.
                    // But if it returns empty we continue.
                    continue;
                }

                if (newResult.totalScore < currentScore - costImprovementTol_)
                {
                    logSmoothingStep (dbg, iter, newResult, indices, currentPath, evaluator, std::chrono::duration<double> (tIter1 - tIter0).count ());

                    currentScore = newResult.totalScore;
                    currentPath = std::move (newResult.path);
                    currentCosts = std::move (newResult.segmentCosts);
                    currentIds = std::move (newResult.segmentIds);
                    successfulIterations++;
                }
                else
                {
                    break;
                }
            }

            auto endTotal = std::chrono::steady_clock::now ();
            if constexpr (!std::is_void_v<DebugInfo>)
            {
                if (dbg)
                {
                    dbg->componentTimes["Smoothing"] = std::chrono::duration<double> (endTotal - tSmoothingStart).count ();
                    dbg->componentTimes["Total Connection"] = std::chrono::duration<double> (endTotal - startTotal).count ();
                    dbg->smoothingIterations = successfulIterations;
                }
            }

            return currentPath;
        }

      private:
        /**
         * @brief Helper to record a debug step in the history.
         * @details
         * Populates a `DebugInfo::Step` with path data, resampling visualization points,
         * fixed anchors (joints/endpoints), and detailed cost breakdowns. This method
         * calculates soft constraint costs on the fly if needed for reporting.
         *
         * @param step      The debug step structure to populate.
         * @param evaluator The evaluator used to score the path.
         */
        template <typename StepT> void computeStepStats (StepT &step, const Evaluator<Steering> &evaluator) const
        {
            const auto *cset = evaluator.getConstraints ();
            if (!cset || step.path.empty ())
                return;

            using PathT = typename Steering::PathType;
            PathT p;
            p.states = step.path;
            // Empty callback is safe here as we only score, not generate
            typename Evaluator<Steering>::EvalContext ctx{step.path.front (), step.path.back (), [] (const PathT &) { /* No-op */ }};
            step.stats.totalCost = cset->score (p, ctx);

            for (const auto &[c, w] : cset->soft)
            {
                double cost = c->cost (p, ctx);
                if (std::isfinite (cost))
                {
                    step.stats.softConstraints[c->name ()] = cost * w;
                }
            }
        }

        /**
         * @brief Helper to record a debug step in the history.
         * @details
         * Populates a `DebugInfo::Step` with path data, resampling visualization points,
         * fixed anchors (joints/endpoints), and detailed cost breakdowns. This method
         * calculates soft constraint costs on the fly if needed for reporting.
         *
         * @param dbg             Pointer to the debug info structure (can be null).
         * @param name            Name of the step (e.g., "Initial Stitch", "Smoothing Iteration 1").
         * @param path            The dense sequence of states for this step.
         * @param resampledPoints A set of points derived from resampling (for visualization).
         * @param fixedAnchors    Key waypoints that were fixed during this step.
         * @param evaluator       The evaluator used to score the path.
         * @param computationTime Time taken for this step in seconds.
         */
        void recordStep (DebugInfo *dbg, const std::string &name, const std::vector<arcgen::core::State> &path, const std::vector<arcgen::core::State> &resampledPoints,
                         const std::vector<arcgen::core::State> &fixedAnchors, const Evaluator<Steering> &evaluator, double computationTime) const
        {
            if constexpr (!std::is_void_v<DebugInfo>)
            {
                if (!dbg)
                    return;

                typename DebugInfo::Step step;
                step.name = name;
                step.path = path;
                step.resampledPoints = resampledPoints;
                step.fixedAnchors = fixedAnchors;

                computeStepStats (step, evaluator);

                step.computationTime = computationTime;
                dbg->history.push_back (std::move (step));
            }
        }
        /// @brief Assigns headings to waypoints based on the vector to the next point.
        void assignHeadings (std::vector<arcgen::core::State> &pts) const
        {
            const std::size_t n = pts.size ();
            if (n < 2)
                return;

            for (std::size_t i = 0; i + 1 < n; ++i)
            {
                const double dx = pts[i + 1].x - pts[i].x;
                const double dy = pts[i + 1].y - pts[i].y;

                const auto [r, theta] = arcgen::core::toPolar (dx, dy);

                if (r < arcgen::core::CURVATURE_TOL)
                {
                    if (i > 0)
                        pts[i].heading = pts[i - 1].heading;
                    continue;
                }

                pts[i].heading = arcgen::core::normalizeAngleSigned (theta);
            }

            pts.back ().heading = pts[n - 2].heading;
        }

        /**
         * @brief Selects indices from the path at a fixed distance interval and at joint transitions.
         *
         * This resampling ensures that the smoothing process has a well-distributed set of
         * pivot points to work with. It explicitly preserves joint points (where steering IDs change)
         * to avoid breaking the path topology unintentionally.
         *
         * @param path The dense path of states to resample.
         * @param ids  Segment IDs corresponding to the path states.
         * @return     Vector of indices into the path, including start, end, and joints.
         */
        std::vector<std::size_t> resampleIndices (const std::vector<arcgen::core::State> &path, const std::vector<int64_t> &ids) const
        {
            std::vector<std::size_t> indices;
            if (path.empty () || path.size () < 3)
                return indices;

            indices.push_back (0);

            double accumulated = 0.0;
            for (std::size_t i = 0; i + 1 < path.size (); ++i)
            {
                const auto &p0 = path[i];
                const auto &p1 = path[i + 1];

                // Joint check
                bool isJoint = (ids[i + 1] != -1 && ids[i] != -1 && ids[i + 1] != ids[i]);
                double d = std::hypot (p1.x - p0.x, p1.y - p0.y);

                accumulated += d;

                if (isJoint || accumulated >= resampleInterval_)
                {
                    if (indices.back () != i + 1)
                        indices.push_back (i + 1);
                    accumulated = 0.0;
                }
            }

            // Ensure last
            if (indices.back () != path.size () - 1)
            {
                indices.push_back (path.size () - 1);
            }

            return indices;
        }

        struct StitchResult
        {
            std::vector<arcgen::core::State> path;
            std::vector<int64_t> segmentIds;
            double totalScore{std::numeric_limits<double>::infinity ()};
            std::unordered_map<int64_t, double> segmentCosts;
            std::vector<std::pair<std::size_t, std::size_t>> connections;
        };

        /**
         * @brief Logs the initial stitch step to the debug info.
         */
        void logInitialStitch (DebugInfo *dbg, const StitchResult &result, const std::vector<arcgen::core::State> &waypoints, const Evaluator<Steering> &evaluator,
                               double duration) const
        {
            if constexpr (std::is_void_v<DebugInfo>)
                return;

            if (!dbg)
                return;

            // Resampled Points: "input shortest path... after assignHeadings" -> waypoints
            // Fixed Anchors: "All points... selected for stitching" -> derived from connections
            std::vector<std::size_t> fixedIdx;
            fixedIdx.reserve (result.connections.size () * 2);
            for (auto [u, v] : result.connections)
            {
                fixedIdx.push_back (u);
                fixedIdx.push_back (v);
            }
            std::ranges::sort (fixedIdx);
            auto ret = std::ranges::unique (fixedIdx);
            fixedIdx.erase (ret.begin (), ret.end ());

            std::vector<arcgen::core::State> anchors;
            anchors.reserve (fixedIdx.size ());
            for (auto idx : fixedIdx)
            {
                if (idx < waypoints.size ())
                    anchors.push_back (waypoints[idx]);
            }

            recordStep (dbg, "Initial Stitch", result.path, waypoints, anchors, evaluator, duration);
        }

        /**
         * @brief Logs a smoothing iteration step to the debug info.
         */
        void logSmoothingStep (DebugInfo *dbg, unsigned int iter, const StitchResult &newResult, const std::vector<std::size_t> &indices,
                               const std::vector<arcgen::core::State> &currentPath, const Evaluator<Steering> &evaluator, double duration) const
        {
            if constexpr (std::is_void_v<DebugInfo>)
                return;

            if (!dbg)
                return;

            // Resampled Points: "All other points in the smoothing set (the indices output)"
            std::vector<arcgen::core::State> resampled;
            resampled.reserve (indices.size ());
            for (auto idx : indices)
                resampled.push_back (currentPath[idx]);

            // Fixed Anchors: "start, end, and all joint points used"
            std::vector<arcgen::core::State> anchors;
            if (!newResult.path.empty ())
            {
                anchors.push_back (newResult.path.front ());
                for (size_t i = 0; i + 1 < newResult.path.size (); ++i)
                {
                    if (newResult.segmentIds[i] != newResult.segmentIds[i + 1])
                    {
                        anchors.push_back (newResult.path[i]);
                        anchors.push_back (newResult.path[i + 1]);
                    }
                }
                anchors.push_back (newResult.path.back ());
            }

            recordStep (dbg, std::format ("Smoothing Iteration {}", iter + 1), newResult.path, resampled, anchors, evaluator, duration);
        }

        struct DFSContext
        {
            std::vector<bool> failedConnections;
            std::vector<bool> deadNodes;
            std::vector<std::vector<arcgen::core::State>> &pathStack;
            std::vector<double> &scores;
            std::vector<std::pair<std::size_t, std::size_t>> &stitched;

            DFSContext (std::size_t count, std::vector<std::vector<arcgen::core::State>> &ps, std::vector<double> &sc, std::vector<std::pair<std::size_t, std::size_t>> &st)
                : failedConnections (count * count, false), deadNodes (count, false), pathStack (ps), scores (sc), stitched (st)
            {
            }
        };

        bool dfsConnect (std::size_t curr, const std::vector<arcgen::core::State> &nodes, const Evaluator<Steering> &evaluator, DFSContext &ctx) const
        {
            const std::size_t count = nodes.size ();
            if (curr == count - 1)
                return true;

            if (ctx.deadNodes[curr])
                return false;

            // Try the farthest reachable node first, walking backward.
            for (std::size_t next = curr == 0 ? count - 2 : count - 1; next > curr; --next)
            {
                if (ctx.failedConnections[curr * count + next] || ctx.deadNodes[next])
                    continue;

                if (auto best = evaluator.bestStatesBetween (nodes[curr], nodes[next]))
                {
                    ctx.pathStack.push_back (std::move (best->first));
                    ctx.scores.push_back (best->second);
                    ctx.stitched.emplace_back (curr, next);

                    if (dfsConnect (next, nodes, evaluator, ctx))
                        return true;

                    // Backtrack
                    ctx.pathStack.pop_back ();
                    ctx.scores.pop_back ();
                    ctx.stitched.pop_back ();
                }
                else
                {
                    ctx.failedConnections[curr * count + next] = true;
                }
            }

            ctx.deadNodes[curr] = true;
            return false;
        }

        StitchResult buildStitchResult (std::vector<std::vector<arcgen::core::State>> &pathStack, const std::vector<double> &scores,
                                        const std::vector<std::pair<std::size_t, std::size_t>> &stitched) const
        {
            std::vector<arcgen::core::State> out;
            std::vector<int64_t> ids;
            std::unordered_map<int64_t, double> costs;
            double totalScore = 0.0;

            for (size_t i = 0; i < pathStack.size (); ++i)
            {
                auto &states = pathStack[i];
                double s = scores[i];
                int64_t newId = nextSteeringId_;
                nextSteeringId_++;

                if (!out.empty () && !states.empty ())
                {
                    // Avoid duplicating junction point
                    for (size_t k = 1; k < states.size (); ++k)
                    {
                        out.push_back (std::move (states[k]));
                        ids.push_back (newId);
                    }
                }
                else
                {
                    for (auto &state : states)
                    {
                        out.push_back (std::move (state));
                        ids.push_back (newId);
                    }
                }

                costs[newId] = s;
                totalScore += s;
            }

            return {out, ids, totalScore, costs, stitched};
        }

        /**
         * @brief Stitches waypoints together using the evaluator to find the farthest valid connections.
         *
         * Implements a greedy "farthest-reachable" strategy via Depth-First Search (DFS).
         * From the current node, it attempts to connect to the farthest possible subsequent node
         * in the list. If a connection is valid (checked by `evaluator`), it greedily takes it
         * and recurses. If the path leads to a dead end, it backtracks.
         *
         * @param nodes     The sequence of coarse waypoints to connect.
         * @param evaluator Evaluator used to generate and score steering candidates.
         * @param dbg       Debug info pointer (unused in this method).
         * @return          A `StitchResult` containing the dense path, steering IDs, total score,
         *                  and the list of successful connections.
         */
        [[nodiscard]] StitchResult stitch (const std::vector<arcgen::core::State> &nodes, const Evaluator<Steering> &evaluator, DebugInfo * /*dbg*/) const
        {
            const std::size_t count = nodes.size ();
            if (count < 3)
                return {{}, {}, std::numeric_limits<double>::infinity (), {}, {}};

            std::vector<std::vector<arcgen::core::State>> pathStack;
            std::vector<double> scores;
            pathStack.reserve (count);
            scores.reserve (count);
            std::vector<std::pair<std::size_t, std::size_t>> stitched;

            if (DFSContext ctx (count, pathStack, scores, stitched); dfsConnect (0, nodes, evaluator, ctx))
            {
                return buildStitchResult (pathStack, scores, stitched);
            }

            return {{}, {}, std::numeric_limits<double>::infinity (), {}, {}};
        }

        struct SegmentView
        {
            /// The unique ID of the steering segment.
            int64_t steeringId;
            /// Start index in the 'indices' vector.
            std::size_t startI;
            /// End index in the 'indices' vector (inclusive).
            std::size_t endI;
            /// Cached cost of this segment.
            double cost{0.0};
        };

        /**
         * @brief Pre-calculates segment views with cached costs for fast lookup.
         * @param indices The resample indices defining the segments.
         * @param ids     The segment IDs.
         * @param costMap Map of steering ID to cost.
         * @return Vector of SegmentView structs.
         */
        std::vector<SegmentView> buildSegments (const std::vector<std::size_t> &indices, const std::vector<int64_t> &ids, const std::unordered_map<int64_t, double> &costMap) const
        {
            std::vector<SegmentView> segments;
            if (indices.empty ())
                return segments;

            std::size_t segStart = 0;
            int64_t currentId = ids[indices[0]];
            for (std::size_t k = 1; k < indices.size (); ++k)
            {
                int64_t nextId = ids[indices[k]];
                if (nextId != currentId)
                {
                    if (currentId != -1)
                    {
                        double c = costMap.contains (currentId) ? costMap.at (currentId) : 0.0;
                        segments.push_back ({currentId, segStart, k - 1, c});
                    }

                    segStart = k;
                    currentId = nextId;
                }
            }
            if (currentId != -1)
            {
                double c = costMap.contains (currentId) ? costMap.at (currentId) : 0.0;
                segments.push_back ({currentId, segStart, indices.size () - 1, c});
            }

            return segments;
        }

        /**
         * @brief Returns a zero-copy span of the path between start and end indices.
         * @param path  The source path vector.
         * @param start Start index.
         * @param end   End index (inclusive).
         * @return std::span over the requested range.
         */
        [[nodiscard]] std::span<const arcgen::core::State> slicePath (const std::vector<arcgen::core::State> &path, std::size_t start, std::size_t end) const
        {
            if (start > end || start >= path.size () || end >= path.size ())
                return {};
            // end is inclusive
            return {path.data () + start, end - start + 1};
        }

        struct ShortcutResult
        {
            std::size_t targetPathIdx;
            double totalCost;
            int64_t newSteeringId;
            std::vector<arcgen::core::State> shortcutPath; ///< Dense states of the shortcut
            double costRetainA{0.0};                       ///< Cost of the retained part of segment A
            double costRetainB{0.0};                       ///< Cost of the retained part of segment B
            double costShortcut{0.0};                      ///< Cost of the new shortcut segment
            bool isStartA{false};                          ///< True if shortcut starts at the beginning of segment A
            bool isEndB{false};                            ///< True if shortcut ends at the end of segment B
        };

        /**
         * @brief Context struct for immutable data shared across stitching operations.
         *
         * Reduces parameter counts for internal helper methods.
         */
        struct StitchContext
        {
            const std::vector<arcgen::core::State> &path;
            const std::vector<int64_t> &ids;
            const std::vector<std::size_t> &indices;
            const std::vector<SegmentView> &segments;
            const Evaluator<Steering> &evaluator;
        };

        /**
         * @brief Computes the score for a potential shortcut between two points.
         *
         * Evaluates the cost of retaining parts of the existing path versus taking a direct
         * connection (shortcut).
         *
         * @param ctx             Immutable context data.
         * @param i               Start index in the resampled path.
         * @param targetPathIdx   Target index in the original path.
         * @param effectiveStartA Effective start index of the current segment.
         * @return                A `ShortcutResult` if the shortcut is valid, otherwise `std::nullopt`.
         */
        std::optional<ShortcutResult> computeShortcutScore (const StitchContext &ctx, std::size_t i, std::size_t targetPathIdx, std::size_t effectiveStartA) const
        {
            const auto &curr = ctx.path[ctx.indices[i]];
            const auto &cand = ctx.path[targetPathIdx];

            // Calculate costs of retained segments purely by evaluation
            double costRetainA = 0.0;
            bool isStartA = (ctx.indices[i] == effectiveStartA);
            if (!isStartA)
            {
                costRetainA = ctx.evaluator.evaluateCost (slicePath (ctx.path, effectiveStartA, ctx.indices[i]));
            }

            // Find true end of segment B in path
            std::size_t endB = targetPathIdx;
            while (endB + 1 < ctx.path.size () && ctx.ids[endB + 1] == ctx.ids[targetPathIdx])
                endB++;

            double costRetainB = 0.0;
            bool isEndB = (targetPathIdx == endB);
            if (!isEndB)
            {
                costRetainB = ctx.evaluator.evaluateCost (slicePath (ctx.path, targetPathIdx, endB));
            }

            // Compute shortcut cost
            auto best = ctx.evaluator.bestStatesBetween (curr, cand);
            if (!best || best->first.empty ())
                return std::nullopt;

            const double shortcutCost = best->second;
            return ShortcutResult{targetPathIdx,
                                  0.0, // Calculated by caller
                                  -1,  // Assigned by caller
                                  std::move (best->first),
                                  costRetainA,
                                  costRetainB,
                                  shortcutCost,
                                  isStartA,
                                  isEndB};
        }

        /**
         * @brief Attempts to find a beneficial shortcut starting from the current point `i`.
         *
         * Searches forward in the resampled path using `lookaheadMatches_` to check if a direct
         * connection exists between the current state and the candidate state.
         *
         * @param ctx               Immutable context data (path, ids, indices, segments, evaluator).
         * @param i                 Current index in `indices` (start of shortcut).
         * @param currentSegIdx     Index of the segment containing `indices[i]`.
         * @param getEffectiveStart Callback to resolve the effective start index of a segment.
         * @return                  A `ShortcutResult` if a beneficial shortcut is found, otherwise `std::nullopt`.
         */
        template <typename StartCallback>
        [[nodiscard]] std::optional<ShortcutResult> findShortcut (const StitchContext &ctx, std::size_t i, std::size_t currentSegIdx, const StartCallback &getEffectiveStart) const
        {
            std::optional<ShortcutResult> bestRes;

            for (unsigned int offset = lookaheadMatches_; offset > 0; --offset)
            {
                std::size_t targetSegIdx = currentSegIdx + offset;
                if (targetSegIdx >= ctx.segments.size ())
                    continue;

                // Compute original cost to compare against using cached segment costs
                double costTotalA = ctx.segments[currentSegIdx].cost;
                double costTotalB = ctx.segments[targetSegIdx].cost;
                double costInter = 0.0;
                for (std::size_t s = currentSegIdx + 1; s < targetSegIdx; ++s)
                    costInter += ctx.segments[s].cost;

                double oldTotalLocal = costTotalA + costInter + costTotalB;
                const auto &targetSeg = ctx.segments[targetSegIdx];

                for (std::size_t k = targetSeg.endI; k >= targetSeg.startI; --k)
                {
                    if (k <= i + 1)
                        break;

                    std::size_t targetPathIdx = ctx.indices[k];
                    std::size_t effectiveStartA = getEffectiveStart (ctx.ids[ctx.indices[i]], ctx.indices[ctx.segments[currentSegIdx].startI]);

                    auto res = computeShortcutScore (ctx, i, targetPathIdx, effectiveStartA);
                    if (!res)
                        continue;

                    double newTotalLocal = res->costRetainA + res->costShortcut + res->costRetainB;
                    if (newTotalLocal >= oldTotalLocal - costImprovementTol_)
                        continue;

                    res->totalCost = newTotalLocal;
                    bestRes = std::move (res);
                    return bestRes; // Greedy return found
                }
            }

            return bestRes;
        }

        /**
         * @brief Holds the ephemeral state during a local stitching optimization pass.
         *
         * Aggregates the evolving path, costs, and connection history as the algorithm
         * iterates through potential shortcuts.
         */
        struct StitchSession
        {
            /// The accumulated dense path of states.
            std::vector<arcgen::core::State> finalPath;
            /// Corresponding steering IDs for each state in the path.
            std::vector<int64_t> finalIds;
            /// Accumulated total cost of the stitched path so far.
            double totalCost{0.0};
            /// Map of steering block IDs to their calculated costs.
            std::unordered_map<int64_t, double> costMap;
            /// Tracks which steering blocks have been accounted for in the total cost to prevent double-counting.
            std::unordered_map<int64_t, bool> usedIds;
            /// Maps steering block IDs to their effective start indices in the original path, handling partial consumption.
            std::unordered_map<int64_t, std::size_t> effectiveStarts;
            /// Records valid shortcut connections (start index, end index) for visualization/debugging.
            std::vector<std::pair<std::size_t, std::size_t>> connections;
            /// Flag indicating if any shortcut was successfully applied during this session.
            bool anyShortcut{false};
        };

        /**
         * @brief Applies a validated shortcut to the stitching session.
         *
         * Updates the session's path, costs, and state maps by determining which parts of the
         * original segments to retain and where to insert the new shortcut.
         *
         * @param session       The current stitching session state.
         * @param ctx           Immutable context data.
         * @param res           The result of the found shortcut containing path and cost details.
         * @param shortcutId    The unique ID assigned to this new shortcut segment.
         * @param i             The current index in the resampled points list (start of shortcut).
         * @param iOut          [Output] Updated index pointer to skip processed points.
         */
        void applyShortcut (StitchSession &session, const StitchContext &ctx, ShortcutResult &res, int64_t shortcutId, std::size_t i, std::size_t &iOut) const
        {
            std::size_t currPathIdx = ctx.indices[i];
            int64_t currentPathId = ctx.ids[currPathIdx];
            int64_t targetPathId = ctx.ids[res.targetPathIdx];

            // A) Fix A Cost (Current Segment)
            if (!res.isStartA)
            {
                if (session.usedIds[currentPathId])
                    session.totalCost -= session.costMap[currentPathId];

                session.totalCost += res.costRetainA;
                session.costMap[currentPathId] = res.costRetainA;
                session.usedIds[currentPathId] = true;
            }

            // B) Shortcut Cost
            session.totalCost += res.costShortcut;
            session.costMap[shortcutId] = res.costShortcut;

            if (!res.isEndB)
            {
                session.costMap[targetPathId] = res.costRetainB;
                session.usedIds[targetPathId] = false; // Will be added by loop later
                session.effectiveStarts[targetPathId] = res.targetPathIdx;
            }
            else
            {
                session.usedIds[targetPathId] = false;
            }

            // Append shortcut states
            if (auto &states = res.shortcutPath; !session.finalPath.empty () && !states.empty ())
            {
                for (size_t p = 1; p < states.size (); ++p)
                {
                    session.finalPath.push_back (std::move (states[p]));
                    session.finalIds.push_back (shortcutId);
                }
            }
            else
            {
                for (auto s : states)
                {
                    session.finalPath.push_back (std::move (s));
                    session.finalIds.push_back (shortcutId);
                }
            }

            session.anyShortcut = true;

            // Advance 'i'
            for (size_t k = i + 1; k < ctx.indices.size (); ++k)
            {
                if (ctx.indices[k] == res.targetPathIdx)
                {
                    session.connections.emplace_back (i, k);
                    iOut = k;
                    break;
                }
            }
        }

        /**
         * @brief Appends a segment from the original path to the session without modification.
         *
         * Used when no shortcut is found. It copies states and IDs from the source path
         * and updates the total cost if the segment hasn't been counted yet.
         *
         * @param session The current stitching session state.
         * @param path    The original dense path.
         * @param ids     The original segment IDs.
         * @param from    Start index in the original path (inclusive).
         * @param to      End index in the original path (inclusive).
         */
        void keepSegment (StitchSession &session, const std::vector<arcgen::core::State> &path, const std::vector<int64_t> &ids, std::size_t from, std::size_t to) const
        {
            for (std::size_t k = from; k <= to; ++k)
            {
                const auto &p = path[k];
                session.finalPath.push_back (p);
                session.finalIds.push_back (ids[k]);

                if (ids[k] != -1 && !session.usedIds[ids[k]])
                {
                    if (auto it = session.costMap.find (ids[k]); it != session.costMap.end ())
                    {
                        session.totalCost += it->second;
                    }
                    session.usedIds[ids[k]] = true;
                }
            }
        }

        /**
         * @brief Optimizes the path locally by attempting to replace dense segments with direct shortcuts.
         *
         * Iterates through the resampled points (indices). For each point, it attempts to find a
         * `findShortcut` to a future point. If a shortcut is found that improves the total cost,
         * it stitches the path:
         *  1. Retained part of current segment (Start -> Shortcut Start)
         *  2. New Shortcut (Shortcut Start -> Shortcut End)
         *  3. Retained part of target segment (Shortcut End -> End)
         *
         * Existing segment IDs are preserved for retained parts, and a new ID is generated for the shortcut.
         * Cost maps are updated accordingly.
         *
         * @param path      The current dense path.
         * @param ids       Segment IDs for the path.
         * @param indices   Resampled points to consider for shortcuts.
         * @param evaluator Evaluator for scoring.
         * @param costMap   Current map of segment costs.
         * @param dbg       Debug info pointer (unused).
         * @return          A `StitchResult` with the potentially improved path and costs.
         */
        [[nodiscard]] StitchResult stitchLocal (const std::vector<arcgen::core::State> &path, const std::vector<int64_t> &ids, const std::vector<std::size_t> &indices,
                                                const Evaluator<Steering> &evaluator, std::unordered_map<int64_t, double> costMap, DebugInfo * /*dbg*/) const
        {
            if (indices.size () < 3)
                return {path, ids, 0.0, costMap, {}};

            // Pre-calculate segments from indices for optimized navigation
            std::vector<SegmentView> segments = buildSegments (indices, ids, costMap);
            StitchContext ctx{path, ids, indices, segments, evaluator};

            StitchSession session;
            session.finalPath.reserve (path.size ());
            session.finalIds.reserve (path.size ());
            session.costMap = std::move (costMap);

            auto getEffectiveStart = [&session] (int64_t id, std::size_t defaultPathIdx) -> std::size_t
            {
                if (auto it = session.effectiveStarts.find (id); it != session.effectiveStarts.end ())
                    return it->second;
                return defaultPathIdx;
            };

            // Start point
            session.finalPath.push_back (path[indices[0]]);
            session.finalIds.push_back (ids[indices[0]]);

            std::size_t currentSegIdx = 0;
            std::size_t i = 0; // Index in 'indices'

            while (i + 1 < indices.size ())
            {
                std::size_t currPathIdx = indices[i];

                // Sync currentSegIdx to current 'i'
                while (currentSegIdx < segments.size () && segments[currentSegIdx].endI < i)
                    currentSegIdx++;

                // If currently in a valid segment (not -1 gap)
                bool canShortcut = (currentSegIdx < segments.size () && ids[currPathIdx] == segments[currentSegIdx].steeringId);
                std::optional<ShortcutResult> shortcut;

                if (canShortcut)
                {
                    shortcut = findShortcut (ctx, i, currentSegIdx, getEffectiveStart);
                }

                if (shortcut)
                {
                    int64_t shortcutId = nextSteeringId_;
                    nextSteeringId_++;
                    applyShortcut (session, ctx, *shortcut, shortcutId, i, i);
                }
                else
                {
                    // Keep Segment
                    std::size_t nextIdx = indices[i + 1];
                    keepSegment (session, path, ids, currPathIdx + 1, nextIdx);
                    i++;
                }
            }

            if (!session.anyShortcut)
                return {path, ids, std::numeric_limits<double>::infinity (), session.costMap, {}};

            return {session.finalPath, session.finalIds, session.totalCost, session.costMap, std::move (session.connections)};
        }
    };

} // namespace arcgen::planner::connector
