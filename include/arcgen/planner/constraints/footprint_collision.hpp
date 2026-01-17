#pragma once
/**
 * @file
 * @brief Footprint-based collision feasibility constraint using a polygonal Robot.
 *
 * Uses a curvature-aware accelerator to skip redundant checks on straight runs:
 * consecutive states with near-zero curvature are accepted as a block if the
 * convex hull of the endpoint footprints lies within the workspace region.
 * Falls back to per-state footprint checks otherwise.
 */

#include <arcgen/core/math.hpp>
#include <arcgen/core/numeric.hpp>
#include <arcgen/planner/constraints/constraints.hpp>
#include <arcgen/planner/geometry/robot.hpp>
#include <arcgen/planner/geometry/workspace.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

namespace arcgen::planner::constraints
{
    namespace bg = boost::geometry;
    using arcgen::core::State;
    using arcgen::planner::geometry::BBox;
    using arcgen::planner::geometry::Point;
    using arcgen::planner::geometry::Polygon;
    using arcgen::planner::geometry::Robot;
    using arcgen::planner::geometry::Workspace;
    using arcgen::steering::Path;

    /**
     * @brief Hard constraint that rejects paths whose polygonal robot footprint
     *        leaves the workspace valid region.
     * @tparam N Maximum number of segments in the steering pattern (Path<N>).
     */
    template <std::size_t N> class FootprintCollisionConstraint final : public HardConstraint<N>
    {
      public:
        /**
         * @brief Construct with a workspace and a polygonal robot.
         * @param workspace Shared pointer to a valid-region workspace
         *                  (may be null; a null or empty workspace accepts all paths).
         * @param robot     Robot footprint in body frame (origin at reference, +X forward).
         * @param curvatureTol Curvature tolerance to classify a straight run.
         */
        explicit FootprintCollisionConstraint (std::shared_ptr<const Workspace> workspace, Robot robot, double curvatureTol = arcgen::core::CURVATURE_TOL)
            : workspace_ (std::move (workspace)), robot_ (std::move (robot)), curvatureTol_ (curvatureTol)
        {
        }

        /**
         * @brief Check whether a candidate path is collision free for the robot footprint.
         * @param cand Candidate path to test.
         * @param ctx  Evaluation context (provides lazy state generation).
         * @return True if the swept footprint is fully inside the valid region; false otherwise.
         */
        bool accept (const Path<N> &cand, const EvalContext<N> &ctx) const noexcept override
        {
            if (!workspace_ || workspace_->empty ())
                return true;

            ctx.ensureStates (cand);
            return cand.states && checkPath (*workspace_, robot_, *cand.states, curvatureTol_);
        }

      private:
        std::shared_ptr<const Workspace> workspace_; ///< Workspace used for collision queries.
        Robot robot_;                                ///< Polygonal robot footprint.
        double curvatureTol_{};                      ///< Curvature tolerance for straight-run detection.

        /**
         * @brief Check if a state is effectively straight (curvature near zero).
         */
        static bool isStraight (const State &s, double tol) { return std::fabs (s.curvature) <= tol; }

        /**
         * @brief Check if a single state is safe.
         */
        static bool checkState (const Workspace &ws, const Robot &robot, const State &s) { return ws.coveredBy (robot.at (s)); }

        /**
         * @brief Check a range of states individually.
         */
        static bool checkRange (const Workspace &ws, const Robot &robot, const std::vector<State> &states, std::size_t start, std::size_t end)
        {
            for (std::size_t k = start; k <= end; ++k)
            {
                if (!checkState (ws, robot, states[k]))
                    return false;
            }
            return true;
        }

        /**
         * @brief Finds the end index of a straight run starting at 'start'.
         * @return Index of the last straight state in the consecutive run.
         */
        static std::size_t findStraightEnd (const std::vector<State> &states, std::size_t start, double tol)
        {
            std::size_t end = start;
            while (end + 1 < states.size () && isStraight (states[end + 1], tol))
            {
                end++;
            }
            return end;
        }

        /**
         * @brief Checks if the convex hull of the footprints at start and end of a straight run is safe.
         */
        static bool checkStraightHull (const Workspace &ws, const Robot &robot, const State &sStart, const State &sEnd)
        {
            const Polygon p0 = robot.at (sStart);
            const Polygon p1 = robot.at (sEnd);

            bg::model::multi_point<Point> cloud;
            // Reserve approximate size: outer rings usually small
            cloud.reserve (p0.outer ().size () + p1.outer ().size ());
            cloud.insert (cloud.end (), p0.outer ().begin (), p0.outer ().end ());
            cloud.insert (cloud.end (), p1.outer ().begin (), p1.outer ().end ());

            Polygon hull;
            bg::convex_hull (cloud, hull);
            bg::correct (hull);

            return ws.coveredBy (hull);
        }

        /**
         * @brief Validates the entire path against the workspace obstacles.
         *
         * Uses a curvature-aware accelerator to skip redundant checks on straight runs.
         */
        static inline bool checkPath (const Workspace &workspace, const Robot &robot, const std::vector<State> &states, double curvatureTol)
        {
            if (states.empty ())
                return true;

            const std::size_t n = states.size ();
            for (std::size_t i = 0; i < n;)
            {
                // 1. Curved state? Check single and move on.
                if (!isStraight (states[i], curvatureTol))
                {
                    if (!checkState (workspace, robot, states[i]))
                        return false;
                    ++i;
                    continue;
                }

                // 2. Straight state - see how far it goes.
                std::size_t j = findStraightEnd (states, i, curvatureTol);

                // If only one point, treat as single check
                if (j == i)
                {
                    if (!checkState (workspace, robot, states[i]))
                        return false;
                    ++i;
                    continue;
                }

                // 3. Optimization: check hull of start/end of straight run
                if (checkStraightHull (workspace, robot, states[i], states[j]))
                {
                    i = j + 1; // Success: skip to after the run
                    continue;
                }

                // 4. Fallback: check every state in the run
                if (!checkRange (workspace, robot, states, i, j))
                    return false;

                i = j + 1;
            }
            return true;
        }
    };

} // namespace arcgen::planner::constraints
