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

            ctx.ensureStates (const_cast<Path<N> &> (cand));
            return cand.states && coveredByStraightHull (*workspace_, robot_, *cand.states, curvatureTol_);
        }

      private:
        std::shared_ptr<const Workspace> workspace_; ///< Workspace used for collision queries.
        Robot robot_;                                ///< Polygonal robot footprint.
        double curvatureTol_{};                      ///< Curvature tolerance for straight-run detection.

        /**
         * @brief Curvature-only straight-run accelerator using convex hull of endpoints.
         * @param workspace     Workspace for coverage queries.
         * @param robot         Robot polygon footprint (body frame).
         * @param states        Discretized states along the candidate path.
         * @param curvatureTol  Threshold to classify near-zero curvature.
         * @return True if the entire sweep is covered by the workspace; false otherwise.
         */
        static inline bool coveredByStraightHull (const Workspace &workspace, const Robot &robot, const std::vector<State> &states, double curvatureTol)
        {
            if (states.empty ())
                return true;

            const std::size_t n = states.size ();
            std::size_t i = 0;
            while (i < n)
            {
                const bool straight0 = std::fabs (states[i].curvature) <= curvatureTol;

                if (straight0)
                {
                    std::size_t j = i;
                    while (j + 1 < n && std::fabs (states[j + 1].curvature) <= curvatureTol)
                        ++j;

                    if (j > i)
                    {
                        const Polygon p0 = robot.at (states[i]);
                        const Polygon p1 = robot.at (states[j]);

                        bg::model::multi_point<Point> cloud;
                        cloud.insert (cloud.end (), p0.outer ().begin (), p0.outer ().end ());
                        cloud.insert (cloud.end (), p1.outer ().begin (), p1.outer ().end ());

                        Polygon hull;
                        bg::convex_hull (cloud, hull);
                        bg::correct (hull);

                        if (workspace.coveredBy (hull))
                        {
                            i = j + 1; // accept entire straight run
                            continue;
                        }
                        // else fall through to per-state checks in [i, j]
                        for (std::size_t k = i; k <= j; ++k)
                        {
                            const Polygon placed = robot.at (states[k]);
                            if (!workspace.coveredBy (placed))
                                return false;
                        }
                        i = j + 1;
                        continue;
                    }
                    // single straight sample
                    if (!workspace.coveredBy (robot.at (states[i])))
                        return false;
                    ++i;
                    continue;
                }

                // curved sample – check per-state
                if (!workspace.coveredBy (robot.at (states[i])))
                    return false;
                ++i;
            }
            return true;
        }
    };

} // namespace arcgen::planner::constraints
