#pragma once
/**
 * @file
 * @brief Collision feasibility constraint against a polygonal workspace.
 *  Discretises the path and verifies all states lie inside.
 */

#include <arcgen/planner/constraints/constraints.hpp>
#include <arcgen/planner/geometry/workspace.hpp>

#include <memory>
#include <utility>

namespace arcgen::planner::constraints
{
    using arcgen::steering::Path;

    /**
     * @brief Hard constraint that rejects paths colliding with the workspace boundary/obstacles.
     * @tparam N Maximum number of segments in the steering pattern (Path<N>).
     */
    template <std::size_t N> class CollisionConstraint final : public HardConstraint<N>
    {
      public:
        /**
         * @brief Construct with a workspace to test against.
         * @param workspace Shared pointer to a valid-region workspace
         *                  (may be null; a null or empty workspace accepts all paths).
         */
        explicit CollisionConstraint (std::shared_ptr<const arcgen::planner::geometry::Workspace> workspace) : workspace_ (std::move (workspace)) {}

        /**
         * @brief Check whether a candidate path is collision free.
         * @param cand Candidate path to test.
         * @param ctx  Evaluation context (provides lazy state generation).
         * @return True if the path is fully inside the valid region; false otherwise.
         */
        bool accept (const Path<N> &cand, const EvalContext<N> &ctx) const noexcept override
        {
            if (!workspace_ || workspace_->empty ())
                return true;

            ctx.ensureStates (cand);
            return cand.states && workspace_->contains (*cand.states);
        }

      private:
        std::shared_ptr<const arcgen::planner::geometry::Workspace> workspace_; ///< Workspace used for collision queries.
    };

} // namespace arcgen::planner::constraints
