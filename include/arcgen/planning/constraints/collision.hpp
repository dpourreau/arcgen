#pragma once
/**
 * @file
 * @brief Collision feasibility constraint against a polygonal workspace.
 *  Discretises the path and verifies all states lie inside.
 */

#include <arcgen/geometry/workspace.hpp>
#include <arcgen/planning/constraints/constraints.hpp>

#include <memory>
#include <utility>

namespace arcgen::planning::constraints
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
        explicit CollisionConstraint (std::shared_ptr<const arcgen::geometry::Workspace> workspace) : workspace_ (std::move (workspace)) {}

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

            // Path::states is mutable; we cast only to allow filling it via ctx.ensureStates without changing semantics.
            ctx.ensureStates (const_cast<Path<N> &> (cand));
            return cand.states && workspace_->contains (*cand.states);
        }

      private:
        std::shared_ptr<const arcgen::geometry::Workspace> workspace_; ///< Workspace used for collision queries.
    };

} // namespace arcgen::planning::constraints
