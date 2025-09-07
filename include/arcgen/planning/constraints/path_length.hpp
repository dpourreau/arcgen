#pragma once
/**
 * @file
 * @brief Soft constraint that penalizes candidate paths by their total length.
 */

#include <arcgen/planning/constraints/constraints.hpp>

namespace arcgen::planning::constraints
{
    using arcgen::steering::Path;

    /**
     * @brief Soft constraint returning the geometric length of a candidate path.
     *
     * This encourages shorter paths. It does not enforce feasibility; combine
     * with hard constraints (e.g., collision) to ensure validity.
     *
     * @tparam N Maximum number of segments in the steering pattern.
     */
    template <std::size_t N> class PathLengthConstraint final : public SoftConstraint<N>
    {
      public:
        /**
         * @brief Compute the cost of a candidate as its total length.
         * @param cand Candidate path produced by the steering law.
         * @param ctx  Evaluation context (unused).
         * @return Path length in meters.
         */
        double cost (const Path<N> &cand, const EvalContext<N> & /*ctx*/) const noexcept override { return cand.length (); }
    };

} // namespace arcgen::planning::constraints
