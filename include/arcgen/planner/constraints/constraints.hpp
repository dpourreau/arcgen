#pragma once
/**
 * @file
 * @brief Constraint interfaces and utilities for candidate-path evaluation.
 *
 * This header defines the evaluation context passed to constraints, the
 * abstract interfaces for hard/soft constraints, and a small container that
 * evaluates feasibility and computes weighted scores.
 */

#include <arcgen/core/state.hpp>
#include <arcgen/steering/path.hpp>

#include <functional>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace arcgen::planner::constraints
{
    using arcgen::core::State;
    using arcgen::steering::Path;

    /**
     * @brief Context provided to constraints during evaluation.
     * @tparam N Maximum number of segments in the steering pattern (Path<N>).
     */
    template <std::size_t N> struct EvalContext
    {
        const State &from; ///< Start state of the connection being evaluated.
        const State &to;   ///< Goal state of the connection being evaluated.

        /**
         * @brief Ensure discretised states exist for a candidate path.
         *        (Filled lazily into Path<N>::states.)
         *
         * Implementations typically call the steering policy's integration.
         */
        std::function<void (const Path<N> &)> ensureStates;
    };

    /**
     * @brief Hard constraints must be satisfied (feasibility).
     * @tparam N Maximum number of segments in the steering pattern (Path<N>).
     */
    template <std::size_t N> struct HardConstraint
    {
        HardConstraint () = default;
        HardConstraint (const HardConstraint &) = default;
        HardConstraint (HardConstraint &&) = default;
        HardConstraint &operator= (const HardConstraint &) = default;
        HardConstraint &operator= (HardConstraint &&) = default;
        virtual ~HardConstraint () = default;

        /**
         * @brief Decide whether a candidate is feasible.
         * @param cand Candidate path to check.
         * @param ctx  Evaluation context (start/goal + helpers).
         * @return True if the candidate is acceptable; false otherwise.
         */
        virtual bool accept (const Path<N> &cand, const EvalContext<N> &ctx) const noexcept = 0;
    };

    /**
     * @brief Soft constraints yield an additive cost; lower is better.
     * @tparam N Maximum number of segments in the steering pattern (Path<N>).
     */
    template <std::size_t N> struct SoftConstraint
    {
        SoftConstraint () = default;
        SoftConstraint (const SoftConstraint &) = default;
        SoftConstraint (SoftConstraint &&) = default;
        SoftConstraint &operator= (const SoftConstraint &) = default;
        SoftConstraint &operator= (SoftConstraint &&) = default;
        virtual ~SoftConstraint () = default;

        /**
         * @brief Compute a cost contribution for a candidate.
         * @param cand Candidate path to score.
         * @param ctx  Evaluation context (start/goal + helpers).
         * @return Finite cost; return +∞ to effectively reject the candidate.
         */
        virtual double cost (const Path<N> &cand, const EvalContext<N> &ctx) const noexcept = 0;

        /**
         * @brief Get the unique name of this constraint for debugging/statistics.
         */
        virtual std::string name () const = 0;
    };

    /**
     * @brief Set of constraints with optional weights for soft terms.
     * @tparam N Maximum number of segments in the steering pattern (Path<N>).
     */
    template <std::size_t N> struct ConstraintSet
    {
        /// Collection of hard constraints (all must pass).
        std::vector<std::shared_ptr<HardConstraint<N>>> hard;

        /// (soft constraint, weight) pairs. Weight must be finite.
        std::vector<std::pair<std::shared_ptr<SoftConstraint<N>>, double>> soft;

        /**
         * @brief Check feasibility of a candidate against all hard constraints.
         * @param cand Candidate path to check.
         * @param ctx  Evaluation context.
         * @return True if all hard constraints accept the candidate.
         */
        [[nodiscard]] bool feasible (const Path<N> &cand, const EvalContext<N> &ctx) const noexcept
        {
            for (const auto &c : hard)
                if (!c->accept (cand, ctx))
                    return false;
            return true;
        }

        /**
         * @brief Compute the weighted sum of soft-constraint costs.
         * @param cand Candidate path to score.
         * @param ctx  Evaluation context.
         * @return Weighted score, or +∞ if any soft constraint returns a non-finite value.
         */
        [[nodiscard]] double score (const Path<N> &cand, const EvalContext<N> &ctx) const noexcept
        {
            double s = 0.0;
            for (const auto &[c, w] : soft)
            {
                const double v = c->cost (cand, ctx);
                if (!std::isfinite (v))
                    return std::numeric_limits<double>::infinity ();
                s += w * v;
            }
            return s;
        }
    };

} // namespace arcgen::planner::constraints
