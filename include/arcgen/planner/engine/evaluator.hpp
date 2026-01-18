#pragma once
/**
 * @file evaluator.hpp
 * @brief Select the best feasible steering candidate under constraints.
 */

#include <arcgen/core/state.hpp>
#include <arcgen/planner/constraints/constraints.hpp>
#include <arcgen/steering/steering.hpp>

#include <limits>
#include <optional>
#include <utility>
#include <vector>

#if defined(_OPENMP)
    #include <omp.h>
#endif

namespace arcgen::planner::engine
{
    /**
     * @brief Evaluates steering candidates against constraints and picks the best feasible one.
     *
     * @tparam Steering Steering policy type (must expose PathType and static kSegments).
     */
    template <class Steering> class Evaluator final
    {
      public:
        using PathT = typename Steering::PathType;
        static constexpr std::size_t N = Steering::kSegments;
        using ConstraintSet = arcgen::planner::constraints::ConstraintSet<N>;
        using EvalContext = arcgen::planner::constraints::EvalContext<N>;

        /**
         * @brief Construct with steering policy and constraint set.
         * @param steering    Steering function policy.
         * @param constraints Constraint set to evaluate against.
         */
        Evaluator (const Steering *steering, const ConstraintSet *constraints) : steering_ (steering), constraints_ (constraints) {}

        /**
         * @brief Get the associated constraints.
         * @return Pointer to the constraint set.
         */
        [[nodiscard]] const ConstraintSet *getConstraints () const noexcept { return constraints_; }

        /**
         * @brief Generate candidates via the steering policy and pick the best feasible one.
         * @return Best candidate states and its score if any.
         */
        [[nodiscard]] std::optional<std::pair<std::vector<arcgen::core::State>, double>> bestStatesBetween (const arcgen::core::State &a, const arcgen::core::State &b) const
        {
            if (!steering_)
                return std::nullopt;
            auto cand = steering_->candidates (a, b);
            if (auto path = best (a, b, cand))
                return std::make_pair (path->first.states.value (), path->second);
            return std::nullopt;
        }

        /**
         * @brief Compute the score of an existing dense path under strict soft-constraints.
         *        Skipping hard feasibility checks (assumed valid).
         * @param path The dense sequence of states to evaluate.
         * @return Weighted soft-constraint score.
         */
        [[nodiscard]] double evaluateCost (std::span<const arcgen::core::State> path) const
        {
            if (!constraints_ || path.empty ())
                return 0.0;

            PathT p;
            // Deep copy required: PathT uses std::vector (owning), input is std::span (view).
            p.states.emplace (path.begin (), path.end ());

            EvalContext ctx{path.front (), path.back (), [] (const PathT &) { /* no-op */ }};
            return constraints_->score (p, ctx);
        }

      private:
        /**
         * @brief Internal helper to evaluate candidates and select the best one.
         * @param a    Start state.
         * @param b    Goal state.
         * @param cand List of candidate paths (mutable for lazy generation).
         * @return Best path and its score, or nullopt if none feasible.
         */
        [[nodiscard]] std::optional<std::pair<PathT, double>> best (const arcgen::core::State &a, const arcgen::core::State &b, std::vector<PathT> &cand) const
        {
            if (!steering_ || !constraints_)
                return std::nullopt;

            EvalContext ctx{a, b, [this, &a] (const PathT &p) { steering_->ensureStates (a, p); }};

            std::optional<std::size_t> argmin;
            double bestScore = std::numeric_limits<double>::infinity ();

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
                    if (!constraints_->feasible (c, ctx))
                        continue;

                    const double score = constraints_->soft.empty () ? c.length () : constraints_->score (c, ctx);
                    if (score < bestLocal)
                    {
                        bestLocal = score;
                        argLocal = static_cast<std::size_t> (i);
                    }
                }

    #pragma omp critical
                {
                    if (argLocal && bestLocal < bestScore)
                    {
                        bestScore = bestLocal;
                        argmin = *argLocal;
                    }
                }
            }
#else
            for (std::size_t i = 0; i < cand.size (); ++i)
            {
                if (!constraints_->feasible (cand[i], ctx))
                    continue;

                const double score = constraints_->soft.empty () ? cand[i].length () : constraints_->score (cand[i], ctx);
                if (score < bestScore)
                {
                    bestScore = score;
                    argmin = i;
                }
            }
#endif

            if (!argmin.has_value ())
                return std::nullopt;

            steering_->ensureStates (a, cand[*argmin]); // ensure discretised states before returning
            return std::make_pair (cand[*argmin], bestScore);
        }

        const Steering *steering_{nullptr};         ///< Steering policy.
        const ConstraintSet *constraints_{nullptr}; ///< Constraints set.
    };

} // namespace arcgen::planner::engine
