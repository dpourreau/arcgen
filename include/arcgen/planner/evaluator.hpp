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

        Evaluator (const Steering *steering, const ConstraintSet *constraints) : steering_ (steering), constraints_ (constraints) {}

        /**
         * @brief Generate candidates via the steering policy and pick the best feasible one.
         * @return Best candidate states if any.
         */
        [[nodiscard]] std::optional<std::vector<arcgen::core::State>> bestStatesBetween (const arcgen::core::State &a, const arcgen::core::State &b) const
        {
            if (!steering_)
                return std::nullopt;
            auto cand = steering_->candidates (a, b);
            if (auto path = best (a, b, cand))
                return path->states;
            return std::nullopt;
        }

      private:
        [[nodiscard]] std::optional<PathT> best (const arcgen::core::State &a, const arcgen::core::State &b, std::vector<PathT> &cand) const
        {
            if (!steering_ || !constraints_)
                return std::nullopt;

            EvalContext ctx{a, b, [&] (PathT &p) { steering_->ensureStates (a, p); }};

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

            if (!argmin)
                return std::nullopt;

            steering_->ensureStates (a, cand[*argmin]); // ensure discretised states before returning
            return cand[*argmin];
        }

        const Steering *steering_{nullptr};
        const ConstraintSet *constraints_{nullptr};
    };

} // namespace arcgen::planner::engine
