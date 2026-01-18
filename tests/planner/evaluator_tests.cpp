/**
 * @file evaluator_tests.cpp
 * @brief Unit tests for the Evaluator, verifying candidate selection and constraint application.
 */

#include <arcgen/core/state.hpp>
#include <arcgen/planner/constraints/constraints.hpp>
#include <arcgen/planner/engine/evaluator.hpp>
#include <arcgen/steering/path.hpp>
#include <gtest/gtest.h>
#include <vector>

using namespace arcgen::planner::engine;
using namespace arcgen::planner::constraints;
using namespace arcgen::core;
using namespace arcgen::steering;

/// @brief Mock Steering Policy with controllable candidates.
struct MockSteering
{
    static constexpr std::size_t kSegments = 1;
    using PathType = Path<kSegments>;

    mutable std::vector<PathType> cannedCandidates;

    [[nodiscard]] std::vector<PathType> candidates (const State & /*start*/, const State & /*goal*/) const { return cannedCandidates; }

    void ensureStates (const State & /*start*/, const PathType &path) const
    {
        if (!path.states)
        {
            path.states = std::vector<State> (1); // Dummy state
        }
    }
};

/// @brief Mock Hard Constraint for testing rejection logic.
template <size_t N> class MockHardConstraint : public HardConstraint<N>
{
    double limit_;

  public:
    explicit MockHardConstraint (double limit) : limit_ (limit) {}
    bool accept (const Path<N> &p, const EvalContext<N> &) const noexcept override { return p.length () <= limit_; }
    // HardConstraint does NOT have name()
};

/// @brief Mock Soft Constraint for testing cost aggregation.
template <size_t N> class MockSoftConstraint : public SoftConstraint<N>
{
  public:
    double cost (const Path<N> &p, const EvalContext<N> &) const noexcept override
    {
        return p.length () * 2.0; // Double the cost
    }
    std::string name () const override { return "MockSoftConstraint"; }
};

class EvaluatorFixture : public ::testing::Test
{
  public:
    static constexpr std::size_t N = MockSteering::kSegments;
    using PathType = MockSteering::PathType;
    using ConstraintSetType = ConstraintSet<N>;
    using EvaluatorType = Evaluator<MockSteering>;

    MockSteering steering_;
    ConstraintSetType constraints_;
    std::unique_ptr<EvaluatorType> evaluator_;

    State start_{0, 0, 0};
    State goal_{10, 10, 0};

    void SetUp () override { evaluator_ = std::make_unique<EvaluatorType> (&steering_, &constraints_); }

    PathType makePath (double len) const
    {
        PathType p;
        p.controls.n = 1;
        p.controls.length = len;
        // ensureStates will populate states if missing
        return p;
    }
};

/// @brief Verify that no candidates results in std::nullopt.
TEST_F (EvaluatorFixture, NoCandidates)
{
    steering_.cannedCandidates.clear ();
    auto result = evaluator_->bestStatesBetween (start_, goal_);
    EXPECT_FALSE (result.has_value ());
}

/// @brief Verify selection of the shortest path when no constraints are present.
TEST_F (EvaluatorFixture, UnconstrainedSelection)
{
    // Should pick shortest path (default cost)
    steering_.cannedCandidates = {makePath (10.0), makePath (5.0), makePath (20.0)};

    auto result = evaluator_->bestStatesBetween (start_, goal_);
    ASSERT_TRUE (result.has_value ());
    EXPECT_DOUBLE_EQ (result->second, 5.0);
}

/// @brief Verify that hard constraints strictly reject invalid paths.
TEST_F (EvaluatorFixture, HardFeasibility)
{
    // Limit length <= 10
    constraints_.hard.emplace_back (std::make_shared<MockHardConstraint<N>> (10.0));

    steering_.cannedCandidates = {
        makePath (5.0),  // Valid
        makePath (15.0), // Invalid
        makePath (8.0)   // Valid, but longer than 5
    };

    auto result = evaluator_->bestStatesBetween (start_, goal_);
    ASSERT_TRUE (result.has_value ());
    EXPECT_DOUBLE_EQ (result->second, 5.0);

    // If we only have invalid candidates
    steering_.cannedCandidates = {makePath (12.0)};
    result = evaluator_->bestStatesBetween (start_, goal_);
    EXPECT_FALSE (result.has_value ());
}

/// @brief Verify that soft constraints influence the optimal choice.
TEST_F (EvaluatorFixture, SoftOptimization)
{
    // Add soft constraint (cost = length * 2) with weight 1.0
    constraints_.soft.emplace_back (std::make_shared<MockSoftConstraint<N>> (), 1.0);

    // Candidates:
    // A: 5.0 (Cost 10.0)
    // B: 3.0 (Cost 6.0)
    steering_.cannedCandidates = {makePath (5.0), makePath (3.0)};

    auto result = evaluator_->bestStatesBetween (start_, goal_);
    ASSERT_TRUE (result.has_value ());
    EXPECT_DOUBLE_EQ (result->second, 6.0); // 3.0 * 2
}

/// @brief Verify cost calculation for a dense path.
TEST_F (EvaluatorFixture, EvaluatorCostDense)
{
    // Test evaluateCost(span<State>)

    // Add soft constraint with weight 1.0
    constraints_.soft.emplace_back (std::make_shared<MockSoftConstraint<N>> (), 1.0);

    std::vector<State> densePath (3);
    // 3 points, length ~2.0
    densePath[0] = {0, 0, 0};
    densePath[1] = {1, 0, 0};
    densePath[2] = {2, 0, 0};

    // MockSoft returns length * 2. Length approx 2.0. Cost should be 4.0.
    double cost = evaluator_->evaluateCost (densePath);
    EXPECT_NEAR (cost, 4.0, 1e-9);
}

/// @brief Verify that infinite soft cost acts as a hard rejection.
TEST_F (EvaluatorFixture, SoftRejection)
{
    // Soft constraint returns infinity should act like hard rejection
    class InfConstraint : public SoftConstraint<N>
    {
      public:
        double cost (const Path<N> &, const EvalContext<N> &) const noexcept override { return std::numeric_limits<double>::infinity (); }
        std::string name () const override { return "InfConstraint"; }
    };

    constraints_.soft.emplace_back (std::make_shared<InfConstraint> (), 1.0);
    steering_.cannedCandidates = {makePath (5.0)};

    auto result = evaluator_->bestStatesBetween (start_, goal_);
    // All candidates have infinite cost -> bestScore stays infinity -> argmin empty -> nullopt
    EXPECT_FALSE (result.has_value ());
}

/// @brief Verify aggregation of multiple soft constraints.
TEST_F (EvaluatorFixture, MultipleSoftConstraints)
{
    // C1: cost = len * 1.0. Weight 1.0.
    // C2: cost = len * 2.0. Weight 0.5.
    // Total = len*1*1 + len*2*0.5 = 2.0 * len.

    class LinearConstraint : public SoftConstraint<N>
    {
        double factor_;

      public:
        explicit LinearConstraint (double f) : factor_ (f) {}
        double cost (const Path<N> &p, const EvalContext<N> &) const noexcept override { return p.length () * factor_; }
        std::string name () const override { return "Linear"; }
    };

    constraints_.soft.emplace_back (std::make_shared<LinearConstraint> (1.0), 1.0);
    constraints_.soft.emplace_back (std::make_shared<LinearConstraint> (2.0), 0.5);

    steering_.cannedCandidates = {makePath (10.0)};

    auto result = evaluator_->bestStatesBetween (start_, goal_);
    ASSERT_TRUE (result.has_value ());
    // Cost: 10*1*1 + 10*2*0.5 = 10 + 10 = 20.
    EXPECT_DOUBLE_EQ (result->second, 20.0);
}

/// @brief Verify bestStatesBetween behavior when steering is null.
TEST_F (EvaluatorFixture, NullSteering)
{
    steering_.cannedCandidates = {makePath (10.0)};
    auto nullSteerEval = std::make_unique<EvaluatorType> (nullptr, &constraints_);
    auto result = nullSteerEval->bestStatesBetween (start_, goal_);
    EXPECT_FALSE (result.has_value ());
}

/// @brief Verify behavior when constraints are null.
TEST_F (EvaluatorFixture, NullConstraints)
{
    steering_.cannedCandidates = {makePath (10.0)};
    auto nullConstEval = std::make_unique<EvaluatorType> (&steering_, nullptr);

    // bestStatesBetween should return nullopt if constraints are null (as per logic in best())
    auto result = nullConstEval->bestStatesBetween (start_, goal_);
    EXPECT_FALSE (result.has_value ());
}

/// @brief Verify evaluateCost returns 0.0 when constraints are null.
TEST_F (EvaluatorFixture, EvaluateCostNull)
{
    auto nullConstEval = std::make_unique<EvaluatorType> (&steering_, nullptr);
    std::vector<State> path (2);
    double cost = nullConstEval->evaluateCost (path);
    EXPECT_DOUBLE_EQ (cost, 0.0);
}

/// @brief Verify evaluateCost returns 0.0 when path is empty.
TEST_F (EvaluatorFixture, EvaluateCostEmpty)
{
    std::vector<State> emptyPath;
    // Ensure constraints exist so we don't hit the null constraint check first
    constraints_.soft.emplace_back (std::make_shared<MockSoftConstraint<N>> (), 1.0);
    double cost = evaluator_->evaluateCost (emptyPath);
    EXPECT_DOUBLE_EQ (cost, 0.0);
}
