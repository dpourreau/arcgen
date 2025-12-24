/**
 * @file path_length_tests.cpp
 * @brief Unit tests for PathLengthConstraint (Soft Constraint).
 */

#include <arcgen/core/state.hpp>
#include <arcgen/planner/constraints/path_length.hpp>
#include <arcgen/steering/path.hpp>

#include <gtest/gtest.h>

using namespace arcgen::planner::constraints;
using namespace arcgen::steering;
using namespace arcgen::core;

/// @brief Test fixture for path length cost calculation.
class PathLengthConstraintTest : public ::testing::Test
{
  protected:
    static constexpr std::size_t N = 3;
    using Constraint = PathLengthConstraint<N>;
    using PathType = Path<N>;
    using Context = EvalContext<N>;

    Constraint constraint_;
    State dummy_{0, 0, 0};
    Context ctx_{dummy_, dummy_, [] (PathType &) {}};
};

/// @brief Verify cost calculation from `p.controls.length`.
TEST_F (PathLengthConstraintTest, CostReturnsLength)
{
    PathType p;
    // Method 1: Set controls length (priority 1)
    p.controls.n = 1;
    p.controls.length = 10.5;

    EXPECT_DOUBLE_EQ (constraint_.cost (p, ctx_), 10.5);
}

/// @brief Verify cost calculation from states if controls are empty.
TEST_F (PathLengthConstraintTest, CostFromStates)
{
    PathType p;
    p.controls.n = 0; // fallback to states
    p.states = std::vector<State>{{0, 0, 0}, {3, 0, 0}, {3, 4, 0}};
    // Length: (0,0)->(3,0) is 3. (3,0)->(3,4) is 4. Total 7.

    EXPECT_DOUBLE_EQ (constraint_.cost (p, ctx_), 7.0);
}

/// @brief Verify 0 cost for empty paths.
TEST_F (PathLengthConstraintTest, EmptyPathZeroCost)
{
    PathType p;
    p.controls.n = 0;
    // states empty
    EXPECT_DOUBLE_EQ (constraint_.cost (p, ctx_), 0.0);
}

/// @brief Verify 0 cost for single-point paths.
TEST_F (PathLengthConstraintTest, SingleStateZeroCost)
{
    PathType p;
    p.controls.n = 0;
    p.states = std::vector<State>{{10, 10, 3}};
    EXPECT_DOUBLE_EQ (constraint_.cost (p, ctx_), 0.0);
}
