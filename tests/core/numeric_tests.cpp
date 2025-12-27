/**
 * @file numeric_tests.cpp
 * @brief Sanity checks for project-wide constants in numeric.hpp
 */

#include <arcgen/core/numeric.hpp>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <numbers>

using namespace arcgen::core;

TEST (NumericTests, EpsilonConsistency)
{
    // Check that EPSILON matches std::numeric_limits<double>::epsilon()
    EXPECT_EQ (EPSILON, std::numeric_limits<double>::epsilon ());
    EXPECT_EQ (EPSILON_V<double>, std::numeric_limits<double>::epsilon ());
    EXPECT_EQ (EPSILON_V<float>, std::numeric_limits<float>::epsilon ());
}

TEST (NumericTests, PiConsistency)
{
    // Check that PI matches std::numbers::pi_v<double>
    EXPECT_EQ (PI, std::numbers::pi_v<double>);
    EXPECT_EQ (PI_V<double>, std::numbers::pi_v<double>);
    EXPECT_EQ (PI_V<float>, std::numbers::pi_v<float>);

    EXPECT_DOUBLE_EQ (PI2, 2.0 * PI);
    EXPECT_DOUBLE_EQ (PI_OVER_TWO, PI / 2.0);
}

TEST (NumericTests, PlannerTolerances)
{
    // Sanity check that tolerances are positive and small (except zero-tols which might be negative/zero)
    EXPECT_GT (REEDS_SHEPP_TOL, 0.0);
    EXPECT_GT (DUBINS_TOL, 0.0);
    EXPECT_GT (CURVATURE_TOL, 0.0);
    EXPECT_GT (GREEDY_MIN_RESAMPLE_INTERVAL, 0.0);

    // Check zero tolerances are reasonably small
    EXPECT_NEAR (REEDS_SHEPP_ZERO_TOL, 0.0, 1e-9);
    EXPECT_NEAR (DUBINS_ZERO_TOL, 0.0, 1e-9);

    // Greedy cost improvement tolerance should be > 0
    EXPECT_GT (GREEDY_COST_IMPROVEMENT_TOL, 0.0);
}
