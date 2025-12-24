/**
 * @file state_tests.cpp
 * @brief Unit tests for State struct and helpers.
 */

#include <arcgen/core/control.hpp>
#include <arcgen/core/state.hpp>
#include <gtest/gtest.h>

using namespace arcgen::core;

/// @brief Verify default construction to zero.
TEST (StateTests, Construction)
{
    State s;
    EXPECT_EQ (s.x, 0.0);
    EXPECT_EQ (s.y, 0.0);
    EXPECT_EQ (s.heading, 0.0);
    EXPECT_EQ (s.curvature, 0.0);
    EXPECT_EQ (s.direction, DrivingDirection::Neutral);
}

/// @brief Verify setting direction from signed values.
TEST (StateTests, SetDirection)
{
    State s;
    s.setDirectionFrom (5.0);
    EXPECT_EQ (s.direction, DrivingDirection::Forward);

    s.setDirectionFrom (-3.0);
    EXPECT_EQ (s.direction, DrivingDirection::Reverse);

    s.setDirectionFrom (0.0);
    EXPECT_EQ (s.direction, DrivingDirection::Neutral);
}
