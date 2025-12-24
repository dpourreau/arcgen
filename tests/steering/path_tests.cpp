/**
 * @file path_tests.cpp
 * @brief Unit tests for arcgen::steering::Path logic, specifically length calculation.
 */

#include <arcgen/steering/path.hpp>
#include <gtest/gtest.h>
#include <vector>

using namespace arcgen::core;
using namespace arcgen::steering;

// Use a concrete N for testing
static constexpr std::size_t kSegments = 3;

TEST (PathTests, EmptyPathLength)
{
    Path<kSegments> p;
    // Default constructor: n=0, states=nullopt
    EXPECT_DOUBLE_EQ (p.length (), 0.0);
}

TEST (PathTests, LengthFromControls)
{
    Path<kSegments> p;
    p.controls.n = 1;
    p.controls.length = 15.5;

    // Should prefer valid controls over states
    p.states = std::vector<State>{{0, 0, 0}, {1, 0, 0}}; // Geom length 1.0

    EXPECT_DOUBLE_EQ (p.length (), 15.5);
}

TEST (PathTests, LengthFromStatesFallback)
{
    Path<kSegments> p;
    p.controls.n = 0; // Invalid controls

    // Create a path: (0,0) -> (3,4) -> (3,0)
    // Seg 1: len 5. Seg 2: len 4. Total = 9.
    std::vector<State> sts;
    sts.push_back ({0, 0, 0});
    sts.push_back ({3, 4, 0});
    sts.push_back ({3, 0, 0});

    p.states = sts;

    EXPECT_DOUBLE_EQ (p.length (), 9.0);
}

TEST (PathTests, LengthFallbackSingleState)
{
    Path<kSegments> p;
    p.controls.n = 0;
    p.states = std::vector<State>{{10, 10, 0}};

    // Only 1 state => length 0
    EXPECT_DOUBLE_EQ (p.length (), 0.0);
}
