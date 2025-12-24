/**
 * @file dubins_tests.cpp
 * @brief Unit tests for Dubins steering function.
 */

#include <steering/steering_fixtures.hpp>

#include <arcgen/steering/dubins.hpp>

using arcgen::steering::Dubins;

using TestedGenerators = ::testing::Types<Dubins>;
INSTANTIATE_TYPED_TEST_SUITE_P (Dubins, SteeringFixture, TestedGenerators);
