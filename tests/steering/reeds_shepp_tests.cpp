/**
 * @file reeds_shepp_tests.cpp
 * @brief Unit tests for Reeds-Shepp steering function.
 */

#include <steering/steering_fixtures.hpp>

#include <arcgen/steering/reeds_shepp.hpp>

using arcgen::steering::ReedsShepp;

using TestedGenerators = ::testing::Types<ReedsShepp>;
INSTANTIATE_TYPED_TEST_SUITE_P (ReedsShepp, SteeringFixture, TestedGenerators);
