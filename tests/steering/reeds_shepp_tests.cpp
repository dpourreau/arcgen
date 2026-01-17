/**
 * @file reeds_shepp_tests.cpp
 * @brief Unit tests for Reeds-Shepp steering function.
 */

#include <steering/steering_fixtures.hpp>

#include <arcgen/steering/reeds_shepp.hpp>
#include <gtest/gtest.h>

using arcgen::steering::ReedsShepp;

using TestedGenerators = ::testing::Types<ReedsShepp>;
INSTANTIATE_TYPED_TEST_SUITE_P (ReedsShepp, SteeringFixture, TestedGenerators);

TEST (ReedsSheppSpecific, MassiveRandomCoverage)
{
    // High volume random testing to hit rare path families (long CCCC, CCSCC etc)
    // and cover all geometric primitive branches (success/fail).
    ReedsShepp gen (3.0, 0.1); // rMin=3.0
    std::mt19937 rng (12345);
    std::uniform_real_distribution pos (-20.0, 20.0);
    std::uniform_real_distribution ang (0.0, arcgen::core::two_pi);

    const int iterations = 5000;
    for (int k = 0; k < iterations; ++k)
    {
        State start{pos (rng), pos (rng), ang (rng)};
        State goal{pos (rng), pos (rng), ang (rng)};
        auto path = gen.shortestPath (start, goal);

        // Basic sanity checks
        // Should always find a path for Reeds-Shepp, unless identity (start==goal)
        if ((!path.states || path.states->empty ()) && euclidean (start, goal) > 1e-6)
        {
            FAIL () << "Failed to find path for random pair";
        }
    }
}
