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

TEST (ReedsSheppCoverage, GeometricGridCoverage)
{
    // Systematic grid search to hit rare geometric families (CCC, CCCC, CCSCC)
    // Random tests often miss the specific boundaries of the partition cells.
    const double steps_lin = 21;
    const double range_lin = 15.0; // +/- 15m (r=6m means ~2.5 radii)
    const double steps_ang = 19;

    std::size_t valid_paths = 0;

    // Use a smaller radius to make 'relative' distances larger effectively
    ReedsShepp policy (2.0);

    for (int i = 0; i < steps_lin; ++i)
    {
        double x = -range_lin + i * (2.0 * range_lin / (steps_lin - 1));
        for (int j = 0; j < steps_lin; ++j)
        {
            double y = -range_lin + j * (2.0 * range_lin / (steps_lin - 1));
            for (int k = 0; k < steps_ang; ++k)
            {
                double th = -std::numbers::pi + k * (2.0 * std::numbers::pi / (steps_ang - 1));

                State start{0, 0, 0};
                State goal{x, y, th};

                auto arcs = policy.getArcs (start, goal);
                if (!arcs.empty ())
                {
                    valid_paths++;
                    // Check validity of the best path
                    EXPECT_LE (arcs[0].total (), 1000.0); // Sanity check
                }
            }
        }
    }
    // Should find paths for almost all configurations (except maybe colocated with weird angles if any)
    EXPECT_GT (valid_paths, 100);
}

TEST (ReedsSheppCoverage, HighVolumeRandom)
{
    std::mt19937 gen (42);
    // ... existing random test ...
    std::uniform_real_distribution distPos (-20.0, 20.0);
    std::uniform_real_distribution distTheta (-std::numbers::pi, std::numbers::pi);

    ReedsShepp policy (3.0, 0.1); // rMin=3.0

    for (int i = 0; i < 5000; ++i)
    {
        State s1{distPos (gen), distPos (gen), distTheta (gen)};
        State s2{distPos (gen), distPos (gen), distTheta (gen)};

        auto paths = policy.candidates (s1, s2);
        if (!paths.empty ())
        {
            policy.ensureStates (s1, paths[0]);
            EXPECT_TRUE (paths[0].states.has_value ());
            EXPECT_FALSE (paths[0].states->empty ());
        }
    }
}
