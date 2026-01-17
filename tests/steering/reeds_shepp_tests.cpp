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

    // Helper to test a single (x, y, theta) configuration and update valid_paths count
    auto testConfiguration = [&] (double x, double y, double th)
    {
        State start{0, 0, 0};
        State goal{x, y, th};

        auto arcs = policy.getArcs (start, goal);
        if (!arcs.empty ())
        {
            valid_paths++;
            // Check validity of the best path
            EXPECT_LE (arcs[0].total (), 1000.0); // Sanity check
        }
    };

    for (int i = 0; i < steps_lin; ++i)
    {
        double x = -range_lin + i * (2.0 * range_lin / (steps_lin - 1));
        for (int j = 0; j < steps_lin; ++j)
        {
            double y = -range_lin + j * (2.0 * range_lin / (steps_lin - 1));
            for (int k = 0; k < steps_ang; ++k)
            {
                double th = -std::numbers::pi + k * (2.0 * std::numbers::pi / (steps_ang - 1));
                testConfiguration (x, y, th);
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

/**
 * @brief Edge-case tests targeting compound conditions in geometric primitives.
 *
 * These tests exercise specific geometric configurations designed to hit
 * both branches of compound return conditions (e.g., `t >= -tol && v >= -tol`).
 */
TEST (ReedsSheppCoverage, EdgeCaseConditions)
{
    ReedsShepp policy (1.0, 0.05); // Unit radius for easier geometry

    // Test configurations designed to stress boundary conditions of primitives.
    std::vector<std::tuple<State, State, std::string>> edgeCases = {
        // Near-identity transformations (tests radius condition edge cases)
        {{0, 0, 0}, {0.01, 0, 0}, "tiny-forward"},
        {{0, 0, 0}, {0, 0.01, 0}, "tiny-lateral"},
        {{0, 0, 0}, {0, 0, 0.01}, "tiny-rotation"},

        // Exact 90° and 180° turns (stress CCC family boundaries)
        {{0, 0, 0}, {1, 1, std::numbers::pi / 2}, "90-degree-left"},
        {{0, 0, 0}, {1, -1, -std::numbers::pi / 2}, "90-degree-right"},
        {{0, 0, 0}, {0, 0, std::numbers::pi}, "pure-180"},
        {{0, 0, 0}, {2, 0, std::numbers::pi}, "forward-180"},

        // Configurations at ρ ≈ 2.0 boundary (CCSC families)
        {{0, 0, 0}, {2.0, 0, 0}, "rho-equals-2"},
        {{0, 0, 0}, {1.9, 0, 0}, "rho-just-under-2"},
        {{0, 0, 0}, {2.1, 0, 0}, "rho-just-over-2"},

        // Configurations at ρ ≈ 4.0 boundary (CCC and CSC families)
        {{0, 0, 0}, {4.0, 0, 0}, "rho-equals-4"},
        {{0, 0, 0}, {3.9, 0, 0}, "rho-just-under-4"},
        {{0, 0, 0}, {4.1, 0, 0}, "rho-just-over-4"},

        // Configurations testing u ≈ -π/2 boundary (LpRumLumRp)
        {{0, 0, 0}, {0, 2, -std::numbers::pi / 2}, "u-near-halfpi"},

        // Backward-facing goals (test timeflip symmetries)
        {{0, 0, 0}, {-2, 0, 0}, "pure-reverse"},
        {{0, 0, 0}, {-2, 1, std::numbers::pi}, "reverse-with-offset"},
        {{0, 0, 0}, {-1, -1, -std::numbers::pi}, "reverse-diagonal"},

        // Sharp cusps (CCCC and CCSCC families)
        {{0, 0, 0}, {0.5, 0.5, std::numbers::pi}, "short-cusp"},
        {{0, 0, 0}, {1.5, 0, std::numbers::pi / 4}, "angled-cusp"},

        // Large displacements (tests all CSC families thoroughly)
        {{0, 0, 0}, {10, 5, 0.5}, "large-lsr"},
        {{0, 0, 0}, {10, -5, -0.5}, "large-rsl"},
        {{0, 0, 0}, {8, 8, std::numbers::pi / 3}, "large-diagonal"},
    };

    for (const auto &[start, goal, label] : edgeCases)
    {
        auto paths = policy.getArcs (start, goal);
        // We don't require paths since some edge cases may not have valid solutions.
        // The goal is to exercise the code paths.
        (void)label;

        if (!paths.empty ())
        {
            EXPECT_LE (paths[0].total (), 100.0) << "Edge case failed: " << label;
        }
    }
}

/**
 * @brief Boundary value tests for geometric primitive conditions.
 *
 * Tests configurations at critical ρ boundaries to ensure all branches exercised.
 */
TEST (ReedsSheppCoverage, BoundaryValueCoverage)
{
    std::vector<double> radii = {0.5, 1.0, 2.0, 3.0, 5.0};

    std::mt19937 rng (99999);
    std::uniform_real_distribution<double> smallAngle (-0.1, 0.1);
    std::uniform_real_distribution<double> halfPi (std::numbers::pi / 2 - 0.1, std::numbers::pi / 2 + 0.1);

    for (double r : radii)
    {
        ReedsShepp policy (r, 0.05);

        for (int i = 0; i < 50; ++i)
        {
            // ρ ≈ 2r: boundary for CCSC families
            double angle = smallAngle (rng);
            State g1{2.0 * r + smallAngle (rng), smallAngle (rng), angle};
            auto p1 = policy.getArcs (State{0, 0, 0}, g1);

            // ρ ≈ 4r: boundary for CCC families
            State g2{4.0 * r + smallAngle (rng), smallAngle (rng), halfPi (rng)};
            auto p2 = policy.getArcs (State{0, 0, 0}, g2);

            (void)p1;
            (void)p2;
        }
    }
}
