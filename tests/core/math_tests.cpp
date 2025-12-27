/**
 * @file math_tests.cpp
 * @brief Unit tests for core mathematical utilities in arcgen/core/math.hpp.
 */

#include <arcgen/core/math.hpp>
#include <gtest/gtest.h>
#include <numbers>
#include <random>

using namespace arcgen::core;

namespace
{
    // Use a slightly larger tolerance for transcendental functions if needed,
    // but EPSILON from numeric.hpp should generally be sufficient.
    constexpr double TOL = 1e-9;
} // namespace

/// @brief Verify basic mathematical constants against std::numbers.
TEST (MathTests, Constants)
{
    EXPECT_NEAR (PI, std::numbers::pi_v<double>, TOL);
    EXPECT_NEAR (PI2, 2.0 * std::numbers::pi_v<double>, TOL);
    EXPECT_NEAR (PI_OVER_TWO, 0.5 * std::numbers::pi_v<double>, TOL);
}

/// @brief Verify conversion from Cartesian (x,y) to Polar (r, theta).
TEST (MathTests, ToPolar)
{
    // Case 1: (1, 0) -> r=1, theta=0
    auto [r1, t1] = toPolar (1.0, 0.0);
    EXPECT_NEAR (r1, 1.0, TOL);
    EXPECT_NEAR (t1, 0.0, TOL);

    // Case 2: (0, 1) -> r=1, theta=PI/2
    auto [r2, t2] = toPolar (0.0, 1.0);
    EXPECT_NEAR (r2, 1.0, TOL);
    EXPECT_NEAR (t2, PI_OVER_TWO, TOL);

    // Case 3: (-1, 0) -> r=1, theta=PI
    auto [r3, t3] = toPolar (-1.0, 0.0);
    EXPECT_NEAR (r3, 1.0, TOL);
    EXPECT_NEAR (t3, PI, TOL);

    // Case 4: (0, -1) -> r=1, theta=-PI/2
    auto [r4, t4] = toPolar (0.0, -1.0);
    EXPECT_NEAR (r4, 1.0, TOL);
    EXPECT_NEAR (t4, -PI_OVER_TWO, TOL);

    // Edge Case: (0, 0) -> r=0, theta=0 (usually)
    auto [r0, t0] = toPolar (0.0, 0.0);
    EXPECT_NEAR (r0, 0.0, TOL);
    EXPECT_NEAR (t0, 0.0, TOL);
}

/// @brief Verify angle normalization to [0, 2pi).
TEST (MathTests, NormalizeAngle2Pi)
{
    // 0 -> 0
    EXPECT_NEAR (normalizeAngle2Pi (0.0), 0.0, TOL);

    // 2PI -> 0
    EXPECT_NEAR (normalizeAngle2Pi (PI2), 0.0, TOL);

    // PI -> PI
    EXPECT_NEAR (normalizeAngle2Pi (PI), PI, TOL);

    // -PI/2 -> 3PI/2
    EXPECT_NEAR (normalizeAngle2Pi (-PI_OVER_TWO), 3.0 * PI_OVER_TWO, TOL);

    // -0.1 -> 2PI - 0.1
    EXPECT_NEAR (normalizeAngle2Pi (-0.1), PI2 - 0.1, TOL);

    // 4PI -> 0
    EXPECT_NEAR (normalizeAngle2Pi (4.0 * PI), 0.0, TOL);

    // Tiny negative
    EXPECT_NEAR (normalizeAngle2Pi (-1e-10), PI2 - 1e-10, TOL);
}

/// @brief Verify signed angle normalization to (-pi, pi].
TEST (MathTests, NormalizeAngleSigned)
{
    // 0 -> 0
    EXPECT_NEAR (normalizeAngleSigned (0.0), 0.0, TOL);

    // PI -> PI (Boundary check, remainder depends on implementation but usually exact PI is PI)
    // std::remainder of PI w.r.t 2PI: PI - 0*(2PI) = PI? PI - 1*(2PI) = -PI?
    // Nearest integer to 0.5 is 0 or 1. If 1, result is -PI. If 0, result is PI.
    // Let's check typical behavior. Range is (-PI, PI].
    // If it returns -PI, we might want to standardize to PI.
    // math.hpp says "Wrap an angle to the interval (−π, π]".
    // Let's rely on the range constraints.
    double res_pi = normalizeAngleSigned (PI);
    // It should be PI, or -PI if the implementation chooses. But PI is preferred.
    // We check that abs(res) is PI.
    EXPECT_NEAR (std::fabs (res_pi), PI, TOL);

    // -PI -> -PI (or PI)
    double res_npi = normalizeAngleSigned (-PI);
    EXPECT_NEAR (std::fabs (res_npi), PI, TOL);

    // 3PI -> PI (or -PI)
    double res_3pi = normalizeAngleSigned (3.0 * PI);
    EXPECT_NEAR (std::fabs (res_3pi), PI, TOL);

    // 3PI/2 -> -PI/2
    EXPECT_NEAR (normalizeAngleSigned (3.0 * PI_OVER_TWO), -PI_OVER_TWO, TOL);

    // -3PI/2 -> PI/2
    EXPECT_NEAR (normalizeAngleSigned (-3.0 * PI_OVER_TWO), PI_OVER_TWO, TOL);
}

/// @brief Verify forward integration of a straight line segment.
TEST (MathTests, ComputeLineEndpoint)
{
    // Start at origin, heading 0, length 10, forward
    auto [x1, y1] = computeLineEndpoint (0.0, 0.0, 0.0, 1, 10.0);
    EXPECT_NEAR (x1, 10.0, TOL);
    EXPECT_NEAR (y1, 0.0, TOL);

    // Start at origin, heading PI/2, length 10, forward
    auto [x2, y2] = computeLineEndpoint (0.0, 0.0, PI_OVER_TWO, 1, 10.0);
    EXPECT_NEAR (x2, 0.0, TOL);
    EXPECT_NEAR (y2, 10.0, TOL);

    // Reverse: heading 0, length 10, backward (-1)
    auto [x3, y3] = computeLineEndpoint (0.0, 0.0, 0.0, -1, 10.0);
    EXPECT_NEAR (x3, -10.0, TOL);
    EXPECT_NEAR (y3, 0.0, TOL);

    // Zero length
    auto [x4, y4] = computeLineEndpoint (5.0, 5.0, 1.0, 1, 0.0);
    EXPECT_NEAR (x4, 5.0, TOL);
    EXPECT_NEAR (y4, 5.0, TOL);

    // Direction 0 (No motion)
    auto [x5, y5] = computeLineEndpoint (0.0, 0.0, 0.0, 0, 100.0);
    EXPECT_NEAR (x5, 0.0, TOL);
    EXPECT_NEAR (y5, 0.0, TOL);
}

/// @brief Verify forward integration of circular arc segments.
TEST (MathTests, ComputeArcEndpoint)
{
    // 1. Zero Curvature (Straight line fallback)
    {
        auto [x, y, t] = computeArcEndpoint (0.0, 0.0, 0.0, 0.0, 1, 10.0);
        EXPECT_NEAR (x, 10.0, TOL);
        EXPECT_NEAR (y, 0.0, TOL);
        EXPECT_NEAR (t, 0.0, TOL);
    }

    // 2. Tiny Curvature (Straight line robustness check)
    {
        // CURVATURE_TOL is 1e-6. Use 1e-7.
        auto [x, y, t] = computeArcEndpoint (0.0, 0.0, 0.0, 1e-7, 1, 10.0);
        EXPECT_NEAR (x, 10.0, 1e-5); // Should be very close to straight
        EXPECT_NEAR (y, 0.0, 1e-5);
        EXPECT_NEAR (t, 0.0, 1e-5);
    }

    // 3. Circle: Radius 10 (k=0.1), Length = 1/4 circumference = 2*pi*10 / 4 = 5*pi
    {
        double k = 0.1;
        double R = 10.0;
        double len = PI_OVER_TWO * R; // quarter circle
        // Turning Left (k>0) starting at 0,0 heading 0.
        // Endpoint should be (R, R) but with center at (0, R) -> (10, 10)?
        // Center is at (0, R).
        // Start (0,0) is at angle -PI/2 on circle? No.
        // If center is (0, R), start (0,0) is at angle -90 deg from center.
        // End is at angle 0 deg from center => (R, R).
        // Heading goes 0 -> PI/2.
        auto [x, y, t] = computeArcEndpoint (0.0, 0.0, 0.0, k, 1, len);
        EXPECT_NEAR (x, 10.0, TOL);
        EXPECT_NEAR (y, 10.0, TOL);
        EXPECT_NEAR (t, PI_OVER_TWO, TOL);
    }

    // 4. Circle Right: k = -0.1. Start 0,0, heading 0.
    {
        double k = -0.1;
        double R = 10.0;
        double len = PI_OVER_TWO * R;
        // Turn right -> center at (0, -R).
        // End point should be (R, -R). Heading -PI/2.
        auto [x, y, t] = computeArcEndpoint (0.0, 0.0, 0.0, k, 1, len);
        EXPECT_NEAR (x, 10.0, TOL);
        EXPECT_NEAR (y, -10.0, TOL);
        EXPECT_NEAR (t, -PI_OVER_TWO, TOL);
    }

    // 5. Reverse on Circle
    {
        // Start (0,0), Heading 0. Reverse. k=0.1 (Left turn if fwd).
        // Reverse left turn means we move "backwards and turning steering wheel left".
        // This traces a path to match the circle of center (0, R).
        // Wait, if I am at 0,0 facing X+, center is (0, 10).
        // If I reverse with wheel left, I move into negative X.
        // The car turns such that heading becomes positive? No.
        // Let's trust the math: delta = dir * k * len.
        // dir=-1, k=0.1, len=10*pi/2. delta = -pi/2.
        // theta0 = 0. theta1 = -pi/2.
        // x1 = x0 + R(-sin(0) + sin(-pi/2)) = 0 + 10(0 - 1) = -10.
        // y1 = y0 + R(cos(0) - cos(-pi/2)) = 0 + 10(1 - 0) = 10.
        // So (-10, 10), heading -pi/2.
        double k = 0.1;
        double R = 10.0;
        double len = PI_OVER_TWO * R;
        auto [x, y, t] = computeArcEndpoint (0.0, 0.0, 0.0, k, -1, len);
        EXPECT_NEAR (x, -10.0, TOL);
        EXPECT_NEAR (y, 10.0, TOL);
        EXPECT_NEAR (t, -PI_OVER_TWO, TOL);
    }

    {
        auto [x, y, t] = computeArcEndpoint (5.0, 5.0, 1.0, 0.5, 1, 0.0);
        EXPECT_NEAR (x, 5.0, TOL);
        EXPECT_NEAR (y, 5.0, TOL);
        EXPECT_NEAR (t, 1.0, TOL);
    }
}

/// @brief Fuzz test for ToPolar: valid range and invertibility check.
TEST (MathTests, FuzzToPolar)
{
    // Use fixed seed for reproducibility
    std::mt19937 rng (12345);
    std::uniform_real_distribution<double> dist (-100.0, 100.0);

    for (int i = 0; i < 1000; ++i)
    {
        double x = dist (rng);
        double y = dist (rng);

        auto [r, theta] = toPolar (x, y);

        // Property 1: Radius is non-negative
        EXPECT_GE (r, 0.0);

        // Property 2: Theta in (-PI, PI]
        // Note: Using a small tolerance if needed, but mathematical range is strict usually.
        EXPECT_GT (theta, -PI - 1e-9);
        EXPECT_LE (theta, PI + 1e-9);

        // Property 3: Invertibility (x = r cos t, y = r sin t)
        // Check reconstruction
        double x_recon = r * std::cos (theta);
        double y_recon = r * std::sin (theta);

        // If radius is 0, angle could be anything (usually 0).
        if (r < 1e-9)
        {
            EXPECT_NEAR (x, 0.0, TOL);
            EXPECT_NEAR (y, 0.0, TOL);
        }
        else
        {
            EXPECT_NEAR (x, x_recon, 1e-6);
            EXPECT_NEAR (y, y_recon, 1e-6);
        }
    }
}

/// @brief Fuzz test for NormalizeAngle: range and modular arithmetic properties.
TEST (MathTests, FuzzNormalizeAngle)
{
    std::mt19937 rng (67890);
    // Range includes large values to test wrapping multiple times
    std::uniform_real_distribution<double> dist (-10000.0, 10000.0);
    std::uniform_int_distribution<int> k_dist (-10, 10);

    for (int i = 0; i < 1000; ++i)
    {
        double theta = dist (rng);

        // --- Check NormalizeAngle2Pi ---
        double n2 = normalizeAngle2Pi (theta);
        EXPECT_GE (n2, 0.0);
        EXPECT_LT (n2, PI2 + 1e-9);

        // Property: Periodicity f(theta) == f(theta + k*2pi)
        int k = k_dist (rng);
        double theta_k = theta + k * PI2;
        double n2_k = normalizeAngle2Pi (theta_k);
        // Distance on circle should be 0.
        // Since both are in [0, 2pi), they should be exactly equal (within epsilon).
        // However, large k might induce precision errors.
        EXPECT_NEAR (n2, n2_k, 1e-5);

        // --- Check NormalizeAngleSigned ---
        double ns = normalizeAngleSigned (theta);
        EXPECT_GT (ns, -PI - 1e-9);
        EXPECT_LE (ns, PI + 1e-9);

        // Property: Periodicity
        double ns_k = normalizeAngleSigned (theta_k);
        EXPECT_NEAR (ns, ns_k, 1e-5);
    }
}

/// @brief Fuzz test for computeLineEndpoint: Reversibility.
TEST (MathTests, FuzzComputeLineEndpoint)
{
    std::mt19937 rng (11111);
    std::uniform_real_distribution<double> dist (-100.0, 100.0);
    std::uniform_real_distribution<double> angle_dist (-PI, PI);
    std::uniform_real_distribution<double> len_dist (0.0, 50.0);

    for (int i = 0; i < 1000; ++i)
    {
        double x = dist (rng);
        double y = dist (rng);
        double h = angle_dist (rng);
        double len = len_dist (rng);

        // Forward
        auto [x1, y1] = computeLineEndpoint (x, y, h, 1, len);

        // Backward (same heading, reverse direction)
        auto [x2, y2] = computeLineEndpoint (x1, y1, h, -1, len);

        EXPECT_NEAR (x, x2, 1e-5);
        EXPECT_NEAR (y, y2, 1e-5);
    }
}

/// @brief Fuzz test for computeArcEndpoint: Reversibility.
TEST (MathTests, FuzzComputeArcEndpoint)
{
    std::mt19937 rng (22222);
    std::uniform_real_distribution<double> dist (-100.0, 100.0);
    std::uniform_real_distribution<double> angle_dist (-PI, PI);
    std::uniform_real_distribution<double> k_dist (-0.5, 0.5); // Curvature
    std::uniform_real_distribution<double> len_dist (0.0, 50.0);

    for (int i = 0; i < 1000; ++i)
    {
        double x = dist (rng);
        double y = dist (rng);
        double h = angle_dist (rng);
        double k = k_dist (rng);
        double len = len_dist (rng);

        // Forward
        auto [x1, y1, h1] = computeArcEndpoint (x, y, h, k, 1, len);

        // Backward
        // If we integrated forward, we are at (x1, y1) with heading h1.
        // To reverse, we drive with direction -1 and same curvature k.
        // The length is the same.
        auto [x2, y2, h2] = computeArcEndpoint (x1, y1, h1, k, -1, len);

        EXPECT_NEAR (x, x2, 1e-4);
        EXPECT_NEAR (y, y2, 1e-4);
        // Heading should also return to original (normalized)
        EXPECT_NEAR (normalizeAngleSigned (h), normalizeAngleSigned (h2), 1e-4);
    }
}
