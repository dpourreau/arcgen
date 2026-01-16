/**
 * @file
 * @brief Unit tests for arcgen::planner::geometry::Robot – SE(2) transforms,
 *        workspace coverage, and plotting helpers.
 */
#include <arcgen.hpp>

#include <utils/output_paths.hpp>
#include <utils/visualizer.hpp>
#include <utils/workspace_generators.hpp>

#include <boost/geometry.hpp>
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

using namespace arcgen::planner::geometry;
using namespace arcgen::core;

namespace bg = boost::geometry;

/**
 * @brief Fixture collecting timing stats for Robot APIs.
 */
class RobotFixture : public ::testing::Test
{
  protected:
};

/**
 * @brief Build a convex triangular wedge footprint pointing toward +X.
 *
 * Reference point is at (0,0) in the body frame and the longitudinal
 * axis points along +X, matching the library's convention.
 */
static Robot makeWedge (double length, double width)
{
    Polygon polygon;
    const double halfLength = 0.5 * length;
    const double halfWidth = 0.5 * width;
    // Triangle: rear-left, tip, rear-right, closed
    polygon.outer () = {{-halfLength, -halfWidth}, {halfLength, 0.0}, {-halfLength, halfWidth}, {-halfLength, -halfWidth}};
    bg::correct (polygon);
    return Robot{std::move (polygon)};
}

/**
 * @test Robot on an ellipse with tangential heading. Validates transform parity,
 *       workspace coverage per placement, area invariance, and optionally plots
 *       per-robot shape when AG_ENABLE_PLOTS is defined.
 */
TEST_F (RobotFixture, Ellipse)
{
    // Workspace large enough to fully contain ellipse + robot footprint
    Polygon outer;
    outer.outer () = {{-20, -15}, {20, -15}, {20, 15}, {-20, 15}, {-20, -15}};
    bg::correct (outer);
    Workspace workspace (std::move (outer), {});

    // Robots with different shapes/anchors
    const double length = 4.0;
    const double width = 2.0;
    std::vector<std::pair<std::string, Robot>> robots;
    robots.reserve (3);
    robots.emplace_back ("rect_center", Robot::rectangle (length, width));
    robots.emplace_back ("rect_rear", Robot::rectangle (length, width, -length * 0.5, 0.0)); // reference at rear-center
    robots.emplace_back ("wedge", makeWedge (length, width));

    // Ellipse parameters and tangential heading
    const double semiMajor = 8.0;
    const double semiMinor = 5.0;
    std::vector<State> states;
    const int sampleCount = 160;
    states.reserve (static_cast<size_t> (sampleCount));
    for (int k = 0; k < sampleCount; ++k)
    {
        const double t = 2.0 * PI * (static_cast<double> (k) / static_cast<double> (sampleCount));
        const double x = semiMajor * std::cos (t);
        const double y = semiMinor * std::sin (t);
        // Tangent direction: derivative (dx/dt, dy/dt) = (-a sin t, b cos t)
        const double dxDt = -semiMajor * std::sin (t);
        const double dyDt = semiMinor * std::cos (t);
        const double heading = std::atan2 (dyDt, dxDt);
        states.push_back (State{x, y, heading});
    }

    // Evaluate all robots on the same ellipse path
    for (const auto &[name, robot] : robots)
    {
        // 1) Transform API parity + timing
        std::vector<Polygon> poses;
        poses.reserve (states.size ());
        {
            for (const auto &s : states)
                poses.push_back (robot.at (s));
            for (std::size_t i = 0; i < states.size (); ++i)
            {
                Polygon pB = robot.at (states[i].x, states[i].y, states[i].heading);
                EXPECT_TRUE (bg::equals (poses[i], pB));
            }

            // 2) Swept coverage: every placement polygon covered by workspace
            {
                bool allCovered = true;
                for (const auto &s : states)
                    allCovered = allCovered && workspace.coveredBy (robot.at (s));
                EXPECT_TRUE (allCovered);
            }

            // 3) Area preservation sanity: spot-check a handful of placements
            const double areaBody = std::fabs (bg::area (robot.body ()));
            for (int i : {0, sampleCount / 8, sampleCount / 4, (3 * sampleCount) / 8, sampleCount / 2})
            {
                Polygon poly = robot.at (states[static_cast<size_t> (i)]);
                EXPECT_NEAR (std::fabs (bg::area (poly)), areaBody, 1e-12);
            }

#ifdef AG_ENABLE_PLOTS
            // Plot workspace (green), ellipse, robot outlines, and red points
            // Plot workspace (green), ellipse, robot outlines, and red points
            auto outPath = arcgen::utils::plotFile ({"robot", "ellipse"}, std::string ("ellipse_") + name + ".svg");
            arcgen::utils::Visualizer svg (outPath.string (), 900);
            const auto &pal = svg.getPalette ();
            svg.drawRegion (workspace);
            svg.drawPath (states);

            for (size_t i = 0; i < states.size (); i += 12)
            {
                svg.drawPolygon (robot.at (states[i]), "none", pal.robotFill, 1.0, 1.0);
                svg.drawPolygon (arcgen::utils::circlePoly (states[i].x, states[i].y, 0.1, 16), "#ff0000", "none", 0.0, 1.0);
            }

            svg.drawAxes ();
            svg.finish ();
#endif
        }

        // per-robot plotting handled inside the loop above
    }
}

/**
 * @test Two half-turns: forward right then reverse left, forming a "V" shape.
 *       Validates Robot transforms, per-placement coverage, and provides plotting.
 */
TEST_F (RobotFixture, HalfTurnVShape)
{
    // Workspace large enough for both semicircles and the robot footprint
    Polygon outer;
    outer.outer () = {{-20, -12}, {20, -12}, {20, 12}, {-20, 12}, {-20, -12}};
    bg::correct (outer);
    Workspace workspace (std::move (outer), {});

    // Robots with different shapes/anchors
    const double length = 3.5;
    const double width = 1.4;
    std::vector<std::pair<std::string, Robot>> robots;
    robots.reserve (3);
    robots.emplace_back ("rect_center", Robot::rectangle (length, width));
    robots.emplace_back ("rect_rear", Robot::rectangle (length, width, -length * 0.5, 0.0));
    robots.emplace_back ("wedge", makeWedge (length, width));

    // Two semicircles of radius r with mirrored centers at (−r,0) and (+r,0),
    // meeting at the apex (0,0) to form a "V".
    // 1) Forward, turning right around C1 (−r,0): t from pi → 0 (clockwise, upper half)
    // 2) Reverse, turning left around C2 (+r,0): t from pi → 2π (counter-clockwise, upper half but reversed)
    const double radius = 6.0;
    const double centerX1 = -radius, centerY1 = 0.0;
    const double centerX2 = +radius, centerY2 = 0.0;

    std::vector<State> states;
    const int sampleCount = 180; // samples across both halves
    states.reserve (static_cast<std::size_t> (sampleCount + 1));

    // First half: forward-right (clockwise, t: pi -> 0) around C1
    for (int k = 0; k <= sampleCount / 2; ++k)
    {
        const double t = PI - (PI * static_cast<double> (k) / static_cast<double> (sampleCount / 2)); // pi -> 0
        const double x = centerX1 + radius * std::cos (t);
        const double y = centerY1 + radius * std::sin (t);
        const double heading = t - PI / 2.0; // clockwise tangent (forward)
        states.push_back (State{x, y, heading});
    }

    // Second half: reverse-left around C2; traverse the UPPER semicircle pi → 0 (clockwise geometry),
    // and flip the tangent by π to represent reverse.
    for (int k = 1; k <= sampleCount / 2; ++k) // start at k=1 to avoid duplicating apex
    {
        const double t = PI - (PI * static_cast<double> (k) / static_cast<double> (sampleCount / 2)); // pi -> 0
        const double x = centerX2 + radius * std::cos (t);
        const double y = centerY2 + radius * std::sin (t);
        const double heading = t + PI / 2.0; // reverse of (t - PI/2)
        states.push_back (State{x, y, heading});
    }

    for (const auto &[name, robot] : robots)
    {
        // 1) Transform API parity + timing
        std::vector<Polygon> placements;
        placements.reserve (states.size ());
        {
            for (const auto &s : states)
                placements.push_back (robot.at (s));
            for (std::size_t i = 0; i < states.size (); ++i)
            {
                Polygon q = robot.at (states[i].x, states[i].y, states[i].heading);
                EXPECT_TRUE (bg::equals (placements[i], q));
            }

            // 2) Swept coverage: every placement polygon covered by workspace
            {
                bool allCovered = true;
                for (const auto &s : states)
                    allCovered = allCovered && workspace.coveredBy (robot.at (s));
                EXPECT_TRUE (allCovered);

#ifdef AG_ENABLE_PLOTS
                // Plot per-shape
                // Plot per-shape
                auto outPath = arcgen::utils::plotFile ({"robot", "vshape"}, std::string ("vshape_") + name + ".svg");
                arcgen::utils::Visualizer svg (outPath.string (), 900);
                const auto &pal = svg.getPalette ();
                svg.drawRegion (workspace);
                svg.drawPath (states);

                for (size_t i = 0; i < states.size (); i += 10)
                {
                    svg.drawPolygon (robot.at (states[i]), "none", pal.robotFill, 1.0, 1.0);
                    svg.drawPolygon (arcgen::utils::circlePoly (states[i].x, states[i].y, 0.1, 16), "#ff0000", "none", 0.0, 1.0);
                }

                svg.drawAxes ();
                svg.finish ();
#endif
            }
        }
    }
}
