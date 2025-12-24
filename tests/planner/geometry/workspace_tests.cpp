/**
 * @file workspace_tests.cpp
 * @brief Unit tests for Workspace class (polygon operations, containment, clipping).
 */

#include <arcgen/planner/geometry/workspace.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>
#include <gtest/gtest.h>

using namespace arcgen::planner::geometry;

namespace bg = boost::geometry;

/// @brief Test fixture for Workspace geometry operations.
class WorkspaceFixture : public ::testing::Test
{
  protected:
    // Helper to create a polygon from WKT
    static Polygon fromWKT (const std::string &wkt)
    {
        Polygon p;
        bg::read_wkt (wkt, p);
        bg::correct (p);
        return p;
    }

    // 10x10 square: (0,0) to (10,10)
    Polygon outer_ = fromWKT ("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0))");
};

TEST_F (WorkspaceFixture, ConstructionNoObstacles)
{
    Workspace ws (outer_);

    EXPECT_FALSE (ws.empty ());
    // Center point
    EXPECT_TRUE (ws.contains (5.0, 5.0));
    // Outside
    EXPECT_FALSE (ws.contains (-1.0, 5.0));
    EXPECT_FALSE (ws.contains (11.0, 5.0));
}

TEST_F (WorkspaceFixture, ConstructionWithObstacles)
{
    // Hole in the middle: 4x4 box at (3,3)
    std::vector<Polygon> obstacles;
    obstacles.push_back (fromWKT ("POLYGON((3 3, 7 3, 7 7, 3 7, 3 3))"));

    Workspace ws (outer_, obstacles);

    EXPECT_FALSE (ws.empty ());

    // In valid region
    EXPECT_TRUE (ws.contains (1.0, 1.0));

    // Inside obstacle
    EXPECT_FALSE (ws.contains (5.0, 5.0));

    // Outside outer boundary
    EXPECT_FALSE (ws.contains (12.0, 5.0));
}

TEST_F (WorkspaceFixture, PathQuery)
{
    std::vector<Polygon> obstacles;
    obstacles.push_back (fromWKT ("POLYGON((3 3, 7 3, 7 7, 3 7, 3 3))"));
    Workspace ws (outer_, obstacles);

    // Valid path
    std::vector<arcgen::core::State> path1;
    path1.push_back ({1.0, 1.0, 0, 0, arcgen::core::DrivingDirection::Forward});
    path1.push_back ({2.0, 2.0, 0, 0, arcgen::core::DrivingDirection::Forward});
    EXPECT_TRUE (ws.contains (path1));

    // Invalid path (passes through obstacle) - Note: contains(path) only checks waypoints!
    std::vector<arcgen::core::State> path2;
    path2.push_back ({1.0, 1.0, 0, 0, arcgen::core::DrivingDirection::Forward});
    path2.push_back ({5.0, 5.0, 0, 0, arcgen::core::DrivingDirection::Forward}); // Inside obstacle
    EXPECT_FALSE (ws.contains (path2));
}

TEST_F (WorkspaceFixture, CoveredBy)
{
    Workspace ws (outer_);

    Polygon smallBox = fromWKT ("POLYGON((1 1, 2 1, 2 2, 1 2, 1 1))");
    EXPECT_TRUE (ws.coveredBy (smallBox));

    Polygon crossingBox = fromWKT ("POLYGON((9 9, 11 9, 11 11, 9 11, 9 9))");
    EXPECT_FALSE (ws.coveredBy (crossingBox));
}

TEST_F (WorkspaceFixture, ClippedTo)
{
    Workspace ws (outer_);
    // Clip with right half: x >= 5
    Polygon clipBox = fromWKT ("POLYGON((5 0, 15 0, 15 10, 5 10, 5 0))");

    Workspace clipped = ws.clippedTo (clipBox);

    EXPECT_FALSE (clipped.empty ());
    EXPECT_FALSE (clipped.contains (1.0, 5.0));  // Left half gone
    EXPECT_TRUE (clipped.contains (6.0, 5.0));   // Right half stays
    EXPECT_FALSE (clipped.contains (12.0, 5.0)); // Outside original outer
}

TEST_F (WorkspaceFixture, BoundaryExclusion)
{
    Workspace ws (outer_);

    // On the line
    EXPECT_FALSE (ws.contains (0.0, 5.0));
    EXPECT_FALSE (ws.contains (10.0, 5.0));

    // Corners
    EXPECT_FALSE (ws.contains (0.0, 0.0));
    EXPECT_FALSE (ws.contains (10.0, 10.0));

    // Just inside epsilon
    EXPECT_TRUE (ws.contains (1e-6, 5.0));
}

TEST_F (WorkspaceFixture, OverlappingObstacles)
{
    std::vector<Polygon> obs;
    // Box 1: [2, 6] x [2, 6]
    obs.push_back (fromWKT ("POLYGON((2 2, 6 2, 6 6, 2 6, 2 2))"));
    // Box 2: [4, 8] x [4, 8]  -> Overlap is [4, 6] x [4, 6]
    obs.push_back (fromWKT ("POLYGON((4 4, 8 4, 8 8, 4 8, 4 4))"));

    Workspace ws (outer_, obs);

    // Inside Box 1 only
    EXPECT_FALSE (ws.contains (3.0, 3.0));
    // Inside Box 2 only
    EXPECT_FALSE (ws.contains (7.0, 7.0));
    // Inside Overlap
    EXPECT_FALSE (ws.contains (5.0, 5.0));

    // Valid region outside
    EXPECT_TRUE (ws.contains (1.0, 1.0));
    EXPECT_TRUE (ws.contains (9.0, 9.0));
}

TEST_F (WorkspaceFixture, CoincidentBoundary)
{
    std::vector<Polygon> obs;
    // Obstacle touching the left edge: [0, 2] x [2, 8]
    obs.push_back (fromWKT ("POLYGON((0 2, 2 2, 2 8, 0 8, 0 2))"));

    Workspace ws (outer_, obs);

    // Inside obstacle
    EXPECT_FALSE (ws.contains (1.0, 5.0));
    // Free space
    EXPECT_TRUE (ws.contains (5.0, 5.0));
    // Check seamlessness of boundary? (Already false on boundary)
    EXPECT_FALSE (ws.contains (0.0, 5.0));
}

TEST_F (WorkspaceFixture, FullyBlocked)
{
    // Obstacle covering everything
    std::vector<Polygon> obs;
    obs.push_back (fromWKT ("POLYGON((-1 -1, 11 -1, 11 11, -1 11, -1 -1))"));

    Workspace ws (outer_, obs);
    EXPECT_TRUE (ws.empty ());
}
