#include <arcgen/planner/graph/astar.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <gtest/gtest.h>

using arcgen::planner::graph::AStar;

namespace
{
    struct TestPoint
    {
        double x_val, y_val;
        double x () const { return x_val; }
        double y () const { return y_val; }
    };

    using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, TestPoint, boost::property<boost::edge_weight_t, double>>;
} // namespace

class AStarFixture : public ::testing::Test
{
  protected:
    AStar<Graph> searcher_;
    Graph g_;

    void addVertex (double x, double y) { boost::add_vertex (TestPoint{x, y}, g_); }

    void addEdge (std::size_t u, std::size_t v, double weight) { boost::add_edge (u, v, weight, g_); }
};

TEST_F (AStarFixture, EmptyGraph)
{
    // 0 vertices
    auto path = searcher_.search (g_, {0, 0, 0}, {10, 10, 0});
    EXPECT_TRUE (path.empty ());
}

TEST_F (AStarFixture, SingleVertex)
{
    addVertex (0, 0); // v0
    // Start/Goal both snap to v0.
    // vStart == vGoal -> Path should be empty (no intermediates)
    auto path = searcher_.search (g_, {1, 1, 0}, {-1, -1, 0});
    EXPECT_TRUE (path.empty ());
}

TEST_F (AStarFixture, SimplePath)
{
    // A(0,0) -- B(5,0) -- C(10,0)
    addVertex (0, 0);  // 0
    addVertex (5, 0);  // 1
    addVertex (10, 0); // 2

    addEdge (0, 1, 5.0);
    addEdge (1, 2, 5.0);

    // Search from near A to near C
    auto path = searcher_.search (g_, {-1, 0, 0}, {11, 0, 0});

    // Expect intermediate B(5,0)
    ASSERT_EQ (path.size (), 1u);
    EXPECT_NEAR (path[0].x, 5.0, 1e-6);
    EXPECT_NEAR (path[0].y, 0.0, 1e-6);
}

TEST_F (AStarFixture, BranchingPath)
{
    //      B(5, 5)
    //     /       \ 
    // A(0,0)       D(10,0)
    //     \       /
    //      C(5,-1)

    addVertex (0, 0);  // 0: A
    addVertex (5, 5);  // 1: B
    addVertex (5, -1); // 2: C
    addVertex (10, 0); // 3: D

    // A-B-D: 5*sqrt(2) + 5*sqrt(2) approx 14.14
    addEdge (0, 1, 7.07);
    addEdge (1, 3, 7.07);

    // A-C-D: sqrt(26) + sqrt(26) approx 5.1 + 5.1 = 10.2
    addEdge (0, 2, 5.1);
    addEdge (2, 3, 5.1);

    auto path = searcher_.search (g_, {0, 0, 0}, {10, 0, 0});

    // Should prefer C as intermediate
    ASSERT_EQ (path.size (), 1u);
    EXPECT_NEAR (path[0].x, 5.0, 1e-6);
    EXPECT_NEAR (path[0].y, -1.0, 1e-6);
}

TEST_F (AStarFixture, Disconnected)
{
    addVertex (0, 0);  // 0
    addVertex (10, 0); // 1
    // No edge

    auto path = searcher_.search (g_, {0, 0, 0}, {10, 0, 0});
    EXPECT_TRUE (path.empty ());
}

TEST_F (AStarFixture, Unreachable)
{
    addVertex (0, 0);  // 0
    addVertex (10, 0); // 1
    // Infinite weight edge? Or just no edge logic is same as disconnected.
    // Boost A* will explore accessible component.

    // Let's make a component A-B and C-D. Start at A, Goal at D.
    addVertex (5, 0);  // 2
    addVertex (15, 0); // 3

    addEdge (0, 2, 5.0); // 0-2
    addEdge (1, 3, 5.0); // 1-3

    auto path = searcher_.search (g_, {0, 0, 0}, {15, 0, 0}); // 0 -> 3
    EXPECT_TRUE (path.empty ());
}

TEST_F (AStarFixture, DirectNeighbor)
{
    // A(0,0) -- B(10,0)
    addVertex (0, 0);
    addVertex (10, 0);
    addEdge (0, 1, 10.0);

    // Search A -> B
    // Path exists, but 0 intermediates.
    auto path = searcher_.search (g_, {0, 0, 0}, {10, 0, 0});
    EXPECT_TRUE (path.empty ());
}

TEST_F (AStarFixture, NaNInput)
{
    addVertex (0, 0);
    addVertex (10, 0);
    addEdge (0, 1, 10.0);

    double nan = std::numeric_limits<double>::quiet_NaN ();
    // Start is NaN -> nearestVertex fails -> returns empty
    auto path = searcher_.search (g_, {nan, 0, 0}, {10, 0, 0});
    EXPECT_TRUE (path.empty ());
}

TEST_F (AStarFixture, Grid10x10)
{
    // Create 10x10 grid with integer coordinates (0..9, 0..9)
    // Vertex index i = y*10 + x.
    for (int y = 0; y < 10; ++y)
    {
        for (int x = 0; x < 10; ++x)
        {
            addVertex (static_cast<double> (x), static_cast<double> (y));
        }
    }

    // Add 4-connected edges with weight 1.0
    for (int y = 0; y < 10; ++y)
    {
        for (int x = 0; x < 10; ++x)
        {
            std::size_t u = static_cast<std::size_t> (y * 10 + x);
            // Right neighbor
            if (x < 9)
            {
                std::size_t v = static_cast<std::size_t> (y * 10 + (x + 1));
                addEdge (u, v, 1.0);
            }
            // Up neighbor
            if (y < 9)
            {
                std::size_t v = static_cast<std::size_t> ((y + 1) * 10 + x);
                addEdge (u, v, 1.0);
            }
        }
    }

    // Search from (0,0) to (9,9)
    // Coords are exact match to vertices.
    auto path = searcher_.search (g_, {0, 0, 0}, {9, 9, 0});

    // Shortest path length is 18 edges.
    // The path contains intermediate vertices only.
    // Total vertices in path = 19 (inclusive).
    // Intermediates = 17.
    EXPECT_EQ (path.size (), 17u);

    if (!path.empty ())
    {
        // Validate first step is adjacent to start (0,0)
        double d0 = std::abs (path.front ().x - 0.0) + std::abs (path.front ().y - 0.0);
        EXPECT_NEAR (d0, 1.0, 1e-6);

        // Validate last step is adjacent to goal (9,9)
        double dN = std::abs (path.back ().x - 9.0) + std::abs (path.back ().y - 9.0);
        EXPECT_NEAR (dN, 1.0, 1e-6);
    }
}
