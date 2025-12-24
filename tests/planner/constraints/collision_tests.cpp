/**
 * @file collision_tests.cpp
 * @brief Unit tests for CollisionConstraint (Point-Robot).
 */

#include <arcgen/core/state.hpp>
#include <arcgen/planner/constraints/collision.hpp>
#include <arcgen/planner/geometry/workspace.hpp>
#include <arcgen/steering/path.hpp>

#include <boost/geometry/io/wkt/wkt.hpp>
#include <gtest/gtest.h>

using namespace arcgen::planner::constraints;
using namespace arcgen::planner::geometry;
using namespace arcgen::core;
using namespace arcgen::steering;

/// @brief Test fixture for point-based collision checking.
class CollisionConstraintFixture : public ::testing::Test
{
  protected:
    static constexpr std::size_t N = 3;
    using Constraint = CollisionConstraint<N>;
    using Context = EvalContext<N>;
    using PathType = Path<N>;

    std::shared_ptr<Workspace> ws_;
    std::unique_ptr<Constraint> constraint_;
    State dummyState_{0, 0, 0};

    void SetUp () override
    {
        // 10x10 Box with a hole at (5,5)
        Polygon outer;
        boost::geometry::read_wkt ("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0))", outer);

        std::vector<Polygon> obstacles;
        Polygon obs;
        boost::geometry::read_wkt ("POLYGON((4 4, 6 4, 6 6, 4 6, 4 4))", obs);
        obstacles.push_back (obs);

        ws_ = std::make_shared<Workspace> (outer, obstacles);
        constraint_ = std::make_unique<Constraint> (ws_);
    }

    Context makeContext (const std::vector<State> &statesToInject)
    {
        return Context{dummyState_, dummyState_, [statesToInject] (PathType &p) { p.states = statesToInject; }};
    }
};

/// @brief Verify valid path inside free space.
TEST_F (CollisionConstraintFixture, ValidPath)
{
    // Path: (1,1) -> (2,2) -> (3,3)
    // All inside
    auto ctx = makeContext ({{1, 1, 0}, {2, 2, 0}, {3, 3, 0}});
    PathType p;
    EXPECT_TRUE (constraint_->accept (p, ctx));
    // p.states should be populated now
    ASSERT_TRUE (p.states.has_value ());
    EXPECT_EQ (p.states->size (), 3u);
}

/// @brief Verify rejection of path going through an obstacle hole.
TEST_F (CollisionConstraintFixture, InvalidPathViaObstacle)
{
    // Path: (1,1) -> (5,5) -> (9,9)
    // (5,5) is in the hole [4,6]x[4,6]
    auto ctx = makeContext ({{1, 1, 0}, {5, 5, 0}, {9, 9, 0}});
    PathType p;
    EXPECT_FALSE (constraint_->accept (p, ctx));
}

/// @brief Verify rejection of path outside workspace boundary.
TEST_F (CollisionConstraintFixture, InvalidPathViaBoundary)
{
    // Path: (1,1) -> (-1, -1)
    // (-1, -1) is outside
    auto ctx = makeContext ({{1, 1, 0}, {-1, -1, 0}});
    PathType p;
    EXPECT_FALSE (constraint_->accept (p, ctx));
}

/// @brief Verify empty workspace accepts all paths.
TEST_F (CollisionConstraintFixture, EmptyWorkspace)
{
    // Workspace logic: if (obstaclesUnion.empty() && !bg::is_empty(outer))...
    // Actually if workspace pointer passed to constraint is null or empty, it should accept all.
    auto emptyWs = std::make_shared<Workspace> (Polygon{});
    Constraint permissive (emptyWs);

    auto ctx = makeContext ({{-100, -100, 0}});
    PathType p;
    EXPECT_TRUE (permissive.accept (p, ctx));
}

/// @brief Verify null workspace pointer is handled gracefully.
TEST_F (CollisionConstraintFixture, NullWorkspace)
{
    Constraint permissive (nullptr);
    auto ctx = makeContext ({{-100, -100, 0}});
    PathType p;
    EXPECT_TRUE (permissive.accept (p, ctx));
}

/// @brief Verify single point queries inside/outside.
TEST_F (CollisionConstraintFixture, SinglePoint)
{
    // Inside
    {
        auto ctx = makeContext ({{2, 2, 0}});
        PathType p;
        EXPECT_TRUE (constraint_->accept (p, ctx));
    }
    // Outside
    {
        auto ctx = makeContext ({{-5, -5, 0}});
        PathType p;
        EXPECT_FALSE (constraint_->accept (p, ctx));
    }
}

/// @brief Verify boundary condition tolerance.
TEST_F (CollisionConstraintFixture, BoundaryTolerance)
{
    // Obstacle at x=4.
    // 3.999 is safe (in 0..10 box, outside 4..6 obs)
    {
        auto ctx = makeContext ({{3.999, 5, 0}});
        PathType p;
        EXPECT_TRUE (constraint_->accept (p, ctx));
    }
    // 4.001 is invalid (inside 4..6 obs)
    {
        auto ctx = makeContext ({{4.001, 5, 0}});
        PathType p;
        EXPECT_FALSE (constraint_->accept (p, ctx));
    }
}
