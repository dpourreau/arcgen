/**
 * @file footprint_collision_tests.cpp
 * @brief Unit tests for FootprintCollisionConstraint (Polygon-Robot).
 */

#include <arcgen/core/state.hpp>
#include <arcgen/planner/constraints/footprint_collision.hpp>
#include <arcgen/planner/geometry/robot.hpp>
#include <arcgen/planner/geometry/workspace.hpp>
#include <arcgen/steering/path.hpp>

#include <boost/geometry/io/wkt/wkt.hpp>
#include <gtest/gtest.h>

using namespace arcgen::planner::constraints;
using namespace arcgen::planner::geometry;
using namespace arcgen::core;
using namespace arcgen::steering;

/// @brief Test fixture for footprint-based collision checking.
class FootprintCollisionFixture : public ::testing::Test
{
  protected:
    static constexpr std::size_t N = 3;
    using Constraint = FootprintCollisionConstraint<N>;
    using Context = EvalContext<N>;
    using PathType = Path<N>;

    std::shared_ptr<Workspace> ws_;
    Robot robot_; // We need to construct this.
    std::unique_ptr<Constraint> constraint_;
    State dummyState_{0, 0, 0};

    // Helper to make a square robot 1x1 centered at (0,0)?
    // Or maybe 2x2.
    // Let's make a 1x1 square: (-0.5, -0.5) to (0.5, 0.5)
    Robot makeBoxRobot ()
    {
        Polygon poly;
        boost::geometry::read_wkt ("POLYGON((-0.5 -0.5, 0.5 -0.5, 0.5 0.5, -0.5 0.5, -0.5 -0.5))", poly);
        return Robot (poly);
    }

    void SetUp () override
    {
        // 10x10 Box with an obstacle at (5,5) size 2x2: [4,6]x[4,6]
        Polygon outer;
        boost::geometry::read_wkt ("POLYGON((0 0, 10 0, 10 10, 0 10, 0 0))", outer);

        std::vector<Polygon> obstacles;
        Polygon obs;
        boost::geometry::read_wkt ("POLYGON((4 4, 6 4, 6 6, 4 6, 4 4))", obs);
        obstacles.push_back (obs);

        ws_ = std::make_shared<Workspace> (outer, obstacles);
        robot_ = makeBoxRobot ();
        constraint_ = std::make_unique<Constraint> (ws_, robot_);
    }

    Context makeContext (const std::vector<State> &statesToInject)
    {
        return Context{dummyState_, dummyState_, [statesToInject] (const PathType &p) { p.states = statesToInject; }};
    }
};

/// @brief Verify valid footprint state.
TEST_F (FootprintCollisionFixture, SimplePass)
{
    // Robot at (2,2) - footprint is [1.5, 2.5]x[1.5, 2.5]
    // Fully inside 10x10, far from 4x4 obstacle.
    auto ctx = makeContext ({{2, 2, 0}});
    PathType p;
    EXPECT_TRUE (constraint_->accept (p, ctx));
}

/// @brief Verify collision when footprint overlaps obstacle.
TEST_F (FootprintCollisionFixture, SimpleFail)
{
    // Robot at (5,5) - center of obstacle.
    auto ctx = makeContext ({{5, 5, 0}});
    PathType p;
    EXPECT_FALSE (constraint_->accept (p, ctx));
}

/// @brief Verify optimization for straight-line segments.
TEST_F (FootprintCollisionFixture, StraightOptimization)
{
    // Path: (1,1) -> (2,2) -> (3,3)
    // Curvature 0.
    // Hull of (1,1) footprint and (3,3) footprint.
    // Box at (1,1) is [0.5, 1.5]. Box at (3,3) is [2.5, 3.5].
    // Hull stretches from 0.5 to 3.5.
    // Obstacle is at [4,6]. Safe.

    // We mark states as straight (k=0)
    std::vector<State> states = {{1, 1, 0, 0}, {2, 2, 0, 0}, {3, 3, 0, 0}};

    auto ctx = makeContext (states);
    PathType p;
    EXPECT_TRUE (constraint_->accept (p, ctx));
}

/// @brief Verify collision detection on straight segments passing through obstacles.
TEST_F (FootprintCollisionFixture, StraightPathWithCollision)
{
    // Path from (2,5) to (8,5).
    // Straight line goes through (5,5) which is obstacle.
    std::vector<State> states = {{2, 5, 0, 0},
                                 {5, 5, 0, 0}, // Colliding state
                                 {8, 5, 0, 0}};

    auto ctx = makeContext (states);
    PathType p;
    EXPECT_FALSE (constraint_->accept (p, ctx));
}

/// @brief Verify collision checks on curved segments (k != 0).
TEST_F (FootprintCollisionFixture, CurvedCheck)
{
    // State (4.5, 3.8) with curvature 1.0 (so not straight).
    // Box: x[4.0, 5.0], y[3.3, 4.3].
    // Obstacle: x[4,6], y[4,6].
    // Overlap in x=[4,5], y=[4, 4.3]. Collision.

    auto ctx = makeContext ({{4.5, 3.8, 0, 1.0}});
    PathType p;
    EXPECT_FALSE (constraint_->accept (p, ctx));
}

/// @brief Verify robustness to null workspace.
TEST_F (FootprintCollisionFixture, NullWorkspace)
{
    Constraint permissive (nullptr, robot_);
    auto ctx = makeContext ({{5, 5, 0}}); // Typically collision
    PathType p;
    EXPECT_TRUE (permissive.accept (p, ctx));
}

/// @brief Verify acceptance of empty paths.
TEST_F (FootprintCollisionFixture, EmptyPath)
{
    auto ctx = makeContext ({});
    PathType p;
    EXPECT_TRUE (constraint_->accept (p, ctx));
}

/// @brief Verify mixed straight and curved segments.
TEST_F (FootprintCollisionFixture, MixedSegments)
{
    // Path: Straight (Safe) -> Curved (Collision) -> Straight (Safe)
    std::vector<State> states = {{1, 1, 0, 0},
                                 {2, 2, 0, 0},
                                 {5, 5, 0, 1.0}, // Collision here, must be checked despite neighbors being straight
                                 {8, 8, 0, 0},
                                 {9, 9, 0, 0}};

    auto ctx = makeContext (states);
    PathType p;
    EXPECT_FALSE (constraint_->accept (p, ctx));
}
