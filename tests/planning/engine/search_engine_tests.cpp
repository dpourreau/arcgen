/**
 * @file search_engine_tests.cpp
 * @brief End-to-end planning tests that plot and validate paths produced by SearchEngine.
 */

#include <arcgen.hpp>

#include <common/plot_dir.hpp>
#include <common/pose_sampling.hpp>
#include <common/test_stats.hpp>
#include <common/visualizer.hpp>
#include <common/workspace_generators.hpp>

#include <boost/geometry.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <random>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

using namespace arcgen::core;
using namespace arcgen::geometry;
using namespace arcgen::steering;
using namespace arcgen::planning;
using arcgen::planning::search::AStar;
namespace connector = arcgen::planning::engine::connector;
using test_helpers::RunningStats;
using test_helpers::ScopedTimer;
using test_helpers::Visualizer;

namespace bg = boost::geometry;

/* ───────── configuration ───────── */
constexpr double R_MIN = 1.5;     ///< [m]
constexpr double STEP = 0.02;     ///< [m] steering discretisation
constexpr double EPS_GOAL = 0.01; ///< [m] goal proximity
constexpr int SAMPLES = 50;       ///< cases per workspace kind

// Robot footprint used for footprint-aware tests
constexpr double ROBOT_LEN = 1.3; ///< [m] rectangle length
constexpr double ROBOT_WID = 0.7; ///< [m] rectangle width

/* ───────── helpers ───────── */

/// @brief Square helper.
template <typename T> [[nodiscard]] constexpr double sqr (T v) noexcept
{
    return double (v) * double (v);
}

/// @brief Euclidean distance between two states (2D).
[[nodiscard]] static double euclidean (const State &a, const State &b) noexcept
{
    return std::sqrt (sqr (a.x - b.x) + sqr (a.y - b.y));
}

/* ───────── engine-config triple for extensibility ───────── */

/// @brief Bundle types for typed tests.
template <class SteeringT, class SearchT, class SkeletonT> struct EngineConfig
{
    using Steering = SteeringT;
    using Search = SearchT;
    using Skeleton = SkeletonT;

    /// @brief Short human-readable name for reporting.
    static constexpr const char *name ()
    {
        if constexpr (std::is_same_v<SteeringT, Dubins>)
            return "dubins";
        if constexpr (std::is_same_v<SteeringT, ReedsShepp>)
            return "reeds_shepp";
        return "generic";
    }
};

/**
 * @brief Typed fixture for end-to-end planning (SearchEngine) with timing & optional SVG plots.
 * @tparam Cfg  EngineConfig triple: Steering, Search, Skeleton.
 */
template <class Cfg> class EngineFixture : public ::testing::Test
{
  protected:
    using Graph = arcgen::geometry::Graph;
    using SteeringT = typename Cfg::Steering;
    using SearchT = typename Cfg::Search;
    using SkeletonT = typename Cfg::Skeleton;
    using EngineT = engine::SearchEngine<SteeringT, SearchT, SkeletonT>;
    using ConnectorT = typename EngineT::ConnectorType;

    EngineFixture () : randomEngine_ (42), unitUniform_ (0.0, 1.0) {}

    /// @brief Print mean planning time (once/type).
    static void TearDownTestSuite () { planStats_.printSummary (std::string (Cfg::name ()) + " – SearchEngine plan()"); }

    /**
     * @brief Pick a random interior (x,y) within the workspace envelope.
     * @param workspace Workspace to sample inside.
     * @param x Sampled X.
     * @param y Sampled Y.
     */
    void sampleInteriorPosition (const Workspace &workspace, double &x, double &y)
    {
        bg::model::box<Point> bb;
        bg::envelope (workspace.region (), bb);

        std::uniform_real_distribution<double> ux (bb.min_corner ().x (), bb.max_corner ().x ());
        std::uniform_real_distribution<double> uy (bb.min_corner ().y (), bb.max_corner ().y ());

        do
        {
            x = ux (randomEngine_);
            y = uy (randomEngine_);
        } while (!workspace.contains (x, y));
    }

    /// @brief Sample heading uniformly in [0, 2π).
    double sampleHeading () { return std::uniform_real_distribution<double> (0.0, PI2) (randomEngine_); }

    /**
     * @brief Ensure the sampled pose meets the “local reachability” rule via half-disks.
     *
     * Rules:
     *  - Dubins:   start requires forward cap inside; goal requires backward cap inside.
     *  - Reeds–Shepp: start/goal each require (forward || backward) cap inside.
     * @param workspace        Workspace.
     * @param s        Pose to validate.
     * @param isStart  True for start, false for goal.
     */
    bool locallyReachable (const Workspace &workspace, const State &s, bool isStart, bool withFootprint) const
    {
        using test_helpers::halfDiskInside;

        double radius = R_MIN * 2; // base local-reachability radius (point robot)

        if (withFootprint)
            radius += (ROBOT_LEN / 2.0);

        if constexpr (std::is_same_v<SteeringT, Dubins>)
        {
            const bool wantForward = isStart; // start→forward, goal→backward
            return halfDiskInside (workspace, s.x, s.y, s.heading, radius, wantForward);
        }
        else
        {
            // Reeds–Shepp: either direction is fine.
            const bool fwd = halfDiskInside (workspace, s.x, s.y, s.heading, radius, true);
            if (fwd)
                return true;
            return halfDiskInside (workspace, s.x, s.y, s.heading, radius, false);
        }
    }

    /**
     * @brief Sample a start pose that is locally reachable per the chosen model.
     * @param workspace Workspace.
     * @param s Output start state.
     */
    void samplePoseStart (const Workspace &workspace, State &s, bool withFootprint = false)
    {
        constexpr int MAX_TRIES = 2000;
        for (int t = 0; t < MAX_TRIES; ++t)
        {
            sampleInteriorPosition (workspace, s.x, s.y);
            s.heading = sampleHeading ();
            s.curvature = 0.0;
            s.direction = DrivingDirection::Neutral;

            if (!locallyReachable (workspace, s, /*isStart=*/true, withFootprint))
                continue;

            if (withFootprint)
            {
                Robot robot = Robot::rectangle (ROBOT_LEN, ROBOT_WID);
                if (!workspace.coveredBy (robot.at (s)))
                    continue;
            }
            return;
        }
        // Fallback: bias heading towards workspace center.
        bg::model::box<Point> bb;
        bg::envelope (workspace.region (), bb);
        const double cx = 0.5 * (bb.min_corner ().x () + bb.max_corner ().x ());
        const double cy = 0.5 * (bb.min_corner ().y () + bb.max_corner ().y ());
        s.heading = std::atan2 (cy - s.y, cx - s.x);
    }

    /**
     * @brief Sample a goal pose that is locally reachable per the chosen model.
     * @param workspace Workspace.
     * @param s Output goal state.
     */
    void samplePoseGoal (const Workspace &workspace, State &s, bool withFootprint = false)
    {
        constexpr int MAX_TRIES = 2000;
        for (int t = 0; t < MAX_TRIES; ++t)
        {
            sampleInteriorPosition (workspace, s.x, s.y);
            s.heading = sampleHeading ();
            s.curvature = 0.0;
            s.direction = DrivingDirection::Neutral;

            if (!locallyReachable (workspace, s, /*isStart=*/false, withFootprint))
                continue;

            if (withFootprint)
            {
                Robot robot = Robot::rectangle (ROBOT_LEN, ROBOT_WID);
                if (!workspace.coveredBy (robot.at (s)))
                    continue;
            }
            return;
        }
        // Fallback: bias heading away from center (so its back faces the center).
        bg::model::box<Point> bb;
        bg::envelope (workspace.region (), bb);
        const double cx = 0.5 * (bb.min_corner ().x () + bb.max_corner ().x ());
        const double cy = 0.5 * (bb.min_corner ().y () + bb.max_corner ().y ());
        const double toCenter = std::atan2 (cy - s.y, cx - s.x);
        s.heading = arcgen::core::normalizeAngleSigned (toCenter + PI); // point outward
    }

    /**
     * @brief Build a fully-configured engine bound to @p workspace.
     * @param workspace Shared workspace.
     */
    std::unique_ptr<EngineT> makeEngine (const std::shared_ptr<Workspace> &workspace)
    {
        auto steering = std::make_shared<SteeringT> (R_MIN, STEP);
        auto search = std::make_shared<SearchT> ();
        auto skeleton = std::make_shared<SkeletonT> ();

        auto connectorPtr = std::make_shared<ConnectorT> ();
        auto eng = std::make_unique<EngineT> (steering, search, skeleton, workspace, connectorPtr);

        namespace C = arcgen::planning::constraints;
        using CSet = C::ConstraintSet<SteeringT::kSegments>;

        CSet cs;
        cs.hard.push_back (std::make_shared<C::CollisionConstraint<SteeringT::kSegments>> (workspace));
        cs.soft.push_back ({std::make_shared<C::PathLengthConstraint<SteeringT::kSegments>> (), 1.0});
        eng->setConstraints (std::move (cs));
        return eng;
    }

    /**
     * @brief Build an engine configured with footprint-aware collision and path length.
     * @param workspace Shared workspace.
     */
    std::unique_ptr<EngineT> makeEngineWithFootprint (const std::shared_ptr<Workspace> &workspace)
    {
        auto steering = std::make_shared<SteeringT> (R_MIN, STEP);
        auto search = std::make_shared<SearchT> ();
        auto skeleton = std::make_shared<SkeletonT> ();

        auto connectorPtr = std::make_shared<ConnectorT> ();
        auto eng = std::make_unique<EngineT> (steering, search, skeleton, workspace, connectorPtr);

        namespace C = arcgen::planning::constraints;
        using CSet = C::ConstraintSet<SteeringT::kSegments>;

        // Choose a modest rectangle footprint (shared constants)
        Robot robot = Robot::rectangle (ROBOT_LEN, ROBOT_WID);

        CSet cs;
        cs.hard.push_back (std::make_shared<C::FootprintCollisionConstraint<SteeringT::kSegments>> (workspace, robot));
        cs.soft.push_back ({std::make_shared<C::PathLengthConstraint<SteeringT::kSegments>> (), 1.0});
        eng->setConstraints (std::move (cs));
        return eng;
    }

    /**
     * @brief Run one plan() on a given workspace (with plotting if enabled).
     *
     * Uses engine getters to access the existing components:
     *  - Region:      engine->getWorkspace()
     *  - Skeleton:    engine->getGlobalGraph() (cached) if present; otherwise
     *                  engine->getSkeleton()->generate(*engine->getWorkspace()) for plotting only.
     *  - No rebuilds of engine components are performed here.
     *
     * @param workspace  Workspace to use.
     * @param id         Case id for file naming.
     * @param label      Group label (used in plot subdirectory).
     */
    void runOneOnWorkspace (const std::shared_ptr<Workspace> &workspace, int id, std::string_view label)
    {
        auto engine = makeEngine (workspace);

        State start{}, goal{};
        samplePoseStart (*workspace, start);
        samplePoseGoal (*workspace, goal);

        typename EngineT::DebugInfo dbg;
        std::vector<State> path;
        {
            ScopedTimer t (planStats_);
            (void)t;
            path = engine->plan (start, goal, &dbg);
        }

        bool ok = true;
        std::ostringstream why;
        if (path.empty ())
        {
            ok = false;
            why << "empty path; ";
        }
        else if (euclidean (path.back (), goal) > EPS_GOAL)
        {
            ok = false;
            why << "goal miss; ";
        }
        if (!path.empty () && !workspace->contains (path))
        {
            ok = false;
            why << "path leaves workspace; ";
        }

#ifdef AG_ENABLE_PLOTS
        const char *tag = ok ? "ok" : "fail";
        std::ostringstream fn;
        fn << tag << '_' << id << ".svg";
        auto outPath = test_helpers::plotFile ({"engine", Cfg::name (), std::string (label)}, fn.str ());

        Visualizer svg (outPath.string (), 900);

        // Draw region from the engine's current workspace (getter).
        if (auto wptr = engine->getWorkspace ())
        {
            svg.drawRegion (*wptr);
        }

        // Draw the actual skeleton graph used (local/global) if present in debug info.
        if (dbg.graph)
        {
            svg.drawSkeleton (*dbg.graph);
        }

        svg.drawPath (path);

        // Debug overlays.
        for (const auto &s : dbg.coarse)
            svg.drawPose (s, 6.0, "#ffb300ff");
        for (auto [i, j] : dbg.stitchedPairs)
        {
            if (i < dbg.waypoints.size () && j < dbg.waypoints.size ())
            {
                std::array<State, 2> seg{dbg.waypoints[i], dbg.waypoints[j]};
                svg.drawPath (std::span<const State> (seg.data (), seg.size ()), "#b833ccff", 1.0, 1.0);
            }
        }

        svg.drawStartPose (start);
        svg.drawGoalPose (goal);

        svg.drawAxes ();
        svg.finish ();
#endif

        ASSERT_TRUE (ok) << "Case " << id << " failed: " << why.str ();
    }

    /**
     * @brief Run one plan() using footprint-aware collision.
     */
    void runOneOnWorkspaceFootprint (const std::shared_ptr<Workspace> &workspace, int id, std::string_view label)
    {
        auto engine = makeEngineWithFootprint (workspace);

        // Use the same footprint as makeEngineWithFootprint for prechecks & validation
        Robot robot = Robot::rectangle (ROBOT_LEN, ROBOT_WID);

        State start{}, goal{};
        // Sample start/goal using footprint-aware reachability and ensure the footprint fits
        samplePoseStart (*workspace, start, /*withFootprint=*/true);
        samplePoseGoal (*workspace, goal, /*withFootprint=*/true);

        typename EngineT::DebugInfo dbg;
        std::vector<State> path;
        {
            ScopedTimer t (planStats_);
            (void)t;
            path = engine->plan (start, goal, &dbg);
        }

        bool ok = true;
        std::ostringstream why;
        if (path.empty ())
        {
            ok = false;
            why << "empty path; ";
        }
        else if (euclidean (path.back (), goal) > EPS_GOAL)
        {
            ok = false;
            why << "goal miss; ";
        }

        std::vector<std::size_t> badIdx;
        if (!path.empty ())
        {
            for (std::size_t i = 0; i < path.size (); ++i)
            {
                if (!workspace->coveredBy (robot.at (path[i])))
                    badIdx.push_back (i);
            }

            if (!badIdx.empty ())
            {
                ok = false;
                why << "path leaves workspace; ";
            }
        }

#ifdef AG_ENABLE_PLOTS
        const char *tag = ok ? "ok" : "fail";
        std::ostringstream fn;
        fn << tag << "_fp_" << id << ".svg";
        auto outPath = test_helpers::plotFile ({"engine", Cfg::name (), std::string (label)}, fn.str ());

        Visualizer svg (outPath.string (), 900);

        if (auto wptr = engine->getWorkspace ())
        {
            svg.drawRegion (*wptr);
        }
        if (dbg.graph)
        {
            svg.drawSkeleton (*dbg.graph);
        }

        svg.drawPath (path);

        for (const auto &s : dbg.coarse)
            svg.drawPose (s, 6.0, "#ffb300ff");
        for (auto [i, j] : dbg.stitchedPairs)
        {
            if (i < dbg.waypoints.size () && j < dbg.waypoints.size ())
            {
                std::array<State, 2> seg{dbg.waypoints[i], dbg.waypoints[j]};
                svg.drawPath (std::span<const State> (seg.data (), seg.size ()), "#b833ccff", 1.0, 1.0);
            }
        }

        // Draw robot footprint outlines along the path
        if (!path.empty ())
        {
            const std::size_t n = path.size ();
            std::size_t stride = (n > 81) ? 40 : (n > 51) ? 25 : 1;
            for (std::size_t i = stride; i + stride < n; i += stride)
                svg.drawPolygon (robot.at (path[i]), "none", "#375765ff", 0.8, 1.0);

            for (std::size_t i : badIdx)
                svg.drawPolygon (robot.at (path[i]), "none", "#ff4444", 0.9, 1.0);
        }

        svg.drawStartPose (start);
        svg.drawGoalPose (goal);
        svg.drawPolygon (robot.at (start), "none", "#375765ff", 0.8, 1.0);
        svg.drawPolygon (robot.at (goal), "none", "#375765ff", 0.8, 1.0);

        svg.drawAxes ();
        svg.finish ();
#endif

        ASSERT_TRUE (ok) << "Footprint case " << id << " failed: " << why.str ();
    }

  private:
    std::mt19937 randomEngine_;
    std::uniform_real_distribution<double> unitUniform_; // kept for future extensions

    inline static RunningStats planStats_{};
};

/* ───────── instantiate engine combos here (extensible) ───────── */
using GraphT = arcgen::geometry::Graph;
using TestedEngines =
    ::testing::Types<EngineConfig<Dubins, AStar<GraphT>, arcgen::geometry::StraightSkeleton>, EngineConfig<ReedsShepp, AStar<GraphT>, arcgen::geometry::StraightSkeleton>>;
TYPED_TEST_SUITE (EngineFixture, TestedEngines);

/* ───────── tests over predefined and random workspaces ───────── */

/// @test Random workspaces (fresh geometry every iteration).
TYPED_TEST (EngineFixture, PlansWithinRandomWorkspaces)
{
    std::mt19937 randomEngine (123);
    for (int k = 0; k < SAMPLES; ++k)
    {
        auto workspace = test_helpers::randomWorkspace (randomEngine);
        this->runOneOnWorkspace (workspace, k, "random");
    }
}

/// @test Maze workspace (narrow corridors).
TYPED_TEST (EngineFixture, PlansWithinMazeWorkspace)
{
    auto workspace = test_helpers::mazeWorkspace ();
    for (int k = 0; k < SAMPLES; ++k)
        this->runOneOnWorkspace (workspace, k, "maze");
}

/// @test Gear workspace (spiky outer ring + circular holes).
TYPED_TEST (EngineFixture, PlansWithinGearWorkspace)
{
    auto workspace = test_helpers::gearWorkspace ();
    for (int k = 0; k < SAMPLES; ++k)
        this->runOneOnWorkspace (workspace, k, "gear");
}

/// @test Random workspaces with footprint-aware collision.
TYPED_TEST (EngineFixture, PlansWithinRandomWorkspacesWithFootprint)
{
    std::mt19937 randomEngine (456);
    for (int k = 0; k < SAMPLES; ++k)
    {
        auto workspace = test_helpers::randomWorkspace (randomEngine);
        this->runOneOnWorkspaceFootprint (workspace, k, "random_fp");
    }
}

/// @test Maze workspace with footprint-aware collision.
TYPED_TEST (EngineFixture, PlansWithinMazeWorkspaceWithFootprint)
{
    auto workspace = test_helpers::mazeWorkspace ();
    for (int k = 0; k < SAMPLES; ++k)
        this->runOneOnWorkspaceFootprint (workspace, k, "maze_fp");
}

/// @test Gear workspace with footprint-aware collision.
TYPED_TEST (EngineFixture, PlansWithinGearWorkspaceWithFootprint)
{
    auto workspace = test_helpers::gearWorkspace ();
    for (int k = 0; k < SAMPLES; ++k)
        this->runOneOnWorkspaceFootprint (workspace, k, "gear_fp");
}
