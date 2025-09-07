/**
 * @file engine_getters_tests.cpp
 * @brief End-to-end planning tests that visualize results using engine getters
 *        (workspace, skeleton, cached global graph) instead of rebuilding components.
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
using test_helpers::RunningStats;
using test_helpers::ScopedTimer;
using test_helpers::Visualizer;

namespace bg = boost::geometry;

/* ───────── configuration ───────── */
constexpr double R_MIN = 1.5;     ///< [m]
constexpr double STEP = 0.50;     ///< [m] steering discretisation
constexpr double EPS_GOAL = 0.05; ///< [m] goal proximity
constexpr int SAMPLES = 50;       ///< cases per workspace kind

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

    EngineFixture () : rng_ (42), uni_ (0.0, 1.0) {}

    /// @brief Print mean planning time (once/type).
    static void TearDownTestSuite () { planStats_.printSummary (std::string (Cfg::name ()) + " – SearchEngine plan()"); }

    /**
     * @brief Pick a random interior (x,y) within the workspace envelope.
     * @param W Workspace to sample inside.
     * @param x Sampled X.
     * @param y Sampled Y.
     */
    void sampleInteriorPosition (const Workspace &W, double &x, double &y)
    {
        bg::model::box<Point> bb;
        bg::envelope (W.region (), bb);

        std::uniform_real_distribution<double> ux (bb.min_corner ().x (), bb.max_corner ().x ());
        std::uniform_real_distribution<double> uy (bb.min_corner ().y (), bb.max_corner ().y ());

        do
        {
            x = ux (rng_);
            y = uy (rng_);
        } while (!W.contains (x, y));
    }

    /// @brief Sample heading uniformly in [0, 2π).
    double sampleHeading () { return std::uniform_real_distribution<double> (0.0, two_pi) (rng_); }

    /**
     * @brief Ensure the sampled pose meets the “local reachability” rule via half-disks.
     *
     * Rules:
     *  - Dubins:   start requires forward cap inside; goal requires backward cap inside.
     *  - Reeds–Shepp: start/goal each require (forward || backward) cap inside.
     * @param W        Workspace.
     * @param s        Pose to validate.
     * @param isStart  True for start, false for goal.
     */
    bool locallyReachable (const Workspace &W, const State &s, bool isStart) const
    {
        using test_helpers::halfDiskInside;

        if constexpr (std::is_same_v<SteeringT, Dubins>)
        {
            const bool wantForward = isStart; // start→forward, goal→backward
            return halfDiskInside (W, s.x, s.y, s.heading, R_MIN * 2, wantForward);
        }
        else
        {
            // Reeds–Shepp: either direction is fine.
            const bool fwd = halfDiskInside (W, s.x, s.y, s.heading, R_MIN * 2, true);
            if (fwd)
                return true;
            return halfDiskInside (W, s.x, s.y, s.heading, R_MIN * 2, false);
        }
    }

    /**
     * @brief Sample a start pose that is locally reachable per the chosen model.
     * @param W Workspace.
     * @param s Output start state.
     */
    void samplePoseStart (const Workspace &W, State &s)
    {
        constexpr int MAX_TRIES = 1000;
        for (int t = 0; t < MAX_TRIES; ++t)
        {
            sampleInteriorPosition (W, s.x, s.y);
            s.heading = sampleHeading ();
            s.curvature = 0.0;
            s.direction = DrivingDirection::Neutral;

            if (locallyReachable (W, s, /*isStart=*/true))
                return;
        }
        // Fallback: bias heading towards workspace center.
        bg::model::box<Point> bb;
        bg::envelope (W.region (), bb);
        const double cx = 0.5 * (bb.min_corner ().x () + bb.max_corner ().x ());
        const double cy = 0.5 * (bb.min_corner ().y () + bb.max_corner ().y ());
        s.heading = std::atan2 (cy - s.y, cx - s.x);
    }

    /**
     * @brief Sample a goal pose that is locally reachable per the chosen model.
     * @param W Workspace.
     * @param s Output goal state.
     */
    void samplePoseGoal (const Workspace &W, State &s)
    {
        constexpr int MAX_TRIES = 1000;
        for (int t = 0; t < MAX_TRIES; ++t)
        {
            sampleInteriorPosition (W, s.x, s.y);
            s.heading = sampleHeading ();
            s.curvature = 0.0;
            s.direction = DrivingDirection::Neutral;

            if (locallyReachable (W, s, /*isStart=*/false))
                return;
        }
        // Fallback: bias heading away from center (so its back faces the center).
        bg::model::box<Point> bb;
        bg::envelope (W.region (), bb);
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

        auto eng = std::make_unique<EngineT> (steering, search, skeleton, workspace);

        namespace C = arcgen::planning::constraints;
        using CSet = C::ConstraintSet<SteeringT::kSegments>;

        CSet cs;
        cs.hard.push_back (std::make_shared<C::CollisionConstraint<SteeringT::kSegments>> (workspace));
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

        State a{}, b{};
        samplePoseStart (*workspace, a);
        samplePoseGoal (*workspace, b);

        typename EngineT::DebugInfo dbg;
        std::vector<State> path;
        {
            ScopedTimer t (planStats_);
            (void)t;
            path = engine->plan (a, b, &dbg);
        }

        bool ok = true;
        std::ostringstream why;
        if (path.empty ())
        {
            ok = false;
            why << "empty path; ";
        }
        else if (euclidean (path.back (), b) > EPS_GOAL)
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
        svg.drawStartPose (a);
        svg.drawGoalPose (b);

        // Debug overlays.
        for (const auto &s : dbg.coarse)
            svg.drawPose (s, 6.0, "#ffb300ff");
        for (auto [i, j] : dbg.stitchedPairs)
        {
            if (i < dbg.waypoints.size () && j < dbg.waypoints.size ())
            {
                std::array<State, 2> seg{dbg.waypoints[i], dbg.waypoints[j]};
                svg.drawPath (std::span<const State> (seg.data (), seg.size ()), "#b833ccff", 1.0);
            }
        }
        svg.drawAxes ();
        svg.finish ();
#endif

        ASSERT_TRUE (ok) << "Case " << id << " failed: " << why.str ();
    }

  private:
    std::mt19937 rng_;
    std::uniform_real_distribution<double> uni_; // kept for future extensions

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
    std::mt19937 rng (123);
    for (int k = 0; k < SAMPLES; ++k)
    {
        auto W = test_helpers::randomWorkspace (rng);
        this->runOneOnWorkspace (W, k, "random");
    }
}

/// @test Maze workspace (narrow corridors).
TYPED_TEST (EngineFixture, PlansWithinMazeWorkspace)
{
    auto W = test_helpers::mazeWorkspace ();
    for (int k = 0; k < SAMPLES; ++k)
        this->runOneOnWorkspace (W, k, "maze");
}

/// @test Gear workspace (spiky outer ring + circular holes).
TYPED_TEST (EngineFixture, PlansWithinGearWorkspace)
{
    auto W = test_helpers::gearWorkspace ();
    for (int k = 0; k < SAMPLES; ++k)
        this->runOneOnWorkspace (W, k, "gear");
}
