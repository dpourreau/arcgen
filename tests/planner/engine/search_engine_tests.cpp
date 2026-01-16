/**
 * @file search_engine_tests.cpp
 * @brief End-to-end planning tests that plot and validate paths produced by SearchEngine.
 */

#include <arcgen.hpp>

#include <utils/output_paths.hpp>
#include <utils/pose_sampling.hpp>
#include <utils/visualizer.hpp>
#include <utils/workspace_generators.hpp>

#include <boost/geometry.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

using namespace arcgen::core;
using namespace arcgen::planner;
using namespace arcgen::planner::geometry;
using namespace arcgen::steering;
using arcgen::planner::graph::AStar;
using AStarSearch = arcgen::planner::graph::AStar<arcgen::planner::geometry::Graph>;
namespace connector = arcgen::planner::connector;
using arcgen::utils::Visualizer;

namespace bg = boost::geometry;

/* ───────── configuration ───────── */
constexpr double R_MIN = 1.5;     ///< [m]
constexpr double STEP = 0.3;      ///< [m] steering discretisation
constexpr double EPS_GOAL = 0.01; ///< [m] goal proximity
constexpr int SAMPLES = 50;       ///< cases per workspace kind
constexpr double LOCAL_BB_MARGIN = 10.0;

// Robot footprint used for footprint-aware tests
constexpr double ROBOT_LEN = 1.3; ///< [m] rectangle length
constexpr double ROBOT_WID = 0.7; ///< [m] rectangle width

/* ───────── reporting structures ───────── */
namespace
{
    struct PlannerStats
    {
        std::string parameters;
        int samples{0};
        int successes{0};
        std::vector<int> runSuccess; // 1 for success, 0 for fail
        std::vector<double> computationTimes;
        std::vector<double> componentTimes_Skeleton;
        std::vector<double> componentTimes_Search;
        std::vector<double> componentTimes_Connection;
        std::vector<double> pathCosts;
        std::map<std::string, int> sourceCounts;
        std::vector<int> smoothingIterations;
    };

    class StatsAggregator
    {
      public:
        static StatsAggregator &instance ()
        {
            static StatsAggregator inst;
            return inst;
        }

        template <typename DebugInfoT> void addResult (const std::string &configName, const std::string &params, bool success, const DebugInfoT *dbg = nullptr)
        {
            std::lock_guard<std::mutex> lock (mutex_);
            auto &s = stats_[configName];
            s.parameters = params;
            s.samples++;
            if (success)
                s.successes++;

            // Only add runSuccess if we are not just correcting a previous mistake but strictly maintaining order.
            // In the previous step I added it. ensuring it's here.
            s.runSuccess.push_back (success ? 1 : 0);

            if (dbg)
            {
                // Computation Time (Total)
                double totalTime = 0.0;

                double tSkel = 0.0;
                if (dbg->componentTimes.count ("Skeleton Generation (Local)"))
                    tSkel += dbg->componentTimes.at ("Skeleton Generation (Local)");
                if (dbg->componentTimes.count ("Skeleton Generation (Global)"))
                    tSkel += dbg->componentTimes.at ("Skeleton Generation (Global)");

                double tSearch = 0.0;
                if (dbg->componentTimes.count ("Graph Search (Local)"))
                    tSearch += dbg->componentTimes.at ("Graph Search (Local)");
                if (dbg->componentTimes.count ("Graph Search (Global)"))
                    tSearch += dbg->componentTimes.at ("Graph Search (Global)");

                double tConn = 0.0;
                // Sum connection times from stages if available
                if (dbg->componentTimes.count ("Connection (Local)"))
                    tConn += dbg->componentTimes.at ("Connection (Local)");
                if (dbg->componentTimes.count ("Connection (Global)"))
                    tConn += dbg->componentTimes.at ("Connection (Global)");

                // Fallback to internal connector total if explicit stage not found (compatibility)
                if (tConn == 0.0 && dbg->componentTimes.count ("Total Connection"))
                    tConn = dbg->componentTimes.at ("Total Connection");

                if (dbg->componentTimes.count ("Total Planning Time"))
                {
                    totalTime = dbg->componentTimes.at ("Total Planning Time");
                }
                else if (tConn == 0.0 && tSkel == 0.0 && tSearch == 0.0)
                {
                    // Fallback: sum all steps if components are missing
                    for (const auto &step : dbg->history)
                        totalTime += step.computationTime;
                }
                else
                {
                    totalTime = tSkel + tSearch + tConn;
                }

                s.computationTimes.push_back (totalTime);
                s.componentTimes_Skeleton.push_back (tSkel);
                s.componentTimes_Search.push_back (tSearch);
                s.componentTimes_Connection.push_back (tConn);

                if (success && dbg->source.has_value ())
                {
                    std::string srcName;
                    // Use the type form DebugInfoT
                    using SourceEnum = typename DebugInfoT::Source;
                    switch (*dbg->source)
                    {
                        case SourceEnum::Direct:
                            srcName = "Direct";
                            break;
                        case SourceEnum::Local:
                            srcName = "Local";
                            break;
                        case SourceEnum::Global:
                            srcName = "Global";
                            break;
                    }
                    s.sourceCounts[srcName]++;
                }

                // Smoothing iterations
                s.smoothingIterations.push_back (dbg->smoothingIterations);

                // Path Cost - Check last step for stats
                if (success && !dbg->history.empty ())
                {
                    s.pathCosts.push_back (dbg->history.back ().stats.totalCost);
                }
            }
        }

        const std::map<std::string, PlannerStats> &getStats () const { return stats_; }

      private:
        std::map<std::string, PlannerStats> stats_;
        std::mutex mutex_;
    };

} // namespace

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
            return "Dubins";
        if constexpr (std::is_same_v<SteeringT, ReedsShepp>)
            return "ReedsShepp";
        return "Generic";
    }
};

/**
 * @brief Typed fixture for end-to-end planning (SearchEngine) with timing & optional SVG plots.
 * @tparam Cfg  EngineConfig triple: Steering, Search, Skeleton.
 */
template <class Cfg> class EngineFixture : public ::testing::Test
{
  protected:
    using Graph = arcgen::planner::geometry::Graph;
    using SteeringT = typename Cfg::Steering;
    using SearchT = typename Cfg::Search;
    using SkeletonT = typename Cfg::Skeleton;
    using EngineT = engine::SearchEngine<SteeringT, SearchT, SkeletonT>;
    using ConnectorT = typename EngineT::ConnectorType;

    EngineFixture () = default;

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
        using arcgen::utils::halfDiskInside;

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
    std::unique_ptr<EngineT> makeEngine (const std::shared_ptr<Workspace> &workspace, bool enableLocalSearch = false, double localBBMargin = 10.0)
    {
        auto steering = std::make_shared<SteeringT> (R_MIN, STEP);
        auto search = std::make_shared<SearchT> ();
        auto skeleton = std::make_shared<SkeletonT> ();

        auto connectorPtr = std::make_shared<ConnectorT> ();
        auto eng = std::make_unique<EngineT> (steering, search, skeleton, workspace, connectorPtr, enableLocalSearch, localBBMargin);

        namespace C = arcgen::planner::constraints;
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
    std::unique_ptr<EngineT> makeEngineWithFootprint (const std::shared_ptr<Workspace> &workspace, bool enableLocalSearch = false, double localBBMargin = 10.0)
    {
        auto steering = std::make_shared<SteeringT> (R_MIN, STEP);
        auto search = std::make_shared<SearchT> ();
        auto skeleton = std::make_shared<SkeletonT> ();

        auto connectorPtr = std::make_shared<ConnectorT> ();
        auto eng = std::make_unique<EngineT> (steering, search, skeleton, workspace, connectorPtr, enableLocalSearch, localBBMargin);

        namespace C = arcgen::planner::constraints;
        using CSet = C::ConstraintSet<SteeringT::kSegments>;

        // Choose a modest rectangle footprint (shared constants)
        Robot robot = Robot::rectangle (ROBOT_LEN, ROBOT_WID);

        CSet cs;
        cs.hard.push_back (std::make_shared<C::FootprintCollisionConstraint<SteeringT::kSegments>> (workspace, robot));
        cs.soft.push_back ({std::make_shared<C::PathLengthConstraint<SteeringT::kSegments>> (), 1.0});
        eng->setConstraints (std::move (cs));
        return eng;
    }

    struct RenderContext
    {
        int id;
        std::string_view label;
        bool ok;
        const EngineT &engine;
        const typename EngineT::DebugInfo &dbg;
        const State &start;
        const State &goal;
        std::optional<Robot> robot = std::nullopt;
    };

    /**
     * @brief Saves debug artifacts (SVGs) to the build directory.
     *
     */
    void saveDebugArtifacts (const RenderContext &ctx)
    {
#ifdef AG_ENABLE_PLOTS
        int stepIdx = 0;
        for (const auto &step : ctx.dbg.history)
        {
            const char *tag = ctx.ok ? "ok" : "fail";
            std::string safeName = step.name;
            std::replace (safeName.begin (), safeName.end (), ' ', '_');

            std::ostringstream fn;
            fn << tag << "_" << (ctx.robot ? "fp_" : "") << ctx.id << "_step_" << stepIdx++ << "_" << safeName << ".svg";
            auto outPath = arcgen::utils::plotFile ({"engine", Cfg::name (), std::string (ctx.label)}, fn.str ());

            Visualizer svg (outPath.string (), 900);
            const auto &pal = svg.getPalette ();

            if (auto wptr = ctx.engine.getWorkspace ())
            {
                svg.drawRegion (*wptr);
            }
            if (ctx.dbg.graph)
            {
                svg.drawSkeleton (*ctx.dbg.graph);
            }

            svg.drawPath (step.path);

            // Resampled Points (Indices/Interpolated with fixed candidates) -> Orange, Small
            for (const auto &s : step.resampledPoints)
                svg.drawPose (s, 6.0, pal.graphEdge);

            // Fixed Anchors (Waypoints/Joints) -> Purple, Big
            for (const auto &s : step.fixedAnchors)
                svg.drawPose (s, 9.0, pal.anchor);

            // Draw robot footprint outlines along the step path if robot is present
            if (ctx.robot && !step.path.empty ())
            {
                const std::size_t n = step.path.size ();
                for (std::size_t i = 0; i < n; ++i)
                    svg.drawPolygon (ctx.robot->at (step.path[i]), "none", pal.robotFill, 0.8, 1.0);
            }

            svg.drawStartPose (ctx.start);
            svg.drawGoalPose (ctx.goal);
            if (ctx.robot)
            {
                svg.drawPolygon (ctx.robot->at (ctx.start), "none", pal.robotFill, 0.8, 1.0);
                svg.drawPolygon (ctx.robot->at (ctx.goal), "none", pal.robotFill, 0.8, 1.0);
            }

            svg.drawAxes ();
            svg.finish ();
        }
#endif
    }

    void collectStats (std::string_view label, bool ok, const EngineT &engine, const typename EngineT::DebugInfo &dbg, bool fp, bool localSearch, double margin)
    {
        std::ostringstream params;
        params << "Steering=" << Cfg::name () << ", R_min=" << R_MIN << ", Step=" << STEP << ", LocalSearch=" << (localSearch ? "On" : "Off") << ", Margin=" << margin
               << ", Footprint=" << (fp ? "Yes" : "No");

        if (auto conn = engine.getConnector ())
        {
            params << ", Resample=" << conn->getResampleInterval () << ", MaxIter=" << conn->getMaxIterations () << ", Lookahead=" << conn->getLookaheadMatches ()
                   << ", MinResample=" << conn->getMinResampleInterval () << ", CostTol=" << conn->getCostImprovementTol ();
        }

        // Infer workspace type from label (e.g., "random", "maze", "random_fp", "maze_fp")
        // Simple heuristic: check if it contains "random" or "maze"
        std::string wsType = "Unknown";
        if (label.find ("random") != std::string_view::npos)
            wsType = "Random";
        else if (label.find ("maze") != std::string_view::npos)
            wsType = "Maze";

        // Construct unique config name
        std::ostringstream cfgName;
        // Key Format: "[Steering] [Workspace] [Footprint] [Local/Default]"
        cfgName << Cfg::name () << " [" << wsType << "]" << (fp ? " (Footprint)" : "") << (localSearch ? " [Local]" : " [Default]");

        StatsAggregator::instance ().addResult (cfgName.str (), params.str (), ok, &dbg);
    }

    /**
     * @brief Run one plan() given a fixed start/goal and configuration.
     */
    void runOneWithState (const std::shared_ptr<Workspace> &workspace, const State &start, const State &goal, int id, std::string_view label, bool enableLocalSearch)
    {
        auto engine = makeEngine (workspace, enableLocalSearch, LOCAL_BB_MARGIN);

        typename EngineT::DebugInfo dbg;
        typename EngineT::DebugInfo *dbgPtr = &dbg; // Always pass dbg for stats, even if plots disabled

        std::vector<State> path;
        auto result = engine->plan (const_cast<State &> (start), const_cast<State &> (goal), dbgPtr);
        if (result)
            path = *result;

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

        collectStats (label, ok, *engine, dbg, false, enableLocalSearch, LOCAL_BB_MARGIN);

#ifdef AG_ENABLE_PLOTS
        saveDebugArtifacts ({id, label, ok, *engine, dbg, start, goal});
#endif

        ASSERT_TRUE (ok) << "Case " << id << " (" << label << ") failed: " << why.str ();
    }

    /**
     * @brief Wrapper to sample and run both default and local-search configurations on the same problem.
     */
    void runOneOnWorkspace (const std::shared_ptr<Workspace> &workspace, int id, [[maybe_unused]] std::string_view labelPrefix)
    {
        State start{}, goal{};
        samplePoseStart (*workspace, start);
        samplePoseGoal (*workspace, goal);

        runOneWithState (workspace, start, goal, id, std::string (labelPrefix), false);
        runOneWithState (workspace, start, goal, id, std::string (labelPrefix) + "_local", true);
    }

    /**
     * @brief Run one plan() with footprint given a fixed start/goal.
     */
    void runOneWithStateFootprint (const std::shared_ptr<Workspace> &workspace, const State &start, const State &goal, int id, std::string_view label, bool enableLocalSearch,
                                   const Robot &robot, double localBBMargin)
    {
        auto engine = makeEngineWithFootprint (workspace, enableLocalSearch, localBBMargin);

        typename EngineT::DebugInfo dbg;
        typename EngineT::DebugInfo *dbgPtr = &dbg;

        std::vector<State> path;
        auto result = engine->plan (const_cast<State &> (start), const_cast<State &> (goal), dbgPtr);
        if (result)
            path = *result;

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

        collectStats (label, ok, *engine, dbg, true, enableLocalSearch, localBBMargin);

#ifdef AG_ENABLE_PLOTS
        saveDebugArtifacts ({id, label, ok, *engine, dbg, start, goal, robot});
#endif

        ASSERT_TRUE (ok) << "Footprint case " << id << " (" << label << ") failed: " << why.str ();
    }

    /**
     * @brief Wrapper to sample and run both default and local-search configurations for footprint tests.
     */
    void runOneOnWorkspaceFootprint (const std::shared_ptr<Workspace> &workspace, int id, [[maybe_unused]] std::string_view labelPrefix)
    {
        // Use the same footprint as makeEngineWithFootprint for prechecks & validation
        Robot robot = Robot::rectangle (ROBOT_LEN, ROBOT_WID);

        State start{}, goal{};
        // Sample start/goal using footprint-aware reachability and ensure the footprint fits
        samplePoseStart (*workspace, start, /*withFootprint=*/true);
        samplePoseGoal (*workspace, goal, /*withFootprint=*/true);

        runOneWithStateFootprint (workspace, start, goal, id, std::string (labelPrefix), false, robot, LOCAL_BB_MARGIN);
        runOneWithStateFootprint (workspace, start, goal, id, std::string (labelPrefix) + "_local", true, robot, LOCAL_BB_MARGIN);
    }

  private:
    std::mt19937 randomEngine_{42};
    std::uniform_real_distribution<double> unitUniform_{0.0, 1.0}; // kept for future extensions
};

/* ───────── instantiate engine combos here (extensible) ───────── */
using GraphT = arcgen::planner::geometry::Graph;
using TestedEngines = ::testing::Types<EngineConfig<Dubins, AStar<GraphT>, arcgen::planner::geometry::StraightSkeleton>,
                                       EngineConfig<ReedsShepp, AStar<GraphT>, arcgen::planner::geometry::StraightSkeleton>>;
TYPED_TEST_SUITE (EngineFixture, TestedEngines);

/* ───────── tests over predefined and random workspaces ───────── */

/// @test Random workspaces (fresh geometry every iteration).
TYPED_TEST (EngineFixture, PlansWithinRandomWorkspaces)
{
    std::mt19937 randomEngine (123);
    for (int k = 0; k < SAMPLES; ++k)
    {
        auto workspace = arcgen::utils::randomWorkspace (randomEngine);
        this->runOneOnWorkspace (workspace, k, "random");
    }
}

/// @test Maze workspace (narrow corridors).
TYPED_TEST (EngineFixture, PlansWithinMazeWorkspace)
{
    auto workspace = arcgen::utils::mazeWorkspace ();
    for (int k = 0; k < SAMPLES; ++k)
        this->runOneOnWorkspace (workspace, k, "maze");
}

/// @test Random workspaces with footprint-aware collision.
TYPED_TEST (EngineFixture, PlansWithinRandomWorkspacesWithFootprint)
{
    std::mt19937 randomEngine (456);
    for (int k = 0; k < SAMPLES; ++k)
    {
        auto workspace = arcgen::utils::randomWorkspace (randomEngine);
        this->runOneOnWorkspaceFootprint (workspace, k, "random_fp");
    }
}

/// @test Maze workspace with footprint-aware collision.
TYPED_TEST (EngineFixture, PlansWithinMazeWorkspaceWithFootprint)
{
    auto workspace = arcgen::utils::mazeWorkspace ();
    for (int k = 0; k < SAMPLES; ++k)
        this->runOneOnWorkspaceFootprint (workspace, k, "maze_fp");
}

// --- Global Report Generator --- //
#ifdef AG_ENABLE_TEST_REPORT

class TestReportEnvironment : public ::testing::Environment
{
    // --- Safe Maths Helpers ---
    static double getMean (const std::vector<double> &v)
    {
        if (v.empty ())
            return 0.0;
        return std::reduce (v.begin (), v.end ()) / static_cast<double> (v.size ());
    }
    static double getMin (const std::vector<double> &v)
    {
        if (v.empty ())
            return 0.0;
        return *std::min_element (v.begin (), v.end ());
    }
    static double getMax (const std::vector<double> &v)
    {
        if (v.empty ())
            return 0.0;
        return *std::max_element (v.begin (), v.end ());
    }
    static double getStdDev (const std::vector<double> &v)
    {
        if (v.size () < 2)
            return 0.0;
        double mean = getMean (v);
        double sqSum = std::transform_reduce (v.begin (), v.end (), 0.0, std::plus<> (), [mean] (double x) { return (x - mean) * (x - mean); });
        return std::sqrt (sqSum / static_cast<double> (v.size () - 1));
    }
    static double getMeanInt (const std::vector<int> &v)
    {
        if (v.empty ())
            return 0.0;
        return static_cast<double> (std::reduce (v.begin (), v.end ())) / static_cast<double> (v.size ());
    }

  public:
    void TearDown () override
    {
        const auto &stats = StatsAggregator::instance ().getStats ();
        if (stats.empty ())
            return;

        // Use stats directory for reports
        std::filesystem::path buildDir = arcgen::utils::statsRoot ();

        writeTextReport (buildDir / "planner_stats_report.txt", stats);
        writeCsvReport (buildDir / "planner_stats.csv", stats);

        std::cout << "\n[PlannerStats] Reports generated in " << buildDir << "\n";
    }

  private:
    void writeTextReport (const std::filesystem::path &path, const std::map<std::string, PlannerStats> &stats)
    {
        std::ofstream p (path);
        if (!p)
            return;

        p << "========================================================================\n"
          << "                    SEARCH ENGINE TEST REPORT                           \n"
          << "========================================================================\n\n";

        for (const auto &[name, s] : stats)
        {
            p << "Configuration: " << name << "\n";
            p << "  Parameters: " << s.parameters << "\n";
            p << "  Stats:\n";
            double successRate = s.samples > 0 ? (100.0 * s.successes / s.samples) : 0.0;
            p << std::fixed << std::setprecision (1);
            p << "    - Success Rate: " << successRate << "% (" << s.successes << "/" << s.samples << ")\n";

            if (s.successes > 0)
            {
                p << std::fixed << std::setprecision (5);
                p << "    - Total Time (s): Avg=" << getMean (s.computationTimes) << " Min=" << getMin (s.computationTimes) << " Max=" << getMax (s.computationTimes)
                  << " Std=" << getStdDev (s.computationTimes) << "\n";

                p << "    - Time Breakdown (Avg s): "
                  << "Skel=" << getMean (s.componentTimes_Skeleton) << ", Search=" << getMean (s.componentTimes_Search) << ", Conn=" << getMean (s.componentTimes_Connection)
                  << "\n";

                p << "    - Path Cost (Avg): " << getMean (s.pathCosts) << "\n";

                p << "    - Smoothing Iters (Avg): " << getMeanInt (s.smoothingIterations) << "\n";

                if (!s.sourceCounts.empty ())
                {
                    p << "    - Source Distribution: ";
                    for (const auto &[src, count] : s.sourceCounts)
                        p << src << ":" << count << " ";
                    p << "\n";
                }
            }
            p << "------------------------------------------------------------------------\n\n";
        }
    }

    void writeCsvReport (const std::filesystem::path &path, const std::map<std::string, PlannerStats> &stats)
    {
        std::ofstream p (path);
        if (!p)
            return;

        // CSV Header
        // Note: This CSV aggregates by config. Per-sample CSV would require storing individual runs.
        // The implementation plan requested a general report, but the CSV suggestion was:
        // "Config,SampleID,Success,Source,TotalTime,SkelTime,SearchTime,ConnTime,PathCost,SmoothingIters"
        // To do that, I would need to have stored every single run's data.
        // The struct has vectors, so I CAN produce this per-sample CSV.

        p << "Config,Success,TotalTime,SkelTime,SearchTime,ConnTime,PathCost,SmoothingIters\n";

        for (const auto &[name, s] : stats)
        {
            size_t n = s.computationTimes.size ();
            for (size_t i = 0; i < n; ++i)
            {
                p << name << ",";
                if (i < s.runSuccess.size ())
                    p << s.runSuccess[i] << ",";
                else
                    p << "0,";
                p << s.computationTimes[i] << ",";
                p << s.componentTimes_Skeleton[i] << ",";
                p << s.componentTimes_Search[i] << ",";
                p << s.componentTimes_Connection[i] << ",";
                if (i < s.pathCosts.size ())
                    p << s.pathCosts[i] << ",";
                else
                    p << "0,";
                if (i < s.smoothingIterations.size ())
                    p << s.smoothingIterations[i];
                else
                    p << "0";
                p << "\n";
            }
        }
    }
};

::testing::Environment *const reportEnv = ::testing::AddGlobalTestEnvironment (new TestReportEnvironment);

#endif
