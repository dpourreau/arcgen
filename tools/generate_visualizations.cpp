/**
 * @file generate_visualizations.cpp
 * @brief Standalone tool for generating SVG sequences for planning algorithm steps.
 */

#include <arcgen.hpp>
#include <arcgen/planner/geometry/robot.hpp>

#include <algorithm>
#include <boost/geometry.hpp>
#include <format>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <utils/output_paths.hpp>
#include <utils/pose_sampling.hpp>
#include <utils/visualizer.hpp>
#include <utils/workspace_generators.hpp>

using arcgen::core::State;
using arcgen::planner::geometry::Point;
using arcgen::planner::geometry::Robot;
using arcgen::planner::geometry::Workspace;
using arcgen::steering::Dubins;
using arcgen::steering::ReedsShepp;

constexpr int NUM_SAMPLES = 10;
constexpr double R_MIN = 1.5;
constexpr double STEP = 0.3;
constexpr double ROBOT_LEN = 1.3;
constexpr double ROBOT_WID = 0.7;

using SkeletonT = arcgen::planner::geometry::StraightSkeleton;
using GraphT = arcgen::planner::geometry::Graph;
using AStarSearch = arcgen::planner::graph::AStar<GraphT>;

template <typename SteeringT> using EngineT = arcgen::planner::engine::SearchEngine<SteeringT, AStarSearch, SkeletonT>;

struct SceneContext
{
    std::shared_ptr<Workspace> workspace;
    const State *start;
    const State *goal;
    const Robot *robot;
    const GraphT *graph = nullptr;
};

template <typename SteeringT> std::string getModelName ()
{
    if constexpr (std::is_same_v<SteeringT, Dubins>)
        return "dubins";
    if constexpr (std::is_same_v<SteeringT, ReedsShepp>)
        return "reeds_shepp";
    return "unknown";
}

/**
 * @brief Factory helper to create a consistent SearchEngine and Robot configuration.
 */
template <typename SteeringT> auto createEngine (const std::shared_ptr<Workspace> &workspace)
{
    auto steering = std::make_shared<SteeringT> (R_MIN, STEP);
    auto search = std::make_shared<AStarSearch> ();
    auto skeleton = std::make_shared<SkeletonT> ();
    auto connectorPtr = std::make_shared<typename EngineT<SteeringT>::ConnectorType> ();

    auto engine = std::make_unique<EngineT<SteeringT>> (steering, search, skeleton, workspace, connectorPtr);
    auto robot = Robot::rectangle (ROBOT_LEN, ROBOT_WID);

    namespace C = arcgen::planner::constraints;
    using CSet = C::ConstraintSet<SteeringT::kSegments>;
    CSet cs;
    cs.hard.emplace_back (std::make_shared<C::FootprintCollisionConstraint<SteeringT::kSegments>> (workspace, robot));
    cs.soft.push_back ({std::make_shared<C::PathLengthConstraint<SteeringT::kSegments>> (), 1.0});
    engine->setConstraints (std::move (cs));

    return std::make_pair (std::move (engine), robot);
}

/**
 * @brief Attempts to plan a path between start and goal.
 */
template <typename SteeringT> std::optional<typename EngineT<SteeringT>::DebugInfo> tryPlan (const std::shared_ptr<Workspace> &workspace, const State &start, const State &goal)
{
    auto [engine, robot] = createEngine<SteeringT> (workspace);

    State startCopy = start;
    State goalCopy = goal;

    typename EngineT<SteeringT>::DebugInfo dbg;
    if (auto result = engine->plan (startCopy, goalCopy, &dbg))
    {
        return dbg;
    }
    return std::nullopt;
}

/**
 * @brief Helper to draw common scene elements (Map, Start/Goal, Skeleton if available).
 */
void drawSceneCommon (arcgen::utils::Visualizer &viz, const SceneContext &ctx)
{
    const auto &pal = viz.getPalette ();
    viz.drawRegion (*ctx.workspace);
    if (ctx.graph)
        viz.drawSkeleton (*ctx.graph);

    viz.drawStartPose (*ctx.start);
    viz.drawPolygon (ctx.robot->at (*ctx.start), "none", pal.robotFill, 0.8);
    viz.drawGoalPose (*ctx.goal);
    viz.drawAxes ();
}

void visualizeMapAndStartGoal (const std::string &baseDir, const SceneContext &ctx)
{
    auto outPath = arcgen::utils::plotFile ({baseDir}, "00_map_start_goal.svg");
    arcgen::utils::Visualizer viz (outPath.string ());
    drawSceneCommon (viz, ctx);
}

void visualizeSkeleton (const std::string &baseDir, const SceneContext &ctx)
{
    auto outPath = arcgen::utils::plotFile ({baseDir}, "01_skeleton.svg");
    arcgen::utils::Visualizer viz (outPath.string ());
    drawSceneCommon (viz, ctx);
}

template <typename EngineDebugInfo> void drawGraphPath (arcgen::utils::Visualizer &viz, const EngineDebugInfo &step, const arcgen::utils::Palette &pal)
{
    if (step.resampledPoints.size () >= 2)
    {
        for (size_t k = 0; k + 1 < step.resampledPoints.size (); ++k)
        {
            viz.lineWorld (step.resampledPoints[k].x, step.resampledPoints[k].y, step.resampledPoints[k + 1].x, step.resampledPoints[k + 1].y, pal.graphEdge, 2.0);
        }
        for (const auto &pt : step.resampledPoints)
            viz.drawPose (pt, 5.0, pal.graphEdge);
    }
}

template <typename EngineDebugInfo> std::vector<std::size_t> calculateFixedAnchors (const EngineDebugInfo &step)
{
    std::vector<std::size_t> indices;
    if (step.path.empty ())
        return indices;

    std::size_t lastIdx = 0;
    for (const auto &anchor : step.fixedAnchors)
    {
        for (std::size_t idx = lastIdx; idx < step.path.size (); ++idx)
        {
            // Simple epsilon check
            constexpr double EPS = 1e-4;
            if (std::abs (step.path[idx].x - anchor.x) < EPS && std::abs (step.path[idx].y - anchor.y) < EPS && std::abs (step.path[idx].heading - anchor.heading) < EPS)
            {
                indices.push_back (idx);
                lastIdx = idx;
                break;
            }
        }
    }
    return indices;
}

template <typename EngineDebugInfo>
void drawPathFrame (arcgen::utils::Visualizer &viz, const EngineDebugInfo &step, std::size_t k, const std::vector<State> &previousPath,
                    const std::vector<std::size_t> &fixedAnchorIndices, const SceneContext &ctx, const arcgen::utils::Palette &pal)
{
    if (!previousPath.empty ())
    {
        viz.drawPath (previousPath, "", 2.0, 0.3);
        for (const auto &p : previousPath)
        {
            viz.drawPolygon (ctx.robot->at (p), "none", pal.robotGhost, 0.5, 1.0, 0.3);
        }
    }

    // Current Partial Path
    std::vector<State> subPath (step.path.begin (), step.path.begin () + static_cast<std::ptrdiff_t> (k + 1));
    viz.drawPath (subPath);

    // Trail footprints
    for (std::size_t m = 0; m < k; ++m)
    {
        viz.drawPolygon (ctx.robot->at (step.path[m]), "none", pal.robotFill, 0.5);
    }
    // Tip footprint
    viz.drawPolygon (ctx.robot->at (step.path[k]), "none", pal.robotFill, 0.8);

    // Anchors
    for (size_t aIdx = 1; aIdx + 1 < fixedAnchorIndices.size (); ++aIdx)
    {
        if (fixedAnchorIndices[aIdx] <= k)
        {
            viz.drawPose (step.fixedAnchors[aIdx], 9.0, pal.anchor);
        }
    }
}

template <typename StepT> void visualizeGraphPathSubstep (const std::string &baseDir, int stepIdx, const std::string &safeName, const StepT &step, const SceneContext &ctx)
{
    // S6185: Use std::format
    auto fn = std::format ("{:02d}_{}_substep_0_Graph_Path.svg", stepIdx, safeName);
    auto outPath = arcgen::utils::plotFile ({baseDir}, fn);
    arcgen::utils::Visualizer viz (outPath.string ());
    drawSceneCommon (viz, ctx);
    drawGraphPath (viz, step, viz.getPalette ());
}

template <typename StepT> void drawSubstepBackground (arcgen::utils::Visualizer &viz, bool isInitial, const StepT &step)
{
    if (isInitial && step.resampledPoints.size () >= 2)
    {
        const auto &pal = viz.getPalette ();
        for (size_t m = 0; m + 1 < step.resampledPoints.size (); ++m)
        {
            viz.lineWorld (step.resampledPoints[m].x, step.resampledPoints[m].y, step.resampledPoints[m + 1].x, step.resampledPoints[m + 1].y, pal.graphEdge, 1.5);
        }
        for (const auto &pt : step.resampledPoints)
            viz.drawPose (pt, 4.0, pal.graphEdge);
    }
}

template <typename StepT>
void visualizeFrameSequence (const std::string &baseDir, int stepIdx, const std::string &safeName, const StepT &step, const SceneContext &ctx,
                             const std::vector<State> &previousPath, bool isInitial)
{
    auto fixedAnchorIndices = calculateFixedAnchors (step);
    int frameCounter = 0;

    for (std::size_t k = 0; k < step.path.size (); ++k)
    {
        auto fn = std::format ("{:02d}_{}_substep_{}.svg", stepIdx, safeName, frameCounter);
        frameCounter++;

        auto outPath = arcgen::utils::plotFile ({baseDir}, fn);
        arcgen::utils::Visualizer viz (outPath.string ());
        drawSceneCommon (viz, ctx);

        drawSubstepBackground (viz, isInitial, step);
        drawPathFrame (viz, step, k, previousPath, fixedAnchorIndices, ctx, viz.getPalette ());
    }
}

template <typename SteeringT>
void visualizeStep (const std::string &baseDir, int stepIdx, const typename EngineT<SteeringT>::DebugInfo::Step &step, const SceneContext &ctx, std::vector<State> &previousPath)
{
    std::string safeName = step.name;
    std::ranges::replace (safeName, ' ', '_');

    bool isInitial = safeName.contains ("Initial");

    if (isInitial)
    {
        visualizeGraphPathSubstep (baseDir, stepIdx, safeName, step, ctx);
    }

    visualizeFrameSequence (baseDir, stepIdx, safeName, step, ctx, previousPath, isInitial);

    if (!step.path.empty ())
        previousPath = step.path;
}

template <typename SteeringT>
void visualizePlan (int id, const std::shared_ptr<Workspace> &workspace, const State &start, const State &goal, const typename EngineT<SteeringT>::DebugInfo &dbg)
{
    auto [engine, robot] = createEngine<SteeringT> (workspace);
    const auto *globalGraph = (dbg.graph.has_value ()) ? &(*dbg.graph) : engine->getGlobalGraph ();

    SceneContext ctx{workspace, &start, &goal, &robot, globalGraph};

    std::string model = getModelName<SteeringT> ();

    std::string baseDir = std::format ("visualization_task/{}_case_{}", model, id);

    visualizeMapAndStartGoal (baseDir, ctx);
    visualizeSkeleton (baseDir, ctx);

    int stepIdxLookup = 2;
    std::vector<State> previousPath;

    for (const auto &step : dbg.history)
    {
        visualizeStep<SteeringT> (baseDir, stepIdxLookup, step, ctx, previousPath);
        stepIdxLookup++;
    }
}

bool isValidState (const State &s, bool isStart, const std::shared_ptr<Workspace> &workspace, const Robot &robot)
{
    if (!workspace->contains (s.x, s.y))
        return false;
    // Strictly use Dubins-like forward check => guarantee forward reachability
    if (!arcgen::utils::halfDiskInside (*workspace, s.x, s.y, s.heading, R_MIN, isStart))
        return false;
    // Footprint check
    return workspace->coveredBy (robot.at (s.x, s.y, s.heading));
}

bool sampleState (State &s, bool isStart, const std::shared_ptr<Workspace> &workspace, const Robot &robot, std::mt19937 &rng) // NOSONAR
{
    namespace bg = boost::geometry;
    bg::model::box<Point> bb;
    bg::envelope (workspace->region (), bb);

    std::uniform_real_distribution ux (bb.min_corner ().x (), bb.max_corner ().x ());
    std::uniform_real_distribution uy (bb.min_corner ().y (), bb.max_corner ().y ());
    std::uniform_real_distribution uh (0.0, arcgen::core::PI2);

    for (int i = 0; i < 5000; ++i)
    {
        s.x = ux (rng);
        s.y = uy (rng);
        s.heading = uh (rng);
        if (isValidState (s, isStart, workspace, robot))
            return true;
    }
    return false;
}

int main ()
{
    try
    {
        std::cout << "[Visualizer] Starting generation for " << NUM_SAMPLES << " paired samples..." << std::endl;
        std::mt19937 rng (43); // NOSONAR
        auto workspace = arcgen::utils::mazeWorkspace ();
        Robot robot = Robot::rectangle (ROBOT_LEN, ROBOT_WID);

        int casesGenerated = 0;
        while (casesGenerated < NUM_SAMPLES)
        {
            State start{};
            State goal{};

            bool pairFound = false;
            for (int k = 0; k < 1000; ++k)
            {
                if (sampleState (start, true, workspace, robot, rng) && sampleState (goal, false, workspace, robot, rng))
                {
                    pairFound = true;
                    break;
                }
            }

            if (pairFound)
            {
                auto dbgDubins = tryPlan<Dubins> (workspace, start, goal);
                if (!dbgDubins)
                    continue;

                auto dbgReeds = tryPlan<ReedsShepp> (workspace, start, goal);
                if (!dbgReeds)
                    continue;

                std::cout << "[Visualizer] Generating Case ID: " << casesGenerated << "..." << std::endl;
                visualizePlan<Dubins> (casesGenerated, workspace, start, goal, *dbgDubins);
                visualizePlan<ReedsShepp> (casesGenerated, workspace, start, goal, *dbgReeds);
                casesGenerated++;
            }
        }

        std::cout << "[Visualizer] Done." << std::endl;
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[Error] Uncaught exception: " << e.what () << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cerr << "[Error] Unknown exception occurred." << std::endl;
        return 2;
    }
}
