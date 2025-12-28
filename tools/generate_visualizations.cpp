/**
 * @file generate_visualizations.cpp
 * @brief Standalone tool for generating SVG sequences for planning algorithm steps.
 */

#include <arcgen.hpp>

#include <boost/geometry.hpp>
#include <iostream>
#include <random>
#include <string>
#include <utils/output_paths.hpp>
#include <utils/pose_sampling.hpp>
#include <utils/visualizer.hpp>
#include <utils/workspace_generators.hpp>
#include <vector>

using namespace arcgen::core;
using namespace arcgen::planner;
using namespace arcgen::planner::geometry;
using namespace arcgen::steering;
using arcgen::planner::graph::AStar;
using AStarSearch = arcgen::planner::graph::AStar<arcgen::planner::geometry::Graph>;
namespace connector = arcgen::planner::connector;

// Configuration
constexpr int NUM_SAMPLES = 10;
constexpr double R_MIN = 1.5;
constexpr double STEP = 0.3;

// Engine Typedefs
using SkeletonT = arcgen::planner::geometry::StraightSkeleton;
using GraphT = arcgen::planner::geometry::Graph;

template <typename SteeringT> using EngineT = engine::SearchEngine<SteeringT, AStarSearch, SkeletonT>;

template <typename SteeringT> std::string getModelName ()
{
    if constexpr (std::is_same_v<SteeringT, Dubins>)
        return "dubins";
    if constexpr (std::is_same_v<SteeringT, ReedsShepp>)
        return "reeds_shepp";
    return "unknown";
}

// Include Robot header
#include <arcgen/planner/geometry/robot.hpp>

constexpr double ROBOT_LEN = 1.3;
constexpr double ROBOT_WID = 0.7;

/**
 * @brief Factory helper to create a consistent SearchEngine and Robot configuration.
 * @tparam SteeringT The steering function type (Dubins or Reeds-Shepp).
 * @param workspace Shared pointer to the workspace.
 * @return A pair containing the unique pointer to the engine and the configured Robot.
 */
template <typename SteeringT> auto createEngine (const std::shared_ptr<Workspace> &workspace)
{
    auto steering = std::make_shared<SteeringT> (R_MIN, STEP);
    auto search = std::make_shared<AStarSearch> ();
    auto skeleton = std::make_shared<SkeletonT> ();
    auto connectorPtr = std::make_shared<typename EngineT<SteeringT>::ConnectorType> ();

    auto engine = std::make_unique<EngineT<SteeringT>> (steering, search, skeleton, workspace, connectorPtr);
    Robot robot = Robot::rectangle (ROBOT_LEN, ROBOT_WID);

    namespace C = arcgen::planner::constraints;
    using CSet = C::ConstraintSet<SteeringT::kSegments>;
    CSet cs;
    cs.hard.push_back (std::make_shared<C::FootprintCollisionConstraint<SteeringT::kSegments>> (workspace, robot));
    cs.soft.push_back ({std::make_shared<C::PathLengthConstraint<SteeringT::kSegments>> (), 1.0});
    engine->setConstraints (std::move (cs));

    return std::make_pair (std::move (engine), robot);
}

/**
 * @brief Attempts to plan a path between start and goal.
 * @tparam SteeringT The steering function type.
 * @param workspace Shared pointer to the workspace.
 * @param start The start state.
 * @param goal The goal state.
 * @return Optional DebugInfo containing the plan history if successful, std::nullopt otherwise.
 */
template <typename SteeringT> std::optional<typename EngineT<SteeringT>::DebugInfo> tryPlan (const std::shared_ptr<Workspace> &workspace, const State &start, const State &goal)
{
    auto [engine, robot] = createEngine<SteeringT> (workspace);

    // Constraints are already set by createEngine

    typename EngineT<SteeringT>::DebugInfo dbg;
    auto result = engine->plan (const_cast<State &> (start), const_cast<State &> (goal), &dbg);

    if (result)
        return dbg;
    return std::nullopt;
}

/**
 * @brief Helper to draw common scene elements (Map, Start/Goal, Skeleton if available).
 * @param viz Reference to the Visualizer instance.
 * @param workspace Shared pointer to the workspace.
 * @param start The start state.
 * @param goal The goal state.
 * @param robot The robot configuration.
 * @param graph Optional pointer to the global graph (for visualization).
 */
void drawSceneCommon (arcgen::utils::Visualizer &viz, const std::shared_ptr<Workspace> &workspace, const State &start, const State &goal, const Robot &robot,
                      const arcgen::planner::geometry::Graph *graph = nullptr)
{
    const auto &pal = viz.getPalette ();
    viz.drawRegion (*workspace);
    if (graph)
        viz.drawSkeleton (*graph);

    viz.drawStartPose (start);
    viz.drawPolygon (robot.at (start), "none", pal.robotFill, 0.8);
    viz.drawGoalPose (goal);
    viz.drawAxes ();
}

/**
 * @brief Generates the sequence of SVG frames for a successful plan.
 * @tparam SteeringT The steering function type.
 * @param id The unique test case ID.
 * @param workspace Shared pointer to the workspace.
 * @param start The start state.
 * @param goal The goal state.
 * @param dbg The debug info containing plan history.
 */
template <typename SteeringT>
void visualizePlan (int id, const std::shared_ptr<Workspace> &workspace, const State &start, const State &goal, const typename EngineT<SteeringT>::DebugInfo &dbg)
{
    // Re-instantiate engine just for graph access if needed
    auto [engine, robot] = createEngine<SteeringT> (workspace);
    const auto *globalGraph = (dbg.graph.has_value ()) ? &(*dbg.graph) : engine->getGlobalGraph ();

    std::string model = getModelName<SteeringT> ();
    std::string baseDir = "visualization_task/" + model + "_case_" + std::to_string (id);

    // 1. Map + Start + Goal
    {
        auto outPath = arcgen::utils::plotFile ({baseDir}, "00_map_start_goal.svg");
        arcgen::utils::Visualizer viz (outPath.string ());
        drawSceneCommon (viz, workspace, start, goal, robot); // No skeleton
    }

    // 2. Skeleton
    {
        auto outPath = arcgen::utils::plotFile ({baseDir}, "01_skeleton.svg");
        arcgen::utils::Visualizer viz (outPath.string ());
        drawSceneCommon (viz, workspace, start, goal, robot, globalGraph);
    }

    // 3. Algorithm Steps
    int stepIdxLookup = 2; // For file naming prefix
    std::vector<State> previousPath;

    for (const auto &step : dbg.history)
    {
        std::string safeName = step.name;
        std::replace (safeName.begin (), safeName.end (), ' ', '_');

        bool isInitial = (step.name.find ("Initial") != std::string::npos);

        // Substep 0: Graph Path (for Initial Stitch) OR Start State (for others)
        // For Initial Stitch, we want to visualize the A* result (Graph Path) as a distinct step
        // BEFORE the stitching starts.
        int subStepCounter = 0;

        if (isInitial)
        {
            std::ostringstream fn;
            fn << (stepIdxLookup < 10 ? "0" : "") << stepIdxLookup << "_" << safeName << "_substep_" << subStepCounter++ << "_Graph_Path.svg";
            auto outPath = arcgen::utils::plotFile ({baseDir}, fn.str ());
            arcgen::utils::Visualizer viz (outPath.string ());
            const auto &pal = viz.getPalette ();
            drawSceneCommon (viz, workspace, start, goal, robot, globalGraph);

            // Draw Orange Graph Path
            if (step.resampledPoints.size () >= 2)
            {
                for (size_t k = 0; k + 1 < step.resampledPoints.size (); ++k)
                {
                    viz.lineWorld (step.resampledPoints[k].x, step.resampledPoints[k].y, step.resampledPoints[k + 1].x, step.resampledPoints[k + 1].y, pal.graphEdge,
                                   2.0); // Orange, thick
                }
                for (const auto &pt : step.resampledPoints)
                    viz.drawPose (pt, 5.0, pal.graphEdge);
            }
        }

        // Granular visualization: Point-by-Point (High Frequency)
        // If it's Initial Stitch, overlaid on top of Orange Graph Path
        // If it's Smoothing, just the path evolution

        // Pre-calculate anchor indices for progressive visualization
        std::vector<std::size_t> fixedAnchorIndices;
        if (!step.path.empty ())
        {
            std::size_t lastIdx = 0;
            for (const auto &anchor : step.fixedAnchors)
            {
                for (std::size_t idx = lastIdx; idx < step.path.size (); ++idx)
                {
                    if (std::abs (step.path[idx].x - anchor.x) < 1e-4 && std::abs (step.path[idx].y - anchor.y) < 1e-4 && std::abs (step.path[idx].heading - anchor.heading) < 1e-4)
                    {
                        fixedAnchorIndices.push_back (idx);
                        lastIdx = idx;
                        break;
                    }
                }
            }
        }

        // Frame counter for this step
        int frameCounter = 0;
        for (std::size_t k = 0; k < step.path.size (); ++k)
        {
            std::ostringstream fn;
            fn << (stepIdxLookup < 10 ? "0" : "") << stepIdxLookup << "_" << safeName << "_substep_" << frameCounter++ << ".svg";
            auto outPath = arcgen::utils::plotFile ({baseDir}, fn.str ());
            arcgen::utils::Visualizer viz (outPath.string ());
            const auto &pal = viz.getPalette ();
            drawSceneCommon (viz, workspace, start, goal, robot, globalGraph);

            // Always show Graph Path in background for Initial Stitch
            if (isInitial && step.resampledPoints.size () >= 2)
            {
                for (size_t m = 0; m + 1 < step.resampledPoints.size (); ++m)
                {
                    viz.lineWorld (step.resampledPoints[m].x, step.resampledPoints[m].y, step.resampledPoints[m + 1].x, step.resampledPoints[m + 1].y, pal.graphEdge, 1.5);
                }
                for (const auto &pt : step.resampledPoints)
                    viz.drawPose (pt, 4.0, pal.graphEdge);
            }

            // Draw Previous Path Ghost (30% opacity) if available
            if (!previousPath.empty ())
            {
                // Path lines with 30% opacity (using default directional colors)
                viz.drawPath (previousPath, "", 2.0, 0.3);

                // Ghost footprints
                for (std::size_t m = 0; m < previousPath.size (); ++m)
                {
                    viz.drawPolygon (robot.at (previousPath[m]), "none", pal.robotGhost, 0.5, 1.0, 0.3);
                }
                viz.drawPolygon (robot.at (previousPath.back ()), "none", pal.robotGhost, 0.5, 1.0, 0.3);
            }

            // Draw cumulative Blue Path & Footprints
            std::vector<State> subPath (step.path.begin (), step.path.begin () + static_cast<std::ptrdiff_t> (k + 1));

            // Draw the full path segment
            viz.drawPath (subPath);

            // Draw footprints (Trail + Current Tip)
            for (std::size_t m = 0; m < k; ++m)
            {
                viz.drawPolygon (robot.at (step.path[m]), "none", pal.robotFill, 0.5);
            }
            // Always draw the current tip footprint
            viz.drawPolygon (robot.at (step.path[k]), "none", pal.robotFill, 0.8);

            // Draw REACHED Fixed Anchors Only (skip first=start, skip last=goal)
            for (size_t aIdx = 1; aIdx + 1 < fixedAnchorIndices.size (); ++aIdx)
            {
                if (fixedAnchorIndices[aIdx] <= k)
                {
                    viz.drawPose (step.fixedAnchors[aIdx], 9.0, pal.anchor);
                }
            }
        }

        stepIdxLookup++;

        // Update previous path for next iteration's ghost
        if (!step.path.empty ())
            previousPath = step.path;
    }
}

int main ()
{
    try
    {
        std::cout << "[Visualizer] Starting generation for " << NUM_SAMPLES << " paired samples..." << std::endl;
        std::mt19937 rng (43);
        auto workspace = arcgen::utils::mazeWorkspace ();

        // Validation helper: strictly enforce Dubins constraints so it works for ReedsShepp too
        Robot robot = Robot::rectangle (ROBOT_LEN, ROBOT_WID); // Same config

        auto isValid = [&] (State &s, bool isStart) -> bool
        {
            if (!workspace->contains (s.x, s.y))
                return false;
            // Strictly use Dubins-like forward check => guarantee forward reachability
            if (!arcgen::utils::halfDiskInside (*workspace, s.x, s.y, s.heading, R_MIN, isStart))
                return false;
            // Footprint check
            if (!workspace->coveredBy (robot.at (s.x, s.y, s.heading)))
                return false;
            return true;
        };

        auto sample = [&] (State &s, bool isStart) -> bool
        {
            bg::model::box<Point> bb;
            bg::envelope (workspace->region (), bb);
            std::uniform_real_distribution<double> ux (bb.min_corner ().x (), bb.max_corner ().x ());
            std::uniform_real_distribution<double> uy (bb.min_corner ().y (), bb.max_corner ().y ());
            std::uniform_real_distribution<double> uh (0.0, PI2);

            for (int i = 0; i < 5000; ++i)
            {
                s.x = ux (rng);
                s.y = uy (rng);
                s.heading = uh (rng);
                if (isValid (s, isStart))
                    return true;
            }
            return false;
        };

        int casesGenerated = 0;
        while (casesGenerated < NUM_SAMPLES)
        {
            State start{}, goal{};
            // Retry loop for valid pair
            bool pairFound = false;
            for (int k = 0; k < 1000; ++k)
            {
                if (sample (start, true) && sample (goal, false))
                {
                    pairFound = true;
                    break;
                }
            }

            if (pairFound)
            {
                // Try planning with both
                auto dbgDubins = tryPlan<Dubins> (workspace, start, goal);
                if (!dbgDubins)
                    continue; // Dubins failed, skip

                auto dbgReeds = tryPlan<ReedsShepp> (workspace, start, goal);
                if (!dbgReeds)
                    continue; // ReedsShepp failed, skip

                // Both succeeded! Visualize and increment.
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
