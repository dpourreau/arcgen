#pragma once
/**
 * @file   arcgen.hpp
 * @brief  Umbrella header – include the full ArcGen public interface.
 *
 * Include this single header in a translation unit to access the entire API:
 *  - Core primitives (state, math, numeric constants, controls)
 *  - Steering CRTP base and policies (Dubins, Reeds–Shepp)
 *  - Planner module (workspace, straight skeleton, constraints, search engine, graph-search CRTP + A*)
 */

/*──────────────────────────── Core ────────────────────────────*/
#include <arcgen/core/control.hpp>
#include <arcgen/core/math.hpp>
#include <arcgen/core/numeric.hpp>
#include <arcgen/core/state.hpp>

/*────────────────────────── Steering ──────────────────────────*/
#include <arcgen/steering/dubins.hpp>      // Dubins policy (3 segments)
#include <arcgen/steering/reeds_shepp.hpp> // Reeds–Shepp policy (5 segments)
#include <arcgen/steering/steering.hpp>    // CRTP base

/*────────────────────────── Planner ───────────────────────────*/
#include <arcgen/planner/connector/greedy_connector.hpp>
#include <arcgen/planner/constraints/collision.hpp>
#include <arcgen/planner/constraints/footprint_collision.hpp>
#include <arcgen/planner/constraints/path_length.hpp>

namespace arcgen::planner
{
    namespace engine
    {
        enum class PlanningError;
    }
    namespace connector
    {
        template <class Steering, class DebugInfo> class GreedyConnector;
    }
} // namespace arcgen::planner

#include <arcgen/planner/engine/search_engine.hpp>       // Three-stage planner
#include <arcgen/planner/geometry/robot.hpp>             // Polygonal robot + sweep utilities
#include <arcgen/planner/geometry/skeleton.hpp>          // Skeleton CRTP base
#include <arcgen/planner/geometry/straight_skeleton.hpp> // CGAL-based straight skeleton
#include <arcgen/planner/geometry/workspace.hpp>         // Valid-region workspace
#include <arcgen/planner/graph/astar.hpp>                // A* adaptor
#include <arcgen/planner/graph/graph_search.hpp>         // Graph-search CRTP base
