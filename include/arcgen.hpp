#pragma once
/**
 * @file   arcgen.hpp
 * @brief  Umbrella header – include the full ArcGen public interface.
 *
 * Include this single header in a translation unit to access the entire API:
 *  - Core primitives (state, math, numeric constants, controls)
 *  - Steering CRTP base and policies (Dubins, Reeds–Shepp)
 *  - Geometry (workspace, straight skeleton + CRTP base)
 *  - Planning (constraints, search engine, graph-search CRTP + A*)
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

/*────────────────────────── Geometry ──────────────────────────*/
#include <arcgen/geometry/robot.hpp>             // Polygonal robot + sweep utilities
#include <arcgen/geometry/skeleton.hpp>          // Skeleton CRTP base
#include <arcgen/geometry/straight_skeleton.hpp> // CGAL-based straight skeleton
#include <arcgen/geometry/workspace.hpp>         // Valid-region workspace

/*────────────────────────── Planning ──────────────────────────*/
#include <arcgen/planning/constraints/collision.hpp>
#include <arcgen/planning/constraints/footprint_collision.hpp>
#include <arcgen/planning/constraints/path_length.hpp>
#include <arcgen/planning/engine/search_engine.hpp>
#include <arcgen/planning/search/astar.hpp>        // A* adaptor
#include <arcgen/planning/search/graph_search.hpp> // Graph-search CRTP base
