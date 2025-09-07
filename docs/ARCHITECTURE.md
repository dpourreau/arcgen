# ArcGen Architecture

This document provides a deeper look at how ArcGen is structured and how the major components fit together.

## Modules

- Core (`include/arcgen/core`)
  - `state.hpp`: SE(2) state with curvature and direction flags
  - `control.hpp`: constant-curvature primitives (`Control`, `ControlSeq<N>`)
  - `math.hpp`: numeric helpers (angle wrapping, arc/line endpoints)
  - `numeric.hpp`: constants and tolerances used across the project

- Steering (`include/arcgen/steering`)
  - `steering.hpp`: CRTP base that converts geometric arcs into concrete controls and discretised states (using `ds`)
  - `dubins.hpp` / `reeds_shepp.hpp`: shortest path generators producing fixed-length arc patterns
  - `path.hpp`: candidate container with controls and lazily-computed states

- Geometry (`include/arcgen/geometry`)
  - `workspace.hpp`: polygonal valid region (outer minus holes), with an R-tree for fast point queries
  - `straight_skeleton.hpp`: builds an interior straight skeleton (CGAL) and converts to a Boost undirected graph with Euclidean edge weights
  - `skeleton.hpp`: CRTP façade for skeleton generators

- Planning (`include/arcgen/planning`)
  - Constraints: `constraints.hpp` (interfaces), `collision.hpp` (hard), `path_length.hpp` (soft)
  - Search: `graph_search.hpp` (CRTP), `astar.hpp` (Boost.Graph A* adaptor)
  - Engine: `engine/search_engine.hpp` — 3-stage planner orchestrating steering, skeleton generation, and graph search

## Engine Pipeline

Given a start and goal pose, the engine attempts:

1) Direct steering: enumerate candidates and pick the best feasible one per constraints.
2) Local skeleton: build an axis-aligned rectangle around the poses (+ margin) and run A* on its skeleton.
3) Global skeleton: build once for the full workspace and reuse; run A* when local fails.

The final path is formed by greedy stitching of steering segments between waypoints (start, A* vertices, goal).

## Constraints

- Hard constraints: must accept a candidate (e.g., collision-free).
- Soft constraints: assign a finite cost; the engine picks the lowest score among feasible candidates.

## Performance Notes

- Build in Release for representative timings.
- Tune `ds` (state discretisation) to balance accuracy vs speed.
- Enable OpenMP (`-DAG_ENABLE_OPENMP=ON`) to parallelise candidate evaluation.
- Skeleton graphs avoid duplicate edges; A* uses Euclidean heuristic.

