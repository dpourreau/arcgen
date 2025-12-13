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

- Planner (`include/arcgen/planner`)
  - Geometry: `geometry/workspace.hpp` (polygonal valid region with R-tree), `geometry/straight_skeleton.hpp` (CGAL → Boost graph), `geometry/skeleton.hpp` (CRTP façade)
  - Constraints: `constraints.hpp` (interfaces), `collision.hpp` (hard), `footprint_collision.hpp` (hard), `path_length.hpp` (soft)
  - Search: `search/graph_search.hpp` (CRTP), `search/astar.hpp` (Boost.Graph A* adaptor)
  - Engine: `search_engine.hpp` and `connector/greedy_connector.hpp` — three-stage planner orchestrating steering, skeleton generation, and graph search

## Engine Pipeline

Given a start and goal pose, the engine attempts:

1) Direct steering: enumerate candidates and pick the best feasible one per constraints.
2) Local skeleton: build an axis-aligned rectangle around the poses (+ margin) and run A* on its skeleton.
3) Global skeleton: build once for the full workspace and reuse; run A* when local fails.

The final path is constructed by the `Connector`:
- **Greedy Stitching**: Connects consecutive waypoints using the steering policy. It employs a "longest-valid-segment" heuristic (DFS) to skip intermediate waypoints if a direct connection is feasible and beneficial.
- **Smoothing**: An iterative pass that resamples the path (preserving joints) and attempts to replace local segments with direct shortcuts. This reduces path length and unnecessary manoeuvres while maintaining feasibility.

## Constraints

- Hard constraints: must accept a candidate (e.g., collision-free).
- Soft constraints: assign a finite cost; the engine picks the lowest score among feasible candidates.

Constraints are organized into a `ConstraintSet` containing separate vectors for hard and soft constraints to ensure clear distinction during evaluation.

## Performance Notes

- Build in Release for representative timings.
- Tune `ds` (state discretisation) to balance accuracy vs speed.
- Enable OpenMP (`-DAG_ENABLE_OPENMP=ON`) to parallelise candidate evaluation.
- Skeleton graphs avoid duplicate edges; A* uses Euclidean heuristic.
