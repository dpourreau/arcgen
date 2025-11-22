# ArcGen

ArcGen is a **header-only** C++23 library for car-like path planning with bounded curvature. It gives you:

* robust **Dubins** (forward-only) and **Reeds–Shepp** (forward & reverse) steering policies,
* a clean **CRTP** design to enumerate and integrate motion primitives,
* a polygonal **Workspace** with fast point-in-region queries (Boost.Geometry),
* a CGAL-powered **straight skeleton** → Boost.Graph for roadmap guidance,
* a composable **constraint** system (hard & soft),
* a three-stage **SearchEngine** that tries (1) direct steering, (2) local skeleton, (3) global skeleton,
* optional **OpenMP** parallelism for picking the best steering candidate,
* tidy tests and optional SVG plotting helpers.

ArcGen is designed to be easy to drop into an existing CMake project via `find_package(arcgen)` and link as `arcgen::arcgen`.

## Features

* **Header-only API**: include `<arcgen.hpp>` and go.
* **C++23 concepts** to validate pluggable pieces (Steering, Skeleton, GraphSearch).
* **Numerically careful** primitives (angle wrapping, tolerances, curvature handling).
* **Workspaces with holes**: build `Workspace` from an outer polygon minus any number of obstacle polygons.
* **Straight skeletons** (CGAL) exported to a **Boost undirected graph** with Euclidean edge weights.
* **A\*** adaptor over Boost.Graph with a straight-line heuristic.
* **Constraints**

  * Hard: e.g., collision feasibility against the workspace.
  * Soft: weighted costs (e.g., total path length).
* **Search engine pipeline**: direct → local (AABB around poses) → global (cached), with greedy stitching.
* **Tests & plots**: property, timing, and end-to-end tests; optional SVG visualizer for quick inspection.

## Requirements

* **Tooling**

  * **CMake ≥ 3.23**
  * A modern **C++23** compiler
* **Libraries**

  * **Boost** (headers): Geometry & Graph (1.72+ recommended)
  * **CGAL** (config package) — used by the straight-skeleton generator (5.x+ recommended)
  * **OpenMP** (optional) — only if you enable `AG_ENABLE_OPENMP`
  * **GoogleTest** (tests only; found via `find_package(GTest CONFIG)`)

> ℹ️ CGAL may internally use legacy `FindBoost`; this project isolates that with a local policy change so your build stays clean.

## Install dependencies

ArcGen is header-only, but you must install CGAL, Boost (headers), and (optionally) OpenMP & GoogleTest.

### macOS (Homebrew)

```bash
brew update
brew install cmake boost cgal
# optional
brew install libomp googletest
```

### Ubuntu/Debian

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libboost-dev libcgal-dev
# optional
sudo apt-get install -y libomp-dev libgtest-dev
```

> OpenMP is optional; enable with `-DAG_ENABLE_OPENMP=ON`.

## Build & Install

Using presets:

```bash
# Release
cmake --preset release
cmake --build --preset release
cmake --install build/release
```

Or the classic way:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$HOME/aabb"
# if AG_BUILD_TESTS=ON
cmake --build build
cmake --install build
```

Build tests & run:

```bash
# Debug (AG_BUILD_TESTS ON in this presets)
cmake --preset debug        
cmake --build --preset debug
ctest --preset debug --output-on-failure
```

## Quick start

```cpp
// main.cpp
#include <arcgen.hpp>
#include <boost/geometry.hpp>

#include <iostream>
#include <memory>
#include <vector>

using namespace arcgen::core;
using namespace arcgen::planner::geometry;
using arcgen::planner::engine::SearchEngine;
using arcgen::planner::search::AStar;
namespace connector = arcgen::planner::engine::connector;
using arcgen::steering::Dubins;
namespace C  = arcgen::planner::constraints;
namespace bg = boost::geometry;

int main() {
  // 1) Workspace: rectangle with two box holes (all CCW, closed)
  Polygon outer;
  outer.outer() = {{0,0}, {40,0}, {40,30}, {0,30}, {0,0}};
  Polygon hole1; hole1.outer() = {{10,8}, {15,8}, {15,12}, {10,12}, {10,8}};
  Polygon hole2; hole2.outer() = {{22,16}, {28,16}, {28,24}, {22,24}, {22,16}};
  bg::correct(outer);
  bg::correct(hole1);
  bg::correct(hole2);

  auto workspace = std::make_shared<Workspace>(
      std::move(outer),
      std::vector<Polygon>{std::move(hole1), std::move(hole2)});

  // 2) Engine = Dubins + A* + StraightSkeleton
  using Graph = arcgen::planner::geometry::Graph;
  using Engine = SearchEngine<Dubins, AStar<Graph>, StraightSkeleton>;
  auto engine = Engine(
      std::make_shared<Dubins>(/*rMin=*/2.5, /*ds=*/0.10),
      std::make_shared<AStar<Graph>>(),
      std::make_shared<StraightSkeleton>(),
      workspace,
      std::make_shared<connector::GreedyConnector<Dubins, Engine::DebugInfo>>());

  // 3) Constraints: collision (hard) with a polygonal robot + path length (soft)
  C::ConstraintSet<Dubins::kSegments> cs;
  Robot robot = Robot::rectangle(/*length*/2.0, /*width*/0.8);
  // For polygonal footprint collision, use FootprintCollisionConstraint
  cs.hard.push_back(std::make_shared<C::FootprintCollisionConstraint<Dubins::kSegments>>(workspace, robot));
  cs.soft.push_back({std::make_shared<C::PathLengthConstraint<Dubins::kSegments>>(), 1.0});
  engine.setConstraints(std::move(cs));

  // 4) Plan between poses
  State start{2,  2, 0.0};
  State goal {35, 25, 0.0};
  auto result = engine.plan(start, goal);

  if (!result) {
    std::cerr << "No path found. Error code: " << (int)result.error() << "\n";
    return 1; // failure
  }

  auto path = *result;
  std::cout << "Path states: " << path.size() << "\n";
  return 0; // success
}
```

Compile & link in your app:

```cmake
find_package(arcgen REQUIRED)
add_executable(demo main.cpp)
target_link_libraries(demo PRIVATE arcgen::arcgen)
```

Performance tips:

- Prefer `-DCMAKE_BUILD_TYPE=Release` when timing or benchmarking.
- Enable OpenMP with `-DAG_ENABLE_OPENMP=ON` to parallelise candidate scoring.

## CMake integration (find\_package + target usage)

ArcGen installs a CMake package config:

```cmake
find_package(arcgen REQUIRED)
target_link_libraries(<your-target> PRIVATE arcgen::arcgen)
```

Because ArcGen is header-only, the imported target primarily propagates include paths and link interface requirements (e.g., `CGAL::CGAL`, OpenMP if enabled). You still control your compiler flags as usual.

## Build options & presets

CMake options (all `OFF` by default unless noted):

* `AG_BUILD_TESTS` — build unit/property/timing tests (requires GTest).
* `AG_ENABLE_PLOTS` — enable SVG plot generation inside tests.
* `AG_STRICT_WARNINGS` (**ON** by default) — a strict warnings profile.
* `AG_ENABLE_IPO` — turn on LTO/IPO if supported.
* `AG_ENABLE_OPENMP` — enable OpenMP in ArcGen (and propagate flags).

Presets (`CMakePresets.json`) you can use out-of-the-box:

* `debug` — Debug, tests ON, plots ON, OpenMP OFF
* `release` — Release, OpenMP OFF
* `asan` — Debug with Address/UB sanitizers

## Repository layout

```
.
├── CMakeLists.txt
├── CMakePresets.json
├── cmake/
│   └── arcgenConfig.cmake.in
├── docs/
│   ├── ARCHITECTURE.md                # High-level design overview
│   ├── STYLE.md                       # Coding, docs, and formatting rules
│   └── TROUBLESHOOTING.md             # Build and usage tips
├── include/
│   └── arcgen/
│       ├── arcgen.hpp                 # Umbrella header
│       ├── core/
│       │   ├── control.hpp            # Control & ControlSeq
│       │   ├── math.hpp               # angle helpers, endpoints, etc.
│       │   ├── numeric.hpp            # constants & tolerances
│       │   └── state.hpp              # pose/curvature/direction
│       ├── planner/
│       │   ├── constraints/
│       │   │   ├── collision.hpp
│       │   │   ├── constraints.hpp    # Hard/Soft interfaces
│       │   │   ├── footprint_collision.hpp
│       │   │   └── path_length.hpp
│       │   ├── connector/             # Stitching strategies
│       │   │   ├── connector.hpp      # Connector concept
│       │   │   └── greedy_connector.hpp
│       │   ├── geometry/
│       │   │   ├── robot.hpp          # Polygonal robot model + transforms
│       │   │   ├── skeleton.hpp       # CRTP base
│       │   │   ├── straight_skeleton.hpp  # CGAL → Boost.Graph
│       │   │   └── workspace.hpp      # polygon set + queries
│       │   ├── search/
│       │   │   ├── astar.hpp          # A* adaptor
│       │   │   └── graph_search.hpp   # CRTP base
│       │   ├── evaluator.hpp          # Constraint-aware candidate selector
│       │   └── search_engine.hpp      # 3-stage planner
│       └── steering/
│           ├── dubins.hpp
│           ├── reeds_shepp.hpp
│           ├── path.hpp               # Path container
│           └── steering.hpp           # CRTP base
├── tests/
│   ├── common/
│   │   ├── plot_dir.hpp
│   │   ├── pose_sampling.hpp
│   │   ├── test_stats.hpp
│   │   ├── visualizer.hpp
│   │   └── workspace_generators.hpp
│   ├── geometry/
│   │   ├── robot_tests.cpp
│   │   └── skeleton_tests.cpp
│   ├── planning/
│   │   ├── engine/search_engine_tests.cpp
│   │   └── search/graph_search_tests.cpp
│   └── steering/
│       └── steering_tests.cpp
└── README.md
```

## Tests

ArcGen ships with four small but meaningful test suites (enabled when `AG_BUILD_TESTS=ON`):

- **Steering** (`tests/steering/steering_tests.cpp`)  
  Property tests for Dubins & Reeds–Shepp: goal proximity, length consistency, monotone steps; prints a timing summary for `shortestPath()`.

- **Geometry / Skeleton** (`tests/geometry/skeleton_tests.cpp`)  
  Validates global & local straight-skeleton graphs: vertices lie inside the valid region and edge weights match Euclidean lengths.

- **Graph search** (`tests/planning/search/graph_search_tests.cpp`)  
  A* on a maze skeleton: verifies that returned waypoints correspond to graph vertices; collects timing stats.

- **End-to-end engine** (`tests/planning/engine/search_engine_tests.cpp`)  
  Full pipeline on random/maze/gear workspaces: feasibility, goal tolerance, global-skeleton caching; prints planning-time stats.

**Tips**

- Use the `asan` preset for sanitizer builds: `cmake --preset asan && cmake --build --preset asan && ctest --preset asan`.
- Plots are off by default; enable with `-DAG_ENABLE_PLOTS=ON` to get SVGs under `build/<preset>/plots/...`.

## Documentation

- `docs/STYLE.md` — Coding, documentation, and formatting guidelines.
- `docs/ARCHITECTURE.md` — Overview of ArcGen internals and design.
- `docs/TROUBLESHOOTING.md` — Common build and usage issues.

## Architecture

At a glance:

- Core: math helpers, numeric tolerances, state/control primitives.
- Steering: CRTP base + Dubins/Reeds–Shepp policies that enumerate geometric candidates and discretise states using `ds`.
- Planner: geometry (polygonal `Workspace`, straight skeleton graph), constraints (collision, footprint, path-length), graph-search CRTP + A*, and a 3‑stage engine (direct → local skeleton → global skeleton with greedy stitching).

See `docs/ARCHITECTURE.md` for more.

## Troubleshooting

See `docs/TROUBLESHOOTING.md` for common build and usage issues (CGAL config, GTest, OpenMP, compiler warnings).

## To-Do

- [x] **Robot polygon model**
  - Introduce a `Robot` class with a convex polygon footprint.
  - Update collision checks to treat the robot as an area, not a point.

- [ ] **Shortcuts & smoothing**
  - Geometric shortcutting with feasibility recheck.
  - Curvature-aware smoothing (e.g., cubic/clothoid) while preserving workspace feasibility.

- [ ] **Stitching strategies**
  - Alternatives to greedy farthest-reachable: k-skip lookahead, DP with penalties, bidirectional stitching.

- [ ] **Continuous-curvature steering**
  - Add curvature-continuous (CC-Dubins / CC-Reeds–Shepp) to ensure curvature continuity.
  - Integrate as additional Steering policies alongside Dubins / Reeds–Shepp.

- [ ] **Speed profiler**
  - Time-parameterize paths with curvature/speed limits and jerk/accel bounds.
  - Emit {s, v, a, t} profiles compatible with downstream controllers.

- [ ] **Additional constraints**
  - Polygonal speed-limit zones.
  - Reverse length / gear-change penalty
  - Time cost (when profiler exists)

## License & acknowledgments

* **License:** This project is licensed under the [`MIT License`](./LICENSE).
* **Acknowledgments:** This project builds on fantastic open-source libraries:

  * [CGAL](https://www.cgal.org/) — straight skeletons
  * [Boost](https://www.boost.org/) — Geometry & Graph
  * [GoogleTest](https://github.com/google/googletest) — tests
  * **Steering algorithms** — The Dubins and Reeds–Shepp generators here are a modern, header-only reimplementation inspired by
    [hbanzhaf/steering_functions](https://github.com/hbanzhaf/steering_functions). Many thanks to that project and its contributors.

If you use ArcGen in academic or industrial work, a citation or a star is always appreciated. ✨
