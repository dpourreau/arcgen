# Tests and Plots

ArcGen ships with a comprehensive test suite covering core primitives, steering functions, geometry/skeletons, graph search, and the high-level three-stage engine.

## Running

Use CMake presets:

```
cmake --preset debug
cmake --build --preset debug
ctest --preset debug --output-on-failure
```

`asan`, `release`, `release-tests`, and `release-plots` presets are also available.
-   `debug`, `asan`, and `release-plots` enable plot generation by default.
-   `release` and `release-tests` do not generate plots.

## Plots

When `AG_ENABLE_PLOTS=ON`, SVGs are written under `build/<preset>/plots/...`. The structure mirrors the test suite (e.g., `plots/steering/dubins/*.svg`).

## Statistics Reports

When `AG_ENABLE_TEST_REPORT=ON` (enabled by default in `release-tests`, `release-plots`, etc.), aggregate statistics are generated in `build/<preset>/stats/`:

-   `planner_stats_report.txt`: Human-readable summary of planner performance.
-   `planner_stats.csv`: Raw data for analysis.

These reports include success rates, timing breakdowns, and source distribution (Direct/Local/Global).

## Notes

- Timings are indicative, not benchmarks. Prefer Release builds for performance evaluation.
- Some tests sample random workspaces/poses; seeds are fixed for reproducibility.

