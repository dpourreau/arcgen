# Tests and Plots

ArcGen ships with property/timing tests for steering, geometry (skeleton), A* graph search, and the end-to-end engine.

## Running

Use CMake presets:

```
cmake --preset debug
cmake --build --preset debug
ctest --preset debug --output-on-failure
```

`asan` and `release` presets are also available. The `debug` and `asan` presets enable plot generation by default.

## Plots

When `AG_ENABLE_PLOTS=ON`, SVGs are written under `build/<preset>/plots/...`. The structure mirrors the test suite (e.g., `plots/steering/dubins/*.svg`).

## Notes

- Timings are indicative, not benchmarks. Prefer Release builds for performance evaluation.
- Some tests sample random workspaces/poses; seeds are fixed for reproducibility.

