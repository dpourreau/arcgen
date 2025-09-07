# Troubleshooting

Common build/runtime issues and how to fix them.

## CGAL and Boost policy warning

You may see a message about CMake policy `CMP0167` (FindBoost is removed). ArcGen isolates CGAL's internal use of legacy `FindBoost`, so this is benign. If the message is noisy, configure with `-Wno-dev` or ignore.

## Debug build is slow

CGAL warns when building in Debug. For realistic performance, configure with `-DCMAKE_BUILD_TYPE=Release` or use the `release` preset.

## GoogleTest not found

If `AG_BUILD_TESTS=ON` and `GTest::gtest_main` is missing:

- macOS (Homebrew): `brew install googletest`
- Ubuntu/Debian: `sudo apt-get install -y libgtest-dev`

Ensure CMake can find the `GTest` config package (`find_package(GTest CONFIG)`); some distributions require building and installing GTest from sources.

## OpenMP on macOS

Install `libomp` with Homebrew and enable with `-DAG_ENABLE_OPENMP=ON`. If you see undefined OpenMP symbols, confirm your compiler and linker flags include OpenMP support.

## Unknown warning options (Clang/AppleClang)

If you see `unknown-warning-option` when building tests, disable strict warnings with `-DAG_STRICT_WARNINGS=OFF` or use the provided presets. The library target itself does not export these warnings to consumers.

## Tests and plots

Use presets to run tests:

```
cmake --preset debug
cmake --build --preset debug
ctest --preset debug --output-on-failure
```

Enable plot generation with `-DAG_ENABLE_PLOTS=ON` (already ON in the `debug` preset). SVGs are written under `build/<preset>/plots/...`.

