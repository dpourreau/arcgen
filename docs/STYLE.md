# Coding, Documentation, and Formatting Guidelines

This document describes the house style for ArcGen. It complements the existing docs and helps keep the codebase consistent and easy to maintain.

## Naming Conventions

Use the following conventions across all production code and tests:

- Classes / Structs: PascalCase (e.g., `SearchEngine`, `Robot`)
- Functions / Methods: camelCase (e.g., `plan`, `rectangle`, `coveredBy`)
- Variables: camelCase (e.g., `radiusMin`, `workspace`, `halfLength`)
- Constants: ALL_CAPS_WITH_UNDERSCORES (e.g., `PI2`, `R_MIN`, `EPS_GOAL`)
- Private Member Variables: camelCase_ with trailing underscore (e.g., `workspace_`, `robot_`)
- Namespaces: snake_case (e.g., `arcgen::planner::geometry`, `arcgen::planner`)
- Enums: `enum class` PascalCase with PascalCase enumerators (e.g., `enum class StageUsed { Direct, Local, Global };`)

Prefer clear, descriptive identifiers over abbreviations. Loop indices like `i`, `j`, `k` are acceptable where customary.

## Doxygen Documentation

Every public class, public function, and important internal logic should include Doxygen comments. Use the multi‑line form for non‑trivial APIs, and `///` for short one‑liners.

Example (multi‑line):

```cpp
/**
 * @brief Compute the world-frame polygon at a given pose.
 * @param x        World X position.
 * @param y        World Y position.
 * @param heading  Heading angle (rad).
 * @return Transformed polygon in world frame.
 */
[[nodiscard]] Polygon at(double x, double y, double heading) const;
```

Example (one‑liner):

```cpp
/// @brief Access the precomputed valid region (multi-polygon).
[[nodiscard]] const MultiPolygon& region() const noexcept;
```

Guidelines:

- Match parameter names in comments and in signatures.
- Prefer `[[nodiscard]]` and `noexcept` where appropriate.
- Document important invariants, preconditions, and non-obvious implementation details.

## Enhancements and Refactoring

When code is ambiguous or poorly structured, refactor it to improve clarity and maintainability. It is acceptable to:

- Clarify variable names (`a`, `b` → `start`, `goal`; `hx` → `halfLength`).
- Extract helpers for readability when it avoids duplication.
- Improve or add Doxygen where it is missing or unclear.

You may use or extend utilities in:

- `include/arcgen/core/math.hpp`
- `include/arcgen/core/numeric.hpp`

Prefer the shared numeric constants in `numeric.hpp` (e.g., `PI2`, `CURVATURE_TOL`, `EPSILON`) to ad‑hoc literals.

## Formatting (.clang-format)

The repository includes a root-level `.clang-format`. Always format any changed lines/files before submitting changes:

```bash
clang-format -i path/to/changed_file.hpp path/to/changed_file.cpp
```

Most editors can be configured to run `clang-format` on save. Keep diffs focused on substantive changes by avoiding unrelated reformatting.

## Additional Recommendations

- Headers should be self-contained and include only what they use.
- Keep changes minimal and focused on the task at hand.
- Maintain exception safety (`noexcept` where possible) and correctness first; optimize only when needed.
- For geometry: ensure polygons are CCW and closed; call `boost::geometry::correct()` where appropriate.
- Prefer explicit units and clarify geometric frames (e.g., body-frame vs world-frame) in comments.

## Safe Coding Standards (MISRA C++23)

ArcGen prioritizes safety and reliability. We follow MISRA C++23 guidelines where applicable, enforced via SonarCloud.

### Key Rules
- **Namespace Pollution (cpp:S1003)**: Never use `using namespace` in header files. Fully qualify types or use type aliases within a class/function scope.
- **Static Initialization (cpp:S2629)**: Avoid order-dependent static initialization. Use the "Construct On First Use" idiom (static variable inside a function) for static globals.
- **Exception Safety (cpp:S5018)**: Move constructors and move assignment operators must be `noexcept`. Use the Rule of Five defaults where possible.
- **Cognitive Complexity (cpp:S3776, cpp:S134)**: Keep functions simple.
    - Max Cognitive Complexity: 25.
    - Max Nesting Depth: 3.
- **Functional Interfaces (cpp:S6045)**: Prefer templates over `std::function` for callback parameters to avoid overhead and virtual calls.
- **Lambda Captures (cpp:S5019)**: Avoid implicit captures (`[&]` or `[=]`). Explicitly capture required variables to prevent dangling references and accidental copies.

