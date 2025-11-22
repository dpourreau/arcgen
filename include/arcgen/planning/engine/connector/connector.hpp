#pragma once
/**
 * @file connector.hpp
 * @brief Concept for stitching coarse graph paths into feasible robot paths.
 */

#include <arcgen/core/state.hpp>
#include <arcgen/planning/engine/evaluator.hpp>

#include <concepts>
#include <utility>
#include <vector>

namespace arcgen::planning::engine::connector
{
    /**
     * @brief Compile-time contract for connector implementations.
     */
    template <class Connector, class Steering, class DebugInfo>
    concept ConnectorLike = requires (const Connector &c, const Evaluator<Steering> &evaluator, const arcgen::core::State &start, const arcgen::core::State &goal,
                                      std::vector<arcgen::core::State> coarse, DebugInfo *dbg) {
        { c.connect (evaluator, start, goal, std::move (coarse), dbg) } -> std::same_as<std::vector<arcgen::core::State>>;
    };

} // namespace arcgen::planning::engine::connector
