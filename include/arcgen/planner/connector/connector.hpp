#define ARCGEN_ENGINE_CONNECTOR_GREEDY_CONNECTOR_HPP
#pragma once
/**
 * @file connector.hpp
 * @brief Concept for stitching coarse graph paths into feasible robot paths.
 */

#include <arcgen.hpp>

#include <concepts>
#include <utility>
#include <vector>

namespace arcgen::planner::connector
{
    using arcgen::planner::engine::Evaluator;

    /**
     * @brief Greedy connector that assigns headings to coarse waypoints and stitches them with farthest-reachable jumps.lementations.
     */
    template <class Connector, class Steering, class DebugInfo>
    concept ConnectorLike = requires (const Connector &c, const Evaluator<Steering> &evaluator, const arcgen::core::State &start, const arcgen::core::State &goal,
                                      std::vector<arcgen::core::State> coarse, DebugInfo *dbg) {
        { c.connect (evaluator, start, goal, std::move (coarse), dbg) } -> std::same_as<std::vector<arcgen::core::State>>;
    };

} // namespace arcgen::planner::connector
