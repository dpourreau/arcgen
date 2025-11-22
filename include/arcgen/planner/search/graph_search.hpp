#pragma once
/**
 * @file
 * @brief CRTP base class for graph-search adaptors.
 *
 * A derived class should implement:
 * \code
 *   std::vector<arcgen::core::State>
 *   searchImpl(const Graph& graph,
 *              const arcgen::core::State& start,
 *              const arcgen::core::State& goal) const;
 * \endcode
 *
 * We intentionally avoid enforcing this with a class-level concept to prevent
 * incomplete-type issues during CRTP inheritance. If the derived class does not
 * provide the required function, you'll get a constrained error at the call site.
 */

#include <arcgen/core/state.hpp>
#include <vector>

namespace arcgen::planner::search
{
    using arcgen::core::State;

    /**
     * @brief Detection concept for the CRTP hook of @ref GraphSearchBase.
     *
     * @tparam D     Candidate derived type.
     * @tparam Graph Boost-like graph type.
     */
    template <class D, class Graph>
    concept HasSearchImpl = requires (const D &d, const Graph &g, const State &s0, const State &s1) {
        { d.searchImpl (g, s0, s1) } -> std::same_as<std::vector<State>>;
    };

    /**
     * @brief CRTP base for graph-search algorithms.
     * @tparam Derived Graph search policy implementing `searchImpl`.
     * @tparam Graph   Boost-like graph type.
     */
    template <typename Derived, typename Graph> class GraphSearchBase
    {
      public:
        /**
         * @brief Run the search on @p graph from @p start to @p goal.
         * @param graph Graph to search.
         * @param start Start state (only x/y are used).
         * @param goal  Goal state (only x/y are used).
         * @return Sequence of intermediate states (start/goal omitted) or empty on failure.
         */
        [[nodiscard]] std::vector<State> search (const Graph &graph, const State &start, const State &goal) const
            noexcept (noexcept (static_cast<const Derived &> (*this).searchImpl (graph, start, goal)))
            requires HasSearchImpl<Derived, Graph>
        {
            return static_cast<const Derived &> (*this).searchImpl (graph, start, goal);
        }

      protected:
        GraphSearchBase () = default;
        ~GraphSearchBase () = default;
    };

} // namespace arcgen::planner::search
