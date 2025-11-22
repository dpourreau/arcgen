#pragma once
/**
 * @file greedy_connector.hpp
 * @brief Greedy farthest-reachable connector (default stitching strategy).
 */

#include <arcgen/planning/engine/connector/connector.hpp>

#include <arcgen/core/math.hpp>
#include <arcgen/core/numeric.hpp>

#include <cmath>
#include <type_traits>
#include <utility>
#include <vector>

namespace arcgen::planning::engine::connector
{
    /**
     * @brief Greedy connector that assigns headings to coarse waypoints and stitches them with farthest-reachable jumps.
     */
    template <class Steering, class DebugInfo = void> class GreedyConnector final
    {
      public:
        [[nodiscard]] std::vector<arcgen::core::State> connect (const Evaluator<Steering> &evaluator, const arcgen::core::State &start, const arcgen::core::State &goal,
                                                                std::vector<arcgen::core::State> coarse, DebugInfo *dbg = nullptr) const
        {
            assignHeadings (coarse);

            std::vector<arcgen::core::State> waypoints;
            waypoints.reserve (coarse.size () + 2);
            waypoints.push_back (start);
            waypoints.insert (waypoints.end (), coarse.begin (), coarse.end ());
            waypoints.push_back (goal);

            if constexpr (!std::is_void_v<DebugInfo>)
            {
                if (dbg)
                {
                    dbg->coarse = std::move (coarse);
                    dbg->waypoints = waypoints;
                    dbg->stitchedPairs.clear ();
                }
            }

            return stitch (waypoints, evaluator, dbg);
        }

      private:
        static void assignHeadings (std::vector<arcgen::core::State> &pts);

        std::vector<arcgen::core::State> stitch (const std::vector<arcgen::core::State> &nodes, const Evaluator<Steering> &evaluator, DebugInfo *dbg) const
        {
            const std::size_t count = nodes.size ();
            if (count < 3)
                return {};

            std::vector<arcgen::core::State> out;
            [[maybe_unused]] std::vector<std::pair<std::size_t, std::size_t>> stitched;
            std::vector<bool> triedFail (count * count, false);

            auto reachable = [&] (std::size_t i, std::size_t j) -> bool
            {
                if (triedFail[i * count + j])
                    return false;

                if (auto best = evaluator.bestStatesBetween (nodes[i], nodes[j]))
                {
                    // merge states; avoid duplicating the junction point
                    if (!out.empty () && !best->empty ())
                        best->erase (best->begin ());

                    out.insert (out.end (), best->begin (), best->end ());

                    if constexpr (!std::is_void_v<DebugInfo>)
                    {
                        if (dbg)
                            stitched.emplace_back (i, j);
                    }
                    return true;
                }
                triedFail[i * count + j] = true;
                return false;
            };

            std::size_t i = 0;
            while (i < count - 1)
            {
                std::size_t j = i + 1;

                // Try the farthest reachable node first, walking backward.
                for (std::size_t k = count - 1; k > i; --k)
                {
                    if (i == 0 && k == count - 1)
                        continue;
                    if (reachable (i, k))
                    {
                        j = k;
                        break;
                    }
                }

                // If the immediate neighbor was tried and failed, no path exists.
                if (j == i + 1 && triedFail[i * count + j])
                    return {};

                i = j;
            }

            if constexpr (!std::is_void_v<DebugInfo>)
            {
                if (dbg)
                    dbg->stitchedPairs = std::move (stitched);
            }
            return out;
        }
    };

    template <class Steering, class DebugInfo> void GreedyConnector<Steering, DebugInfo>::assignHeadings (std::vector<arcgen::core::State> &pts)
    {
        const std::size_t n = pts.size ();
        if (n < 2)
            return;

        for (std::size_t i = 0; i + 1 < n; ++i)
        {
            const double dx = pts[i + 1].x - pts[i].x;
            const double dy = pts[i + 1].y - pts[i].y;

            if (std::fabs (dx) < arcgen::core::CURVATURE_TOL && std::fabs (dy) < arcgen::core::CURVATURE_TOL)
            {
                if (i > 0)
                    pts[i].heading = pts[i - 1].heading;
                continue;
            }

            pts[i].heading = arcgen::core::normalizeAngleSigned (std::atan2 (dy, dx));
        }

        pts.back ().heading = pts[n - 2].heading;
    }

} // namespace arcgen::planning::engine::connector
