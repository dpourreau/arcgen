#pragma once
/**
 * @file   workspace_generators.hpp
 * @brief  Small library of reproducible workspace builders for tests.
 *
 * All polygons are CCW and closed. Helpers return shared_ptr<Workspace>
 * to make passing around cheap and convenient.
 */

#include <arcgen.hpp>
#include <random>

namespace test_helpers
{
    using namespace arcgen::geometry;
    using arcgen::core::PI;
    using arcgen::core::PI2;

    /**
     * @brief Axis-aligned rectangle polygon (closed).
     * @param xMin Minimum x.
     * @param yMin Minimum y.
     * @param xMax Maximum x.
     * @param yMax Maximum y.
     * @return CCW, closed rectangle polygon.
     */
    inline Polygon rectPoly (double xMin, double yMin, double xMax, double yMax)
    {
        Polygon p;
        p.outer () = {{xMin, yMin}, {xMax, yMin}, {xMax, yMax}, {xMin, yMax}, {xMin, yMin}};
        boost::geometry::correct (p);
        return p;
    }

    /**
     * @brief Random “spiky” outer polygon plus a random number of box holes.
     * @param rng  PRNG used for reproducibility.
     * @return Workspace with holes.
     */
    inline std::shared_ptr<Workspace> randomWorkspace (std::mt19937 &rng)
    {
        std::uniform_real_distribution<> uni (0.0, 1.0);

        Polygon outer;
        outer.outer () = {{0, 0}, {15, 5}, {20, 0}, {10, -5}, {20, -20}, {40, -20}, {40, 40}, {5, 35}, {0, 30}, {0, 0}};
        boost::geometry::correct (outer);

        const int holeCount = 1 + static_cast<int> (uni (rng) * 5.0);
        std::vector<Polygon> holes;
        holes.reserve (static_cast<std::size_t> (holeCount));

        for (int i = 0; i < holeCount; ++i)
        {
            const double w = 2.0 + uni (rng) * 3.0;
            const double h = 2.0 + uni (rng) * 3.0;
            const double xx = 3.0 + uni (rng) * 30.0;
            const double yy = -15.0 + uni (rng) * 45.0;
            holes.push_back (rectPoly (xx, yy, xx + w, yy + h));
        }
        return std::make_shared<Workspace> (std::move (outer), std::move (holes));
    }

    /**
     * @brief Narrow corridor maze: outer box with alternating slits.
     * @return Workspace for path-planning stress tests.
     */
    inline std::shared_ptr<Workspace> mazeWorkspace ()
    {
        constexpr double W = 60.0, H = 40.0;
        Polygon outer = rectPoly (0, 0, W, H);

        constexpr int L = 10;
        constexpr double THICKNESS = 0.8;
        constexpr int SEGMENTS = 8;

        std::vector<Polygon> holes;
        const double dx = W / (L + 1), dy = H / SEGMENTS;

        for (int i = 1; i <= L; ++i)
        {
            const double x = i * dx;
            for (int j = 0; j < SEGMENTS; ++j)
            {
                if ((i + j) & 1)
                    continue; // leave a gap
                const double y0 = j * dy;
                holes.push_back (rectPoly (x - THICKNESS / 2.0, y0, x + THICKNESS / 2.0, y0 + dy));
            }
        }
        return std::make_shared<Workspace> (std::move (outer), std::move (holes));
    }

    /**
     * @brief Approximate circle polygon with N vertices (closed).
     * @param cx  Center X
     * @param cy  Center Y
     * @param r   Radius
     * @param n   Number of samples along the circle (≥ 3 recommended)
     */
    inline Polygon circlePoly (double cx, double cy, double r, int n = 20)
    {
        Polygon p;
        auto &ring = p.outer ();
        ring.reserve (static_cast<std::size_t> (n + 1));
        for (int k = 0; k < n; ++k)
        {
            const double a = PI2 * static_cast<double> (k) / static_cast<double> (n);
            ring.emplace_back (cx + r * std::cos (a), cy + r * std::sin (a));
        }
        ring.push_back (ring.front ());
        boost::geometry::correct (p);
        return p;
    }

    /**
     * @brief “Gear” shaped outer polygon with two circular holes.
     */
    inline std::shared_ptr<Workspace> gearWorkspace ()
    {
        Polygon outer;
        auto &r = outer.outer ();
        constexpr int N = 10;
        constexpr double R1 = 40.0, R2 = 25.0;
        for (int k = 0; k < N; ++k)
        {
            const double a = PI2 * static_cast<double> (k) / static_cast<double> (N);
            const double R = (k & 1) ? R2 : R1;
            r.emplace_back (R * std::cos (a), R * std::sin (a));
        }
        r.push_back (r.front ());
        boost::geometry::correct (outer);

        std::vector<Polygon> holes;
        holes.push_back (circlePoly (10.0, 0.0, 6.0, 24));
        holes.push_back (circlePoly (-12.0, -5.0, 5.0, 24));

        return std::make_shared<Workspace> (std::move (outer), std::move (holes));
    }
} // namespace test_helpers
