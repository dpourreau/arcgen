#include <arcgen.hpp>

#include <utils/output_paths.hpp>
#include <utils/visualizer.hpp>
#include <utils/workspace_generators.hpp>

#include <boost/geometry.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <limits>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using namespace arcgen::core;
using namespace arcgen::planner::geometry;

using arcgen::utils::Visualizer;

namespace bg = boost::geometry;

/* ───────── parameters ───────── */
constexpr double MIN_SEP = 8.0; ///< [m] minimum start↔goal separation
constexpr int SAMPLES = 50;     ///< queries per engine
constexpr double STEP_L = 0.20; ///< [m] straight-line discretisation
constexpr double EPS_XY = 1e-3; ///< [m] state↔vertex tolerance

/* ───────── helpers ───────── */
template <class Graph> static std::optional<typename boost::graph_traits<Graph>::vertex_descriptor> nearestVertex (const Graph &g, double x, double y)
{
    using V = typename boost::graph_traits<Graph>::vertex_descriptor;
    double best = std::numeric_limits<double>::max ();
    std::optional<V> v{};
    for (V vv : boost::make_iterator_range (vertices (g)))
    {
        const Point &p = g[vv];
        const double d2 = (p.x () - x) * (p.x () - x) + (p.y () - y) * (p.y () - y);
        if (d2 < best)
        {
            best = d2;
            v = vv;
        }
    }
    if (best < EPS_XY * EPS_XY)
        return v;
    return std::nullopt;
}

static bool straightInside (const Workspace &W, const State &a, const State &b)
{
    const double L = std::hypot (b.x - a.x, b.y - a.y);
    const int N = std::max (1, static_cast<int> (std::ceil (L / STEP_L)));
    for (int k = 1; k < N; ++k)
    {
        const double t = static_cast<double> (k) / static_cast<double> (N);
        const double x = (1.0 - t) * a.x + t * b.x;
        const double y = (1.0 - t) * a.y + t * b.y;
        if (!W.contains (x, y))
            return false;
    }
    return true;
}

/* ───────── name tag for nice labels ───────── */
template <class> struct NameTag
{
    static constexpr const char *str = "Generic";
};

using AStarSearch = arcgen::planner::graph::AStar<arcgen::planner::geometry::Graph>;
template <> struct NameTag<AStarSearch>
{
    static constexpr const char *str = "AStar";
};

/**
 * @brief Typed fixture for timing graph-search backends on a fixed maze.
 * @tparam SearchT Graph-search adaptor type (CRTP over Boost.Graph).
 */
template <class SearchT> class MazeTimingFixture : public ::testing::Test
{
  protected:
    using Search = SearchT;
    using Graph = arcgen::planner::geometry::Graph;

    inline static Workspace W_{*arcgen::utils::mazeWorkspace ()};
    inline static arcgen::planner::geometry::StraightSkeleton skel_;
    inline static Graph G_{skel_.generate (W_)};

    MazeTimingFixture () : rng_ (seedCounter_++) {}

    void runOne ([[maybe_unused]] int id)
    {
        State s{}, g{};
        sample (s);
        do
        {
            sample (g);
        } while (std::hypot (s.x - g.x, s.y - g.y) < MIN_SEP);

        std::vector<State> coarse;
        {
            coarse = search_.search (G_, s, g);
        }

        bool ok = !coarse.empty ();
        if (!ok && straightInside (W_, s, g))
            ok = true;

        if (ok && !coarse.empty ())
        {
            for (auto &st : coarse)
            {
                if (!nearestVertex (G_, st.x, st.y))
                {
                    ok = false;
                    break;
                }
            }
        }

#ifdef AG_ENABLE_PLOTS
        std::ostringstream fn;
        fn << NameTag<Search>::str << '_' << (ok ? "ok_" : "fail_") << id << ".svg";
        auto outPath = arcgen::utils::plotFile ({"search", NameTag<Search>::str}, fn.str ());
        Visualizer svg (outPath.string (), 900);
        svg.drawRegion (W_);
        svg.drawSkeleton (G_);
        if (coarse.empty ())
        {
            std::array<State, 2> line{s, g};
            svg.drawPath (std::span<const State> (line.data (), line.size ()));
        }
        else
        {
            std::vector<State> p{s};
            p.insert (p.end (), coarse.begin (), coarse.end ());
            p.push_back (g);
            svg.drawPath (p);
        }
        svg.drawStartPose (s);
        svg.drawGoalPose (g);
        svg.finish ();
#endif
        ASSERT_TRUE (ok);
    }

  private:
    void sample (State &st)
    {
        bg::model::box<Point> bb;
        bg::envelope (W_.region (), bb);
        std::uniform_real_distribution<double> ux (bb.min_corner ().x (), bb.max_corner ().x ());
        std::uniform_real_distribution<double> uy (bb.min_corner ().y (), bb.max_corner ().y ());
        std::uniform_real_distribution<double> uh (0.0, two_pi);

        do
        {
            st.x = ux (rng_);
            st.y = uy (rng_);
        } while (!W_.contains (st.x, st.y));
        st.heading = uh (rng_);
    }

    Search search_;
    std::mt19937 rng_;
    inline static unsigned seedCounter_ = 1;
};

/* ───────── which engines to time ───────── */
using TestedSearches = ::testing::Types<AStarSearch>;
TYPED_TEST_SUITE (MazeTimingFixture, TestedSearches);

/* ───────── actual benchmark ───────── */
TYPED_TEST (MazeTimingFixture, GuidelineOrDirectLine)
{
    for (int k = 0; k < SAMPLES; ++k)
        this->runOne (k);
}
