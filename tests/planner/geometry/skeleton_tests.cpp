#include <arcgen.hpp>

#include <common/plot_dir.hpp>
#include <common/test_stats.hpp>
#include <common/visualizer.hpp>
#include <common/workspace_generators.hpp>

#include <boost/geometry.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/range/iterator_range.hpp>

#include <chrono>
#include <filesystem>
#include <gtest/gtest.h>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

using namespace arcgen::planner::geometry;
using namespace arcgen::core;
using test_helpers::RunningStats;
using test_helpers::ScopedTimer;
using test_helpers::Visualizer;

namespace bg = boost::geometry;

/* ───────────── constants (style-uniform) ───────────── */
constexpr double EPS_WEIGHT = 1e-4;       ///< Edge weight ≈ Euclidean length
constexpr int GLOBAL_SAMPLES = 50;        ///< Random workspaces per generator
constexpr int LOCAL_PAIRS = 30;           ///< Start/goal pairs per workspace
constexpr double R_MIN_TEST = 3.0;        ///< [m] steering radius equivalent
constexpr double LOCAL_MARGIN_MULT = 1.0; ///< Multiplier on r_min for local box

/* ───────────── tiny utility ───────────── */
static double edgeLength (const Point &p, const Point &q)
{
    const double dx = q.x () - p.x (), dy = q.y () - p.y ();
    return std::sqrt (dx * dx + dy * dy);
}

/* ───────────── name traits ───────────── */
template <class> struct SkelName
{
    static constexpr const char *value = "generic";
};
template <> struct SkelName<StraightSkeleton>
{
    static constexpr const char *value = "StraightSkeleton";
};

/**
 * @brief Typed fixture validating skeleton generators (global + local) with timing.
 * @tparam SkeletonT  Skeleton generator type under test.
 */
template <class SkeletonT> class SkeletonFixture : public ::testing::Test
{
  protected:
    using Graph = arcgen::planner::geometry::Graph;
    using VDesc = boost::graph_traits<Graph>::vertex_descriptor;
    using EDesc = boost::graph_traits<Graph>::edge_descriptor;

    SkeletonFixture () : rng_ (1337) {}

    /// @brief Print average timings for this skeleton (once per type).
    static void TearDownTestSuite ()
    {
        globalStats_.printSummary (std::string (SkelName<SkeletonT>::value) + " – global skeleton generation");
        localStats_.printSummary (std::string (SkelName<SkeletonT>::value) + " – local skeleton generation");
    }

    /**
     * @brief Run a single global-case check with timing.
     * @param W   Workspace.
     * @param id  Case id (used only for debug/plots).
     * @param id  Label (used only for plots).
     */
    void runGlobalCase (const Workspace &W, int id, std::string_view label)
    {
        Graph G;

        { // time only the generation
            test_helpers::ScopedTimer timer (globalStats_);
            (void)timer;
            G = skel_.generate (W);
        }

        bool ok = true;
        std::ostringstream why;

        if (boost::num_vertices (G) == 0 || boost::num_edges (G) == 0)
        {
            ok = false;
            why << "empty graph; ";
        }

        const auto &region = W.region ();
        for (VDesc v : boost::make_iterator_range (vertices (G)))
        {
            if (!bg::within (G[v], region))
            {
                ok = false;
                why << "vertex outside region; ";
                break;
            }
        }

        auto wmap = get (boost::edge_weight, G);
        for (EDesc e : boost::make_iterator_range (edges (G)))
        {
            const Point &p = G[source (e, G)];
            const Point &q = G[target (e, G)];
            const Point mid{(p.x () + q.x ()) * 0.5, (p.y () + q.y ()) * 0.5};
            if (!bg::within (mid, region))
            {
                ok = false;
                why << "edge crosses obstacle; ";
                break;
            }
            if (std::fabs (wmap[e] - edgeLength (p, q)) > EPS_WEIGHT)
            {
                ok = false;
                why << "edge weight mismatch; ";
                break;
            }
        }

#ifdef AG_ENABLE_PLOTS
        // Optional: plot for debug
        const char *tag = ok ? "ok" : "fail";
        std::ostringstream fn;
        fn << tag << '_' << id << ".svg";
        auto outPath = test_helpers::plotFile ({"skeleton", SkelName<SkeletonT>::value, "global", std::string (label)}, fn.str ());
        Visualizer svg (outPath.string (), 900);
        svg.drawRegion (W);
        svg.drawSkeleton (G);
        svg.drawAxes ();
        svg.finish ();
#endif
        ASSERT_TRUE (ok) << "case " << id << " failed: " << why.str ();
    }

    /**
     * @brief Run local (stage-2 equivalent) checks around random pose pairs.
     */
    void runLocalCases (const Workspace &W, int wid, int pairCount, double rMin, double marginMult, std::string_view label)
    {
        if (W.empty ())
            return;

        bg::model::box<Point> bb;
        bg::envelope (W.region (), bb);
        std::uniform_real_distribution<double> ux (bb.min_corner ().x (), bb.max_corner ().x ());
        std::uniform_real_distribution<double> uy (bb.min_corner ().y (), bb.max_corner ().y ());

        const double margin = marginMult * rMin;
        auto sampleInside = [&] () -> Point
        {
            Point p{};
            do
            {
                p.x (ux (rng_));
                p.y (uy (rng_));
            } while (!W.contains (p.x (), p.y ()));
            return p;
        };

        for (int i = 0; i < pairCount; ++i)
        {
            const Point a = sampleInside ();
            const Point b = sampleInside ();

            const double xMin = std::min (a.x (), b.x ()) - margin;
            const double yMin = std::min (a.y (), b.y ()) - margin;
            const double xMax = std::max (a.x (), b.x ()) + margin;
            const double yMax = std::max (a.y (), b.y ()) + margin;

            Polygon rect;
            rect.outer ().resize (5);
            rect.outer ()[0] = Point{xMin, yMin};
            rect.outer ()[1] = Point{xMax, yMin};
            rect.outer ()[2] = Point{xMax, yMax};
            rect.outer ()[3] = Point{xMin, yMax};
            rect.outer ()[4] = Point{xMin, yMin};
            bg::correct (rect);

            Workspace localW = W.clippedTo (rect);
            if (localW.empty ())
                continue;

            Graph G;
            {
                ScopedTimer t (localStats_);
                (void)t;
                G = skel_.generate (localW);
            }

            bool ok = true;
            std::ostringstream why;

            if (boost::num_vertices (G) == 0 || boost::num_edges (G) == 0)
            {
                ok = false;
                why << "empty local graph; ";
            }

            const auto &region = localW.region ();
            auto wmap = get (boost::edge_weight, G);

            for (VDesc v : boost::make_iterator_range (vertices (G)))
            {
                if (!bg::within (G[v], region))
                {
                    ok = false;
                    why << "local vertex outside region; ";
                    break;
                }
            }

            for (EDesc e : boost::make_iterator_range (edges (G)))
            {
                const Point &p = G[source (e, G)];
                const Point &q = G[target (e, G)];
                const Point mid{(p.x () + q.x ()) * 0.5, (p.y () + q.y ()) * 0.5};
                if (!bg::within (mid, region))
                {
                    ok = false;
                    why << "local edge crosses obstacle; ";
                    break;
                }
                if (std::fabs (wmap[e] - edgeLength (p, q)) > EPS_WEIGHT)
                {
                    ok = false;
                    why << "local edge weight mismatch; ";
                    break;
                }
            }

#ifdef AG_ENABLE_PLOTS
            const char *tag = ok ? "ok_local" : "fail_local";
            std::ostringstream fn;
            fn << tag << "_w" << wid << "_p" << i << ".svg";
            auto outPath = test_helpers::plotFile ({"skeleton", SkelName<SkeletonT>::value, "local", std::string (label)}, fn.str ());
            Visualizer svg (outPath.string (), 900);
            svg.drawRegion (localW);
            svg.drawSkeleton (G);

            State sa{a.x (), a.y (), 0.0}, sb{b.x (), b.y (), 0.0};
            svg.drawStartPose (sa);
            svg.drawGoalPose (sb);

            svg.drawAxes ();
            svg.finish ();
#endif
            ASSERT_TRUE (ok) << "local case W" << wid << "/pair" << i << " failed: " << why.str ();
        }
    }

    std::mt19937 &rng () { return rng_; }
    const std::mt19937 &rng () const { return rng_; }

  private:
    SkeletonT skel_;
    std::mt19937 rng_;

    inline static RunningStats globalStats_{};
    inline static RunningStats localStats_{};
};

using TestedSkeletons = ::testing::Types<StraightSkeleton>;
TYPED_TEST_SUITE (SkeletonFixture, TestedSkeletons);

/* ───────────── global tests ───────────── */
TYPED_TEST (SkeletonFixture, GlobalRandomAndPredefined)
{
    for (int k = 0; k < GLOBAL_SAMPLES; ++k)
        this->runGlobalCase (*test_helpers::randomWorkspace (this->rng ()), k, "random");

    this->runGlobalCase (*test_helpers::mazeWorkspace (), 1001, "maze");
    this->runGlobalCase (*test_helpers::gearWorkspace (), 1002, "gear");
    ;
}

/* ───────────── local tests ───────────── */
TYPED_TEST (SkeletonFixture, LocalAroundRandomPairs)
{
    for (int k = 0; k < GLOBAL_SAMPLES; ++k)
    {
        auto w = *test_helpers::randomWorkspace (this->rng ());
        this->runLocalCases (w, k, LOCAL_PAIRS, R_MIN_TEST, LOCAL_MARGIN_MULT, "random");
    }
    this->runLocalCases (*test_helpers::mazeWorkspace (), 2001, LOCAL_PAIRS, R_MIN_TEST, LOCAL_MARGIN_MULT, "maze");
    this->runLocalCases (*test_helpers::gearWorkspace (), 2002, LOCAL_PAIRS, R_MIN_TEST, LOCAL_MARGIN_MULT, "gear");
}
