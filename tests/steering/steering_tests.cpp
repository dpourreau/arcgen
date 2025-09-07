#include <common/plot_dir.hpp>
#include <common/test_stats.hpp>
#include <common/visualizer.hpp>

#include <arcgen.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using namespace arcgen::core;
using namespace arcgen::steering;
using test_helpers::RunningStats;
using test_helpers::ScopedTimer;
using test_helpers::Visualizer;

/* ───────── configuration ───────── */
constexpr double R_MIN = 3.0;     ///< [m]
constexpr double STEP = 0.30;     ///< [m]
constexpr double EPS_GOAL = 0.05; ///< [m]
constexpr double EPS_LEN = 0.02;  ///< [m]
constexpr int SAMPLES = 100;      ///< random pairs / generator

/* ───────── name traits ───────── */
template <class G> struct GenName
{
    static constexpr const char *value = "generic";
};
template <> struct GenName<Dubins>
{
    static constexpr const char *value = "dubins";
};
template <> struct GenName<ReedsShepp>
{
    static constexpr const char *value = "reeds_shepp";
};

/* ───────── utilities ───────── */
namespace
{
    [[nodiscard]] inline double euclidean (const State &a, const State &b) noexcept
    {
        const double dx = a.x - b.x, dy = a.y - b.y;
        return std::sqrt (dx * dx + dy * dy);
    }
    [[nodiscard]] inline double polylineLength (std::span<const State> pts)
    {
        if (pts.size () < 2)
            return 0.0;
        double L = 0.0;
        for (std::size_t i = 1; i < pts.size (); ++i)
            L += euclidean (pts[i - 1], pts[i]);
        return L;
    }
} // namespace

/**
 * @brief Typed fixture for steering generators (property + timing).
 * @tparam Generator  Steering policy type.
 */
template <class Generator> class SteeringFixture : public ::testing::Test
{
  protected:
    SteeringFixture () : gen_{Generator{R_MIN, STEP}}, rng_ (42), pos_ (-10.0, 10.0), ang_ (0.0, arcgen::core::two_pi) {}

    static void TearDownTestSuite () { timing_.printSummary (std::string (GenName<Generator>::value) + " – shortestPath()"); }

    void runOneProperty (int id)
    {
        // random start / goal
        State a{}, b{};
        a.x = pos_ (rng_);
        a.y = pos_ (rng_);
        a.heading = ang_ (rng_);
        b.x = pos_ (rng_);
        b.y = pos_ (rng_);
        b.heading = ang_ (rng_);

        // time + compute path
        typename Generator::PathType result{};
        {
            ScopedTimer t (timing_);
            (void)t;
            result = gen_.shortestPath (a, b);
        }

        bool ok = true;
        std::ostringstream fail;

        if (!result.states || result.states->empty ())
        {
            ok = false;
            fail << "empty path; ";
        }
        else
        {
            // final pose near goal
            const double miss = euclidean (result.states->back (), b);
            if (miss > EPS_GOAL)
            {
                ok = false;
                fail << "goal miss (" << miss << " m); ";
            }

            // length consistency
            const double declared = result.length ();
            const double realised = polylineLength (*result.states);
            if (std::isfinite (declared) && std::fabs (declared - realised) > EPS_LEN)
            {
                ok = false;
                fail << "length mismatch (" << declared << " vs " << realised << "); ";
            }

            // monotone (non-negative) Euclidean steps
            for (std::size_t i = 1; i < result.states->size (); ++i)
            {
                const double step = euclidean ((*result.states)[i - 1], (*result.states)[i]);
                if (step < -1e-12)
                {
                    ok = false;
                    fail << "non-monotone step at " << i << "; ";
                    break;
                }
            }
        }

#ifdef AG_ENABLE_PLOTS
        // Optional plot
        const char *tag = ok ? "ok" : "fail";
        std::ostringstream fn;
        fn << tag << '_' << id << ".svg";
        auto outPath = test_helpers::plotFile ({"steering", GenName<Generator>::value}, fn.str ());
        Visualizer svg (outPath.string (), 800);
        if (result.states)
            svg.drawPath (*result.states);
        if (result.states && !result.states->empty ())
        {
            svg.drawStartPose (result.states->front (), 9.0, "#462ac7");
            svg.drawGoalPose (result.states->back (), 9.0, "#20b255");
        }
        svg.drawAxes ();
        svg.finish ();
#endif
        ASSERT_TRUE (ok) << "Case " << id << " failed: " << fail.str ();
    }

  private:
    Generator gen_;
    std::mt19937 rng_;
    std::uniform_real_distribution<> pos_;
    std::uniform_real_distribution<> ang_;

    inline static RunningStats timing_{};
};

using TestedGenerators = ::testing::Types<Dubins, ReedsShepp>;
TYPED_TEST_SUITE (SteeringFixture, TestedGenerators);

TYPED_TEST (SteeringFixture, PropertiesOnRandomPairs)
{
    for (int k = 0; k < SAMPLES; ++k)
        this->runOneProperty (k);
}
