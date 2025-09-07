#pragma once
/**
 * @file   test_stats.hpp
 * @brief  Lightweight timing and running-statistics helpers for tests.
 */

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <utility>

namespace test_helpers
{
    /**
     * @brief Online running statistics (Welford) for timing and metrics.
     *
     * Accumulates count, mean, variance, min and max without storing samples.
     */
    class RunningStats
    {
      public:
        /// @brief Add a new sample value (e.g., milliseconds).
        void addSample (double value) noexcept
        {
            ++count_;
            const double delta = value - mean_;
            mean_ += delta / static_cast<double> (count_);
            const double delta2 = value - mean_;
            m2_ += delta * delta2;
            if (value < min_)
                min_ = value;
            if (value > max_)
                max_ = value;
        }

        /// @brief Number of samples seen so far.
        [[nodiscard]] std::size_t count () const noexcept { return count_; }
        /// @brief Sample mean.
        [[nodiscard]] double mean () const noexcept { return mean_; }
        /// @brief Unbiased sample standard deviation (n>1), else 0.
        [[nodiscard]] double stddev () const noexcept { return (count_ > 1) ? std::sqrt (m2_ / static_cast<double> (count_ - 1)) : 0.0; }
        /// @brief Minimum recorded value.
        [[nodiscard]] double min () const noexcept { return (count_ ? min_ : 0.0); }
        /// @brief Maximum recorded value.
        [[nodiscard]] double max () const noexcept { return (count_ ? max_ : 0.0); }

        /**
         * @brief Print a compact summary line to stdout.
         * @param label  Descriptive label, e.g., "Dubins plan".
         */
        void printSummary (const std::string &label) const
        {
            if (!count_)
                return;
            std::cout << std::fixed << std::setprecision (3) << "[     INFO ] " << label << " — mean " << mean_ << " ms"
                      << "  std " << stddev () << " ms"
                      << "  min " << min () << " ms"
                      << "  max " << max () << " ms"
                      << "  (" << count_ << " samples)\n";
        }

      private:
        std::size_t count_ = 0;
        double mean_ = 0.0;
        double m2_ = 0.0;
        double min_ = std::numeric_limits<double>::infinity ();
        double max_ = -std::numeric_limits<double>::infinity ();
    };

    /**
     * @brief Simple scope timer that adds elapsed milliseconds to a @ref RunningStats.
     */
    class ScopedTimer
    {
      public:
        explicit ScopedTimer (RunningStats &stats) : stats_ (stats), t0_ (Clock::now ()) {}
        ~ScopedTimer ()
        {
            const auto t1 = Clock::now ();
            const double ms = std::chrono::duration<double, std::milli> (t1 - t0_).count ();
            stats_.addSample (ms);
        }

      private:
        using Clock = std::chrono::steady_clock;
        RunningStats &stats_;
        Clock::time_point t0_;
    };

    /**
     * @brief Measure the execution time (ms) of a callable and return it.
     * @tparam F   Callable type with signature `void()`.
     * @param fn   Function to time.
     * @return     Elapsed time in milliseconds.
     */
    template <class F> double measureMs (F &&fn)
    {
        using Clock = std::chrono::steady_clock;
        const auto t0 = Clock::now ();
        std::forward<F> (fn) ();
        const auto t1 = Clock::now ();
        return std::chrono::duration<double, std::milli> (t1 - t0).count ();
    }

} // namespace test_helpers
