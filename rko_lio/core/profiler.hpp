/*
 * MIT License
 *
 * Copyright (c) 2025 Meher V.R. Malladi.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file profiler.hpp
 * @brief Utility classes for scoped timing and profiling measurements.
 *
 * @deprecated This profiling utility is deprecated and scheduled for removal in a future release.
 * A more fully featured alternative, likely tracy, will be used instead.
 *
 * Contains the ScopedProfiler RAII class for timing named code blocks,
 * a simple Timer class for quick timing prints, and the SCOPED_PROFILER macro for convenience.
 */
#pragma once
#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>

namespace rko_lio::core {

/**
 * Scoped RAII profiler that measures elapsed time for a named code block.
 *
 * This class records the execution time between construction and destruction (or explicit finish call)
 * and aggregates statistics including count, total time, and max time per named scope.
 *
 * The stats are saved in a static map. Will survive till the end of execution.
 *
 */
class ScopedProfiler {
public:
  explicit ScopedProfiler(std::string name_) : name(std::move(name_)), start(Clock::now()) {}
  ScopedProfiler(const ScopedProfiler&) = delete;
  ScopedProfiler(ScopedProfiler&&) = delete;
  ScopedProfiler& operator=(const ScopedProfiler&) = delete;
  ScopedProfiler& operator=(ScopedProfiler&&) = delete;

  /// Explicitly finish timing and update aggregated statistics.
  void finish() {
    if (!finished) {
      auto& entry = profile_data.map[name];
      const MilliSeconds elapsed = Clock::now() - start;

      ++entry.count;

      // Update Welford's online algorithm for variance
      const double delta = (elapsed - entry.mean).count();
      entry.mean += MilliSeconds{delta / entry.count};
      entry.M2 += delta * (elapsed - entry.mean).count();

      if (elapsed > entry.max_time) {
        entry.max_time = elapsed;
      }

      entry.total += elapsed;

      finished = true;
    }
  }

  /// Automatically finish timing on destruction if not already finished.
  ~ScopedProfiler() { finish(); }

  /// Print aggregated profiling results to stdout.
  static void print_results() { profile_data.print_results(); }

private:
  using Clock = std::chrono::high_resolution_clock;
  using TimePoint = Clock::time_point;
  using Seconds = std::chrono::duration<double>;
  using MilliSeconds = std::chrono::duration<double, std::milli>;

  struct ProfilingInfo {
    size_t count{0};
    MilliSeconds total{};
    MilliSeconds max_time{};
    MilliSeconds mean{};
    double M2{0.0}; // Welford variance accumulator

    MilliSeconds stddev() const {
      if (count < 2) {
        return MilliSeconds{0.0};
      }
      return MilliSeconds{std::sqrt(M2 / (count - 1))};
    }
  };

  class ProfilingInfoMap {
  public:
    std::unordered_map<std::string, ProfilingInfo> map;
    ProfilingInfoMap() = default;
    ProfilingInfoMap(const ProfilingInfoMap&) = delete;
    ProfilingInfoMap(ProfilingInfoMap&&) = delete;
    ProfilingInfoMap& operator=(const ProfilingInfoMap&) = delete;
    ProfilingInfoMap& operator=(ProfilingInfoMap&&) = delete;
    void print_results() {
      if (!map.empty()) {
        std::cout << "Profiling results\n";
      }
      for (const auto& [name, info] : map) {
        std::cout << "\t" << name << ":\n"
                  << "\t\tExecution count: " << info.count << "\n"
                  << "\t\tAverage time: " << std::fixed << std::setprecision(2) << info.mean.count() << " ms\n"
                  << "\t\tStd. dev. time: " << info.stddev().count() << " ms\n"
                  << "\t\tAverage frequency: " << (1.0 / Seconds(info.mean).count()) << "Hz\n"
                  << "\t\tMax (worst-case) frequency: " << (1.0 / Seconds(info.max_time).count()) << "Hz\n";
      }
    }
    ~ProfilingInfoMap() { print_results(); }
  };
  static inline ProfilingInfoMap profile_data;

  std::string name;
  TimePoint start;
  bool finished = false;
};

/** Simple timing utility that prints elapsed time for a scope or code block. */
struct Timer {
  using Clock = std::chrono::high_resolution_clock;
  using TimePoint = std::chrono::time_point<Clock>;
  using Duration = std::chrono::duration<double>;

  Timer() : label("Execution"), start_time(Clock::now()) {}
  explicit Timer(const std::string& label) : label(label), start_time(Clock::now()) {}
  Timer(const Timer&) = default;
  Timer(Timer&&) = default;
  Timer& operator=(const Timer&) = default;
  Timer& operator=(Timer&&) = default;
  ~Timer() {
    auto end_time = Clock::now();
    Duration duration = end_time - start_time;
    std::cout << label << " took " << duration.count() << " seconds.\n";
  }
  std::string label;
  TimePoint start_time;
};

} // namespace rko_lio::core

#define CONCAT_IMPL(x, y) x##y
#define CONCAT(x, y) CONCAT_IMPL(x, y)
/**
 * @def SCOPED_PROFILER(name)
 * Macro helper to create a ScopedProfiler instance with an automatic unique name.
 *
 * Use this macro at the start of a scope or function to measure execution time and collect
 * profiling data aggregating count, average, and max times for the named scope.
 *
 * @note This macro and the associated ScopedProfiler class are planned for deprecation.
 */
#define SCOPED_PROFILER(name) rko_lio::core::ScopedProfiler CONCAT(profiler_, __LINE__)(name)
