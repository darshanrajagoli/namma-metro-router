/**
 * @file benchmark.cpp
 * @brief Implementations of calibrate_tsc_ns() and compute_percentiles().
 *
 * benchmark.hpp declares these as non-inline, non-template functions, so exactly
 * one translation unit must define them. That unit is this one; it is compiled
 * into namma_metro_core (see CMakeLists.txt).
 */

#include "benchmark.hpp"
#include <ctime>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <cstdio>

namespace namma_metro::bench {

// ═══════════════════════════════════════════════════════════════════════════
// calibrate_tsc_ns
// ═══════════════════════════════════════════════════════════════════════════

double calibrate_tsc_ns(uint64_t calibration_ms) {
    // Read wall clock before TSC burst
    struct timespec t0_wall, t1_wall;
    clock_gettime(CLOCK_MONOTONIC_RAW, &t0_wall);
    const uint64_t tsc0 = rdtsc_start();

    // Busy-wait for calibration_ms milliseconds
    struct timespec now;
    uint64_t elapsed_ns = 0;
    do {
        clock_gettime(CLOCK_MONOTONIC_RAW, &now);
        const int64_t diff_s  = static_cast<int64_t>(now.tv_sec)  - t0_wall.tv_sec;
        const int64_t diff_ns = static_cast<int64_t>(now.tv_nsec) - t0_wall.tv_nsec;
        elapsed_ns = static_cast<uint64_t>(diff_s * 1'000'000'000LL + diff_ns);
    } while (elapsed_ns < calibration_ms * 1'000'000ULL);

    const uint64_t tsc1 = rdtsc_end();
    clock_gettime(CLOCK_MONOTONIC_RAW, &t1_wall);

    // Wall-clock elapsed in nanoseconds (use end-bracket wall for accuracy)
    const int64_t wall_s  = static_cast<int64_t>(t1_wall.tv_sec)  - t0_wall.tv_sec;
    const int64_t wall_ns = static_cast<int64_t>(t1_wall.tv_nsec) - t0_wall.tv_nsec;
    const double total_wall_ns = static_cast<double>(wall_s) * 1e9 + static_cast<double>(wall_ns);

    const uint64_t tsc_delta = tsc1 - tsc0;

    if (tsc_delta == 0 || total_wall_ns <= 0.0) {
        std::fprintf(stderr, "[BENCH WARN] TSC calibration failed — returned 0 delta. "
                             "Falling back to 1.0 ticks/ns (1 GHz). "
                             "Results will be inaccurate on non-1GHz hardware.\n");
        return 1.0;
    }

    // ticks_per_ns = tsc_delta / total_wall_ns
    return static_cast<double>(tsc_delta) / total_wall_ns;
}

// ═══════════════════════════════════════════════════════════════════════════
// compute_percentiles
// ═══════════════════════════════════════════════════════════════════════════

LatencyStats compute_percentiles(std::vector<double>& samples) {
    if (samples.empty()) {
        return LatencyStats{0, 0, 0, 0, 0, 0, 0};
    }

    std::sort(samples.begin(), samples.end());

    const std::size_t n = samples.size();

    auto percentile = [&](double pct) -> double {
        const double idx = pct * static_cast<double>(n - 1);
        const std::size_t lo = static_cast<std::size_t>(idx);
        const std::size_t hi = lo + 1 < n ? lo + 1 : lo;
        const double frac = idx - static_cast<double>(lo);
        return samples[lo] * (1.0 - frac) + samples[hi] * frac;
    };

    const double sum  = std::accumulate(samples.begin(), samples.end(), 0.0);
    const double mean = sum / static_cast<double>(n);

    return LatencyStats{
        .p50_ns   = percentile(0.50),
        .p95_ns   = percentile(0.95),
        .p99_ns   = percentile(0.99),
        .mean_ns  = mean,
        .min_ns   = samples.front(),
        .max_ns   = samples.back(),
        .n_samples = n
    };
}

} // namespace namma_metro::bench
