#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <algorithm>
#include <numeric>
#include <cstdio>

#ifdef _MSC_VER
  #include <intrin.h>
#else
  #include <cpuid.h>
  #include <x86intrin.h>
#endif

/**
 * @file benchmark.hpp
 * @brief Cycle-accurate RDTSC benchmarking harness for microsecond-scale routing latency.
 *
 * ╔══════════════════════════════════════════════════════════════╗
 * ║  HOW HFT FIRMS MEASURE LATENCY — AND WHY THIS MATTERS       ║
 * ╠══════════════════════════════════════════════════════════════╣
 * ║  Raw wall-clock (std::chrono) costs ~100 ns per call on      ║
 * ║  Linux via the vDSO. Against this engine's 54-300 us         ║
 * ║  queries that is ~0.2% — chrono would in fact be adequate.   ║
 * ║  RDTSC is used because it resolves single expansions and     ║
 * ║  makes the serialisation question explicit; it is more       ║
 * ║  rigour than the workload strictly demands.                  ║
 * ║                                                              ║
 * ║  RDTSC reads the 64-bit Time Stamp Counter: a monotonically  ║
 * ║  incrementing counter driven by the invariant TSC frequency  ║
 * ║  (fixed at the nominal CPU frequency, independent of P-states║
 * ║  when boost is disabled). Resolution: ~1 clock cycle.       ║
 * ║                                                              ║
 * ║  Out-of-order execution hazard:                             ║
 * ║  CPUs reorder instructions speculatively. Without barriers,  ║
 * ║  instructions from inside the timed region can execute       ║
 * ║  BEFORE the opening RDTSC, or instructions from outside can  ║
 * ║  execute INSIDE — invalidating both boundaries.             ║
 * ║                                                              ║
 * ║  Solution: serializing instructions.                         ║
 * ║    Start: CPUID → RDTSC   (CPUID is fully serializing)      ║
 * ║    End:   RDTSCP → CPUID  (RDTSCP waits for prior retires;  ║
 * ║                             trailing CPUID prevents later    ║
 * ║                             instructions from leaking in)   ║
 * ║                                                              ║
 * ║  This is the canonical Intel-recommended sequence.          ║
 * ║  See: Intel White Paper "How to Benchmark Code Execution     ║
 * ║  Times on Intel® IA-32 and IA-64 Instruction Set Archs"     ║
 * ╚══════════════════════════════════════════════════════════════╝
 *
 * PRE-REQUISITES (run before any benchmark):
 *   scripts/stabilize_cpu.sh  — sets governor=performance, disables boost.
 *                               NOTE: written for Intel; its BD PROCHOT step
 *                               (MSR 0x1FC) does not apply on AMD. See README.
 *   taskset -c 3              — pin to core 3. NOT core 0, which handles OS
 *                               interrupt delivery. Note this is affinity only:
 *                               the core is not isolcpu-isolated, so other work
 *                               can still be scheduled on it.
 *   arena.prefault()          — pre-fault all arena pages
 */

namespace namma_metro::bench {

// ═══════════════════════════════════════════════════════════════════════════
// § 1.  Serialized RDTSC read primitives
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Read TSC at the START of a timed region.
 *
 * Sequence: CPUID (full serialization barrier) → RDTSC (read TSC).
 * CPUID prevents any prior instruction from executing after the counter read.
 *
 * @return  64-bit TSC value (lower 32 bits in eax, upper 32 in edx).
 */
[[nodiscard]] inline uint64_t rdtsc_start() noexcept {
    uint32_t hi, lo;
    __asm__ volatile (
        "cpuid\n\t"          // serializing barrier — drain the pipeline
        "rdtsc\n\t"          // read TSC: edx = high 32 bits, eax = low 32 bits
        : "=d"(hi), "=a"(lo) // outputs
        :                    // no inputs
        : "rbx", "rcx"       // cpuid clobbers rbx, rcx
    );
    return (static_cast<uint64_t>(hi) << 32) | lo;
}

/**
 * @brief Read TSC at the END of a timed region.
 *
 * Sequence: RDTSCP (waits for all prior instructions to retire) → CPUID
 * (prevents any later instructions from leaking into the measurement window).
 *
 * RDTSCP also reads the IA32_TSC_AUX MSR into ecx (CPU id), which can
 * detect cross-core TSC migration on multi-socket systems. On Dell G15
 * (single socket), this is informational only.
 *
 * @return  64-bit TSC value.
 */
[[nodiscard]] inline uint64_t rdtsc_end() noexcept {
    uint32_t hi, lo;
    __asm__ volatile (
        "rdtscp\n\t"         // ordered TSC read: waits for prior retirements
        "mov %%edx, %0\n\t"
        "mov %%eax, %1\n\t"
        "cpuid\n\t"          // prevent later instructions from leaking in
        : "=r"(hi), "=r"(lo)
        :
        : "rax", "rbx", "rcx", "rdx"
    );
    return (static_cast<uint64_t>(hi) << 32) | lo;
}

// ═══════════════════════════════════════════════════════════════════════════
// § 2.  TSC → nanoseconds calibration
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Measure the TSC tick frequency using a short busy-wait calibration.
 *
 * Call once at program startup. With Precision Boost disabled and governor set
 * to `performance`, the TSC frequency equals the rated base clock of the CPU.
 * On the Dell G15 5535 (AMD Ryzen 5 7640HS, Zen 4) the base clock is ~4.3 GHz,
 * and TSC calibration on this machine reports ~4.29 GHz — consistent with that.
 *
 * @param  calibration_ms  Busy-wait duration for calibration (default 100ms).
 * @return TSC ticks per nanosecond (as a double for fractional precision).
 */
[[nodiscard]] double calibrate_tsc_ns(uint64_t calibration_ms = 100);

/**
 * @brief Convert raw TSC delta to nanoseconds.
 * @param tsc_delta   Raw TSC ticks measured between rdtsc_start/rdtsc_end.
 * @param ticks_per_ns TSC frequency in ticks/ns (from calibrate_tsc_ns()).
 */
[[nodiscard]] inline double ticks_to_ns(uint64_t tsc_delta, double ticks_per_ns) noexcept {
    return static_cast<double>(tsc_delta) / ticks_per_ns;
}

// ═══════════════════════════════════════════════════════════════════════════
// § 3.  Percentile computation
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Compute latency distribution statistics from raw nanosecond samples.
 */
struct LatencyStats {
    double p50_ns;  ///< Median latency
    double p95_ns;  ///< 95th percentile
    double p99_ns;  ///< 99th percentile — primary HFT quality metric
    double mean_ns;
    double min_ns;
    double max_ns;
    std::size_t n_samples;

    /// Pretty-print to stdout in the format expected by README.md table.
    void print() const {
        std::printf(
            "\n╔══════════════════════════════════════════╗\n"
            "║      Namma Metro Routing Engine           ║\n"
            "║      Latency Distribution (ns)            ║\n"
            "╠══════════════════════════════════════════╣\n"
            "║  Samples : %8zu                     ║\n"
            "║  Min     : %10.1f                   ║\n"
            "║  Mean    : %10.1f                   ║\n"
            "║  p50     : %10.1f  ← median         ║\n"
            "║  p95     : %10.1f                   ║\n"
            "║  p99     : %10.1f  ← key HFT metric ║\n"
            "║  Max     : %10.1f                   ║\n"
            "╚══════════════════════════════════════════╝\n",
            n_samples, min_ns, mean_ns, p50_ns, p95_ns, p99_ns, max_ns
        );
    }
};

/**
 * @brief Compute p50, p95, p99 from a vector of latency samples (in ns).
 *
 * Sorts the input vector in-place (pass a copy if original order needed).
 *
 * @param samples  Mutable reference to latency samples in nanoseconds.
 * @return         Computed LatencyStats struct.
 */
[[nodiscard]] LatencyStats compute_percentiles(std::vector<double>& samples);

// ═══════════════════════════════════════════════════════════════════════════
// § 4.  Benchmark runner template
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Execute @p n_queries timed trials of @p query_fn and return stats.
 *
 * Protocol:
 *   1. Pre-fault the arena (call arena.prefault() before invoking this).
 *   2. Execute @p n_warmup warm-up queries (results discarded).
 *   3. Execute @p n_queries measured queries, recording TSC deltas.
 *   4. Convert ticks → nanoseconds and compute percentiles.
 *
 * @tparam QueryFn   Callable with signature `void()` wrapping one routing query.
 *                   Must not allocate heap memory.
 *
 * @param query_fn   The routing query to benchmark.
 * @param ticks_per_ns TSC calibration value from calibrate_tsc_ns().
 * @param n_queries  Number of measured iterations (default: 10,000).
 * @param n_warmup   Number of warm-up iterations (default: 1,000).
 * @return           LatencyStats with p50/p95/p99 in nanoseconds.
 */
template <typename QueryFn>
[[nodiscard]] LatencyStats run_benchmark(
    QueryFn&&      query_fn,
    double         ticks_per_ns,
    std::size_t    n_queries = 10'000,
    std::size_t    n_warmup  = 1'000
) {
    // ── Warm-up (prime L1/L2/TLB caches) ──────────────────────────────
    for (std::size_t i = 0; i < n_warmup; ++i) {
        query_fn();
    }

    // ── Measured runs ─────────────────────────────────────────────────
    std::vector<double> latencies;
    latencies.reserve(n_queries);

    for (std::size_t i = 0; i < n_queries; ++i) {
        const uint64_t t0 = rdtsc_start();
        query_fn();
        const uint64_t t1 = rdtsc_end();
        latencies.push_back(ticks_to_ns(t1 - t0, ticks_per_ns));
    }

    return compute_percentiles(latencies);
}

} // namespace namma_metro::bench
