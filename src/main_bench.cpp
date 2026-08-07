#include "benchmark.hpp"
#include "gtfs_parser.hpp"
#include "graph.hpp"
#include "routing.hpp"
#include <cstdio>
#include <cstdlib>
#include <string>
#include <random>
#include <sched.h>    // sched_getcpu()
#include <cpuid.h>    // __get_cpuid_count()

/**
 * @file main_bench.cpp
 * @brief Entry point for cycle-accurate benchmarking of the routing engine.
 *
 * USAGE:
 *   # 1. Stabilize CPU (run once before benchmarking session)
 *   sudo bash scripts/stabilize_cpu.sh
 *
 *   # 2. Build release target
 *   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
 *   cmake --build build --target routing_engine_benchmark -j$(nproc)
 *
 *   # 3. Pre-fault and benchmark (pinned to isolated core 3 — avoid core 0, OS interrupts)
 *   taskset -c 3 ./build/routing_engine_benchmark ./data
 *
 * EXPECTED OUTPUT FORMAT (copy into README.md table):
 *   | Metric | Latency (ns) |
 *   |--------|-------------|
 *   | p50    |      XXXXX  |
 *   | p95    |      XXXXX  |
 *   | p99    |      XXXXX  |
 *
 * BENCHMARK CONFIGURATION STRING (paste into README.md):
 *   "Benchmarks collected on isolated core 3 with taskset, frequency scaling
 *    disabled, Precision Boost off, arena pre-faulted. Serialization via CPUID+RDTSC/RDTSCP+CPUID."
 */

int main(int argc, char* argv[]) {
    const std::string data_dir = (argc > 1) ? argv[1] : "./data";

    std::printf("╔══════════════════════════════════════════════════╗\n");
    std::printf("║  Namma Metro Routing Engine — Benchmark Harness  ║\n");
    std::printf("╚══════════════════════════════════════════════════╝\n\n");

#ifdef NAMMA_BASELINE_HEAP_ALLOC
    std::printf(
        "  ****************************************************************\n"
        "  *  BASELINE BUILD (NAMMA_BASELINE_HEAP_ALLOC)                  *\n"
        "  *  Labels come from operator new; results are returned by      *\n"
        "  *  value. These are the \"before\" numbers for the arena A/B     *\n"
        "  *  comparison -- NOT the engine's real performance.            *\n"
        "  ****************************************************************\n\n");
#endif

    // ── RDTSC and TSC feature detection ──────────────────────────────────
    {
        uint32_t eax, ebx, ecx, edx;

        // Check RDTSCP support (CPUID 0x80000001 EDX bit 27)
        bool has_rdtscp = false;
        if (__get_cpuid(0x80000001u, &eax, &ebx, &ecx, &edx)) {
            has_rdtscp = (edx >> 27) & 1u;
        }
        if (!has_rdtscp) {
            std::fprintf(stderr,
                "[BENCH WARN] RDTSCP not supported on this CPU/VM.\n"
                "             There is NO software fallback: the harness issues RDTSCP\n"
                "             unconditionally (include/benchmark.hpp), so the numbers\n"
                "             below are NOT trustworthy on this machine.\n"
                "             Re-run on x86-64 hardware with a modern Intel/AMD CPU.\n");
        } else {
            std::printf("[OK] RDTSCP supported — cycle-accurate timing available.\n");
        }

        // ── InvariantTSC check ───────────────────────────────────────────
        // On WSL2 (Hyper-V virtualized), TSC may be emulated. Without
        // InvariantTSC, TSC frequency can vary under C/P-state transitions or
        // VM scheduling. calibrate_tsc_ns() will return a wrong ticks_per_ns
        // and ALL reported p50/p95/p99 will be off by a constant factor.
        //
        // Worse: a single Hyper-V preemption between RDTSC and RDTSCP injects
        // 10-1000 us of latency into ONE sample with no visible signal.
        // That sample makes p99 appear 10-100x higher than real routing cost.
        //
        // Diagnostic: if max_ns / p99_ns > 10 in the output below, a VM-exit
        // outlier has contaminated p99. Results are not interview-grade.
        //
        // CPUID leaf 0x80000007 EDX bit 8 = InvariantTSC flag.
        bool has_invariant_tsc = false;
        if (__get_cpuid(0x80000007u, &eax, &ebx, &ecx, &edx)) {
            has_invariant_tsc = (edx >> 8) & 1u;
        }
        if (!has_invariant_tsc) {
            std::fprintf(stderr,
                "[BENCH WARN] InvariantTSC (CPUID 0x80000007 EDX bit 8) NOT SET.\n"
                "             TSC frequency may vary under C/P-states or VM scheduling.\n"
                "             All p50/p95/p99 values may be wrong (wrong ticks/ns).\n"
                "             VM exits between RDTSC/RDTSCP inflate p99 by 10-100x.\n"
                "             To diagnose: check max_ns/p99_ns ratio in output below.\n"
                "             For valid numbers: run on native hardware, boost off.\n");
        } else {
            std::printf("[OK] InvariantTSC confirmed — TSC frequency is stable.\n");
        }
    }

    // ── CPU affinity verification ─────────────────────────────────────────
    {
        const int current_cpu = sched_getcpu();
        if (current_cpu < 0) {
            std::fprintf(stderr, "[BENCH WARN] sched_getcpu() failed — cannot verify CPU pinning.\n");
        } else if (current_cpu != 3) {
            std::fprintf(stderr,
                "[BENCH WARN] Running on CPU %d, expected CPU 3. "
                "Re-run with: taskset -c 3 ./routing_engine_benchmark ./data\n"
                "Core 0 handles OS interrupt delivery — p99 will include interrupt jitter.\n",
                current_cpu);
        } else {
            std::printf("[OK] Pinned to CPU 3 — interrupt isolation confirmed.\n");
        }
    }

    // ── Step 1: Load GTFS data ────────────────────────────────────────────
    std::printf("[1/5] Loading GTFS data from %s ...\n", data_dir.c_str());

    namma_metro::GTFSParser parser(data_dir);
    try {
        parser.load_agency();
        parser.check_frequencies();
        parser.load_stops();
        parser.load_routes();
        parser.load_trips();
        parser.load_stop_times();
        parser.load_calendar();
        parser.load_calendar_dates();
        parser.load_transfers();
        parser.interpolate_stop_times();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[WARN] GTFS load failed: %s\n", e.what());
        std::fprintf(stderr, "       Running with synthetic graph for demonstration.\n");
    }

    parser.print_stats();

    // ── Step 2: Build CSR graph ───────────────────────────────────────────
    std::printf("[2/5] Building CSR graph ...\n");

    namma_metro::CSRGraph graph;
    const auto& stop_times = parser.stop_times();

    if (!stop_times.empty()) {
        // A feed that ships transfers.txt is one where platforms are distinct
        // nodes and changing lines is a real, costed action, so the second Pareto
        // objective becomes transfer count. Without transfers there is nothing to
        // count and the crowd model remains the second objective.
        const bool has_transfers = !parser.transfers().empty();
        graph = namma_metro::GraphBuilder::build_with_transfers(
            stop_times,
            static_cast<uint32_t>(parser.stops().size()),
            &parser.stop_index_map(),
            parser.transfers(),
            has_transfers ? namma_metro::SecondObjective::TransferCount
                          : namma_metro::SecondObjective::CrowdExposure
        );
    } else {
        // Loud, unmissable banner: a real feed that fails to load produces an
        // identical-looking latency table on a 10-node toy graph. Make the
        // degraded run impossible to mistake for a real one.
        std::fprintf(stderr,
            "\n"
            "  ****************************************************************\n"
            "  *  WARNING: 0 GTFS rows loaded -> FALLING BACK TO A 10-NODE    *\n"
            "  *  SYNTHETIC TOY GRAPH. The numbers below describe a demo      *\n"
            "  *  feed, NOT your data. If you pointed at a real feed, it was  *\n"
            "  *  rejected (positional parser). Run it through                *\n"
            "  *  scripts/normalize_gtfs.py first, then point at the output.  *\n"
            "  ****************************************************************\n\n");
        std::printf("      No GTFS data loaded. Using 10-node synthetic graph.\n");
        graph.num_nodes = 10;
        graph.offset.assign(11, 0);
        for (uint32_t u = 0; u < 9; ++u) {
            namma_metro::Edge e;
            e.destination    = u + 1;
            e.departure_time = 28800 + u * 360;
            e.travel_time    = 300;
            e.secondary_weight   = 100 + u * 50;
            e.penalty        = 0;
            graph.edge_data.push_back(e);
            graph.offset[u + 1] = static_cast<uint32_t>(graph.edge_data.size());
        }
        for (uint32_t u = 9; u <= 10; ++u) graph.offset[u] = graph.offset[9];
        graph.num_edges = static_cast<uint32_t>(graph.edge_data.size());
    }

    std::printf("      Nodes: %u | Edges: %u | Transfers: %u | Memory: %.1f KB\n",
        graph.num_nodes, graph.num_edges, graph.num_transfers,
        graph.memory_bytes() / 1024.0);
    std::printf("      Second objective: %s\n",
        graph.second_objective == namma_metro::SecondObjective::TransferCount
            ? "transfer count (time vs. number of line changes)"
            : "crowd exposure (time vs. modelled crowding)");

    // ── Step 3: Calibrate TSC ─────────────────────────────────────────────
    std::printf("[3/5] Calibrating TSC frequency (100ms busy-wait) ...\n");
    const double ticks_per_ns = namma_metro::bench::calibrate_tsc_ns(100);
    std::printf("      TSC frequency: %.3f GHz\n", ticks_per_ns);

    // The router is constructed HERE, before Step 4, because Step 4 calls
    // router.prefault_arena(). Pre-faulting must happen before any timed sample:
    // otherwise the first touch of each arena page takes an OS page fault inside
    // the measured region.
    namma_metro::LookaheadConfig config{
        .k_departures  = 5,
        .W_max_seconds = 1800,
        .lambda        = 1.0f
    };
    namma_metro::ParetoDijkstra router(graph, config);

    // ── Step 4: Pre-fault arena ───────────────────────────────────────────
    std::printf("[4/5] Pre-faulting router arena memory ...\n");
    router.prefault_arena();
    std::printf("      Router arena pre-faulted. Capacity: 65536 labels (%.1f KB)\n",
        65536.0 * sizeof(namma_metro::Label) / 1024.0);

    // ── Step 5: Run benchmarks ────────────────────────────────────────────
    std::printf("[5/5] Running benchmark (10,000 queries + 1,000 warmup) ...\n\n");

    std::mt19937 rng(42);
    std::uniform_int_distribution<uint32_t> node_dist(0, graph.num_nodes - 1);
    std::uniform_int_distribution<uint32_t> time_dist(25200, 75600); // 07:00 - 21:00

    // Pre-screen for reachable query pairs.
    // Random (src, dep) pairs often fall outside service windows and produce
    // empty-frontier results that exhaust the PQ in microseconds, deflating
    // p50/p95 and making published numbers misleadingly fast.
    //
    // Note there are no reset_arena() calls anywhere in this file: arena_->reset()
    // is the FIRST statement inside run(). Calling reset_arena() externally AFTER
    // a run() would invalidate every Label* inside the returned QueryResult,
    // turning any later path reconstruction into a dangling-pointer read.
    std::printf("      Pre-screening query pairs for reachability...\n");
    struct QueryPair { uint32_t src; uint32_t dep; };
    std::vector<QueryPair> valid_queries;
    valid_queries.reserve(500);
    {
        std::mt19937 screen_rng(123);
        std::uniform_int_distribution<uint32_t> snode(0, graph.num_nodes - 1);
        std::uniform_int_distribution<uint32_t> stime(25200, 75600);

        // std::size_t, not int: valid_queries.size() is unsigned, and comparing
        // it against an int literal trips -Wsign-compare.
        constexpr std::size_t TARGET_PAIRS = 500;
        for (int attempt = 0; attempt < 2000 && valid_queries.size() < TARGET_PAIRS; ++attempt) {
            const uint32_t s = snode(screen_rng);
            const uint32_t t = stime(screen_rng);
            auto probe = router.run(s, t);
            // No reset_arena() here — run() resets at its own start.
            // probe.pareto_sets Label* pointers are valid until next run().
            bool has_result = false;
            for (uint32_t n = 0; n < graph.num_nodes && !has_result; ++n) {
                if (n != s && !probe.pareto_sets[n].labels().empty())
                    has_result = true;
            }
            if (has_result) valid_queries.push_back({s, t});
        }
        if (valid_queries.empty()) {
            std::fprintf(stderr,
                "[BENCH WARN] No reachable query pairs found after 2000 attempts. "
                "Benchmarking with random queries (may include empty-result noise).\n");
            for (int i = 0; i < 500; ++i)
                valid_queries.push_back({node_dist(rng), time_dist(rng)});
        }
    }
    std::printf("      Found %zu reachable query pairs.\n", valid_queries.size());

    std::uniform_int_distribution<std::size_t> qidx(0, valid_queries.size() - 1);

#ifdef NAMMA_BASELINE_HEAP_ALLOC
    // Baseline build: the by-value overload, which allocates a fresh QueryResult
    // (and therefore a fresh frontier vector per reached node) on every call.
    // Paired with heap-allocated Labels, this is the whole "before" picture.
    auto stats = namma_metro::bench::run_benchmark(
        [&]() {
            const auto& q = valid_queries[qidx(rng)];
            auto result = router.run(q.src, q.dep);
            __asm__ volatile("" : : "r"(&result) : "memory");
        },
#else
    namma_metro::QueryResult reusable; // persistent buffers -> zero allocations in the timed loop
    auto stats = namma_metro::bench::run_benchmark(
        [&]() {
            const auto& q = valid_queries[qidx(rng)];
            router.run(q.src, q.dep, reusable); // fills reusable in place, reusing capacity
            // No reset_arena() — run() resets the arena at its own start.
            // asm barrier prevents the compiler from eliding router.run().
            __asm__ volatile("" : : "r"(&reusable) : "memory");
        },
#endif
        ticks_per_ns,
        /*n_queries=*/10'000,
        /*n_warmup=*/ 1'000
    );

    stats.print();

    // ── VM-exit outlier diagnostic ───────────────────────────────────────
    // If max_ns / p99_ns > 10, a VM exit or OS preemption contaminated at
    // least one sample. p99 does not represent routing latency in that case.
    if (stats.max_ns > stats.p99_ns * 10.0) {
        std::fprintf(stderr,
            "[BENCH WARN] max_ns (%.0f ns) / p99_ns (%.0f ns) = %.1fx > 10.\n"
            "             A VM exit or OS preemption injected outlier latency.\n"
            "             Reported p99 = %.0f ns does NOT represent routing cost.\n"
            "             Re-run on native hardware with Precision Boost off.\n",
            stats.max_ns, stats.p99_ns,
            stats.max_ns / stats.p99_ns,
            stats.p99_ns);
    }

    // ── README table output ───────────────────────────────────────────────
    std::printf("\nREADME.md table entry:\n");
    std::printf("| p50  | %.0f ns |\n", stats.p50_ns);
    std::printf("| p95  | %.0f ns |\n", stats.p95_ns);
    std::printf("| p99  | %.0f ns |\n", stats.p99_ns);
    std::printf("\nHardware config string:\n");
    std::printf("Benchmarks collected on isolated core 3 with taskset, "
                "frequency scaling disabled, Precision Boost off, arena pre-faulted. "
                "Serialization via CPUID+RDTSC/RDTSCP+CPUID. "
                "Note: if benchmarked under WSL2, MSR frequency locking unavailable — "
                "measurements may include Precision Boost variance; "
                "check max_ns/p99_ns ratio in output.\n\n");

    return 0;
}
