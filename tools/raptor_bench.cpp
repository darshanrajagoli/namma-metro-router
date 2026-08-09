// tools/raptor_bench.cpp — the controlled comparison this repository owed itself.
//
// The README carried, for a long time, an honest and strategically damaging
// sentence: "RAPTOR would very likely be faster for this specific objective."
// It was a guess. This binary replaces it with a measurement.
//
// WHAT MAKES THIS A CONTROLLED COMPARISON RATHER THAN TWO BENCHMARKS
// ══════════════════════════════════════════════════════════════════
//   Same feed           — one GTFSParser, one stop index, both structures built
//                         from it, with identical segment-admission rules.
//   Same query set      — the same screened (origin, departure) pairs, in the
//                         same order, for every arm.
//   Same timing harness — the project's CPUID/RDTSC serialised timer.
//   Interleaved         — one query per arm per iteration, so a thermal ramp or
//                         a scheduler hiccup lands on all arms equally instead
//                         of on whichever ran second.
//   Correctness gated   — the run REFUSES to report timings until the two
//                         implementations have been shown to agree on this
//                         feed. A speed comparison between an algorithm and a
//                         wrong algorithm is not a result.
//
// THE THREE ARMS, AND WHY THREE
// ═════════════════════════════
//   engine-shipped   the Pareto engine exactly as main_bench.cpp runs it
//                    (k=5, W_max=1800, lambda=1.0). What the project ships.
//   engine-timeopt   the same engine with lambda=0, so departure selection
//                    minimises arrival. Isolates the cost of the SECOND
//                    OBJECTIVE from the cost of the search.
//   raptor           rounds over routes; the (arrival, trips) frontier falls
//                    out of the round index.
//
// All three answer "one origin, one departure, every destination". The first
// and third both return a bi-criteria frontier; the second returns one too but
// with a degenerate second dimension.
//
// USAGE
//   routing_engine_raptor_bench <feed_dir> [--queries N] [--timed N] [--seed N]
//   taskset -c 3 ./build/routing_engine_raptor_bench ./feeds/norm/bart

#include "benchmark.hpp"
#include "graph.hpp"
#include "gtfs_parser.hpp"
#include "raptor.hpp"
#include "routing.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <new>
#include <random>
#include <string>
#include <vector>

using namespace namma_metro;

namespace
{

    constexpr uint32_t WINDOW_START = 25200;
    constexpr uint32_t WINDOW_END = 75600;

    /// Linear interpolation at index p*(n-1) — the same definition as
    /// bench::compute_percentiles and tools/study.cpp, so every percentile this
    /// repository prints means the same thing.
    double pct(const std::vector<double> &sorted, double p)
    {
        if (sorted.empty())
            return 0.0;
        const std::size_t n = sorted.size();
        const double idx = p * static_cast<double>(n - 1);
        const std::size_t lo = static_cast<std::size_t>(idx);
        const std::size_t hi = (lo + 1 < n) ? lo + 1 : lo;
        const double frac = idx - static_cast<double>(lo);
        return sorted[lo] * (1.0 - frac) + sorted[hi] * frac;
    }

    struct Arm
    {
        const char *name;
        std::vector<double> samples;
    };

} // namespace

int main(int argc, char **argv)
{
    std::string feed_dir;
    uint32_t n_queries = 200, n_timed = 3000, seed = 123;
    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        if (a == "--queries" && i + 1 < argc)
            n_queries = static_cast<uint32_t>(std::stoul(argv[++i]));
        else if (a == "--timed" && i + 1 < argc)
            n_timed = static_cast<uint32_t>(std::stoul(argv[++i]));
        else if (a == "--seed" && i + 1 < argc)
            seed = static_cast<uint32_t>(std::stoul(argv[++i]));
        else if (a.rfind("--", 0) == 0)
        {
            std::fprintf(stderr, "unknown option %s\n", a.c_str());
            return 2;
        }
        else
            feed_dir = a;
    }
    if (feed_dir.empty())
    {
        std::fprintf(stderr,
                     "usage: routing_engine_raptor_bench <feed_dir> [--queries N] [--timed N] [--seed N]\n");
        return 2;
    }

    std::printf("╔════════════════════════════════════════════════════════════════╗\n");
    std::printf("║  Time-dependent Pareto-Dijkstra  vs  RAPTOR — same everything   ║\n");
    std::printf("╚════════════════════════════════════════════════════════════════╝\n\n");

    // ── Load once, build both ────────────────────────────────────────────────
    GTFSParser parser(feed_dir);
    try
    {
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
    }
    catch (const std::exception &e)
    {
        std::fprintf(stderr, "load failed: %s\n", e.what());
        return 1;
    }
    if (parser.stop_times().empty())
    {
        std::fprintf(stderr, "feed carried no stop_times rows\n");
        return 1;
    }

    const bool has_tr = !parser.transfers().empty();
    const uint32_t num_stops = static_cast<uint32_t>(parser.stops().size());
    CSRGraph graph = GraphBuilder::build_with_transfers(
        parser.stop_times(), num_stops, &parser.stop_index_map(), parser.transfers(),
        has_tr ? SecondObjective::TransferCount : SecondObjective::CrowdExposure);
    RaptorTimetable tt = RaptorBuilder::build(
        parser.stop_times(), num_stops, &parser.stop_index_map(), parser.transfers());

    std::printf("feed              : %s\n", feed_dir.c_str());
    std::printf("graph  (engine)   : %u nodes, %u service edges, %u transfer edges, %.0f KB\n",
                graph.num_nodes, graph.num_edges, graph.num_transfers, graph.memory_bytes() / 1024.0);
    std::printf("timetable (raptor): %u routes, %u trips, %u footpaths, %.0f KB\n",
                tt.num_routes, tt.num_trips, tt.num_transfers, tt.memory_bytes() / 1024.0);
    std::printf("                    %u pattern split(s) for overtaking, %u trip split(s) for a bad segment\n",
                tt.routes_split_for_overtaking, tt.trips_split_for_bad_segment);
    if (!transfers_are_transitively_closed(tt))
        std::printf("[WARN] footpaths are not transitively closed — RAPTOR arrivals may be upper bounds.\n");
    std::printf("\n");

    // ── Query set ────────────────────────────────────────────────────────────
    // 30 rounds, not 12: the cap is a safety net, and a RAPTOR that hits it is
    // no longer exact, which would turn the correctness gate below into a
    // report of a configuration problem dressed up as a correctness one.
    Raptor raptor(tt, 30);
    RaptorResult rr;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<uint32_t> snode(0, graph.num_nodes - 1);
    std::uniform_int_distribution<uint32_t> stime(WINDOW_START, WINDOW_END);

    struct Q
    {
        uint32_t src, dep;
    };
    std::vector<Q> queries;
    for (uint32_t a = 0; a < n_queries * 40 + 200 && queries.size() < n_queries; ++a)
    {
        const uint32_t s = snode(rng), t = stime(rng);
        raptor.run(s, t, rr);
        uint32_t reached = 0;
        for (uint32_t v = 0; v < graph.num_nodes && reached < 2; ++v)
            if (v != s && rr.reached(v))
                ++reached;
        if (reached >= 2)
            queries.push_back({s, t});
    }
    if (queries.empty())
    {
        std::fprintf(stderr, "no reachable query pairs in the 07:00-21:00 window\n");
        return 1;
    }
    std::printf("query set         : %zu reachable (origin, departure) pairs, seed %u\n\n",
                queries.size(), seed);

    // ── Gate: do the two agree? ──────────────────────────────────────────────
    // The engine is given an unbounded lookahead window and lambda = 0, which
    // makes its departure selection exact. Under those settings the two
    // algorithms solve an identical problem and any disagreement is a bug.
    std::printf("=== Correctness gate: unrestricted engine vs RAPTOR ===\n");
    {
        LookaheadConfig exact_cfg{.k_departures = UINT32_MAX, .W_max_seconds = 172800, .lambda = 0.0f};
        ParetoDijkstra exact_router(graph, exact_cfg);
        QueryResult out;
        uint64_t compared = 0, mismatched = 0;
        uint32_t reported = 0;
        bool exhausted = false;

        // The unrestricted scan is O(deg(u)) per link, so this is much slower
        // than a shipped query. Cap it: agreement on a few dozen one-to-all
        // queries is already hundreds of thousands of compared arrivals.
        const std::size_t gate_n = std::min<std::size_t>(queries.size(), 40);
        for (std::size_t q = 0; q < gate_n; ++q)
        {
            try
            {
                exact_router.run(queries[q].src, queries[q].dep, out);
            }
            catch (const std::bad_alloc &)
            {
                exhausted = true;
                break;
            }
            raptor.run(queries[q].src, queries[q].dep, rr);
            if (rr.hit_round_cap)
            {
                // A truncated RAPTOR reports upper bounds, so a disagreement
                // here would say nothing about either implementation.
                std::printf("  RAPTOR hit its round cap on query %zu; skipping it in the gate.\n", q);
                continue;
            }

            for (uint32_t v = 0; v < graph.num_nodes; ++v)
            {
                uint32_t engine_best = RAPTOR_UNREACHED;
                for (const Label *l : out.pareto_sets[v].labels())
                    engine_best = std::min(engine_best, l->arrival_time);
                ++compared;
                if (engine_best != rr.best(v))
                {
                    ++mismatched;
                    if (reported < 5)
                    {
                        std::printf("  MISMATCH src=%u dep=%u node=%u : engine %u, raptor %u\n",
                                    queries[q].src, queries[q].dep, v, engine_best, rr.best(v));
                        ++reported;
                    }
                }
            }
        }
        if (exhausted)
        {
            std::printf("  arena exhausted with an unbounded window — gate could not run.\n");
            std::printf("  Refusing to report timings: correctness is unverified on this feed.\n");
            return 1;
        }
        std::printf("  %llu arrivals compared over %zu one-to-all queries\n",
                    static_cast<unsigned long long>(compared), gate_n);
        if (mismatched != 0)
        {
            std::printf("  %llu MISMATCHES. Refusing to report timings.\n",
                        static_cast<unsigned long long>(mismatched));
            return 1;
        }
        std::printf("  identical on every one. Both implementations agree on this feed.\n\n");
    }

    // ── Timing ───────────────────────────────────────────────────────────────
    const double ticks_per_ns = bench::calibrate_tsc_ns(100);
    std::printf("=== Interleaved timing ===\n");
    std::printf("TSC %.3f GHz. ", ticks_per_ns);

    double timer_overhead = 0.0;
    {
        std::vector<double> empty;
        empty.reserve(4000);
        for (int i = 0; i < 4000; ++i)
        {
            const uint64_t a = bench::rdtsc_start();
            const uint64_t b = bench::rdtsc_end();
            empty.push_back(bench::ticks_to_ns(b - a, ticks_per_ns));
        }
        std::sort(empty.begin(), empty.end());
        timer_overhead = pct(empty, 0.5);
        std::printf("Empty timed region costs %.0f ns at p50 — every number below "
                    "includes that, both arms equally.\n\n",
                    timer_overhead);
    }

    LookaheadConfig shipped{.k_departures = 5, .W_max_seconds = 1800, .lambda = 1.0f};
    LookaheadConfig timeopt{.k_departures = 5, .W_max_seconds = 1800, .lambda = 0.0f};
    ParetoDijkstra router_shipped(graph, shipped);
    ParetoDijkstra router_timeopt(graph, timeopt);
    router_shipped.prefault_arena();
    router_timeopt.prefault_arena();
    raptor.prefault();

    QueryResult out_a, out_b;
    RaptorResult out_c;

    // Warm-up outside the measured region: first-touch page faults and cold
    // branch predictors belong to setup, not to the algorithm.
    for (uint32_t i = 0; i < 200; ++i)
    {
        const auto &q = queries[i % queries.size()];
        router_shipped.run(q.src, q.dep, out_a);
        router_timeopt.run(q.src, q.dep, out_b);
        raptor.run(q.src, q.dep, out_c);
    }

    Arm arms[3] = {{"engine-shipped", {}}, {"engine-timeopt", {}}, {"raptor", {}}};
    for (auto &arm : arms)
        arm.samples.reserve(n_timed);

    // Count reached destinations once, from the exact side, so the per-destination
    // figures below divide by the same denominator for every arm.
    double reached_sum = 0.0;
    for (const auto &q : queries)
    {
        raptor.run(q.src, q.dep, rr);
        uint32_t c = 0;
        for (uint32_t v = 0; v < graph.num_nodes; ++v)
            if (v != q.src && rr.reached(v))
                ++c;
        reached_sum += c;
    }
    const double mean_reached = reached_sum / queries.size();

    for (uint32_t i = 0; i < n_timed; ++i)
    {
        const auto &q = queries[i % queries.size()];

        const uint64_t a0 = bench::rdtsc_start();
        router_shipped.run(q.src, q.dep, out_a);
        const uint64_t a1 = bench::rdtsc_end();
        __asm__ volatile("" : : "r"(&out_a) : "memory");

        const uint64_t b0 = bench::rdtsc_start();
        router_timeopt.run(q.src, q.dep, out_b);
        const uint64_t b1 = bench::rdtsc_end();
        __asm__ volatile("" : : "r"(&out_b) : "memory");

        const uint64_t c0 = bench::rdtsc_start();
        raptor.run(q.src, q.dep, out_c);
        const uint64_t c1 = bench::rdtsc_end();
        __asm__ volatile("" : : "r"(&out_c) : "memory");

        arms[0].samples.push_back(bench::ticks_to_ns(a1 - a0, ticks_per_ns));
        arms[1].samples.push_back(bench::ticks_to_ns(b1 - b0, ticks_per_ns));
        arms[2].samples.push_back(bench::ticks_to_ns(c1 - c0, ticks_per_ns));
    }

    std::printf("%-16s %12s %12s %12s %14s\n", "arm", "p50 (ns)", "p95 (ns)", "p99 (ns)", "ns/destination");
    std::printf("%-16s %12s %12s %12s %14s\n", "----------------", "------------",
                "------------", "------------", "--------------");
    double p50[3];
    for (int i = 0; i < 3; ++i)
    {
        std::sort(arms[i].samples.begin(), arms[i].samples.end());
        p50[i] = pct(arms[i].samples, 0.50);
        std::printf("%-16s %12.0f %12.0f %12.0f %14.1f\n",
                    arms[i].name, p50[i], pct(arms[i].samples, 0.95), pct(arms[i].samples, 0.99),
                    mean_reached > 0 ? p50[i] / mean_reached : 0.0);
    }
    std::printf("\n%u paired samples, mean %.1f destinations reached per query.\n", n_timed, mean_reached);

    if (p50[2] > 0.0)
    {
        const double r = p50[0] / p50[2];
        std::printf("\nRAPTOR is %.2fx %s than the shipped engine at p50 on this feed.\n",
                    r >= 1.0 ? r : 1.0 / r, r >= 1.0 ? "FASTER" : "SLOWER");
        std::printf("Memory: engine %.0f KB vs raptor %.0f KB (%.2fx).\n",
                    graph.memory_bytes() / 1024.0, tt.memory_bytes() / 1024.0,
                    tt.memory_bytes() > 0
                        ? static_cast<double>(graph.memory_bytes()) / static_cast<double>(tt.memory_bytes())
                        : 0.0);
    }

    // A caveat worth printing next to the numbers rather than in a document
    // nobody opens: the two arms return different second objectives.
    std::printf(
        "\nRead the arms carefully. engine-shipped returns a frontier over "
        "(arrival, %s);\nRAPTOR returns one over (arrival, number of trips). Those coincide only "
        "where\nchanging service requires changing platform — see include/raptor.hpp.\n",
        graph.second_objective == SecondObjective::TransferCount ? "platform walks" : "modelled crowd");

    return 0;
}
