// tools/study.cpp — one feed in, one row of measurements out.
//
// THE QUESTION
// ════════════
// Multi-objective transit routing is universally assumed necessary, and the
// machinery it needs — frontier maintenance, dominance pruning, label-correcting
// revisits — is not free. The engine's own diagnostic found that on Namma Metro
// the frontier carries a single label at every one of 24,600 observations, and
// on BART it carries a genuine trade-off at 71% of reached nodes. Same code,
// same objective, opposite answers.
//
// So: across real networks, how often does a genuine time-versus-changes
// trade-off exist, and which structural property of a network predicts it?
//
// WHAT THIS BINARY MEASURES, AND WHY IT MEASURES IT TWICE
// ═══════════════════════════════════════════════════════
// The obvious approach — run tools/diag.cpp on many feeds — answers a narrower
// question than it appears to. diag reports the frontier THIS ENGINE finds, and
// this engine expands one composite-optimal departure per link chosen from the
// next k within a bounded window. A small frontier could mean the network has no
// trade-off, or it could mean the search never looked. Those are different
// findings and a study cannot conflate them.
//
// So every feed is measured twice:
//
//   RAPTOR  — exact. Round k is by construction the earliest arrival using at
//             most k trips, so the (arrival, trips) Pareto frontier falls out of
//             the round structure with no dominance machinery and no heuristic.
//             This is the answer to the research question.
//
//   ENGINE  — the shipped Pareto-Dijkstra with its default lookahead. This is
//             the answer to "what does this implementation see", which is a
//             different and also interesting number.
//
// and the difference between them is itself reported: with the departure
// selection set to minimise arrival but the lookahead window left at its shipped
// size, how often is the engine's earliest arrival later than the true one, and
// by how much? That isolates the cost of the bounded-wait heuristic on real
// timetables, which the existing test suite can only demonstrate on fixtures.
//
// USAGE
//   routing_engine_study <feed_dir> [--slug NAME] [--queries N] [--timed N]
//                        [--seed N] [--csv-only] [--header]
//   routing_engine_study --header        # print the CSV header and exit
//
// Output is a human-readable report on stdout followed by one CSV row prefixed
// with "CSV:". scripts/run_study.py collects those lines across feeds.

#include "benchmark.hpp"
#include "graph.hpp"
#include "gtfs_parser.hpp"
#include "raptor.hpp"
#include "routing.hpp"
#include "topology.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <new>
#include <random>
#include <string>
#include <vector>

using namespace namma_metro;

namespace
{

    // The sampling window shared with main_bench.cpp and tools/diag.cpp, so
    // every number in this project describes the same slice of the service day.
    constexpr uint32_t WINDOW_START = 25200; // 07:00
    constexpr uint32_t WINDOW_END = 75600;   // 21:00

    struct Options
    {
        std::string feed_dir;
        std::string slug = "unnamed";
        uint32_t n_queries = 200;
        uint32_t n_timed = 0; // 0 = calibrate against a time budget
        uint32_t seed = 123;
        bool csv_only = false;
        double timing_budget_s = 3.0;

        // Generous, because the cap is a safety net and not a modelling choice.
        // RAPTOR stops on its own the moment a round improves nothing, so on a
        // normal network this is never reached — but Vienna's tram network
        // needed more than twelve, and a truncated RAPTOR is no longer exact,
        // which turned the correctness gate red for the right reason. Memory is
        // (max_rounds + 1) x stops x 4 bytes: 5 MB on the largest feed here.
        uint32_t max_rounds = 30;
    };

    std::string csv_header()
    {
        return std::string("slug,status,nodes,edges,transfers,second_objective,")
             + "graph_kb,raptor_kb,routes,raptor_trips,routes_split,trips_split,footpaths_closed,"
             + topology_csv_header() + ","
             + "queries,rt_obs,rt_reached,rt_k1,rt_k2plus,rt_k2plus_frac,rt_max_k,rt_mean_k,"
             + "rt_max_round_used,rt_cap_hits,"
             + "en_obs,en_reached,en_k2plus_frac,en_max_k,en_mean_k,"
             + "ag_compared,ag_equal,ag_later,ag_earlier,ag_engine_unreached,ag_skipped_capped,"
             + "ag_suboptimal_frac,ag_gap_mean_s,ag_gap_p95_s,ag_gap_max_s,"
             + "t_samples,t_overhead_ns,t_engine_p50,t_engine_p95,t_engine_p99,"
             + "t_raptor_p50,t_raptor_p95,t_raptor_p99,t_ratio_p50,note";
    }

    /// Percentile of an already-sorted vector.
    ///
    /// Linear interpolation at index p*(n-1) — byte for byte the definition in
    /// bench::compute_percentiles (src/benchmark.cpp), so a p99 printed here and
    /// a p99 printed by main_bench.cpp mean the same thing. Three percentile
    /// conventions in one repository is three sets of numbers that look
    /// comparable and are not.
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

    /// A CSV field must not contain a comma; feed notes are the only free text.
    std::string csv_escape(std::string s)
    {
        for (char &c : s)
            if (c == ',' || c == '\n' || c == '"')
                c = ' ';
        return s;
    }

} // namespace

int main(int argc, char **argv)
{
    Options opt;
    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        auto next = [&](const char *what) -> std::string
        {
            if (i + 1 >= argc)
            {
                std::fprintf(stderr, "%s needs a value\n", what);
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--header")
        {
            std::printf("%s\n", csv_header().c_str());
            return 0;
        }
        else if (a == "--slug")
            opt.slug = next("--slug");
        else if (a == "--queries")
            opt.n_queries = static_cast<uint32_t>(std::stoul(next("--queries")));
        else if (a == "--timed")
            opt.n_timed = static_cast<uint32_t>(std::stoul(next("--timed")));
        else if (a == "--seed")
            opt.seed = static_cast<uint32_t>(std::stoul(next("--seed")));
        else if (a == "--max-rounds")
            opt.max_rounds = static_cast<uint32_t>(std::stoul(next("--max-rounds")));
        else if (a == "--csv-only")
            opt.csv_only = true;
        else if (a.rfind("--", 0) == 0)
        {
            std::fprintf(stderr, "unknown option %s\n", a.c_str());
            return 2;
        }
        else
            opt.feed_dir = a;
    }
    if (opt.feed_dir.empty())
    {
        std::fprintf(stderr, "usage: routing_engine_study <feed_dir> [--slug NAME] ...\n");
        return 2;
    }

    const bool verbose = !opt.csv_only;

    // A failed feed still emits a row: a study that silently omits the feeds it
    // could not process reports a biased sample.
    auto bail = [&](const char *status, const std::string &note)
    {
        // Pad to the full column count so every row parses with the same arity.
        // A header with F fields has F-1 commas; "slug,status" is 2 fields, and
        // `note` is the last, so F-3 empty fields go in between.
        const std::string header = csv_header();
        const std::size_t commas = static_cast<std::size_t>(
            std::count(header.begin(), header.end(), ','));
        std::printf("CSV:%s,%s", opt.slug.c_str(), status);
        for (std::size_t i = 2; i < commas; ++i)
            std::printf(",");
        std::printf(",%s\n", csv_escape(note).c_str());
        return 1;
    };

    // ── Load ──────────────────────────────────────────────────────────────────
    GTFSParser parser(opt.feed_dir);
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
        return bail("load_failed", e.what());
    }
    if (parser.stop_times().empty())
        return bail("no_stop_times", "feed parsed but carried no stop_times rows");

    // ── Build both structures from the SAME index ─────────────────────────────
    const bool has_tr = !parser.transfers().empty();
    CSRGraph graph;
    RaptorTimetable tt;
    TopologyMetrics topo;
    try
    {
        graph = GraphBuilder::build_with_transfers(
            parser.stop_times(), static_cast<uint32_t>(parser.stops().size()),
            &parser.stop_index_map(), parser.transfers(),
            has_tr ? SecondObjective::TransferCount : SecondObjective::CrowdExposure);
        tt = RaptorBuilder::build(parser.stop_times(),
                                  static_cast<uint32_t>(parser.stops().size()),
                                  &parser.stop_index_map(), parser.transfers());
        topo = compute_topology(parser.stop_times(), parser.trips(),
                                static_cast<uint32_t>(parser.stops().size()),
                                &parser.stop_index_map(), parser.transfers());
    }
    catch (const std::exception &e)
    {
        return bail("build_failed", e.what());
    }
    if (graph.num_nodes == 0 || graph.num_edges == 0)
        return bail("empty_graph", "no routable edges after admission filtering");

    const bool closed = transfers_are_transitively_closed(tt);

    if (verbose)
    {
        std::printf("╔══════════════════════════════════════════════════════════════╗\n");
        std::printf("║  Multi-feed trade-off study — %-30s ║\n", opt.slug.c_str());
        std::printf("╚══════════════════════════════════════════════════════════════╝\n\n");
        std::printf("feed        : %s\n", opt.feed_dir.c_str());
        std::printf("graph       : %u nodes, %u edges, %u transfers, second objective %s\n",
                    graph.num_nodes, graph.num_edges, graph.num_transfers,
                    graph.second_objective == SecondObjective::TransferCount ? "transfer count"
                                                                            : "crowd exposure");
        std::printf("raptor      : %u routes, %u trips (%u pattern splits for overtaking, "
                    "%u trip splits)\n",
                    tt.num_routes, tt.num_trips, tt.routes_split_for_overtaking,
                    tt.trips_split_for_bad_segment);
        std::printf("memory      : graph %.0f KB, raptor %.0f KB\n",
                    graph.memory_bytes() / 1024.0, tt.memory_bytes() / 1024.0);
        std::printf("topology    : %s\n", topo.summary().c_str());
        if (!closed)
            std::printf("[WARN] the footpath relation is NOT transitively closed; RAPTOR's single\n"
                        "       relaxation pass may return upper bounds rather than exact arrivals.\n");
        std::printf("\n");
    }

    // ── Query set ─────────────────────────────────────────────────────────────
    // Screened with RAPTOR, not with the engine: screening with the engine would
    // pick query pairs the engine happens to handle well, which is exactly the
    // bias the comparison is trying to measure.
    Raptor raptor(tt, opt.max_rounds);
    RaptorResult rr;
    std::mt19937 rng(opt.seed);
    std::uniform_int_distribution<uint32_t> snode(0, graph.num_nodes - 1);
    std::uniform_int_distribution<uint32_t> stime(WINDOW_START, WINDOW_END);

    struct Query
    {
        uint32_t src, dep;
    };
    std::vector<Query> queries;
    queries.reserve(opt.n_queries);
    const uint32_t max_attempts = opt.n_queries * 40 + 200;
    for (uint32_t attempt = 0; attempt < max_attempts && queries.size() < opt.n_queries; ++attempt)
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
        return bail("no_reachable_queries", "no origin reached two stops in the sampling window");
    if (verbose)
        std::printf("query set   : %zu reachable (origin, departure) pairs, seed %u\n\n",
                    queries.size(), opt.seed);

    // ── Pass 1: RAPTOR's exact frontier ───────────────────────────────────────
    std::map<uint32_t, uint64_t> rt_hist;
    uint64_t rt_obs = 0, rt_reached = 0, rt_k2plus = 0, rt_k_sum = 0, rt_cap_hits = 0;
    uint32_t rt_max_k = 0, rt_max_round = 0;
    std::vector<std::vector<uint32_t>> oracle(queries.size());
    // A query that hit the round cap has a TRUNCATED oracle: its arrivals are
    // upper bounds, so the engine can legitimately beat it and the comparison
    // would report a correctness failure that is really a configuration one.
    // Such queries are excluded from the agreement pass and counted.
    std::vector<uint8_t> oracle_exact(queries.size(), 1);

    for (std::size_t q = 0; q < queries.size(); ++q)
    {
        raptor.run(queries[q].src, queries[q].dep, rr);
        if (rr.hit_round_cap)
        {
            ++rt_cap_hits;
            oracle_exact[q] = 0;
        }
        rt_max_round = std::max(rt_max_round, rr.rounds);
        oracle[q].assign(graph.num_nodes, RAPTOR_UNREACHED);
        for (uint32_t v = 0; v < graph.num_nodes; ++v)
        {
            oracle[q][v] = rr.best(v);
            if (v == queries[q].src)
                continue; // the trivial stay-put option is not a journey
            ++rt_obs;
            if (!rr.reached(v))
                continue;
            ++rt_reached;
            const uint32_t k = rr.frontier_size(v);
            ++rt_hist[k];
            rt_k_sum += k;
            rt_max_k = std::max(rt_max_k, k);
            if (k > 1)
                ++rt_k2plus;
        }
    }

    // ── Pass 2: the shipped engine's own frontier ─────────────────────────────
    LookaheadConfig shipped{.k_departures = 5, .W_max_seconds = 1800, .lambda = 1.0f};
    uint64_t en_obs = 0, en_reached = 0, en_k2plus = 0, en_k_sum = 0;
    uint32_t en_max_k = 0;
    bool arena_exhausted = false;
    std::string note;

    {
        ParetoDijkstra router(graph, shipped);
        QueryResult out;
        for (const auto &q : queries)
        {
            try
            {
                router.run(q.src, q.dep, out);
            }
            catch (const std::bad_alloc &)
            {
                arena_exhausted = true;
                note = "engine arena (65536 labels) exhausted on this feed";
                break;
            }
            for (uint32_t v = 0; v < graph.num_nodes; ++v)
            {
                if (v == q.src)
                    continue;
                ++en_obs;
                const std::size_t k = out.pareto_sets[v].size();
                if (k == 0)
                    continue;
                ++en_reached;
                en_k_sum += k;
                en_max_k = std::max(en_max_k, static_cast<uint32_t>(k));
                if (k > 1)
                    ++en_k2plus;
            }
        }
    }

    // ── Pass 3: how much does the bounded-wait window cost? ───────────────────
    // lambda = 0 so departure selection minimises ARRIVAL: this isolates the
    // window and the k budget from the choice of second objective. Any gap here
    // is the price of the lookahead heuristic, measured against an exact
    // algorithm on a real timetable.
    LookaheadConfig time_optimal{.k_departures = 5, .W_max_seconds = 1800, .lambda = 0.0f};
    uint64_t ag_compared = 0, ag_equal = 0, ag_later = 0, ag_earlier = 0, ag_unreached = 0;
    uint64_t ag_skipped_capped = 0;
    std::vector<double> gaps;

    if (!arena_exhausted)
    {
        ParetoDijkstra router(graph, time_optimal);
        QueryResult out;
        for (std::size_t q = 0; q < queries.size(); ++q)
        {
            if (!oracle_exact[q])
            {
                ++ag_skipped_capped;
                continue;
            }
            try
            {
                router.run(queries[q].src, queries[q].dep, out);
            }
            catch (const std::bad_alloc &)
            {
                arena_exhausted = true;
                note = "engine arena exhausted during the optimality comparison";
                break;
            }
            for (uint32_t v = 0; v < graph.num_nodes; ++v)
            {
                if (v == queries[q].src)
                    continue;
                const uint32_t truth = oracle[q][v];
                if (truth == RAPTOR_UNREACHED)
                    continue;

                uint32_t engine_best = RAPTOR_UNREACHED;
                for (const Label *l : out.pareto_sets[v].labels())
                    engine_best = std::min(engine_best, l->arrival_time);

                if (engine_best == RAPTOR_UNREACHED)
                {
                    ++ag_unreached;
                    continue;
                }
                ++ag_compared;
                if (engine_best == truth)
                    ++ag_equal;
                else if (engine_best > truth)
                {
                    ++ag_later;
                    gaps.push_back(static_cast<double>(engine_best - truth));
                }
                else
                {
                    // Impossible unless one of the two is wrong. Loud, not silent.
                    ++ag_earlier;
                }
            }
        }
    }
    if (ag_earlier > 0)
    {
        std::fprintf(stderr,
                     "[STUDY ERROR] the engine reported an arrival EARLIER than the exact "
                     "oracle on %llu observations. One of the two implementations is wrong; "
                     "no measurement in this row should be trusted.\n",
                     static_cast<unsigned long long>(ag_earlier));
        if (note.empty())
            note = "engine beat the oracle: results invalid";
    }
    std::sort(gaps.begin(), gaps.end());

    // ── Pass 4: interleaved timing ────────────────────────────────────────────
    // One engine query and one RAPTOR query per iteration, on the same
    // (origin, departure) pair, so any drift in machine conditions lands on both
    // arms equally. This is the same discipline tools/ab.py uses for the arena
    // A/B, applied to two algorithms instead of two allocators.
    double t_overhead = 0.0, e50 = 0, e95 = 0, e99 = 0, r50 = 0, r95 = 0, r99 = 0;
    uint32_t timed = 0;

    if (!arena_exhausted)
    {
        const double ticks_per_ns = bench::calibrate_tsc_ns(100);

        // What does the timer itself cost? On a virtualised host the serialising
        // CPUID in the harness can be a VM exit costing microseconds, which would
        // swamp a fast RAPTOR query. Measuring it lets the reader judge whether
        // the comparison has any resolution at all.
        {
            std::vector<double> empties;
            empties.reserve(2000);
            for (int i = 0; i < 2000; ++i)
            {
                const uint64_t a = bench::rdtsc_start();
                const uint64_t b = bench::rdtsc_end();
                empties.push_back(bench::ticks_to_ns(b - a, ticks_per_ns));
            }
            std::sort(empties.begin(), empties.end());
            t_overhead = pct(empties, 0.50);
        }

        ParetoDijkstra router(graph, shipped);
        router.prefault_arena();
        raptor.prefault();
        QueryResult out;
        RaptorResult rout;

        // Warm up and calibrate the sample count against a wall-clock budget, so
        // a 40-feed sweep does not take an afternoon on the largest network.
        const auto warm_start = std::chrono::steady_clock::now();
        constexpr int kWarm = 20;
        for (int i = 0; i < kWarm; ++i)
        {
            const auto &q = queries[static_cast<std::size_t>(i) % queries.size()];
            router.run(q.src, q.dep, out);
            raptor.run(q.src, q.dep, rout);
        }
        const double per_iter_s =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - warm_start).count() / kWarm;

        uint32_t n = opt.n_timed;
        if (n == 0)
        {
            const double budget = opt.timing_budget_s;
            n = per_iter_s > 0.0 ? static_cast<uint32_t>(budget / per_iter_s) : 2000u;
            n = std::max(200u, std::min(n, 5000u));
        }

        std::vector<double> et, rt;
        et.reserve(n);
        rt.reserve(n);
        for (uint32_t i = 0; i < n; ++i)
        {
            const auto &q = queries[i % queries.size()];

            const uint64_t a0 = bench::rdtsc_start();
            router.run(q.src, q.dep, out);
            const uint64_t a1 = bench::rdtsc_end();
            __asm__ volatile("" : : "r"(&out) : "memory");

            const uint64_t b0 = bench::rdtsc_start();
            raptor.run(q.src, q.dep, rout);
            const uint64_t b1 = bench::rdtsc_end();
            __asm__ volatile("" : : "r"(&rout) : "memory");

            et.push_back(bench::ticks_to_ns(a1 - a0, ticks_per_ns));
            rt.push_back(bench::ticks_to_ns(b1 - b0, ticks_per_ns));
        }
        timed = n;
        std::sort(et.begin(), et.end());
        std::sort(rt.begin(), rt.end());
        e50 = pct(et, 0.50);
        e95 = pct(et, 0.95);
        e99 = pct(et, 0.99);
        r50 = pct(rt, 0.50);
        r95 = pct(rt, 0.95);
        r99 = pct(rt, 0.99);
    }

    // ── Report ────────────────────────────────────────────────────────────────
    const double rt_k2plus_frac = rt_reached ? static_cast<double>(rt_k2plus) / rt_reached : 0.0;
    const double rt_mean_k = rt_reached ? static_cast<double>(rt_k_sum) / rt_reached : 0.0;
    const double en_k2plus_frac = en_reached ? static_cast<double>(en_k2plus) / en_reached : 0.0;
    const double en_mean_k = en_reached ? static_cast<double>(en_k_sum) / en_reached : 0.0;
    const double subopt_frac = ag_compared ? static_cast<double>(ag_later) / ag_compared : 0.0;
    double gap_mean = 0.0;
    for (const double g : gaps)
        gap_mean += g;
    if (!gaps.empty())
        gap_mean /= gaps.size();

    if (verbose)
    {
        std::printf("=== RAPTOR: the exact (arrival, trips) frontier ===\n");
        for (const auto &kv : rt_hist)
            std::printf("  k=%2u : %9llu  (%5.2f%%)\n", kv.first,
                        static_cast<unsigned long long>(kv.second),
                        100.0 * static_cast<double>(kv.second) / static_cast<double>(rt_reached));
        std::printf("  reached observations      : %llu of %llu\n",
                    static_cast<unsigned long long>(rt_reached),
                    static_cast<unsigned long long>(rt_obs));
        std::printf("  with a real trade-off k>1 : %llu  (%.2f%%)\n",
                    static_cast<unsigned long long>(rt_k2plus), 100.0 * rt_k2plus_frac);
        std::printf("  max / mean frontier size  : %u / %.3f\n", rt_max_k, rt_mean_k);
        std::printf("  deepest round used        : %u   (round cap hit on %llu queries)\n\n",
                    rt_max_round, static_cast<unsigned long long>(rt_cap_hits));

        std::printf("=== Engine: the frontier the shipped search finds ===\n");
        std::printf("  reached observations      : %llu of %llu\n",
                    static_cast<unsigned long long>(en_reached),
                    static_cast<unsigned long long>(en_obs));
        std::printf("  with k>1                  : %.2f%%   max %u, mean %.3f\n\n",
                    100.0 * en_k2plus_frac, en_max_k, en_mean_k);

        std::printf("=== Cost of the bounded-wait lookahead (lambda=0, k=5, W=1800s) ===\n");
        std::printf("  compared                  : %llu\n", static_cast<unsigned long long>(ag_compared));
        std::printf("  exactly equal             : %llu  (%.3f%%)\n",
                    static_cast<unsigned long long>(ag_equal),
                    ag_compared ? 100.0 * static_cast<double>(ag_equal) / ag_compared : 0.0);
        std::printf("  engine later than optimal : %llu  (%.3f%%)  mean gap %.1fs, p95 %.1fs, max %.1fs\n",
                    static_cast<unsigned long long>(ag_later), 100.0 * subopt_frac,
                    gap_mean, pct(gaps, 0.95), gaps.empty() ? 0.0 : gaps.back());
        std::printf("  engine found nothing      : %llu   (destination reachable, window missed it)\n",
                    static_cast<unsigned long long>(ag_unreached));
        std::printf("  engine EARLIER than exact : %llu   (must be zero)\n",
                    static_cast<unsigned long long>(ag_earlier));
        std::printf("  queries skipped           : %llu   (RAPTOR hit its %u-round cap, so its\n"
                    "                              arrivals there are upper bounds, not exact)\n\n",
                    static_cast<unsigned long long>(ag_skipped_capped), opt.max_rounds);

        if (timed > 0)
        {
            std::printf("=== Interleaved timing, %u paired samples (timer overhead %.0f ns) ===\n",
                        timed, t_overhead);
            std::printf("  engine  p50 %9.0f ns | p95 %9.0f | p99 %9.0f\n", e50, e95, e99);
            std::printf("  raptor  p50 %9.0f ns | p95 %9.0f | p99 %9.0f\n", r50, r95, r99);
            std::printf("  ratio   p50 %.2fx %s\n\n", r50 > 0 ? e50 / r50 : 0.0,
                        (r50 > 0 && e50 > r50) ? "(RAPTOR faster)" : "(engine faster)");
        }
        if (!note.empty())
            std::printf("note: %s\n\n", note.c_str());
    }

    // ── The row ───────────────────────────────────────────────────────────────
    std::printf("CSV:%s,%s,%u,%u,%u,%s,%.1f,%.1f,%u,%u,%u,%u,%d,%s,"
                "%zu,%llu,%llu,%llu,%llu,%.6f,%u,%.4f,%u,%llu,"
                "%llu,%llu,%.6f,%u,%.4f,"
                "%llu,%llu,%llu,%llu,%llu,%llu,"
                "%.6f,%.1f,%.1f,%.1f,"
                "%u,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.4f,%s\n",
                opt.slug.c_str(),
                arena_exhausted ? "partial" : "ok",
                graph.num_nodes, graph.num_edges, graph.num_transfers,
                graph.second_objective == SecondObjective::TransferCount ? "transfers" : "crowd",
                graph.memory_bytes() / 1024.0, tt.memory_bytes() / 1024.0,
                tt.num_routes, tt.num_trips, tt.routes_split_for_overtaking,
                tt.trips_split_for_bad_segment, closed ? 1 : 0,
                topology_csv_row(topo).c_str(),
                queries.size(),
                static_cast<unsigned long long>(rt_obs),
                static_cast<unsigned long long>(rt_reached),
                static_cast<unsigned long long>(rt_reached - rt_k2plus),
                static_cast<unsigned long long>(rt_k2plus),
                rt_k2plus_frac, rt_max_k, rt_mean_k, rt_max_round,
                static_cast<unsigned long long>(rt_cap_hits),
                static_cast<unsigned long long>(en_obs),
                static_cast<unsigned long long>(en_reached),
                en_k2plus_frac, en_max_k, en_mean_k,
                static_cast<unsigned long long>(ag_compared),
                static_cast<unsigned long long>(ag_equal),
                static_cast<unsigned long long>(ag_later),
                static_cast<unsigned long long>(ag_earlier),
                static_cast<unsigned long long>(ag_unreached),
                static_cast<unsigned long long>(ag_skipped_capped),
                subopt_frac, gap_mean, pct(gaps, 0.95), gaps.empty() ? 0.0 : gaps.back(),
                timed, t_overhead, e50, e95, e99, r50, r95, r99,
                (r50 > 0.0) ? e50 / r50 : 0.0,
                csv_escape(note).c_str());

    return (ag_earlier > 0) ? 3 : 0;
}
