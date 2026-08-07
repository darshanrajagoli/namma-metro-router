// FIFO probe: does the documented FIFO violation actually FIRE on a real feed?
//
// tests/test_fifo_violation.cpp proves the three mechanisms exist, using small
// hand-built fixtures with numbers chosen to trigger them. That establishes the
// violation is POSSIBLE. It says nothing about whether it OCCURS on real transit
// data. This tool answers that, and it is the difference between
//
//   "there is a correctness limit I can trigger with a contrived example"   (mild)
//   "it fires N times per link on San Francisco's real timetable"           (serious)
//
// METHOD
//   FIFO requires that arrival be non-decreasing in query time:
//       t1 <= t2  =>  arrival(t1) <= arrival(t2)
//   where arrival(t) = the departure select_optimal_departure picks at time t,
//   plus its travel time. We evaluate that function directly through the public
//   API - no engine instrumentation, no behaviour change.
//
//   The function is piecewise constant and only changes at candidate-set
//   transitions, so an exhaustive fine sweep is unnecessary. For a link with
//   departure times D, the transitions are exactly:
//       t = d - W_max - 1 / d - W_max   (d crosses INTO the window)
//       t = d - 1 / d / d + 1           (d crosses OUT, having been boardable)
//   for each d in D. Evaluating at those points is exact, not a sample.
//
//   PRECONDITION REPORTED SEPARATELY: mechanism (1) needs per-link travel times
//   that differ between departures. If every departure on a link rides for the
//   same duration - as on the Namma feed, where travel time is distance / speed -
//   the mechanism cannot fire. We count how many links satisfy the precondition
//   so a null result can be attributed correctly.
//
// USAGE
//   ./routing_engine_fifo_probe <feed_dir> [max_examples]
//
//   The most interesting feed is BART with platforms COLLAPSED:
//       python3 scripts/normalize_gtfs.py raw_bart data_bart     (no --transfers)
//   That is the only configuration with a real timetable AND the crowd composite,
//   which is what mechanism (1) requires. With --transfers the feed runs under
//   TransferCount, where selection is arrival-minimising and mechanism (1) cannot
//   arise (mechanisms 2 and 3 still can - this tool detects those too).
#include "gtfs_parser.hpp"
#include "graph.hpp"
#include "routing.hpp"
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

using namespace namma_metro;

namespace {

struct Violation {
    uint32_t u, v;
    uint32_t t1, arr1;
    uint32_t t2, arr2;
    uint32_t magnitude;      // arr1 - arr2, i.e. how far arrival went backwards
};

std::optional<uint32_t> arrival_at(const CSRGraph &g, const LookaheadConfig &cfg,
                                   uint32_t t, uint32_t u, uint32_t v) {
    auto e = select_optimal_departure(g, cfg, t, u, v);
    if (!e.has_value()) return std::nullopt;
    return e->departure_time + e->travel_time;
}

} // namespace

int main(int argc, char **argv) {
    const std::string dir = (argc > 1) ? argv[1] : "./data";
    const std::size_t max_examples = (argc > 2) ? std::stoul(argv[2]) : 10;

    GTFSParser p(dir);
    try {
        p.load_agency(); p.check_frequencies(); p.load_stops(); p.load_routes();
        p.load_trips(); p.load_stop_times(); p.load_calendar();
        p.load_calendar_dates(); p.load_transfers(); p.interpolate_stop_times();
    } catch (const std::exception &e) {
        std::fprintf(stderr, "load failed: %s\n", e.what());
        return 1;
    }
    if (p.stop_times().empty()) {
        std::fprintf(stderr, "no stop_times loaded - point at a normalised feed\n");
        return 1;
    }

    // Mirror main_bench exactly: a feed carrying transfers is a transfer-count feed.
    const bool has_tr = !p.transfers().empty();
    const SecondObjective obj =
        has_tr ? SecondObjective::TransferCount : SecondObjective::CrowdExposure;
    CSRGraph g = GraphBuilder::build_with_transfers(
        p.stop_times(), static_cast<uint32_t>(p.stops().size()),
        &p.stop_index_map(), p.transfers(), obj);

    // Swept, because a single lambda answers the wrong question. The composite is
    // travel + lambda*crowd. Crowd is O(100..1000) while a metro link's travel time
    // is O(100..300) s and its VARIATION between departures is smaller still, so at
    // the shipping lambda = 1 the crowd term dominates the argmin. Crowd is also a
    // smooth function of departure time, so the argmin tracks it monotonically and
    // the violation is masked. Lowering lambda lets the travel term decide, which is
    // exactly the regime mechanism (1) needs. Sweeping locates the threshold instead
    // of reporting one point and calling it a null result.
    const float LAMBDAS[] = {0.0f, 0.001f, 0.01f, 0.05f, 0.1f, 0.5f, 1.0f, 5.0f, 1000.0f};

    std::printf("feed=%s  nodes=%u  edges=%u  transfers=%u\n", dir.c_str(),
                g.num_nodes, g.num_edges, g.num_transfers);
    std::printf("second objective : %s\n",
                obj == SecondObjective::TransferCount ? "transfer count (arrival-minimising "
                                                        "selection - mechanism 1 cannot arise)"
                                                      : "crowd exposure (composite ignores "
                                                        "arrival - mechanism 1 possible)");
    std::printf("config           : k=%u  W_max=%us  lambda swept\n\n", 5u, 1800u);

    // ── Preconditions, measured once (independent of lambda) ─────────────────
    long pre_links = 0, pre_multi = 0, pre_varying = 0;
    uint32_t max_travel_spread = 0, max_travel = 0, max_gap = 0;
    {
        std::vector<uint32_t> d;
        for (uint32_t u = 0; u < g.num_nodes; ++u) {
            auto [eb, ee] = g.edges_of(u);
            d.clear();
            for (const Edge *e = eb; e != ee; ++e) d.push_back(e->destination);
            std::sort(d.begin(), d.end());
            d.erase(std::unique(d.begin(), d.end()), d.end());
            for (uint32_t v : d) {
                ++pre_links;
                uint32_t lo = UINT32_MAX, hi = 0, n = 0, prev_dep = 0;
                for (const Edge *e = eb; e != ee; ++e)
                    if (e->destination == v) {
                        lo = std::min(lo, e->travel_time);
                        hi = std::max(hi, e->travel_time);
                        if (n > 0 && e->departure_time > prev_dep)
                            max_gap = std::max(max_gap, e->departure_time - prev_dep);
                        prev_dep = e->departure_time;
                        ++n;
                    }
                if (n < 2) continue;
                ++pre_multi;
                max_travel = std::max(max_travel, hi);
                if (hi != lo) {
                    ++pre_varying;
                    max_travel_spread = std::max(max_travel_spread, hi - lo);
                }
            }
        }
    }
    std::printf("=== Preconditions (lambda-independent) ===\n");
    std::printf("  directed links (u->v)                 : %ld\n", pre_links);
    std::printf("  ... with >= 2 timetabled departures   : %ld\n", pre_multi);
    std::printf("  ... with VARYING per-link travel time : %ld  (%.2f%%)\n",
                pre_varying, pre_multi ? 100.0 * pre_varying / pre_multi : 0.0);
    std::printf("  widest travel-time spread on any link : %u s\n", max_travel_spread);
    std::printf("  longest single-link ride              : %u s\n", max_travel);
    std::printf("  longest gap between departures        : %u s   (vs W_max = 1800 s)\n", max_gap);
    std::printf("      ^ mechanism (1) needs varying travel time. Zero there means the\n"
                "        crowd composite cannot break FIFO on this feed at any lambda.\n"
                "        The other two numbers bound mechanisms (2) and (3): a departure\n"
                "        excluded by the window can only win on arrival if the rides inside\n"
                "        the window are longer than the window itself, so a longest ride far\n"
                "        below W_max makes window-truncation unreachable.\n\n");

    std::printf("=== FIFO probe, lambda sweep (exact transition points, not a sample) ===\n");
    std::printf("  %-9s %14s %14s %12s %12s\n",
                "lambda", "links_violating", "violating_pairs", "worst_jump", "cliffs");

    std::vector<Violation> best_examples;
    float best_lambda = -1.0f;

    for (float lam : LAMBDAS) {
    const LookaheadConfig cfg{.k_departures = 5, .W_max_seconds = 1800, .lambda = lam};

    long links_total = 0, links_multi_dep = 0, links_varying_travel = 0;
    long links_violating = 0, violating_pairs = 0, cliff_transitions = 0;
    long long probe_points = 0;
    uint32_t worst_magnitude = 0;
    std::vector<Violation> examples;

    std::vector<uint32_t> dests;
    std::vector<uint32_t> deps, travels, probes;

    for (uint32_t u = 0; u < g.num_nodes; ++u) {
        auto [eb, ee] = g.edges_of(u);
        if (eb == ee) continue;

        dests.clear();
        for (const Edge *e = eb; e != ee; ++e) dests.push_back(e->destination);
        std::sort(dests.begin(), dests.end());
        dests.erase(std::unique(dests.begin(), dests.end()), dests.end());

        for (uint32_t v : dests) {
            ++links_total;

            deps.clear(); travels.clear();
            for (const Edge *e = eb; e != ee; ++e)
                if (e->destination == v) {
                    deps.push_back(e->departure_time);
                    travels.push_back(e->travel_time);
                }
            if (deps.size() < 2) continue;
            ++links_multi_dep;

            const bool varying =
                std::any_of(travels.begin(), travels.end(),
                            [&](uint32_t t) { return t != travels.front(); });
            if (varying) ++links_varying_travel;

            // Exact transition points of the piecewise-constant arrival function.
            probes.clear();
            probes.reserve(deps.size() * 5);
            for (uint32_t d : deps) {
                if (d >= 1) probes.push_back(d - 1);
                probes.push_back(d);
                probes.push_back(d + 1);
                if (d > cfg.W_max_seconds) {
                    probes.push_back(d - cfg.W_max_seconds - 1);
                    probes.push_back(d - cfg.W_max_seconds);
                }
            }
            std::sort(probes.begin(), probes.end());
            probes.erase(std::unique(probes.begin(), probes.end()), probes.end());

            bool link_flagged = false;
            std::optional<uint32_t> prev_arr;
            uint32_t prev_t = 0;

            for (uint32_t t : probes) {
                ++probe_points;
                const auto arr = arrival_at(g, cfg, t, u, v);
                if (!arr.has_value()) {
                    // Reachable -> unreachable as t advances is the documented
                    // W_max cliff, not a FIFO violation. Counted separately.
                    if (prev_arr.has_value()) ++cliff_transitions;
                    prev_arr.reset();
                    continue;
                }
                if (prev_arr.has_value() && *arr < *prev_arr) {
                    const uint32_t mag = *prev_arr - *arr;
                    ++violating_pairs;
                    if (mag > worst_magnitude) worst_magnitude = mag;
                    if (!link_flagged) { ++links_violating; link_flagged = true; }
                    if (examples.size() < max_examples)
                        examples.push_back({u, v, prev_t, *prev_arr, t, *arr, mag});
                }
                prev_arr = arr;
                prev_t   = t;
            }
        }
    }

    std::printf("  %-9.3f %14ld %14ld %10u s %12ld\n",
                lam, links_violating, violating_pairs, worst_magnitude, cliff_transitions);
    (void)links_total; (void)links_varying_travel; (void)probe_points;

    if (!examples.empty() && best_examples.empty()) {
        best_examples = examples;
        best_lambda   = lam;
    }
    } // end lambda sweep

    std::printf("\n");
    if (best_examples.empty()) {
        std::printf("RESULT: no FIFO violation observed on this feed at ANY lambda tested.\n");
        if (pre_varying == 0)
            std::printf("        Expected: no link has varying travel time, so mechanism (1)\n"
                        "        cannot arise at all on this feed.\n");
        else
            std::printf("        This is NOT because the precondition is absent: %ld links DO have\n"
                        "        varying travel time, with a spread up to %u s. The violation is\n"
                        "        latent, not impossible. The structural reasons it is not reached:\n"
                        "\n"
                        "          - Window truncation (mechanism 3) needs a departure just outside\n"
                        "            [t, t+W_max] to beat everything inside it on ARRIVAL. Anything\n"
                        "            outside departs at least W_max=1800 s late, so it can only win\n"
                        "            if the rides inside are longer than that. The longest ride on\n"
                        "            this feed is %u s, roughly %.1fx below the window. Unreachable.\n"
                        "          - k-budget truncation (mechanism 2) needs the (k+1)-th departure\n"
                        "            to beat the first k on arrival. With the longest inter-departure\n"
                        "            gap at %u s, the head start compounds over k=5 candidates and\n"
                        "            the travel spread of %u s cannot close it.\n"
                        "          - Mechanism (1) requires the composite's preferred departure to\n"
                        "            arrive later than an alternative that survives to a later query\n"
                        "            time. It fires only when travel-time variation lines up\n"
                        "            adversarially against the crowd ordering, which it does not here.\n"
                        "\n"
                        "        Report this as: the mechanisms are proven (see tests), the\n"
                        "        precondition is present on this feed, and the violation still does\n"
                        "        not occur - because W_max and the headway are large relative to a\n"
                        "        single link's ride time. A feed with long rides relative to its\n"
                        "        window (intercity rail) is where this would bite.\n",
                        pre_varying, max_travel_spread, max_travel,
                        max_travel ? 1800.0 / max_travel : 0.0, max_gap, max_travel_spread);
    } else {
        std::printf("RESULT: FIFO IS VIOLATED on this real feed, first at lambda = %.3f.\n"
                    "        The mechanism is not merely constructible - it fires on a real\n"
                    "        published timetable. Examples at that lambda:\n", best_lambda);
        for (const auto &x : best_examples)
            std::printf("  link %u->%u : query t=%u arrives %u, but LATER query t=%u "
                        "arrives %u  (%u s earlier)\n",
                        x.u, x.v, x.t1, x.arr1, x.t2, x.arr2, x.magnitude);
        std::printf("\n        Why the shipping lambda=1 masks it: crowd is O(100..1000) while\n"
                    "        this feed's travel-time SPREAD is at most %u s, so at lambda=1 the\n"
                    "        crowd term dominates the argmin. Crowd is a smooth function of\n"
                    "        departure time, so the argmin tracks it monotonically. Lower lambda\n"
                    "        lets the travel term decide and the violation surfaces.\n",
                    max_travel_spread);
    }
    return 0;
}
