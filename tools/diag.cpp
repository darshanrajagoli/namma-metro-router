// Diagnostics: is the bi-criteria machinery actually load-bearing?
//
// Answers three questions the benchmark cannot:
//   1. How large are the Pareto frontiers in practice? If every node holds one
//      label, the engine is a single-objective Dijkstra wearing a costume.
//   2. Does lambda change anything? If routes are identical at lambda=0 and
//      lambda=100, the crowd objective is inert.
//   3. What is the real peak label count vs the 65,536-slot arena?
#include "gtfs_parser.hpp"
#include "graph.hpp"
#include "routing.hpp"
#include <cstdio>
#include <map>
#include <random>
#include <string>
#include <vector>
#include <algorithm>

using namespace namma_metro;

struct Outcome { uint32_t arrival; uint32_t crowd; bool reached; };

static std::vector<Outcome> collect(ParetoDijkstra& r, uint32_t src, uint32_t dep, uint32_t n) {
    auto res = r.run(src, dep);
    std::vector<Outcome> out(n, {0, 0, false});
    for (uint32_t v = 0; v < n; ++v) {
        const auto& L = res.pareto_sets[v].labels();
        if (L.empty()) continue;
        // Canonical representative: earliest arrival on the frontier.
        out[v] = {L.front()->arrival_time, L.front()->secondary_cost, true};
    }
    return out;
}

int main(int argc, char** argv) {
    const std::string dir = (argc > 1) ? argv[1] : "./data";
    GTFSParser p(dir);
    try {
        p.load_agency(); p.check_frequencies(); p.load_stops(); p.load_routes();
        p.load_trips(); p.load_stop_times(); p.load_calendar();
        p.load_calendar_dates(); p.load_transfers(); p.interpolate_stop_times();
    } catch (const std::exception& e) { std::fprintf(stderr, "load failed: %s\n", e.what()); return 1; }

    if (p.stop_times().empty()) { std::fprintf(stderr, "no stop_times loaded\n"); return 1; }

    // Mirror main_bench: a feed carrying transfers is a transfer-count feed.
    const bool has_tr = !p.transfers().empty();
    CSRGraph g = GraphBuilder::build_with_transfers(
        p.stop_times(), static_cast<uint32_t>(p.stops().size()), &p.stop_index_map(),
        p.transfers(),
        has_tr ? SecondObjective::TransferCount : SecondObjective::CrowdExposure);
    std::printf("feed=%s  nodes=%u  edges=%u  transfers=%u  second objective=%s\n\n",
                dir.c_str(), g.num_nodes, g.num_edges, g.num_transfers,
                has_tr ? "transfer count" : "crowd exposure");

    // ── Query set: reachable (src, dep) pairs, same screening as the benchmark ──
    std::mt19937 rng(123);
    std::uniform_int_distribution<uint32_t> snode(0, g.num_nodes - 1);
    std::uniform_int_distribution<uint32_t> stime(25200, 75600);
    LookaheadConfig base{.k_departures = 5, .W_max_seconds = 1800, .lambda = 1.0f};
    ParetoDijkstra probe(g, base);
    std::vector<std::pair<uint32_t,uint32_t>> Q;
    for (int a = 0; a < 4000 && Q.size() < 300; ++a) {
        uint32_t s = snode(rng), t = stime(rng);
        auto res = probe.run(s, t);
        bool any = false;
        for (uint32_t v = 0; v < g.num_nodes && !any; ++v)
            if (v != s && !res.pareto_sets[v].empty()) any = true;
        if (any) Q.emplace_back(s, t);
    }
    std::printf("query set: %zu reachable (src, departure) pairs\n\n", Q.size());

    // ── Q1: frontier size distribution ──────────────────────────────────────
    std::map<std::size_t, long> hist;
    long reached = 0, multi = 0, total_labels = 0, peak_labels_one_query = 0;
    std::size_t max_k = 0;
    for (auto [s, t] : Q) {
        auto res = probe.run(s, t);
        long this_query = 0;
        for (uint32_t v = 0; v < g.num_nodes; ++v) {
            std::size_t k = res.pareto_sets[v].size();
            this_query += static_cast<long>(k);
            if (k == 0) continue;
            ++reached; hist[k]++; total_labels += static_cast<long>(k);
            if (k > 1) ++multi;
            max_k = std::max(max_k, k);
        }
        peak_labels_one_query = std::max(peak_labels_one_query, this_query);
    }
    std::printf("=== Q1. Pareto frontier sizes (per reached node, over all queries) ===\n");
    for (auto [k, c] : hist)
        std::printf("  k=%2zu : %8ld nodes  (%5.2f%%)\n", k, c, 100.0 * c / reached);
    std::printf("  reached-node observations : %ld\n", reached);
    std::printf("  with k>1 (a real trade-off): %ld  (%.3f%%)\n", multi, 100.0 * multi / reached);
    std::printf("  max frontier size observed: %zu   (worst-case bound assumes k <= 16)\n", max_k);
    std::printf("  mean frontier size        : %.3f\n", double(total_labels) / reached);
    std::printf("  peak surviving labels in one query: %ld  (arena capacity 65536)\n\n",
                peak_labels_one_query);

    // ── Q2: does lambda change the answer? ──────────────────────────────────
    std::printf("=== Q2. Lambda sensitivity (vs lambda=0, the time-only optimum) ===\n");
    const float lambdas[] = {0.0f, 0.5f, 1.0f, 5.0f, 50.0f, 1000.0f};
    std::vector<std::vector<Outcome>> ref;
    for (auto [s, t] : Q) { LookaheadConfig c = base; c.lambda = 0.0f;
        ParetoDijkstra r(g, c); ref.push_back(collect(r, s, t, g.num_nodes)); }

    for (float lam : lambdas) {
        LookaheadConfig c = base; c.lambda = lam;
        ParetoDijkstra r(g, c);
        long diff_arr = 0, diff_crowd = 0, cmp = 0;
        double sum_arr_delta = 0, sum_crowd_delta = 0;
        for (std::size_t i = 0; i < Q.size(); ++i) {
            auto cur = collect(r, Q[i].first, Q[i].second, g.num_nodes);
            for (uint32_t v = 0; v < g.num_nodes; ++v) {
                if (!ref[i][v].reached || !cur[v].reached) continue;
                ++cmp;
                if (cur[v].arrival != ref[i][v].arrival) ++diff_arr;
                if (cur[v].crowd   != ref[i][v].crowd)   ++diff_crowd;
                sum_arr_delta   += double(cur[v].arrival) - double(ref[i][v].arrival);
                sum_crowd_delta += double(cur[v].crowd)   - double(ref[i][v].crowd);
            }
        }
        std::printf("  lambda=%-7.1f arrival differs %6.2f%% | crowd differs %6.2f%% | "
                    "mean dt %+8.1fs | mean dcrowd %+9.1f\n",
                    lam, 100.0 * diff_arr / cmp, 100.0 * diff_crowd / cmp,
                    sum_arr_delta / cmp, sum_crowd_delta / cmp);
    }
    return 0;
}
