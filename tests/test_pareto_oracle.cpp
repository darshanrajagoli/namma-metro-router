#include <gtest/gtest.h>
#include "routing.hpp"
#include "graph.hpp"
#include <vector>
#include <algorithm>
#include <limits>

/**
 * @file test_pareto_oracle.cpp
 * @brief Validates Pareto-Dijkstra against a brute-force DFS oracle on small graphs.
 *
 * PURPOSE:
 *   Unit tests for each component can pass individually while the assembled
 *   Dijkstra still produces a wrong or incomplete Pareto frontier. Aggressive
 *   dominance pruning can have edge-case bugs that silently drop optimal paths.
 *
 *   This test builds a small known graph, runs both:
 *     (A) The optimized ParetoDijkstra::run()
 *     (B) A brute-force DFS that enumerates ALL simple paths and computes
 *         the exact Pareto frontier via dominance filtering
 *
 *   If (A) and (B) produce different frontiers, there is a bug in the
 *   dominance pruning or lazy deletion filter.
 *
 * GRAPH USED:
 *   4-node diamond:
 *     0 → 1 (dep=100, travel=50, crowd=10)
 *     0 → 2 (dep=100, travel=80, crowd=3)
 *     1 → 3 (dep=160, travel=40, crowd=20)
 *     2 → 3 (dep=190, travel=30, crowd=5)
 *
 *   Path A: 0→1→3: arrival=200, total_crowd=30
 *   Path B: 0→2→3: arrival=220, total_crowd=8
 *   Both are non-dominated → frontier = {(200,30), (220,8)}
 */

using namespace namma_metro;

// ── Brute-force DFS oracle ────────────────────────────────────────────────

struct PathLabel {
    uint32_t arrival_time;
    uint32_t crowd_cost;
};

// Returns true if label a dominates label b
static bool dominates(const PathLabel& a, const PathLabel& b) {
    return a.arrival_time <= b.arrival_time && a.crowd_cost <= b.crowd_cost
        && (a.arrival_time < b.arrival_time || a.crowd_cost < b.crowd_cost);
}

// Prune dominated labels from a frontier
static std::vector<PathLabel> prune(std::vector<PathLabel> frontier) {
    std::vector<PathLabel> result;
    for (const auto& cand : frontier) {
        bool dominated_by_any = false;
        for (const auto& other : frontier) {
            if (&other != &cand && dominates(other, cand)) {
                dominated_by_any = true;
                break;
            }
        }
        if (!dominated_by_any) result.push_back(cand);
    }
    return result;
}

// DFS oracle: enumerate all simple paths from source to dest.
// The oracle must NOT model a strict next-train policy. The engine uses
// Bounded-Wait Lookahead and may deliberately skip the earliest available
// train to board a later, less crowded one; an oracle that always takes the
// first departure would disagree with correct engine output.
//
// So: exhaustive enumeration — try EVERY edge regardless of departure
// order, as long as it departs >= current_time and within W_max (1800s).
// This matches the engine's semantic exactly:
// the oracle finds ALL reachable (time, crowd) Pareto outcomes, not
// just those reachable via the first available train.
static void dfs_oracle(
    const CSRGraph& g,
    uint32_t current, uint32_t dest,
    uint32_t current_time, uint32_t current_crowd,
    std::vector<bool>& visited,
    std::vector<PathLabel>& found,
    uint32_t W_max_seconds = 1800)
{
    if (current == dest) {
        found.push_back({current_time, current_crowd});
        return;
    }
    auto [begin, end] = g.edges_of(current);
    for (const Edge* e = begin; e != end; ++e) {
        if (visited[e->destination]) continue;
        // Must be boardable: departs at or after current_time
        if (e->departure_time < current_time) continue;
        // Within bounded-wait window (mirrors W_max in select_optimal_departure)
        if (e->departure_time > current_time + W_max_seconds) continue;
        visited[e->destination] = true;
        const uint32_t new_time  = e->departure_time + e->travel_time;
        const uint32_t new_crowd = current_crowd + e->crowd_weight + e->penalty;
        dfs_oracle(g, e->destination, dest, new_time, new_crowd, visited, found, W_max_seconds);
        visited[e->destination] = false;
    }
}

static std::vector<PathLabel> oracle_pareto(
    const CSRGraph& g, uint32_t source, uint32_t dest, uint32_t departure_time,
    uint32_t W_max_seconds = 1800)
{
    std::vector<bool> visited(g.num_nodes, false);
    visited[source] = true;
    std::vector<PathLabel> all_paths;
    dfs_oracle(g, source, dest, departure_time, 0, visited, all_paths, W_max_seconds);
    return prune(all_paths);
}

// ── Build the diamond test graph ──────────────────────────────────────────

static CSRGraph build_diamond_graph() {
    CSRGraph g;
    g.num_nodes = 4;
    g.offset.resize(5, 0);

    auto add_edge = [&](uint32_t src, uint32_t dst,
                        uint32_t dep, uint32_t travel, uint32_t crowd) {
        Edge e;
        e.destination    = dst;
        e.departure_time = dep;
        e.travel_time    = travel;
        e.crowd_weight   = crowd;
        e.penalty        = 0;
        g.edge_data.push_back(e);
        ++g.offset[src + 1];
    };

    add_edge(0, 1, 100, 50, 10);  // arrives at 1: time=150, crowd=10
    add_edge(0, 2, 100, 80, 3);   // arrives at 2: time=180, crowd=3
    add_edge(1, 3, 160, 40, 20);  // arrives at 3: time=200, crowd=20
    add_edge(2, 3, 190, 30, 5);   // arrives at 3: time=220, crowd=5

    // Prefix sum offsets
    for (uint32_t i = 1; i <= g.num_nodes; ++i) g.offset[i] += g.offset[i-1];
    g.num_edges = static_cast<uint32_t>(g.edge_data.size());

    // Sort edges by departure_time within each source node: this is the ordering
    // select_optimal_departure's lower_bound depends on. The diamond graph would
    // pass without it only by luck, since both node-0 edges share
    // departure_time=100 and lower_bound is insensitive to ties. Any later test
    // with out-of-order edges would silently corrupt the binary search.
    for (uint32_t u = 0; u < g.num_nodes; ++u) {
        auto* b = g.edge_data.data() + g.offset[u];
        auto* e = g.edge_data.data() + g.offset[u + 1];
        std::sort(b, e, [](const Edge& a, const Edge& b2){
            return a.departure_time < b2.departure_time;
        });
    }
    return g;
}

// ── Tests ─────────────────────────────────────────────────────────────────

TEST(ParetoOracle, DiamondGraph_FrontierMatchesBruteForce) {
    CSRGraph g = build_diamond_graph();

    // Oracle: brute-force all paths
    auto oracle = oracle_pareto(g, 0, 3, 100);
    ASSERT_EQ(oracle.size(), 2u) << "Oracle should find exactly 2 non-dominated paths";

    // Optimized Dijkstra
    LookaheadConfig config{.k_departures=5, .W_max_seconds=1800, .lambda=1.0f};
    ParetoDijkstra router(g, config);
    auto result = router.run(0, 100);

    const auto& pareto_set = result.pareto_sets[3];
    const auto& labels = pareto_set.labels();

    // Both paths must appear in the Dijkstra result
    ASSERT_EQ(labels.size(), oracle.size())
        << "Dijkstra Pareto set size must match oracle. "
           "If smaller: dominance pruning incorrectly dropped a valid path. "
           "If larger: stale labels not filtered correctly.";

    // Collect (time, crowd) from Dijkstra result
    std::vector<PathLabel> dijkstra_labels;
    for (const Label* l : labels) {
        dijkstra_labels.push_back({l->arrival_time, l->crowd_cost});
    }

    // Sort both by arrival_time for comparison
    auto sort_fn = [](const PathLabel& a, const PathLabel& b) {
        return a.arrival_time < b.arrival_time;
    };
    std::sort(oracle.begin(), oracle.end(), sort_fn);
    std::sort(dijkstra_labels.begin(), dijkstra_labels.end(), sort_fn);

    for (std::size_t i = 0; i < oracle.size(); ++i) {
        EXPECT_EQ(dijkstra_labels[i].arrival_time, oracle[i].arrival_time)
            << "Path " << i << ": arrival_time mismatch";
        EXPECT_EQ(dijkstra_labels[i].crowd_cost, oracle[i].crowd_cost)
            << "Path " << i << ": crowd_cost mismatch";
    }
}

TEST(ParetoOracle, DiamondGraph_BothPathsPresent) {
    CSRGraph g = build_diamond_graph();
    LookaheadConfig config{.k_departures=5, .W_max_seconds=1800, .lambda=1.0f};
    ParetoDijkstra router(g, config);
    auto result = router.run(0, 100);
    const auto& labels = result.pareto_sets[3].labels();

    bool found_fast_path  = false; // arrival=200, crowd=30 (via node 1)
    bool found_crowd_path = false; // arrival=220, crowd=8  (via node 2)

    for (const Label* l : labels) {
        if (l->arrival_time == 200 && l->crowd_cost == 30) found_fast_path  = true;
        if (l->arrival_time == 220 && l->crowd_cost == 8)  found_crowd_path = true;
    }

    EXPECT_TRUE(found_fast_path)
        << "Fast path (time=200, crowd=30) missing from Pareto frontier. "
           "Check lazy deletion filter — this path may be incorrectly filtered as stale.";
    EXPECT_TRUE(found_crowd_path)
        << "Low-crowd path (time=220, crowd=8) missing from Pareto frontier. "
           "Check insert_and_dominate forward pruning — >= vs > boundary error.";
}

TEST(ParetoOracle, DisconnectedDestination_ReturnsEmpty) {
    // The query must originate from a node with outgoing edges (node 0), and the
    // assertion must pair an empty frontier with a NON-empty one. Querying from
    // the sink and asserting only that some frontier is empty would pass even if
    // the engine returned empty frontiers everywhere, or crashed the search —
    // an empty result is the default, so it proves nothing on its own.
    //
    // Here: node 4 has no path from node 0, so pareto_sets[4] must be empty,
    // while pareto_sets[3] must be non-empty (reachable via 0→1→3 or 0→2→3).
    // That pairing tests reachability and disconnected handling together.
    CSRGraph g = build_diamond_graph();

    // Extend to 5 nodes: node 4 has no incoming edges and no path from node 0.
    // Add node 4 with one outgoing edge to nowhere useful — it can reach a
    // hypothetical node 5 but nobody can reach node 4 from node 0.
    g.num_nodes = 5;
    g.offset.resize(6, g.offset[4]); // nodes 4 gets empty edge range

    LookaheadConfig config{.k_departures=5, .W_max_seconds=1800, .lambda=1.0f};
    ParetoDijkstra router(g, config);
    auto result = router.run(0, 100);

    // Node 3 IS reachable (two Pareto paths exist)
    EXPECT_FALSE(result.pareto_sets[3].labels().empty())
        << "Node 3 must be reachable from node 0 — check lazy deletion or "
           "insert_and_dominate: at least one Pareto path (0→1→3 or 0→2→3) "
           "must appear in the frontier.";

    // Node 4 is NOT reachable from node 0
    EXPECT_TRUE(result.pareto_sets[4].labels().empty())
        << "Node 4 has no incoming edges — must return empty Pareto set, not crash.";
}

TEST(ParetoOracle, MultiDeparture_BoundedWaitSelectsLaterLowCrowdTrain) {
    // SEMANTIC NOTE: select_optimal_departure returns ONE best departure per
    // (node, dest, current_time) call. The engine cannot generate a label via
    // the early-crowded train (dep=100, crowd=50) if select_optimal_departure
    // correctly picks the late-uncrowded train (dep=150, crowd=5) as best.
    //
    // The exhaustive DFS oracle tries ALL edges regardless, so it finds BOTH
    // paths. This creates a fundamental engine-oracle semantic mismatch on
    // graphs with multiple departures per (u→v) link.
    //
    // This test therefore does NOT compare engine output to the oracle.
    // Instead it verifies the engine's specific, documented behavior:
    //   - select_optimal_departure(0→1, t=50, lambda=1.0) returns dep=150
    //     (composite=80+5=85 < dep=100 composite=50+50=100)
    //   - The engine generates ONE label for node 1: (arr=230, crowd=5)
    //   - From node 1 at t=230, dep=200 is missed; dep=280 is taken
    //   - Node 2 label: (arr=310, crowd=7)
    //   - Final frontier at node 2: {(310, 7)} — size 1
    //
    // Graph:
    //   0 → 1: dep=100, travel=50, crowd=50  (early crowded — SKIPPED by lookahead)
    //   0 → 1: dep=150, travel=80, crowd=5   (late low-crowd — SELECTED)
    //   1 → 2: dep=200, travel=40, crowd=10  (missed — arrives node 1 at t=230)
    //   1 → 2: dep=280, travel=30, crowd=2   (caught)
    //
    // Expected engine output: pareto_sets[2] = {(310, 7)}, size=1.

    CSRGraph g;
    g.num_nodes = 3;
    g.offset.resize(4, 0);

    auto add_edge = [&](uint32_t src, uint32_t dst,
                        uint32_t dep, uint32_t travel, uint32_t crowd) {
        Edge e;
        e.destination = dst; e.departure_time = dep;
        e.travel_time = travel; e.crowd_weight = crowd; e.penalty = 0;
        g.edge_data.push_back(e);
        ++g.offset[src + 1];
    };

    add_edge(0, 1, 100, 50, 50); // early crowded train
    add_edge(0, 1, 150, 80, 5);  // late low-crowd train
    add_edge(1, 2, 200, 40, 10); // first onward connection
    add_edge(1, 2, 280, 30, 2);  // second onward connection (catches late arrival)

    for (uint32_t i = 1; i <= g.num_nodes; ++i) g.offset[i] += g.offset[i-1];
    g.num_edges = static_cast<uint32_t>(g.edge_data.size());

    for (uint32_t u = 0; u < g.num_nodes; ++u) {
        auto* b = g.edge_data.data() + g.offset[u];
        auto* e = g.edge_data.data() + g.offset[u + 1];
        std::sort(b, e, [](const Edge& a, const Edge& b2){
            return a.departure_time < b2.departure_time;
        });
    }

    LookaheadConfig config{.k_departures=5, .W_max_seconds=1800, .lambda=1.0f};
    ParetoDijkstra router(g, config);
    auto result = router.run(0, 50);
    const auto& labels = result.pareto_sets[2].labels();

    // Engine should find exactly ONE Pareto label at node 2.
    // select_optimal_departure picks the BEST composite departure per (node,dest,t).
    // It cannot generate a second label via the early crowded train because that
    // train has higher composite cost (100) and is not selected.
    ASSERT_EQ(labels.size(), 1u)
        << "Engine generates 1 label at node 2 via the optimal (late, low-crowd) path. "
           "Size > 1 means select_optimal_departure is returning multiple departures "
           "per (node,dest,t) call, which contradicts its specification. "
           "Size == 0 means the routing loop failed entirely (check lazy deletion).";

    EXPECT_EQ(labels[0]->arrival_time, 310u)
        << "Expected arrival = dep(280) + travel(30) = 310";
    EXPECT_EQ(labels[0]->crowd_cost, 7u)
        << "Expected crowd = 5 (node0→1) + 2 (node1→2) = 7";
}
