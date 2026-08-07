#include <gtest/gtest.h>
#include "routing.hpp"
#include "graph.hpp"
#include <vector>

/**
 * @file test_lazy_deletion.cpp
 * @brief Google Test suite for the Lazy Deletion priority queue filter in Dijkstra.
 *
 * PURPOSE:
 *   std::priority_queue does not support decrease-key. The standard workaround
 *   is "lazy deletion": push duplicate (stale) labels with higher cost into the
 *   heap, and skip them when popped if a better label has already been processed.
 *
 *   Without the lazy deletion filter, the algorithm:
 *     1. Processes stale labels wastefully (wrong path costs)
 *     2. Produces incorrect results on graphs with multiple relaxation paths
 *
 *   These tests verify that ParetoDijkstra::run() produces CORRECT shortest
 *   paths on graphs that require multiple relaxations (i.e., would fail without
 *   lazy deletion).
 *
 * THE FILTER UNDER TEST:
 *   In src/routing.cpp, inside the ParetoDijkstra::run() Dijkstra loop,
 *   immediately after popping a label L from the priority queue:
 *
 *   THIS IS PARETO-DIJKSTRA. Do NOT use the single-objective check:
 *     if (L.arrival_time > best_time[L.node]) continue;
 *   That discards valid Pareto labels with higher time but lower crowd cost.
 *   It produces an incomplete frontier that may pass some tests but is wrong
 *   on real multi-path graphs.
 *
 *   CORRECT BI-CRITERIA lazy deletion filter (STRICT dominance):
 *     bool dominated = false;
 *     for (const Label* s : result.pareto_sets[L.node].labels()) {
 *         if (s->arrival_time <= L.arrival_time &&
 *             s->secondary_cost  <= L.secondary_cost  &&
 *             (s->arrival_time < L.arrival_time ||   // ← REQUIRED: at least one
 *              s->secondary_cost   < L.secondary_cost)) {    //   dimension strictly better
 *             dominated = true;
 *             break;
 *         }
 *     }
 *     if (dominated) continue;
 *
 *   WHY THE STRICT CLAUSE IS MANDATORY:
 *     The source label is inserted into pareto_sets[source] BEFORE being pushed
 *     to the PQ (see run() initialisation). When it is first popped, non-strict
 *     dominance (<=,<= only) finds the label in the settled set and self-dominates
 *     it → run() returns empty frontiers for every query. Adding the strict-one-
 *     better clause prevents a label from dominating itself.
 *
 *   This is O(k) per pop where k = settled label count at that node.
 *   Measured max k is 5 (BART under TransferCount) and 1 on the Namma feed,
 *   against a worst-case bound of 16, so the scan is short and bounded.
 *
 * CORRECTNESS ARGUMENT:
 *   The Dijkstra invariant: once a node is finalized (min-cost label popped),
 *   all subsequent pops of that node are stale. With lazy deletion, stale pops
 *   are O(1) to detect and skip. Without it, stale relaxations propagate
 *   incorrect costs to neighbors, violating the optimality subpath property.
 */

using namespace namma_metro;

// ─────────────────────────────────────────────────────────────────────────
// Test fixture: builds controlled CSR graphs for Dijkstra testing
// ─────────────────────────────────────────────────────────────────────────
class LazyDeletionTest : public ::testing::Test {
protected:
    CSRGraph graph;

    /**
     * @brief Build graph from adjacency list: edges[u] = list of (v, dep, trav, crowd, pen)
     */
    void build_graph(uint32_t num_nodes,
                     std::vector<std::vector<std::tuple<uint32_t,uint32_t,uint32_t,uint32_t,uint32_t>>> adj) {
        graph.num_nodes = num_nodes;
        graph.offset.resize(num_nodes + 1, 0);

        // Count edges per node
        for (uint32_t u = 0; u < num_nodes; ++u) {
            graph.offset[u] = static_cast<uint32_t>(graph.edge_data.size());
            if (u < adj.size()) {
                for (auto& [v, dep, trav, crowd, pen] : adj[u]) {
                    Edge e;
                    e.destination    = v;
                    e.departure_time = dep;
                    e.travel_time    = trav;
                    e.secondary_weight   = crowd;
                    e.penalty        = pen;
                    graph.edge_data.push_back(e);
                }
            }
        }
        graph.offset[num_nodes] = static_cast<uint32_t>(graph.edge_data.size());
        graph.num_edges = static_cast<uint32_t>(graph.edge_data.size());

        // Sort each node's edges by departure_time: select_optimal_departure()
        // uses std::lower_bound to find the first eligible departure, which
        // requires the range to be sorted. GraphBuilder guarantees this for real
        // feeds; test fixtures that construct CSR ranges by hand must do it
        // themselves or the binary search returns silently wrong results.
        for (uint32_t u = 0; u < num_nodes; ++u) {
            auto* begin = graph.edge_data.data() + graph.offset[u];
            auto* end   = graph.edge_data.data() + graph.offset[u + 1];
            std::sort(begin, end, [](const Edge& a, const Edge& b){
                return a.departure_time < b.departure_time;
            });
        }
    }

    LookaheadConfig config{.k_departures = 10, .W_max_seconds = 86400, .lambda = 0.0f};
    // NOTE ON lambda=0 IN TESTS 1–5:
    // With lambda=0, the Pareto frontier at every node degenerates to a SINGLE
    // time-optimal label (all secondary_cost=0, only arrival_time matters).
    // In this degenerate case, the single-objective filter:
    //   if (current.arrival_time > best_time[current.node]) continue;
    // produces IDENTICAL output to the correct bi-criteria filter.
    // Tests 1–5 therefore DO NOT discriminate between correct and wrong
    // lazy deletion implementations. They test basic routing correctness only.
    //
    // Test 6 (lambda=1.0f) is the ONLY test that catches the scalar vs. bi-criteria
    // lazy deletion bug. It must pass for the implementation to be correct.
};

// ─────────────────────────────────────────────────────────────────────────
// Test 1: Linear chain — 0 → 1 → 2
// Single path, no ambiguity. Baseline correctness check.
// ─────────────────────────────────────────────────────────────────────────
TEST_F(LazyDeletionTest, LinearChain_CorrectArrivalTimes) {
    build_graph(3, {
        /* node 0 */ {{1, 1000, 300, 0, 0}},   // 0→1: dep=1000, trav=300 → arr=1300
        /* node 1 */ {{2, 1350, 200, 0, 0}},   // 1→2: dep=1350, trav=200 → arr=1550
        /* node 2 */ {}
    });

    ParetoDijkstra router(graph, config);
    auto result = router.run(0, 1000);

    ASSERT_FALSE(result.pareto_sets.empty());

    // Node 1: earliest arrival = 1300
    ASSERT_FALSE(result.pareto_sets[1].empty())
        << "Node 1 must be reachable";
    EXPECT_EQ(result.pareto_sets[1].labels()[0]->arrival_time, 1300u)
        << "Node 1 arrival should be departure(1000) + travel(300) = 1300";

    // Node 2: earliest arrival = 1350 + 200 = 1550
    ASSERT_FALSE(result.pareto_sets[2].empty())
        << "Node 2 must be reachable";
    EXPECT_EQ(result.pareto_sets[2].labels()[0]->arrival_time, 1550u)
        << "Node 2 arrival should be 1550";
}

// ─────────────────────────────────────────────────────────────────────────
// Test 2: Diamond graph — two paths to node 2, only one is better
//
//   0 ─(300s)→ 1 ─(200s)→ 2    [total: 500s]
//   0 ─(100s)→ 3 ─(600s)→ 2    [total: 700s]  ← longer
//
// Without lazy deletion, node 2 may be relaxed twice (via both paths).
// The lazy deletion filter must ensure the second relaxation (via 3)
// is correctly recognized as non-improving and skipped (or produces
// the correct Pareto outcome).
// ─────────────────────────────────────────────────────────────────────────
TEST_F(LazyDeletionTest, DiamondGraph_ShortestPathCorrect) {
    const uint32_t dep = 10000; // departure time for all edges from node 0

    build_graph(4, {
        /* node 0 */ {
            {1, dep,       300, 50, 0},    // 0→1: arr = dep + 300
            {3, dep,       100, 20, 0},    // 0→3: arr = dep + 100
        },
        /* node 1 */ {
            {2, dep + 320, 200, 50, 0},    // 1→2: arr = dep + 320 + 200 = dep + 520
        },
        /* node 2 */ {},
        /* node 3 */ {
            {2, dep + 120, 600, 80, 0},    // 3→2: arr = dep + 120 + 600 = dep + 720
        },
    });

    ParetoDijkstra router(graph, config);
    auto result = router.run(0, dep);

    ASSERT_FALSE(result.pareto_sets[2].empty())
        << "Node 2 must be reachable via both paths";

    // With lambda=0 (time-only): best path is 0→1→2 with arrival = dep + 520
    uint32_t best_arrival = result.pareto_sets[2].labels()[0]->arrival_time;
    EXPECT_EQ(best_arrival, dep + 520)
        << "Best arrival at node 2 should be via 0→1→2 (dep+" << dep + 520
        << "), not via 0→3→2 (dep+" << dep + 720 << "). "
           "If you get dep+720, your lazy deletion is likely missing or incorrect.";
}

// ─────────────────────────────────────────────────────────────────────────
// Test 3: Real stale-label detection
//
// A stale label is one pushed to the heap, then superseded before being
// popped. The lazy deletion filter must detect and skip it.
//
// Graph:
//   0 → 2 direct, arr=1600, crowd=0           ← STALE after path B settles
//   0 → 1 → 2,   arr=1500, crowd=0            ← CORRECT (better time)
//
// Path B (via node 1) departs first from node 0, so node 2 is settled at
// 1500 before the stale direct-path label (arr=1600) is popped.
// Without lazy deletion, 1600 would overwrite or re-relax incorrectly.
// ─────────────────────────────────────────────────────────────────────────
TEST_F(LazyDeletionTest, RealStaleLabelDetection_StalePopSkipped) {
    build_graph(3, {
        /* node 0 */ {
            {1, /*dep=*/1000, /*trav=*/100, /*crowd=*/0, /*pen=*/0}, // → node 1, arr=1100
            {2, /*dep=*/1000, /*trav=*/600, /*crowd=*/0, /*pen=*/0}, // → node 2, arr=1600 (stale)
        },
        /* node 1 */ {
            {2, /*dep=*/1150, /*trav=*/350, /*crowd=*/0, /*pen=*/0}, // → node 2, arr=1500 (best)
        },
        /* node 2 */ {},
    });

    ParetoDijkstra router(graph, config);
    auto result = router.run(0, 1000);

    ASSERT_FALSE(result.pareto_sets[2].empty());
    EXPECT_EQ(result.pareto_sets[2].labels()[0]->arrival_time, 1500u)
        << "Best arrival at node 2 must be 1500 via path 0→1→2. "
           "Getting 1600 means the stale direct-path label was not filtered.";
}

// ─────────────────────────────────────────────────────────────────────────
// Test 4: Unreachable node produces empty Pareto set
// ─────────────────────────────────────────────────────────────────────────
TEST_F(LazyDeletionTest, UnreachableNode_EmptyParetoSet) {
    build_graph(3, {
        /* node 0 */ {{1, 1000, 100, 0, 0}},
        /* node 1 */ {},
        /* node 2 */ {}
    });

    ParetoDijkstra router(graph, config);
    auto result = router.run(0, 1000);

    EXPECT_TRUE(result.pareto_sets[2].empty())
        << "Node 2 is unreachable; its Pareto set must be empty";
    EXPECT_FALSE(result.pareto_sets[1].empty())
        << "Node 1 IS reachable and must have a non-empty Pareto set";
}

// ─────────────────────────────────────────────────────────────────────────
// Test 5: Source node has arrival_time = departure_time, crowd = 0
// ─────────────────────────────────────────────────────────────────────────
TEST_F(LazyDeletionTest, SourceNode_HasZeroCostLabel) {
    build_graph(2, {
        /* node 0 */ {{1, 5000, 200, 100, 0}},
        /* node 1 */ {}
    });

    ParetoDijkstra router(graph, config);
    auto result = router.run(0, 5000);

    ASSERT_FALSE(result.pareto_sets[0].empty())
        << "Source node must always have a label";
    EXPECT_EQ(result.pareto_sets[0].labels()[0]->arrival_time, 5000u)
        << "Source label arrival_time must equal departure_time";
    EXPECT_EQ(result.pareto_sets[0].labels()[0]->secondary_cost, 0u)
        << "Source label secondary_cost must be 0";
}

// ─────────────────────────────────────────────────────────────────────────
// Test 6: Bi-criteria frontier with lambda > 0  (CRITICAL — exposes
//         single-objective lazy deletion bug)
//
// All other tests use lambda=0, where the Pareto frontier degenerates to
// one time-optimal label per node.  With lambda=0 the WRONG check:
//     if (current.arrival_time > best_time[current.node]) continue;
// produces identical results to the correct bi-criteria filter, giving
// false confidence.
//
// This test uses lambda=1.0f and two paths with different (time, crowd)
// tradeoffs. Both labels are Pareto-optimal and must both survive.
//
// Graph (two paths from node 0 to node 2):
//   Path A: 0 → 2 direct.  arr=1300, crowd=100.  Fast but crowded.
//   Path B: 0 → 1 → 2.     arr=1600, crowd=10.   Slow but comfortable.
//
// (1300, 100) and (1600, 10) are incomparable — neither dominates the other.
// Correct frontier: {(1300,100), (1600,10)}.
// Single-objective bug: only {(1300,100)} survives (1600 > best_time[2]).
// ─────────────────────────────────────────────────────────────────────────
TEST_F(LazyDeletionTest, BiCriteriaFrontier_BothLabelsPresent) {
    LookaheadConfig biconfig{.k_departures = 10, .W_max_seconds = 86400, .lambda = 1.0f};

    build_graph(3, {
        /* node 0 */ {
            {2, /*dep=*/1000, /*trav=*/300, /*crowd=*/100, /*pen=*/0}, // arr=1300, crowd=100
            {1, /*dep=*/1000, /*trav=*/200, /*crowd=*/0,   /*pen=*/0}, // arr=1200 at node 1
        },
        /* node 1 */ {
            {2, /*dep=*/1250, /*trav=*/350, /*crowd=*/10,  /*pen=*/0}, // arr=1600, crowd=10
        },
        /* node 2 */ {},
    });

    ParetoDijkstra router(graph, biconfig);
    auto result = router.run(0, 1000);

    ASSERT_FALSE(result.pareto_sets[2].empty());

    ASSERT_EQ(result.pareto_sets[2].size(), 2u)
        << "Pareto frontier at node 2 must have exactly 2 labels: "
           "(arr=1300, crowd=100) and (arr=1600, crowd=10). "
           "size()==1 means lazy deletion is applying the single-objective check "
           "`if (arrival_time > best_time[node]) continue`, which discards the "
           "second label as stale even though it is Pareto-non-dominated. The "
           "filter must test strict dominance on BOTH criteria — see the "
           "lazy-deletion block in ParetoDijkstra::run.";

    EXPECT_EQ(result.pareto_sets[2].labels()[0]->arrival_time, 1300u);
    EXPECT_EQ(result.pareto_sets[2].labels()[0]->secondary_cost,   100u);
    EXPECT_EQ(result.pareto_sets[2].labels()[1]->arrival_time, 1600u);
    EXPECT_EQ(result.pareto_sets[2].labels()[1]->secondary_cost,    10u);
}

