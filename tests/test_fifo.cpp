#include <gtest/gtest.h>
#include "routing.hpp"
#include "graph.hpp"
#include <optional>
#include <vector>
#include <algorithm>

/**
 * @file test_fifo.cpp
 * @brief Google Test suite for Bounded-Wait Lookahead departure selection.
 *        Covers the cases where the policy behaves correctly. It does NOT
 *        establish FIFO preservation — see tests/test_fifo_violation.cpp.
 *
 * PURPOSE:
 *   These tests discriminate Bounded-Wait Lookahead from a naive "strict
 *   next-train" policy: a strict-next-train implementation fails them.
 *
 * MATHEMATICAL CONTEXT:
 *   FIFO property: t1 + w(u,v,t1) ≤ t2 + w(u,v,t2) for all t1 ≤ t2.
 *   In off-peak scenarios, morning-peak crowd penalties cause a later departure
 *   to yield a lower composite cost. A strict next-train policy selects a
 *   high-penalty early departure when a slightly-later, low-penalty departure
 *   exists within the W_max window.
 *
 *   Tests 9-11 additionally pin the reachability consequence of the W_max
 *   window — see the Known Limitations table in README.md.
 *
 * SCOPE — WHAT THIS FILE DOES NOT SHOW:
 *   These tests exercise the lookahead where it behaves correctly. They do NOT
 *   establish that FIFO holds in general, and it does not: see
 *   tests/test_fifo_violation.cpp, which constructs three independent mechanisms
 *   that break it and the end-to-end case where dominance pruning consequently
 *   hides a reachable destination. Test 7 below
 *   (FIFOMonotonicity_ArrivalNonDecreasing) passes because its fixture uses a
 *   monotone crowd profile; it is not a general monotonicity proof.
 */

using namespace namma_metro;

// ─────────────────────────────────────────────────────────────────────────
// Test fixture: builds a minimal mock graph for controlled experiments
// ─────────────────────────────────────────────────────────────────────────
class BoundedWaitTest : public ::testing::Test {
protected:
    /**
     * @brief Build a two-node graph (0 → 1) with multiple timed service edges.
     *
     * Node 0 = source station (e.g., Majestic)
     * Node 1 = destination station (e.g., MG Road)
     *
     * @param edges_u0  List of {departure_time, travel_time, secondary_weight, penalty}
     *                  tuples for edges from node 0 to node 1.
     */
    void build_graph(std::vector<std::tuple<uint32_t,uint32_t,uint32_t,uint32_t>> edges_u0) {
        graph.num_nodes = 2;
        graph.offset.resize(3, 0);

        for (auto& [dep, trav, crowd, pen] : edges_u0) {
            Edge e;
            e.destination    = 1;
            e.departure_time = dep;
            e.travel_time    = trav;
            e.secondary_weight   = crowd;
            e.penalty        = pen;
            graph.edge_data.push_back(e);
        }

        graph.offset[0] = 0;
        graph.offset[1] = static_cast<uint32_t>(graph.edge_data.size());
        graph.offset[2] = static_cast<uint32_t>(graph.edge_data.size());
        graph.num_edges = static_cast<uint32_t>(graph.edge_data.size());
    }

    CSRGraph graph;
    LookaheadConfig default_config{
        .k_departures = 5,
        .W_max_seconds = 1800,  // 30 minutes
        .lambda = 1.0f
    };
};

// ─────────────────────────────────────────────────────────────────────────
// Test 1: Trivial — single available departure, must be selected
// ─────────────────────────────────────────────────────────────────────────
TEST_F(BoundedWaitTest, SingleDeparture_MustBeSelected) {
    build_graph({
        {/* dep= */ 28800, /* trav= */ 300, /* crowd= */ 100, /* pen= */ 0}
        // 08:00 departure, 5-min journey, moderate crowd
    });

    auto result = select_optimal_departure(graph, default_config, 28800, 0, 1);

    ASSERT_TRUE(result.has_value())
        << "Should select the only available departure";
    EXPECT_EQ(result->departure_time, 28800u);
    EXPECT_EQ(result->travel_time,    300u);
}

// ─────────────────────────────────────────────────────────────────────────
// Test 2: No departure within W_max → nullopt
// ─────────────────────────────────────────────────────────────────────────
TEST_F(BoundedWaitTest, NoDepartureWithinWindow_ReturnsNullopt) {
    build_graph({
        {/* dep= */ 28800, /* trav= */ 300, /* crowd= */ 100, /* pen= */ 0}
        // Departure at 08:00 but current_time = 10:00 (36000s)
        // 28800 < 36000: already departed, not boardable
    });

    auto result = select_optimal_departure(graph, default_config, 36000, 0, 1);

    EXPECT_FALSE(result.has_value())
        << "Departure before current_time must NOT be selected";
}

// ─────────────────────────────────────────────────────────────────────────
// Test 3: Departure beyond W_max → nullopt
// ─────────────────────────────────────────────────────────────────────────
TEST_F(BoundedWaitTest, DepartureBeyondWmax_ReturnsNullopt) {
    LookaheadConfig tight_config{.k_departures = 5, .W_max_seconds = 600, .lambda = 1.0f};

    build_graph({
        {/* dep= */ 30000, /* trav= */ 300, /* crowd= */ 50, /* pen= */ 0}
        // current_time = 28800 (08:00), departure at 08:20 = 30000
        // wait = 1200s > W_max=600s → must return nullopt
    });

    auto result = select_optimal_departure(graph, tight_config, 28800, 0, 1);

    EXPECT_FALSE(result.has_value())
        << "Departure outside W_max window must NOT be selected";
}

// ─────────────────────────────────────────────────────────────────────────
// Test 4: THE KEY TEST — strict next-train fails, lookahead wins
//
// Scenario: Off-peak morning.
//   Train A departs in 2 min with high crowd (just finished morning rush).
//   Train B departs in 12 min with very low crowd (off-peak).
//
// A naive "strict next-train" policy selects Train A.
// composite(A) = trav_A + crowd_A + pen_A = 300 + 800 + 50 = 1150
// composite(B) = trav_B + crowd_B + pen_B = 280 + 100 + 0  = 380
//
// The Bounded-Wait Lookahead MUST select Train B.
// ─────────────────────────────────────────────────────────────────────────
TEST_F(BoundedWaitTest, OffPeak_WaitingForSecondTrain_IsOptimal) {
    const uint32_t current_time = 32400; // 09:00 (post morning peak)

    build_graph({
        // Train A: departs in 2 min — high crowd (morning residual)
        {current_time + 120, /* trav= */ 300, /* crowd= */ 800, /* pen= */ 50},
        // Train B: departs in 12 min — low crowd (off-peak)
        {current_time + 720, /* trav= */ 280, /* crowd= */ 100, /* pen= */ 0},
    });

    auto result = select_optimal_departure(graph, default_config, current_time, 0, 1);

    ASSERT_TRUE(result.has_value())
        << "Should find at least one valid departure";

    // composite(A) = 300 + 800 + 50 = 1150
    // composite(B) = 280 + 100 +  0 =  380
    // Lookahead must select B (departure at current_time + 720)
    EXPECT_EQ(result->departure_time, current_time + 720)
        << "Lookahead should select Train B (lower composite cost). "
           "If your implementation selects Train A, you have implemented "
           "strict next-train (wrong). Implement the Bounded-Wait Lookahead.";

    EXPECT_EQ(result->secondary_weight, 100u)
        << "Selected edge should have secondary_weight=100 (Train B)";
}

// ─────────────────────────────────────────────────────────────────────────
// Test 5: Lambda=0 (time-only) → minimize ARRIVAL TIME, not travel time alone
//
// With lambda=0: composite = departure_time + travel_time + penalty
//   (crowd term drops out; the tie-breaker must be ARRIVAL time, not travel time)
//
// Train A: dep=current+120, travel=300, pen=50
//   arrival      = current+420
//   composite(A) = (current+120+300) + 50 = current+470
// Train B: dep=current+720, travel=280, pen=0
//   arrival      = current+1000
//   composite(B) = (current+720+280) + 0  = current+1000
//
// composite(A) < composite(B) → Train A must be selected.
//
// The distinction matters: minimizing travel_time alone would pick Train B,
// which arrives 530s later. With lambda=0 the objective is arrival time.
// ─────────────────────────────────────────────────────────────────────────
TEST_F(BoundedWaitTest, LambdaZero_MinimizesArrivalTime_NotTravelTimeAlone) {
    LookaheadConfig time_only{.k_departures = 5, .W_max_seconds = 1800, .lambda = 0.0f};
    const uint32_t current_time = 32400;

    build_graph({
        {current_time + 120, /* trav= */ 300, /* crowd= */ 800, /* pen= */ 50},
        {current_time + 720, /* trav= */ 280, /* crowd= */ 100, /* pen= */ 0},
    });

    auto result = select_optimal_departure(graph, time_only, current_time, 0, 1);

    ASSERT_TRUE(result.has_value());
    // composite(A) = dep_A + trav_A + pen_A = (current+420) + 50 = current+470
    // composite(B) = dep_B + trav_B + pen_B = (current+1000) + 0 = current+1000
    // A has strictly lower composite → A is selected.
    EXPECT_EQ(result->departure_time, current_time + 120)
        << "With lambda=0, composite = departure_time + travel_time + penalty. "
           "composite(A)=current+470 < composite(B)=current+1000, so Train A wins. "
           "Selecting Train B means travel_time alone is being minimized rather "
           "than arrival time.";
}

// ─────────────────────────────────────────────────────────────────────────
// Test 6: k_departures limit respected
//
// Provide 10 departures within W_max, but k=3.
// Algorithm should only examine the first 3 chronological departures.
// ─────────────────────────────────────────────────────────────────────────
TEST_F(BoundedWaitTest, KDeparturesLimitRespected) {
    LookaheadConfig k3_config{.k_departures = 3, .W_max_seconds = 7200, .lambda = 1.0f};
    const uint32_t current_time = 28800;

    // 10 services, each 10 minutes apart. The 5th has extremely low cost.
    // With k=3, it should NOT be visible; best among first 3 is selected.
    build_graph({
        {current_time + 100,  300, 500, 20},  // index 0 (earliest)
        {current_time + 200,  310, 490, 18},  // index 1
        {current_time + 300,  290, 480, 15},  // index 2 — best of first 3
        {current_time + 400,  280,  50,  0},  // index 3 — great, but k=3 can't see it
        {current_time + 500,  270,  10,  0},  // index 4 — even better, still invisible
    });

    auto result = select_optimal_departure(graph, k3_config, current_time, 0, 1);

    ASSERT_TRUE(result.has_value());
    // composite = travel_time + lambda*secondary_weight + penalty, lambda = 1:
    //   index 0: 300 + 500 + 20 = 820
    //   index 1: 310 + 490 + 18 = 818
    //   index 2: 290 + 480 + 15 = 785  ← minimum among the first k=3
    EXPECT_EQ(result->departure_time, current_time + 300u)
        << "With k=3 only the first three departures are evaluated; index 2 has the "
           "lowest composite (785) among them. Departures at +400 and +500 have lower "
           "composite still, but must be invisible.";
    EXPECT_NE(result->departure_time, current_time + 400u)
        << "4th departure must be invisible when k=3";
}

// ─────────────────────────────────────────────────────────────────────────
// Test 7: FIFO monotonicity check
//
// Verify empirically that the composite arrival function is non-decreasing
// (FIFO preserved) for a sequence of departure times.
//
// For each consecutive pair (t1, t2) with t1 < t2:
//   arrival(t1) = dep_time(t1) + travel_time(t1)  (using selected departure)
//   arrival(t2) = dep_time(t2) + travel_time(t2)
//   FIFO requires: arrival(t1) ≤ arrival(t2)
// ─────────────────────────────────────────────────────────────────────────
TEST_F(BoundedWaitTest, FIFOMonotonicity_ArrivalNonDecreasing) {
    build_graph({
        {28800,  300, 200, 10},  // 08:00
        {29100,  290, 150,  5},  // 08:05
        {29400,  280,  80,  2},  // 08:10
        {29700,  270,  50,  0},  // 08:15
        {30000,  260,  30,  0},  // 08:20
    });

    std::vector<uint32_t> query_times = {
        28700, 28800, 28900, 29000, 29100, 29200, 29400, 29600
    };

    std::optional<uint32_t> prev_arrival;

    for (uint32_t t : query_times) {
        auto result = select_optimal_departure(graph, default_config, t, 0, 1);
        if (!result.has_value()) continue;

        uint32_t arrival = result->departure_time + result->travel_time;

        if (prev_arrival.has_value()) {
            EXPECT_LE(*prev_arrival, arrival)
                << "FIFO violated! Departure at later time yielded earlier arrival. "
                << "prev_arrival=" << *prev_arrival << " arrival=" << arrival
                << " at query_time=" << t;
        }
        prev_arrival = arrival;
    }
}

// ─────────────────────────────────────────────────────────────────────────
// Test 8: Edge case — current_time exactly equals departure_time (boardable)
// ─────────────────────────────────────────────────────────────────────────
TEST_F(BoundedWaitTest, ExactlyAtDeparture_IsBoardable) {
    build_graph({
        {28800, 300, 100, 0}
    });

    // Arrive at station exactly at 28800 — should still be able to board.
    auto result = select_optimal_departure(graph, default_config, 28800, 0, 1);

    ASSERT_TRUE(result.has_value())
        << "departure_time == current_time must be boardable (≥ is the condition, not >)";
    EXPECT_EQ(result->departure_time, 28800u);
}

// ─────────────────────────────────────────────────────────────────────────
// Tests 9–11: the W_max reachability cliff, end to end.
//
// The bounded-wait window is a search-space bound, not a routing preference:
// when no service to a neighbour departs within W_max_seconds,
// select_optimal_departure returns nullopt and the Dijkstra loop simply never
// relaxes that link. Downstream stations then report as UNREACHABLE even though
// a passenger could reach them by waiting longer.
//
// On the metro feeds this engine targets (≤30 min headway) the window never
// binds. On a sparse or late-night feed it does. These tests pin that boundary
// so it is a documented, tested property rather than a surprise — see the
// "Known Limitations" table in README.md.
// ─────────────────────────────────────────────────────────────────────────
class ReachabilityWindowTest : public ::testing::Test {
protected:
    /**
     * @brief Three-station chain 0 → 1 → 2 with a controllable gap at node 1.
     *
     * Node 0 departs at 08:00 and reaches node 1 at 08:05. The onward service
     * 1 → 2 departs @p gap_seconds after that arrival, so @p gap_seconds alone
     * decides whether node 2 falls inside the bounded-wait window.
     */
    void build_chain(uint32_t gap_seconds) {
        const uint32_t dep0     = 28800;              // 08:00 leg 0 → 1
        const uint32_t arrive1  = dep0 + 300;         // 08:05 at node 1
        const uint32_t dep1     = arrive1 + gap_seconds;

        graph = CSRGraph{};
        graph.num_nodes = 3;

        Edge leg0;
        leg0.destination    = 1;
        leg0.departure_time = dep0;
        leg0.travel_time    = 300;
        leg0.secondary_weight   = 100;
        leg0.penalty        = 0;

        Edge leg1;
        leg1.destination    = 2;
        leg1.departure_time = dep1;
        leg1.travel_time    = 300;
        leg1.secondary_weight   = 100;
        leg1.penalty        = 0;

        graph.edge_data = {leg0, leg1};
        // CSR: node 0 owns edge [0,1), node 1 owns edge [1,2), node 2 has none.
        graph.offset    = {0, 1, 2, 2};
        graph.num_edges = 2;
    }

    /// True when @p node has at least one Pareto label, i.e. it was reached.
    static bool reached(const QueryResult& r, uint32_t node) {
        return !r.pareto_sets[node].labels().empty();
    }

    CSRGraph graph;
};

// Test 9: the cliff itself — a station reachable in the graph is reported
// unreachable because the onward service lies beyond the default window.
TEST_F(ReachabilityWindowTest, SparseFeed_StationBeyondWindowIsReportedUnreachable) {
    build_chain(/* gap_seconds= */ 3600);  // next train in 1 h, window is 30 min

    LookaheadConfig default_window{.k_departures = 5, .W_max_seconds = 1800, .lambda = 1.0f};
    ParetoDijkstra router(graph, default_window);
    auto result = router.run(0, 28800);

    EXPECT_TRUE(reached(result, 1))
        << "Node 1 is one leg away and well inside the window — it must be reached";
    EXPECT_FALSE(reached(result, 2))
        << "Documented limitation: the onward service departs 3600s after arrival at "
           "node 1, beyond W_max_seconds=1800, so the link is never relaxed and node 2 "
           "reports unreachable. This is a search-space bound, not a claim that no "
           "journey exists.";
}

// Test 10: the documented mitigation — widening the window restores reachability.
TEST_F(ReachabilityWindowTest, WideningWindowRestoresReachability) {
    build_chain(/* gap_seconds= */ 3600);

    LookaheadConfig wide_window{.k_departures = 5, .W_max_seconds = 7200, .lambda = 1.0f};
    ParetoDijkstra router(graph, wide_window);
    auto result = router.run(0, 28800);

    ASSERT_TRUE(reached(result, 2))
        << "With W_max_seconds=7200 the 3600s wait is inside the window, so node 2 "
           "must be reached. If this fails the mitigation documented in README.md's "
           "Known Limitations table no longer works.";

    // Arrival is 08:00 + 300s ride + 3600s wait + 300s ride = 12600s (11:30).
    const auto& labels = result.pareto_sets[2].labels();
    ASSERT_EQ(labels.size(), 1u) << "One route exists, so the frontier holds one label";
    EXPECT_EQ(labels.front()->arrival_time, 28800u + 300u + 3600u + 300u)
        << "Waiting is modelled as arrival at the onward departure plus its ride time";
}

// Test 11: the window does not bind on a dense feed — the regime this engine
// actually targets. Both configurations must agree.
TEST_F(ReachabilityWindowTest, DenseFeed_WindowDoesNotBind) {
    build_chain(/* gap_seconds= */ 300);  // 5-min headway, as on Namma Metro

    LookaheadConfig default_window{.k_departures = 5, .W_max_seconds = 1800, .lambda = 1.0f};
    LookaheadConfig wide_window{.k_departures = 5, .W_max_seconds = 7200, .lambda = 1.0f};

    ParetoDijkstra tight_router(graph, default_window);
    ParetoDijkstra wide_router(graph, wide_window);

    auto tight = tight_router.run(0, 28800);
    auto wide  = wide_router.run(0, 28800);

    ASSERT_TRUE(reached(tight, 2)) << "A 300s wait is well inside the default window";
    ASSERT_TRUE(reached(wide, 2));

    ASSERT_EQ(tight.pareto_sets[2].labels().size(), wide.pareto_sets[2].labels().size());
    EXPECT_EQ(tight.pareto_sets[2].labels().front()->arrival_time,
              wide.pareto_sets[2].labels().front()->arrival_time)
        << "At metro headways the bounded-wait window must have no observable effect";
}
