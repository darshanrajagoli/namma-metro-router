#include <gtest/gtest.h>
#include "routing.hpp"
#include "graph.hpp"
#include <optional>
#include <vector>
#include <algorithm>

/**
 * @file test_fifo.cpp
 * @brief Google Test suite validating FIFO preservation under Bounded-Wait Lookahead.
 *
 * PURPOSE:
 *   These tests are specifically engineered to FAIL if you implement a naive
 *   "strict next-train" policy. You must implement the Bounded-Wait Lookahead
 *   in select_optimal_departure() to make them pass.
 *
 * MATHEMATICAL CONTEXT:
 *   FIFO property: t1 + w(u,v,t1) ≤ t2 + w(u,v,t2) for all t1 ≤ t2.
 *   In off-peak scenarios, morning-peak crowd penalties cause a later departure
 *   to yield a lower composite cost. A strict next-train policy selects a
 *   high-penalty early departure when a slightly-later, low-penalty departure
 *   exists within the W_max window.
 *
 * You must implement select_optimal_departure() to pass ALL these tests.
 * You will be asked to derive the FIFO constraint proof at interview.
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
     * @param edges_u0  List of {departure_time, travel_time, crowd_weight, penalty}
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
            e.crowd_weight   = crowd;
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

    EXPECT_EQ(result->crowd_weight, 100u)
        << "Selected edge should have crowd_weight=100 (Train B)";
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
// ⚠ Previous version of this test was WRONG: it expected departure current+720
//   (Train B) because the comment miscalculated composite(A) as 300 (dropping
//   pen=50) and concluded B wins on travel_time alone. That validated the FM6
//   bug — minimizing travel_time instead of arrival_time when lambda=0.
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
           "composite(A)=current+470 < composite(B)=current+1000. Train A must win. "
           "If Train B is selected: your implementation minimizes travel_time alone "
           "(the FM6 bug). Fix: composite = dep_time + travel_time + lambda*crowd + penalty.";
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
    // Among first 3: composite = [820, 818, 785]. Index 2 wins.
    EXPECT_EQ(result->departure_time, current_time + 300u)
        << "With k=3, only first 3 departures are evaluated. "
           "Index 2 (dep+300) has composite cost 285+15=800... adjust per your formula. "
           "Key: departure at +400 or +500 must NOT be selected.";
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
