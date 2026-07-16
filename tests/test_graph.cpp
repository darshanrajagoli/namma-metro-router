#include <gtest/gtest.h>
#include "graph.hpp"
#include "gtfs_parser.hpp"
#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @file test_graph.cpp
 * @brief Google Test suite for the CSR graph and GraphBuilder::build().
 *
 * PURPOSE:
 *   Two contracts are verified here.
 *
 *   (1) CSRGraph accessors — edges_of(), out_degree(), memory_bytes(), empty().
 *       The whole routing engine reads the graph exclusively through these, so
 *       the offset/edge_data indexing must be exact.
 *
 *   (2) GraphBuilder::build() — turning interpolated, FK-validated GTFS stop
 *       times into a CSR graph. The build must:
 *         • emit one directed edge per consecutive (stop_i → stop_{i+1}) pair
 *           within a trip,
 *         • sort each node's outgoing edges by departure_time (required for the
 *           binary search in select_optimal_departure),
 *         • inject the Gaussian crowd model,
 *         • drop negative-duration edges and clamp zero-duration edges to 1s,
 *         • fail loudly when a stop_index_map is supplied with num_stops == 0.
 *
 * NOTE on the debug FIFO assertion:
 *   In a debug build, build() throws std::logic_error if the crowd model drops
 *   faster than 1 unit/second between two same-destination edges. The synthetic
 *   Gaussian model never does this (|d/dt(crowd)| ≈ 0.15 ≪ 1), so the builder
 *   tests below — which all use the model's own crowd values — never trip it.
 */

using namespace namma_metro;

// ═══════════════════════════════════════════════════════════════════════════
// Part 1 — CSRGraph accessors (graph constructed directly, no builder)
// ═══════════════════════════════════════════════════════════════════════════

namespace {

// A hand-built 3-node graph:
//   node 0 → {1, 1, 2}   (3 outgoing edges)
//   node 1 → {2}         (1 outgoing edge)
//   node 2 → {}          (no outgoing edges)
CSRGraph make_manual_graph() {
    CSRGraph g;
    g.num_nodes = 3;
    g.offset = {0, 3, 4, 4}; // length num_nodes + 1

    auto push = [&](uint32_t dst, uint32_t dep, uint32_t trav,
                    uint32_t crowd, uint32_t pen) {
        Edge e;
        e.destination    = dst;
        e.departure_time = dep;
        e.travel_time    = trav;
        e.crowd_weight   = crowd;
        e.penalty        = pen;
        g.edge_data.push_back(e);
    };

    // node 0's three edges (already sorted by departure_time)
    push(1, 100, 10, 5, 0);
    push(1, 200, 12, 4, 0);
    push(2, 150, 20, 9, 0);
    // node 1's single edge
    push(2, 300, 8, 1, 0);

    g.num_edges = static_cast<uint32_t>(g.edge_data.size());
    return g;
}

} // namespace

TEST(CSRGraphTest, OutDegree_CountsOutgoingEdges) {
    CSRGraph g = make_manual_graph();
    EXPECT_EQ(g.out_degree(0), 3u);
    EXPECT_EQ(g.out_degree(1), 1u);
    EXPECT_EQ(g.out_degree(2), 0u);
}

TEST(CSRGraphTest, EdgesOf_ReturnsContiguousRange) {
    CSRGraph g = make_manual_graph();

    auto [b0, e0] = g.edges_of(0);
    ASSERT_EQ(e0 - b0, 3);
    EXPECT_EQ(b0[0].destination, 1u);
    EXPECT_EQ(b0[0].departure_time, 100u);
    EXPECT_EQ(b0[2].destination, 2u);

    auto [b1, e1] = g.edges_of(1);
    ASSERT_EQ(e1 - b1, 1);
    EXPECT_EQ(b1[0].destination, 2u);
    EXPECT_EQ(b1[0].departure_time, 300u);

    auto [b2, e2] = g.edges_of(2);
    EXPECT_EQ(e2 - b2, 0) << "Node 2 has no outgoing edges (empty range)";
}

TEST(CSRGraphTest, MemoryBytes_MatchesLayout) {
    CSRGraph g = make_manual_graph();
    // offset has num_nodes+1 = 4 uint32_t entries; edge_data has 4 Edges.
    const size_t expected = sizeof(uint32_t) * g.offset.size()
                          + sizeof(Edge) * g.edge_data.size();
    EXPECT_EQ(g.memory_bytes(), expected);
    EXPECT_EQ(g.memory_bytes(), static_cast<size_t>(4 * 4 + 4 * 20)); // 96 bytes
}

TEST(CSRGraphTest, Empty_TrueOnlyForZeroNodes) {
    CSRGraph empty;
    EXPECT_TRUE(empty.empty());

    CSRGraph g = make_manual_graph();
    EXPECT_FALSE(g.empty());
}

// ═══════════════════════════════════════════════════════════════════════════
// Part 2 — GraphBuilder::build()
// ═══════════════════════════════════════════════════════════════════════════

namespace {

StopTimeRecord stop_time(std::string trip, uint32_t arr, uint32_t dep,
                         std::string stop, uint32_t seq) {
    StopTimeRecord r;
    r.trip_id        = std::move(trip);
    r.arrival_time   = arr;
    r.departure_time = dep;
    r.stop_id        = std::move(stop);
    r.stop_sequence  = seq;
    r.pickup_type    = 0;
    r.drop_off_type  = 0;
    return r;
}

// Replicates synthetic_crowd_weight() from graph_builder.cpp for assertions.
uint32_t expected_crowd(uint32_t departure_time) {
    constexpr double PEAK = 28800.0, SIGMA = 3600.0, BASE = 10.0, AMP = 90.0;
    const double t = static_cast<double>(departure_time);
    const double raw = BASE + AMP * std::exp(-((t - PEAK) * (t - PEAK)) /
                                             (2.0 * SIGMA * SIGMA));
    return static_cast<uint32_t>(raw * 10.0);
}

const std::unordered_map<std::string, uint32_t> kThreeStopIndex = {
    {"S0", 0}, {"S1", 1}, {"S2", 2}};

} // namespace

// One trip with three stops must yield two consecutive edges, S0→S1 and S1→S2.
TEST(GraphBuilderTest, SingleTrip_ProducesConsecutiveEdges) {
    std::vector<StopTimeRecord> st = {
        stop_time("T1", 28800, 28800, "S0", 0),
        stop_time("T1", 29100, 29100, "S1", 1),
        stop_time("T1", 29400, 29400, "S2", 2),
    };
    auto idx = kThreeStopIndex;
    CSRGraph g = GraphBuilder::build(st, 3, &idx);

    EXPECT_EQ(g.num_nodes, 3u);
    EXPECT_EQ(g.num_edges, 2u);
    EXPECT_EQ(g.out_degree(0), 1u);
    EXPECT_EQ(g.out_degree(1), 1u);
    EXPECT_EQ(g.out_degree(2), 0u);

    auto [b0, e0] = g.edges_of(0);
    ASSERT_EQ(e0 - b0, 1);
    EXPECT_EQ(b0[0].destination, 1u);
    EXPECT_EQ(b0[0].departure_time, 28800u);
    EXPECT_EQ(b0[0].travel_time, 300u); // 29100 - 28800

    auto [b1, e1] = g.edges_of(1);
    ASSERT_EQ(e1 - b1, 1);
    EXPECT_EQ(b1[0].destination, 2u);
    EXPECT_EQ(b1[0].travel_time, 300u); // 29400 - 29100
}

// The Gaussian crowd model must be injected (peak value 1000 at 08:00).
TEST(GraphBuilderTest, InjectsGaussianCrowdModel) {
    std::vector<StopTimeRecord> st = {
        stop_time("T1", 28800, 28800, "S0", 0), // departs exactly at 08:00 peak
        stop_time("T1", 29100, 29100, "S1", 1),
    };
    auto idx = kThreeStopIndex;
    CSRGraph g = GraphBuilder::build(st, 3, &idx);

    ASSERT_EQ(g.num_edges, 1u);
    auto [b, e] = g.edges_of(0);
    EXPECT_EQ(b[0].crowd_weight, 1000u)
        << "Crowd at the 08:00 peak must scale to 1000";
    EXPECT_EQ(b[0].crowd_weight, expected_crowd(28800));
    EXPECT_EQ(b[0].penalty, 0u) << "Penalty is 0 throughout the scaffold";
}

// Two trips serving the same leg must be sorted ascending by departure_time.
TEST(GraphBuilderTest, EdgesSortedByDepartureTime) {
    std::vector<StopTimeRecord> st = {
        // Listed later-first to prove the builder sorts.
        stop_time("T2", 30000, 30000, "S0", 0),
        stop_time("T2", 30300, 30300, "S1", 1),
        stop_time("T1", 28800, 28800, "S0", 0),
        stop_time("T1", 29100, 29100, "S1", 1),
    };
    std::unordered_map<std::string, uint32_t> idx = {{"S0", 0}, {"S1", 1}};
    CSRGraph g = GraphBuilder::build(st, 2, &idx);

    ASSERT_EQ(g.out_degree(0), 2u);
    auto [b, e] = g.edges_of(0);
    EXPECT_EQ(b[0].departure_time, 28800u);
    EXPECT_EQ(b[1].departure_time, 30000u)
        << "Outgoing edges of a node must be sorted ascending by departure_time";
}

// arrival < departure (dirty interpolation) → the edge is dropped.
TEST(GraphBuilderTest, NegativeTravelTime_IsDropped) {
    std::vector<StopTimeRecord> st = {
        stop_time("T1", 28800, 28800, "S0", 0),
        stop_time("T1", 28000, 28000, "S1", 1), // arrives BEFORE S0 departs
    };
    std::unordered_map<std::string, uint32_t> idx = {{"S0", 0}, {"S1", 1}};
    // Emits a "[GRAPH WARN] negative travel time" diagnostic to stderr (expected).
    CSRGraph g = GraphBuilder::build(st, 2, &idx);
    EXPECT_EQ(g.num_edges, 0u) << "A negative-duration edge must not be created";
}

// arrival == departure → travel_time clamped to 1s (no zero-weight self-loop).
TEST(GraphBuilderTest, ZeroTravelTime_ClampedToOne) {
    std::vector<StopTimeRecord> st = {
        stop_time("T1", 28800, 28800, "S0", 0),
        stop_time("T1", 28800, 28800, "S1", 1), // identical timestamp
    };
    std::unordered_map<std::string, uint32_t> idx = {{"S0", 0}, {"S1", 1}};
    CSRGraph g = GraphBuilder::build(st, 2, &idx);

    ASSERT_EQ(g.num_edges, 1u);
    auto [b, e] = g.edges_of(0);
    EXPECT_EQ(b[0].travel_time, 1u)
        << "Zero-duration edges must be clamped to 1s to avoid infinite relaxation";
}

// Empty stop-time input yields a node-only graph with no edges.
TEST(GraphBuilderTest, EmptyStopTimes_ReturnsNodeOnlyGraph) {
    std::vector<StopTimeRecord> st;
    CSRGraph g = GraphBuilder::build(st, 5, nullptr);
    EXPECT_EQ(g.num_nodes, 5u);
    EXPECT_EQ(g.num_edges, 0u);
    EXPECT_EQ(g.offset.size(), 6u);
    for (uint32_t n = 0; n < 5; ++n) EXPECT_EQ(g.out_degree(n), 0u);
}

// Supplying a stop_index_map with num_stops == 0 must throw (silent-empty guard).
TEST(GraphBuilderTest, MapWithZeroStops_Throws) {
    std::vector<StopTimeRecord> st = {
        stop_time("T1", 100, 100, "S0", 0),
        stop_time("T1", 200, 200, "S1", 1),
    };
    std::unordered_map<std::string, uint32_t> idx = {{"S0", 0}, {"S1", 1}};
    EXPECT_THROW(GraphBuilder::build(st, 0, &idx), std::invalid_argument)
        << "build() must fail loudly rather than silently return an empty graph";
}

// Fallback local-index path (no map): still produces correct edges.
TEST(GraphBuilderTest, NullIndexMap_UsesFallbackLocalIndex) {
    std::vector<StopTimeRecord> st = {
        stop_time("T1", 28800, 28800, "S0", 0),
        stop_time("T1", 29100, 29100, "S1", 1),
        stop_time("T1", 29400, 29400, "S2", 2),
    };
    // Emits a "[GRAPH WARN] called without stop_index_map" diagnostic (expected).
    CSRGraph g = GraphBuilder::build(st, 3, nullptr);

    EXPECT_EQ(g.num_nodes, 3u);
    EXPECT_EQ(g.num_edges, 2u);
    // Local index is assigned in sorted (trip, seq) order: S0=0, S1=1, S2=2.
    auto [b0, e0] = g.edges_of(0);
    ASSERT_EQ(e0 - b0, 1);
    EXPECT_EQ(b0[0].destination, 1u);
}

// pickup_type == 1 on the source stop suppresses the edge out of that stop.
TEST(GraphBuilderTest, NoPickupStop_SuppressesOutgoingEdge) {
    std::vector<StopTimeRecord> st = {
        stop_time("T1", 28800, 28800, "S0", 0),
        stop_time("T1", 29100, 29100, "S1", 1),
        stop_time("T1", 29400, 29400, "S2", 2),
    };
    st[1].pickup_type = 1; // no boarding at S1 → the S1→S2 edge is skipped
    auto idx = kThreeStopIndex;
    CSRGraph g = GraphBuilder::build(st, 3, &idx);

    EXPECT_EQ(g.num_edges, 1u) << "Only S0→S1 should remain; S1 is no-pickup";
    EXPECT_EQ(g.out_degree(0), 1u);
    EXPECT_EQ(g.out_degree(1), 0u);
}
