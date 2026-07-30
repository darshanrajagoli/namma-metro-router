#include <gtest/gtest.h>
#include "graph.hpp"
#include "gtfs_parser.hpp"
#include "routing.hpp"
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @file test_transfers.cpp
 * @brief Contracts for the transfer layer and the TransferCount second objective.
 *
 * WHY A TRANSFER LAYER EXISTS
 *   Without it, a feed whose platforms are separate nodes offers no way to change
 *   line at all (the graph is disconnected between platforms), and a feed whose
 *   platforms are collapsed onto one station node makes changing line instant and
 *   free (journey times are understated). Transfer edges are the walk between two
 *   platforms of one station.
 *
 * WHY IT IS A SEPARATE CSR ARRAY
 *   Service edges are sorted by departure_time and located with std::lower_bound.
 *   A transfer has no departure time — it is available the moment the passenger
 *   arrives — so putting it in that array needs a sentinel time, and any sentinel
 *   collides with a real departure (0 is a legitimate 00:00:00). A separate array
 *   removes the ambiguity and leaves the service-edge search untouched. The tests
 *   below pin both halves of that claim.
 *
 * WHAT TransferCount CHANGES
 *   Under SecondObjective::TransferCount every service edge carries
 *   secondary_weight = 0 and each transfer contributes exactly 1, so a label's
 *   accumulated secondary_cost is literally "how many times did this passenger
 *   change platform". The Pareto frontier then trades arrival time against
 *   number of changes — the classic transit-routing bi-criteria problem.
 */

using namespace namma_metro;

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

TransferRecord transfer(std::string from, std::string to, uint32_t secs) {
    TransferRecord t;
    t.from_stop_id     = std::move(from);
    t.to_stop_id       = std::move(to);
    t.min_transfer_time = secs;
    return t;
}

// Four platforms: P0/P1 belong to station A, P2/P3 to station B.
const std::unordered_map<std::string, uint32_t> kIdx = {
    {"P0", 0}, {"P1", 1}, {"P2", 2}, {"P3", 3}};

// One trip on each of two lines, so both platforms carry service.
std::vector<StopTimeRecord> two_line_stop_times() {
    return {
        // Line 1: P0 -> P2, departs 08:00:00 (28800), 300s ride.
        stop_time("L1", 28800, 28800, "P0", 0),
        stop_time("L1", 29100, 29100, "P2", 1),
        // Line 2: P1 -> P3, departs 08:10:00 (29400), 300s ride.
        stop_time("L2", 29400, 29400, "P1", 0),
        stop_time("L2", 29700, 29700, "P3", 1),
    };
}

} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Part 1 — Graph construction
// ═══════════════════════════════════════════════════════════════════════════

// A graph built without transfers must be indistinguishable from the old one:
// empty transfer layer, empty ranges, crowd objective.
TEST(TransferLayer, AbsentByDefault_GraphIsUnchanged) {
    auto st = two_line_stop_times();
    auto idx = kIdx;
    CSRGraph g = GraphBuilder::build(st, 4, &idx);

    EXPECT_EQ(g.num_transfers, 0u);
    EXPECT_FALSE(g.has_transfers());
    EXPECT_EQ(g.second_objective, SecondObjective::CrowdExposure);

    // transfers_of() must be safe to call and yield an empty range on every node,
    // so callers need no has_transfers() guard around the relaxation loop.
    for (uint32_t u = 0; u < g.num_nodes; ++u) {
        auto [b, e] = g.transfers_of(u);
        EXPECT_EQ(b, e) << "node " << u << " must expose an empty transfer range";
    }
}

TEST(TransferLayer, BuildsCorrectCSRAdjacency) {
    auto st = two_line_stop_times();
    auto idx = kIdx;
    std::vector<TransferRecord> tr = {
        transfer("P0", "P1", 120),  // station A, both directions
        transfer("P1", "P0", 120),
        transfer("P2", "P3", 90),   // station B, both directions
        transfer("P3", "P2", 90),
    };

    CSRGraph g = GraphBuilder::build_with_transfers(
        st, 4, &idx, tr, SecondObjective::TransferCount);

    ASSERT_EQ(g.num_transfers, 4u);
    ASSERT_TRUE(g.has_transfers());
    ASSERT_EQ(g.transfer_offset.size(), g.num_nodes + 1u);
    EXPECT_EQ(g.transfer_offset[g.num_nodes], g.num_transfers)
        << "CSR row pointer must terminate at the total transfer count";

    // Each platform has exactly one transfer, to its twin.
    const uint32_t twin[4] = {1, 0, 3, 2};
    const uint32_t secs[4] = {120, 120, 90, 90};
    for (uint32_t u = 0; u < 4; ++u) {
        auto [b, e] = g.transfers_of(u);
        ASSERT_EQ(e - b, 1) << "platform " << u << " should have one transfer";
        EXPECT_EQ(b->destination, twin[u]);
        EXPECT_EQ(b->travel_time, secs[u]);
    }
}

// A transfer to the platform you are already on is a zero-length self-edge that
// the router would re-expand on every visit for no benefit.
TEST(TransferLayer, SelfTransfersAreDropped) {
    auto st = two_line_stop_times();
    auto idx = kIdx;
    std::vector<TransferRecord> tr = {
        transfer("P0", "P0", 60),   // dropped
        transfer("P0", "P1", 120),  // kept
    };

    CSRGraph g = GraphBuilder::build_with_transfers(
        st, 4, &idx, tr, SecondObjective::TransferCount);

    EXPECT_EQ(g.num_transfers, 1u);
    auto [b, e] = g.transfers_of(0);
    ASSERT_EQ(e - b, 1);
    EXPECT_EQ(b->destination, 1u);
}

// An unknown stop_id must be skipped, not resolved to a garbage node index and
// not fatal — the rest of the feed is still perfectly routable.
TEST(TransferLayer, UnresolvedStopIdsAreSkippedNotFatal) {
    auto st = two_line_stop_times();
    auto idx = kIdx;
    std::vector<TransferRecord> tr = {
        transfer("P0", "GHOST", 120),   // to-side unknown
        transfer("GHOST", "P1", 120),   // from-side unknown
        transfer("P2", "P3", 90),       // valid
    };

    CSRGraph g = GraphBuilder::build_with_transfers(
        st, 4, &idx, tr, SecondObjective::TransferCount);

    EXPECT_EQ(g.num_transfers, 1u);
    auto [b, e] = g.transfers_of(2);
    ASSERT_EQ(e - b, 1);
    EXPECT_EQ(b->destination, 3u);
}

// GTFS uses min_transfer_time = 0 for a same-platform change, and transfers are
// emitted in both directions — a genuine 0 would be a zero-weight cycle.
TEST(TransferLayer, ZeroTransferTimeIsClampedToOneSecond) {
    auto st = two_line_stop_times();
    auto idx = kIdx;
    std::vector<TransferRecord> tr = {
        transfer("P0", "P1", 0),
        transfer("P1", "P0", 0),
    };

    CSRGraph g = GraphBuilder::build_with_transfers(
        st, 4, &idx, tr, SecondObjective::TransferCount);

    ASSERT_EQ(g.num_transfers, 2u);
    for (uint32_t u = 0; u < 2; ++u) {
        auto [b, e] = g.transfers_of(u);
        ASSERT_EQ(e - b, 1);
        EXPECT_EQ(b->travel_time, 1u) << "zero-second transfers must clamp to 1s";
    }
}

// Transfer records name stops by id. Resolving them against a locally-rebuilt
// index that disagrees with the parser's would wire transfers between the wrong
// platforms — silently, since every index is a valid node number.
TEST(TransferLayer, TransfersWithoutStopIndexMapThrow) {
    auto st = two_line_stop_times();
    std::vector<TransferRecord> tr = {transfer("P0", "P1", 120)};

    EXPECT_THROW(
        GraphBuilder::build_with_transfers(st, 4, nullptr, tr,
                                           SecondObjective::TransferCount),
        std::invalid_argument);
}

// Under TransferCount, riding a train must be free in the second objective, so
// that accumulated secondary_cost counts changes and nothing else.
TEST(TransferLayer, TransferCountZeroesServiceEdgeSecondaryWeight) {
    auto st = two_line_stop_times();
    auto idx = kIdx;
    std::vector<TransferRecord> tr = {transfer("P0", "P1", 120)};

    CSRGraph transfer_mode = GraphBuilder::build_with_transfers(
        st, 4, &idx, tr, SecondObjective::TransferCount);
    for (const Edge& e : transfer_mode.edge_data) {
        EXPECT_EQ(e.secondary_weight, 0u)
            << "service edges must not contribute to a transfer count";
    }

    // The crowd objective on the same data must still carry crowd weights,
    // confirming the mode flag is what changed the outcome.
    CSRGraph crowd_mode = GraphBuilder::build_with_transfers(
        st, 4, &idx, tr, SecondObjective::CrowdExposure);
    ASSERT_FALSE(crowd_mode.edge_data.empty());
    bool any_nonzero = false;
    for (const Edge& e : crowd_mode.edge_data)
        if (e.secondary_weight > 0) any_nonzero = true;
    EXPECT_TRUE(any_nonzero) << "crowd mode must still populate secondary_weight";
}

// The service-edge array must be completely untouched by the transfer layer:
// same count, same per-node departure_time ordering that lower_bound depends on.
TEST(TransferLayer, ServiceEdgeArrayAndOrderingAreUnaffected) {
    auto st = two_line_stop_times();
    auto idx = kIdx;
    std::vector<TransferRecord> tr = {
        transfer("P0", "P1", 120), transfer("P1", "P0", 120),
    };

    CSRGraph without = GraphBuilder::build(st, 4, &idx);
    CSRGraph with    = GraphBuilder::build_with_transfers(
        st, 4, &idx, tr, SecondObjective::CrowdExposure);

    ASSERT_EQ(without.num_edges, with.num_edges);
    ASSERT_EQ(without.offset, with.offset);
    for (uint32_t i = 0; i < with.num_edges; ++i) {
        EXPECT_EQ(with.edge_data[i].destination,    without.edge_data[i].destination);
        EXPECT_EQ(with.edge_data[i].departure_time, without.edge_data[i].departure_time);
    }
    for (uint32_t u = 0; u < with.num_nodes; ++u) {
        auto [b, e] = with.edges_of(u);
        for (const Edge* p = b; p != e && p + 1 != e; ++p)
            EXPECT_LE(p->departure_time, (p + 1)->departure_time)
                << "service edges must stay sorted by departure_time at node " << u;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Part 2 — Routing across transfers
// ═══════════════════════════════════════════════════════════════════════════

// The whole point: a passenger arriving at P0 can walk to P1 and catch line 2.
// Without the transfer layer P3 is unreachable from P0 entirely.
TEST(TransferRouting, TransferMakesOtherwiseUnreachablePlatformReachable) {
    auto st = two_line_stop_times();
    auto idx = kIdx;
    LookaheadConfig cfg{.k_departures = 5, .W_max_seconds = 1800, .lambda = 1.0f};

    // Without transfers: P0 reaches only P2 (its own line).
    CSRGraph no_tr = GraphBuilder::build(st, 4, &idx);
    ParetoDijkstra r1(no_tr, cfg);
    auto res1 = r1.run(0, 28800);
    EXPECT_FALSE(res1.pareto_sets[2].empty()) << "own-line destination must be reached";
    EXPECT_TRUE(res1.pareto_sets[3].empty())
        << "without a transfer layer there is no way onto line 2";

    // With transfers: P0 -> (walk 120s) -> P1 -> line 2 -> P3.
    std::vector<TransferRecord> tr = {transfer("P0", "P1", 120)};
    CSRGraph with_tr = GraphBuilder::build_with_transfers(
        st, 4, &idx, tr, SecondObjective::TransferCount);
    ParetoDijkstra r2(with_tr, cfg);
    auto res2 = r2.run(0, 28800);

    ASSERT_FALSE(res2.pareto_sets[3].empty())
        << "P3 must be reachable via the transfer at station A";

    // Walk at 28800 arrives P1 at 28920; line 2 departs 29400, arrives P3 at 29700.
    const Label* l = res2.pareto_sets[3].labels().front();
    EXPECT_EQ(l->arrival_time, 29700u);
    EXPECT_EQ(l->secondary_cost, 1u) << "exactly one platform change was made";
}

// A transfer must cost real time, not be instantaneous.
TEST(TransferRouting, TransferCostsItsMinimumTime) {
    auto st = two_line_stop_times();
    auto idx = kIdx;
    std::vector<TransferRecord> tr = {transfer("P0", "P1", 450)};

    CSRGraph g = GraphBuilder::build_with_transfers(
        st, 4, &idx, tr, SecondObjective::TransferCount);
    LookaheadConfig cfg{.k_departures = 5, .W_max_seconds = 1800, .lambda = 1.0f};
    ParetoDijkstra r(g, cfg);
    auto res = r.run(0, 28800);

    ASSERT_FALSE(res.pareto_sets[1].empty());
    EXPECT_EQ(res.pareto_sets[1].labels().front()->arrival_time, 28800u + 450u)
        << "arriving at the twin platform must cost min_transfer_time";
}

// The headline contract: arrival time traded against number of changes.
//
//   P0 --(direct, slow: departs 28800, 900s)--> P3      arrive 29700, 0 changes
//   P0 --(walk 120s)--> P1 --(fast: departs 29000, 200s)--> P3
//                                                      arrive 29200, 1 change
//
// Neither dominates: the transfer route is 500s earlier but costs a change.
// Both must survive on the frontier.
TEST(TransferRouting, FrontierTradesArrivalTimeAgainstTransferCount) {
    std::vector<StopTimeRecord> st = {
        // Slow direct service P0 -> P3.
        stop_time("SLOW", 28800, 28800, "P0", 0),
        stop_time("SLOW", 29700, 29700, "P3", 1),
        // Fast service from the twin platform P1 -> P3.
        stop_time("FAST", 29000, 29000, "P1", 0),
        stop_time("FAST", 29200, 29200, "P3", 1),
    };
    auto idx = kIdx;
    std::vector<TransferRecord> tr = {transfer("P0", "P1", 120)};

    CSRGraph g = GraphBuilder::build_with_transfers(
        st, 4, &idx, tr, SecondObjective::TransferCount);
    LookaheadConfig cfg{.k_departures = 5, .W_max_seconds = 1800, .lambda = 1.0f};
    ParetoDijkstra r(g, cfg);
    auto res = r.run(0, 28800);

    const auto& labels = res.pareto_sets[3].labels();
    ASSERT_EQ(labels.size(), 2u)
        << "Both routes are Pareto-optimal: the direct one arrives later with 0 "
           "changes, the transfer one arrives earlier with 1. A frontier of size 1 "
           "means one of them was wrongly dominated.";

    // labels_ is sorted ascending by arrival_time with secondary strictly decreasing.
    EXPECT_EQ(labels[0]->arrival_time,   29200u);
    EXPECT_EQ(labels[0]->secondary_cost, 1u) << "the earlier arrival costs a change";
    EXPECT_EQ(labels[1]->arrival_time,   29700u);
    EXPECT_EQ(labels[1]->secondary_cost, 0u) << "the later arrival is change-free";
}

// Under TransferCount every service edge has secondary_weight 0, so the crowd
// term cannot discriminate between departures on a link. Departure selection
// must therefore minimise ARRIVAL. Minimising ride duration instead picks a
// later train that arrives strictly later while saving no transfers — worse on
// both objectives at once.
TEST(TransferRouting, DepartureSelectionMinimisesArrivalUnderTransferCount) {
    std::vector<StopTimeRecord> st = {
        // Early, slower train: departs 28800, 600s ride -> arrives 29400.
        stop_time("EARLY", 28800, 28800, "P0", 0),
        stop_time("EARLY", 29400, 29400, "P2", 1),
        // Later, faster train: departs 29300, 200s ride -> arrives 29500.
        stop_time("LATER", 29300, 29300, "P0", 0),
        stop_time("LATER", 29500, 29500, "P2", 1),
    };
    auto idx = kIdx;
    std::vector<TransferRecord> tr = {transfer("P0", "P1", 120)};

    CSRGraph g = GraphBuilder::build_with_transfers(
        st, 4, &idx, tr, SecondObjective::TransferCount);
    // lambda > 0 deliberately: this is the regime that used to pick on duration.
    LookaheadConfig cfg{.k_departures = 5, .W_max_seconds = 1800, .lambda = 1.0f};
    ParetoDijkstra r(g, cfg);
    auto res = r.run(0, 28800);

    ASSERT_FALSE(res.pareto_sets[2].empty());
    EXPECT_EQ(res.pareto_sets[2].labels().front()->arrival_time, 29400u)
        << "must board the EARLY train (arrives 29400), not the LATER one that has "
           "the shorter ride but arrives 29500.";
}
