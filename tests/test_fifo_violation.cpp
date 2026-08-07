#include <gtest/gtest.h>
#include "routing.hpp"
#include "graph.hpp"
#include <algorithm>
#include <optional>
#include <vector>

/**
 * @file test_fifo_violation.cpp
 * @brief Pins the boundary where this engine's FIFO guarantee STOPS holding.
 *
 * WHY THIS FILE EXISTS
 *   tests/test_fifo.cpp shows the Bounded-Wait Lookahead behaving correctly, and
 *   tests/test_fifo_invariant.cpp checks the crowd model's derivative bound. Neither
 *   tests the mechanism that actually breaks FIFO here, so the limitation lived only
 *   in prose. These tests are the counterpart: they construct FIFO violations and
 *   assert the CURRENT behaviour, so the boundary is a pinned, discoverable property
 *   rather than a claim in a README.
 *
 *   Same treatment as the W_max reachability cliff in test_fifo.cpp (tests 9-11):
 *   these tests PASS. A failure means the behaviour changed — read the message.
 *
 * THE FIFO PROPERTY
 *   For all t1 <= t2:  t1 + w(u,v,t1) <= t2 + w(u,v,t2)
 *   In words: showing up at the platform LATER must never let you arrive EARLIER.
 *   Dijkstra's optimal-substructure argument depends on it.
 *
 * THREE INDEPENDENT MECHANISMS BREAK IT, and they are tested separately below.
 *
 *   (1) THE COMPOSITE IGNORES ARRIVAL TIME.  Under CrowdExposure with lambda > 0,
 *       select_optimal_departure minimises  travel_time + lambda*crowd + penalty.
 *       That expression contains no departure_time, so the choice is blind to when
 *       the passenger actually arrives. With per-link travel times that differ
 *       between departures, a later query can select a different, faster train and
 *       arrive earlier. (Test 1.)
 *
 *   (2) THE k BUDGET TRUNCATES.  Only the first k_departures candidates to v are
 *       examined. As current_time advances the window slides, so a departure that
 *       was beyond the k-th candidate can become visible — and it may arrive
 *       earlier than anything in the old candidate set. (Test 2.)
 *
 *   (3) THE W_max WINDOW TRUNCATES.  Same argument for the [t, t + W_max] bound:
 *       a departure just outside the window at t1 can fall inside it at t2 > t1.
 *       (Test 3.)
 *
 *   Mechanisms (2) and (3) fire even in the ARRIVAL-MINIMISING regime (lambda == 0,
 *   or SecondObjective::TransferCount), where the composite IS the arrival time.
 *   So switching to arrival-minimising selection does not by itself restore FIFO.
 *   This is worth knowing: it means the obvious "fix" is incomplete.
 *
 * WHY IT DOES NOT BITE ON THE MEASURED FEEDS
 *   Test 4 is the control. When per-link travel time is CONSTANT across departures
 *   — which is exactly how scripts/build_namma_metro_gtfs.py derives it, from
 *   distance / average speed — arrival is monotone in the chosen departure, the
 *   chosen departure is non-decreasing in query time, and FIFO holds. The BART
 *   configuration avoids mechanism (1) separately, by running under TransferCount.
 *
 * THE CONSEQUENCE, END TO END
 *   Test 5 is the one that matters. FIFO underpins "consistency under extension":
 *   if label L1 dominates L2 at node u, then extending both must preserve that.
 *   docs/write-up.tex asserts this "holds trivially for non-negative edge weights",
 *   but that argument silently assumes both labels are extended by the SAME edge.
 *   They are not — select_optimal_departure picks a different edge per arrival time.
 *   Test 5 exhibits a graph where a DOMINATED label at an intermediate node was the
 *   only one that could reach the destination, so pruning it renders a genuinely
 *   reachable node unreachable.
 */

using namespace namma_metro;

namespace {

/// Build a single-source graph over @p num_nodes from an explicit edge list, and
/// restore the CSR invariant the router depends on: within each node's range,
/// edges MUST be sorted ascending by departure_time, because
/// select_optimal_departure binary-searches them.
struct EdgeSpec {
    uint32_t src, dst, dep, travel, crowd;
};

CSRGraph build(uint32_t num_nodes, std::vector<EdgeSpec> specs) {
    CSRGraph g;
    g.num_nodes = num_nodes;
    g.offset.assign(num_nodes + 1, 0);

    std::stable_sort(specs.begin(), specs.end(),
                     [](const EdgeSpec &a, const EdgeSpec &b) {
                         if (a.src != b.src) return a.src < b.src;
                         return a.dep < b.dep;
                     });

    for (const auto &s : specs) {
        Edge e;
        e.destination      = s.dst;
        e.departure_time   = s.dep;
        e.travel_time      = s.travel;
        e.secondary_weight = s.crowd;
        e.penalty          = 0;
        g.edge_data.push_back(e);
        ++g.offset[s.src + 1];
    }
    for (uint32_t i = 1; i <= num_nodes; ++i) g.offset[i] += g.offset[i - 1];
    g.num_edges = static_cast<uint32_t>(g.edge_data.size());
    return g;
}

/// Arrival time of the departure select_optimal_departure picks, or nullopt.
std::optional<uint32_t> arrival_at(const CSRGraph &g, const LookaheadConfig &cfg,
                                   uint32_t t, uint32_t u, uint32_t v) {
    auto e = select_optimal_departure(g, cfg, t, u, v);
    if (!e.has_value()) return std::nullopt;
    return e->departure_time + e->travel_time;
}

} // namespace

// ═════════════════════════════════════════════════════════════════════════════
// Test 1 — mechanism (1): the composite ignores arrival time.
//
//   Train X: departs 100, rides 400, crowd   5  -> composite 405, arrives 500
//   Train Y: departs 110, rides  50, crowd 400  -> composite 450, arrives 160
//
// At t=100 both are boardable and X wins on composite, so the engine arrives at
// 500. At t=105 train X has already left, only Y remains, and the engine arrives
// at 160. Querying LATER produced an EARLIER arrival.
// ═════════════════════════════════════════════════════════════════════════════
TEST(FIFOViolation, CrowdComposite_LaterQueryYieldsEarlierArrival) {
    CSRGraph g = build(2, {
        {0, 1, /*dep=*/100, /*travel=*/400, /*crowd=*/  5},
        {0, 1, /*dep=*/110, /*travel=*/ 50, /*crowd=*/400},
    });
    LookaheadConfig cfg{.k_departures = 5, .W_max_seconds = 1800, .lambda = 1.0f};
    ASSERT_EQ(g.second_objective, SecondObjective::CrowdExposure)
        << "This mechanism is specific to the crowd composite; the graph must be "
           "in CrowdExposure mode for the test to mean anything.";

    const auto early = arrival_at(g, cfg, 100, 0, 1);
    const auto late  = arrival_at(g, cfg, 105, 0, 1);

    ASSERT_TRUE(early.has_value());
    ASSERT_TRUE(late.has_value());

    EXPECT_EQ(*early, 500u)
        << "At t=100 both trains are boardable. composite(X)=400+1*5=405 beats "
           "composite(Y)=50+1*400=450, so X is chosen and arrival is 100+400=500.";
    EXPECT_EQ(*late, 160u)
        << "At t=105 train X (departing 100) is gone, so only Y is boardable: "
           "arrival is 110+50=160.";

    EXPECT_GT(*early, *late)
        << "DOCUMENTED VIOLATION: querying at t=105 (later) arrives at " << *late
        << ", earlier than querying at t=100 which arrives at " << *early << ". "
           "FIFO requires arrival to be non-decreasing in query time. The cause is "
           "that the crowd composite (travel + lambda*crowd + penalty) contains no "
           "departure_time term, so departure selection is blind to arrival. "
           "If this assertion fails, the selection rule changed — check whether "
           "arrival time was folded into the composite, and if so update "
           "docs/write-up.tex and the Known Limitations table in README.md.";
}

// ═════════════════════════════════════════════════════════════════════════════
// Test 2 — mechanism (2): the k budget truncates, even when minimising arrival.
//
// Six departures. The first five are slow (ride 1000); the sixth is nearly
// instant (ride 1). With k=5 the sixth is invisible at t=100, but at t=101 the
// first has dropped out and the sixth becomes the fifth candidate.
//
// lambda = 0 => arrival-minimising regime. The violation still fires, which is
// the point: making selection arrival-minimising does NOT restore FIFO.
// ═════════════════════════════════════════════════════════════════════════════
TEST(FIFOViolation, KBudgetTruncation_FiresEvenWhenMinimisingArrival) {
    CSRGraph g = build(2, {
        {0, 1, 100, 1000, 0}, {0, 1, 101, 1000, 0}, {0, 1, 102, 1000, 0},
        {0, 1, 103, 1000, 0}, {0, 1, 104, 1000, 0},
        {0, 1, 105,    1, 0},   // fast train, 6th in line
    });
    LookaheadConfig cfg{.k_departures = 5, .W_max_seconds = 86400, .lambda = 0.0f};

    const auto early = arrival_at(g, cfg, 100, 0, 1);
    const auto late  = arrival_at(g, cfg, 101, 0, 1);

    ASSERT_TRUE(early.has_value());
    ASSERT_TRUE(late.has_value());

    EXPECT_EQ(*early, 1100u)
        << "At t=100 the k=5 budget covers departures 100..104, all riding 1000s. "
           "Best arrival among them is 100+1000=1100. The fast train at 105 is the "
           "sixth candidate and is never examined.";
    EXPECT_EQ(*late, 106u)
        << "At t=101 the departure at 100 has dropped out, so the budget now covers "
           "101..105 and the fast train is visible: 105+1=106.";

    EXPECT_GT(*early, *late)
        << "DOCUMENTED VIOLATION via k-budget truncation, in the ARRIVAL-MINIMISING "
           "regime (lambda=0), where the composite IS the arrival time. This proves "
           "the violation is not caused solely by the crowd composite: the candidate "
           "SET itself moves with query time. Switching departure selection to "
           "minimise arrival — the obvious fix for Test 1 — would not remove this.";
}

// ═════════════════════════════════════════════════════════════════════════════
// Test 3 — mechanism (3): the W_max window truncates, same regime.
//
// A slow train inside the window, and a fast train 1 second beyond it. Advancing
// the query time by 1 second slides the window far enough to admit the fast one,
// while the slow one is still boardable.
// ═════════════════════════════════════════════════════════════════════════════
TEST(FIFOViolation, WindowTruncation_FiresEvenWhenMinimisingArrival) {
    CSRGraph g = build(2, {
        {0, 1,  200, 5000, 0},  // slow, comfortably inside the window
        {0, 1, 1901,    1, 0},  // fast, one second beyond it at t=100
    });
    LookaheadConfig cfg{.k_departures = 5, .W_max_seconds = 1800, .lambda = 0.0f};

    const auto early = arrival_at(g, cfg, 100, 0, 1);
    const auto late  = arrival_at(g, cfg, 101, 0, 1);

    ASSERT_TRUE(early.has_value());
    ASSERT_TRUE(late.has_value());

    EXPECT_EQ(*early, 5200u)
        << "At t=100 the window is [100, 1900]; the departure at 1901 falls outside "
           "it, so only the slow train is a candidate: 200+5000=5200.";
    EXPECT_EQ(*late, 1902u)
        << "At t=101 the window is [101, 1901] and now includes the fast train: "
           "1901+1=1902. The slow train is still boardable, so this is a genuine "
           "widening of the candidate set, not a substitution.";

    EXPECT_GT(*early, *late)
        << "DOCUMENTED VIOLATION via W_max window truncation, again in the "
           "arrival-minimising regime. Together with Test 2 this shows both "
           "truncation bounds — k and W_max — make the candidate set a function of "
           "query time, and any such bound can break FIFO regardless of how the "
           "winner is scored.";
}

// ═════════════════════════════════════════════════════════════════════════════
// Test 4 — THE CONTROL: constant per-link travel time restores FIFO.
//
// This is why the violation does not bite on the measured feeds.
// scripts/build_namma_metro_gtfs.py derives travel time from
// distance / average speed, so every departure on a given link rides for the
// same duration. Then arrival = departure + tau with tau fixed, so arrival is
// monotone in the chosen departure; and the chosen departure cannot move
// backwards as query time advances, because candidates are always >= t.
// Therefore arrival is non-decreasing. Swept over many query times.
// ═════════════════════════════════════════════════════════════════════════════
TEST(FIFOViolation, ConstantPerLinkTravelTime_ArrivalIsMonotone) {
    constexpr uint32_t TAU = 300; // identical for every departure on this link
    CSRGraph g = build(2, {
        {0, 1,  100, TAU, 1000},
        {0, 1,  400, TAU,  500},
        {0, 1,  700, TAU,  100},   // least crowded — chosen while in window
        {0, 1, 1000, TAU,  900},
        {0, 1, 1300, TAU,  200},
    });
    LookaheadConfig cfg{.k_departures = 5, .W_max_seconds = 1800, .lambda = 1.0f};

    std::optional<uint32_t> prev;
    for (uint32_t t = 0; t <= 1400; t += 25) {
        const auto arr = arrival_at(g, cfg, t, 0, 1);
        if (!arr.has_value()) continue;   // past the last departure
        if (prev.has_value()) {
            EXPECT_LE(*prev, *arr)
                << "FIFO must hold when per-link travel time is constant. "
                   "Arrival fell from " << *prev << " to " << *arr
                << " at query time " << t << ". If this fails, the argument that the "
                   "Namma Metro feed is safe from the violations in Tests 1-3 no "
                   "longer holds, and README.md's Known Limitations entry is wrong.";
        }
        prev = arr;
    }
    ASSERT_TRUE(prev.has_value()) << "The sweep must have selected at least one train";
}

// ═════════════════════════════════════════════════════════════════════════════
// Test 5 — THE CONSEQUENCE: dominance pruning can hide the only viable route.
//
// FIFO underpins "consistency under extension": if L1 dominates L2 at node u,
// extending both must preserve that. docs/write-up.tex claims this holds
// trivially for non-negative edge weights — but that argument assumes both
// labels are extended by the SAME edge, and they are not: the edge chosen is a
// function of arrival time.
//
//   0 --(dep 0, 100s)--> 1                       reaches node 1 at t=100, crowd 0
//   0 --(dep 0,  50s)--> 2 --(dep 50, 55s)--> 1  reaches node 1 at t=105, crowd 0
//
// (100, 0) strictly dominates (105, 0), so the second label is pruned at node 1.
// But the only onward service is:
//
//   1 --(dep 1901, 1s)--> 3
//
// From t=100 the window is [100, 1900] and 1901 is outside it. From t=105 the
// window is [105, 1905] and 1901 is inside it. So the PRUNED label was the only
// one that could reach node 3 — and node 3 is reported unreachable even though
// the journey 0 -> 2 -> 1 -> 3 is legal under the engine's own bounded-wait rule
// (the 1796s wait at node 1 is within W_max = 1800).
// ═════════════════════════════════════════════════════════════════════════════
TEST(FIFOViolation, DominatedPredecessorPruning_HidesAReachableDestination) {
    CSRGraph g = build(4, {
        {0, 1,    0, 100, 0},   // direct: node 1 at t=100
        {0, 2,    0,  50, 0},   // detour leg 1: node 2 at t=50
        {2, 1,   50,  55, 0},   // detour leg 2: node 1 at t=105
        {1, 3, 1901,   1, 0},   // onward: reachable only from t >= 101
    });
    LookaheadConfig cfg{.k_departures = 5, .W_max_seconds = 1800, .lambda = 0.0f};

    // Sanity: the two ways of reaching node 1, and the fact that only the later
    // one can catch the onward service.
    EXPECT_EQ(arrival_at(g, cfg, 100, 1, 3), std::nullopt)
        << "From t=100 the onward departure at 1901 lies outside [100, 1900]";
    EXPECT_EQ(arrival_at(g, cfg, 105, 1, 3), std::make_optional<uint32_t>(1902u))
        << "From t=105 the same departure lies inside [105, 1905]";

    ParetoDijkstra router(g, cfg);
    auto result = router.run(0, 0);

    ASSERT_FALSE(result.pareto_sets[1].empty()) << "Node 1 must be reached";
    EXPECT_EQ(result.pareto_sets[1].size(), 1u)
        << "Node 1 keeps one label: (100, 0) strictly dominates (105, 0), so the "
           "detour label is pruned by insert_and_dominate.";
    EXPECT_EQ(result.pareto_sets[1].labels().front()->arrival_time, 100u)
        << "The surviving label is the earlier arrival, as dominance requires";

    EXPECT_TRUE(result.pareto_sets[3].empty())
        << "DOCUMENTED CONSEQUENCE: node 3 is reported unreachable, but the journey "
           "0 ->(50s)-> 2 ->(55s)-> 1 ->(wait 1796s, 1s)-> 3 arriving at t=1902 is "
           "legal under this engine's own rules — every boarding is at or after the "
           "passenger's arrival and the 1796s wait is inside W_max=1800. The label "
           "that could have made the connection was discarded at node 1 for being "
           "Pareto-dominated. This falsifies the claim in docs/write-up.tex that "
           "consistency under extension 'holds trivially for non-negative edge "
           "weights': that argument assumes both labels are extended by the same "
           "edge, but select_optimal_departure chooses a different edge per arrival "
           "time. If this test starts failing, the engine has become more complete "
           "than documented — verify against the oracle and update write-up.tex §2.";
}
