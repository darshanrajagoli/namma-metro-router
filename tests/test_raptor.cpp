#include <gtest/gtest.h>
#include "graph.hpp"
#include "gtfs_parser.hpp"
#include "raptor.hpp"
#include "routing.hpp"
#include <algorithm>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @file test_raptor.cpp
 * @brief Contracts for the RAPTOR implementation.
 *
 * RAPTOR is load-bearing in three different ways here, and each needs a
 * different kind of test:
 *
 *   1. It is a SECOND IMPLEMENTATION of the same problem, so the strongest
 *     test available is agreement with the existing engine. Part 4 runs both
 *     over randomised networks with the engine's lookahead window opened up so
 *     that neither is allowed a heuristic shortcut, and asserts the earliest
 *     arrival matches at every node of every query. Two unrelated algorithms
 *     agreeing on thousands of answers is worth more than any number of
 *     hand-written expectations.
 *
 *   2. It is an ORACLE, so it must never be optimistic. Part 4 also asserts the
 *     one-sided property that makes it usable as one: with the shipped
 *     lookahead settings the engine may be LATER than RAPTOR but never earlier.
 *     If that direction ever flips, the oracle is broken and every measurement
 *     taken with it is void.
 *
 *   3. Its round structure IS the second objective, so the frontier extracted
 *     from the rounds has to be the real time-versus-trips frontier. Part 3
 *     builds a network with a deliberate fast-but-two-trips / slow-but-direct
 *     choice and pins both points.
 *
 * Parts 1 and 2 cover the preprocessing, where the subtle failures live: a stop
 * pattern whose trips overtake, a trip with one unusable segment, a footpath
 * relation that is not transitively closed.
 */

using namespace namma_metro;

namespace
{

    StopTimeRecord st(std::string trip, uint32_t arr, uint32_t dep,
                      std::string stop, uint32_t seq)
    {
        StopTimeRecord r;
        r.trip_id = std::move(trip);
        r.arrival_time = arr;
        r.departure_time = dep;
        r.stop_id = std::move(stop);
        r.stop_sequence = seq;
        r.pickup_type = 0;
        r.drop_off_type = 0;
        return r;
    }

    TransferRecord tr(std::string from, std::string to, uint32_t secs)
    {
        TransferRecord t;
        t.from_stop_id = std::move(from);
        t.to_stop_id = std::move(to);
        t.min_transfer_time = secs;
        return t;
    }

    // Six nodes is enough for every structural case below.
    const std::unordered_map<std::string, uint32_t> kIdx = {
        {"A", 0}, {"B", 1}, {"C", 2}, {"D", 3}, {"E", 4}, {"F", 5}};

    /// The lookahead settings under which the Pareto engine is NOT allowed to
    /// take a shortcut: consider every departure, over a whole day, and pick the
    /// one that arrives earliest. Under these settings the two algorithms are
    /// solving exactly the same problem and must agree exactly.
    LookaheadConfig unrestricted()
    {
        LookaheadConfig c;
        c.k_departures = UINT32_MAX;
        c.W_max_seconds = 172800; // two days; window_end is computed in uint64
        c.lambda = 0.0f;          // arrival-minimising departure selection
        return c;
    }

    /// Earliest arrival at each node according to the Pareto engine: the minimum
    /// over its frontier, or RAPTOR_UNREACHED when the node was never reached.
    std::vector<uint32_t> engine_best_arrivals(ParetoDijkstra &router, const CSRGraph &g,
                                               uint32_t src, uint32_t dep)
    {
        auto res = router.run(src, dep);
        std::vector<uint32_t> out(g.num_nodes, RAPTOR_UNREACHED);
        for (uint32_t v = 0; v < g.num_nodes; ++v)
        {
            for (const Label *l : res.pareto_sets[v].labels())
                out[v] = std::min(out[v], l->arrival_time);
        }
        return out;
    }

} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Part 1 — Timetable construction
// ═══════════════════════════════════════════════════════════════════════════

TEST(RaptorBuild, SingleTripBecomesOneRouteWithOneTrip)
{
    auto idx = kIdx;
    std::vector<StopTimeRecord> stop_times = {
        st("T1", 28800, 28800, "A", 0),
        st("T1", 29100, 29130, "B", 1),
        st("T1", 29400, 29400, "C", 2),
    };
    auto tt = RaptorBuilder::build(stop_times, 6, &idx, {});

    ASSERT_EQ(tt.num_routes, 1u);
    EXPECT_EQ(tt.num_trips, 1u);
    EXPECT_EQ(tt.route_length(0), 3u);
    EXPECT_EQ(tt.stops_of(0)[0], 0u);
    EXPECT_EQ(tt.stops_of(0)[1], 1u);
    EXPECT_EQ(tt.stops_of(0)[2], 2u);

    const auto *times = tt.trip_times(0, 0);
    EXPECT_EQ(times[0].departure, 28800u);
    EXPECT_EQ(times[1].arrival, 29100u);
    EXPECT_EQ(times[1].departure, 29130u); // dwell preserved
    EXPECT_EQ(times[2].arrival, 29400u);
}

// Two trips over the same stops are one route with two trips, ordered in time.
// This is the whole point of the pattern grouping: the route is scanned once
// and the trip is chosen inside it.
TEST(RaptorBuild, SamePatternTripsShareARouteAndAreOrdered)
{
    auto idx = kIdx;
    std::vector<StopTimeRecord> stop_times = {
        st("LATE", 30000, 30000, "A", 0),
        st("LATE", 30300, 30300, "B", 1),
        st("EARLY", 28800, 28800, "A", 0),
        st("EARLY", 29100, 29100, "B", 1),
    };
    auto tt = RaptorBuilder::build(stop_times, 6, &idx, {});

    ASSERT_EQ(tt.num_routes, 1u);
    ASSERT_EQ(tt.route_trip_count[0], 2u);
    EXPECT_EQ(tt.trip_times(0, 0)[0].departure, 28800u);
    EXPECT_EQ(tt.trip_times(0, 1)[0].departure, 30000u);
    EXPECT_EQ(tt.routes_split_for_overtaking, 0u);
}

// An express that leaves later and arrives earlier overtakes the stopper. The
// binary search for "earliest catchable trip" assumes trips are totally ordered
// in time, so such a pattern MUST be split into two routes or the search will
// silently return a trip that is not the earliest.
TEST(RaptorBuild, OvertakingTripsAreSplitIntoSeparateRoutes)
{
    auto idx = kIdx;
    std::vector<StopTimeRecord> stop_times = {
        // Stopper: leaves 08:00, arrives 08:30.
        st("STOPPER", 28800, 28800, "A", 0),
        st("STOPPER", 30600, 30600, "B", 1),
        // Express: leaves 08:05, arrives 08:15 — overtakes.
        st("EXPRESS", 29100, 29100, "A", 0),
        st("EXPRESS", 29700, 29700, "B", 1),
    };
    auto tt = RaptorBuilder::build(stop_times, 6, &idx, {});

    EXPECT_EQ(tt.num_routes, 2u);
    EXPECT_EQ(tt.routes_split_for_overtaking, 1u);
    EXPECT_EQ(tt.num_trips, 2u);
}

// GraphBuilder drops the individual segment whose arrival precedes its
// departure and keeps the rest of the trip. RAPTOR has to make the same choice,
// which for a trip-based structure means splitting rather than dropping —
// otherwise the two engines are routing over different networks and no
// comparison between them means anything.
TEST(RaptorBuild, TripWithAnUnusableSegmentIsSplitNotDropped)
{
    auto idx = kIdx;
    std::vector<StopTimeRecord> stop_times = {
        st("T", 28800, 28800, "A", 0),
        st("T", 29100, 29100, "B", 1),
        st("T", 29000, 29000, "C", 2), // arrives BEFORE B departs: inadmissible
        st("T", 30000, 30000, "D", 3),
    };
    auto tt = RaptorBuilder::build(stop_times, 6, &idx, {});

    ASSERT_EQ(tt.num_routes, 2u);
    EXPECT_GE(tt.trips_split_for_bad_segment, 1u);

    // A->B survives as one route and C->D as the other; B->C is gone.
    std::vector<std::vector<uint32_t>> patterns;
    for (uint32_t r = 0; r < tt.num_routes; ++r)
        patterns.push_back({tt.stops_of(r), tt.stops_of(r) + tt.route_length(r)});
    std::sort(patterns.begin(), patterns.end());
    EXPECT_EQ(patterns[0], (std::vector<uint32_t>{0, 1}));
    EXPECT_EQ(patterns[1], (std::vector<uint32_t>{2, 3}));
}

// GraphBuilder clamps a zero-length segment to one second so it cannot create a
// zero-weight cycle. RAPTOR must produce the identical arrival time.
TEST(RaptorBuild, ZeroDurationSegmentIsClampedExactlyAsGraphBuilderDoes)
{
    auto idx = kIdx;
    std::vector<StopTimeRecord> stop_times = {
        st("T", 28800, 28800, "A", 0),
        st("T", 28800, 28800, "B", 1), // zero nominal travel time
    };
    auto tt = RaptorBuilder::build(stop_times, 6, &idx, {});
    ASSERT_EQ(tt.num_routes, 1u);
    EXPECT_EQ(tt.trip_times(0, 0)[1].arrival, 28801u);

    CSRGraph g = GraphBuilder::build(stop_times, 6, &idx);
    ASSERT_EQ(g.num_edges, 1u);
    EXPECT_EQ(g.edge_data[0].departure_time + g.edge_data[0].travel_time, 28801u);
}

TEST(RaptorBuild, MissingStopIndexMapThrows)
{
    std::vector<StopTimeRecord> stop_times = {
        st("T", 28800, 28800, "A", 0),
        st("T", 29100, 29100, "B", 1),
    };
    EXPECT_THROW(RaptorBuilder::build(stop_times, 6, nullptr, {}), std::invalid_argument);
}

TEST(RaptorBuild, EmptyFeedProducesAWellFormedEmptyTimetable)
{
    auto idx = kIdx;
    auto tt = RaptorBuilder::build({}, 6, &idx, {});
    EXPECT_EQ(tt.num_routes, 0u);
    EXPECT_EQ(tt.num_trips, 0u);
    EXPECT_EQ(tt.num_stops, 6u);

    Raptor r(tt);
    auto res = r.run(0, 28800);
    EXPECT_TRUE(res.reached(0));  // the source, trivially
    EXPECT_FALSE(res.reached(1)); // nothing else
}

// ═══════════════════════════════════════════════════════════════════════════
// Part 2 — Footpaths
// ═══════════════════════════════════════════════════════════════════════════

// normalize_gtfs.py emits every ordered platform pair of a station, so the
// footpath relation is complete within each station and therefore transitively
// closed. RAPTOR's single relaxation pass per round depends on that.
TEST(RaptorFootpaths, CompletePerStationRelationIsTransitivelyClosed)
{
    auto idx = kIdx;
    std::vector<StopTimeRecord> stop_times = {
        st("T", 28800, 28800, "A", 0),
        st("T", 29100, 29100, "D", 1),
    };
    // A, B, C are three platforms of one station: all six ordered pairs.
    std::vector<TransferRecord> transfers = {
        tr("A", "B", 60), tr("B", "A", 60), tr("A", "C", 60),
        tr("C", "A", 60), tr("B", "C", 60), tr("C", "B", 60)};
    auto tt = RaptorBuilder::build(stop_times, 6, &idx, transfers);
    EXPECT_EQ(tt.num_transfers, 6u);
    EXPECT_TRUE(transfers_are_transitively_closed(tt));
}

// A transfers.txt can encode A->B and B->C without A->C. One relaxation pass
// would then miss the two-leg walk, so the builder adds the shortcut.
TEST(RaptorFootpaths, MissingShortcutIsAddedByTheBuilder)
{
    auto idx = kIdx;
    std::vector<StopTimeRecord> stop_times = {
        st("T", 28800, 28800, "A", 0),
        st("T", 29100, 29100, "D", 1),
    };
    std::vector<TransferRecord> transfers = {tr("A", "B", 60), tr("B", "C", 60)};
    auto tt = RaptorBuilder::build(stop_times, 6, &idx, transfers);

    EXPECT_TRUE(transfers_are_transitively_closed(tt));
    EXPECT_EQ(tt.num_transfers, 3u); // A->B, B->C and the new A->C
    EXPECT_EQ(tt.footpaths_added_by_closure, 1u);

    const auto [b, e] = tt.transfers_of(0);
    bool found = false;
    for (const TransferEdge *t = b; t != e; ++t)
        if (t->destination == 2)
        {
            found = true;
            EXPECT_EQ(t->travel_time, 120u);
        }
    EXPECT_TRUE(found) << "the two-leg walk A->B->C must exist as a footpath A->C";
}

// The case that actually bit: a feed supplies a DIRECT time between two
// platforms that is longer than walking via a third. Closure must shorten it,
// not merely add missing pairs — otherwise the single pass returns the slow
// direct walk and calls it exact.
TEST(RaptorFootpaths, ADirectWalkSlowerThanTheDetourIsShortened)
{
    auto idx = kIdx;
    std::vector<StopTimeRecord> stop_times = {
        st("T", 28800, 28800, "A", 0),
        st("T", 29100, 29100, "D", 1),
    };
    std::vector<TransferRecord> transfers = {
        tr("A", "B", 30), tr("B", "C", 30), tr("A", "C", 600)};
    auto tt = RaptorBuilder::build(stop_times, 6, &idx, transfers);

    EXPECT_TRUE(transfers_are_transitively_closed(tt));
    const auto [b, e] = tt.transfers_of(0);
    for (const TransferEdge *t = b; t != e; ++t)
    {
        if (t->destination == 2)
        {
            EXPECT_EQ(t->travel_time, 60u) << "600s direct must be replaced by the 60s detour";
        }
    }
}

// A journey may legitimately start with a walk to a sibling platform. Without
// round-0 footpath relaxation the origin looks stranded.
TEST(RaptorFootpaths, JourneyMayBeginWithAWalkFromTheSource)
{
    auto idx = kIdx;
    // Service leaves from B only; the passenger starts at A.
    std::vector<StopTimeRecord> stop_times = {
        st("T", 29000, 29000, "B", 0),
        st("T", 29300, 29300, "C", 1),
    };
    std::vector<TransferRecord> transfers = {tr("A", "B", 120), tr("B", "A", 120)};
    auto tt = RaptorBuilder::build(stop_times, 6, &idx, transfers);

    Raptor r(tt);
    auto res = r.run(/*source=*/0, /*departure=*/28800);
    EXPECT_EQ(res.arrival(0, 1), 28920u); // walked to B by 08:02
    EXPECT_EQ(res.best(2), 29300u);       // then rode to C
    EXPECT_EQ(res.rounds_to_best(2), 1u); // one vehicle used
}

// ═══════════════════════════════════════════════════════════════════════════
// Part 3 — Rounds are the second objective
// ═══════════════════════════════════════════════════════════════════════════

// The classic trade-off: a direct service that is slow, and a two-leg
// alternative that is faster. Both are non-dominated, so the exact frontier at
// the destination has two points and they are exactly these two.
TEST(RaptorFrontier, FastWithAChangeAndSlowDirectAreBothOnTheFrontier)
{
    auto idx = kIdx;
    std::vector<StopTimeRecord> stop_times = {
        // Slow direct A -> D, 08:00 to 09:00.
        st("DIRECT", 28800, 28800, "A", 0),
        st("DIRECT", 32400, 32400, "D", 1),
        // Fast leg 1: A -> B, 08:00 to 08:20.
        st("FAST1", 28800, 28800, "A", 0),
        st("FAST1", 30000, 30000, "B", 1),
        // Fast leg 2: B -> D, 08:25 to 08:40.
        st("FAST2", 30300, 30300, "B", 0),
        st("FAST2", 31200, 31200, "D", 1),
    };
    auto tt = RaptorBuilder::build(stop_times, 6, &idx, {});
    Raptor r(tt);
    auto res = r.run(0, 28800);

    EXPECT_EQ(res.arrival(1, 3), 32400u); // one trip: the slow direct
    EXPECT_EQ(res.arrival(2, 3), 31200u); // two trips: the fast pair
    EXPECT_EQ(res.frontier_size(3), 2u);
    EXPECT_EQ(res.best(3), 31200u);
    EXPECT_EQ(res.rounds_to_best(3), 2u);
}

// When the direct service is also the fastest, there is no trade-off and the
// frontier has exactly one point — the case the multi-feed study is counting.
TEST(RaptorFrontier, NoTradeOffWhenDirectIsAlsoFastest)
{
    auto idx = kIdx;
    std::vector<StopTimeRecord> stop_times = {
        st("DIRECT", 28800, 28800, "A", 0),
        st("DIRECT", 30000, 30000, "D", 1),
        st("SLOW1", 28800, 28800, "A", 0),
        st("SLOW1", 30600, 30600, "B", 1),
        st("SLOW2", 30900, 30900, "B", 0),
        st("SLOW2", 32400, 32400, "D", 1),
    };
    auto tt = RaptorBuilder::build(stop_times, 6, &idx, {});
    Raptor r(tt);
    auto res = r.run(0, 28800);

    EXPECT_EQ(res.frontier_size(3), 1u);
    EXPECT_EQ(res.best(3), 30000u);
}

// The round cap must be visible when it binds. A silently truncated answer is
// indistinguishable from an exact one, which is the failure mode that matters.
TEST(RaptorRounds, HittingTheRoundCapIsReported)
{
    auto idx = kIdx;
    std::vector<StopTimeRecord> stop_times = {
        st("L1", 28800, 28800, "A", 0),
        st("L1", 29100, 29100, "B", 1),
        st("L2", 29400, 29400, "B", 0),
        st("L2", 29700, 29700, "C", 1),
        st("L3", 30000, 30000, "C", 0),
        st("L3", 30300, 30300, "D", 1),
    };
    auto tt = RaptorBuilder::build(stop_times, 6, &idx, {});

    Raptor capped(tt, /*max_rounds=*/1);
    auto truncated = capped.run(0, 28800);
    EXPECT_TRUE(truncated.hit_round_cap);
    EXPECT_FALSE(truncated.reached(3)); // D needs three vehicles

    Raptor full(tt, /*max_rounds=*/8);
    auto exact = full.run(0, 28800);
    EXPECT_FALSE(exact.hit_round_cap);
    EXPECT_EQ(exact.best(3), 30300u);
    EXPECT_EQ(exact.rounds_to_best(3), 3u);
}

// Scratch hygiene: the engine reuses its marked-stop stamps and route queue
// across queries. A leak between queries would show up as an answer that
// depends on what was asked before it.
TEST(RaptorRounds, ReusedResultBufferMatchesAFreshOne)
{
    auto idx = kIdx;
    std::vector<StopTimeRecord> stop_times = {
        st("L1", 28800, 28800, "A", 0),
        st("L1", 29100, 29100, "B", 1),
        st("L2", 29400, 29400, "B", 0),
        st("L2", 29700, 29700, "C", 1),
    };
    auto tt = RaptorBuilder::build(stop_times, 6, &idx, {});
    Raptor r(tt);

    RaptorResult reused;
    r.run(1, 29000, reused); // a different query first
    r.run(0, 28800, reused);

    Raptor fresh_engine(tt);
    auto fresh = fresh_engine.run(0, 28800);

    ASSERT_EQ(reused.rounds, fresh.rounds);
    for (uint32_t p = 0; p < tt.num_stops; ++p)
        EXPECT_EQ(reused.best(p), fresh.best(p)) << "stop " << p;
}

// ═══════════════════════════════════════════════════════════════════════════
// Part 4 — Agreement with the Pareto engine
// ═══════════════════════════════════════════════════════════════════════════
namespace
{

    struct RandomFeed
    {
        std::vector<StopTimeRecord> stop_times;
        std::vector<TransferRecord> transfers;
        std::unordered_map<std::string, uint32_t> idx;
        uint32_t num_stops = 0;
    };

    /// A small randomised network: `lines` paths through a shared pool of stops,
    /// each running `trips_per_line` times at an irregular headway, plus a
    /// complete footpath clique over one pair of stops so transfers exist.
    ///
    /// Irregular headways matter: a constant headway makes arrival monotone in
    /// departure for trivial reasons and would hide exactly the disagreements
    /// this test is looking for.
    RandomFeed make_random_feed(uint32_t seed, uint32_t num_stops,
                                uint32_t lines, uint32_t trips_per_line)
    {
        std::mt19937 rng(seed);
        RandomFeed f;
        f.num_stops = num_stops;
        for (uint32_t i = 0; i < num_stops; ++i)
            f.idx["S" + std::to_string(i)] = i;

        std::uniform_int_distribution<uint32_t> pick(0, num_stops - 1);
        std::uniform_int_distribution<uint32_t> len(2, std::min(6u, num_stops));
        std::uniform_int_distribution<uint32_t> ride(60, 600);
        std::uniform_int_distribution<uint32_t> dwell(0, 60);
        std::uniform_int_distribution<uint32_t> gap(120, 1500);

        for (uint32_t l = 0; l < lines; ++l)
        {
            // A path of distinct stops.
            std::vector<uint32_t> path;
            while (path.size() < len(rng))
            {
                const uint32_t s = pick(rng);
                if (std::find(path.begin(), path.end(), s) == path.end())
                    path.push_back(s);
            }
            if (path.size() < 2)
                continue;

            uint32_t t0 = 25200 + gap(rng);
            for (uint32_t k = 0; k < trips_per_line; ++k)
            {
                const std::string trip = "L" + std::to_string(l) + "_" + std::to_string(k);
                uint32_t t = t0;
                for (std::size_t p = 0; p < path.size(); ++p)
                {
                    const uint32_t arr = t;
                    const uint32_t dep = (p + 1 == path.size()) ? t : t + dwell(rng);
                    f.stop_times.push_back(
                        st(trip, arr, dep, "S" + std::to_string(path[p]), static_cast<uint32_t>(p)));
                    t = dep + ride(rng);
                }
                t0 += gap(rng);
            }
        }

        // One two-platform station, both directions, so the footpath relation
        // stays transitively closed.
        if (num_stops >= 2)
        {
            f.transfers.push_back(tr("S0", "S1", 90));
            f.transfers.push_back(tr("S1", "S0", 90));
        }
        return f;
    }

} // namespace

// The central claim: with the engine's departure-selection window opened up so
// that it is no longer allowed a heuristic shortcut, two completely different
// algorithms must agree on the earliest arrival at every node.
TEST(RaptorVsEngine, UnrestrictedEngineAgreesExactlyOnEarliestArrival)
{
    uint64_t compared = 0;
    for (uint32_t seed = 1; seed <= 25; ++seed)
    {
        auto f = make_random_feed(seed, 12, 6, 5);
        if (f.stop_times.empty())
            continue;

        CSRGraph g = GraphBuilder::build_with_transfers(
            f.stop_times, f.num_stops, &f.idx, f.transfers, SecondObjective::TransferCount);
        auto tt = RaptorBuilder::build(f.stop_times, f.num_stops, &f.idx, f.transfers);
        ASSERT_TRUE(transfers_are_transitively_closed(tt)) << "seed " << seed;

        ParetoDijkstra router(g, unrestricted());
        Raptor raptor(tt, 16);

        for (uint32_t src = 0; src < f.num_stops; ++src)
        {
            for (const uint32_t dep : {25200u, 28800u, 32400u})
            {
                const auto engine = engine_best_arrivals(router, g, src, dep);
                auto rr = raptor.run(src, dep);
                ASSERT_FALSE(rr.hit_round_cap) << "seed " << seed << " src " << src;

                for (uint32_t v = 0; v < f.num_stops; ++v)
                {
                    ++compared;
                    EXPECT_EQ(engine[v], rr.best(v))
                        << "seed " << seed << " src " << src << " dep " << dep << " node " << v;
                }
            }
        }
    }
    EXPECT_GT(compared, 5000u) << "the agreement test must actually compare something";
}

// The property that makes RAPTOR usable as an oracle for the SHIPPED engine
// settings: the bounded-wait lookahead can only ever cost time, never save it.
// If this direction ever inverts, either the engine is finding journeys that do
// not exist or RAPTOR is missing some, and every optimality-gap number measured
// with it is void.
TEST(RaptorVsEngine, ShippedLookaheadIsNeverEarlierThanTheOracle)
{
    LookaheadConfig shipped; // k = 5, W_max = 1800, lambda = 1.0 — the defaults
    uint64_t compared = 0, engine_later = 0;

    for (uint32_t seed = 100; seed <= 120; ++seed)
    {
        auto f = make_random_feed(seed, 10, 5, 4);
        if (f.stop_times.empty())
            continue;

        CSRGraph g = GraphBuilder::build_with_transfers(
            f.stop_times, f.num_stops, &f.idx, f.transfers, SecondObjective::TransferCount);
        auto tt = RaptorBuilder::build(f.stop_times, f.num_stops, &f.idx, f.transfers);

        ParetoDijkstra router(g, shipped);
        Raptor raptor(tt, 16);

        for (uint32_t src = 0; src < f.num_stops; ++src)
        {
            const auto engine = engine_best_arrivals(router, g, src, 28800);
            auto rr = raptor.run(src, 28800);
            for (uint32_t v = 0; v < f.num_stops; ++v)
            {
                if (!rr.reached(v))
                {
                    // RAPTOR explores strictly more, so anything it cannot reach
                    // the engine must not reach either.
                    EXPECT_EQ(engine[v], RAPTOR_UNREACHED)
                        << "seed " << seed << " src " << src << " node " << v;
                    continue;
                }
                if (engine[v] == RAPTOR_UNREACHED)
                    continue; // the window cost reachability entirely; allowed
                ++compared;
                EXPECT_GE(engine[v], rr.best(v))
                    << "engine beat the oracle: seed " << seed << " src " << src << " node " << v;
                if (engine[v] > rr.best(v))
                    ++engine_later;
            }
        }
    }
    EXPECT_GT(compared, 500u);
    // Not asserted as a specific number — how often the window bites depends on
    // the random feed. Recorded so a change in the engine's defaults that made
    // it exact would be visible here rather than silent.
    RecordProperty("engine_later_than_oracle", static_cast<int>(engine_later));
}
