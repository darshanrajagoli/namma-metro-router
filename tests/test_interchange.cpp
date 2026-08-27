#include <gtest/gtest.h>

#include "gtfs_parser.hpp"
#include "interchange.hpp"
#include "topology.hpp"

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @file test_interchange.cpp
 * @brief Contracts for the "where should one new interchange go" search.
 *
 * THE TEST NETWORK, AND WHY IT IS SHAPED LIKE THIS
 * ════════════════════════════════════════════════
 * Two straight lines that never meet:
 *
 *      A --- B --- C          (line 1, at longitude 77.5900)
 *      D --- E --- F          (line 2)
 *
 * Every station is served in both directions, all day. The two lines are far
 * enough apart that no passenger can get from one to the other — the station
 * graph has two components — EXCEPT that E has been placed 217 m from B.
 *
 * So the right answer is known by inspection before any code runs: B <-> E is
 * the one connection that joins the network, and it must beat every other
 * candidate. A and C also fall within 800 m of E, which gives the ranking
 * something to actually rank rather than a single candidate that wins by
 * default.
 *
 * The invariant worth more than any individual number is monotonicity: a
 * footpath adds options and can never take one away, so no candidate may ever
 * REDUCE reachability. If a future change to the closure, the station merge or
 * the round accounting breaks that, this is the test that says so.
 */

using namespace namma_metro;

namespace
{

    StopTimeRecord st(std::string trip, uint32_t arr, uint32_t dep, std::string stop, uint32_t seq)
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

    StopRecord stop_rec(std::string id, std::string name, double lat, double lon)
    {
        StopRecord s;
        s.stop_id = std::move(id);
        s.stop_name = std::move(name);
        s.stop_lat = lat;
        s.stop_lon = lon;
        s.location_type = 0;
        return s;
    }

    /// Owns every buffer InterchangeInputs points at, so the pointers stay valid
    /// for as long as the test does.
    struct TwoLineNetwork
    {
        std::vector<StopTimeRecord> stop_times;
        std::vector<StopRecord> stops;
        std::vector<TransferRecord> transfers; // empty: the feed joins nothing
        std::vector<TripRecord> trips;         // empty: line metrics not needed here
        std::unordered_map<std::string, uint32_t> idx;
        StationGraph sg;

        TwoLineNetwork()
        {
            idx = {{"A", 0}, {"B", 1}, {"C", 2}, {"D", 3}, {"E", 4}, {"F", 5}};

            // Line 1 runs due north at a fixed longitude. On line 2, D and F sit
            // 0.01 deg east (~1085 m, comfortably out of walking range) while E is
            // pulled to within 217 m of B. Only that pair can be walked.
            stops.push_back(stop_rec("A", "Alpha", 12.9700, 77.5900));
            stops.push_back(stop_rec("B", "Bravo", 12.9750, 77.5900));
            stops.push_back(stop_rec("C", "Charlie", 12.9800, 77.5900));
            stops.push_back(stop_rec("D", "Delta", 12.9700, 77.6000));
            stops.push_back(stop_rec("E", "Echo", 12.9750, 77.5920));
            stops.push_back(stop_rec("F", "Foxtrot", 12.9800, 77.6000));

            // Every ten minutes from 07:00 to 20:00, five minutes per hop, both
            // directions on both lines. Frequent enough that every sampled
            // departure in AccessibilityConfig finds service.
            uint32_t n = 0;
            for (uint32_t t = 25200; t <= 72000; t += 600)
            {
                auto run = [&](const char *a, const char *b, const char *c)
                {
                    const std::string id = "T" + std::to_string(n++);
                    stop_times.push_back(st(id, t, t, a, 0));
                    stop_times.push_back(st(id, t + 300, t + 300, b, 1));
                    stop_times.push_back(st(id, t + 600, t + 600, c, 2));
                };
                run("A", "B", "C");
                run("C", "B", "A");
                run("D", "E", "F");
                run("F", "E", "D");
            }

            // The metrics themselves are test_topology.cpp's subject; all this
            // fixture needs is the station graph written through the out-param.
            (void)compute_topology(stop_times, trips, 6, &idx, transfers, &sg);
        }

        [[nodiscard]] InterchangeInputs inputs() const
        {
            InterchangeInputs in;
            in.stop_times = &stop_times;
            in.num_stops = 6;
            in.stop_index_map = &idx;
            in.base_transfers = &transfers;
            in.stops = &stops;
            in.station_graph = &sg;
            return in;
        }
    };

    InterchangeSearchConfig config_with(double max_walk)
    {
        InterchangeSearchConfig cfg;
        cfg.max_walk_m = max_walk;
        cfg.accessibility.thresholds_s = {2700};
        cfg.accessibility.max_changes = 2;
        cfg.threshold_index = 0;
        return cfg;
    }

    bool has_pair(const std::vector<InterchangeCandidate> &v, uint32_t a, uint32_t b)
    {
        for (const auto &c : v)
            if (c.station_a == std::min(a, b) && c.station_b == std::max(a, b))
                return true;
        return false;
    }

} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Distance
// ═══════════════════════════════════════════════════════════════════════════

TEST(Haversine, IsZeroOnIdentityAndSymmetric)
{
    EXPECT_DOUBLE_EQ(haversine_m(12.97, 77.59, 12.97, 77.59), 0.0);
    EXPECT_NEAR(haversine_m(12.97, 77.59, 12.98, 77.60),
                haversine_m(12.98, 77.60, 12.97, 77.59), 1e-9);
}

TEST(Haversine, OneDegreeOfLatitudeIsAboutOneHundredAndElevenKilometres)
{
    // The meridian arc of one degree on a sphere of radius 6371 km is 111.19 km.
    EXPECT_NEAR(haversine_m(0.0, 0.0, 1.0, 0.0), 111195.0, 50.0);
}

TEST(Haversine, MatchesTheTestNetworkGeometry)
{
    // B and E, the pair the whole fixture is built around.
    EXPECT_NEAR(haversine_m(12.9750, 77.5900, 12.9750, 77.5920), 217.0, 5.0);
}

// ═══════════════════════════════════════════════════════════════════════════
// Candidate generation
// ═══════════════════════════════════════════════════════════════════════════

TEST(InterchangeCandidates, ExcludesStationsAVehicleAlreadyRunsBetween)
{
    TwoLineNetwork net;
    const auto cands = generate_interchange_candidates(net.inputs(), config_with(2000.0));

    // A-B, B-C, D-E and E-F are all served consecutively, so none of them is an
    // interchange to be built — a footpath there is a walk beside a ride.
    EXPECT_FALSE(has_pair(cands, 0, 1));
    EXPECT_FALSE(has_pair(cands, 1, 2));
    EXPECT_FALSE(has_pair(cands, 3, 4));
    EXPECT_FALSE(has_pair(cands, 4, 5));
}

TEST(InterchangeCandidates, RespectsTheWalkingRadius)
{
    TwoLineNetwork net;

    // At 300 m only B-E survives; A-E and C-E are ~597 m away.
    const auto tight = generate_interchange_candidates(net.inputs(), config_with(300.0));
    ASSERT_EQ(tight.size(), 1u);
    EXPECT_EQ(tight[0].station_a, 1u); // B
    EXPECT_EQ(tight[0].station_b, 4u); // E

    // At 800 m the two diagonals join it, and nothing else: every remaining pair
    // is over a kilometre apart.
    const auto wide = generate_interchange_candidates(net.inputs(), config_with(800.0));
    EXPECT_EQ(wide.size(), 3u);
    EXPECT_TRUE(has_pair(wide, 1, 4));
    EXPECT_TRUE(has_pair(wide, 0, 4));
    EXPECT_TRUE(has_pair(wide, 2, 4));
}

TEST(InterchangeCandidates, AreSortedByDistanceAndNormalised)
{
    TwoLineNetwork net;
    const auto cands = generate_interchange_candidates(net.inputs(), config_with(800.0));
    ASSERT_FALSE(cands.empty());

    for (const auto &c : cands)
        EXPECT_LT(c.station_a, c.station_b) << "each pair must appear once, low index first";

    for (std::size_t i = 1; i < cands.size(); ++i)
        EXPECT_LE(cands[i - 1].distance_m, cands[i].distance_m);
}

TEST(InterchangeCandidates, WalkTimeFollowsDistanceButNeverDropsBelowTheFloor)
{
    TwoLineNetwork net;
    InterchangeSearchConfig cfg = config_with(800.0);
    cfg.walk_speed_mps = 1.0; // one metre per second: seconds == metres
    cfg.min_walk_time_s = 60;

    const auto cands = generate_interchange_candidates(net.inputs(), cfg);
    ASSERT_FALSE(cands.empty());
    for (const auto &c : cands)
    {
        const uint32_t expected =
            std::max(cfg.min_walk_time_s, static_cast<uint32_t>(std::lround(c.distance_m)));
        EXPECT_EQ(c.walk_time_s, expected);
        EXPECT_GE(c.walk_time_s, cfg.min_walk_time_s);
    }
}

TEST(InterchangeCandidates, DistinctlyNamedStationsAreNotFlagged)
{
    TwoLineNetwork net;
    for (const auto &c : generate_interchange_candidates(net.inputs(), config_with(800.0)))
        EXPECT_FALSE(c.same_name);
}

TEST(InterchangeCandidates, FlagsPairsThatShareAStationName)
{
    // What an unjoined station complex looks like in a feed: one physical
    // station arriving as two components because transfers.txt does not join
    // them. Renaming Echo to Bravo reproduces exactly that against B.
    TwoLineNetwork net;
    net.stops[4].stop_name = "Bravo";

    const auto cands = generate_interchange_candidates(net.inputs(), config_with(800.0));
    ASSERT_FALSE(cands.empty());

    bool saw_the_pair = false;
    for (const auto &c : cands)
    {
        const bool is_b_e = (c.station_a == 1u && c.station_b == 4u);
        EXPECT_EQ(c.same_name, is_b_e) << "only the same-named pair may be flagged";
        saw_the_pair = saw_the_pair || is_b_e;
    }
    EXPECT_TRUE(saw_the_pair);
}

TEST(InterchangeCandidates, NameMatchingIgnoresCaseAndSurroundingSpace)
{
    TwoLineNetwork net;
    net.stops[4].stop_name = "  bRaVo ";

    for (const auto &c : generate_interchange_candidates(net.inputs(), config_with(800.0)))
    {
        if (c.station_a == 1u && c.station_b == 4u)
        {
            EXPECT_TRUE(c.same_name);
        }
    }
}

TEST(InterchangeCandidates, NullInputsThrow)
{
    TwoLineNetwork net;
    InterchangeInputs bad = net.inputs();
    bad.station_graph = nullptr;
    EXPECT_THROW((void)generate_interchange_candidates(bad, config_with(800.0)),
                 std::invalid_argument);

    InterchangeInputs bad2 = net.inputs();
    bad2.stops = nullptr;
    EXPECT_THROW((void)generate_interchange_candidates(bad2, config_with(800.0)),
                 std::invalid_argument);
}

// ═══════════════════════════════════════════════════════════════════════════
// The baseline
// ═══════════════════════════════════════════════════════════════════════════

TEST(InterchangeBaseline, MeasuresTheUnmodifiedNetwork)
{
    TwoLineNetwork net;
    const SurfaceMeans base = measure_baseline(net.inputs(), config_with(800.0));

    // Every station reaches the two others on its own line within 45 minutes and
    // cannot reach the other line at all, so the change budget buys nothing.
    EXPECT_DOUBLE_EQ(base.direct, 2.0);
    EXPECT_DOUBLE_EQ(base.full, 2.0);
    EXPECT_DOUBLE_EQ(base.gap, 0.0);
}

TEST(InterchangeBaseline, RejectsAThresholdIndexThatIsNotThere)
{
    TwoLineNetwork net;
    InterchangeSearchConfig cfg = config_with(800.0);
    cfg.threshold_index = 3; // only one threshold was configured
    EXPECT_THROW((void)measure_baseline(net.inputs(), cfg), std::invalid_argument);
}

// ═══════════════════════════════════════════════════════════════════════════
// Evaluation
// ═══════════════════════════════════════════════════════════════════════════

TEST(InterchangeSearch, JoiningTheTwoComponentsIsTheWinningCandidate)
{
    TwoLineNetwork net;
    const auto in = net.inputs();
    const auto cfg = config_with(800.0);

    const auto cands = generate_interchange_candidates(in, cfg);
    const auto ranked = evaluate_interchange_candidates(in, cands, cfg);
    ASSERT_EQ(ranked.size(), cands.size());

    // B <-> E is the connection that joins the network, and it is also the
    // shortest walk, so it must come first.
    EXPECT_EQ(ranked.front().candidate.station_a, 1u);
    EXPECT_EQ(ranked.front().candidate.station_b, 4u);
    EXPECT_GT(ranked.front().delta_reach, 0.0);

    // Reaching the far line needs a second vehicle, so the winning candidate
    // must open up more at the full change budget than at zero changes.
    EXPECT_GT(ranked.front().delta_reach, ranked.front().delta_direct);
    EXPECT_GT(ranked.front().delta_gap, 0.0);
}

TEST(InterchangeSearch, AFootpathNeverReducesReachability)
{
    // The invariant the whole method rests on. A footpath only ever adds an
    // option, and the transitive closure only ever shortens a walk, so no
    // candidate may make any measure worse. A regression in the closure, the
    // station merge or the round accounting would show up here first.
    TwoLineNetwork net;
    const auto in = net.inputs();
    const auto cfg = config_with(800.0);

    const auto ranked = evaluate_interchange_candidates(
        in, generate_interchange_candidates(in, cfg), cfg);
    ASSERT_FALSE(ranked.empty());

    for (const auto &e : ranked)
    {
        EXPECT_GE(e.delta_reach, -1e-9) << "a footpath made the network worse";
        EXPECT_GE(e.delta_direct, -1e-9) << "a footpath made the direct network worse";
        EXPECT_GE(e.after.full, e.after.direct) << "full budget must dominate zero changes";
    }
}

TEST(InterchangeSearch, EveryCandidateIsMeasuredAgainstTheSameBaseline)
{
    // If the station set were recomputed per candidate, the two stations being
    // joined would merge and `before` would drift between rows. It must not.
    TwoLineNetwork net;
    const auto in = net.inputs();
    const auto cfg = config_with(800.0);

    const auto ranked = evaluate_interchange_candidates(
        in, generate_interchange_candidates(in, cfg), cfg);
    ASSERT_GE(ranked.size(), 2u);

    for (const auto &e : ranked)
    {
        EXPECT_DOUBLE_EQ(e.before.direct, ranked.front().before.direct);
        EXPECT_DOUBLE_EQ(e.before.full, ranked.front().before.full);
        EXPECT_DOUBLE_EQ(e.before.gap, ranked.front().before.gap);
    }
}

TEST(InterchangeSearch, ResultsAreRankedByGainDescending)
{
    TwoLineNetwork net;
    const auto in = net.inputs();
    const auto cfg = config_with(800.0);

    const auto ranked = evaluate_interchange_candidates(
        in, generate_interchange_candidates(in, cfg), cfg);
    for (std::size_t i = 1; i < ranked.size(); ++i)
        EXPECT_GE(ranked[i - 1].delta_reach, ranked[i].delta_reach);
}

TEST(InterchangeSearch, DeltasAgreeWithTheBeforeAndAfterTheyWereComputedFrom)
{
    TwoLineNetwork net;
    const auto in = net.inputs();
    const auto cfg = config_with(800.0);

    const auto ranked = evaluate_interchange_candidates(
        in, generate_interchange_candidates(in, cfg), cfg);
    for (const auto &e : ranked)
    {
        EXPECT_NEAR(e.delta_reach, e.after.full - e.before.full, 1e-12);
        EXPECT_NEAR(e.delta_direct, e.after.direct - e.before.direct, 1e-12);
        EXPECT_NEAR(e.delta_gap, e.after.gap - e.before.gap, 1e-12);
        EXPECT_NEAR(e.before.gap, e.before.full - e.before.direct, 1e-12);
        EXPECT_NEAR(e.after.gap, e.after.full - e.after.direct, 1e-12);
    }
}

TEST(InterchangeSearch, NoCandidatesMeansNoWork)
{
    TwoLineNetwork net;
    const auto ranked = evaluate_interchange_candidates(net.inputs(), {}, config_with(800.0));
    EXPECT_TRUE(ranked.empty());
}
