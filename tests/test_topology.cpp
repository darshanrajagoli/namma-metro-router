#include <gtest/gtest.h>
#include "gtfs_parser.hpp"
#include "topology.hpp"
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @file test_topology.cpp
 * @brief Contracts for the structural metrics the multi-feed study correlates on.
 *
 * The cyclomatic number is the load-bearing one. The whole tree hypothesis —
 * "Namma Metro has no trade-offs because it is a tree" — is the claim that
 * mu = L - S + C is zero, so a test suite that does not pin mu on hand-built
 * trees and hand-built cycles is not testing the hypothesis at all.
 *
 * The second thing worth pinning is the platform-to-station merge. On a feed
 * that keeps platforms separate, every interchange is a little clique joined by
 * transfer edges. Measuring topology on platforms would count those cliques as
 * cycles and report a mesh where the city has a tree — an artefact of the
 * encoding masquerading as a property of the network.
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

    TripRecord trip_on(std::string trip_id, std::string route_id)
    {
        TripRecord t;
        t.trip_id = std::move(trip_id);
        t.route_id = std::move(route_id);
        t.service_id = "WEEKDAY";
        t.direction_id = 0;
        return t;
    }

    TransferRecord tr(std::string a, std::string b, uint32_t s)
    {
        TransferRecord t;
        t.from_stop_id = std::move(a);
        t.to_stop_id = std::move(b);
        t.min_transfer_time = s;
        return t;
    }

    const std::unordered_map<std::string, uint32_t> kIdx = {
        {"A", 0}, {"B", 1}, {"C", 2}, {"D", 3}, {"E", 4}, {"F", 5}};

    /// A -> B -> C at a fixed headway, so links have measurable service.
    std::vector<StopTimeRecord> line(const std::string &prefix,
                                     const std::vector<std::string> &stops,
                                     uint32_t first_dep, uint32_t headway, uint32_t runs)
    {
        std::vector<StopTimeRecord> out;
        for (uint32_t r = 0; r < runs; ++r)
        {
            uint32_t t = first_dep + r * headway;
            for (std::size_t i = 0; i < stops.size(); ++i)
            {
                out.push_back(st(prefix + "_" + std::to_string(r), t, t, stops[i],
                                 static_cast<uint32_t>(i)));
                t += 300;
            }
        }
        return out;
    }

} // namespace

// A single line is a path: L = S - 1, one component, so mu = 0.
TEST(Topology, SingleLineIsATreeWithZeroCyclomaticNumber)
{
    auto idx = kIdx;
    auto stop_times = line("L1", {"A", "B", "C"}, 28800, 300, 4);
    std::vector<TripRecord> trips;
    for (uint32_t r = 0; r < 4; ++r)
        trips.push_back(trip_on("L1_" + std::to_string(r), "PURPLE"));

    const auto m = compute_topology(stop_times, trips, 6, &idx, {});
    EXPECT_EQ(m.stations, 3u);
    EXPECT_EQ(m.links, 2u);
    EXPECT_EQ(m.components, 1u);
    EXPECT_EQ(m.cyclomatic, 0);
    EXPECT_EQ(m.lines, 1u);
    EXPECT_EQ(m.deg1_stations, 2u); // the two termini
    EXPECT_EQ(m.deg2_stations, 1u);
    EXPECT_EQ(m.interchange_stations, 0u);
}

// Two lines crossing at one station is still a tree: four links, five stations,
// one component. This is the Namma Metro shape and it is why the frontier is
// degenerate there.
TEST(Topology, TwoLinesCrossingAtOneStationIsStillATree)
{
    auto idx = kIdx;
    auto stop_times = line("L1", {"A", "C", "B"}, 28800, 300, 3);
    auto second = line("L2", {"D", "C", "E"}, 28900, 300, 3);
    stop_times.insert(stop_times.end(), second.begin(), second.end());

    std::vector<TripRecord> trips;
    for (uint32_t r = 0; r < 3; ++r)
    {
        trips.push_back(trip_on("L1_" + std::to_string(r), "PURPLE"));
        trips.push_back(trip_on("L2_" + std::to_string(r), "GREEN"));
    }

    const auto m = compute_topology(stop_times, trips, 6, &idx, {});
    EXPECT_EQ(m.stations, 5u);
    EXPECT_EQ(m.links, 4u);
    EXPECT_EQ(m.cyclomatic, 0);
    EXPECT_EQ(m.lines, 2u);
    EXPECT_EQ(m.interchange_stations, 1u); // C carries both lines
    EXPECT_EQ(m.max_station_degree, 4u);
}

// Close the loop and the cyclomatic number becomes exactly one. This is the
// structural difference between a network with alternatives and one without.
TEST(Topology, ClosingALoopGivesCyclomaticNumberOne)
{
    auto idx = kIdx;
    auto stop_times = line("L1", {"A", "B", "C"}, 28800, 300, 3);
    auto back = line("L2", {"C", "D", "A"}, 28900, 300, 3);
    stop_times.insert(stop_times.end(), back.begin(), back.end());

    std::vector<TripRecord> trips;
    for (uint32_t r = 0; r < 3; ++r)
    {
        trips.push_back(trip_on("L1_" + std::to_string(r), "R1"));
        trips.push_back(trip_on("L2_" + std::to_string(r), "R2"));
    }

    const auto m = compute_topology(stop_times, trips, 6, &idx, {});
    EXPECT_EQ(m.stations, 4u);
    EXPECT_EQ(m.links, 4u);
    EXPECT_EQ(m.components, 1u);
    EXPECT_EQ(m.cyclomatic, 1);
}

// Two disconnected lines: mu = L - S + C must use C = 2, not 1. Getting the
// component count wrong turns two trees into a spurious cycle.
TEST(Topology, DisconnectedComponentsAreCountedSoTheForestStaysAForest)
{
    auto idx = kIdx;
    auto stop_times = line("L1", {"A", "B"}, 28800, 300, 3);
    auto other = line("L2", {"C", "D"}, 28900, 300, 3);
    stop_times.insert(stop_times.end(), other.begin(), other.end());

    std::vector<TripRecord> trips;
    for (uint32_t r = 0; r < 3; ++r)
    {
        trips.push_back(trip_on("L1_" + std::to_string(r), "R1"));
        trips.push_back(trip_on("L2_" + std::to_string(r), "R2"));
    }

    const auto m = compute_topology(stop_times, trips, 6, &idx, {});
    EXPECT_EQ(m.stations, 4u);
    EXPECT_EQ(m.links, 2u);
    EXPECT_EQ(m.components, 2u);
    EXPECT_EQ(m.cyclomatic, 0);
}

// The encoding artefact this merge exists to prevent: platforms A and B are one
// station joined by a transfer. Without the merge the network has an extra
// station and the interchange looks like structure it is not.
TEST(Topology, PlatformsOfOneStationAreMergedBeforeMeasuring)
{
    auto idx = kIdx;
    // Line 1 serves platform A; line 2 serves platform B; A and B are the same
    // physical station. C and D are the far ends.
    auto stop_times = line("L1", {"C", "A"}, 28800, 300, 3);
    auto second = line("L2", {"B", "D"}, 28900, 300, 3);
    stop_times.insert(stop_times.end(), second.begin(), second.end());

    std::vector<TripRecord> trips;
    for (uint32_t r = 0; r < 3; ++r)
    {
        trips.push_back(trip_on("L1_" + std::to_string(r), "R1"));
        trips.push_back(trip_on("L2_" + std::to_string(r), "R2"));
    }
    const std::vector<TransferRecord> transfers = {tr("A", "B", 120), tr("B", "A", 120)};

    const auto merged = compute_topology(stop_times, trips, 6, &idx, transfers);
    EXPECT_EQ(merged.stations, 3u); // C, {A,B}, D
    EXPECT_EQ(merged.links, 2u);
    EXPECT_EQ(merged.components, 1u);
    EXPECT_EQ(merged.cyclomatic, 0);
    EXPECT_EQ(merged.interchange_stations, 1u);

    // Without the transfer records the same feed looks like two disconnected
    // fragments — which is exactly the wrong picture, and why the merge matters.
    const auto unmerged = compute_topology(stop_times, trips, 6, &idx, {});
    EXPECT_EQ(unmerged.stations, 4u);
    EXPECT_EQ(unmerged.components, 2u);
}

// Two lines sharing a stretch of track is the OTHER way an alternative can
// exist, and it is invisible to the cyclomatic number.
TEST(Topology, SharedTrackIsReportedAsRouteOverlap)
{
    auto idx = kIdx;
    auto stop_times = line("L1", {"A", "B", "C"}, 28800, 600, 3);
    auto second = line("L2", {"A", "B", "D"}, 28900, 600, 3);
    stop_times.insert(stop_times.end(), second.begin(), second.end());

    std::vector<TripRecord> trips;
    for (uint32_t r = 0; r < 3; ++r)
    {
        trips.push_back(trip_on("L1_" + std::to_string(r), "R1"));
        trips.push_back(trip_on("L2_" + std::to_string(r), "R2"));
    }

    const auto m = compute_topology(stop_times, trips, 6, &idx, {});
    EXPECT_EQ(m.links, 3u);                  // A-B, B-C, B-D
    EXPECT_NEAR(m.shared_link_fraction, 1.0 / 3.0, 1e-9); // only A-B is shared
    EXPECT_NEAR(m.mean_lines_per_link, 4.0 / 3.0, 1e-9);
}

// Most feeds publish each direction as its own route_id. Counting those
// directly makes every station on a single line look like an interchange, which
// is how BART's interchange density came out at exactly 1.000 before this.
TEST(Topology, OppositeDirectionsOfOneLineAreNotTwoLines)
{
    auto idx = kIdx;
    auto stop_times = line("OUT", {"A", "B", "C"}, 28800, 600, 3);
    auto back = line("IN", {"C", "B", "A"}, 28900, 600, 3);
    stop_times.insert(stop_times.end(), back.begin(), back.end());

    std::vector<TripRecord> trips;
    for (uint32_t r = 0; r < 3; ++r)
    {
        trips.push_back(trip_on("OUT_" + std::to_string(r), "PURPLE_OUTBOUND"));
        trips.push_back(trip_on("IN_" + std::to_string(r), "PURPLE_INBOUND"));
    }

    const auto m = compute_topology(stop_times, trips, 6, &idx, {});
    EXPECT_EQ(m.lines, 1u) << "two route_ids over the same stations are one line";
    EXPECT_EQ(m.interchange_stations, 0u);
    EXPECT_DOUBLE_EQ(m.interchange_density, 0.0);
    EXPECT_DOUBLE_EQ(m.mean_lines_per_link, 1.0);
    EXPECT_DOUBLE_EQ(m.shared_link_fraction, 0.0);
}

// Headway is measured inside the 07:00-21:00 window the benchmark samples, so a
// thin overnight tail cannot flatter a network's service level.
TEST(Topology, HeadwayIsTheMedianGapInsideTheSamplingWindow)
{
    auto idx = kIdx;
    auto stop_times = line("L1", {"A", "B"}, 28800, 600, 10); // every 10 minutes
    std::vector<TripRecord> trips;
    for (uint32_t r = 0; r < 10; ++r)
        trips.push_back(trip_on("L1_" + std::to_string(r), "R1"));

    const auto m = compute_topology(stop_times, trips, 6, &idx, {});
    EXPECT_NEAR(m.median_link_headway_s, 600.0, 1e-9);
    EXPECT_EQ(m.trips, 10u);
}

// Headway is measured per directed PLATFORM link. Two lines using DIFFERENT
// platforms of the same station pair must not have their departures merged:
// that is what made BART report a median headway of zero seconds.
TEST(Topology, HeadwayIsNotHalvedByMergingSeparatePlatforms)
{
    auto idx = kIdx;
    // A/B are the two platforms of one station; C/D the two of another.
    // One service runs A->C, the other B->D, both every 10 minutes and offset
    // by five, so a station-level merge would see a 5-minute headway.
    auto stop_times = line("N", {"A", "C"}, 28800, 600, 8);
    auto south = line("S", {"B", "D"}, 29100, 600, 8);
    stop_times.insert(stop_times.end(), south.begin(), south.end());

    std::vector<TripRecord> trips;
    for (uint32_t r = 0; r < 8; ++r)
    {
        trips.push_back(trip_on("N_" + std::to_string(r), "R1"));
        trips.push_back(trip_on("S_" + std::to_string(r), "R2"));
    }
    const std::vector<TransferRecord> transfers = {
        tr("A", "B", 90), tr("B", "A", 90), tr("C", "D", 90), tr("D", "C", 90)};

    const auto m = compute_topology(stop_times, trips, 6, &idx, transfers);
    EXPECT_EQ(m.stations, 2u);
    EXPECT_EQ(m.links, 1u);
    EXPECT_NEAR(m.median_link_headway_s, 600.0, 1e-9)
        << "merging the two platform links would report 300s";
}

TEST(Topology, MissingIndexMapThrows)
{
    auto stop_times = line("L1", {"A", "B"}, 28800, 300, 2);
    EXPECT_THROW({ auto ignored = compute_topology(stop_times, {}, 6, nullptr, {}); (void)ignored; },
                 std::invalid_argument);
}

// A feed with no trips.txt still yields correct structure; only the line-based
// metrics go to zero. The study must not silently drop such a feed.
TEST(Topology, MissingTripsTableStillYieldsStructure)
{
    auto idx = kIdx;
    auto stop_times = line("L1", {"A", "B", "C"}, 28800, 300, 2);
    const auto m = compute_topology(stop_times, {}, 6, &idx, {});
    EXPECT_EQ(m.stations, 3u);
    EXPECT_EQ(m.links, 2u);
    EXPECT_EQ(m.lines, 0u);
    EXPECT_EQ(m.interchange_stations, 0u);
}

// The station graph handed to the renderer must be the one the metrics describe.
TEST(Topology, ExportedStationGraphMatchesTheMetrics)
{
    auto idx = kIdx;
    auto stop_times = line("L1", {"A", "B", "C"}, 28800, 300, 2);
    StationGraph sg;
    const auto m = compute_topology(stop_times, {}, 6, &idx, {}, &sg);

    EXPECT_EQ(sg.links.size(), m.links);
    EXPECT_EQ(sg.stations.size(), m.stations);
    EXPECT_EQ(sg.station_of.size(), 6u);
    for (const auto &e : sg.links)
        EXPECT_LT(e.first, e.second) << "links must be stored with first < second";
}

// The CSV row must line up with the header, or every downstream analysis reads
// the wrong column while looking perfectly healthy.
TEST(Topology, CsvRowFieldCountMatchesTheHeader)
{
    auto idx = kIdx;
    auto stop_times = line("L1", {"A", "B"}, 28800, 300, 2);
    const auto m = compute_topology(stop_times, {}, 6, &idx, {});

    const std::string h = topology_csv_header();
    const std::string r = topology_csv_row(m);
    EXPECT_EQ(std::count(h.begin(), h.end(), ','), std::count(r.begin(), r.end(), ','))
        << "header: " << h << "\nrow:    " << r;
}
