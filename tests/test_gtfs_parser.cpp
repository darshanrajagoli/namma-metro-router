#include <gtest/gtest.h>
#include "gtfs_parser.hpp"
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

/**
 * @file test_gtfs_parser.cpp
 * @brief Google Test suite for parse_gtfs_time() and the GTFSParser pipeline.
 *
 * PURPOSE:
 *   Two layers are covered.
 *
 *   (1) parse_gtfs_time() — the time parser that, unlike std::get_time, must
 *       accept GTFS post-midnight hours (e.g. "25:15:00" = 90900s) and reject
 *       malformed strings with std::invalid_argument.
 *
 *   (2) GTFSParser — the relational loader. Tests write small fixture feeds to a
 *       temp directory and verify:
 *         • the dense stop_id → index map is assigned in load order,
 *         • foreign-key validation drops orphaned stop_times,
 *         • geometric interpolation fills blank intermediate timestamps,
 *         • a missing file raises std::runtime_error.
 *
 *   The fixture writes only the files each test needs, so the cases stay
 *   independent and the expected diagnostics on std::cerr are localised.
 */

using namespace namma_metro;

// ═══════════════════════════════════════════════════════════════════════════
// Part 1 — parse_gtfs_time (pure function, no I/O)
// ═══════════════════════════════════════════════════════════════════════════

TEST(ParseGtfsTime, StandardTimesWithinDay) {
    EXPECT_EQ(parse_gtfs_time("00:00:00"), 0u);
    EXPECT_EQ(parse_gtfs_time("08:30:00"), 30600u);
    EXPECT_EQ(parse_gtfs_time("12:00:00"), 43200u);
    EXPECT_EQ(parse_gtfs_time("23:59:59"), 86399u);
}

TEST(ParseGtfsTime, PostMidnightHoursAreAccepted) {
    // The whole reason this function exists: hours may legally exceed 23.
    EXPECT_EQ(parse_gtfs_time("24:00:00"), 86400u);
    EXPECT_EQ(parse_gtfs_time("25:15:00"), 90900u);
    EXPECT_EQ(parse_gtfs_time("26:30:00"), 95400u);
}

TEST(ParseGtfsTime, MalformedStringsThrow) {
    EXPECT_THROW(parse_gtfs_time(""), std::invalid_argument);        // empty
    EXPECT_THROW(parse_gtfs_time("0830"), std::invalid_argument);    // no colon
    EXPECT_THROW(parse_gtfs_time("08:30"), std::invalid_argument);   // one colon
    EXPECT_THROW(parse_gtfs_time("ab:cd:ef"), std::invalid_argument);// non-numeric
    EXPECT_THROW(parse_gtfs_time("08:75:00"), std::invalid_argument);// minutes >= 60
    EXPECT_THROW(parse_gtfs_time("08:30:99"), std::invalid_argument);// seconds >= 60
}

// ═══════════════════════════════════════════════════════════════════════════
// Part 2 — GTFSParser (fixture-backed, writes temp GTFS files)
// ═══════════════════════════════════════════════════════════════════════════

class GTFSParserTest : public ::testing::Test {
protected:
    std::filesystem::path dir_;

    void SetUp() override {
        const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
        dir_ = std::filesystem::temp_directory_path() /
               (std::string("namma_gtfs_") + info->test_suite_name() + "_" + info->name());
        std::error_code ec;
        std::filesystem::remove_all(dir_, ec); // clear any stale leftovers
        std::filesystem::create_directories(dir_);
    }

    void TearDown() override {
        std::error_code ec;
        std::filesystem::remove_all(dir_, ec);
    }

    void write_file(const std::string& name, const std::string& content) {
        std::ofstream f(dir_ / name);
        ASSERT_TRUE(f.is_open()) << "Failed to write fixture " << name;
        f << content;
    }

    std::string data_dir() const { return dir_.string(); }

    // A minimal, internally consistent feed: 1 agency, 3 stops, 1 route, 1 trip.
    // stop_times is written separately by each test so the time data can vary.
    void write_base_feed() {
        write_file("agency.txt",
            "agency_id,agency_name,agency_url,agency_timezone\n"
            "BMRCL,Namma Metro,https://example.com,Asia/Kolkata\n");
        write_file("stops.txt",
            "stop_id,stop_name,stop_lat,stop_lon,stop_desc\n"
            "S0,Stop Zero,12.9716,77.5946,\n"
            "S1,Stop One,12.9750,77.6000,\n"
            "S2,Stop Two,12.9800,77.6050,\n");
        write_file("routes.txt",
            "route_id,agency_id,route_short_name,route_long_name,route_type\n"
            "R0,BMRCL,Purple,Purple Line,1\n");
        write_file("trips.txt",
            "route_id,service_id,trip_id,trip_headsign\n"
            "R0,WD,T0,Eastbound\n");
    }

    // Load the relational core in the mandated order.
    void load_core(GTFSParser& p) {
        p.load_agency();
        p.load_stops();
        p.load_routes();
        p.load_trips();
    }
};

TEST_F(GTFSParserTest, LoadsRelationalEntities) {
    write_base_feed();
    write_file("stop_times.txt",
        "trip_id,arrival_time,departure_time,stop_id,stop_sequence\n"
        "T0,08:00:00,08:00:00,S0,0\n"
        "T0,08:05:00,08:05:00,S1,1\n"
        "T0,08:10:00,08:10:00,S2,2\n");

    GTFSParser p(data_dir());
    load_core(p);
    p.load_stop_times();

    EXPECT_EQ(p.agencies().size(), 1u);
    EXPECT_EQ(p.stops().size(), 3u);
    EXPECT_EQ(p.routes().size(), 1u);
    EXPECT_EQ(p.trips().size(), 1u);
    EXPECT_EQ(p.stop_times().size(), 3u);
}

TEST_F(GTFSParserTest, StopIndexMap_IsDenseInLoadOrder) {
    write_base_feed();
    GTFSParser p(data_dir());
    p.load_agency();
    p.load_stops();

    const auto& idx = p.stop_index_map();
    ASSERT_EQ(idx.size(), 3u);
    EXPECT_EQ(idx.at("S0"), 0u);
    EXPECT_EQ(idx.at("S1"), 1u);
    EXPECT_EQ(idx.at("S2"), 2u);
}

TEST_F(GTFSParserTest, ForeignKey_DropsOrphanedStopTime) {
    write_base_feed();
    // The middle row references a trip_id that was never declared in trips.txt.
    write_file("stop_times.txt",
        "trip_id,arrival_time,departure_time,stop_id,stop_sequence\n"
        "T0,08:00:00,08:00:00,S0,0\n"
        "GHOST,08:05:00,08:05:00,S1,1\n"   // orphaned trip_id → dropped
        "T0,08:10:00,08:10:00,S2,2\n");

    GTFSParser p(data_dir());
    load_core(p);
    p.load_stop_times();

    EXPECT_EQ(p.stop_times().size(), 2u)
        << "The stop_time with an orphaned trip_id must be dropped";
    EXPECT_GE(p.stats().rows_dropped_fk, 1u)
        << "Dropping an orphaned FK must be recorded in ParseStats";
}

TEST_F(GTFSParserTest, ForeignKey_DropsOrphanedStopId) {
    write_base_feed();
    write_file("stop_times.txt",
        "trip_id,arrival_time,departure_time,stop_id,stop_sequence\n"
        "T0,08:00:00,08:00:00,S0,0\n"
        "T0,08:05:00,08:05:00,S_NOPE,1\n"  // orphaned stop_id → dropped
        "T0,08:10:00,08:10:00,S2,2\n");

    GTFSParser p(data_dir());
    load_core(p);
    p.load_stop_times();

    EXPECT_EQ(p.stop_times().size(), 2u)
        << "The stop_time with an orphaned stop_id must be dropped";
    EXPECT_GE(p.stats().rows_dropped_fk, 1u);
}

TEST_F(GTFSParserTest, Interpolation_FillsBlankIntermediateTimes) {
    write_base_feed();
    // S1 has blank arrival/departure — must be geometrically interpolated
    // between S0 (08:00:00 = 28800) and S2 (08:10:00 = 29400).
    write_file("stop_times.txt",
        "trip_id,arrival_time,departure_time,stop_id,stop_sequence\n"
        "T0,08:00:00,08:00:00,S0,0\n"
        "T0,,,S1,1\n"
        "T0,08:10:00,08:10:00,S2,2\n");

    GTFSParser p(data_dir());
    load_core(p);
    p.load_stop_times();
    p.interpolate_stop_times();

    // After interpolation, no record may retain the blank sentinel.
    for (const auto& r : p.stop_times()) {
        EXPECT_NE(r.arrival_time, UINT32_MAX) << "Blank time left un-interpolated";
        EXPECT_NE(r.departure_time, UINT32_MAX);
    }

    // Locate the (now-filled) S1 record and check it lies strictly inside the gap.
    uint32_t s1_time = 0;
    bool found = false;
    for (const auto& r : p.stop_times()) {
        if (r.stop_id == "S1") { s1_time = r.arrival_time; found = true; }
    }
    ASSERT_TRUE(found);
    EXPECT_GT(s1_time, 28800u)
        << "Interpolated S1 time must be after S0's departure";
    EXPECT_LT(s1_time, 29400u)
        << "Interpolated S1 time must be before S2's arrival";
    EXPECT_GE(p.stats().times_interpolated, 1u)
        << "An interpolation event must be recorded in ParseStats";
}

TEST_F(GTFSParserTest, MissingFile_ThrowsRuntimeError) {
    // No files written — load must fail with a clear error rather than crash.
    GTFSParser p(data_dir());
    EXPECT_THROW(p.load_agency(), std::runtime_error);
}
