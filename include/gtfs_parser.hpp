#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <optional>
#include <stdexcept>
#include <iostream>

/**
 * @file gtfs_parser.hpp
 * @brief GTFS feed ingestion, sanitization, and relational integrity enforcement.
 *
 * Analogous to a financial tick-data sanitization pipeline:
 *   - Drop malformed records (orphaned foreign keys) before they poison downstream state.
 *   - Interpolate missing intermediate timestamps geometrically.
 *   - Enforce O(1) FK lookups via unordered_set pre-load.
 *
 * Data flow:
 *   agency.txt → AgencyRecord
 *   routes.txt → RouteRecord        (FK: agency_id)
 *   trips.txt  → TripRecord         (FK: route_id)
 *   stops.txt  → StopRecord
 *   stop_times.txt → StopTimeRecord (FK: trip_id, stop_id)
 */

namespace namma_metro {

// ═══════════════════════════════════════════════════════════════════════════
// § 1.  Primary GTFS relational entity structs
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Represents one record from agency.txt
 *
 * GTFS fields: agency_id, agency_name, agency_url, agency_timezone
 * Regional extension: agency_lang (optional per BMRCL/BMTC feeds)
 */
struct AgencyRecord {
    std::string agency_id;      ///< Primary key (e.g., "BMRCL")
    std::string agency_name;    ///< Human-readable operator name
    std::string agency_url;     ///< Official website URL
    std::string agency_timezone;///< IANA timezone string (e.g., "Asia/Kolkata")
    std::string agency_lang;    ///< BCP-47 language code (optional, may be empty)
};

/**
 * @brief Represents one record from stops.txt
 *
 * Geographic coordinates are stored as doubles for full WGS-84 precision.
 * stop_id is mapped to a dense uint32_t node index during CSR construction.
 */
struct StopRecord {
    std::string stop_id;        ///< Primary key (string from GTFS)
    std::string stop_name;      ///< Human-readable station name (e.g., "Majestic")
    double      stop_lat;       ///< WGS-84 latitude  (range: -90  .. +90)
    double      stop_lon;       ///< WGS-84 longitude (range: -180 .. +180)
    std::string stop_desc;      ///< Optional description (may be empty)
    std::string zone_id;        ///< Fare zone (optional)
    uint8_t     location_type;  ///< 0=stop, 1=station, 2=entrance/exit
    std::string parent_station; ///< stop_id of parent station (optional)
};

/**
 * @brief Represents one record from routes.txt
 *
 * route_type codes relevant to Namma Metro:
 *   1 = Subway/Metro  (Purple Line / Green Line)
 *   3 = Bus           (BMTC)
 */
struct RouteRecord {
    std::string route_id;        ///< Primary key
    std::string agency_id;       ///< FK → AgencyRecord::agency_id
    std::string route_short_name;///< E.g., "Purple", "Green"
    std::string route_long_name; ///< E.g., "Challaghatta – Baiyappanahalli"
    uint8_t     route_type;      ///< GTFS route_type integer code
    std::string route_color;     ///< Hex colour string without '#' (e.g., "8B008B")
    std::string route_text_color;///< Foreground hex colour for labels
};

/**
 * @brief Represents one record from trips.txt
 */
struct TripRecord {
    std::string route_id;       ///< FK → RouteRecord::route_id
    std::string service_id;     ///< References calendar.txt / calendar_dates.txt
    std::string trip_id;        ///< Primary key
    std::string trip_headsign;  ///< Destination display text (e.g., "Baiyappanahalli")
    std::string shape_id;       ///< Optional reference to shapes.txt
    uint8_t     direction_id;   ///< 0 = outbound, 1 = inbound
};

/**
 * @brief Represents one record from stop_times.txt (after interpolation).
 *
 * GTFS legally permits blank arrival_time / departure_time for intermediate
 * stops. All such blanks are filled by geometric interpolation before any
 * record reaches graph construction. See @ref interpolate_stop_times().
 *
 * Times are stored as seconds-past-midnight (uint32_t) to support GTFS
 * post-24h timestamps (e.g., "25:15:00" = 90900 seconds).
 */
struct StopTimeRecord {
    std::string trip_id;         ///< FK → TripRecord::trip_id
    uint32_t    arrival_time;    ///< Seconds past midnight (post-interpolation, always set)
    uint32_t    departure_time;  ///< Seconds past midnight (post-interpolation, always set)
    std::string stop_id;         ///< FK → StopRecord::stop_id
    uint32_t    stop_sequence;   ///< 0-based ordering within the trip
    uint8_t     pickup_type;     ///< 0=regular, 1=no pickup, 2=phone agency, 3=coord w/ driver
    uint8_t     drop_off_type;   ///< Same encoding as pickup_type
};

// ═══════════════════════════════════════════════════════════════════════════
// § 2.  Time parsing utility
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Parse a GTFS time string into total seconds past midnight.
 *
 * Standard C++ time libraries (`std::get_time`, `strptime`) cap at 23:59:59
 * and either crash or silently overflow on GTFS post-midnight times such as
 * "25:15:00" or "26:30:00". These are NOT invalid data — they represent trips
 * that depart before midnight and terminate after it, encoded with a
 * monotonically increasing clock to preserve stop sequence ordering within a
 * single operational day.
 *
 * This function manually splits the string on ':' and computes:
 *   seconds = hours * 3600 + minutes * 60 + secs
 * where `hours` may legally be ≥ 24.
 *
 * @param time_str  GTFS time string in "HH:MM:SS" format.
 *                  Hours field may exceed 23 (e.g., "25:15:00" → 90900).
 * @return          Total seconds past midnight as uint32_t.
 *                  Max representable: 99:59:59 = 359999 s ≈ 4.16 days.
 *                  In practice GTFS values rarely exceed 28:00:00 = 100800 s.
 * @throws std::invalid_argument if the string is malformed (wrong delimiter
 *         count, non-numeric fields, or empty string).
 *
 * @example
 *   parse_gtfs_time("08:30:00") == 30600
 *   parse_gtfs_time("25:15:00") == 90900   // post-midnight trip
 *   parse_gtfs_time("00:00:00") == 0
 */
uint32_t parse_gtfs_time(const std::string& time_str);

// ═══════════════════════════════════════════════════════════════════════════
// § 3.  The GTFSParser class
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Stateful GTFS feed loader with referential integrity enforcement.
 *
 * Loading order is strictly defined to enable O(1) FK validation:
 *   1. load_agency()     → populates valid_agency_ids_
 *   2. load_stops()      → populates valid_stop_ids_
 *   3. load_routes()     → validates agency FK, populates valid_route_ids_
 *   4. load_trips()      → validates route FK, populates valid_trip_ids_
 *   5. load_stop_times() → validates trip + stop FKs, applies interpolation
 *
 * Orphaned records are dropped with a std::cerr diagnostic warning.
 * This is identical in philosophy to a financial pipeline dropping ticks
 * whose instrument_id cannot be found in the reference data master.
 */
class GTFSParser {
public:
    explicit GTFSParser(std::string data_directory);

    // ── Load methods (call in order) ──────────────────────────────────────
    void load_agency(const std::string& filename = "agency.txt");
    void load_stops(const std::string& filename = "stops.txt");
    void load_routes(const std::string& filename = "routes.txt");
    void load_trips(const std::string& filename = "trips.txt");
    void load_stop_times(const std::string& filename = "stop_times.txt");

    /**
     * @brief Load calendar.txt — defines which service_ids run on which weekdays.
     *
     * Some GTFS feeds use ONLY calendar_dates.txt and have no calendar.txt.
     * Call both load_calendar() and load_calendar_dates() — whichever file
     * exists will be parsed; the other is silently skipped.
     *
     * NOTE: Service day filtering is logged but not yet enforced on trip loading.
     * All trips are included regardless of service_id. A production system would
     * cross-reference trip.service_id against active service_ids for the query date.
     */
    void load_calendar(const std::string& filename = "calendar.txt");

    /**
     * @brief Load calendar_dates.txt — date-specific service exceptions.
     *
     * Modern GTFS feeds increasingly use ONLY calendar_dates.txt. If calendar.txt
     * is absent but calendar_dates.txt exists, this is the sole service schedule.
     */
    void load_calendar_dates(const std::string& filename = "calendar_dates.txt");

    /**
     * @brief Check whether frequencies.txt exists and warn loudly if so.
     *
     * frequencies.txt is NOT currently parsed. If BMRCL uses frequency-based
     * scheduling, the graph will be missing edges. Call this after load_agency()
     * to detect the issue early before graph construction.
     */
    void check_frequencies(const std::string& filename = "frequencies.txt") const;

    // ── Geometric interpolation ───────────────────────────────────────────
    /**
     * @brief Fill blank arrival/departure times on intermediate stops.
     *
     * The GTFS spec (§ stop_times.txt) permits NULL times for intermediate
     * stops provided the first and last stops have defined times. Blank
     * entries are represented here as UINT32_MAX (sentinel) after the
     * initial CSV parse.
     *
     * Algorithm:
     *   For each trip, scan the sorted stop-time sequence. When a sentinel
     *   is encountered, identify the nearest preceding and succeeding defined
     *   timepoints [prev, next]. Compute the Haversine arc-distance sum
     *   across the gap. Distribute time linearly proportional to cumulative
     *   distance from `prev`.
     *
     * This guarantees that every StopTimeRecord reaching graph construction
     * has a valid, non-sentinel arrival_time and departure_time.
     *
     * @pre  load_stop_times() and load_stops() must have been called.
     * @post No StopTimeRecord in stop_times_ has arrival_time == UINT32_MAX.
     */
    void interpolate_stop_times();

    // ── Accessors ─────────────────────────────────────────────────────────
    [[nodiscard]] const std::vector<AgencyRecord>&   agencies()   const { return agencies_;   }
    [[nodiscard]] const std::vector<StopRecord>&     stops()      const { return stops_;      }
    [[nodiscard]] const std::vector<RouteRecord>&    routes()     const { return routes_;     }
    [[nodiscard]] const std::vector<TripRecord>&     trips()      const { return trips_;      }
    [[nodiscard]] const std::vector<StopTimeRecord>& stop_times() const { return stop_times_; }

    /// Dense stop_id string → uint32_t node index mapping (built after load_stops)
    [[nodiscard]] const std::unordered_map<std::string, uint32_t>& stop_index_map() const {
        return stop_index_map_;
    }

    // ── Diagnostics ───────────────────────────────────────────────────────
    struct ParseStats {
        uint64_t rows_read       = 0;
        uint64_t rows_accepted   = 0;
        uint64_t rows_dropped_fk = 0;  ///< Dropped due to orphaned FK
        uint64_t rows_dropped_fmt= 0;  ///< Dropped due to malformed fields
        uint64_t times_interpolated = 0;
    };

    [[nodiscard]] const ParseStats& stats() const { return stats_; }
    void print_stats() const;

private:
    std::string data_dir_;

    std::vector<AgencyRecord>   agencies_;
    std::vector<StopRecord>     stops_;
    std::vector<RouteRecord>    routes_;
    std::vector<TripRecord>     trips_;
    std::vector<StopTimeRecord> stop_times_;

    /// Pre-loaded validated primary key sets for O(1) FK lookup
    std::unordered_set<std::string> valid_agency_ids_;
    std::unordered_set<std::string> valid_stop_ids_;
    std::unordered_set<std::string> valid_route_ids_;
    std::unordered_set<std::string> valid_trip_ids_;

    /// Dense stop index assignment (populated during load_stops)
    std::unordered_map<std::string, uint32_t> stop_index_map_;

    ParseStats stats_;

    // ── Internal helpers ──────────────────────────────────────────────────
    std::string resolve_path(const std::string& filename) const;
    static std::vector<std::string> split_csv_line(const std::string& line);
    static double haversine_m(double lat1, double lon1, double lat2, double lon2);
};

} // namespace namma_metro
