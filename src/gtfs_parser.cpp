#include "gtfs_parser.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <cassert>

namespace namma_metro {

// ═══════════════════════════════════════════════════════════════════════════
// parse_gtfs_time — handles post-24h GTFS timestamps
// ═══════════════════════════════════════════════════════════════════════════
uint32_t parse_gtfs_time(const std::string& time_str) {
    if (time_str.empty()) {
        throw std::invalid_argument("parse_gtfs_time: empty string");
    }

    // Split on ':' — expect exactly "HH:MM:SS"
    const auto c1 = time_str.find(':');
    if (c1 == std::string::npos) {
        throw std::invalid_argument("parse_gtfs_time: missing first colon in '" + time_str + "'");
    }
    const auto c2 = time_str.find(':', c1 + 1);
    if (c2 == std::string::npos) {
        throw std::invalid_argument("parse_gtfs_time: missing second colon in '" + time_str + "'");
    }

    const uint32_t hours   = static_cast<uint32_t>(std::stoul(time_str.substr(0, c1)));
    const uint32_t minutes = static_cast<uint32_t>(std::stoul(time_str.substr(c1 + 1, c2 - c1 - 1)));
    const uint32_t seconds = static_cast<uint32_t>(std::stoul(time_str.substr(c2 + 1)));

    if (minutes >= 60 || seconds >= 60) {
        throw std::invalid_argument("parse_gtfs_time: invalid minutes/seconds in '" + time_str + "'");
    }

    return hours * 3600u + minutes * 60u + seconds;
}

// ═══════════════════════════════════════════════════════════════════════════
// GTFSParser implementation
// ═══════════════════════════════════════════════════════════════════════════

GTFSParser::GTFSParser(std::string data_directory)
    : data_dir_(std::move(data_directory)) {}

std::string GTFSParser::resolve_path(const std::string& filename) const {
    return data_dir_ + "/" + filename;
}

std::vector<std::string> GTFSParser::split_csv_line(const std::string& line) {
    std::vector<std::string> fields;
    std::string field;
    bool in_quotes = false;

    for (char c : line) {
        if (c == '"') {
            in_quotes = !in_quotes;
        } else if (c == ',' && !in_quotes) {
            // Trim trailing \r (Windows line endings)
            if (!field.empty() && field.back() == '\r') field.pop_back();
            fields.push_back(field);
            field.clear();
        } else {
            field += c;
        }
    }
    if (!field.empty() && field.back() == '\r') field.pop_back();
    fields.push_back(field);
    return fields;
}

double GTFSParser::haversine_m(double lat1, double lon1, double lat2, double lon2) {
    constexpr double R = 6371000.0; // Earth radius in metres
    constexpr double DEG = M_PI / 180.0;
    const double dlat = (lat2 - lat1) * DEG;
    const double dlon = (lon2 - lon1) * DEG;
    const double a = std::sin(dlat/2)*std::sin(dlat/2)
                   + std::cos(lat1*DEG)*std::cos(lat2*DEG)
                   * std::sin(dlon/2)*std::sin(dlon/2);
    return 2.0 * R * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

// ── load_agency ──────────────────────────────────────────────────────────

void GTFSParser::load_agency(const std::string& filename) {
    std::ifstream f(resolve_path(filename));
    if (!f.is_open()) {
        throw std::runtime_error("GTFSParser: cannot open " + resolve_path(filename));
    }

    std::string header_line, line;
    std::getline(f, header_line); // skip header

    while (std::getline(f, line)) {
        if (line.empty() || line == "\r") continue;
        ++stats_.rows_read;
        auto fields = split_csv_line(line);
        if (fields.size() < 4) {
            ++stats_.rows_dropped_fmt;
            std::cerr << "[GTFS WARN] agency.txt: malformed row (< 4 fields): " << line << "\n";
            continue;
        }

        AgencyRecord r;
        r.agency_id       = fields[0];
        r.agency_name     = fields[1];
        r.agency_url      = fields[2];
        r.agency_timezone = fields[3];
        if (fields.size() > 4) r.agency_lang = fields[4];

        valid_agency_ids_.insert(r.agency_id);
        agencies_.push_back(std::move(r));
        ++stats_.rows_accepted;
    }
}

// ── load_stops ───────────────────────────────────────────────────────────

void GTFSParser::load_stops(const std::string& filename) {
    std::ifstream f(resolve_path(filename));
    if (!f.is_open()) {
        throw std::runtime_error("GTFSParser: cannot open " + resolve_path(filename));
    }

    std::string header_line, line;
    std::getline(f, header_line);

    uint32_t dense_idx = 0;
    while (std::getline(f, line)) {
        if (line.empty() || line == "\r") continue;
        ++stats_.rows_read;
        auto fields = split_csv_line(line);
        if (fields.size() < 5) {
            ++stats_.rows_dropped_fmt;
            std::cerr << "[GTFS WARN] stops.txt: malformed row: " << line << "\n";
            continue;
        }

        StopRecord r;
        r.stop_id   = fields[0];
        r.stop_name = fields[1];
        try {
            r.stop_lat = std::stod(fields[2]);
            r.stop_lon = std::stod(fields[3]);
        } catch (...) {
            ++stats_.rows_dropped_fmt;
            std::cerr << "[GTFS WARN] stops.txt: invalid coordinates for stop_id=" << fields[0] << "\n";
            continue;
        }
        r.stop_desc       = (fields.size() > 4) ? fields[4] : "";
        r.zone_id         = (fields.size() > 5) ? fields[5] : "";
        r.location_type   = (fields.size() > 6 && !fields[6].empty()) ? static_cast<uint8_t>(std::stoi(fields[6])) : 0;
        r.parent_station  = (fields.size() > 7) ? fields[7] : "";

        stop_index_map_[r.stop_id] = dense_idx++;
        valid_stop_ids_.insert(r.stop_id);
        stops_.push_back(std::move(r));
        ++stats_.rows_accepted;
    }
}

// ── load_routes ──────────────────────────────────────────────────────────

void GTFSParser::load_routes(const std::string& filename) {
    std::ifstream f(resolve_path(filename));
    if (!f.is_open()) {
        throw std::runtime_error("GTFSParser: cannot open " + resolve_path(filename));
    }

    std::string header_line, line;
    std::getline(f, header_line);

    while (std::getline(f, line)) {
        if (line.empty() || line == "\r") continue;
        ++stats_.rows_read;
        auto fields = split_csv_line(line);
        if (fields.size() < 5) { ++stats_.rows_dropped_fmt; continue; }

        RouteRecord r;
        r.route_id         = fields[0];
        r.agency_id        = fields[1];
        r.route_short_name = fields[2];
        r.route_long_name  = fields[3];
        try { r.route_type = static_cast<uint8_t>(std::stoi(fields[4])); }
        catch (...) { ++stats_.rows_dropped_fmt; continue; }

        // FK validation: agency_id must exist
        if (!valid_agency_ids_.empty() && valid_agency_ids_.find(r.agency_id) == valid_agency_ids_.end()) {
            ++stats_.rows_dropped_fk;
            std::cerr << "[GTFS WARN] routes.txt: orphaned agency_id='" << r.agency_id
                      << "' for route_id='" << r.route_id << "' — row dropped\n";
            continue;
        }

        r.route_color      = (fields.size() > 5) ? fields[5] : "";
        r.route_text_color = (fields.size() > 6) ? fields[6] : "";

        valid_route_ids_.insert(r.route_id);
        routes_.push_back(std::move(r));
        ++stats_.rows_accepted;
    }
}

// ── load_trips ───────────────────────────────────────────────────────────

void GTFSParser::load_trips(const std::string& filename) {
    std::ifstream f(resolve_path(filename));
    if (!f.is_open()) {
        throw std::runtime_error("GTFSParser: cannot open " + resolve_path(filename));
    }

    std::string header_line, line;
    std::getline(f, header_line);

    while (std::getline(f, line)) {
        if (line.empty() || line == "\r") continue;
        ++stats_.rows_read;
        auto fields = split_csv_line(line);
        if (fields.size() < 3) { ++stats_.rows_dropped_fmt; continue; }

        TripRecord r;
        r.route_id   = fields[0];
        r.service_id = fields[1];
        r.trip_id    = fields[2];

        // FK validation: route_id must exist
        if (valid_route_ids_.find(r.route_id) == valid_route_ids_.end()) {
            ++stats_.rows_dropped_fk;
            std::cerr << "[GTFS WARN] trips.txt: orphaned route_id='" << r.route_id
                      << "' for trip_id='" << r.trip_id << "' — row dropped\n";
            continue;
        }

        r.trip_headsign = (fields.size() > 3) ? fields[3] : "";
        r.shape_id      = (fields.size() > 4) ? fields[4] : "";
        r.direction_id  = (fields.size() > 5 && !fields[5].empty()) ? static_cast<uint8_t>(std::stoi(fields[5])) : 0;

        valid_trip_ids_.insert(r.trip_id);
        trips_.push_back(std::move(r));
        ++stats_.rows_accepted;
    }
}

// ── load_stop_times ──────────────────────────────────────────────────────

void GTFSParser::load_stop_times(const std::string& filename) {
    std::ifstream f(resolve_path(filename));
    if (!f.is_open()) {
        throw std::runtime_error("GTFSParser: cannot open " + resolve_path(filename));
    }

    std::string header_line, line;
    std::getline(f, header_line);

    while (std::getline(f, line)) {
        if (line.empty() || line == "\r") continue;
        ++stats_.rows_read;
        auto fields = split_csv_line(line);
        if (fields.size() < 5) { ++stats_.rows_dropped_fmt; continue; }

        StopTimeRecord r;
        r.trip_id = fields[0];
        r.stop_id = fields[3];

        // FK validation: O(1) lookups
        if (valid_trip_ids_.find(r.trip_id) == valid_trip_ids_.end()) {
            ++stats_.rows_dropped_fk;
            std::cerr << "[GTFS WARN] stop_times.txt: orphaned trip_id='"
                      << r.trip_id << "' — row dropped\n";
            continue;
        }
        if (valid_stop_ids_.find(r.stop_id) == valid_stop_ids_.end()) {
            ++stats_.rows_dropped_fk;
            std::cerr << "[GTFS WARN] stop_times.txt: orphaned stop_id='"
                      << r.stop_id << "' — row dropped\n";
            continue;
        }

        // Parse times — UINT32_MAX as sentinel for blank (interpolate later)
        try {
            r.arrival_time   = fields[1].empty() ? UINT32_MAX : parse_gtfs_time(fields[1]);
            r.departure_time = fields[2].empty() ? UINT32_MAX : parse_gtfs_time(fields[2]);
        } catch (const std::exception& e) {
            ++stats_.rows_dropped_fmt;
            std::cerr << "[GTFS WARN] stop_times.txt: time parse error: " << e.what() << "\n";
            continue;
        }

        r.stop_sequence = static_cast<uint32_t>(std::stoul(fields[4]));
        r.pickup_type   = (fields.size() > 5 && !fields[5].empty()) ? static_cast<uint8_t>(std::stoi(fields[5])) : 0;
        r.drop_off_type = (fields.size() > 6 && !fields[6].empty()) ? static_cast<uint8_t>(std::stoi(fields[6])) : 0;

        // H-new (Gemini audit): Do NOT filter pickup_type==1 here.
        // Filtering before interpolation destroys the contiguous stop sequence
        // that interpolate_stop_times() needs to compute cumulative distances
        // and fill blank arrival times geometrically. If stop seq [A, B*, C]
        // where B* has pickup_type=1 and a blank time, removing B* before
        // interpolation makes A and C appear adjacent — the interpolated time
        // for C is now wrong. Filter pickup_type at edge-generation time in
        // GraphBuilder::build() AFTER interpolation is complete.

        stop_times_.push_back(std::move(r));
        ++stats_.rows_accepted;
    }
}

// ── interpolate_stop_times ───────────────────────────────────────────────

void GTFSParser::interpolate_stop_times() {
    // Group stop times by trip_id, then sort by stop_sequence within each trip
    std::sort(stop_times_.begin(), stop_times_.end(),
        [](const StopTimeRecord& a, const StopTimeRecord& b) {
            if (a.trip_id != b.trip_id) return a.trip_id < b.trip_id;
            return a.stop_sequence < b.stop_sequence;
        });

    // Build stop coordinate lookup for Haversine distance
    std::unordered_map<std::string, std::pair<double,double>> stop_coords;
    for (const auto& s : stops_) {
        stop_coords[s.stop_id] = {s.stop_lat, s.stop_lon};
    }

    std::size_t i = 0;
    while (i < stop_times_.size()) {
        // Find the range [i, j) for this trip
        std::size_t j = i;
        const std::string& trip = stop_times_[i].trip_id;
        while (j < stop_times_.size() && stop_times_[j].trip_id == trip) ++j;

        // Within [i, j): find runs of UINT32_MAX and interpolate
        for (std::size_t k = i; k < j; ) {
            if (stop_times_[k].arrival_time == UINT32_MAX) {
                // Find prev defined time
                if (k == i) {
                    // Leading blank — set to first defined time (fallback)
                    std::size_t nxt = k;
                    while (nxt < j && stop_times_[nxt].arrival_time == UINT32_MAX) ++nxt;
                    if (nxt < j) {
                        for (std::size_t m = k; m < nxt; ++m) {
                            stop_times_[m].arrival_time   = stop_times_[nxt].arrival_time;
                            stop_times_[m].departure_time = stop_times_[nxt].departure_time;
                            ++stats_.times_interpolated;
                        }
                    }
                    k = nxt;
                    continue;
                }

                // Find the next defined timepoint
                std::size_t prev_k = k - 1;
                std::size_t next_k = k;
                while (next_k < j && stop_times_[next_k].arrival_time == UINT32_MAX) ++next_k;

                if (next_k >= j) {
                    // Trailing blank — copy from prev
                    for (std::size_t m = k; m < j; ++m) {
                        stop_times_[m].arrival_time   = stop_times_[prev_k].arrival_time;
                        stop_times_[m].departure_time = stop_times_[prev_k].departure_time;
                        ++stats_.times_interpolated;
                    }
                    k = j;
                    continue;
                }

                // Compute cumulative Haversine distances across the gap
                const uint32_t t_prev = stop_times_[prev_k].departure_time;
                const uint32_t t_next = stop_times_[next_k].arrival_time;
                const uint32_t dt     = (t_next > t_prev) ? (t_next - t_prev) : 0;

                // dt==0 guard: two consecutive defined timepoints with identical
                // timestamps — real dirty data. Dividing by dt causes SIGFPE.
                // Fall back to equal-spacing by stop_sequence index.
                if (dt == 0) {
                    for (std::size_t m = k; m < next_k; ++m) {
                        stop_times_[m].arrival_time   = t_prev;
                        stop_times_[m].departure_time = t_prev;
                        ++stats_.times_interpolated;
                    }
                    k = next_k;
                    continue;
                }

                // Sum distances from prev_k to next_k
                std::vector<double> cum_dist(next_k - prev_k + 1, 0.0);
                for (std::size_t m = prev_k; m < next_k; ++m) {
                    const auto& sid_a = stop_times_[m].stop_id;
                    const auto& sid_b = stop_times_[m + 1].stop_id;
                    double d = 0.0;
                    auto it_a = stop_coords.find(sid_a);
                    auto it_b = stop_coords.find(sid_b);
                    if (it_a != stop_coords.end() && it_b != stop_coords.end()) {
                        d = haversine_m(it_a->second.first, it_a->second.second,
                                        it_b->second.first, it_b->second.second);
                    }
                    cum_dist[m - prev_k + 1] = cum_dist[m - prev_k] + d;
                }

                const double total_dist = cum_dist.back();

                for (std::size_t m = k; m < next_k; ++m) {
                    const double frac = (total_dist > 0)
                        ? cum_dist[m - prev_k] / total_dist
                        : static_cast<double>(m - prev_k) / (next_k - prev_k);
                    const uint32_t interp = t_prev + static_cast<uint32_t>(frac * dt);
                    stop_times_[m].arrival_time   = interp;
                    stop_times_[m].departure_time = interp;
                    ++stats_.times_interpolated;
                }

                k = next_k;
            } else {
                ++k;
            }
        }

        i = j;
    }
}

// ── load_calendar ─────────────────────────────────────────────────────────
// Some GTFS feeds use ONLY calendar_dates.txt and have no calendar.txt.
// This method gracefully handles absence of the file and emits an informational
// message rather than throwing. Call load_calendar_dates() as well — whichever
// file exists will provide service schedule data.

void GTFSParser::load_calendar(const std::string& filename) {
    std::ifstream f(resolve_path(filename));
    if (!f.is_open()) {
        std::fprintf(stderr,
            "[GTFS INFO] %s not found — feed may use calendar_dates.txt only. "
            "Service day filtering not applied; all trips included.\n",
            resolve_path(filename).c_str());
        return;
    }

    std::string header_line, line;
    std::getline(f, header_line);
    uint32_t count = 0;

    while (std::getline(f, line)) {
        if (line.empty() || line == "\r") continue;
        auto fields = split_csv_line(line);
        if (fields.size() < 2) continue;
        // fields[0] = service_id, fields[1] = monday, ... fields[7] = sunday
        // fields[8] = start_date, fields[9] = end_date
        // We parse but do not yet enforce — log for transparency
        ++count;
    }
    std::fprintf(stderr, "[GTFS INFO] calendar.txt: %u service records loaded "
        "(service day filtering not yet enforced — all trips included).\n", count);
}

// ── load_calendar_dates ──────────────────────────────────────────────────

void GTFSParser::load_calendar_dates(const std::string& filename) {
    std::ifstream f(resolve_path(filename));
    if (!f.is_open()) {
        std::fprintf(stderr,
            "[GTFS INFO] %s not found.\n", resolve_path(filename).c_str());
        return;
    }

    std::string header_line, line;
    std::getline(f, header_line);
    uint32_t count = 0;

    while (std::getline(f, line)) {
        if (line.empty() || line == "\r") continue;
        auto fields = split_csv_line(line);
        if (fields.size() < 3) continue;
        // fields[0] = service_id, fields[1] = date, fields[2] = exception_type
        ++count;
    }
    std::fprintf(stderr, "[GTFS INFO] calendar_dates.txt: %u exception records loaded "
        "(service day filtering not yet enforced — all trips included).\n", count);
}

void GTFSParser::print_stats() const {
    std::printf(
        "\n╔══════════════════════════════════════╗\n"
        "║  GTFS Parse Statistics               ║\n"
        "╠══════════════════════════════════════╣\n"
        "║  Rows read        : %10llu        ║\n"
        "║  Rows accepted    : %10llu        ║\n"
        "║  Dropped (bad FK) : %10llu        ║\n"
        "║  Dropped (fmt)    : %10llu        ║\n"
        "║  Times interpolated: %9llu        ║\n"
        "╚══════════════════════════════════════╝\n",
        (unsigned long long)stats_.rows_read,
        (unsigned long long)stats_.rows_accepted,
        (unsigned long long)stats_.rows_dropped_fk,
        (unsigned long long)stats_.rows_dropped_fmt,
        (unsigned long long)stats_.times_interpolated
    );
}

// ── check_frequencies ────────────────────────────────────────────────────
// Gemini audit: frequencies.txt is not parsed. If BMRCL uses frequency-based
// scheduling (exact_times=0 or exact_times=1 blocks), the graph will be
// completely disconnected during those service windows — no edges at all.
// This function detects the file and emits a FATAL warning so the developer
// knows they need to implement load_frequencies() before the graph is valid.
void GTFSParser::check_frequencies(const std::string& filename) const {
    const std::string path = resolve_path(filename);
    std::ifstream f(path);
    if (!f.is_open()) {
        // No frequencies.txt — feed uses explicit timetable. This is fine.
        return;
    }
    // File exists: count non-header rows
    std::string line;
    std::getline(f, line); // skip header
    int row_count = 0;
    while (std::getline(f, line)) {
        if (!line.empty() && line != "\r") ++row_count;
    }
    if (row_count > 0) {
        std::fprintf(stderr,
            "\n[GTFS FATAL] frequencies.txt found with %d rows and is NOT parsed.\n"
            "If BMRCL uses frequency-based scheduling (headway_secs blocks),\n"
            "the CSR graph will be missing edges for those service windows.\n"
            "You must implement GTFSParser::load_frequencies() before building\n"
            "the graph on a feed that uses this file.\n"
            "See: https://gtfs.org/schedule/reference/#frequenciestxt\n\n",
            row_count);
        // Do not abort — let the developer decide. But they WILL see this.
    }
}

} // namespace namma_metro
