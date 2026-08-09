#include "crowd_model.hpp"
#include "gtfs_parser.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace namma_metro
{

    namespace
    {

        /// Fold a station name to a comparison key.
        ///
        /// Deliberately conservative. Only case, punctuation and a trailing
        /// "metro station" / "station" are removed. Words like "road" and
        /// "cross" are NOT stripped, because they distinguish real stations
        /// ("Mysore Road" is not "Mysore"), and a matcher that merges two
        /// stations silently attributes one station's ridership to another.
        std::string normalise_station_name(const std::string &raw)
        {
            std::string s;
            s.reserve(raw.size());
            for (const char c : raw)
            {
                const unsigned char u = static_cast<unsigned char>(c);
                if (std::isalnum(u))
                    s.push_back(static_cast<char>(std::tolower(u)));
                else
                    s.push_back(' ');
            }

            // Collapse runs of spaces and trim.
            std::string t;
            t.reserve(s.size());
            bool space = true; // leading spaces are dropped
            for (const char c : s)
            {
                if (c == ' ')
                {
                    if (!space)
                    {
                        t.push_back(' ');
                        space = true;
                    }
                }
                else
                {
                    t.push_back(c);
                    space = false;
                }
            }
            while (!t.empty() && t.back() == ' ')
                t.pop_back();

            // Drop a trailing "metro station" or "station", but never if that
            // would leave nothing behind.
            const auto ends_with = [&t](const std::string &suffix)
            {
                return t.size() > suffix.size() + 1 &&
                       t.compare(t.size() - suffix.size(), suffix.size(), suffix) == 0 &&
                       t[t.size() - suffix.size() - 1] == ' ';
            };
            if (ends_with("metro station"))
                t.erase(t.size() - std::string("metro station").size() - 1);
            else if (ends_with("station"))
                t.erase(t.size() - std::string("station").size() - 1);

            return t;
        }

        std::vector<std::string> split_on(const std::string &line, char delim)
        {
            std::vector<std::string> out;
            std::string field;
            std::istringstream ss(line);
            while (std::getline(ss, field, delim))
            {
                while (!field.empty() && (field.back() == '\r' || field.back() == ' '))
                    field.pop_back();
                std::size_t b = 0;
                while (b < field.size() && field[b] == ' ')
                    ++b;
                out.push_back(field.substr(b));
            }
            return out;
        }

        int find_column(const std::vector<std::string> &header, const std::string &want)
        {
            for (std::size_t i = 0; i < header.size(); ++i)
            {
                std::string h;
                for (const char c : header[i])
                {
                    const unsigned char u = static_cast<unsigned char>(c);
                    if (std::isalnum(u))
                        h.push_back(static_cast<char>(std::tolower(u)));
                }
                if (h == want)
                    return static_cast<int>(i);
            }
            return -1;
        }

    } // namespace

    float HourlyLoadSurface::peak() const noexcept
    {
        float m = 0.0f;
        for (const float v : load)
            m = std::max(m, v);
        return m;
    }

    std::unordered_map<std::string, std::string> load_station_aliases(const std::string &path)
    {
        std::unordered_map<std::string, std::string> out;
        std::ifstream f(path);
        if (!f.is_open())
            return out; // absent is fine: most feeds need no aliases

        std::string line;
        while (std::getline(f, line))
        {
            if (line.empty() || line[0] == '#')
                continue;
            // Pipe-separated, not comma-separated. Two of the station names this
            // file exists to fix contain commas — "Sir M. Visvesvaraya Stn.,
            // Central College" is one — so a comma separator would split a name
            // in half and the alias would silently never match.
            const auto parts = split_on(line, '|');
            if (parts.size() < 2 || parts[0].empty() || parts[1].empty())
                continue;
            out[parts[0]] = parts[1];
        }
        return out;
    }

    HourlyLoadSurface load_hourly_ridership_csv(
        const std::string &csv_path,
        const std::vector<StopRecord> &stops,
        const std::unordered_map<std::string, uint32_t> &stop_index_map,
        const std::unordered_map<std::string, std::string> &aliases)
    {
        std::ifstream f(csv_path);
        if (!f.is_open())
            throw std::runtime_error("load_hourly_ridership_csv: cannot open " + csv_path);

        std::string header_line;
        if (!std::getline(f, header_line))
            throw std::runtime_error("load_hourly_ridership_csv: empty file " + csv_path);

        // The published BMRCL export is semicolon-separated; a republication
        // might not be. Pick whichever delimiter yields more header fields.
        const char delim =
            (split_on(header_line, ';').size() >= split_on(header_line, ',').size()) ? ';' : ',';
        const auto header = split_on(header_line, delim);

        const int col_date = find_column(header, "date");
        const int col_hour = find_column(header, "hour");
        const int col_station = find_column(header, "station");
        const int col_riders = find_column(header, "ridership");
        if (col_date < 0 || col_hour < 0 || col_station < 0 || col_riders < 0)
            throw std::runtime_error(
                "load_hourly_ridership_csv: " + csv_path +
                " must have Date, Hour, Station and Ridership columns; got: " + header_line);

        // ── Map GTFS stop names to node indices ───────────────────────────────
        // A normalised name can legitimately cover several nodes when the feed
        // keeps platforms separate, so the value is a list and the station's
        // ridership is attributed to every platform of that station.
        std::unordered_map<std::string, std::vector<uint32_t>> nodes_by_name;
        for (const auto &s : stops)
        {
            const auto it = stop_index_map.find(s.stop_id);
            if (it == stop_index_map.end())
                continue;
            nodes_by_name[normalise_station_name(s.stop_name)].push_back(it->second);
        }

        HourlyLoadSurface surf;
        surf.num_stops = static_cast<uint32_t>(stops.size());
        surf.source_path = csv_path;
        surf.load.assign(static_cast<std::size_t>(surf.num_stops) * 24, 0.0f);

        // Accumulate sums and the set of (station, date) days seen, so the mean
        // is over days actually present rather than over an assumed calendar.
        std::vector<double> sum(static_cast<std::size_t>(surf.num_stops) * 24, 0.0);
        std::set<std::string> dates;
        std::unordered_set<std::string> unmatched_names;
        std::unordered_set<uint32_t> matched_nodes;

        std::string line;
        while (std::getline(f, line))
        {
            if (line.empty())
                continue;
            const auto fields = split_on(line, delim);
            const std::size_t need = static_cast<std::size_t>(
                std::max(std::max(col_date, col_hour), std::max(col_station, col_riders)));
            if (fields.size() <= need)
                continue;

            ++surf.rows_read;
            dates.insert(fields[static_cast<std::size_t>(col_date)]);

            const std::string &raw_station = fields[static_cast<std::size_t>(col_station)];
            std::string lookup = raw_station;
            if (const auto a = aliases.find(raw_station); a != aliases.end())
                lookup = a->second;

            const auto hit = nodes_by_name.find(normalise_station_name(lookup));
            if (hit == nodes_by_name.end())
            {
                ++surf.rows_unmatched;
                unmatched_names.insert(raw_station);
                continue;
            }

            long hour = 0, riders = 0;
            try
            {
                hour = std::stol(fields[static_cast<std::size_t>(col_hour)]);
                riders = std::stol(fields[static_cast<std::size_t>(col_riders)]);
            }
            catch (const std::exception &)
            {
                ++surf.rows_unmatched;
                continue;
            }
            if (hour < 0 || hour > 23 || riders < 0)
                continue;

            for (const uint32_t node : hit->second)
            {
                sum[static_cast<std::size_t>(node) * 24 + static_cast<std::size_t>(hour)] +=
                    static_cast<double>(riders);
                matched_nodes.insert(node);
            }
        }

        surf.days_averaged = static_cast<uint32_t>(dates.size());
        if (!dates.empty())
        {
            surf.first_date = *dates.begin();
            surf.last_date = *dates.rbegin();
        }
        const double denom = surf.days_averaged > 0 ? surf.days_averaged : 1.0;
        for (std::size_t i = 0; i < sum.size(); ++i)
            surf.load[i] = static_cast<float>(sum[i] / denom);

        surf.stops_matched = static_cast<uint32_t>(matched_nodes.size());
        surf.unmatched_ridership_names.assign(unmatched_names.begin(), unmatched_names.end());
        std::sort(surf.unmatched_ridership_names.begin(), surf.unmatched_ridership_names.end());

        for (const auto &s : stops)
        {
            const auto it = stop_index_map.find(s.stop_id);
            if (it == stop_index_map.end())
                continue;
            if (matched_nodes.find(it->second) == matched_nodes.end())
                surf.unmatched_gtfs_stops.push_back(s.stop_name);
        }
        std::sort(surf.unmatched_gtfs_stops.begin(), surf.unmatched_gtfs_stops.end());
        surf.unmatched_gtfs_stops.erase(
            std::unique(surf.unmatched_gtfs_stops.begin(), surf.unmatched_gtfs_stops.end()),
            surf.unmatched_gtfs_stops.end());

        return surf;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Applying the surface
    // ═══════════════════════════════════════════════════════════════════════════

    namespace
    {

        /// Sample the surface at an arbitrary second of the day.
        ///
        /// Hourly counts are treated as samples at the CENTRE of their hour
        /// (hh:30:00) and interpolated linearly, which is what keeps the weight
        /// function's derivative bounded — a raw step function drops by the full
        /// bucket difference in one second and breaks the FIFO argument outright.
        /// Times past 24:00 (legal in GTFS) wrap, because the ridership file is
        /// indexed by clock hour.
        float sample_load(const HourlyLoadSurface &s, uint32_t node, uint32_t t, bool interpolate)
        {
            const uint32_t sec_of_day = t % 86400u;
            if (!interpolate)
                return s.at(node, sec_of_day / 3600u);

            // Position relative to hour centres: centre of hour h is at
            // h*3600 + 1800 seconds.
            const double x = (static_cast<double>(sec_of_day) - 1800.0) / 3600.0;
            const double floor_x = std::floor(x);
            const double frac = x - floor_x;
            const int32_t h0 = static_cast<int32_t>(floor_x);
            const uint32_t lo = static_cast<uint32_t>((h0 % 24 + 24) % 24);
            const uint32_t hi = (lo + 1u) % 24u;
            const float a = s.at(node, lo);
            const float b = s.at(node, hi);
            return static_cast<float>(a + (b - a) * frac);
        }

    } // namespace

    uint32_t gaussian_crowd_weight(uint32_t departure_time_seconds) noexcept
    {
        // Byte-for-byte the model in src/graph_builder.cpp. tests/test_crowd_model.cpp
        // pins the two together by building a graph and comparing edge weights.
        constexpr double PEAK_TIME = 28800.0;
        constexpr double SIGMA = 3600.0;
        constexpr double BASE = 10.0;
        constexpr double AMPLITUDE = 90.0;
        const double t = static_cast<double>(departure_time_seconds);
        const double crowd_raw = BASE + AMPLITUDE * std::exp(-((t - PEAK_TIME) * (t - PEAK_TIME)) / (2.0 * SIGMA * SIGMA));
        return static_cast<uint32_t>(crowd_raw * 10.0);
    }

    CrowdApplyReport apply_hourly_crowd(
        CSRGraph &g,
        const HourlyLoadSurface &surface,
        CrowdModelConfig config)
    {
        if (g.second_objective != SecondObjective::CrowdExposure)
            throw std::invalid_argument(
                "apply_hourly_crowd(): graph's second objective is TransferCount. "
                "Writing crowd scores into secondary_weight there would turn the "
                "transfer count into a crowd score without changing its name. "
                "Build the graph with SecondObjective::CrowdExposure first.");

        if (surface.num_stops != g.num_nodes)
            throw std::invalid_argument(
                "apply_hourly_crowd(): the load surface covers " +
                std::to_string(surface.num_stops) + " stops but the graph has " +
                std::to_string(g.num_nodes) + " nodes. They must come from the same feed.");

        CrowdApplyReport rep;
        const float peak = surface.peak();
        if (peak <= 0.0f)
        {
            // Nothing matched, or an all-zero file. Refuse rather than write
            // zeros everywhere, which would look exactly like a working run with
            // an uninteresting result.
            throw std::invalid_argument(
                "apply_hourly_crowd(): the load surface is empty or all zero "
                "(peak = 0). Check the station-name match rate reported by "
                "load_hourly_ridership_csv().");
        }
        const float unit = config.scale / peak;

        double weight_sum = 0.0;

        for (uint32_t u = 0; u < g.num_nodes; ++u)
        {
            const bool stop_has_data = [&]
            {
                for (uint32_t h = 0; h < 24; ++h)
                    if (surface.at(u, h) > 0.0f)
                        return true;
                return false;
            }();

            for (uint32_t i = g.offset[u]; i < g.offset[u + 1]; ++i)
            {
                Edge &e = g.edge_data[i];
                const float load = sample_load(surface, u, e.departure_time, config.interpolate);
                const float w = load * unit;
                e.secondary_weight = static_cast<uint32_t>(w < 0.0f ? 0.0f : w + 0.5f);
                ++rep.edges_rewritten;
                if (!stop_has_data)
                    ++rep.edges_from_unmatched_stop;
                rep.max_weight = std::max(rep.max_weight, static_cast<float>(e.secondary_weight));
                weight_sum += e.secondary_weight;
            }
        }

        if (rep.edges_rewritten > 0)
            rep.mean_weight = weight_sum / rep.edges_rewritten;

        // ── Verify the FIFO derivative bound on the graph we just produced ────
        // The interpolation argument says this cannot fail for scale <= 3600,
        // but the argument is about the continuous function and the graph holds
        // sampled departures. So measure it.
        //
        // Only CONSECUTIVE departures on a link are compared, and that is exact
        // rather than an approximation: for three departures t1 < t2 < t3 on one
        // link, the slope across (t1, t3) is a convex combination of the two
        // consecutive slopes and therefore never more negative than both. The
        // steepest fall is always attained by an adjacent pair. Checking every
        // pair instead is quadratic in a node's out-degree, which on a national
        // feed is tens of thousands of edges per node.
        //
        // GraphBuilder guarantees each node's edges are sorted by departure
        // time, so "the previous departure to this destination" is one lookup.
        std::unordered_map<uint32_t, std::pair<uint32_t, uint32_t>> last_on_link;
        for (uint32_t u = 0; u < g.num_nodes; ++u)
        {
            last_on_link.clear();
            const auto [begin, end] = g.edges_of(u);
            for (const Edge *e = begin; e != end; ++e)
            {
                const auto it = last_on_link.find(e->destination);
                if (it != last_on_link.end())
                {
                    const int64_t dt = static_cast<int64_t>(e->departure_time) - static_cast<int64_t>(it->second.first);
                    const int64_t dw = static_cast<int64_t>(e->secondary_weight) - static_cast<int64_t>(it->second.second);
                    if (dt > 0 && dw < 0)
                    {
                        const double slope = static_cast<double>(-dw) / static_cast<double>(dt);
                        rep.max_negative_slope = std::max(rep.max_negative_slope, slope);
                    }
                }
                last_on_link[e->destination] = {e->departure_time, e->secondary_weight};
            }
        }
        rep.fifo_bound_holds = (rep.max_negative_slope <= 1.0);

        if (!rep.fifo_bound_holds)
            std::fprintf(stderr,
                         "[CROWD WARN] the applied crowd field drops by up to %.3f units per "
                         "second on some link, exceeding the d/dt(crowd) >= -1 bound the FIFO "
                         "argument requires. Lower CrowdModelConfig::scale or leave "
                         "interpolation on.\n",
                         rep.max_negative_slope);

        return rep;
    }

} // namespace namma_metro
