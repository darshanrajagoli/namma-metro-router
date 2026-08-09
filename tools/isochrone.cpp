// tools/isochrone.cpp — the accessibility surface, as a map and as a table.
//
// WHAT THIS IS FOR
// ════════════════
// The engine's actual output is, for one origin and one departure time, the
// non-dominated ways of reaching every other station at once. Presented as a
// latency table that reads as a benchmark. Presented as an accessibility
// surface it reads as a statement about a city.
//
// Two modes:
//
//   --origin STOP_ID   ONE isochrone. For each station: the earliest arrival
//                      with 0 changes, with at most 1, with at most 2. The
//                      interesting column is the difference.
//
//   (default)          The whole-network surface. For every station, the mean
//                      number of stations reachable within 30/45/60 minutes at
//                      each change budget, averaged over six departure times
//                      across the day, plus the GAP between the full change
//                      budget and none — the part of the network that is
//                      nominally accessible and practically is not for anyone
//                      who cannot change.
//
// Both modes write a CSV and a self-contained SVG.
//
// USAGE
//   routing_engine_isochrone <feed_dir> [--origin STOP_ID] [--at HH:MM]
//                            [--out-prefix PATH] [--thresholds 1800,2700,3600]
//                            [--max-changes 2] [--labels]

#include "accessibility.hpp"
#include "graph.hpp"
#include "gtfs_parser.hpp"
#include "raptor.hpp"
#include "topology.hpp"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using namespace namma_metro;

namespace
{

    std::vector<uint32_t> parse_uint_list(const std::string &s)
    {
        std::vector<uint32_t> out;
        std::stringstream ss(s);
        std::string tok;
        while (std::getline(ss, tok, ','))
            if (!tok.empty())
                out.push_back(static_cast<uint32_t>(std::stoul(tok)));
        return out;
    }

    uint32_t parse_hhmm(const std::string &s)
    {
        const auto colon = s.find(':');
        if (colon == std::string::npos)
            return static_cast<uint32_t>(std::stoul(s));
        return static_cast<uint32_t>(std::stoul(s.substr(0, colon))) * 3600u +
               static_cast<uint32_t>(std::stoul(s.substr(colon + 1))) * 60u;
    }

    std::string hhmmss(uint32_t t)
    {
        char b[16];
        std::snprintf(b, sizeof(b), "%02u:%02u:%02u", t / 3600, (t % 3600) / 60, t % 60);
        return std::string(b);
    }

    /// CSV fields are written unquoted, so a stop name carrying a comma would
    /// silently shift every later column. Replace rather than quote: this is a
    /// data file for analysis, not a faithful reproduction of the feed's text.
    std::string csv_safe(std::string s)
    {
        for (char &c : s)
            if (c == ',' || c == '\n' || c == '\r' || c == '"')
                c = ' ';
        return s;
    }

    /// Last path component, so a chart's subtitle names the feed rather than
    /// publishing whatever absolute path the machine happened to use.
    std::string feed_label(const std::string &dir)
    {
        std::string s = dir;
        while (!s.empty() && (s.back() == '/' || s.back() == '\\'))
            s.pop_back();
        const auto cut = s.find_last_of("/\\");
        return (cut == std::string::npos) ? s : s.substr(cut + 1);
    }

    bool write_file(const std::string &path, const std::string &content)
    {
        std::ofstream f(path, std::ios::binary);
        if (!f)
        {
            std::fprintf(stderr, "cannot write %s\n", path.c_str());
            return false;
        }
        f << content;
        return true;
    }

} // namespace

int main(int argc, char **argv)
{
    std::string feed_dir, origin_stop, out_prefix = "accessibility";
    uint32_t at_time = 28800; // 08:00
    uint32_t max_changes = 2;
    bool labels = false;
    std::vector<uint32_t> thresholds{1800, 2700, 3600};

    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        if (a == "--origin" && i + 1 < argc)
            origin_stop = argv[++i];
        else if (a == "--at" && i + 1 < argc)
            at_time = parse_hhmm(argv[++i]);
        else if (a == "--out-prefix" && i + 1 < argc)
            out_prefix = argv[++i];
        else if (a == "--thresholds" && i + 1 < argc)
            thresholds = parse_uint_list(argv[++i]);
        else if (a == "--max-changes" && i + 1 < argc)
            max_changes = static_cast<uint32_t>(std::stoul(argv[++i]));
        else if (a == "--labels")
            labels = true;
        else if (a.rfind("--", 0) == 0)
        {
            std::fprintf(stderr, "unknown option %s\n", a.c_str());
            return 2;
        }
        else
            feed_dir = a;
    }
    if (feed_dir.empty())
    {
        std::fprintf(stderr,
                     "usage: routing_engine_isochrone <feed_dir> [--origin STOP_ID] [--at HH:MM]\n"
                     "                                [--out-prefix PATH] [--thresholds a,b,c]\n"
                     "                                [--max-changes N] [--labels]\n");
        return 2;
    }
    if (thresholds.empty())
    {
        std::fprintf(stderr, "--thresholds must list at least one value\n");
        return 2;
    }

    // ── Load ──────────────────────────────────────────────────────────────────
    GTFSParser parser(feed_dir);
    try
    {
        parser.load_agency();
        parser.check_frequencies();
        parser.load_stops();
        parser.load_routes();
        parser.load_trips();
        parser.load_stop_times();
        parser.load_calendar();
        parser.load_calendar_dates();
        parser.load_transfers();
        parser.interpolate_stop_times();
    }
    catch (const std::exception &e)
    {
        std::fprintf(stderr, "load failed: %s\n", e.what());
        return 1;
    }
    if (parser.stop_times().empty())
    {
        std::fprintf(stderr, "feed carried no stop_times rows\n");
        return 1;
    }

    const uint32_t num_stops = static_cast<uint32_t>(parser.stops().size());
    RaptorTimetable tt = RaptorBuilder::build(
        parser.stop_times(), num_stops, &parser.stop_index_map(), parser.transfers());

    StationGraph sg;
    const TopologyMetrics topo = compute_topology(
        parser.stop_times(), parser.trips(), num_stops, &parser.stop_index_map(),
        parser.transfers(), &sg);

    std::printf("feed     : %s\n", feed_dir.c_str());
    std::printf("network  : %s\n", topo.summary().c_str());
    if (!transfers_are_transitively_closed(tt))
        std::printf("[WARN] footpaths are not transitively closed; arrivals may be upper bounds.\n");

    // Draw and report at STATION level, not platform level: two platforms of one
    // station are one place a passenger can get to, and counting them twice
    // would inflate every reachability number at exactly the interchanges the
    // change-budget analysis is about.
    std::vector<uint32_t> stations = sg.stations;
    if (stations.empty())
    {
        std::fprintf(stderr, "no served stations in this feed\n");
        return 1;
    }
    // Every platform of a station maps to its representative, so a destination
    // counts once however many platforms it has.
    std::vector<uint8_t> is_station_rep(num_stops, 0);
    for (const uint32_t s : stations)
        is_station_rep[s] = 1;

    auto placements = placements_from_stops(parser.stops(), parser.stop_index_map(), stations);
    std::printf("drawing  : %zu stations, %zu links\n\n", placements.size(), sg.links.size());

    // ═════════════════════════════════════════════════════════════════════════
    // Mode 1 — one origin, one departure: an isochrone with a change budget
    // ═════════════════════════════════════════════════════════════════════════
    if (!origin_stop.empty())
    {
        const auto it = parser.stop_index_map().find(origin_stop);
        if (it == parser.stop_index_map().end())
        {
            std::fprintf(stderr, "origin stop_id '%s' is not in this feed\n", origin_stop.c_str());
            return 1;
        }
        const uint32_t origin = sg.station_of[it->second];

        Raptor raptor(tt, std::max(max_changes + 2u, 4u));
        auto rr = raptor.run(origin, at_time);

        // "at_most" is not pedantry: column `minutes_at_most_1_change` is the
        // best journey using ONE change OR NONE, because tau is cumulative over
        // rounds. Reading it as "journeys with exactly one change" would make
        // every column look wrong against the one before it.
        std::ostringstream csv;
        csv << "node,stop_id,stop_name,lat,lon";
        for (uint32_t c = 0; c <= max_changes; ++c)
            csv << ",minutes_at_most_" << c << (c == 1 ? "_change" : "_changes");
        csv << ",penalty_minutes_if_no_changes\n";

        std::unordered_map<uint32_t, const StopRecord *> by_node;
        for (const auto &s : parser.stops())
        {
            const auto x = parser.stop_index_map().find(s.stop_id);
            if (x != parser.stop_index_map().end())
                by_node.emplace(x->second, &s);
        }

        std::vector<double> values;
        values.reserve(placements.size());
        uint32_t reached_direct = 0, reached_any = 0;

        for (const auto &p : placements)
        {
            const StopRecord *rec = by_node.count(p.node) ? by_node[p.node] : nullptr;
            csv << p.node << ',' << csv_safe(rec ? rec->stop_id : "") << ','
                << csv_safe(p.name) << ',' << p.lat << ',' << p.lon;

            double direct = -1.0, best = -1.0;
            for (uint32_t c = 0; c <= max_changes; ++c)
            {
                const uint32_t round = std::min(c + 1u, rr.rounds);
                const uint32_t arr = rr.arrival(round, p.node);
                if (arr == RAPTOR_UNREACHED)
                    csv << ",";
                else
                {
                    const double mins = (static_cast<double>(arr) - at_time) / 60.0;
                    csv << ',' << mins;
                    if (c == 0)
                        direct = mins;
                    best = (best < 0.0) ? mins : std::min(best, mins);
                }
            }
            // The origin is drawn on the map but is not a destination: counting
            // "you can reach where you already are" would inflate both totals by
            // one and, on a small network, visibly.
            if (p.node != origin)
            {
                if (best >= 0.0)
                    ++reached_any;
                if (direct >= 0.0)
                    ++reached_direct;
            }

            // The number this whole tool exists to surface: how much longer the
            // journey is for someone who will not change. Blank when they cannot
            // get there at all without changing, which is a stronger statement
            // than any number.
            if (best >= 0.0 && direct >= 0.0)
            {
                csv << ',' << (direct - best);
                values.push_back(direct - best);
            }
            else if (best >= 0.0)
            {
                csv << ",unreachable_without_changing";
                values.push_back(-1.0); // rendered at the bottom of the ramp
            }
            else
            {
                // Not reachable at all from here at this time. Also drawn at the
                // bottom of the ramp, and NOT at zero: zero means "changing buys
                // you nothing", which would paint an unreachable station the same
                // colour as a perfectly convenient one. The CSV keeps the two
                // apart — this cell is blank, the case above is labelled.
                csv << ",";
                values.push_back(-1.0);
            }
            csv << '\n';
        }

        MapStyle style;
        style.title = "Reachability penalty for not changing";
        style.subtitle = feed_label(feed_dir) + " — from " + origin_stop + " departing " +
                         hhmmss(at_time) + "; colour = extra minutes if the journey must be direct";
        style.legend_label = "extra minutes if the journey must be direct (-1 = no direct journey exists)";
        style.labels = labels;

        const std::string csv_path = out_prefix + "-isochrone.csv";
        const std::string svg_path = out_prefix + "-isochrone.svg";
        if (!write_file(csv_path, csv.str()))
            return 1;
        if (!write_file(svg_path, render_station_map_svg(placements, sg.links, values, style)))
            return 1;

        std::printf("=== Isochrone from %s at %s ===\n", origin_stop.c_str(), hhmmss(at_time).c_str());
        std::printf("  other stations reachable at all        : %u of %zu\n",
                    reached_any, placements.size() - 1);
        std::printf("  reachable without changing            : %u\n", reached_direct);
        std::printf("  reachable ONLY by changing            : %u\n", reached_any - reached_direct);
        std::printf("\nwrote %s\nwrote %s\n", csv_path.c_str(), svg_path.c_str());
        return 0;
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Mode 2 — the whole-network surface
    // ═════════════════════════════════════════════════════════════════════════
    AccessibilityConfig cfg;
    cfg.thresholds_s = thresholds;
    cfg.max_changes = max_changes;

    // Origins AND destinations are station representatives, so a feed that keeps
    // platforms separate does not count each platform of a station as somewhere
    // else you can get to. This is the library function that
    // tests/test_accessibility.cpp exercises — the tool must not carry its own
    // copy of the loop, or the tests would be covering code nothing runs.
    const AccessibilitySurface surface =
        compute_accessibility(tt, stations, cfg, stations);

    // ── Report and write ─────────────────────────────────────────────────────
    std::unordered_map<uint32_t, const StopRecord *> by_node;
    for (const auto &s : parser.stops())
    {
        const auto x = parser.stop_index_map().find(s.stop_id);
        if (x != parser.stop_index_map().end())
            by_node.emplace(x->second, &s);
    }

    std::ostringstream csv;
    csv << "node,stop_id,stop_name,lat,lon,departures_with_service";
    for (uint32_t c = 0; c <= max_changes; ++c)
        for (uint32_t t = 0; t < surface.num_thresholds; ++t)
            csv << ",reach_" << c << "ch_" << cfg.thresholds_s[t] << "s";
    for (uint32_t t = 0; t < surface.num_thresholds; ++t)
        csv << ",gap_" << cfg.thresholds_s[t] << "s";
    csv << '\n';

    // The headline threshold for the map and the summary: whichever supplied
    // value is closest to 45 minutes, the conventional planning cut.
    //
    // Not "index 1", which is what this used to be. That happens to be 45
    // minutes for the default 30/45/60 list and is silently something else for
    // any other --thresholds, so the map's own subtitle would have named a
    // number the map was not drawing.
    uint32_t headline_t = 0;
    {
        constexpr uint32_t kPlanningCut = 2700; // 45 minutes
        uint32_t best_gap = UINT32_MAX;
        for (uint32_t t = 0; t < surface.num_thresholds; ++t)
        {
            const uint32_t v = cfg.thresholds_s[t];
            const uint32_t gap = (v > kPlanningCut) ? (v - kPlanningCut) : (kPlanningCut - v);
            if (gap < best_gap)
            {
                best_gap = gap;
                headline_t = t;
            }
        }
    }

    std::vector<double> values;
    values.reserve(placements.size());
    std::unordered_map<uint32_t, std::size_t> origin_slot;
    for (std::size_t i = 0; i < surface.per_origin.size(); ++i)
        origin_slot.emplace(surface.per_origin[i].node, i);

    double sum_direct = 0.0, sum_full = 0.0;
    for (const auto &sa : surface.per_origin)
    {
        const StopRecord *rec = by_node.count(sa.node) ? by_node[sa.node] : nullptr;
        csv << sa.node << ',' << csv_safe(rec ? rec->stop_id : "") << ','
            << csv_safe(rec ? rec->stop_name : "") << ','
            << (rec ? rec->stop_lat : 0.0) << ',' << (rec ? rec->stop_lon : 0.0) << ','
            << sa.departures_with_service;
        for (const double x : sa.counts)
            csv << ',' << x;
        for (uint32_t t = 0; t < surface.num_thresholds; ++t)
            csv << ',' << (sa.counts[static_cast<std::size_t>(max_changes) * surface.num_thresholds + t] - sa.counts[t]);
        csv << '\n';
        sum_direct += sa.counts[headline_t];
        sum_full += sa.counts[static_cast<std::size_t>(max_changes) * surface.num_thresholds + headline_t];
    }

    for (const auto &p : placements)
    {
        const auto s = origin_slot.find(p.node);
        values.push_back(s == origin_slot.end() ? 0.0 : surface.gap(static_cast<uint32_t>(s->second), headline_t));
    }

    char sub[512];
    std::snprintf(sub, sizeof(sub),
                  "%s — stations reachable within %u min with <=%u changes, minus those reachable "
                  "with none; mean over %zu departures",
                  feed_label(feed_dir).c_str(), cfg.thresholds_s[headline_t] / 60, max_changes,
                  cfg.departures.size());

    MapStyle style;
    style.title = "What changing lines buys you";
    style.subtitle = sub;
    style.legend_label = "extra stations reachable";
    style.labels = labels;

    const std::string csv_path = out_prefix + "-surface.csv";
    const std::string svg_path = out_prefix + "-surface.svg";
    if (!write_file(csv_path, csv.str()))
        return 1;
    if (!write_file(svg_path, render_station_map_svg(placements, sg.links, values, style)))
        return 1;

    // Percentages are of the OTHER stations, not of all of them: an origin is
    // not one of its own destinations, so the reachable maximum is n - 1.
    const double n_st = static_cast<double>(stations.size());
    const double n_dest = (n_st > 1.0) ? (n_st - 1.0) : 1.0;
    std::printf("=== Accessibility surface (%zu departures across the day) ===\n",
                cfg.departures.size());
    std::printf("  at %u minutes, averaged over every station, out of %.0f reachable others:\n",
                cfg.thresholds_s[headline_t] / 60, n_dest);
    std::printf("    reachable with 0 changes      : %.1f stations (%.1f%%)\n",
                sum_direct / n_st, 100.0 * sum_direct / n_st / n_dest);
    std::printf("    reachable with up to %u changes: %.1f stations (%.1f%%)\n",
                max_changes, sum_full / n_st, 100.0 * sum_full / n_st / n_dest);
    std::printf("    the gap                       : %.1f stations per origin\n",
                (sum_full - sum_direct) / n_st);
    std::printf("\nwrote %s\nwrote %s\n", csv_path.c_str(), svg_path.c_str());
    return 0;
}
