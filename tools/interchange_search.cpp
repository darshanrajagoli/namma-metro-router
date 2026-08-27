// tools/interchange_search.cpp — where should one new interchange go?
//
// WHAT THIS IS FOR
// ════════════════
// tools/isochrone.cpp measures what a network gives its passengers. This asks
// the design question that follows: given money for exactly ONE new interchange,
// where does it go to open up the most of the city?
//
// Every pair of stations within walking distance that the network does not
// already join is proposed as a footpath, the timetable is rebuilt around it,
// and the accessibility surface is recomputed. Candidates are ranked by the
// change they actually produced, not by how close together they look.
//
// It writes three things:
//   <prefix>-candidates.csv  every candidate, ranked, with before/after/delta
//   <prefix>-sites.svg       the network, stations coloured by the best gain any
//                            candidate touching that station achieved
//   stdout                   the ranked table and the baseline it is measured against
//
// USAGE
//   routing_engine_interchange_search <feed_dir> [--max-walk METRES]
//                                     [--threshold SECONDS] [--max-changes N]
//                                     [--walk-speed MPS] [--top N]
//                                     [--out-prefix PATH] [--labels]

#include "accessibility.hpp"
#include "graph.hpp"
#include "gtfs_parser.hpp"
#include "interchange.hpp"
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
    std::string feed_dir, out_prefix = "interchange";
    double max_walk = 800.0;
    double walk_speed = 1.2;
    uint32_t threshold_s = 2700;
    uint32_t max_changes = 2;
    std::size_t top_n = 10;
    bool labels = false;

    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        if (a == "--max-walk" && i + 1 < argc)
            max_walk = std::stod(argv[++i]);
        else if (a == "--walk-speed" && i + 1 < argc)
            walk_speed = std::stod(argv[++i]);
        else if (a == "--threshold" && i + 1 < argc)
            threshold_s = static_cast<uint32_t>(std::stoul(argv[++i]));
        else if (a == "--max-changes" && i + 1 < argc)
            max_changes = static_cast<uint32_t>(std::stoul(argv[++i]));
        else if (a == "--top" && i + 1 < argc)
            top_n = static_cast<std::size_t>(std::stoul(argv[++i]));
        else if (a == "--out-prefix" && i + 1 < argc)
            out_prefix = argv[++i];
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
                     "usage: routing_engine_interchange_search <feed_dir> [--max-walk METRES]\n"
                     "                                  [--threshold SECONDS] [--max-changes N]\n"
                     "                                  [--walk-speed MPS] [--top N]\n"
                     "                                  [--out-prefix PATH] [--labels]\n");
        return 2;
    }
    if (max_walk <= 0.0 || walk_speed <= 0.0 || threshold_s == 0)
    {
        std::fprintf(stderr, "--max-walk, --walk-speed and --threshold must all be positive\n");
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

    // Computed ONCE, from the unmodified feed, and never recomputed. Stations are
    // the connected components of the transfer graph, so recomputing this with a
    // candidate footpath added would merge the two stations being connected into
    // one and quietly change the denominator of every count. See interchange.hpp.
    StationGraph sg;
    const TopologyMetrics topo = compute_topology(
        parser.stop_times(), parser.trips(), num_stops, &parser.stop_index_map(),
        parser.transfers(), &sg);

    std::printf("feed     : %s\n", feed_dir.c_str());
    std::printf("network  : %s\n", topo.summary().c_str());
    if (sg.stations.empty())
    {
        std::fprintf(stderr, "no served stations in this feed\n");
        return 1;
    }

    // ── Names for reporting ──────────────────────────────────────────────────
    // Built before the search rather than after it, because the "no candidates"
    // branch below also needs to name stations.
    std::unordered_map<uint32_t, const StopRecord *> by_node;
    for (const StopRecord &s : parser.stops())
    {
        const auto x = parser.stop_index_map().find(s.stop_id);
        if (x != parser.stop_index_map().end())
            by_node.emplace(x->second, &s);
    }
    auto name_of = [&](uint32_t node) -> std::string
    {
        const auto it = by_node.find(node);
        return it == by_node.end() ? ("node " + std::to_string(node)) : it->second->stop_name;
    };
    auto id_of = [&](uint32_t node) -> std::string
    {
        const auto it = by_node.find(node);
        return it == by_node.end() ? std::string() : it->second->stop_id;
    };

    InterchangeInputs in;
    in.stop_times = &parser.stop_times();
    in.num_stops = num_stops;
    in.stop_index_map = &parser.stop_index_map();
    in.base_transfers = &parser.transfers();
    in.stops = &parser.stops();
    in.station_graph = &sg;

    InterchangeSearchConfig cfg;
    cfg.max_walk_m = max_walk;
    cfg.walk_speed_mps = walk_speed;
    cfg.accessibility.thresholds_s = {threshold_s};
    cfg.accessibility.max_changes = max_changes;
    cfg.threshold_index = 0;

    std::vector<InterchangeCandidate> candidates;
    SurfaceMeans base;
    try
    {
        candidates = generate_interchange_candidates(in, cfg);
        base = measure_baseline(in, cfg);
    }
    catch (const std::exception &e)
    {
        std::fprintf(stderr, "search setup failed: %s\n", e.what());
        return 1;
    }

    const double n_st = static_cast<double>(sg.stations.size());
    const double n_dest = (n_st > 1.0) ? (n_st - 1.0) : 1.0;

    std::printf("\n=== Baseline: the network as it is ===\n");
    std::printf("  stations                          : %zu\n", sg.stations.size());
    std::printf("  cyclomatic number (independent cycles): %lld%s\n",
                static_cast<long long>(topo.cyclomatic),
                topo.cyclomatic == 0 ? "  <- a forest: exactly one route between any two stations"
                                     : "");
    std::printf("  at %u min, mean over every station, out of %.0f reachable others:\n",
                threshold_s / 60, n_dest);
    std::printf("    reachable with 0 changes        : %.2f stations\n", base.direct);
    std::printf("    reachable with up to %u changes  : %.2f stations\n", max_changes, base.full);
    std::printf("    the gap                         : %.2f stations\n", base.gap);

    if (candidates.empty())
    {
        std::printf("\nNo candidate interchanges: no two unjoined stations lie within %.0f m.\n",
                    max_walk);

        // "None" on its own is not a result. Re-run generation with the radius
        // removed and report how far apart the closest unjoined pair actually
        // is, which turns the empty answer into a statement about the network:
        // this is a city where a footpath interchange is not an available
        // intervention at any plausible walking distance.
        InterchangeSearchConfig wide = cfg;
        wide.max_walk_m = 1e9;
        try
        {
            const auto all = generate_interchange_candidates(in, wide);
            if (!all.empty())
            {
                const InterchangeCandidate &n = all.front(); // sorted by distance
                std::printf("The closest pair of stations with no service between them is %.0f m "
                            "apart\n(%s <-> %s) -- about a %.0f-minute walk at %.1f m/s.\n",
                            n.distance_m, name_of(n.station_a).c_str(),
                            name_of(n.station_b).c_str(),
                            (n.distance_m / walk_speed) / 60.0, walk_speed);
                std::printf("A footpath interchange is not an available intervention here.\n");
            }
        }
        catch (const std::exception &)
        {
            // Diagnostic only; the empty result above is already the answer.
        }
        return 0;
    }

    std::printf("\n=== Sweeping %zu candidate interchanges (<= %.0f m apart) ===\n",
                candidates.size(), max_walk);
    std::printf("Each one rebuilds the timetable and recomputes the whole surface.\n");
    std::fflush(stdout);

    std::vector<InterchangeEvaluation> ranked;
    try
    {
        ranked = evaluate_interchange_candidates(
            in, candidates, cfg,
            [](std::size_t done, std::size_t total)
            {
                if (done % 10 == 0 || done == total)
                {
                    std::printf("\r  %zu / %zu", done, total);
                    std::fflush(stdout);
                }
            });
    }
    catch (const std::exception &e)
    {
        std::fprintf(stderr, "\nevaluation failed: %s\n", e.what());
        return 1;
    }
    std::printf("\n");

    // ── CSV ──────────────────────────────────────────────────────────────────
    std::ostringstream csv;
    csv << "rank,node_a,stop_id_a,name_a,node_b,stop_id_b,name_b,distance_m,walk_time_s,same_name,"
        << "reach_direct_before,reach_full_before,gap_before,"
        << "reach_direct_after,reach_full_after,gap_after,"
        << "delta_direct,delta_reach,delta_gap\n";
    for (std::size_t i = 0; i < ranked.size(); ++i)
    {
        const InterchangeEvaluation &e = ranked[i];
        csv << (i + 1) << ','
            << e.candidate.station_a << ',' << csv_safe(id_of(e.candidate.station_a)) << ','
            << csv_safe(name_of(e.candidate.station_a)) << ','
            << e.candidate.station_b << ',' << csv_safe(id_of(e.candidate.station_b)) << ','
            << csv_safe(name_of(e.candidate.station_b)) << ','
            << e.candidate.distance_m << ',' << e.candidate.walk_time_s << ','
            << (e.candidate.same_name ? 1 : 0) << ','
            << e.before.direct << ',' << e.before.full << ',' << e.before.gap << ','
            << e.after.direct << ',' << e.after.full << ',' << e.after.gap << ','
            << e.delta_direct << ',' << e.delta_reach << ',' << e.delta_gap << '\n';
    }

    // ── Map: which stations are worth connecting ─────────────────────────────
    //
    // Colour is the best gain any candidate TOUCHING that station achieved, so
    // the map answers "where is an interchange worth building" rather than
    // naming one pair. The winning link is deliberately NOT drawn as an edge:
    // render_station_map_svg draws every link identically, and a proposed
    // footpath painted like existing track would be a picture of a network that
    // does not exist.
    std::unordered_map<uint32_t, double> best_at;
    for (const InterchangeEvaluation &e : ranked)
    {
        auto &a = best_at[e.candidate.station_a];
        a = std::max(a, e.delta_reach);
        auto &b = best_at[e.candidate.station_b];
        b = std::max(b, e.delta_reach);
    }

    auto placements = placements_from_stops(parser.stops(), parser.stop_index_map(), sg.stations);
    std::vector<double> values;
    values.reserve(placements.size());
    for (const NodePlacement &p : placements)
    {
        const auto it = best_at.find(p.node);
        values.push_back(it == best_at.end() ? 0.0 : it->second);
    }

    char sub[512];
    std::snprintf(sub, sizeof(sub),
                  "%s - best gain in stations reachable within %u min (<=%u changes) from one new "
                  "footpath touching this station; %zu candidates <= %.0f m",
                  feed_label(feed_dir).c_str(), threshold_s / 60, max_changes,
                  ranked.size(), max_walk);

    MapStyle style;
    style.title = "Where one new interchange would pay";
    style.subtitle = sub;
    style.legend_label = "extra stations reachable, mean per origin";
    style.labels = labels;

    const std::string csv_path = out_prefix + "-candidates.csv";
    const std::string svg_path = out_prefix + "-sites.svg";
    if (!write_file(csv_path, csv.str()))
        return 1;
    if (!write_file(svg_path, render_station_map_svg(placements, sg.links, values, style)))
        return 1;

    // ── Report ───────────────────────────────────────────────────────────────
    //
    // Same-name pairs are held out of the headline. They are usually one station
    // complex whose transfers.txt does not join all of its platforms, so the
    // interchange already exists and what is missing is a data record. Ranking
    // them alongside genuine proposals would recommend spending money on
    // something already built. @see InterchangeCandidate::same_name
    std::vector<const InterchangeEvaluation *> proposals, feed_gaps;
    for (const InterchangeEvaluation &e : ranked)
        (e.candidate.same_name ? feed_gaps : proposals).push_back(&e);

    auto print_table = [&](const std::vector<const InterchangeEvaluation *> &rows, std::size_t n)
    {
        std::printf("%-34s %-34s %7s %6s %8s %8s\n",
                    "station A", "station B", "metres", "walk", "+reach", "+gap");
        for (std::size_t i = 0; i < std::min(n, rows.size()); ++i)
        {
            const InterchangeEvaluation &e = *rows[i];
            std::printf("%-34.34s %-34.34s %7.0f %5us %+8.2f %+8.2f\n",
                        name_of(e.candidate.station_a).c_str(),
                        name_of(e.candidate.station_b).c_str(),
                        e.candidate.distance_m, e.candidate.walk_time_s,
                        e.delta_reach, e.delta_gap);
        }
    };

    std::printf("\n=== Top %zu proposed interchanges ===\n", std::min(top_n, proposals.size()));
    if (proposals.empty())
        std::printf("  (none: every candidate joined two stations sharing a name)\n");
    else
        print_table(proposals, top_n);

    if (!feed_gaps.empty())
    {
        std::printf("\n=== Held out: %zu pair(s) whose two stations share a name ===\n",
                    feed_gaps.size());
        std::printf("These are probably ONE station complex that the feed's transfers.txt does not\n"
                    "fully join, so the interchange already exists and the missing thing is a data\n"
                    "record. Shown because a feed gap of this size distorts every reachability\n"
                    "number computed from the feed -- not because anything should be built.\n");
        print_table(feed_gaps, std::min<std::size_t>(top_n, 5));
    }

    std::size_t positive = 0;
    for (const InterchangeEvaluation &e : ranked)
        if (e.delta_reach > 0.0)
            ++positive;

    std::printf("\n  candidates that improved reachability at all: %zu of %zu\n",
                positive, ranked.size());
    if (!proposals.empty())
    {
        const InterchangeEvaluation &b = *proposals.front();
        std::printf("  best buy: %s <-> %s  (%.0f m, %u s walk)\n",
                    name_of(b.candidate.station_a).c_str(),
                    name_of(b.candidate.station_b).c_str(),
                    b.candidate.distance_m, b.candidate.walk_time_s);
        std::printf("    mean stations reachable within %u min: %.2f -> %.2f  (%+.2f, %+.1f%%)\n",
                    threshold_s / 60, b.before.full, b.after.full, b.delta_reach,
                    b.before.full > 0.0 ? 100.0 * b.delta_reach / b.before.full : 0.0);
        std::printf("    reachable without changing          : %.2f -> %.2f  (%+.2f)\n",
                    b.before.direct, b.after.direct, b.delta_direct);
        std::printf("    the gap (only reachable by changing): %.2f -> %.2f  (%+.2f)\n",
                    b.before.gap, b.after.gap, b.delta_gap);
    }
    std::printf("\nwrote %s\nwrote %s\n", csv_path.c_str(), svg_path.c_str());
    return 0;
}
