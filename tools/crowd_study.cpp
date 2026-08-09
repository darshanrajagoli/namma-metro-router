// tools/crowd_study.cpp — does a crowd model built from real ridership make the
// second objective load-bearing, where the synthetic one did not?
//
// THE BACKGROUND
// ══════════════
// The engine's original second objective is a Gaussian in time of day and
// nothing else, so every edge departing at the same second carries the same
// crowd cost wherever it is in the city. tools/diag.cpp measured the
// consequence: a single-label frontier at 96-100% of nodes. A second objective
// that is constant across space cannot distinguish two routes.
//
// include/crowd_model.hpp replaces it with a surface over (station, hour) built
// from BMRCL's station-wise hourly entry counts, obtained under the Right to
// Information Act and republished as open data. That field DOES vary in space.
// This binary runs both models over the same feed in the same process and
// reports what changes.
//
// WHAT TO EXPECT, STATED BEFORE MEASURING SO THE RESULT CANNOT BE READ BACKWARDS
// ═════════════════════════════════════════════════════════════════════════════
// A spatially varying objective is NECESSARY for a route-choice trade-off. It
// is not sufficient. On a network whose station graph is a forest there is
// exactly one path between any pair of stations, so there is no second route
// for a second objective to prefer, whatever that objective measures. Namma
// Metro's station graph has a cyclomatic number of zero (tools/study.cpp
// reports it). So the prediction is:
//
//   - the frontier stays single-label, because the topology forbids anything else;
//   - but lambda now changes the chosen DEPARTURE differently at different
//     stations, which the time-only model could not do;
//   - and the crowd cost accumulated along a journey becomes a real number that
//     differs between origins, which it previously could not.
//
// If the first prediction fails, the tree argument is wrong and that is the more
// interesting outcome. Either way it is measured rather than asserted.
//
// USAGE
//   routing_engine_crowd_study <feed_dir> --ridership station-hourly.csv
//                              [--aliases aliases.csv] [--queries N] [--seed N]

#include "crowd_model.hpp"
#include "graph.hpp"
#include "gtfs_parser.hpp"
#include "routing.hpp"
#include "topology.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <random>
#include <string>
#include <vector>

using namespace namma_metro;

namespace
{

    constexpr uint32_t WINDOW_START = 25200;
    constexpr uint32_t WINDOW_END = 75600;

    struct FrontierStats
    {
        std::map<std::size_t, uint64_t> hist;
        uint64_t reached = 0, k2plus = 0, label_sum = 0;
        std::size_t max_k = 0;
        double mean_secondary = 0.0;
        uint64_t secondary_samples = 0;
    };

    struct Query
    {
        uint32_t src, dep;
    };

    FrontierStats measure(const CSRGraph &g, const LookaheadConfig &cfg,
                          const std::vector<Query> &queries)
    {
        FrontierStats s;
        ParetoDijkstra router(g, cfg);
        QueryResult out;
        double secondary_sum = 0.0;
        for (const auto &q : queries)
        {
            router.run(q.src, q.dep, out);
            for (uint32_t v = 0; v < g.num_nodes; ++v)
            {
                if (v == q.src)
                    continue;
                const auto &labels = out.pareto_sets[v].labels();
                if (labels.empty())
                    continue;
                ++s.reached;
                const std::size_t k = labels.size();
                ++s.hist[k];
                s.label_sum += k;
                s.max_k = std::max(s.max_k, k);
                if (k > 1)
                    ++s.k2plus;
                secondary_sum += labels.front()->secondary_cost;
                ++s.secondary_samples;
            }
        }
        if (s.secondary_samples)
            s.mean_secondary = secondary_sum / s.secondary_samples;
        return s;
    }

    void print_frontier(const char *label, const FrontierStats &s)
    {
        std::printf("  %-28s reached %llu | k>1 %.3f%% | max k %zu | mean k %.4f | mean crowd %.1f\n",
                    label,
                    static_cast<unsigned long long>(s.reached),
                    s.reached ? 100.0 * static_cast<double>(s.k2plus) / static_cast<double>(s.reached) : 0.0,
                    s.max_k,
                    s.reached ? static_cast<double>(s.label_sum) / static_cast<double>(s.reached) : 0.0,
                    s.mean_secondary);
    }

    /// How much does lambda change the answer? Compares each node's
    /// earliest-arrival label against the lambda = 0 reference.
    struct LambdaEffect
    {
        double arrival_differs_pct = 0.0;
        double crowd_differs_pct = 0.0;
        double mean_dt = 0.0;
        double mean_dcrowd = 0.0;
    };

    LambdaEffect lambda_sweep(const CSRGraph &g, const std::vector<Query> &queries, float lambda)
    {
        LookaheadConfig ref_cfg{.k_departures = 5, .W_max_seconds = 1800, .lambda = 0.0f};
        LookaheadConfig cur_cfg{.k_departures = 5, .W_max_seconds = 1800, .lambda = lambda};
        ParetoDijkstra ref_router(g, ref_cfg), cur_router(g, cur_cfg);
        QueryResult ref_out, cur_out;

        uint64_t cmp = 0, diff_arr = 0, diff_crowd = 0;
        double sum_dt = 0.0, sum_dc = 0.0;

        // Snapshot buffers, allocated once and refilled per query.
        std::vector<std::pair<uint32_t, uint32_t>> ref_vals(g.num_nodes, {0, 0});
        std::vector<uint8_t> ref_ok(g.num_nodes, 0);

        for (const auto &q : queries)
        {
            ref_router.run(q.src, q.dep, ref_out);
            // Snapshot the reference by VALUE rather than holding the Label*.
            // Those pointers live in a router's arena and are invalidated by
            // that router's next run(). It happens that the two arms use two
            // routers with two arenas, so reading them later would work here —
            // and that is exactly the kind of accidental safety that breaks the
            // day someone merges the two loops. Copy the four bytes.
            std::fill(ref_ok.begin(), ref_ok.end(), 0);
            for (uint32_t v = 0; v < g.num_nodes; ++v)
            {
                const auto &L = ref_out.pareto_sets[v].labels();
                if (L.empty())
                    continue;
                ref_vals[v] = {L.front()->arrival_time, L.front()->secondary_cost};
                ref_ok[v] = 1;
            }

            cur_router.run(q.src, q.dep, cur_out);
            for (uint32_t v = 0; v < g.num_nodes; ++v)
            {
                const auto &L = cur_out.pareto_sets[v].labels();
                if (!ref_ok[v] || L.empty())
                    continue;
                ++cmp;
                const uint32_t a = L.front()->arrival_time, c = L.front()->secondary_cost;
                if (a != ref_vals[v].first)
                    ++diff_arr;
                if (c != ref_vals[v].second)
                    ++diff_crowd;
                sum_dt += static_cast<double>(a) - ref_vals[v].first;
                sum_dc += static_cast<double>(c) - ref_vals[v].second;
            }
        }

        LambdaEffect e;
        if (cmp)
        {
            e.arrival_differs_pct = 100.0 * static_cast<double>(diff_arr) / static_cast<double>(cmp);
            e.crowd_differs_pct = 100.0 * static_cast<double>(diff_crowd) / static_cast<double>(cmp);
            e.mean_dt = sum_dt / static_cast<double>(cmp);
            e.mean_dcrowd = sum_dc / static_cast<double>(cmp);
        }
        return e;
    }

} // namespace

int main(int argc, char **argv)
{
    std::string feed_dir, ridership_csv, aliases_csv;
    uint32_t n_queries = 300, seed = 123;

    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        if (a == "--ridership" && i + 1 < argc)
            ridership_csv = argv[++i];
        else if (a == "--aliases" && i + 1 < argc)
            aliases_csv = argv[++i];
        else if (a == "--queries" && i + 1 < argc)
            n_queries = static_cast<uint32_t>(std::stoul(argv[++i]));
        else if (a == "--seed" && i + 1 < argc)
            seed = static_cast<uint32_t>(std::stoul(argv[++i]));
        else if (a.rfind("--", 0) == 0)
        {
            std::fprintf(stderr, "unknown option %s\n", a.c_str());
            return 2;
        }
        else
            feed_dir = a;
    }
    if (feed_dir.empty() || ridership_csv.empty())
    {
        std::fprintf(stderr,
                     "usage: routing_engine_crowd_study <feed_dir> --ridership FILE.csv "
                     "[--aliases FILE.csv] [--queries N] [--seed N]\n\n"
                     "The ridership file is required and is never substituted with a default:\n"
                     "the point of this tool is to replace an invented crowd model with a\n"
                     "measured one, and inventing a fallback would defeat it.\n"
                     "  scripts/fetch_ridership.py downloads the BMRCL file.\n");
        return 2;
    }

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

    // Force the crowd objective. A feed that ships transfers.txt would otherwise
    // build a transfer-count graph, and writing crowd scores into that would
    // rename the objective without changing its name.
    CSRGraph g_gauss = GraphBuilder::build_with_transfers(
        parser.stop_times(), num_stops, &parser.stop_index_map(), parser.transfers(),
        SecondObjective::CrowdExposure);
    CSRGraph g_real = g_gauss; // same topology, same departures; only weights differ

    const TopologyMetrics topo = compute_topology(
        parser.stop_times(), parser.trips(), num_stops, &parser.stop_index_map(), parser.transfers());

    std::printf("╔══════════════════════════════════════════════════════════════════╗\n");
    std::printf("║  Crowd model A/B — synthetic Gaussian vs measured ridership       ║\n");
    std::printf("╚══════════════════════════════════════════════════════════════════╝\n\n");
    std::printf("feed     : %s\n", feed_dir.c_str());
    std::printf("graph    : %u nodes, %u edges (second objective forced to crowd exposure)\n",
                g_gauss.num_nodes, g_gauss.num_edges);
    std::printf("topology : %s\n", topo.summary().c_str());
    std::printf("           cyclomatic number %lld — %s\n\n",
                static_cast<long long>(topo.cyclomatic),
                topo.cyclomatic == 0
                    ? "a FOREST: exactly one path between any two stations, so no second "
                      "objective of any kind can create a route-choice trade-off"
                    : "contains cycles, so alternative routes exist");

    // ── Load the measured surface ────────────────────────────────────────────
    HourlyLoadSurface surface;
    try
    {
        const auto aliases = aliases_csv.empty()
                                 ? std::unordered_map<std::string, std::string>{}
                                 : load_station_aliases(aliases_csv);
        surface = load_hourly_ridership_csv(ridership_csv, parser.stops(),
                                            parser.stop_index_map(), aliases);
    }
    catch (const std::exception &e)
    {
        std::fprintf(stderr, "ridership load failed: %s\n", e.what());
        return 1;
    }

    std::printf("=== Measured load surface ===\n");
    std::printf("  file           : %s\n", surface.source_path.c_str());
    std::printf("  rows           : %u (%u unmatched)\n", surface.rows_read, surface.rows_unmatched);
    std::printf("  period         : %s .. %s (%u distinct dates, averaged)\n",
                surface.first_date.c_str(), surface.last_date.c_str(), surface.days_averaged);
    std::printf("  stops matched  : %u of %u (%.1f%% coverage)\n",
                surface.stops_matched, surface.num_stops, 100.0 * surface.coverage());
    std::printf("  peak cell      : %.1f entries in one station-hour\n", surface.peak());
    if (!surface.unmatched_ridership_names.empty())
    {
        std::printf("  ridership names with no matching stop (%zu):\n",
                    surface.unmatched_ridership_names.size());
        for (const auto &n : surface.unmatched_ridership_names)
            std::printf("      %s\n", n.c_str());
    }
    if (!surface.unmatched_gtfs_stops.empty())
    {
        std::printf("  feed stops with no ridership data (%zu):\n", surface.unmatched_gtfs_stops.size());
        for (const auto &n : surface.unmatched_gtfs_stops)
            std::printf("      %s\n", n.c_str());
    }
    std::printf("\n");

    CrowdApplyReport rep;
    try
    {
        rep = apply_hourly_crowd(g_real, surface, {});
    }
    catch (const std::exception &e)
    {
        std::fprintf(stderr, "applying the crowd surface failed: %s\n", e.what());
        return 1;
    }
    std::printf("=== Applying it ===\n");
    std::printf("  edges rewritten            : %u (%u from stops with no data)\n",
                rep.edges_rewritten, rep.edges_from_unmatched_stop);
    std::printf("  weight range               : max %.0f, mean %.1f  (Gaussian model: 100..1000)\n",
                rep.max_weight, rep.mean_weight);
    std::printf("  steepest fall per second   : %.4f  (FIFO needs <= 1.0) -> %s\n\n",
                rep.max_negative_slope, rep.fifo_bound_holds ? "holds" : "VIOLATED");

    // ── Query set, screened once and shared by both arms ─────────────────────
    std::mt19937 rng(seed);
    std::uniform_int_distribution<uint32_t> snode(0, g_gauss.num_nodes - 1);
    std::uniform_int_distribution<uint32_t> stime(WINDOW_START, WINDOW_END);
    LookaheadConfig screen{.k_departures = 5, .W_max_seconds = 1800, .lambda = 1.0f};
    ParetoDijkstra probe(g_gauss, screen);
    QueryResult probe_out;

    std::vector<Query> queries;
    for (uint32_t a = 0; a < n_queries * 30 + 200 && queries.size() < n_queries; ++a)
    {
        const uint32_t s = snode(rng), t = stime(rng);
        probe.run(s, t, probe_out);
        bool any = false;
        for (uint32_t v = 0; v < g_gauss.num_nodes && !any; ++v)
            if (v != s && !probe_out.pareto_sets[v].empty())
                any = true;
        if (any)
            queries.push_back({s, t});
    }
    if (queries.empty())
    {
        std::fprintf(stderr, "no reachable query pairs\n");
        return 1;
    }
    std::printf("query set : %zu reachable (origin, departure) pairs, seed %u\n\n", queries.size(), seed);

    // ── The frontier, under each model ───────────────────────────────────────
    LookaheadConfig cfg{.k_departures = 5, .W_max_seconds = 1800, .lambda = 1.0f};
    std::printf("=== Pareto frontier (lambda = 1.0) ===\n");
    const FrontierStats fa = measure(g_gauss, cfg, queries);
    const FrontierStats fb = measure(g_real, cfg, queries);
    print_frontier("A. Gaussian, time only", fa);
    print_frontier("B. measured, station x hour", fb);
    std::printf("\n");

    // ── Lambda sensitivity, under each model ─────────────────────────────────
    std::printf("=== Lambda sensitivity, against the lambda = 0 route ===\n");
    std::printf("  %-10s %-28s %10s %10s %12s %14s\n",
                "lambda", "model", "arr diff", "crowd diff", "mean dt (s)", "mean dcrowd");
    for (const float lam : {0.5f, 1.0f, 5.0f, 50.0f, 1000.0f})
    {
        const LambdaEffect ea = lambda_sweep(g_gauss, queries, lam);
        const LambdaEffect eb = lambda_sweep(g_real, queries, lam);
        std::printf("  %-10.1f %-28s %9.3f%% %9.3f%% %12.1f %14.1f\n",
                    lam, "A. Gaussian", ea.arrival_differs_pct, ea.crowd_differs_pct,
                    ea.mean_dt, ea.mean_dcrowd);
        std::printf("  %-10s %-28s %9.3f%% %9.3f%% %12.1f %14.1f\n",
                    "", "B. measured", eb.arrival_differs_pct, eb.crowd_differs_pct,
                    eb.mean_dt, eb.mean_dcrowd);
    }

    std::printf("\n=== Reading this ===\n");
    if (fb.k2plus == 0 && topo.cyclomatic == 0)
        std::printf(
            "  The frontier is still single-label everywhere, and the station graph is a\n"
            "  forest (cyclomatic number 0). Those two facts are the same fact: with one\n"
            "  path between any two stations there is no alternative ROUTE to trade against,\n"
            "  so no second objective can populate the frontier here. What the measured model\n"
            "  changes is the crowd cost attached to a journey and which DEPARTURE lambda\n"
            "  selects — real effects, visible in the sweep above, that the time-only model\n"
            "  could not produce because it was identical at every station.\n");
    else if (fb.k2plus > fa.k2plus)
        std::printf(
            "  The measured model produces a trade-off at %.3f%% of reached nodes where the\n"
            "  Gaussian produced one at %.3f%%. A spatially varying objective made the\n"
            "  machinery load-bearing on this network.\n",
            100.0 * static_cast<double>(fb.k2plus) / static_cast<double>(fb.reached ? fb.reached : 1),
            100.0 * static_cast<double>(fa.k2plus) / static_cast<double>(fa.reached ? fa.reached : 1));
    else
        std::printf(
            "  The measured model did not increase the trade-off rate on this network.\n"
            "  Check the cyclomatic number above before concluding anything about the model:\n"
            "  a network with cycles and no trade-off is a different and stronger finding\n"
            "  than a network without cycles, where none is possible.\n");

    return 0;
}
