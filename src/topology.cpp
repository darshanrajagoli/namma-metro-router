#include "topology.hpp"
#include "gtfs_parser.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace namma_metro
{

    namespace
    {

        /// Union-find over platform indices. Used twice: to merge platforms into
        /// stations over the transfer graph, and to count connected components of
        /// the resulting station graph.
        class DisjointSet
        {
        public:
            explicit DisjointSet(std::size_t n) : parent_(n), rank_(n, 0)
            {
                for (std::size_t i = 0; i < n; ++i)
                    parent_[i] = static_cast<uint32_t>(i);
            }

            uint32_t find(uint32_t x)
            {
                while (parent_[x] != x)
                {
                    parent_[x] = parent_[parent_[x]]; // path halving
                    x = parent_[x];
                }
                return x;
            }

            void unite(uint32_t a, uint32_t b)
            {
                a = find(a);
                b = find(b);
                if (a == b)
                    return;
                if (rank_[a] < rank_[b])
                    std::swap(a, b);
                parent_[b] = a;
                if (rank_[a] == rank_[b])
                    ++rank_[a];
            }

        private:
            std::vector<uint32_t> parent_;
            std::vector<uint8_t> rank_;
        };

        /// The 07:00-21:00 sampling window used by main_bench.cpp and tools/diag.cpp.
        /// Headway is measured inside it so the number describes the service a
        /// benchmarked query actually sees, not the thin early-morning tail.
        constexpr uint32_t WINDOW_START = 25200;
        constexpr uint32_t WINDOW_END = 75600;

        double median_of(std::vector<double> &v)
        {
            if (v.empty())
                return 0.0;
            std::sort(v.begin(), v.end());
            const std::size_t n = v.size();
            return (n % 2 == 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
        }

    } // namespace

    TopologyMetrics compute_topology(
        const std::vector<StopTimeRecord> &stop_times,
        const std::vector<TripRecord> &trips,
        uint32_t num_stops,
        const std::unordered_map<std::string, uint32_t> *stop_index_map,
        const std::vector<TransferRecord> &transfers,
        StationGraph *out_graph)
    {
        if (stop_index_map == nullptr)
            throw std::invalid_argument(
                "compute_topology(): stop_index_map is required so the metrics "
                "describe the same node numbering as the graph they are compared to.");

        TopologyMetrics m;
        m.platforms = num_stops;
        if (out_graph != nullptr)
        {
            out_graph->station_of.clear();
            out_graph->links.clear();
            out_graph->stations.clear();
        }
        if (num_stops == 0 || stop_times.empty())
            return m;

        // ── Step 1: platforms -> stations, over the transfer graph ────────────
        DisjointSet station_of(num_stops);
        for (const auto &t : transfers)
        {
            const auto a = stop_index_map->find(t.from_stop_id);
            const auto b = stop_index_map->find(t.to_stop_id);
            if (a == stop_index_map->end() || b == stop_index_map->end())
                continue;
            if (a->second >= num_stops || b->second >= num_stops)
                continue;
            station_of.unite(a->second, b->second);
        }

        // ── Step 2: trip_id -> route_id, for the line-based metrics ───────────
        std::unordered_map<std::string, const std::string *> route_of_trip;
        route_of_trip.reserve(trips.size() * 2);
        for (const auto &t : trips)
            route_of_trip.emplace(t.trip_id, &t.route_id);

        // ── Step 3: walk the trips and collect segments ───────────────────────
        std::vector<const StopTimeRecord *> sorted;
        sorted.reserve(stop_times.size());
        for (const auto &r : stop_times)
            sorted.push_back(&r);
        std::sort(sorted.begin(), sorted.end(),
                  [](const StopTimeRecord *a, const StopTimeRecord *b)
                  {
                      if (a->trip_id != b->trip_id)
                          return a->trip_id < b->trip_id;
                      return a->stop_sequence < b->stop_sequence;
                  });

        // Undirected station link -> set of distinct route_ids serving it.
        // std::map keyed on the ordered pair keeps the iteration deterministic;
        // the metrics are order-independent, but a deterministic build makes a
        // diff of two study runs meaningful.
        std::map<std::pair<uint32_t, uint32_t>, std::set<std::string>> link_routes;
        // Directed PLATFORM link -> departure times, for headway.
        //
        // Platform level, not station level. On a feed that keeps platforms
        // separate, the two directions between a pair of stations collapse onto
        // the same directed station pair, so their departures interleave and the
        // reported headway comes out shorter than any train actually runs. A
        // directed platform pair is one track in one direction, which is what
        // "how often does a train run here" means.
        //
        // Measured on the study's own feeds, the difference is real but modest:
        // BART 660s at platform level against 600s at station level, Boston
        // 1980s against 1800s — about 9% in both — and no difference at all on a
        // feed whose platforms are already collapsed. It is not the cause of the
        // zero-second headway this project once reported; that was the feed
        // carrying three service-day variants of every trip at once, and it is
        // fixed in scripts/prefilter_gtfs.py.
        std::map<std::pair<uint32_t, uint32_t>, std::vector<uint32_t>> platform_link_departures;
        std::unordered_set<uint32_t> served_platforms;
        std::unordered_map<uint32_t, std::set<std::string>> station_routes;
        std::unordered_set<std::string> trips_with_segment;
        // route_id -> the set of stations it serves. Used to collapse the two
        // directions of one line, which most feeds publish as two route_ids.
        std::map<std::string, std::set<uint32_t>> route_stations;

        uint32_t segments = 0;
        uint32_t first_dep = UINT32_MAX, last_dep = 0;

        for (std::size_t i = 0; i + 1 < sorted.size(); ++i)
        {
            const StopTimeRecord &from = *sorted[i];
            const StopTimeRecord &to = *sorted[i + 1];
            if (from.trip_id != to.trip_id)
                continue;

            // Exactly GraphBuilder's admission rules — the metrics must describe
            // the network the router actually sees, not the raw feed.
            if (from.pickup_type == 1 || to.drop_off_type == 1)
                continue;
            if (from.departure_time == UINT32_MAX || to.arrival_time == UINT32_MAX)
                continue;
            if (to.arrival_time < from.departure_time)
                continue;

            const auto it_u = stop_index_map->find(from.stop_id);
            const auto it_v = stop_index_map->find(to.stop_id);
            if (it_u == stop_index_map->end() || it_v == stop_index_map->end())
                continue;
            if (it_u->second >= num_stops || it_v->second >= num_stops)
                continue;

            ++segments;
            served_platforms.insert(it_u->second);
            served_platforms.insert(it_v->second);
            trips_with_segment.insert(from.trip_id);
            first_dep = std::min(first_dep, from.departure_time);
            last_dep = std::max(last_dep, from.departure_time);

            const uint32_t su = station_of.find(it_u->second);
            const uint32_t sv = station_of.find(it_v->second);

            const std::string *route_id = nullptr;
            if (const auto rt = route_of_trip.find(from.trip_id); rt != route_of_trip.end())
            {
                route_id = rt->second;
                station_routes[su].insert(*route_id);
                station_routes[sv].insert(*route_id);
                auto &served = route_stations[*route_id];
                served.insert(su);
                served.insert(sv);
            }

            // A segment between two platforms of the SAME station is a shunting
            // move, not a link a passenger can choose. It still counts as
            // service (above) and the route still serves that station, but it
            // contributes no edge to the station graph — and, below, no headway
            // either, so that the frequency metric describes exactly the links
            // the structural metrics report and not a superset of them.
            if (su == sv)
                continue;

            platform_link_departures[{it_u->second, it_v->second}].push_back(from.departure_time);

            // operator[] creates the link entry even when the trip's route_id is
            // unknown, so a feed without trips.txt still yields a correct link
            // count with an empty route set (the line metrics then read zero).
            const auto key = std::minmax(su, sv);
            auto &routes_here = link_routes[{key.first, key.second}];
            if (route_id != nullptr)
                routes_here.insert(*route_id);
        }

        m.segments = segments;
        m.trips = static_cast<uint32_t>(trips_with_segment.size());
        m.platforms_served = static_cast<uint32_t>(served_platforms.size());
        if (first_dep != UINT32_MAX && last_dep >= first_dep)
            m.service_span_hours = (last_dep - first_dep) / 3600.0;

        // ── Step 4: the station graph ─────────────────────────────────────────
        std::unordered_set<uint32_t> stations;
        for (const uint32_t p : served_platforms)
            stations.insert(station_of.find(p));
        m.stations = static_cast<uint32_t>(stations.size());
        m.links = static_cast<uint32_t>(link_routes.size());

        std::unordered_map<uint32_t, uint32_t> degree;
        for (const uint32_t s : stations)
            degree[s] = 0;
        for (const auto &kv : link_routes)
        {
            ++degree[kv.first.first];
            ++degree[kv.first.second];
        }

        uint64_t degree_sum = 0;
        for (const auto &kv : degree)
        {
            const uint32_t d = kv.second;
            degree_sum += d;
            m.max_station_degree = std::max(m.max_station_degree, d);
            if (d == 1)
                ++m.deg1_stations;
            else if (d == 2)
                ++m.deg2_stations;
            else if (d >= 3)
                ++m.deg3plus_stations;
        }
        if (m.stations > 0)
        {
            m.mean_station_degree = static_cast<double>(degree_sum) / m.stations;
            m.deg3plus_fraction = static_cast<double>(m.deg3plus_stations) / m.stations;
        }

        // Components of the station graph, over SERVED stations only. An
        // isolated served station (possible only if all its segments were
        // same-station shunts) counts as its own component, which is what the
        // cyclomatic formula requires.
        {
            DisjointSet comp(num_stops);
            for (const auto &kv : link_routes)
                comp.unite(kv.first.first, kv.first.second);
            std::unordered_set<uint32_t> roots;
            for (const uint32_t s : stations)
                roots.insert(comp.find(s));
            m.components = static_cast<uint32_t>(roots.size());
        }

        m.cyclomatic = static_cast<int64_t>(m.links) - static_cast<int64_t>(m.stations) + static_cast<int64_t>(m.components);

        // Kansky's indices are defined for planar graphs with at least three
        // vertices; below that the denominators are zero or negative and the
        // index is meaningless rather than extreme. Report 0 and let the study
        // exclude those networks explicitly.
        if (m.stations > 2)
        {
            const double S = m.stations;
            m.alpha_index = (2.0 * S - 5.0) > 0.0 ? static_cast<double>(m.cyclomatic) / (2.0 * S - 5.0) : 0.0;
            m.gamma_index = static_cast<double>(m.links) / (3.0 * (S - 2.0));
        }
        if (m.stations > 0)
            m.beta_index = static_cast<double>(m.links) / m.stations;

        // ── Step 5: lines, interchange, overlap ───────────────────────────────
        //
        // A "line" is NOT a route_id. Most feeds publish each direction as its
        // own route_id — BART's twelve route_ids are six lines, there and back —
        // and counting those directly makes every station on any line look like
        // an interchange, which pushed BART's interchange density to exactly
        // 1.000 and made the metric useless.
        //
        // Two route_ids that serve the same SET of stations are treated as one
        // line. That collapses the two directions, keeps short-turn variants
        // grouped (they already share a route_id), and needs nothing from the
        // feed beyond what is already loaded. It would merge two genuinely
        // different services over an identical station set, which is rare and
        // arguably the right answer anyway.
        std::map<std::set<uint32_t>, uint32_t> line_of_station_set;
        std::unordered_map<std::string, uint32_t> line_of_route;
        for (const auto &kv : route_stations)
        {
            const auto [it, inserted] = line_of_station_set.emplace(
                kv.second, static_cast<uint32_t>(line_of_station_set.size()));
            (void)inserted;
            line_of_route[kv.first] = it->second;
        }
        m.lines = static_cast<uint32_t>(line_of_station_set.size());

        auto lines_of = [&](const std::set<std::string> &route_ids)
        {
            std::set<uint32_t> out;
            for (const auto &r : route_ids)
                if (const auto it = line_of_route.find(r); it != line_of_route.end())
                    out.insert(it->second);
            return out;
        };

        for (const uint32_t s : stations)
        {
            const auto it = station_routes.find(s);
            if (it != station_routes.end() && lines_of(it->second).size() >= 2)
                ++m.interchange_stations;
        }
        if (m.stations > 0)
            m.interchange_density = static_cast<double>(m.interchange_stations) / m.stations;

        if (!link_routes.empty())
        {
            uint64_t total = 0, shared = 0;
            for (const auto &kv : link_routes)
            {
                const std::size_t n = lines_of(kv.second).size();
                total += n;
                if (n >= 2)
                    ++shared;
            }
            m.mean_lines_per_link = static_cast<double>(total) / link_routes.size();
            m.shared_link_fraction = static_cast<double>(shared) / link_routes.size();
        }

        // ── Step 6: headway ───────────────────────────────────────────────────
        {
            std::vector<double> per_link;
            per_link.reserve(platform_link_departures.size());
            for (auto &kv : platform_link_departures)
            {
                std::vector<uint32_t> deps;
                deps.reserve(kv.second.size());
                for (const uint32_t d : kv.second)
                    if (d >= WINDOW_START && d <= WINDOW_END)
                        deps.push_back(d);
                if (deps.size() < 2)
                    continue;
                std::sort(deps.begin(), deps.end());
                std::vector<double> gaps;
                gaps.reserve(deps.size() - 1);
                for (std::size_t i = 1; i < deps.size(); ++i)
                    gaps.push_back(static_cast<double>(deps[i] - deps[i - 1]));
                per_link.push_back(median_of(gaps));
            }
            m.median_link_headway_s = median_of(per_link);
        }

        // ── Step 7: hand back the graph these metrics describe ────────────────
        if (out_graph != nullptr)
        {
            out_graph->station_of.assign(num_stops, 0);
            for (uint32_t p = 0; p < num_stops; ++p)
                out_graph->station_of[p] = station_of.find(p);
            out_graph->links.reserve(link_routes.size());
            for (const auto &kv : link_routes)
                out_graph->links.push_back(kv.first); // std::map: already sorted
            out_graph->stations.assign(stations.begin(), stations.end());
            std::sort(out_graph->stations.begin(), out_graph->stations.end());
        }

        return m;
    }

    std::string TopologyMetrics::summary() const
    {
        char buf[512];
        std::snprintf(buf, sizeof(buf),
                      "%u stations (%u platforms), %u links, %u components, cyclomatic %lld, "
                      "%u lines, interchange density %.3f, median headway %.0fs",
                      stations, platforms, links, components,
                      static_cast<long long>(cyclomatic), lines,
                      interchange_density, median_link_headway_s);
        return std::string(buf);
    }

    std::string topology_csv_header()
    {
        return "platforms,platforms_served,stations,links,components,cyclomatic,"
               "alpha_index,beta_index,gamma_index,"
               "deg1_stations,deg2_stations,deg3plus_stations,deg3plus_fraction,"
               "mean_station_degree,max_station_degree,"
               "lines,interchange_stations,interchange_density,"
               "mean_lines_per_link,shared_link_fraction,"
               "trips,segments,service_span_hours,median_link_headway_s";
    }

    std::string topology_csv_row(const TopologyMetrics &m)
    {
        char buf[768];
        std::snprintf(buf, sizeof(buf),
                      "%u,%u,%u,%u,%u,%lld,"
                      "%.6f,%.6f,%.6f,"
                      "%u,%u,%u,%.6f,"
                      "%.6f,%u,"
                      "%u,%u,%.6f,"
                      "%.6f,%.6f,"
                      "%u,%u,%.3f,%.1f",
                      m.platforms, m.platforms_served, m.stations, m.links, m.components,
                      static_cast<long long>(m.cyclomatic),
                      m.alpha_index, m.beta_index, m.gamma_index,
                      m.deg1_stations, m.deg2_stations, m.deg3plus_stations, m.deg3plus_fraction,
                      m.mean_station_degree, m.max_station_degree,
                      m.lines, m.interchange_stations, m.interchange_density,
                      m.mean_lines_per_link, m.shared_link_fraction,
                      m.trips, m.segments, m.service_span_hours, m.median_link_headway_s);
        return std::string(buf);
    }

} // namespace namma_metro
