#include "interchange.hpp"

#include "gtfs_parser.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace namma_metro
{

    namespace
    {

        /// Pack an ordered node pair into one key. Both indices are uint32_t, so
        /// this is lossless and the set lookups below stay allocation-free per probe.
        constexpr uint64_t pair_key(uint32_t a, uint32_t b) noexcept
        {
            return (static_cast<uint64_t>(a) << 32) | b;
        }

        /// Lowercased and stripped of leading/trailing whitespace. Deliberately
        /// nothing cleverer: a normaliser that also dropped punctuation would
        /// start merging names that a reader can see are different, and this
        /// flag is only ever advisory. @see InterchangeCandidate::same_name
        std::string normalised_name(const std::string &s)
        {
            std::size_t b = 0, e = s.size();
            while (b < e && std::isspace(static_cast<unsigned char>(s[b])))
                ++b;
            while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1])))
                --e;

            std::string out;
            out.reserve(e - b);
            for (std::size_t i = b; i < e; ++i)
                out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(s[i]))));
            return out;
        }

        void require_inputs(const InterchangeInputs &in)
        {
            if (in.stop_times == nullptr)
                throw std::invalid_argument("InterchangeInputs::stop_times is null");
            if (in.stop_index_map == nullptr)
                throw std::invalid_argument("InterchangeInputs::stop_index_map is null");
            if (in.base_transfers == nullptr)
                throw std::invalid_argument("InterchangeInputs::base_transfers is null");
            if (in.stops == nullptr)
                throw std::invalid_argument("InterchangeInputs::stops is null");
            if (in.station_graph == nullptr)
                throw std::invalid_argument("InterchangeInputs::station_graph is null");
        }

        void require_threshold(const InterchangeSearchConfig &cfg)
        {
            if (cfg.threshold_index >= cfg.accessibility.thresholds_s.size())
                throw std::invalid_argument(
                    "InterchangeSearchConfig::threshold_index is out of range for "
                    "accessibility.thresholds_s");
        }

        /// node index -> the feed record for that node, for coordinates and names.
        /// A node with no record cannot be placed and is therefore not a candidate.
        std::unordered_map<uint32_t, const StopRecord *> records_by_node(const InterchangeInputs &in)
        {
            std::unordered_map<uint32_t, const StopRecord *> out;
            out.reserve(in.stops->size());
            for (const StopRecord &s : *in.stops)
            {
                const auto it = in.stop_index_map->find(s.stop_id);
                if (it != in.stop_index_map->end())
                    out.emplace(it->second, &s);
            }
            return out;
        }

        /// node index -> stop_id, needed because TransferRecord addresses stops by
        /// their GTFS string id rather than by node.
        std::unordered_map<uint32_t, std::string> stop_ids_by_node(const InterchangeInputs &in)
        {
            std::unordered_map<uint32_t, std::string> out;
            out.reserve(in.stop_index_map->size());
            for (const auto &[id, node] : *in.stop_index_map)
                out.emplace(node, id);
            return out;
        }

        /// Mean over origins of the zero-change and full-budget counts at one
        /// threshold. Means rather than totals so the numbers stay comparable
        /// across networks of different sizes.
        SurfaceMeans surface_means(const AccessibilitySurface &s, uint32_t threshold_index)
        {
            SurfaceMeans m;
            if (s.per_origin.empty() || s.num_budgets == 0 || s.num_thresholds == 0)
                return m;

            const std::size_t t = threshold_index;
            const std::size_t full_base =
                static_cast<std::size_t>(s.num_budgets - 1) * s.num_thresholds;

            double direct = 0.0, full = 0.0;
            for (const StationAccessibility &o : s.per_origin)
            {
                direct += o.counts[t];
                full += o.counts[full_base + t];
            }
            const double n = static_cast<double>(s.per_origin.size());
            m.direct = direct / n;
            m.full = full / n;
            m.gap = m.full - m.direct;
            return m;
        }

        /// Build the timetable and measure the surface. `extra` is appended to the
        /// feed's own transfers; empty means the unmodified network.
        ///
        /// Deliberately a full RaptorBuilder::build() rather than a patched copy of
        /// an existing timetable: the footpath relation has to be transitively
        /// re-closed around the new edge, and the builder is the one place in this
        /// repository that knows how. See trap 2 in interchange.hpp.
        SurfaceMeans measure_with(const InterchangeInputs &in,
                                  const InterchangeSearchConfig &cfg,
                                  const std::vector<TransferRecord> &extra)
        {
            std::vector<TransferRecord> transfers = *in.base_transfers;
            transfers.insert(transfers.end(), extra.begin(), extra.end());

            const RaptorTimetable tt = RaptorBuilder::build(
                *in.stop_times, in.num_stops, in.stop_index_map, transfers);

            // Origins and destinations are both the fixed station representatives,
            // so a modification can never change what counts as a place.
            const AccessibilitySurface surface = compute_accessibility(
                tt, in.station_graph->stations, cfg.accessibility, in.station_graph->stations);

            return surface_means(surface, cfg.threshold_index);
        }

    } // namespace

    // ═══════════════════════════════════════════════════════════════════════════

    double haversine_m(double lat_a, double lon_a, double lat_b, double lon_b)
    {
        constexpr double kEarthRadiusM = 6371000.0;
        constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

        const double phi1 = lat_a * kDeg2Rad;
        const double phi2 = lat_b * kDeg2Rad;
        const double dphi = (lat_b - lat_a) * kDeg2Rad;
        const double dlam = (lon_b - lon_a) * kDeg2Rad;

        const double s1 = std::sin(dphi * 0.5);
        const double s2 = std::sin(dlam * 0.5);
        const double h = s1 * s1 + std::cos(phi1) * std::cos(phi2) * s2 * s2;

        // Clamp before asin: rounding can push h a hair above 1 for antipodal
        // inputs, and std::sqrt of a negative 1 - h would be NaN.
        return 2.0 * kEarthRadiusM * std::asin(std::sqrt(std::min(1.0, std::max(0.0, h))));
    }

    // ═══════════════════════════════════════════════════════════════════════════

    std::vector<InterchangeCandidate> generate_interchange_candidates(
        const InterchangeInputs &in, const InterchangeSearchConfig &cfg)
    {
        require_inputs(in);

        const auto by_node = records_by_node(in);
        const std::vector<uint32_t> &stations = in.station_graph->stations;

        // Pairs a vehicle already runs between. A footpath there would be a walk
        // beside a ride, not an interchange.
        std::unordered_set<uint64_t> adjacent;
        adjacent.reserve(in.station_graph->links.size() * 2);
        for (const auto &[u, v] : in.station_graph->links)
        {
            adjacent.insert(pair_key(u, v));
            adjacent.insert(pair_key(v, u));
        }

        // Pairs the feed already joins by transfer. On a well-formed input this
        // can never fire between two DISTINCT station representatives, because
        // stations are the connected components of the transfer graph. Checked
        // anyway: silently proposing an interchange that already exists would be
        // a very quiet way to produce a wrong answer.
        std::unordered_set<uint64_t> already_walkable;
        already_walkable.reserve(in.base_transfers->size() * 2);
        for (const TransferRecord &t : *in.base_transfers)
        {
            const auto f = in.stop_index_map->find(t.from_stop_id);
            const auto g = in.stop_index_map->find(t.to_stop_id);
            if (f == in.stop_index_map->end() || g == in.stop_index_map->end())
                continue;
            if (f->second >= in.station_graph->station_of.size() ||
                g->second >= in.station_graph->station_of.size())
                continue;
            const uint32_t a = in.station_graph->station_of[f->second];
            const uint32_t b = in.station_graph->station_of[g->second];
            already_walkable.insert(pair_key(a, b));
            already_walkable.insert(pair_key(b, a));
        }

        std::vector<InterchangeCandidate> out;

        for (std::size_t i = 0; i < stations.size(); ++i)
        {
            const uint32_t a = stations[i];
            const auto ra = by_node.find(a);
            if (ra == by_node.end())
                continue;

            for (std::size_t j = i + 1; j < stations.size(); ++j)
            {
                const uint32_t b = stations[j];
                const auto rb = by_node.find(b);
                if (rb == by_node.end())
                    continue;

                const uint32_t lo = std::min(a, b), hi = std::max(a, b);
                if (adjacent.count(pair_key(lo, hi)) != 0)
                    continue;
                if (already_walkable.count(pair_key(lo, hi)) != 0)
                    continue;

                const double d = haversine_m(ra->second->stop_lat, ra->second->stop_lon,
                                             rb->second->stop_lat, rb->second->stop_lon);
                if (!(d <= cfg.max_walk_m))
                    continue; // also rejects NaN coordinates

                InterchangeCandidate c;
                c.station_a = lo;
                c.station_b = hi;
                c.distance_m = d;
                c.walk_time_s = std::max(
                    cfg.min_walk_time_s,
                    static_cast<uint32_t>(std::lround(d / std::max(0.1, cfg.walk_speed_mps))));
                c.same_name = normalised_name(ra->second->stop_name) ==
                              normalised_name(rb->second->stop_name);
                out.push_back(c);
            }
        }

        std::sort(out.begin(), out.end(),
                  [](const InterchangeCandidate &x, const InterchangeCandidate &y)
                  {
                      if (x.distance_m != y.distance_m)
                          return x.distance_m < y.distance_m;
                      if (x.station_a != y.station_a)
                          return x.station_a < y.station_a;
                      return x.station_b < y.station_b;
                  });
        return out;
    }

    // ═══════════════════════════════════════════════════════════════════════════

    SurfaceMeans measure_baseline(const InterchangeInputs &in, const InterchangeSearchConfig &cfg)
    {
        require_inputs(in);
        require_threshold(cfg);
        return measure_with(in, cfg, {});
    }

    // ═══════════════════════════════════════════════════════════════════════════

    std::vector<InterchangeEvaluation> evaluate_interchange_candidates(
        const InterchangeInputs &in,
        const std::vector<InterchangeCandidate> &candidates,
        const InterchangeSearchConfig &cfg,
        const std::function<void(std::size_t, std::size_t)> &progress)
    {
        require_inputs(in);
        require_threshold(cfg);

        std::vector<InterchangeEvaluation> out;
        if (candidates.empty())
            return out;

        const SurfaceMeans base = measure_with(in, cfg, {});
        const auto ids = stop_ids_by_node(in);

        out.reserve(candidates.size());
        for (std::size_t k = 0; k < candidates.size(); ++k)
        {
            const InterchangeCandidate &c = candidates[k];

            const auto ia = ids.find(c.station_a);
            const auto ib = ids.find(c.station_b);
            if (ia == ids.end() || ib == ids.end())
                continue; // no stop_id to name it by; cannot be expressed as GTFS

            // Both directions. A one-way footpath would model a turnstile, not a
            // walkway, and would make the result depend on which node happened to
            // get the lower index.
            std::vector<TransferRecord> extra(2);
            extra[0].from_stop_id = ia->second;
            extra[0].to_stop_id = ib->second;
            extra[0].min_transfer_time = c.walk_time_s;
            extra[1].from_stop_id = ib->second;
            extra[1].to_stop_id = ia->second;
            extra[1].min_transfer_time = c.walk_time_s;

            InterchangeEvaluation e;
            e.candidate = c;
            e.before = base;
            e.after = measure_with(in, cfg, extra);
            e.delta_reach = e.after.full - e.before.full;
            e.delta_direct = e.after.direct - e.before.direct;
            e.delta_gap = e.after.gap - e.before.gap;
            out.push_back(e);

            if (progress)
                progress(k + 1, candidates.size());
        }

        std::sort(out.begin(), out.end(),
                  [](const InterchangeEvaluation &x, const InterchangeEvaluation &y)
                  {
                      if (x.delta_reach != y.delta_reach)
                          return x.delta_reach > y.delta_reach; // best first
                      if (x.candidate.walk_time_s != y.candidate.walk_time_s)
                          return x.candidate.walk_time_s < y.candidate.walk_time_s;
                      if (x.candidate.station_a != y.candidate.station_a)
                          return x.candidate.station_a < y.candidate.station_a;
                      return x.candidate.station_b < y.candidate.station_b;
                  });
        return out;
    }

} // namespace namma_metro
