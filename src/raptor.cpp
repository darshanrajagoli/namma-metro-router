#include "raptor.hpp"
#include "gtfs_parser.hpp"

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <map>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace namma_metro
{

    // ═══════════════════════════════════════════════════════════════════════════
    // § 1.  RaptorBuilder
    // ═══════════════════════════════════════════════════════════════════════════
    //
    // Three stages, in this order, and the order matters:
    //
    //   (a) Cut each GTFS trip into maximal runs of segments that GraphBuilder
    //       would also have admitted. A trip with one bad segment becomes two
    //       runs, not zero, because the graph model keeps the good segments.
    //   (b) Group runs by their exact stop sequence. That is RAPTOR's notion of
    //       a route, and it is finer than GTFS's route_id.
    //   (c) Split each group further until no trip overtakes another, because
    //       the query's "earliest catchable trip" search is a binary search over
    //       trip index and that is only valid on a totally ordered group.
    //
    // Everything is emitted in a deterministic order — patterns in sorted stop-
    // sequence order, trips in sorted time order with trip_id as the final tie-
    // break. The same reasoning as the std::sort tie-break in graph_builder.cpp:
    // an arbitrary order still produces correct answers, but it makes two runs
    // of the same binary disagree on which of two tied journeys is reported,
    // which silently invalidates any measurement taken from them.

    namespace
    {

        /// One maximal run of admissible segments carved out of one GTFS trip.
        struct RawRun
        {
            std::vector<uint32_t> stops;
            std::vector<RaptorTimetable::StopTime> times;
            const std::string *trip_id; ///< Borrowed; used only as a sort tie-break.
        };

        /// Pointwise "no later than" over a whole run — the non-overtaking test.
        bool no_later_than(const std::vector<RaptorTimetable::StopTime> &a,
                           const std::vector<RaptorTimetable::StopTime> &b) noexcept
        {
            assert(a.size() == b.size());
            for (std::size_t i = 0; i < a.size(); ++i)
                if (a[i].arrival > b[i].arrival || a[i].departure > b[i].departure)
                    return false;
            return true;
        }

    } // namespace

    RaptorTimetable RaptorBuilder::build(
        const std::vector<StopTimeRecord> &stop_times,
        uint32_t num_stops,
        const std::unordered_map<std::string, uint32_t> *stop_index_map,
        const std::vector<TransferRecord> &transfers)
    {
        // No local-index fallback, deliberately. A RaptorTimetable is only ever
        // used next to a CSRGraph built from the parser's index; a locally
        // rebuilt index would number the same stops differently, and the two
        // structures would then describe different networks while every node id
        // still looked valid. GraphBuilder tolerates a null map for the benefit
        // of unit tests that build both sides from the same source; here the
        // whole point is cross-structure comparison, so it is an error.
        if (stop_index_map == nullptr)
            throw std::invalid_argument(
                "RaptorBuilder::build(): stop_index_map is required. Pass "
                "parser.stop_index_map() — the same map used for the CSRGraph, or "
                "the two structures will index different networks.");

        if (!stop_times.empty() && num_stops == 0)
            throw std::invalid_argument(
                "RaptorBuilder::build(): num_stops == 0 with non-empty stop_times. "
                "Pass parser.stops().size().");

        RaptorTimetable tt;
        tt.num_stops = num_stops;
        tt.stop_route_offset.assign(static_cast<std::size_t>(num_stops) + 1, 0);

        // ── Stage (a): sort by (trip_id, stop_sequence), then cut into runs ────
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

        std::vector<RawRun> runs;
        uint32_t unresolved = 0;
        uint32_t trips_split = 0;

        std::size_t i = 0;
        while (i < sorted.size())
        {
            std::size_t j = i;
            const std::string &trip = sorted[i]->trip_id;
            while (j < sorted.size() && sorted[j]->trip_id == trip)
                ++j;

            // Walk the trip's stops, extending the current run while the next
            // segment satisfies exactly GraphBuilder's admission rules, and
            // closing it otherwise.
            RawRun run;
            run.trip_id = &trip;
            bool run_open = false;
            uint32_t eff_prev_dep = 0; // departure of the run's last stop

            // A run is only ever opened by pushing BOTH endpoints of an
            // admissible segment, so an open run always holds at least two
            // stops and is always usable.
            auto close_run = [&]()
            {
                if (run_open)
                {
                    assert(run.stops.size() >= 2);
                    runs.push_back(run);
                }
                run.stops.clear();
                run.times.clear();
                run_open = false;
            };

            for (std::size_t k = i; k + 1 < j; ++k)
            {
                const StopTimeRecord &from = *sorted[k];
                const StopTimeRecord &to = *sorted[k + 1];

                const auto it_u = stop_index_map->find(from.stop_id);
                const auto it_v = stop_index_map->find(to.stop_id);
                const bool resolvable = (it_u != stop_index_map->end() && it_v != stop_index_map->end());
                if (!resolvable)
                    ++unresolved;

                const bool admissible =
                    resolvable &&
                    it_u->second < num_stops && it_v->second < num_stops &&
                    from.pickup_type != 1 && to.drop_off_type != 1 &&
                    from.departure_time != UINT32_MAX && to.arrival_time != UINT32_MAX &&
                    to.arrival_time >= from.departure_time;

                if (!admissible)
                {
                    if (run_open)
                    {
                        ++trips_split;
                        close_run();
                    }
                    continue;
                }

                if (!run_open)
                {
                    // Open a run at `from`. Its arrival is unused (we board
                    // here), so it is set to the departure to keep the array
                    // monotone and any accidental read harmless.
                    run.stops.push_back(it_u->second);
                    run.times.push_back({from.departure_time, from.departure_time});
                    eff_prev_dep = from.departure_time;
                    run_open = true;
                }

                // Same one-second clamp GraphBuilder applies, so the two models
                // agree to the second on segments of zero nominal duration.
                const uint32_t eff_arr = std::max(to.arrival_time, eff_prev_dep + 1u);

                // A passenger riding through `to` leaves on this same trip only
                // if the trip is still there when they arrive. If the feed says
                // otherwise, the graph model cannot ride through either — the
                // outgoing edge departs before the incoming edge lands — so the
                // run must end at `to` and a new one start there.
                const bool can_ride_through = (to.departure_time != UINT32_MAX) && (to.departure_time >= eff_arr);

                run.stops.push_back(it_v->second);
                run.times.push_back({eff_arr, can_ride_through ? to.departure_time : eff_arr});
                eff_prev_dep = to.departure_time;

                if (!can_ride_through)
                {
                    ++trips_split;
                    close_run();
                }
            }
            close_run();

            i = j;
        }

        tt.rows_unresolved_stop = unresolved;
        tt.trips_split_for_bad_segment = trips_split;

        if (runs.empty())
        {
            // A feed with no usable trip still gets a well-formed, empty
            // timetable: offsets present and consistent, so every accessor is
            // safe and the caller does not need a special case.
            tt.route_stop_offset.assign(1, 0);
            tt.route_time_offset.assign(1, 0);
            if (!transfers.empty())
                std::fprintf(stderr,
                             "[RAPTOR INFO] no admissible trips; transfer layer omitted "
                             "(footpaths alone cannot form a journey).\n");
            return tt;
        }

        // ── Stage (b): group runs by exact stop sequence ───────────────────────
        // std::map, not unordered_map: ordering the patterns by their stop
        // sequence makes route ids a deterministic function of the feed.
        std::map<std::vector<uint32_t>, std::vector<std::size_t>> patterns;
        for (std::size_t idx = 0; idx < runs.size(); ++idx)
            patterns[runs[idx].stops].push_back(idx);

        // ── Stage (c): split each pattern until no trip overtakes another ─────
        // Greedy first-fit into bins. Within a bin, every trip is pointwise no
        // later than the next one added, and "pointwise no later" is transitive,
        // so checking only against the bin's last trip is sufficient. In the
        // overwhelmingly common case a pattern needs exactly one bin, and the
        // loop costs one comparison per trip.
        struct Route
        {
            const std::vector<uint32_t> *stops;
            std::vector<std::size_t> trips; // indices into `runs`
        };
        std::vector<Route> routes;
        uint32_t split_count = 0;

        for (auto &kv : patterns)
        {
            const std::vector<uint32_t> &pattern = kv.first;
            std::vector<std::size_t> &members = kv.second;

            std::sort(members.begin(), members.end(),
                      [&runs](std::size_t a, std::size_t b)
                      {
                          const auto &ta = runs[a].times;
                          const auto &tb = runs[b].times;
                          for (std::size_t s = 0; s < ta.size(); ++s)
                          {
                              if (ta[s].departure != tb[s].departure)
                                  return ta[s].departure < tb[s].departure;
                              if (ta[s].arrival != tb[s].arrival)
                                  return ta[s].arrival < tb[s].arrival;
                          }
                          // Identical timings on identical stops: order by the
                          // originating trip_id so the build is reproducible.
                          return *runs[a].trip_id < *runs[b].trip_id;
                      });

            std::vector<Route> bins;
            for (std::size_t m : members)
            {
                bool placed = false;
                for (auto &bin : bins)
                {
                    if (no_later_than(runs[bin.trips.back()].times, runs[m].times))
                    {
                        bin.trips.push_back(m);
                        placed = true;
                        break;
                    }
                }
                if (!placed)
                {
                    bins.push_back(Route{&pattern, {m}});
                    if (bins.size() > 1)
                        ++split_count;
                }
            }
            for (auto &bin : bins)
                routes.push_back(std::move(bin));
        }

        tt.routes_split_for_overtaking = split_count;

        // ── Emit the flat arrays ───────────────────────────────────────────────
        tt.num_routes = static_cast<uint32_t>(routes.size());
        tt.route_stop_offset.assign(routes.size() + 1, 0);
        tt.route_time_offset.assign(routes.size() + 1, 0);
        tt.route_trip_count.assign(routes.size(), 0);

        std::size_t total_stops = 0, total_times = 0, total_trips = 0;
        for (std::size_t r = 0; r < routes.size(); ++r)
        {
            const std::size_t len = routes[r].stops->size();
            const std::size_t ntr = routes[r].trips.size();
            tt.route_stop_offset[r] = static_cast<uint32_t>(total_stops);
            tt.route_time_offset[r] = static_cast<uint32_t>(total_times);
            tt.route_trip_count[r] = static_cast<uint32_t>(ntr);
            total_stops += len;
            total_times += len * ntr;
            total_trips += ntr;
        }
        tt.route_stop_offset[routes.size()] = static_cast<uint32_t>(total_stops);
        tt.route_time_offset[routes.size()] = static_cast<uint32_t>(total_times);
        tt.num_trips = static_cast<uint32_t>(total_trips);

        tt.route_stops.resize(total_stops);
        tt.route_times.resize(total_times);

        for (std::size_t r = 0; r < routes.size(); ++r)
        {
            const auto &pattern = *routes[r].stops;
            std::copy(pattern.begin(), pattern.end(),
                      tt.route_stops.begin() + tt.route_stop_offset[r]);

            const std::size_t len = pattern.size();
            std::size_t at = tt.route_time_offset[r];
            for (std::size_t t = 0; t < routes[r].trips.size(); ++t)
            {
                const auto &times = runs[routes[r].trips[t]].times;
                std::copy(times.begin(), times.end(), tt.route_times.begin() + at);
                at += len;
            }
        }

        // ── stop -> (route, position) index ────────────────────────────────────
        // Counting pass then fill pass, the same two-pass CSR construction the
        // graph builder uses, so the index is contiguous per stop.
        std::vector<uint32_t> counts(static_cast<std::size_t>(num_stops) + 1, 0);
        for (uint32_t r = 0; r < tt.num_routes; ++r)
        {
            const uint32_t len = tt.route_length(r);
            const uint32_t *st = tt.stops_of(r);
            for (uint32_t pos = 0; pos < len; ++pos)
                ++counts[st[pos] + 1];
        }
        for (uint32_t p = 1; p <= num_stops; ++p)
            counts[p] += counts[p - 1];
        tt.stop_route_offset = counts;
        tt.stop_routes.resize(counts[num_stops]);

        std::vector<uint32_t> fill(counts.begin(), counts.end());
        for (uint32_t r = 0; r < tt.num_routes; ++r)
        {
            const uint32_t len = tt.route_length(r);
            const uint32_t *st = tt.stops_of(r);
            for (uint32_t pos = 0; pos < len; ++pos)
                tt.stop_routes[fill[st[pos]]++] = {r, pos};
        }
        // Routes are emitted in increasing r and increasing pos, so each stop's
        // slice is already sorted by (route, index). Assert rather than re-sort.
        assert([&]
               {
                   for (uint32_t p = 0; p < num_stops; ++p)
                       for (uint32_t x = tt.stop_route_offset[p] + 1; x < tt.stop_route_offset[p + 1]; ++x)
                           if (!(tt.stop_routes[x - 1].route < tt.stop_routes[x].route ||
                                 (tt.stop_routes[x - 1].route == tt.stop_routes[x].route &&
                                  tt.stop_routes[x - 1].index < tt.stop_routes[x].index)))
                               return false;
                   return true;
               }() && "stop_routes must be sorted by (route, index)");

        // ── Footpaths ──────────────────────────────────────────────────────────
        // Identical admission and clamping to GraphBuilder's transfer layer, so
        // that a comparison of the two engines is a comparison of algorithms and
        // not of two subtly different networks.
        if (!transfers.empty())
        {
            struct TempTransfer
            {
                uint32_t source;
                TransferEdge edge;
            };
            std::vector<TempTransfer> temp;
            temp.reserve(transfers.size());
            uint32_t dropped = 0;

            for (const auto &t : transfers)
            {
                const auto from_it = stop_index_map->find(t.from_stop_id);
                const auto to_it = stop_index_map->find(t.to_stop_id);
                if (from_it == stop_index_map->end() || to_it == stop_index_map->end())
                {
                    ++dropped;
                    continue;
                }
                const uint32_t u = from_it->second, v = to_it->second;
                if (u >= num_stops || v >= num_stops || u == v)
                {
                    ++dropped;
                    continue;
                }
                temp.push_back({u, {v, (t.min_transfer_time == 0) ? 1u : t.min_transfer_time}});
            }

            if (dropped > 0)
                std::fprintf(stderr,
                             "[RAPTOR INFO] footpaths: dropped %u unresolved/self/out-of-range records.\n",
                             dropped);

            // ── Transitive closure ────────────────────────────────────────────
            // RAPTOR relaxes footpaths ONCE per round, which is only correct
            // when the relation is transitively closed. It is easy to assume a
            // generated transfers.txt satisfies that — normalize_gtfs.py emits
            // every ordered platform pair of a station, so the graph is complete
            // within each station — but completeness is not closure. BART's feed
            // supplies real min_transfer_times for some pairs and the normaliser
            // defaults the rest, and on eleven of its stations the direct time
            // between two platforms is LONGER than going via a third. One pass
            // then misses the cheaper two-leg walk and every arrival behind it
            // is an upper bound.
            //
            // That is not hypothetical: it is what the study tool's correctness
            // gate caught on BART, where the Pareto engine — which relaxes
            // transfers through its priority queue and so chains them for free —
            // beat this "exact" oracle on 121 observations.
            //
            // Closing the relation also keeps the two models identical, which is
            // the point: the graph model can already walk A -> B -> C, so its
            // effective walk time is min(direct, via). Adding the shortcut makes
            // RAPTOR agree rather than changing what either one means.
            //
            // Cost is a Dijkstra per node over the footpath graph alone. Those
            // components are the platforms of one station — single digits — so
            // this is microseconds, but the guard below refuses to blow up on a
            // feed whose transfers happen to form one huge component.
            {
                std::vector<std::vector<std::pair<uint32_t, uint32_t>>> adj(num_stops);
                for (const auto &x : temp)
                    adj[x.source].push_back({x.edge.destination, x.edge.travel_time});

                constexpr std::size_t kMaxComponentDegree = 4096;
                std::vector<TempTransfer> closed;
                closed.reserve(temp.size());
                uint32_t skipped_components = 0;

                // Distinct pairs present before closure, so the count of ADDED
                // footpaths is accurate even when the feed lists a pair twice.
                std::unordered_set<uint64_t> original;
                original.reserve(temp.size() * 2);
                for (const auto &x : temp)
                    original.insert((static_cast<uint64_t>(x.source) << 32) | x.edge.destination);

                std::vector<uint32_t> dist(num_stops, UINT32_MAX);
                std::vector<uint32_t> touched;
                for (uint32_t s = 0; s < num_stops; ++s)
                {
                    if (adj[s].empty())
                        continue;

                    // Plain Dijkstra with a std::vector frontier: components have
                    // a handful of nodes, so a heap would cost more than it saves.
                    touched.clear();
                    dist[s] = 0;
                    touched.push_back(s);
                    std::vector<uint32_t> settled;
                    std::vector<uint32_t> queue{s};
                    while (!queue.empty())
                    {
                        std::size_t best_i = 0;
                        for (std::size_t q = 1; q < queue.size(); ++q)
                        {
                            if (dist[queue[q]] < dist[queue[best_i]])
                                best_i = q;
                        }
                        const uint32_t u = queue[best_i];
                        queue[best_i] = queue.back();
                        queue.pop_back();
                        settled.push_back(u);
                        if (settled.size() > kMaxComponentDegree)
                        {
                            ++skipped_components;
                            break;
                        }
                        for (const auto &[v, w] : adj[u])
                        {
                            const uint64_t nd = static_cast<uint64_t>(dist[u]) + w;
                            if (nd < dist[v])
                            {
                                if (dist[v] == UINT32_MAX)
                                    touched.push_back(v);
                                dist[v] = static_cast<uint32_t>(nd);
                                queue.push_back(v);
                            }
                        }
                    }

                    // dist[] is final for every touched node once the search
                    // above drains, so one entry per reachable node is emitted
                    // here with the shortest walk — which may be shorter than
                    // the feed's own direct time, and that is the point.
                    for (const uint32_t v : touched)
                    {
                        if (v != s && dist[v] != UINT32_MAX)
                            closed.push_back({s, {v, dist[v]}});
                        dist[v] = UINT32_MAX;
                    }
                }

                uint32_t added = 0;
                for (const auto &x : closed)
                    if (original.find((static_cast<uint64_t>(x.source) << 32) | x.edge.destination) == original.end())
                        ++added;
                tt.footpaths_added_by_closure = added;
                if (skipped_components > 0)
                    std::fprintf(stderr,
                                 "[RAPTOR WARN] %u footpath component(s) exceeded %zu nodes; "
                                 "their closure was truncated and arrivals through them may be "
                                 "upper bounds. transfers_are_transitively_closed() will say so.\n",
                                 skipped_components, kMaxComponentDegree);
                temp.swap(closed);
            }

            std::sort(temp.begin(), temp.end(),
                      [](const TempTransfer &a, const TempTransfer &b)
                      {
                          if (a.source != b.source)
                              return a.source < b.source;
                          return a.edge.destination < b.edge.destination;
                      });

            tt.transfer_offset.assign(static_cast<std::size_t>(num_stops) + 1, 0);
            for (const auto &x : temp)
                ++tt.transfer_offset[x.source + 1];
            for (uint32_t p = 1; p <= num_stops; ++p)
                tt.transfer_offset[p] += tt.transfer_offset[p - 1];

            tt.transfer_data.resize(tt.transfer_offset[num_stops]);
            std::vector<uint32_t> tfill(tt.transfer_offset.begin(), tt.transfer_offset.end());
            for (const auto &x : temp)
                tt.transfer_data[tfill[x.source]++] = x.edge;

            tt.num_transfers = static_cast<uint32_t>(tt.transfer_data.size());
        }

        return tt;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // § 2.  transfers_are_transitively_closed
    // ═══════════════════════════════════════════════════════════════════════════

    bool transfers_are_transitively_closed(const RaptorTimetable &tt)
    {
        if (tt.transfer_offset.empty())
            return true; // no footpaths: the assumption is vacuous

        // Direct lookup for "is there a p->r footpath, and how long".
        std::unordered_map<uint64_t, uint32_t> direct;
        direct.reserve(tt.transfer_data.size() * 2);
        for (uint32_t p = 0; p < tt.num_stops; ++p)
        {
            const auto [b, e] = tt.transfers_of(p);
            for (const TransferEdge *t = b; t != e; ++t)
            {
                const uint64_t key = (static_cast<uint64_t>(p) << 32) | t->destination;
                auto it = direct.find(key);
                if (it == direct.end() || t->travel_time < it->second)
                    direct[key] = t->travel_time;
            }
        }

        for (uint32_t p = 0; p < tt.num_stops; ++p)
        {
            const auto [b1, e1] = tt.transfers_of(p);
            for (const TransferEdge *t1 = b1; t1 != e1; ++t1)
            {
                const auto [b2, e2] = tt.transfers_of(t1->destination);
                for (const TransferEdge *t2 = b2; t2 != e2; ++t2)
                {
                    if (t2->destination == p)
                        continue; // walking back is not a shortcut to prove
                    const uint64_t key = (static_cast<uint64_t>(p) << 32) | t2->destination;
                    const auto it = direct.find(key);
                    const uint64_t two_leg = static_cast<uint64_t>(t1->travel_time) + t2->travel_time;
                    if (it == direct.end() || static_cast<uint64_t>(it->second) > two_leg)
                        return false;
                }
            }
        }
        return true;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // § 3.  Raptor query
    // ═══════════════════════════════════════════════════════════════════════════

    Raptor::Raptor(const RaptorTimetable &tt, uint32_t max_rounds)
        : tt_(tt), max_rounds_(max_rounds == 0 ? 1u : max_rounds)
    {
        marked_.assign(tt_.num_stops, 0);
        marked_list_.reserve(tt_.num_stops);
        queue_pos_.assign(tt_.num_routes, NOT_QUEUED);
        queue_list_.reserve(tt_.num_routes);
    }

    void Raptor::prefault()
    {
        // Mirrors ParetoDijkstra::prefault_arena(): touch every page of the
        // scratch so the first timed query does not pay soft page faults that
        // belong to setup rather than to the algorithm.
        for (auto &m : marked_)
            m = 0;
        for (auto &q : queue_pos_)
            q = NOT_QUEUED;
    }

    uint32_t Raptor::earliest_trip(uint32_t r, uint32_t pos, uint32_t t) const noexcept
    {
        // Binary search over trip index for the first trip whose departure at
        // `pos` is >= t. Valid precisely because RaptorBuilder guarantees trips
        // on a route do not overtake, which makes departure-at-pos monotone in
        // the trip index. Without that guarantee this search silently returns a
        // trip that is not the earliest, which is why the builder splits.
        const uint32_t n = tt_.route_trip_count[r];
        const uint32_t len = tt_.route_length(r);
        const RaptorTimetable::StopTime *base = tt_.route_times.data() + tt_.route_time_offset[r] + pos;

        uint32_t lo = 0, hi = n;
        while (lo < hi)
        {
            const uint32_t mid = lo + (hi - lo) / 2;
            if (base[static_cast<std::size_t>(mid) * len].departure < t)
                lo = mid + 1;
            else
                hi = mid;
        }
        return lo; // == n when no trip is catchable
    }

    void Raptor::run(uint32_t source_stop, uint32_t departure_time, RaptorResult &out)
    {
        const uint32_t n = tt_.num_stops;

        out.num_stops = n;
        out.max_rounds = max_rounds_;
        out.hit_round_cap = false;
        out.rounds = 0;

        const std::size_t layers = static_cast<std::size_t>(max_rounds_) + 1;
        const std::size_t cells = layers * n;
        if (out.tau.size() != cells)
            out.tau.assign(cells, RAPTOR_UNREACHED);
        else
            std::fill(out.tau.begin(), out.tau.end(), RAPTOR_UNREACHED);

        if (n == 0)
            return;
        assert(source_stop < n);

        uint32_t *tau0 = out.tau.data();
        tau0[source_stop] = departure_time;

        // marked_ is left all-zero by the previous run (the round loop clears
        // exactly the entries it set), so no O(V) wipe is needed here.
        marked_list_.clear();
        marked_[source_stop] = 1;
        marked_list_.push_back(source_stop);

        // Round-0 footpaths: a journey may legitimately begin with a walk to
        // another platform of the same station. Omitting this makes any
        // origin whose useful services leave from a sibling platform look
        // unreachable in round 1.
        {
            const auto [tb, te] = tt_.transfers_of(source_stop);
            for (const TransferEdge *t = tb; t != te; ++t)
            {
                const uint32_t arr = departure_time + t->travel_time;
                if (arr < tau0[t->destination])
                {
                    tau0[t->destination] = arr;
                    if (!marked_[t->destination])
                    {
                        marked_[t->destination] = 1;
                        marked_list_.push_back(t->destination);
                    }
                }
            }
        }

        for (uint32_t k = 1; k <= max_rounds_; ++k)
        {
            const uint32_t *prev = out.tau.data() + static_cast<std::size_t>(k - 1) * n;
            uint32_t *cur = out.tau.data() + static_cast<std::size_t>(k) * n;
            std::copy(prev, prev + n, cur);

            // ── Collect the routes to scan, and where to start on each ────────
            queue_list_.clear();
            for (const uint32_t p : marked_list_)
            {
                const auto [rb, re] = tt_.routes_of(p);
                for (const RaptorTimetable::RouteStop *rs = rb; rs != re; ++rs)
                {
                    uint32_t &q = queue_pos_[rs->route];
                    if (q == NOT_QUEUED)
                    {
                        q = rs->index;
                        queue_list_.push_back(rs->route);
                    }
                    else if (rs->index < q)
                    {
                        q = rs->index;
                    }
                }
            }
            for (const uint32_t p : marked_list_)
                marked_[p] = 0;
            marked_list_.clear();

            // ── Scan each queued route once, front to back ────────────────────
            for (const uint32_t r : queue_list_)
            {
                const uint32_t start = queue_pos_[r];
                queue_pos_[r] = NOT_QUEUED; // reset now; no second pass needed

                const uint32_t len = tt_.route_length(r);
                const uint32_t ntrips = tt_.route_trip_count[r];
                const uint32_t *stops = tt_.stops_of(r);

                uint32_t trip = ntrips; // ntrips == "not on a vehicle yet"
                const RaptorTimetable::StopTime *times = nullptr;

                for (uint32_t pos = start; pos < len; ++pos)
                {
                    const uint32_t p = stops[pos];

                    // (1) Riding: can this vehicle improve the arrival at p?
                    if (trip != ntrips)
                    {
                        const uint32_t arr = times[pos].arrival;
                        if (arr < cur[p])
                        {
                            cur[p] = arr;
                            if (!marked_[p])
                            {
                                marked_[p] = 1;
                                marked_list_.push_back(p);
                            }
                        }
                    }

                    // (2) Boarding: having arrived at p in an EARLIER round
                    // (prev, never cur — using cur would let one round chain
                    // two vehicles and the round index would stop counting
                    // trips), can we catch this route earlier than whatever we
                    // are on?
                    const uint32_t ready = prev[p];
                    if (ready != RAPTOR_UNREACHED &&
                        (trip == ntrips || ready <= times[pos].departure))
                    {
                        const uint32_t et = earliest_trip(r, pos, ready);
                        if (et != ntrips && (trip == ntrips || et < trip))
                        {
                            trip = et;
                            times = tt_.trip_times(r, trip);
                        }
                    }
                }
            }

            // ── Footpaths ─────────────────────────────────────────────────────
            // One pass over the stops the route scan just improved. Newly
            // marked destinations are deliberately NOT re-relaxed: RAPTOR
            // assumes the footpath relation is transitively closed, so a
            // two-leg walk is already present as a single footpath. The
            // assumption is checkable — transfers_are_transitively_closed() —
            // and the tools check it rather than assume it.
            const std::size_t from_route_scan = marked_list_.size();
            for (std::size_t idx = 0; idx < from_route_scan; ++idx)
            {
                const uint32_t p = marked_list_[idx];
                const uint32_t base = cur[p];
                const auto [tb, te] = tt_.transfers_of(p);
                for (const TransferEdge *t = tb; t != te; ++t)
                {
                    const uint32_t arr = base + t->travel_time;
                    if (arr < cur[t->destination])
                    {
                        cur[t->destination] = arr;
                        if (!marked_[t->destination])
                        {
                            marked_[t->destination] = 1;
                            marked_list_.push_back(t->destination);
                        }
                    }
                }
            }

            if (marked_list_.empty())
            {
                // Round k improved nothing, so tau_k == tau_{k-1} and every
                // later round would too. The answer is exact.
                break;
            }
            out.rounds = k;
            if (k == max_rounds_)
            {
                // Still improving at the cap: another round might have helped,
                // so the caller must not treat this answer as exact.
                out.hit_round_cap = true;
            }
        }

        // Leave marked_ all-zero for the next query.
        for (const uint32_t p : marked_list_)
            marked_[p] = 0;
        marked_list_.clear();
    }

    RaptorResult Raptor::run(uint32_t source_stop, uint32_t departure_time)
    {
        RaptorResult out;
        run(source_stop, departure_time, out);
        return out;
    }

} // namespace namma_metro
