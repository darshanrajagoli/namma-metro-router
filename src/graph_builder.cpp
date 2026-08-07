#include "graph.hpp"
#include "gtfs_parser.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace namma_metro
{

    // ── Gaussian crowd model ──────────────────────────────────────────────────
    // Models rush-hour crowding as a Gaussian centered at 8AM (28800s).
    // Parameters tuned to Namma Metro ridership patterns.
    // secondary_weight ∈ [0, 1000] (0=empty, 1000=crush load).
    //
    // FIFO safety: |d/dt(crowd)| is zero AT the peak and maximal at |t - PEAK| = SIGMA,
    // where it equals 900 * e^(-1/2) / SIGMA ≈ 0.15 << 1.0 — constraint satisfied.
    // Verified by debug-mode assertion in build() below.
    static uint32_t synthetic_crowd_weight(uint32_t departure_time_seconds)
    {
        constexpr double PEAK_TIME = 28800.0; // 08:00 AM in seconds
        constexpr double SIGMA = 3600.0;      // 1-hour std dev
        constexpr double BASE = 10.0;         // minimum crowd score (out of 100)
        constexpr double AMPLITUDE = 90.0;    // peak additional crowd (out of 100)
        const double t = static_cast<double>(departure_time_seconds);
        const double crowd_raw = BASE + AMPLITUDE * std::exp(
                                                        -((t - PEAK_TIME) * (t - PEAK_TIME)) / (2.0 * SIGMA * SIGMA));
        // Scale [BASE, BASE+AMPLITUDE] → [0, 1000]
        return static_cast<uint32_t>(crowd_raw * 10.0);
    }

    // Transfer-free convenience overload: the crowd objective, no transfer layer.
    // This is the path every existing caller and test takes, and it is byte-for-byte
    // equivalent to the pre-transfer builder.
    CSRGraph GraphBuilder::build(
        const std::vector<StopTimeRecord> &stop_times,
        uint32_t num_stops,
        const std::unordered_map<std::string, uint32_t> *stop_index_map)
    {
        return build_with_transfers(stop_times, num_stops, stop_index_map,
                                    {}, SecondObjective::CrowdExposure);
    }

    CSRGraph GraphBuilder::build_with_transfers(
        const std::vector<StopTimeRecord> &stop_times,
        uint32_t num_stops,
        const std::unordered_map<std::string, uint32_t> *stop_index_map,
        const std::vector<TransferRecord> &transfers,
        SecondObjective objective)
    {
        // Transfers name stops by id. Resolving them against a locally-rebuilt
        // index that disagrees with the parser's would wire transfers between the
        // wrong platforms — silently, since every index is a valid node number.
        if (!transfers.empty() && stop_index_map == nullptr)
        {
            throw std::invalid_argument(
                "GraphBuilder::build_with_transfers(): transfers supplied without a "
                "stop_index_map. Transfer records reference stop_ids, which can only "
                "be resolved against the parser's index. Pass parser.stop_index_map().");
        }

        if (stop_times.empty())
        {
            CSRGraph g;
            g.num_nodes = num_stops;
            g.num_edges = 0;
            g.offset.assign(num_stops + 1, 0);
            g.second_objective = objective;
            // No service edges means no journeys, so a transfer layer would be
            // unreachable. Leave it empty rather than half-built.
            return g;
        }

        // Guard: if a stop_index_map is provided externally, num_stops must be > 0.
        // When stop_index_map != nullptr and num_stops == 0, the fallback path
        // (stop_idx_local) is never populated (it only fills in the nullptr branch),
        // so actual_nodes collapses to 0 and every edge is silently dropped.
        // The returned graph has 0 nodes/0 edges with no warning. Fail loudly.
        if (stop_index_map != nullptr && num_stops == 0)
        {
            throw std::invalid_argument(
                "GraphBuilder::build(): stop_index_map provided but num_stops == 0. "
                "Pass parser.stops().size() as num_stops. "
                "Without it, all edges are silently dropped and the graph is empty.");
        }

        // ── Step 1: Sort by trip_id, then stop_sequence ───────────────────────
        std::vector<const StopTimeRecord *> sorted_ptrs;
        sorted_ptrs.reserve(stop_times.size());
        for (const auto &r : stop_times)
            sorted_ptrs.push_back(&r);

        std::sort(sorted_ptrs.begin(), sorted_ptrs.end(),
                  [](const StopTimeRecord *a, const StopTimeRecord *b)
                  {
                      if (a->trip_id != b->trip_id)
                          return a->trip_id < b->trip_id;
                      return a->stop_sequence < b->stop_sequence;
                  });

        // ── H3: Stop-ID → dense node index mapping ────────────────────────────
        // If the caller provides GTFSParser::stop_index_map(), use it exclusively.
        // Building a local index from sorted_ptrs insertion order produces a
        // different ordering than the parser's index, corrupting all queries that
        // pass parser stop IDs as source/destination. This was a silent bug: the
        // wrong-index graph would pass unit tests (which use the local index) but
        // silently route to wrong nodes in production.
        std::unordered_map<std::string, uint32_t> stop_idx_local;
        const std::unordered_map<std::string, uint32_t> *idx_map = stop_index_map;

        if (idx_map == nullptr)
        {
            // Fallback: build local index from insertion order.
            // WARNING: only safe for unit tests where stop IDs come from the same
            // source. Do NOT use this path in production — always pass stop_index_map.
            uint32_t next_idx = 0;
            for (const auto *r : sorted_ptrs)
            {
                stop_idx_local.emplace(r->stop_id, next_idx);
                next_idx = static_cast<uint32_t>(stop_idx_local.size());
            }
            idx_map = &stop_idx_local;
            std::fprintf(stderr,
                         "[GRAPH WARN] GraphBuilder::build() called without stop_index_map. "
                         "Local fallback index built. Stop IDs from GTFSParser will be mismatched. "
                         "Pass parser.stop_index_map() for production use.\n");
        }

        struct TempEdge
        {
            uint32_t source;
            Edge edge;
        };
        std::vector<TempEdge> temp_edges;
        temp_edges.reserve(sorted_ptrs.size());

        std::size_t i = 0;
        while (i < sorted_ptrs.size())
        {
            std::size_t j = i;
            const std::string &trip = sorted_ptrs[i]->trip_id;
            while (j < sorted_ptrs.size() && sorted_ptrs[j]->trip_id == trip)
                ++j;

            // Generate one edge per consecutive pair within the trip
            for (std::size_t k = i; k + 1 < j; ++k)
            {
                const StopTimeRecord &from = *sorted_ptrs[k];
                const StopTimeRecord &to = *sorted_ptrs[k + 1];

                // Skip non-boardable stops (pickup_type=1 means no pickup allowed)
                if (from.pickup_type == 1)
                    continue;
                // Also skip if drop-off only at destination
                if (to.drop_off_type == 1)
                    continue;

                // Skip if times are sentinel (should not happen after interpolation)
                if (from.departure_time == UINT32_MAX || to.arrival_time == UINT32_MAX)
                    continue;

                auto it_u = idx_map->find(from.stop_id);
                auto it_v = idx_map->find(to.stop_id);

                // Defensive check: stop_id should always be in idx_map after FK
                // validation in the GTFS parser. If not (e.g. caller passed a
                // stop_index_map from a DIFFERENT GTFSParser instance), skip
                // silently with a diagnostic rather than throwing std::out_of_range
                // with no context.
                if (it_u == idx_map->end())
                {
                    std::fprintf(stderr,
                                 "[GRAPH WARN] stop_id '%s' not in stop_index_map — "
                                 "trip=%s stop_seq=%u skipped. "
                                 "Did you pass the correct GTFSParser::stop_index_map()?\n",
                                 from.stop_id.c_str(), from.trip_id.c_str(), from.stop_sequence);
                    continue;
                }
                if (it_v == idx_map->end())
                {
                    std::fprintf(stderr,
                                 "[GRAPH WARN] stop_id '%s' not in stop_index_map — "
                                 "trip=%s stop_seq=%u skipped. "
                                 "Did you pass the correct GTFSParser::stop_index_map()?\n",
                                 to.stop_id.c_str(), to.trip_id.c_str(), to.stop_sequence);
                    continue;
                }
                const uint32_t u = it_u->second;
                const uint32_t v = it_v->second;

                // C10: Reject edges where interpolation produced arrival before departure
                if (to.arrival_time < from.departure_time)
                {
                    std::fprintf(stderr, "[GRAPH WARN] negative travel time dropped: "
                                         "trip=%s stop_seq=%u→%u (dep=%u arr=%u)\n",
                                 from.trip_id.c_str(), from.stop_sequence, to.stop_sequence,
                                 from.departure_time, to.arrival_time);
                    continue;
                }

                // C11: Zero-weight self-edges cause infinite Dijkstra relaxation loops.
                // Enforce minimum travel time of 1 second.
                const uint32_t raw_travel = to.arrival_time - from.departure_time;
                const uint32_t travel_time = (raw_travel == 0) ? 1u : raw_travel;

                // What the second Pareto dimension measures on a SERVICE edge:
                //
                //   CrowdExposure — the synthetic Gaussian crowd model.
                //     Overflow safety: secondary_weight ∈ [0, 1000] (scale ×10 on
                //     [0,100]). Accumulated cost over a path is bounded by
                //     max_edges_per_path (≤ |V| ≈ 100 for a simple path) × 1000 =
                //     100,000, far below UINT32_MAX. Recompute this if scaling to a
                //     much larger network.
                //
                //   TransferCount — zero. Riding a train is free in this objective;
                //     only TransferEdge contributes, so the accumulated cost is
                //     literally "how many times did the passenger change platform".
                const uint32_t secondary =
                    (objective == SecondObjective::TransferCount)
                        ? 0u
                        : synthetic_crowd_weight(from.departure_time);

                TempEdge te;
                te.source = u;
                te.edge.destination = v;
                te.edge.departure_time = from.departure_time;
                te.edge.travel_time = travel_time;
                te.edge.secondary_weight = secondary;
                te.edge.penalty = 0;
                // penalty is a STATIC field on Edge: set to 0 here, and 0 for the
                // entire lifetime of the graph. Nothing computes it at query time —
                // select_optimal_departure returns Edge by VALUE (a copy of the
                // stored edge) and never writes back — so opt_edge->penalty is always
                // 0 where routing.cpp accumulates
                //   new_crowd = current.secondary_cost + secondary_weight + penalty.
                //
                // Consequence to be aware of before relying on it: the penalty term of
                // the composite objective contributes nothing in this build. The FIFO
                // proof's d/dt(penalty) >= -1 condition is trivially satisfied (0 >= -1)
                // and correspondingly never exercised. The secondary_weight term above is
                // the objective's only active second dimension.
                //
                // To make penalty meaningful, encode a time-dependent wait surcharge
                // here (from dwell-time distributions) and store it in the Edge. Doing
                // so re-activates the derivative bound, so the debug FIFO invariant
                // check below and tests/test_fifo_invariant.cpp must be extended to
                // cover it.
                temp_edges.push_back(te);
            }

            i = j;
        }

        // ── Step 3: Sort temp_edges by source, then departure_time ────────────
        // Sorting by departure_time within each source enables binary-search
        // in select_optimal_departure (Bounded-Wait Lookahead).
        //
        // The comparator is a TOTAL order, not just (source, departure_time).
        // std::sort is not stable, so any pair it considers equivalent may come
        // out in either order — and ties on departure_time are common (every
        // train leaving a station at the same second). Different orderings feed
        // insert_and_dominate in different sequences, and where two candidates
        // tie on both objectives, a different one survives. The frontier stays
        // correct either way, but the RESULTS STOP BEING REPRODUCIBLE run to
        // run, which quietly invalidates any measurement taken from them.
        // Breaking ties on destination then travel_time makes the build
        // deterministic for a given input.
        std::sort(temp_edges.begin(), temp_edges.end(),
                  [](const TempEdge &a, const TempEdge &b)
                  {
                      if (a.source != b.source)
                          return a.source < b.source;
                      if (a.edge.departure_time != b.edge.departure_time)
                          return a.edge.departure_time < b.edge.departure_time;
                      if (a.edge.destination != b.edge.destination)
                          return a.edge.destination < b.edge.destination;
                      return a.edge.travel_time < b.edge.travel_time;
                  });

        // ── Step 4: Build CSR offset array (prefix sum) ────────────────────────
        const uint32_t actual_nodes = (num_stops > 0)
                                          ? num_stops
                                          : static_cast<uint32_t>(stop_idx_local.size());

        CSRGraph g;
        g.num_nodes = actual_nodes;
        g.offset.assign(actual_nodes + 1, 0);

        for (const auto &te : temp_edges)
        {
            if (te.source < actual_nodes)
            {
                ++g.offset[te.source + 1];
            }
        }
        // Prefix sum
        for (uint32_t n = 1; n <= actual_nodes; ++n)
        {
            g.offset[n] += g.offset[n - 1];
        }

        // ── Step 5: Fill edge_data in CSR order ────────────────────────────────
        g.edge_data.resize(g.offset[actual_nodes]);
        g.num_edges = static_cast<uint32_t>(g.edge_data.size());

        std::vector<uint32_t> fill_pos(g.offset.begin(), g.offset.end());
        for (const auto &te : temp_edges)
        {
            if (te.source < actual_nodes)
            {
                g.edge_data[fill_pos[te.source]++] = te.edge;
            }
        }

        assert(g.offset[actual_nodes] == g.num_edges);

        // ── Step 6: Transfer layer ────────────────────────────────────────────
        // Transfer edges model the walk between two platforms of one physical
        // station — the cost of changing lines (Majestic on Namma Metro; the
        // cross-platform interchanges on BART).
        //
        // These go into their OWN CSR arrays, not into edge_data. A transfer has
        // no departure time (it is available whenever the passenger arrives), so
        // placing it in the departure-time-sorted service array would require a
        // sentinel time, and every sentinel value collides with some real
        // departure — 0 is a legitimate 00:00:00. Keeping them separate removes
        // the ambiguity and leaves the service-edge binary search untouched.
        g.second_objective = objective;

        if (!transfers.empty())
        {
            struct TempTransfer { uint32_t source; TransferEdge edge; };
            std::vector<TempTransfer> temp_transfers;
            temp_transfers.reserve(transfers.size());

            uint32_t unresolved = 0, out_of_range = 0, self_loops = 0;

            for (const auto &t : transfers)
            {
                const auto from_it = idx_map->find(t.from_stop_id);
                const auto to_it   = idx_map->find(t.to_stop_id);
                if (from_it == idx_map->end() || to_it == idx_map->end())
                {
                    ++unresolved;
                    continue;
                }
                const uint32_t u_idx = from_it->second;
                const uint32_t v_idx = to_it->second;

                if (u_idx >= actual_nodes || v_idx >= actual_nodes)
                {
                    ++out_of_range;
                    continue;
                }
                // A self-transfer is a zero-length cycle the router would expand
                // on every visit for no benefit.
                if (u_idx == v_idx)
                {
                    ++self_loops;
                    continue;
                }

                // Same guard as C11 on service edges: clamp to a minimum of 1
                // second. GTFS legitimately uses min_transfer_time = 0 for a
                // same-platform change, and BART's feed does. Transfers are
                // bidirectional, so a pair of genuinely zero-cost transfer edges
                // is a zero-weight cycle. Dominance does terminate on it (an
                // identical (arrival, secondary) label is rejected by the
                // same-arrival-time check in insert_and_dominate), but relying on
                // that is fragile, and a platform change taking literally zero
                // seconds is not physical anyway.
                TempTransfer tt;
                tt.source = u_idx;
                tt.edge.destination = v_idx;
                tt.edge.travel_time = (t.min_transfer_time == 0) ? 1u : t.min_transfer_time;
                temp_transfers.push_back(tt);
            }

            if (unresolved > 0 || out_of_range > 0 || self_loops > 0)
            {
                std::fprintf(stderr,
                             "[GRAPH INFO] transfer layer: dropped %u unresolved stop_id, "
                             "%u out-of-range index, %u self-loop records.\n",
                             unresolved, out_of_range, self_loops);
            }

            // Group by source. Correctness needs no ordering within a node's
            // range — the router scans transfers linearly, since they are all
            // simultaneously available, so there is no binary search to satisfy.
            // Determinism does: sorting on source alone leaves same-source
            // transfers in an arbitrary order (std::sort is not stable), which
            // changes the relaxation sequence and therefore which of two tied
            // labels survives. Break the tie on destination.
            std::sort(temp_transfers.begin(), temp_transfers.end(),
                      [](const TempTransfer &a, const TempTransfer &b)
                      {
                          if (a.source != b.source)
                              return a.source < b.source;
                          return a.edge.destination < b.edge.destination;
                      });

            g.transfer_offset.assign(actual_nodes + 1, 0);
            for (const auto &tt : temp_transfers)
                ++g.transfer_offset[tt.source + 1];
            for (uint32_t n = 1; n <= actual_nodes; ++n)
                g.transfer_offset[n] += g.transfer_offset[n - 1];

            g.transfer_data.resize(g.transfer_offset[actual_nodes]);
            std::vector<uint32_t> tfill(g.transfer_offset.begin(), g.transfer_offset.end());
            for (const auto &tt : temp_transfers)
                g.transfer_data[tfill[tt.source]++] = tt.edge;

            g.num_transfers = static_cast<uint32_t>(g.transfer_data.size());
            assert(g.transfer_offset[actual_nodes] == g.num_transfers);
        }

        // Debug-mode FIFO invariant check.
        // For each (u→v) edge pair with departure times t1 < t2:
        // verify secondary_weight(t2) - secondary_weight(t1) >= -(t2 - t1)
        // i.e., penalty cannot drop faster than 1 unit per second.
        // If this fires, the crowd model parameters need adjustment.
#ifndef NDEBUG
        for (uint32_t n = 0; n < g.num_nodes; ++n)
        {
            auto [begin, end] = g.edges_of(n);
            for (const Edge *e1 = begin; e1 != end; ++e1)
            {
                for (const Edge *e2 = e1 + 1; e2 != end; ++e2)
                {
                    if (e1->destination != e2->destination)
                        continue;
                    const int64_t dt = static_cast<int64_t>(e2->departure_time) - static_cast<int64_t>(e1->departure_time);
                    const int64_t dc = static_cast<int64_t>(e2->secondary_weight) - static_cast<int64_t>(e1->secondary_weight);
                    if (dt > 0 && dc < -dt)
                    {
                        // v8 fix: assert(false) here sent SIGABRT, killing the test
                        // binary before any JUnit XML was written — CI showed a crash
                        // with no named test failure. Replaced with throw so the test
                        // framework catches it and records a structured failure.
                        std::fprintf(stderr,
                                     "[FIFO VIOLATION] node %u→%u: crowd drops %lld over %lld seconds "
                                     "(violates dc >= -dt). Dijkstra WILL produce wrong results.\n"
                                     "Check crowd model parameters (sigma, amplitude) in graph_builder.cpp.\n",
                                     n, e1->destination, (long long)-dc, (long long)dt);
                        throw std::logic_error(
                            "FIFO invariant violated in graph_builder: "
                            "secondary_weight drops faster than 1 unit/second. "
                            "Adjust AMPLITUDE or SIGMA in graph_builder.cpp. "
                            "See stderr for the offending node pair.");
                    }
                }
            }
        }
#endif

        return g;
    }

} // namespace namma_metro
