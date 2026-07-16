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
    // crowd_weight ∈ [0, 1000] (0=empty, 1000=crush load).
    //
    // FIFO safety: max |d/dt(crowd)| ≈ 0.18 << 1.0 — constraint satisfied.
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

    CSRGraph GraphBuilder::build(
        const std::vector<StopTimeRecord> &stop_times,
        uint32_t num_stops,
        const std::unordered_map<std::string, uint32_t> *stop_index_map)
    {
        if (stop_times.empty())
        {
            CSRGraph g;
            g.num_nodes = num_stops;
            g.num_edges = 0;
            g.offset.assign(num_stops + 1, 0);
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

                // H7: Skip non-boardable stops (pickup_type=1 means no pickup allowed)
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

                // C5: Inject synthetic Gaussian crowd model
                // G6: Overflow safety — crowd_weight ∈ [0, 1000] (scale factor × 10 on [0,100]).
                // Accumulated crowd_cost over a full BMRCL path is bounded:
                //   max_edges_per_path ≤ |V| ≈ 100 (no repeated nodes in a simple path)
                //   max_crowd_weight_per_edge = 1000
                //   max_crowd_cost = 100 × 1000 = 100,000
                // UINT32_MAX = 4,294,967,295 >> 100,000 → no uint32_t overflow possible.
                // If scaling to larger networks, recompute: crowd_weight_max × diameter ≤ UINT32_MAX.
                const uint32_t crowd = synthetic_crowd_weight(from.departure_time);

                TempEdge te;
                te.source = u;
                te.edge.destination = v;
                te.edge.departure_time = from.departure_time;
                te.edge.travel_time = travel_time;
                te.edge.crowd_weight = crowd;
                te.edge.penalty = 0;
                // Integration-Audit Item-6 fix: penalty is a static field on Edge.
                // It is set to 0 here and STAYS 0 throughout the scaffold.
                // The previous comment "Populated at query time by select_optimal_departure"
                // was wrong and dangerous: select_optimal_departure returns Edge by VALUE
                // (a copy of the stored edge). Nothing in the scaffold writes a computed
                // penalty back into that copy, so opt_edge->penalty is always 0 when
                // accumulated into new_crowd = current.crowd_cost + crowd_weight + penalty.
                //
                // Consequence: the penalty dimension of the bi-criteria objective is
                // silently zero for all queries in the scaffold. The FIFO proof's
                // d/dt(penalty) >= -1 condition is therefore trivially satisfied (0 >= -1)
                // but untestable — the constraint is never exercised.
                //
                // This is INTENTIONAL for the scaffold: the three components can be
                // implemented and verified correctly with penalty=0. For the full
                // production implementation (summer 2026 with real BMRCL GTFS), penalty
                // should encode a time-dependent wait surcharge computed in graph_builder
                // from historical dwell distributions, stored in the Edge, and non-zero.
                temp_edges.push_back(te);
            }

            i = j;
        }

        // ── Step 3: Sort temp_edges by source, then departure_time ────────────
        // Sorting by departure_time within each source enables binary-search
        // in select_optimal_departure (Bounded-Wait Lookahead).
        std::sort(temp_edges.begin(), temp_edges.end(),
                  [](const TempEdge &a, const TempEdge &b)
                  {
                      if (a.source != b.source)
                          return a.source < b.source;
                      return a.edge.departure_time < b.edge.departure_time;
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

        // ── G5: Transfer edges (not yet implemented) ──────────────────────────
        // Transfer edges model the time cost of changing lines at interchange
        // stations (e.g., Majestic / Kempegowda: Purple ↔ Green line transfer).
        //
        // What a production implementation would add here:
        //   For each interchange stop pair (u, v) where u and v are the same
        //   physical station on different lines, add a transfer edge:
        //     Edge te;
        //     te.destination    = v_node_index;
        //     te.departure_time = TRANSFER_AVAILABLE_ALL_TIMES; // or time-windowed
        //     te.travel_time    = MIN_TRANSFER_TIME_SECONDS;    // e.g., 300s (5 min)
        //     te.crowd_weight   = platform_density(u, t);       // Gaussian, same model
        //     te.penalty        = 0;
        //
        // CROWD PENALTY ON TRANSFERS: The platform density during a transfer is
        // modelled identically to service edges — synthetic_crowd_weight(departure_time).
        // FIFO safety is preserved by the same Gaussian derivative bound: max |d/dt| ≈ 0.18.
        //
        // WHY NOT IMPLEMENTED HERE:
        //   Namma Metro interchange data (which stop_id pairs correspond to physical
        //   transfers) must be read from either:
        //     (a) transfers.txt in the GTFS feed (standard GTFS optional file), or
        //     (b) a hardcoded interchange table specific to BMRCL topology.
        //   The BMRCL GTFS feed may or may not include transfers.txt.
        //   See project_state.md "COMPONENT 4 (future): Transfer edge generation".
        //
        // ⚠ ORDERING INVARIANT FOR FUTURE IMPLEMENTORS:
        //   Transfer edges MUST be added to temp_edges BEFORE the Step 3 sort:
        //     std::sort(temp_edges.begin(), temp_edges.end(), ...)
        //   Adding them to g.edge_data directly (after the sort + CSR fill) would
        //   place them outside the CSR offset[] range (unreachable via edges_of())
        //   AND in unsorted order within each source node's range, silently breaking
        //   the lower_bound binary search in select_optimal_departure().
        //   There is no runtime error — Dijkstra simply never sees the transfer edges.

        // M7: Debug-mode FIFO invariant check.
        // For each (u→v) edge pair with departure times t1 < t2:
        // verify crowd_weight(t2) - crowd_weight(t1) >= -(t2 - t1)
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
                    const int64_t dc = static_cast<int64_t>(e2->crowd_weight) - static_cast<int64_t>(e1->crowd_weight);
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
                            "crowd_weight drops faster than 1 unit/second. "
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
