#pragma once

#include "graph.hpp" // TransferEdge, and the SecondObjective enum's documentation
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @file raptor.hpp
 * @brief RAPTOR — Round-bAsed Public Transit Optimized Router.
 *
 * Delling, Pajor and Werneck, "Round-Based Public Transit Routing",
 * ALENEX 2012 / Transportation Science 49(3), 2015.
 *
 * WHY THIS EXISTS IN A REPOSITORY BUILT AROUND A TIME-DEPENDENT GRAPH
 * ═══════════════════════════════════════════════════════════════════
 * The README used to concede that "RAPTOR would very likely be faster for this
 * specific objective" without measuring it. This is the measurement. RAPTOR
 * earns its place here three times over:
 *
 *   1. As a SPEED baseline. Same feed, same query set, same machine, same
 *      timing harness, runs interleaved so clock drift hits both arms equally.
 *      tools/raptor_bench.cpp.
 *
 *   2. As an EXACTNESS ORACLE. RAPTOR scans every trip of every route it
 *      reaches; it has no lookahead window and no departure-selection
 *      heuristic. The Pareto engine, by contrast, expands ONE composite-optimal
 *      departure per link chosen from the next `k_departures` within
 *      `W_max_seconds` (routing.hpp §3). That restriction can only ever make an
 *      arrival later, never earlier, so RAPTOR gives a lower bound the engine
 *      can be measured against on real feeds rather than on fixtures.
 *
 *   3. As the ground truth for "does this network admit a genuine trade-off?".
 *      Round k of RAPTOR is, by construction, the earliest arrival using at
 *      most k trips, so the exact (arrival time, number of trips) Pareto
 *      frontier falls out of the round structure with no dominance machinery at
 *      all. tools/study.cpp uses that to answer the multi-feed question with an
 *      exact algorithm instead of with the engine's own restricted search.
 *
 * MODEL EQUIVALENCE WITH GraphBuilder — read this before comparing numbers
 * ════════════════════════════════════════════════════════════════════════
 * A speed or exactness comparison is meaningless unless both algorithms are
 * solving the same problem on the same network. RaptorBuilder therefore
 * replicates GraphBuilder's edge-admission rules exactly (see
 * src/graph_builder.cpp, "Generate one edge per consecutive pair"):
 *
 *   - a segment stop[i] -> stop[i+1] is admissible only when neither time is
 *     the UINT32_MAX interpolation sentinel, both stop_ids resolve in the index
 *     map, pickup_type[i] != 1, drop_off_type[i+1] != 1, and
 *     arrival[i+1] >= departure[i];
 *   - travel time is clamped to a minimum of one second, so the effective
 *     arrival at stop[i+1] is max(arrival[i+1], departure[i] + 1);
 *   - a trip whose segment is inadmissible is SPLIT there rather than dropped,
 *     because in the graph model the surviving segments remain traversable.
 *
 * One modelling difference is deliberate and is itself a finding, so it is
 * stated rather than hidden. The graph model has no notion of "which vehicle am
 * I on": arriving at a node, a passenger may board any departure from it at no
 * cost. Its transfer count therefore counts PLATFORM WALKS, whereas RAPTOR's
 * round index counts TRIPS. The two agree whenever changing service requires
 * changing platform and differ by one for every same-platform interchange. So
 * the engine's second objective systematically under-counts changes wherever
 * two routes share a platform. tools/study.cpp reports both numbers side by
 * side and never averages them together.
 *
 * WHAT IS AND IS NOT IMPLEMENTED
 * ══════════════════════════════
 *   Implemented: the base algorithm (Algorithm 1 of the paper) with footpaths,
 *   local pruning, and route-pattern construction including the non-overtaking
 *   split described below.
 *
 *   Not implemented: target pruning (this engine answers one-to-all queries, so
 *   there is no target to prune against), rRAPTOR (range queries), McRAPTOR
 *   (additional criteria beyond arrival/trips), and any preprocessing-based
 *   acceleration. Those are out of scope: the point is a like-for-like
 *   comparison against a one-to-all bi-criteria search, not the fastest
 *   possible RAPTOR.
 */

namespace namma_metro
{

    // Defined in gtfs_parser.hpp; only referenced through the builder signature.
    struct StopTimeRecord;
    struct TransferRecord;

    /// Sentinel for "not reached". Chosen to match the parser's blank-time
    /// sentinel so a stray sentinel is visible rather than plausible.
    inline constexpr uint32_t RAPTOR_UNREACHED = UINT32_MAX;

    // ═══════════════════════════════════════════════════════════════════════════
    // § 1.  RaptorTimetable — the preprocessed, immutable route structure
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * @brief Flat, index-addressed timetable: routes, their stops, their trips.
     *
     * "Route" here is RAPTOR's meaning, not GTFS's: a maximal set of trips that
     * serve exactly the same ordered sequence of stops and do not overtake one
     * another. A GTFS route_id typically fans out into several of these, and one
     * GTFS pattern may be split further to restore the non-overtaking property
     * (see @ref RaptorTimetable::routes_split_for_overtaking).
     *
     * Layout follows the same principle as CSRGraph: everything is a flat array
     * with an offset table, so scanning a route touches contiguous memory. That
     * is the structural reason RAPTOR is expected to win on speed, and keeping
     * the layout honest is what makes the comparison fair.
     *
     *   stops of route r         : route_stops[route_stop_offset[r] .. r+1)
     *   times of trip t of route r at position i:
     *       route_times[ route_time_offset[r] + t * route_length(r) + i ]
     *   routes serving stop p    : stop_routes[stop_route_offset[p] .. p+1)
     *   footpaths out of stop p  : transfer_data[transfer_offset[p] .. p+1)
     */
    struct RaptorTimetable
    {
        /// One (arrival, departure) pair for one trip at one position on a route.
        struct StopTime
        {
            uint32_t arrival;
            uint32_t departure;
        };
        static_assert(sizeof(StopTime) == 8, "StopTime must stay 8 bytes: two trips per cache line pair.");

        /// Where a route appears in a stop's route list. `index` is the position
        /// of that stop within the route, so a scan can start midway.
        struct RouteStop
        {
            uint32_t route;
            uint32_t index;
        };

        uint32_t num_stops = 0;
        uint32_t num_routes = 0;
        uint32_t num_trips = 0; ///< Total trips across all routes (post-split).

        std::vector<uint32_t> route_stops;
        std::vector<uint32_t> route_stop_offset; ///< size num_routes + 1

        std::vector<StopTime> route_times;
        std::vector<uint32_t> route_time_offset; ///< size num_routes + 1, indexes route_times
        std::vector<uint32_t> route_trip_count;  ///< size num_routes

        std::vector<RouteStop> stop_routes;
        std::vector<uint32_t> stop_route_offset; ///< size num_stops + 1

        std::vector<TransferEdge> transfer_data;
        std::vector<uint32_t> transfer_offset; ///< size num_stops + 1, or empty
        uint32_t num_transfers = 0;

        // ── Build diagnostics. Printed by the tools; never silently discarded. ──

        /// Extra routes created because a GTFS stop pattern contained trips that
        /// overtake one another. RAPTOR's incremental "earliest catchable trip"
        /// scan assumes trips on a route are totally ordered in time at every
        /// stop; a feed that violates that must be split or the search is wrong.
        uint32_t routes_split_for_overtaking = 0;

        /// Trips cut in two because a segment failed GraphBuilder's admission
        /// rules (sentinel time, negative travel time, boarding restriction).
        uint32_t trips_split_for_bad_segment = 0;

        /// Stop-time rows whose stop_id did not resolve against the index map.
        uint32_t rows_unresolved_stop = 0;

        /// Footpaths the builder ADDED to make the relation transitively closed,
        /// which RAPTOR's single relaxation pass per round requires. Non-zero
        /// means the feed's own transfer times had a two-leg walk shorter than
        /// the direct one somewhere — common, and silently wrong without this.
        uint32_t footpaths_added_by_closure = 0;

        [[nodiscard]] uint32_t route_length(uint32_t r) const noexcept
        {
            return route_stop_offset[r + 1] - route_stop_offset[r];
        }

        [[nodiscard]] const uint32_t *stops_of(uint32_t r) const noexcept
        {
            return route_stops.data() + route_stop_offset[r];
        }

        /// Times of trip @p t of route @p r, as an array of route_length(r) entries.
        [[nodiscard]] const StopTime *trip_times(uint32_t r, uint32_t t) const noexcept
        {
            return route_times.data() + route_time_offset[r] + static_cast<std::size_t>(t) * route_length(r);
        }

        [[nodiscard]] std::pair<const RouteStop *, const RouteStop *>
        routes_of(uint32_t p) const noexcept
        {
            return {stop_routes.data() + stop_route_offset[p],
                    stop_routes.data() + stop_route_offset[p + 1]};
        }

        [[nodiscard]] std::pair<const TransferEdge *, const TransferEdge *>
        transfers_of(uint32_t p) const noexcept
        {
            if (transfer_offset.empty())
                return {nullptr, nullptr};
            return {transfer_data.data() + transfer_offset[p],
                    transfer_data.data() + transfer_offset[p + 1]};
        }

        /// Bytes held by the timetable — the RAPTOR side of the memory comparison.
        [[nodiscard]] std::size_t memory_bytes() const noexcept
        {
            return sizeof(uint32_t) * (route_stops.size() + route_stop_offset.size() + route_time_offset.size() + route_trip_count.size() + stop_route_offset.size() + transfer_offset.size()) + sizeof(StopTime) * route_times.size() + sizeof(RouteStop) * stop_routes.size() + sizeof(TransferEdge) * transfer_data.size();
        }
    };

    // ═══════════════════════════════════════════════════════════════════════════
    // § 2.  RaptorBuilder
    // ═══════════════════════════════════════════════════════════════════════════

    class RaptorBuilder
    {
    public:
        /**
         * @brief Build a RaptorTimetable from the same inputs GraphBuilder takes.
         *
         * @param stop_times     Interpolated, FK-validated stop-time records.
         * @param num_stops      Node count; must equal the CSRGraph's num_nodes
         *                       for the two structures to be index-compatible.
         * @param stop_index_map GTFSParser::stop_index_map(). REQUIRED. Unlike
         *                       GraphBuilder there is no local-index fallback:
         *                       a RaptorTimetable is only ever used alongside a
         *                       CSRGraph, and two structures built from two
         *                       different indices would compare different
         *                       networks while looking identical.
         * @param transfers      Platform-to-platform transfers, treated as
         *                       footpaths and TRANSITIVELY CLOSED here before
         *                       use. RAPTOR relaxes footpaths once per round,
         *                       which is only correct on a closed relation, and
         *                       real feeds are not closed: BART supplies a
         *                       direct platform-to-platform time that is longer
         *                       than walking via a third platform. Closure is
         *                       also what keeps this model identical to the
         *                       graph model, which can already chain walks.
         *                       @see RaptorTimetable::footpaths_added_by_closure
         *                       and transfers_are_transitively_closed().
         *
         * @throws std::invalid_argument if @p stop_index_map is null, or if
         *         @p num_stops is zero while stop_times is non-empty.
         *
         * @post For every route r and trip index t < route_trip_count[r] - 1,
         *       trip t is pointwise no later than trip t+1 at every position.
         */
        static RaptorTimetable build(
            const std::vector<StopTimeRecord> &stop_times,
            uint32_t num_stops,
            const std::unordered_map<std::string, uint32_t> *stop_index_map,
            const std::vector<TransferRecord> &transfers);

    private:
        RaptorBuilder() = delete;
    };

    /**
     * @brief Verify the assumption RAPTOR's single footpath pass rests on.
     *
     * Returns true when for every pair of footpaths p->q and q->r there is a
     * footpath p->r no longer than their sum. RaptorBuilder::build() establishes
     * this by construction, so on a timetable it produced this should always be
     * true; it is kept, and called by the tools, because "should always" is what
     * an assumption sounds like right up until a component-size guard trips or
     * someone hand-builds a timetable in a test. When it is false, one
     * relaxation pass per round can miss a two-leg walk and every arrival behind
     * it is an upper bound rather than exact.
     *
     * Cost is O(sum over p of deg(p)^2) with a hash lookup per pair. Intended
     * for a once-per-feed check at load time, not for a query loop.
     */
    [[nodiscard]] bool transfers_are_transitively_closed(const RaptorTimetable &tt);

    // ═══════════════════════════════════════════════════════════════════════════
    // § 3.  RaptorResult
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * @brief Per-round earliest arrival times for one query.
     *
     * `tau(k, p)` is the earliest arrival at stop p using AT MOST k trips, so
     * the sequence is non-increasing in k by construction: round k starts as a
     * copy of round k-1. That is what makes the exact (arrival, trips) Pareto
     * frontier free — it is precisely the set of rounds at which tau strictly
     * improves. @see frontier_size().
     */
    struct RaptorResult
    {
        uint32_t num_stops = 0;
        uint32_t rounds = 0;        ///< Highest round index actually computed.
        uint32_t max_rounds = 0;    ///< Capacity: tau holds max_rounds + 1 layers.
        bool hit_round_cap = false; ///< Search was still improving when it stopped.

        /// Flat (max_rounds + 1) x num_stops array of arrival times.
        std::vector<uint32_t> tau;

        [[nodiscard]] uint32_t arrival(uint32_t k, uint32_t p) const noexcept
        {
            return tau[static_cast<std::size_t>(k) * num_stops + p];
        }

        /// Earliest arrival over any number of trips.
        [[nodiscard]] uint32_t best(uint32_t p) const noexcept { return arrival(rounds, p); }

        [[nodiscard]] bool reached(uint32_t p) const noexcept { return best(p) != RAPTOR_UNREACHED; }

        /**
         * @brief Size of the exact (arrival, trips) Pareto frontier at stop @p p.
         *
         * Counts the rounds at which the arrival time strictly improves,
         * starting at round 0 so that a destination reachable on foot alone
         * (another platform of the origin station) is counted as the zero-trip
         * journey it is. Zero for an unreached stop, one when there is a single
         * sensible journey, and greater than one exactly when the network
         * offers a real time-versus-changes trade-off to this stop.
         *
         * The SOURCE stop scores 1 — the trivial "stay where you are" option at
         * round 0 — so callers iterating every stop should skip the source, as
         * tools/study.cpp does.
         */
        [[nodiscard]] uint32_t frontier_size(uint32_t p) const noexcept
        {
            uint32_t k_count = 0;
            uint32_t prev = RAPTOR_UNREACHED;
            for (uint32_t k = 0; k <= rounds; ++k)
            {
                const uint32_t a = arrival(k, p);
                if (a != RAPTOR_UNREACHED && a != prev)
                    ++k_count;
                prev = a;
            }
            return k_count;
        }

        /**
         * @brief Fewest trips that achieve the earliest arrival.
         *
         * Returns 0 for an unreached stop AND for a stop whose best arrival
         * needs no vehicle at all (the source, or a platform reachable from it
         * on foot). Use reached() to tell those two apart.
         */
        [[nodiscard]] uint32_t rounds_to_best(uint32_t p) const noexcept
        {
            const uint32_t b = best(p);
            if (b == RAPTOR_UNREACHED)
                return 0;
            for (uint32_t k = 0; k <= rounds; ++k)
                if (arrival(k, p) == b)
                    return k;
            return rounds;
        }
    };

    // ═══════════════════════════════════════════════════════════════════════════
    // § 4.  Raptor — the query engine
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * @brief One-to-all RAPTOR query engine over a prebuilt RaptorTimetable.
     *
     * Reuses its scratch buffers between calls, exactly as ParetoDijkstra reuses
     * its arena and frontier vectors, so the timed loop in a benchmark performs
     * no heap allocation in either arm. Constructing one of these per query and
     * then comparing against the engine's reusing path would measure allocator
     * behaviour rather than algorithms.
     *
     * Thread safety: the timetable is read-only and shareable; a Raptor instance
     * owns mutable scratch and is not.
     */
    class Raptor
    {
    public:
        /**
         * @param tt          Prebuilt timetable. Must outlive this object.
         * @param max_rounds  Hard cap on rounds. The search normally stops on
         *                    its own when a round marks no stop; the cap only
         *                    binds on pathological networks, and when it does
         *                    RaptorResult::hit_round_cap is set so the caller
         *                    cannot mistake a truncated answer for an exact one.
         *                    12 rounds means journeys of up to 12 vehicles.
         */
        explicit Raptor(const RaptorTimetable &tt, uint32_t max_rounds = 12);

        /// Allocation-free hot path: fills @p out in place, reusing its capacity.
        void run(uint32_t source_stop, uint32_t departure_time, RaptorResult &out);

        /// Convenience overload. Allocates; prefer the above in benchmarks.
        [[nodiscard]] RaptorResult run(uint32_t source_stop, uint32_t departure_time);

        /// Touch every scratch page before timing, mirroring prefault_arena().
        void prefault();

        [[nodiscard]] uint32_t max_rounds() const noexcept { return max_rounds_; }

    private:
        const RaptorTimetable &tt_;
        uint32_t max_rounds_;

        // Scratch, all sized once at construction and reused. `marked_` is a
        // flat byte-per-stop stamp rather than a set: the round loop tests
        // membership once per stop per round, and a byte array beats any hashed
        // structure at that access pattern.
        std::vector<uint8_t> marked_;
        std::vector<uint8_t> marked_next_;
        std::vector<uint32_t> marked_list_;

        // Routes queued for scanning this round, with the earliest position at
        // which each was marked. queue_pos_[r] == NOT_QUEUED means absent, which
        // keeps the "already queued?" test O(1) without clearing the whole array
        // (only the queued entries are reset, via queue_list_).
        std::vector<uint32_t> queue_pos_;
        std::vector<uint32_t> queue_list_;

        static constexpr uint32_t NOT_QUEUED = UINT32_MAX;

        /// Earliest trip of route @p r boardable at position @p pos at time @p t,
        /// or route_trip_count[r] if none. Binary search; valid because the
        /// builder guarantees trips on a route do not overtake.
        [[nodiscard]] uint32_t earliest_trip(uint32_t r, uint32_t pos, uint32_t t) const noexcept;
    };

} // namespace namma_metro
