#pragma once

#include "accessibility.hpp"
#include "raptor.hpp"
#include "topology.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @file interchange.hpp
 * @brief Where should one new interchange go?
 *
 * THE INVERSE QUESTION
 * ════════════════════
 * accessibility.hpp measures what a network gives its passengers: how much is
 * reachable within a time budget, at each change budget, and the gap between
 * those. That is a description. This file asks the design question that follows
 * from it, which is the one a transport authority actually has to answer:
 *
 *     given money for exactly ONE new interchange, where does it go?
 *
 * The engine becomes the inner loop of a search over network modifications.
 * Every candidate is proposed, built, measured against the unmodified network,
 * and ranked by what it actually delivered — not by intuition about which
 * stations "look close on the map".
 *
 * This matters for Bengaluru specifically. The multi-feed study found Namma
 * Metro's station graph is a forest: cyclomatic number zero, so there is exactly
 * one route between any pair of stations and the trade-off rate is 0.00%. That
 * is a finding about a city — no alternatives, no redundancy, and if one segment
 * fails there is no second path. A tree is also the network where a single new
 * connection can change the most, because the first cycle you add is the first
 * choice any passenger has ever had.
 *
 * WHAT A CANDIDATE IS
 * ═══════════════════
 * A walking connection between two stations that are physically close but which
 * the network does not currently join — a new footpath, in GTFS terms a pair of
 * transfers.txt records. That is deliberately the cheapest possible intervention
 * and the only one this data can honestly support: building new track would
 * require run times, alignment and cost data that no feed carries, whereas
 * "these two stations are 300 m apart and there is no way to change between
 * them" is fully determined by stops.txt.
 *
 * TWO TRAPS, BOTH OF WHICH WOULD HAVE PRODUCED PLAUSIBLE NONSENSE
 * ═══════════════════════════════════════════════════════════════
 * 1. THE STATION SET MUST NOT MOVE. compute_topology() merges platforms into
 *    stations by taking connected components of the transfer graph. Add a
 *    transfer between two distinct stations and recompute, and those two
 *    stations become ONE station — so the "after" network has fewer stations
 *    than the "before" network and every count is measured against a different
 *    denominator. The comparison would be meaningless and would look fine.
 *
 *    So the StationGraph is computed ONCE, from the unmodified feed, and is an
 *    input here rather than something this file derives. Origins, destinations
 *    and station identity are fixed across every evaluation. Only the footpath
 *    layer changes.
 *
 *    A corollary worth stating: because stations ARE the components of the
 *    transfer graph, two distinct station representatives never already have a
 *    transfer between them. Candidate generation still checks, but the check can
 *    only fire on a malformed input.
 *
 * 2. THE TIMETABLE IS REBUILT, NOT PATCHED. It is tempting to copy the
 *    RaptorTimetable and splice two edges into transfer_data, since the routes
 *    and trips plainly do not change. That would be wrong: RAPTOR relaxes
 *    footpaths once per round and is only correct when the footpath relation is
 *    transitively CLOSED, so a new edge has to be chained against every existing
 *    one (see the closure in src/raptor.cpp, and the BART bug it was written
 *    for). Re-running RaptorBuilder::build() gets the closure right by
 *    construction and keeps exactly one implementation of it in the repository.
 *    It costs a full preprocess per candidate, and that is the correct trade.
 *
 * WHAT IS RANKED, AND WHAT IS MERELY REPORTED
 * ═══════════════════════════════════════════
 * The ranking key is @ref InterchangeEvaluation::delta_reach — the change in the
 * mean number of stations reachable within the threshold using the full change
 * budget. That is "how much more of the city opens up", and it is the number a
 * planner is buying.
 *
 * @ref InterchangeEvaluation::delta_gap is reported beside it and is NOT ranked
 * on, because its sign is not a verdict. A new interchange that widens the gap
 * has delivered its gain only to passengers who are willing and able to change;
 * one that narrows it has delivered gain to everyone. Both can be good buys.
 * Collapsing them into a single score would be exactly the averaging-away this
 * project spent a study arguing against, so both are carried to the output and
 * the reader decides.
 *
 * WHAT IS NOT CLAIMED
 * ═══════════════════
 * Reachable STATIONS, not reachable jobs or people — the same limit
 * accessibility.hpp states, for the same reason. The walk time is MODELLED from
 * straight-line distance at a fixed speed (see InterchangeSearchConfig); it
 * knows nothing about roads, level changes, signals or whether a walkway could
 * physically be built. And a candidate's measured gain includes passengers
 * simply walking between the two stations, which is a real effect a real
 * footpath would also have, but is not the same thing as a train connection.
 *
 * This ranks candidates on one clearly stated criterion. It does not decide
 * where to build anything.
 */

namespace namma_metro
{

    struct StopRecord;
    struct StopTimeRecord;
    struct TransferRecord;

    // ═══════════════════════════════════════════════════════════════════════════
    // § 1.  Candidates
    // ═══════════════════════════════════════════════════════════════════════════

    /// A walking connection the network does not currently offer.
    struct InterchangeCandidate
    {
        uint32_t station_a = 0;   ///< Node index of a station representative.
        uint32_t station_b = 0;   ///< Always > station_a, so a pair appears once.
        double distance_m = 0.0;  ///< Great-circle distance between the two.
        uint32_t walk_time_s = 0; ///< Modelled walk, applied in both directions.

        /// The two stations carry the same name, compared case-insensitively
        /// after trimming.
        ///
        /// This is a DATA-QUALITY signal, not a routing one, and it exists
        /// because ignoring it produces confident nonsense. Stations here are
        /// the connected components of the feed's transfer graph, so a station
        /// complex whose transfers.txt does not join all of its platforms
        /// arrives as two separate "stations" — and the search then proposes
        /// building an interchange between a station and itself. On the New York
        /// subway feed that yields candidates like 59 St-Columbus Circle to
        /// 59 St-Columbus Circle, seventeen metres apart.
        ///
        /// The interchange there already exists; what is missing is the transfer
        /// record. Such a pair is a finding about the FEED and must not be
        /// reported as somewhere to spend money.
        ///
        /// The signal is suggestive rather than decisive in both directions: New
        /// York has several genuinely distinct stations called "14 St", and a
        /// real complex can spell its two halves differently. Read it beside
        /// @ref distance_m — a few tens of metres is not two stations — and
        /// verify before acting.
        bool same_name = false;
    };

    /// Great-circle distance in metres. Spherical earth; at city scale the
    /// difference from an ellipsoidal model is far below the error in treating a
    /// walk as a straight line at all.
    [[nodiscard]] double haversine_m(double lat_a, double lon_a, double lat_b, double lon_b);

    // ═══════════════════════════════════════════════════════════════════════════
    // § 2.  Configuration
    // ═══════════════════════════════════════════════════════════════════════════

    /// The surface used for ranking. One threshold by default: every extra
    /// threshold is carried through every candidate's evaluation, and the ranking
    /// only ever reads one of them.
    [[nodiscard]] inline AccessibilityConfig default_search_accessibility()
    {
        AccessibilityConfig c;
        c.thresholds_s = {2700}; // the 45-minute planning cut
        return c;
    }

    struct InterchangeSearchConfig
    {
        /// Furthest apart two stations may be and still be considered walkable.
        /// 800 m is the conventional planning figure for a transfer walk.
        double max_walk_m = 800.0;

        /// Walking speed used to turn distance into a transfer time. 1.2 m/s is
        /// the standard pedestrian planning speed (~4.3 km/h).
        double walk_speed_mps = 1.2;

        /// Floor on the modelled walk, so two stations 20 m apart do not become a
        /// free transfer. Also covers platform egress and access.
        uint32_t min_walk_time_s = 60;

        /// Thresholds, departures and change budget for the surface.
        AccessibilityConfig accessibility = default_search_accessibility();

        /// Which entry of accessibility.thresholds_s the ranking reads.
        uint32_t threshold_index = 0;
    };

    // ═══════════════════════════════════════════════════════════════════════════
    // § 3.  Inputs and results
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * @brief Everything the search reads, borrowed rather than owned.
     *
     * @warning @ref station_graph must have been computed from the UNMODIFIED
     *          feed and is never recomputed. See trap 1 in the file comment.
     */
    struct InterchangeInputs
    {
        const std::vector<StopTimeRecord> *stop_times = nullptr;
        uint32_t num_stops = 0;
        const std::unordered_map<std::string, uint32_t> *stop_index_map = nullptr;
        const std::vector<TransferRecord> *base_transfers = nullptr;
        const std::vector<StopRecord> *stops = nullptr;
        const StationGraph *station_graph = nullptr;
    };

    /// Aggregate of one accessibility surface at one threshold: means taken over
    /// origins, so networks and modifications of different sizes compare.
    struct SurfaceMeans
    {
        double direct = 0.0; ///< Mean stations reachable with zero changes.
        double full = 0.0;   ///< Mean reachable with the full change budget.
        double gap = 0.0;    ///< full - direct.
    };

    /// What one candidate did, measured against the unmodified network.
    struct InterchangeEvaluation
    {
        InterchangeCandidate candidate;

        SurfaceMeans before;
        SurfaceMeans after;

        double delta_reach = 0.0;  ///< after.full - before.full. THE RANKING KEY.
        double delta_direct = 0.0; ///< after.direct - before.direct.
        double delta_gap = 0.0;    ///< after.gap - before.gap. Reported, not ranked.
    };

    // ═══════════════════════════════════════════════════════════════════════════
    // § 4.  The search
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * @brief Every station pair within walking distance that is not already joined.
     *
     * Excluded: pairs already adjacent in the station graph (a vehicle already
     * runs between them, so a footpath adds a walk beside a ride rather than an
     * interchange), pairs already sharing a transfer (impossible on a well-formed
     * input — see the corollary in the file comment — but checked anyway), and
     * anything beyond @ref InterchangeSearchConfig::max_walk_m.
     *
     * Returned sorted by distance ascending, then by node index, so a run is
     * reproducible and the cheapest interventions come first.
     *
     * @throws std::invalid_argument if any pointer in @p in is null.
     */
    [[nodiscard]] std::vector<InterchangeCandidate> generate_interchange_candidates(
        const InterchangeInputs &in,
        const InterchangeSearchConfig &cfg = {});

    /**
     * @brief Measure the unmodified network once.
     *
     * Exposed separately because a caller usually wants to print the baseline
     * before starting a long sweep, and because every evaluation's `before` is
     * this same value.
     *
     * @throws std::invalid_argument if any pointer in @p in is null, or if
     *         cfg.threshold_index is out of range.
     */
    [[nodiscard]] SurfaceMeans measure_baseline(
        const InterchangeInputs &in,
        const InterchangeSearchConfig &cfg = {});

    /**
     * @brief Build, measure and rank every candidate.
     *
     * Cost is one RaptorBuilder::build() plus |stations| * |departures| RAPTOR
     * queries per candidate. Both are small on a city feed and this is linear in
     * candidates, so a sweep is minutes rather than hours — but it is not free,
     * and @p progress exists so a caller can say so.
     *
     * @param progress Optional, called with (done, total) after each candidate.
     *
     * @return One evaluation per candidate, sorted by delta_reach descending,
     *         ties broken by shorter walk then by node index. Never reordered by
     *         delta_gap: see "what is ranked" in the file comment.
     *
     * @throws std::invalid_argument on null pointers or a bad threshold_index.
     */
    [[nodiscard]] std::vector<InterchangeEvaluation> evaluate_interchange_candidates(
        const InterchangeInputs &in,
        const std::vector<InterchangeCandidate> &candidates,
        const InterchangeSearchConfig &cfg = {},
        const std::function<void(std::size_t, std::size_t)> &progress = {});

} // namespace namma_metro
