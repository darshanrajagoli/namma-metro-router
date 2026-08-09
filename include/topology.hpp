#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @file topology.hpp
 * @brief Structural metrics of a transit network, computed from a GTFS feed.
 *
 * WHY THIS EXISTS
 * ═══════════════
 * The engine's diagnostic found that on Namma Metro the bi-criteria frontier
 * carries a single label at every one of 24,600 observations, and on BART it
 * carries a genuine trade-off at 71% of reached nodes. Same code, same
 * objective, opposite answers. The obvious explanation is the shape of the
 * network — Namma is three lines meeting at two points, which is a tree, and in
 * a tree there is exactly one route between any pair of points.
 *
 * "Obvious explanation" is not a result. To turn it into one you need a number
 * per network that can be correlated against the measured trade-off rate across
 * many networks, which is what this file computes. The metrics are the standard
 * ones from transport geography and graph theory, not invented for this project:
 *
 *   - CYCLOMATIC NUMBER (circuit rank) mu = L - S + C, the number of
 *     independent cycles. It is ZERO exactly when the network is a forest.
 *     This is the direct formalisation of "is there more than one way to go",
 *     and it is the metric the tree hypothesis actually predicts.
 *   - ALPHA (meshedness), BETA and GAMMA indices — Kansky's classical
 *     connectivity indices, normalised so networks of different sizes compare.
 *   - INTERCHANGE DENSITY — the fraction of stations where a passenger can
 *     change line, which is where any alternative has to be constructed.
 *   - ROUTE OVERLAP — the fraction of physical links carried by more than one
 *     line, which is the other way an alternative can exist (two lines running
 *     parallel over the same track).
 *   - SERVICE FREQUENCY — how often the choice is actually offered.
 *
 * THE UNIT OF ANALYSIS IS THE STATION, NOT THE PLATFORM
 * ═════════════════════════════════════════════════════
 * A feed normalised with --transfers keeps platforms as separate nodes, so a
 * two-line interchange appears as several nodes joined by transfer edges.
 * Measuring topology on platforms would count every interchange as a cycle
 * created by the transfer walk itself, which is an artefact of the encoding and
 * not a property of the city. So platforms are first merged into stations
 * (connected components of the transfer graph) and every metric below is
 * computed on the station graph. On a feed normalised WITHOUT --transfers,
 * platforms are already collapsed and the merge is the identity.
 *
 * WHAT COUNTS AS A LINK
 * ═════════════════════
 * An undirected pair of distinct stations that some trip serves consecutively,
 * under exactly the segment-admission rules GraphBuilder and RaptorBuilder use.
 * Track geometry is not consulted: two stations are adjacent if a vehicle runs
 * between them without stopping in between, which is the sense in which a
 * passenger has a choice.
 */

namespace namma_metro
{

    struct StopTimeRecord;
    struct TripRecord;
    struct TransferRecord;

    /**
     * @brief Structural summary of one network. Every field is a plain number so
     *        the whole struct can be written as one row of a study CSV.
     */
    struct TopologyMetrics
    {
        // ── Size ──────────────────────────────────────────────────────────────
        uint32_t platforms = 0;        ///< Nodes in the CSR graph (== stops.txt rows).
        uint32_t platforms_served = 0; ///< Platforms touched by at least one admissible segment.
        uint32_t stations = 0;         ///< Served platforms merged over the transfer graph.
        uint32_t links = 0;            ///< Distinct undirected station pairs served consecutively.
        uint32_t components = 0;       ///< Connected components of the station graph.

        // ── Cycle structure — the tree hypothesis lives here ──────────────────
        int64_t cyclomatic = 0;   ///< L - S + C. Zero iff the network is a forest.
        double alpha_index = 0.0; ///< mu / (2S - 5), the planar meshedness index.
        double beta_index = 0.0;  ///< L / S.
        double gamma_index = 0.0; ///< L / (3(S - 2)), planar connectivity.

        // ── Degree distribution ───────────────────────────────────────────────
        uint32_t deg1_stations = 0;     ///< Termini.
        uint32_t deg2_stations = 0;     ///< Through stations.
        uint32_t deg3plus_stations = 0; ///< Junctions: where a route can diverge.
        double deg3plus_fraction = 0.0;
        double mean_station_degree = 0.0;
        uint32_t max_station_degree = 0;

        // ── Lines and interchange ─────────────────────────────────────────────
        //
        // A LINE is not a route_id. Most feeds publish each direction of a
        // service as its own route_id, so counting route_ids makes every station
        // on any line look like an interchange. Two route_ids serving the same
        // SET of stations are therefore treated as one line, which collapses the
        // directions and leaves short-turn variants grouped under the route_id
        // they already share. @see the derivation in src/topology.cpp step 5.
        uint32_t lines = 0;                ///< Distinct direction-collapsed lines.
        uint32_t interchange_stations = 0; ///< Stations served by >= 2 lines.
        double interchange_density = 0.0;  ///< interchange_stations / stations.
        double mean_lines_per_link = 0.0;
        double shared_link_fraction = 0.0; ///< Links carried by >= 2 lines.

        // ── Service ───────────────────────────────────────────────────────────
        uint32_t trips = 0;              ///< Distinct trip_ids contributing a segment.
        uint32_t segments = 0;           ///< Admissible directed stop-to-stop segments.
        double service_span_hours = 0.0; ///< Last departure minus first, in hours.
        /// Median over directed PLATFORM links of that link's median headway,
        /// restricted to the 07:00-21:00 window the benchmark samples. Platform
        /// rather than station because the two directions between a pair of
        /// stations use different platforms, and merging them interleaves their
        /// departures into a headway shorter than any train runs (measured: 660s
        /// against 600s on BART, 1980s against 1800s on Boston). Zero when no
        /// link has two departures in the window.
        double median_link_headway_s = 0.0;

        /// Human-readable one-line summary, for tool output.
        [[nodiscard]] std::string summary() const;
    };

    /**
     * @brief The station-level physical network the metrics are computed on.
     *
     * Exposed because the accessibility surface needs to DRAW this graph, and
     * recomputing it there would risk the map and the metrics disagreeing about
     * what the network is.
     */
    struct StationGraph
    {
        /// Platform index -> the platform index chosen to represent its station.
        /// Identity on a feed whose platforms are already collapsed.
        std::vector<uint32_t> station_of;
        /// Undirected links between station representatives, deduplicated and
        /// sorted, with first < second.
        std::vector<std::pair<uint32_t, uint32_t>> links;
        /// Station representatives that carry at least one segment, sorted.
        std::vector<uint32_t> stations;
    };

    /**
     * @brief Compute the metrics above from parsed GTFS tables.
     *
     * @param stop_times     Interpolated, FK-validated stop-time records.
     * @param trips          Trip records, used only to attribute segments to a
     *                       GTFS route_id. Pass an empty vector to skip the
     *                       line-based metrics; they are then reported as zero.
     * @param num_stops      Node count (parser.stops().size()).
     * @param stop_index_map Parser's stop_id -> dense index map. REQUIRED, for
     *                       the same reason RaptorBuilder requires it: the
     *                       metrics must describe the same numbering as the
     *                       graph they will be correlated against.
     * @param transfers      Transfer records; used to merge platforms into
     *                       stations. Empty is fine and means the feed already
     *                       collapses platforms.
     * @param out_graph      Optional. When non-null, receives the station graph
     *                       the metrics were computed on, so a caller that
     *                       needs to draw the network gets exactly the same one.
     *
     * @throws std::invalid_argument if @p stop_index_map is null.
     */
    [[nodiscard]] TopologyMetrics compute_topology(
        const std::vector<StopTimeRecord> &stop_times,
        const std::vector<TripRecord> &trips,
        uint32_t num_stops,
        const std::unordered_map<std::string, uint32_t> *stop_index_map,
        const std::vector<TransferRecord> &transfers,
        StationGraph *out_graph = nullptr);

    /// Column headers for TopologyMetrics, comma-separated, matching csv_row().
    [[nodiscard]] std::string topology_csv_header();

    /// One CSV row of the metrics, in the order given by topology_csv_header().
    [[nodiscard]] std::string topology_csv_row(const TopologyMetrics &m);

} // namespace namma_metro
