#pragma once

#include "graph.hpp"
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @file crowd_model.hpp
 * @brief Position- and time-dependent crowd weights driven by measured ridership.
 *
 * WHAT WAS WRONG WITH THE ORIGINAL CROWD MODEL
 * ════════════════════════════════════════════
 * src/graph_builder.cpp scores every edge with a Gaussian in time of day alone:
 *
 *     crowd(t) = 10 + 90 * exp(-(t - 08:00)^2 / (2 * 3600^2))
 *
 * It is a function of ONE variable, and that variable is the same for every
 * edge leaving at the same second anywhere in the city. So two different routes
 * between the same pair of stations, departing at the same time, are assigned
 * the same crowd cost, and the second Pareto objective has nothing to
 * discriminate. The engine's own diagnostic measured the consequence: a
 * single-label frontier at 96-100% of nodes. The machinery was not broken; the
 * objective was constant.
 *
 * WHAT THIS REPLACES IT WITH
 * ══════════════════════════
 * A measured surface over (station, hour). BMRCL's station-wise hourly entry
 * counts, obtained under the Right to Information Act and republished as open
 * data, give the number of passengers entering each of the 83 Namma Metro
 * stations in each hour of each day. Averaged over the days in the file, that
 * is a crowd field that varies in SPACE as well as time — which is the property
 * the original model lacked and the only property that can create a route-choice
 * trade-off.
 *
 * See docs/crowd-model.md for provenance, the exact file, its checksum, and the
 * script that fetches it.
 *
 * WHAT THIS DOES NOT CLAIM
 * ════════════════════════
 * Station entries are not train occupancy. A passenger entering at station A
 * loads the train from A onwards, not at A alone, and the count says nothing
 * about direction or about how full the train already was. Treating entries as
 * a proxy for the crowd a passenger experiences BOARDING at that station is a
 * modelling assumption, and it is the assumption this file makes. It is a
 * better assumption than "crowding is identical everywhere in the city", which
 * is what it replaces, and it is worse than a load-profile model built from
 * origin-destination flows, which the same dataset would support and which is
 * left as future work. The number this produces is a defensible proxy, not a
 * measurement of occupancy, and no tool here describes it as one.
 *
 * FIFO SAFETY IS PRESERVED BY CONSTRUCTION, AND CHECKED
 * ═════════════════════════════════════════════════════
 * Raw hourly buckets are a step function; a step DOWN of size d at an hour
 * boundary means the weight falls by d in one second, which violates the
 * d/dt(crowd) >= -1 bound the FIFO argument rests on (graph.hpp, routing.hpp
 * §3). The buckets are therefore read as samples at the CENTRE of each hour and
 * interpolated linearly between them, which bounds the derivative by
 * (max adjacent difference) / 3600. With the default scale of 1000 that is at
 * most 0.28 per second, comfortably inside the bound — and apply_hourly_crowd()
 * computes the realised maximum and reports it rather than trusting the
 * arithmetic above.
 */

namespace namma_metro
{

    struct StopRecord;

    // ═══════════════════════════════════════════════════════════════════════════
    // § 1.  The measured load surface
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * @brief Mean passengers entering each stop in each hour of the day.
     *
     * Indexed [node * 24 + hour]. Stops the ridership file does not name keep a
     * load of 0 and are listed in @ref unmatched_gtfs_stops so the caller can
     * decide whether the coverage is good enough — silently defaulting an
     * unmatched station to the network mean would invent data.
     */
    struct HourlyLoadSurface
    {
        uint32_t num_stops = 0;
        std::vector<float> load; ///< size num_stops * 24, absolute passenger counts

        // ── Provenance. Printed by every tool that consumes this. ─────────────
        std::string source_path;  ///< File the numbers came from.
        std::string source_sha256;///< Checksum of that file, when the caller supplies it.
        uint32_t days_averaged = 0;
        std::string first_date, last_date;

        // ── Match quality ─────────────────────────────────────────────────────
        uint32_t rows_read = 0;
        uint32_t rows_unmatched = 0;   ///< Rows whose station name matched no stop.
        uint32_t stops_matched = 0;    ///< GTFS stops that received data.
        std::vector<std::string> unmatched_ridership_names;
        std::vector<std::string> unmatched_gtfs_stops;

        [[nodiscard]] float at(uint32_t node, uint32_t hour) const noexcept
        {
            return load[static_cast<std::size_t>(node) * 24 + hour];
        }

        [[nodiscard]] bool empty() const noexcept { return load.empty(); }

        /// Highest single (stop, hour) value; the scale's normalisation point.
        [[nodiscard]] float peak() const noexcept;

        /// Coverage as a fraction of stops that got data. Below ~0.9 the surface
        /// is not representative and the tools say so.
        [[nodiscard]] double coverage() const noexcept
        {
            return num_stops == 0 ? 0.0 : static_cast<double>(stops_matched) / num_stops;
        }
    };

    /**
     * @brief Parse a station-hourly ridership CSV into an HourlyLoadSurface.
     *
     * Expected columns, by header name (order-independent, so a republished file
     * with extra columns still parses): Date, Hour, Station, Ridership.
     * The delimiter is auto-detected between ';' and ',' from the header line,
     * because the published BMRCL export uses semicolons.
     *
     * Values are averaged over every date present in the file, so the result is
     * "a typical hour of a typical day in the covered period", not any single
     * day. The covered period is recorded in the returned struct.
     *
     * Station names are matched to GTFS stop names after normalisation
     * (lower-cased, punctuation to spaces, runs of spaces collapsed, a trailing
     * "metro station"/"station" removed). An optional alias map is applied
     * first, keyed by the RAW ridership name, for the handful of stations whose
     * published name and GTFS name genuinely differ.
     *
     * @throws std::runtime_error if the file cannot be opened or the header does
     *         not contain the four required columns.
     */
    [[nodiscard]] HourlyLoadSurface load_hourly_ridership_csv(
        const std::string &csv_path,
        const std::vector<StopRecord> &stops,
        const std::unordered_map<std::string, uint32_t> &stop_index_map,
        const std::unordered_map<std::string, std::string> &aliases = {});

    /// Load `ridership_name|gtfs_stop_name` pairs (one per line, `#` comments
    /// allowed) for load_hourly_ridership_csv's alias argument. A missing file
    /// is not an error and yields an empty map.
    ///
    /// Pipe-separated because several of the names this file exists to fix
    /// contain commas — BMRCL publishes "Sir M. Visvesvaraya Stn., Central
    /// College" — and a comma separator would cut the key in half, leaving an
    /// alias that silently never matches anything.
    [[nodiscard]] std::unordered_map<std::string, std::string>
    load_station_aliases(const std::string &path);

    // ═══════════════════════════════════════════════════════════════════════════
    // § 2.  Applying it to a built graph
    // ═══════════════════════════════════════════════════════════════════════════

    struct CrowdModelConfig
    {
        /// secondary_weight assigned to the busiest (station, hour) in the feed.
        /// Everything else scales linearly below it. 1000 matches the range the
        /// original Gaussian model produced, so downstream lambda values and the
        /// overflow argument in graph_builder.cpp carry over unchanged.
        float scale = 1000.0f;

        /// Interpolate between hourly samples instead of using raw steps. Leave
        /// this on unless you are deliberately measuring the FIFO violation a
        /// step function causes; apply_hourly_crowd() will report it either way.
        bool interpolate = true;
    };

    struct CrowdApplyReport
    {
        uint32_t edges_rewritten = 0;
        uint32_t edges_from_unmatched_stop = 0; ///< Source stop had no ridership data.
        float max_weight = 0.0f;
        double mean_weight = 0.0;
        /// Largest observed |d(weight)/dt| in units per second, over every pair
        /// of consecutive departures on the same (u -> v) link. The FIFO
        /// argument requires the NEGATIVE side of this to stay within 1.0.
        double max_negative_slope = 0.0;
        bool fifo_bound_holds = true;
    };

    /**
     * @brief Overwrite every service edge's secondary_weight from the surface.
     *
     * Works on an already-built CSRGraph rather than inside GraphBuilder, so the
     * shipped build path is untouched and the two crowd models can be measured
     * against each other on the same feed in the same process.
     *
     * Requires @p g.second_objective == SecondObjective::CrowdExposure. Applying
     * a crowd field to a graph whose second objective counts transfers would
     * silently turn transfer counts into crowd scores, so it throws instead.
     *
     * The crowd of an edge is read at its SOURCE stop and its departure time —
     * the station the passenger boards at and the moment they board.
     *
     * @throws std::invalid_argument on an objective mismatch or a size mismatch
     *         between the graph and the surface.
     */
    CrowdApplyReport apply_hourly_crowd(
        CSRGraph &g,
        const HourlyLoadSurface &surface,
        CrowdModelConfig config = {});

    /**
     * @brief The original time-only Gaussian, exposed for A/B comparison.
     *
     * Identical to the static function inside graph_builder.cpp. Duplicated
     * deliberately rather than exported from there: the point of this module is
     * to compare against the shipped model, and a comparison whose baseline can
     * drift when the other file is edited is not a comparison. If the two ever
     * disagree, tests/test_crowd_model.cpp fails.
     */
    [[nodiscard]] uint32_t gaussian_crowd_weight(uint32_t departure_time_seconds) noexcept;

} // namespace namma_metro
