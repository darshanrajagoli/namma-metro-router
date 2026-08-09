#pragma once

#include "raptor.hpp"
#include "topology.hpp"
#include <cstdint>
#include <string>
#include <vector>

/**
 * @file accessibility.hpp
 * @brief Accessibility surfaces: what is reachable, by when, at how many changes.
 *
 * THE REFRAMING
 * ═════════════
 * This repository has been presenting its output as a latency table — p50, p95,
 * p99 for a single-source, all-destinations query. That is a benchmark of the
 * implementation, and it hides what the query actually computes.
 *
 * A one-to-all bi-criteria search returns, for one origin and one departure
 * time, the non-dominated ways to reach EVERY other station simultaneously.
 * That is not a routing result. It is an accessibility surface with its
 * trade-off structure preserved instead of collapsed to a scalar.
 *
 * Transport planning has used accessibility for decades — "how much of the city
 * can you reach in 45 minutes" — and almost always computes it from travel time
 * alone. That quietly asserts every 45-minute journey is equivalent. They are
 * not: 45 minutes direct is not 45 minutes with two changes, and the difference
 * falls hardest on exactly the people for whom an interchange is a barrier
 * rather than an inconvenience — carrying luggage or shopping, travelling with
 * a small child, elderly, or using a wheelchair.
 *
 * So this file computes the same thing at each change budget separately:
 *
 *     reachable(origin, T, c) = stations reachable within T seconds
 *                               using at most c changes
 *
 * and the quantity worth looking at is the GAP between c = 0 and c = 2. That is
 * the part of the city which is nominally accessible and practically is not.
 * It is invisible to every standard accessibility metric.
 *
 * WHY RAPTOR COMPUTES THIS AND THE PARETO ENGINE DOES NOT
 * ═══════════════════════════════════════════════════════
 * RAPTOR's round k IS "at most k trips", so the whole surface is one query: the
 * round-k layer of tau, thresholded. The Pareto engine's second objective counts
 * platform walks rather than vehicles, which is the same number only when every
 * interchange requires changing platform (see raptor.hpp). For an accessibility
 * claim about passengers, "how many vehicles must I board" is the honest
 * quantity, so this module is built on RAPTOR.
 *
 * WHAT IS NOT CLAIMED
 * ═══════════════════
 * Counting reachable STATIONS is not counting reachable jobs, schools or
 * people. A proper accessibility study weights destinations by what is at them,
 * which needs land-use and demographic data joined to the network and is a data
 * project of its own. Station counts are the network-only version of the
 * measure: defensible, reproducible from the feed alone, and clearly labelled
 * as what it is everywhere it appears.
 */

namespace namma_metro
{

    struct StopRecord;

    struct AccessibilityConfig
    {
        /// Travel-time budgets, in seconds. The classic planning cut is 45 min.
        std::vector<uint32_t> thresholds_s{1800, 2700, 3600};

        /// Departure times sampled across the service day, in seconds past
        /// midnight. Accessibility at 08:00 and at 14:00 are different numbers
        /// and averaging over a fixed set makes cities comparable.
        std::vector<uint32_t> departures{25200, 28800, 32400, 43200, 61200, 68400};

        /// Report change budgets 0 .. max_changes inclusive. A change budget of
        /// c corresponds to RAPTOR round c + 1.
        uint32_t max_changes = 2;
    };

    /// Per-origin result: mean count of reachable stations for each
    /// (change budget, threshold) pair, averaged over the sampled departures.
    struct StationAccessibility
    {
        uint32_t node = 0;
        /// counts[c * thresholds + t]
        std::vector<double> counts;
        /// Departures at which this origin reached at least one other station.
        uint32_t departures_with_service = 0;
    };

    struct AccessibilitySurface
    {
        AccessibilityConfig config;
        uint32_t num_stops = 0;
        uint32_t num_thresholds = 0;
        uint32_t num_budgets = 0; ///< max_changes + 1
        std::vector<StationAccessibility> per_origin;

        [[nodiscard]] double count(uint32_t origin_slot, uint32_t changes, uint32_t threshold) const
        {
            return per_origin[origin_slot].counts[static_cast<std::size_t>(changes) * num_thresholds + threshold];
        }

        /// The headline number: stations reachable within `threshold` with the
        /// full change budget, minus those reachable with none. What a passenger
        /// who cannot change loses.
        [[nodiscard]] double gap(uint32_t origin_slot, uint32_t threshold) const
        {
            return count(origin_slot, num_budgets - 1, threshold) - count(origin_slot, 0, threshold);
        }
    };

    /**
     * @brief Compute the surface for every origin in @p origins.
     *
     * @param tt       Prebuilt timetable.
     * @param origins  Node indices to use as origins. Pass every served station
     *                 for a whole-network surface, or one node for an isochrone.
     * @param config   Thresholds, departures and change budget.
     *
     * Cost is |origins| * |departures| RAPTOR queries. On a 100-station feed
     * with six departures that is 600 queries — well under a second.
     */
    [[nodiscard]] AccessibilitySurface compute_accessibility(
        const RaptorTimetable &tt,
        const std::vector<uint32_t> &origins,
        const AccessibilityConfig &config = {});

    // ═══════════════════════════════════════════════════════════════════════════
    // § 2.  Rendering
    // ═══════════════════════════════════════════════════════════════════════════

    /// Everything the renderer needs about a node, gathered from the feed.
    struct NodePlacement
    {
        uint32_t node = 0;
        double lat = 0.0;
        double lon = 0.0;
        std::string name;
    };

    struct MapStyle
    {
        uint32_t width = 1000;
        uint32_t height = 800;
        uint32_t margin = 48;
        std::string title;
        std::string subtitle;
        /// Label shown under the colour ramp.
        std::string legend_label = "value";
        /// Draw station names. Sensible below ~120 stations, illegible above.
        bool labels = false;
    };

    /**
     * @brief Render a station map as a single self-contained SVG string.
     *
     * @param placements  One entry per node to draw. Nodes absent from this list
     *                    are not drawn even if they appear in @p links.
     * @param links       Undirected station links (TopologyMetrics' station
     *                    graph), drawn as the network's edges.
     * @param values      One value per placement, in the same order, used to
     *                    colour the station markers. Pass an empty vector for an
     *                    uncoloured network diagram.
     *
     * Deliberately dependency-free and deliberately not clever: an equirectangular
     * projection with latitude-scaled longitude, which is accurate enough at
     * city scale and needs no projection library. It writes its own background,
     * so the file renders the same wherever it is opened.
     */
    [[nodiscard]] std::string render_station_map_svg(
        const std::vector<NodePlacement> &placements,
        const std::vector<std::pair<uint32_t, uint32_t>> &links,
        const std::vector<double> &values,
        const MapStyle &style = {});

    /// Collect coordinates and names for @p nodes from the parsed stops table.
    [[nodiscard]] std::vector<NodePlacement> placements_from_stops(
        const std::vector<StopRecord> &stops,
        const std::unordered_map<std::string, uint32_t> &stop_index_map,
        const std::vector<uint32_t> &nodes);

} // namespace namma_metro
