#include <gtest/gtest.h>
#include "accessibility.hpp"
#include "gtfs_parser.hpp"
#include "raptor.hpp"
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @file test_accessibility.cpp
 * @brief Contracts for the accessibility surface and its renderer.
 *
 * The measurement this module exists for is the GAP: how much of the network is
 * reachable within a time budget with a change budget, versus without one. The
 * tests build a network where the answer is known by inspection — a direct
 * service that covers part of the network quickly, and a second service that
 * covers the rest but only after a change — and pin both halves.
 *
 * The renderer is tested for the properties a chart has to have to be usable
 * rather than for its appearance: well-formed XML with no unescaped feed text,
 * one marker per station, and a projection that does not stretch one axis
 * against the other. A map with independently scaled axes is a lie about
 * distance, and distance is the entire subject.
 */

using namespace namma_metro;

namespace
{

    StopTimeRecord st(std::string trip, uint32_t arr, uint32_t dep, std::string stop, uint32_t seq)
    {
        StopTimeRecord r;
        r.trip_id = std::move(trip);
        r.arrival_time = arr;
        r.departure_time = dep;
        r.stop_id = std::move(stop);
        r.stop_sequence = seq;
        r.pickup_type = 0;
        r.drop_off_type = 0;
        return r;
    }

    const std::unordered_map<std::string, uint32_t> kIdx = {
        {"A", 0}, {"B", 1}, {"C", 2}, {"D", 3}};

    /// A -> B -> C direct, every 10 minutes, five minutes per hop.
    /// B -> D on a separate service, so D always needs one change from A.
    std::vector<StopTimeRecord> two_service_network()
    {
        std::vector<StopTimeRecord> out;
        for (uint32_t r = 0; r < 24; ++r)
        {
            const uint32_t t = 25200 + r * 600;
            const std::string m = "M" + std::to_string(r);
            out.push_back(st(m, t, t, "A", 0));
            out.push_back(st(m, t + 300, t + 300, "B", 1));
            out.push_back(st(m, t + 600, t + 600, "C", 2));

            const std::string s = "S" + std::to_string(r);
            out.push_back(st(s, t + 420, t + 420, "B", 0));
            out.push_back(st(s, t + 720, t + 720, "D", 1));
        }
        return out;
    }

} // namespace

// With no change budget, only the stations on the direct service count. With a
// budget of one, D joins them. That difference IS the measurement.
TEST(Accessibility, ChangeBudgetChangesWhatCounts)
{
    auto idx = kIdx;
    auto tt = RaptorBuilder::build(two_service_network(), 4, &idx, {});

    AccessibilityConfig cfg;
    cfg.thresholds_s = {1800};              // 30 minutes
    cfg.departures = {25200, 28800, 32400}; // 07:00, 08:00, 09:00
    cfg.max_changes = 1;

    const auto surface = compute_accessibility(tt, {0}, cfg);
    ASSERT_EQ(surface.per_origin.size(), 1u);
    EXPECT_EQ(surface.num_budgets, 2u);

    EXPECT_DOUBLE_EQ(surface.count(0, 0, 0), 2.0); // B and C, no change
    EXPECT_DOUBLE_EQ(surface.count(0, 1, 0), 3.0); // and D, with one
    EXPECT_DOUBLE_EQ(surface.gap(0, 0), 1.0);
}

// The destination filter is what stops a platform-separated feed from counting
// every platform of a station as somewhere else you can get to. tools/isochrone
// passes one representative per station for exactly this reason, so the filter
// has to actually filter.
TEST(Accessibility, DestinationFilterRestrictsWhatIsCounted)
{
    auto idx = kIdx;
    auto tt = RaptorBuilder::build(two_service_network(), 4, &idx, {});

    AccessibilityConfig cfg;
    cfg.thresholds_s = {1800};
    cfg.departures = {25200, 28800};
    cfg.max_changes = 1;

    // Everything counts: B, C and D are all reachable from A within 30 minutes.
    const auto all = compute_accessibility(tt, {0}, cfg);
    EXPECT_DOUBLE_EQ(all.count(0, 1, 0), 3.0);

    // Count only C and D — B is reachable but is not a destination here.
    const auto some = compute_accessibility(tt, {0}, cfg, {2, 3});
    EXPECT_DOUBLE_EQ(some.count(0, 1, 0), 2.0);
    EXPECT_DOUBLE_EQ(some.count(0, 0, 0), 1.0) << "only C is reachable without changing";

    // The origin never counts as its own destination, even when listed.
    const auto with_origin = compute_accessibility(tt, {0}, cfg, {0, 2, 3});
    EXPECT_DOUBLE_EQ(with_origin.count(0, 1, 0), 2.0);
}

// A budget that excludes everything is a real answer, not a bug: the surface
// must report zero rather than falling back to a longer budget.
TEST(Accessibility, ATightTimeBudgetReachesNothing)
{
    auto idx = kIdx;
    auto tt = RaptorBuilder::build(two_service_network(), 4, &idx, {});

    AccessibilityConfig cfg;
    cfg.thresholds_s = {60}; // one minute
    cfg.departures = {28800};
    cfg.max_changes = 2;

    const auto surface = compute_accessibility(tt, {0}, cfg);
    EXPECT_DOUBLE_EQ(surface.count(0, 0, 0), 0.0);
    EXPECT_DOUBLE_EQ(surface.count(0, 2, 0), 0.0);
}

// Counts are means over EVERY sampled departure, including those with no
// service. Dividing by only the productive ones would flatter a station whose
// service stops in the evening, and accessibility is a claim about the day.
TEST(Accessibility, DeparturesWithoutServiceStillCountInTheDenominator)
{
    auto idx = kIdx;
    auto tt = RaptorBuilder::build(two_service_network(), 4, &idx, {});

    AccessibilityConfig cfg;
    cfg.thresholds_s = {1800};
    // 07:00 has service; 23:00 does not (the fixture stops before 21:00).
    cfg.departures = {25200, 82800};
    cfg.max_changes = 0;

    const auto surface = compute_accessibility(tt, {0}, cfg);
    EXPECT_DOUBLE_EQ(surface.count(0, 0, 0), 1.0); // 2 reachable, over 2 departures
    EXPECT_EQ(surface.per_origin[0].departures_with_service, 1u);
}

// More budget can never reach less: tau is non-increasing in the round index,
// so the counts must be monotone. A violation would mean the round layers are
// being read wrong.
TEST(Accessibility, CountsAreMonotoneInBothBudgets)
{
    auto idx = kIdx;
    auto tt = RaptorBuilder::build(two_service_network(), 4, &idx, {});

    AccessibilityConfig cfg;
    cfg.thresholds_s = {600, 1800, 3600};
    cfg.departures = {25200, 28800};
    cfg.max_changes = 2;

    const auto surface = compute_accessibility(tt, {0, 1}, cfg);
    for (uint32_t o = 0; o < surface.per_origin.size(); ++o)
    {
        for (uint32_t c = 0; c + 1 < surface.num_budgets; ++c)
            for (uint32_t t = 0; t < surface.num_thresholds; ++t)
                EXPECT_LE(surface.count(o, c, t), surface.count(o, c + 1, t))
                    << "origin " << o << " threshold " << t;
        for (uint32_t c = 0; c < surface.num_budgets; ++c)
            for (uint32_t t = 0; t + 1 < surface.num_thresholds; ++t)
                EXPECT_LE(surface.count(o, c, t), surface.count(o, c, t + 1))
                    << "origin " << o << " budget " << c;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// The renderer
// ═══════════════════════════════════════════════════════════════════════════

namespace
{

    std::vector<NodePlacement> square_placements()
    {
        return {{0, 12.90, 77.50, "West & South"},
                {1, 12.90, 77.70, "East <side>"},
                {2, 13.10, 77.70, "North \"end\""},
                {3, 13.10, 77.50, "Corner's"}};
    }

    std::size_t count_occurrences(const std::string &hay, const std::string &needle)
    {
        std::size_t n = 0, at = 0;
        while ((at = hay.find(needle, at)) != std::string::npos)
        {
            ++n;
            at += needle.size();
        }
        return n;
    }

} // namespace

// Station names come from third-party feeds and contain ampersands, angle
// brackets and apostrophes. Unescaped, they produce a file no viewer will open.
TEST(AccessibilityMap, FeedTextIsEscapedAndTheSvgIsWellFormed)
{
    const auto p = square_placements();
    const std::vector<std::pair<uint32_t, uint32_t>> links = {{0, 1}, {1, 2}, {2, 3}, {3, 0}};
    const std::vector<double> values = {0.0, 1.0, 2.0, 3.0};

    MapStyle style;
    style.title = "Title & test";
    style.labels = true;
    const std::string svg = render_station_map_svg(p, links, values, style);

    EXPECT_NE(svg.find("<svg"), std::string::npos);
    EXPECT_NE(svg.rfind("</svg>"), std::string::npos);
    EXPECT_EQ(count_occurrences(svg, "<circle"), 4u);
    EXPECT_EQ(count_occurrences(svg, "<line"), 4u);

    EXPECT_NE(svg.find("West &amp; South"), std::string::npos);
    EXPECT_NE(svg.find("East &lt;side&gt;"), std::string::npos);
    EXPECT_NE(svg.find("Title &amp; test"), std::string::npos);
    // A raw ampersand followed by a space is the classic malformed-XML signature.
    EXPECT_EQ(svg.find("& "), std::string::npos);
}

// One scale for both axes. Independently fitting each axis would make a
// north-south line and an east-west line of the same length look different,
// which misrepresents the only thing a transit map is for.
TEST(AccessibilityMap, ProjectionUsesOneScaleForBothAxes)
{
    // A region twice as wide in longitude as it is tall in latitude.
    std::vector<NodePlacement> p = {{0, 0.0, 0.0, "SW"}, {1, 0.0, 2.0, "SE"},
                                    {2, 1.0, 2.0, "NE"}, {3, 1.0, 0.0, "NW"}};
    MapStyle style;
    style.width = 1000;
    style.height = 1000;
    const std::string svg = render_station_map_svg(p, {}, {}, style);

    // Pull the four circle centres back out and check the drawn aspect ratio
    // matches the geographic one (cos(latitude) ~ 1 near the equator).
    std::vector<double> xs, ys;
    std::size_t at = 0;
    while ((at = svg.find("<circle cx=\"", at)) != std::string::npos)
    {
        at += std::string("<circle cx=\"").size();
        xs.push_back(std::stod(svg.substr(at)));
        const std::size_t cy = svg.find("cy=\"", at);
        ys.push_back(std::stod(svg.substr(cy + 4)));
    }
    ASSERT_EQ(xs.size(), 4u);

    const double dx = *std::max_element(xs.begin(), xs.end()) - *std::min_element(xs.begin(), xs.end());
    const double dy = *std::max_element(ys.begin(), ys.end()) - *std::min_element(ys.begin(), ys.end());
    EXPECT_NEAR(dx / dy, 2.0, 0.02);
}

// Degenerate inputs must produce a readable file, not a crash or a division by
// zero: a single station has zero geographic span in both directions.
TEST(AccessibilityMap, DegenerateInputsStillRender)
{
    EXPECT_NE(render_station_map_svg({}, {}, {}).find("no stations"), std::string::npos);

    const std::vector<NodePlacement> one = {{0, 12.97, 77.59, "Only"}};
    const std::string svg = render_station_map_svg(one, {{0, 0}}, {5.0});
    EXPECT_EQ(count_occurrences(svg, "<circle"), 1u);
    EXPECT_NE(svg.find("</svg>"), std::string::npos);
}

// A link naming a node that is not being drawn must be skipped, not dereferenced.
TEST(AccessibilityMap, LinksToUndrawnNodesAreSkipped)
{
    const auto p = square_placements();
    const std::vector<std::pair<uint32_t, uint32_t>> links = {{0, 1}, {1, 99}};
    const std::string svg = render_station_map_svg(p, links, {});
    EXPECT_EQ(count_occurrences(svg, "<line"), 1u);
}
