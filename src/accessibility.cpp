#include "accessibility.hpp"
#include "gtfs_parser.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <unordered_map>

namespace namma_metro
{

    AccessibilitySurface compute_accessibility(
        const RaptorTimetable &tt,
        const std::vector<uint32_t> &origins,
        const AccessibilityConfig &config,
        const std::vector<uint32_t> &destinations)
    {
        AccessibilitySurface surface;
        surface.config = config;
        surface.num_stops = tt.num_stops;
        surface.num_thresholds = static_cast<uint32_t>(config.thresholds_s.size());
        surface.num_budgets = config.max_changes + 1;
        surface.per_origin.reserve(origins.size());

        // Materialise the destination set once. Empty means every stop.
        std::vector<uint32_t> dests;
        if (destinations.empty())
        {
            dests.reserve(tt.num_stops);
            for (uint32_t v = 0; v < tt.num_stops; ++v)
                dests.push_back(v);
        }
        else
        {
            dests = destinations;
        }

        // A change budget of c means at most c + 1 vehicles, i.e. RAPTOR round
        // c + 1. Ask the engine for one more round than the largest budget so
        // the deepest layer is genuinely computed rather than clamped.
        Raptor raptor(tt, std::max(config.max_changes + 2u, 4u));
        RaptorResult rr;

        for (const uint32_t origin : origins)
        {
            StationAccessibility sa;
            sa.node = origin;
            sa.counts.assign(static_cast<std::size_t>(surface.num_budgets) * surface.num_thresholds, 0.0);

            for (const uint32_t dep : config.departures)
            {
                raptor.run(origin, dep, rr);

                bool any = false;
                for (uint32_t c = 0; c <= config.max_changes; ++c)
                {
                    // tau is non-increasing in the round index, so "at most c
                    // changes" is one layer lookup, not a scan.
                    const uint32_t round = std::min(c + 1u, rr.rounds);
                    for (const uint32_t v : dests)
                    {
                        if (v == origin || v >= tt.num_stops)
                            continue;
                        const uint32_t arr = rr.arrival(round, v);
                        if (arr == RAPTOR_UNREACHED)
                            continue;
                        any = true;
                        // Departure times are inside the service day and arrivals
                        // are never earlier, so this subtraction cannot wrap.
                        const uint32_t elapsed = arr - dep;
                        for (uint32_t t = 0; t < surface.num_thresholds; ++t)
                            if (elapsed <= config.thresholds_s[t])
                                sa.counts[static_cast<std::size_t>(c) * surface.num_thresholds + t] += 1.0;
                    }
                }
                if (any)
                    ++sa.departures_with_service;
            }

            // Mean over ALL sampled departures, including those with no service.
            // Dividing by only the productive departures would flatter a station
            // whose service stops at 20:00 by pretending the evening does not
            // exist, and accessibility is precisely a claim about the whole day.
            const double n = static_cast<double>(config.departures.size());
            if (n > 0.0)
                for (double &x : sa.counts)
                    x /= n;

            surface.per_origin.push_back(std::move(sa));
        }

        return surface;
    }

    std::vector<NodePlacement> placements_from_stops(
        const std::vector<StopRecord> &stops,
        const std::unordered_map<std::string, uint32_t> &stop_index_map,
        const std::vector<uint32_t> &nodes)
    {
        std::unordered_map<uint32_t, const StopRecord *> by_node;
        by_node.reserve(stops.size() * 2);
        for (const auto &s : stops)
        {
            const auto it = stop_index_map.find(s.stop_id);
            if (it != stop_index_map.end())
                by_node.emplace(it->second, &s);
        }

        std::vector<NodePlacement> out;
        out.reserve(nodes.size());
        for (const uint32_t n : nodes)
        {
            const auto it = by_node.find(n);
            if (it == by_node.end())
                continue;
            NodePlacement p;
            p.node = n;
            p.lat = it->second->stop_lat;
            p.lon = it->second->stop_lon;
            p.name = it->second->stop_name;
            out.push_back(std::move(p));
        }
        return out;
    }

    namespace
    {

        /// Escape the five characters that would otherwise break XML. Station
        /// names come from third-party feeds and do contain ampersands.
        std::string xml_escape(const std::string &s)
        {
            std::string out;
            out.reserve(s.size());
            for (const char c : s)
            {
                switch (c)
                {
                case '&': out += "&amp;"; break;
                case '<': out += "&lt;"; break;
                case '>': out += "&gt;"; break;
                case '"': out += "&quot;"; break;
                case '\'': out += "&apos;"; break;
                default:
                    // Drop control characters rather than emit invalid XML.
                    if (static_cast<unsigned char>(c) >= 0x20)
                        out.push_back(c);
                }
            }
            return out;
        }

        /// A five-stop sequential ramp, light to dark, chosen so the ordering
        /// survives greyscale printing and the darkest end reads as "most".
        /// Interpolation is done in sRGB, which is not perceptually uniform;
        /// the stops are spaced to compensate rather than pretending otherwise.
        std::string ramp(double x)
        {
            static const int stops[5][3] = {
                {0xEF, 0xF3, 0xFF}, {0xBD, 0xD7, 0xE7}, {0x6B, 0xAE, 0xD6}, {0x31, 0x82, 0xBD}, {0x08, 0x51, 0x9C}};
            if (!(x >= 0.0))
                x = 0.0;
            if (x > 1.0)
                x = 1.0;
            const double scaled = x * 4.0;
            const int i = std::min(3, static_cast<int>(scaled));
            const double f = scaled - i;
            char buf[8];
            std::snprintf(buf, sizeof(buf), "#%02X%02X%02X",
                          static_cast<int>(stops[i][0] + (stops[i + 1][0] - stops[i][0]) * f),
                          static_cast<int>(stops[i][1] + (stops[i + 1][1] - stops[i][1]) * f),
                          static_cast<int>(stops[i][2] + (stops[i + 1][2] - stops[i][2]) * f));
            return std::string(buf);
        }

    } // namespace

    std::string render_station_map_svg(
        const std::vector<NodePlacement> &placements,
        const std::vector<std::pair<uint32_t, uint32_t>> &links,
        const std::vector<double> &values,
        const MapStyle &style)
    {
        std::ostringstream svg;
        const double W = style.width, H = style.height, M = style.margin;
        const double legend_h = 54.0;

        svg << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
            << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << style.width
            << "\" height=\"" << style.height << "\" viewBox=\"0 0 " << style.width
            << " " << style.height << "\" font-family=\"system-ui, sans-serif\">\n"
            << "<rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>\n";

        if (placements.empty())
        {
            svg << "<text x=\"" << W / 2 << "\" y=\"" << H / 2
                << "\" text-anchor=\"middle\" fill=\"#666\" font-size=\"16\">"
                << "no stations to draw</text>\n</svg>\n";
            return svg.str();
        }

        // ── Projection ────────────────────────────────────────────────────────
        // Equirectangular with longitude scaled by cos(mean latitude), which
        // keeps a city-scale map from stretching east-west. Anything more is a
        // projection library, and a metro network spans a few tens of kilometres.
        double min_lat = 90.0, max_lat = -90.0, min_lon = 180.0, max_lon = -180.0, sum_lat = 0.0;
        for (const auto &p : placements)
        {
            min_lat = std::min(min_lat, p.lat);
            max_lat = std::max(max_lat, p.lat);
            min_lon = std::min(min_lon, p.lon);
            max_lon = std::max(max_lon, p.lon);
            sum_lat += p.lat;
        }
        const double k = std::cos(sum_lat / placements.size() * M_PI / 180.0);
        const double span_x = std::max(1e-9, (max_lon - min_lon) * k);
        const double span_y = std::max(1e-9, max_lat - min_lat);
        const double plot_w = W - 2 * M;
        const double plot_h = H - 2 * M - legend_h;
        // One scale for both axes: an independently stretched map is a lie about
        // distance, and distance is the whole subject here.
        const double scale = std::min(plot_w / span_x, plot_h / span_y);
        const double off_x = M + (plot_w - span_x * scale) / 2.0;
        const double off_y = M + (plot_h - span_y * scale) / 2.0;

        std::unordered_map<uint32_t, std::size_t> slot;
        slot.reserve(placements.size() * 2);
        for (std::size_t i = 0; i < placements.size(); ++i)
            slot.emplace(placements[i].node, i);

        auto px = [&](const NodePlacement &p) { return off_x + (p.lon - min_lon) * k * scale; };
        auto py = [&](const NodePlacement &p) { return off_y + (max_lat - p.lat) * scale; };

        // ── Title ─────────────────────────────────────────────────────────────
        double text_y = 28.0;
        if (!style.title.empty())
        {
            svg << "<text x=\"" << M << "\" y=\"" << text_y
                << "\" font-size=\"18\" font-weight=\"600\" fill=\"#111\">"
                << xml_escape(style.title) << "</text>\n";
            text_y += 20.0;
        }
        if (!style.subtitle.empty())
            svg << "<text x=\"" << M << "\" y=\"" << text_y
                << "\" font-size=\"12\" fill=\"#555\">" << xml_escape(style.subtitle) << "</text>\n";

        // ── Links ─────────────────────────────────────────────────────────────
        svg << "<g stroke=\"#c9ced6\" stroke-width=\"1.6\" stroke-linecap=\"round\">\n";
        for (const auto &e : links)
        {
            const auto a = slot.find(e.first), b = slot.find(e.second);
            if (a == slot.end() || b == slot.end())
                continue;
            svg << "<line x1=\"" << px(placements[a->second]) << "\" y1=\"" << py(placements[a->second])
                << "\" x2=\"" << px(placements[b->second]) << "\" y2=\"" << py(placements[b->second])
                << "\"/>\n";
        }
        svg << "</g>\n";

        // ── Stations ──────────────────────────────────────────────────────────
        double vmin = 0.0, vmax = 0.0;
        const bool coloured = values.size() == placements.size();
        if (coloured)
        {
            vmin = *std::min_element(values.begin(), values.end());
            vmax = *std::max_element(values.begin(), values.end());
        }
        const double vrange = (vmax > vmin) ? (vmax - vmin) : 1.0;

        svg << "<g stroke=\"#37414f\" stroke-width=\"1\">\n";
        for (std::size_t i = 0; i < placements.size(); ++i)
        {
            const std::string fill = coloured ? ramp((values[i] - vmin) / vrange) : "#6BAED6";
            svg << "<circle cx=\"" << px(placements[i]) << "\" cy=\"" << py(placements[i])
                << "\" r=\"4.5\" fill=\"" << fill << "\"><title>"
                << xml_escape(placements[i].name);
            if (coloured)
                svg << " — " << values[i];
            svg << "</title></circle>\n";
        }
        svg << "</g>\n";

        if (style.labels)
        {
            svg << "<g font-size=\"9\" fill=\"#333\">\n";
            for (const auto &p : placements)
                svg << "<text x=\"" << px(p) + 6.0 << "\" y=\"" << py(p) + 3.0 << "\">"
                    << xml_escape(p.name) << "</text>\n";
            svg << "</g>\n";
        }

        // ── Legend ────────────────────────────────────────────────────────────
        if (coloured)
        {
            const double lx = M, ly = H - legend_h + 8.0, lw = 240.0, lh = 12.0;
            for (int i = 0; i < 60; ++i)
                svg << "<rect x=\"" << lx + lw * i / 60.0 << "\" y=\"" << ly
                    << "\" width=\"" << (lw / 60.0 + 0.6) << "\" height=\"" << lh
                    << "\" fill=\"" << ramp(i / 59.0) << "\"/>\n";
            svg << "<text x=\"" << lx << "\" y=\"" << ly + lh + 14.0
                << "\" font-size=\"11\" fill=\"#333\">" << vmin << "</text>\n"
                << "<text x=\"" << lx + lw << "\" y=\"" << ly + lh + 14.0
                << "\" font-size=\"11\" fill=\"#333\" text-anchor=\"end\">" << vmax << "</text>\n"
                << "<text x=\"" << lx + lw + 14.0 << "\" y=\"" << ly + lh - 1.0
                << "\" font-size=\"11\" fill=\"#333\">" << xml_escape(style.legend_label) << "</text>\n";
        }

        svg << "</svg>\n";
        return svg.str();
    }

} // namespace namma_metro
