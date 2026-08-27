// tools/risk_probe.cpp — does dominance pruning survive a risk objective?
//
// WHAT THIS IS FOR
// ════════════════
// The engine is fast because it throws labels away. `docs/write-up.tex` §4 says
// that if delay were modelled stochastically the right objective would be CVaR,
// and leaves it there. CVaR does not accumulate along a path, so the question
// that has to be answered before any of that is worth building is whether a
// label-correcting search can prune at all — and if so, in what order.
//
// This prints the answer in two halves.
//
//   THE COUNTEREXAMPLES. Four instances, each small enough to read as
//   arithmetic, showing exactly where each candidate pruning rule loses the
//   optimum. They are built by include/risk.hpp, not here, so the witness this
//   prints and the witness tests/test_risk.cpp asserts cannot drift apart.
//
//   THE SWEEPS. Randomised networks measured against exhaustive enumeration
//   over the same journey set. Three questions: how often does an unsafe rule
//   actually cost anything, what does the safe rule cost in frontier size, and
//   does that cost grow with how finely the delay distribution is resolved.
//   The last one is the one that decides whether the direction is viable.
//
// It reads no feed and uses no delay data, by design — every claim here is
// structural, and a structural claim cannot be made true by realistic numbers or
// false by unrealistic ones. See the header for why that is a deliberate scope
// boundary rather than a missing feature.
//
// USAGE
//   routing_engine_risk_probe [--trials N] [--nodes N] [--out-degree N]
//                             [--headway S] [--out-prefix PATH] [--no-csv]

#include "risk.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace namma_metro;

namespace
{

    const char *order_name(DominanceOrder o)
    {
        switch (o)
        {
        case DominanceOrder::None:
            return "none";
        case DominanceOrder::Statewise:
            return "statewise";
        case DominanceOrder::FirstOrderStochastic:
            return "stochastic";
        case DominanceOrder::AllTailAverages:
            return "all-tail-averages";
        case DominanceOrder::Scalar:
            return "scalar";
        }
        return "?";
    }

    /// Ordered weakest-pruning first, so every table reads down the chain in the
    /// same direction the header describes it.
    const DominanceOrder kOrders[] = {
        DominanceOrder::None,
        DominanceOrder::Statewise,
        DominanceOrder::FirstOrderStochastic,
        DominanceOrder::AllTailAverages,
        DominanceOrder::Scalar,
    };

    std::string law_text(const ArrivalLaw &law)
    {
        std::ostringstream s;
        s << "[";
        for (std::size_t i = 0; i < law.size(); ++i)
            s << (i ? " " : "") << law.by_scenario()[i];
        s << "]";
        return s.str();
    }

    void print_law(const char *tag, const ArrivalLaw &law, std::size_t m)
    {
        std::printf("    %-26s %-24s mean %7.1f   CVaR %7.1f\n",
                    tag, law_text(law).c_str(), law.mean(),
                    static_cast<double>(law.tail_total(m)) / static_cast<double>(m));
    }

    void print_search_witness(const char *title, const char *claim, const SearchWitness &w)
    {
        std::printf("\n  %s\n  %s\n", title, claim);

        const ArrivalLaw source = ArrivalLaw::certain(w.network.num_scenarios, w.departure_time_s);
        const ArrivalLaw a = extend(source, w.network.legs[0]);
        const ArrivalLaw b = extend(source, w.network.legs[1]);

        std::printf("    %zu equally likely days; CVaR is the mean of the worst %zu.\n",
                    w.network.num_scenarios, w.tail_count);
        print_law("route A at interchange", a, w.tail_count);
        print_law("route B at interchange", b, w.tail_count);
        std::printf("    %s dominance: A over B %s, B over A %s  ->  B is discarded\n",
                    order_name(w.unsafe_rule.order),
                    dominates(a, b, w.unsafe_rule) ? "yes" : "no",
                    dominates(b, a, w.unsafe_rule) ? "yes" : "no");

        const ArrivalLaw ax = extend(a, w.network.legs[2]);
        const ArrivalLaw bx = extend(b, w.network.legs[2]);
        print_law("route A at destination", ax, w.tail_count);
        print_law("route B at destination", bx, w.tail_count);

        RiskSearchConfig cfg;
        cfg.max_legs = 3;
        const auto all = enumerate_journeys(w.network, w.source, w.departure_time_s, cfg.max_legs);
        const auto truth = best_tail_total(all, w.destination, w.tail_count);

        const PruningRule rules[] = {w.unsafe_rule, w.safe_rule};
        for (const PruningRule &rule : rules)
        {
            cfg.rule = rule;
            const auto r = risk_search(w.network, w.source, w.departure_time_s, cfg);
            const auto found = best_tail_total(r, w.destination, w.tail_count);
            const double best =
                found ? static_cast<double>(*found) / static_cast<double>(w.tail_count) : 0.0;
            std::printf("    pruning by %-18s -> CVaR %7.1f   %s\n",
                        order_name(rule.order), best,
                        (found && truth && *found == *truth)
                            ? "optimal"
                            : "WRONG (the optimum was discarded)");
        }
        if (truth)
            std::printf("    exhaustive enumeration        -> CVaR %7.1f   the answer\n",
                        static_cast<double>(*truth) / static_cast<double>(w.tail_count));
    }

    struct SweepRow
    {
        std::string sweep;
        std::string parameter;
        int64_t value = 0;
        std::size_t scenarios = 0;
        DominanceOrder order = DominanceOrder::None;
        int trials = 0;
        int inexact = 0;
        double mean_frontier = 0.0;
        std::size_t max_frontier = 0;
        double risky_prunings = 0.0;
        double mean_regret_s = 0.0;
    };

    /// One column of a sweep: the same instance family, seeds 1..trials, measured
    /// under all five orders. The caller varies exactly one field of @p base per
    /// column, so a difference between two columns has exactly one cause.
    std::vector<SweepRow> sweep_column(const RandomInstanceConfig &base,
                                       const std::string &sweep,
                                       const std::string &parameter,
                                       int64_t value,
                                       int trials,
                                       std::size_t tail_count)
    {
        std::vector<SweepRow> rows;
        std::size_t scenarios = 0;

        for (const DominanceOrder order : kOrders)
        {
            SweepRow row;
            row.sweep = sweep;
            row.parameter = parameter;
            row.value = value;
            row.order = order;
            row.trials = trials;

            double frontier_sum = 0.0;
            double regret_sum = 0.0;
            double risky_sum = 0.0;

            for (int t = 1; t <= trials; ++t)
            {
                RandomInstanceConfig cfg = base;
                cfg.seed = static_cast<uint64_t>(t);
                const RandomInstance inst = make_random_instance(cfg);
                scenarios = inst.network.num_scenarios;

                const RuleMeasurement m =
                    measure_rule(inst, PruningRule{order, tail_count},
                                 static_cast<uint32_t>(cfg.num_nodes - 1));

                frontier_sum += m.mean_frontier;
                risky_sum += static_cast<double>(m.unsupported_prunings);
                row.max_frontier = std::max(row.max_frontier, m.max_frontier);
                if (!m.exact)
                {
                    ++row.inexact;
                    regret_sum += static_cast<double>(m.regret_tail_total) /
                                  static_cast<double>(tail_count);
                }
            }

            row.scenarios = scenarios;
            row.mean_frontier = frontier_sum / trials;
            row.risky_prunings = risky_sum / trials;
            row.mean_regret_s = row.inexact ? regret_sum / row.inexact : 0.0;
            rows.push_back(row);
        }
        return rows;
    }

    void print_exactness(const std::vector<SweepRow> &rows, int trials)
    {
        std::printf("    %-10s", "disruption");
        for (const DominanceOrder o : kOrders)
            if (o != DominanceOrder::None)
                std::printf(" %18s", order_name(o));
        std::printf("\n");

        for (std::size_t i = 0; i < rows.size(); i += 5)
        {
            std::printf("    %6lld s  ", static_cast<long long>(rows[i].value));
            for (std::size_t k = 1; k < 5; ++k)
            {
                const SweepRow &r = rows[i + k];
                char cell[32];
                std::snprintf(cell, sizeof cell, "%d/%d", r.inexact, trials);
                std::printf(" %18s", cell);
            }
            std::printf("\n");
        }
    }

    void print_frontier(const std::vector<SweepRow> &rows, const char *label)
    {
        std::printf("    %-12s %8s", label, "scen");
        for (const DominanceOrder o : kOrders)
            std::printf(" %12s", order_name(o));
        std::printf("\n");

        for (std::size_t i = 0; i < rows.size(); i += 5)
        {
            std::printf("    %-12lld %8zu", static_cast<long long>(rows[i].value), rows[i].scenarios);
            for (std::size_t k = 0; k < 5; ++k)
                std::printf(" %12.2f", rows[i + k].mean_frontier);
            std::printf("\n");
        }
    }

    bool write_csv(const std::string &path, const std::vector<SweepRow> &rows)
    {
        std::ofstream f(path, std::ios::binary);
        if (!f)
        {
            std::fprintf(stderr, "cannot write %s\n", path.c_str());
            return false;
        }
        f << "sweep,parameter,value,scenarios,order,trials,inexact,mean_frontier,"
             "max_frontier,risky_prunings_per_instance,mean_regret_s\n";
        for (const SweepRow &r : rows)
        {
            f << r.sweep << ',' << r.parameter << ',' << r.value << ',' << r.scenarios << ','
              << order_name(r.order) << ',' << r.trials << ',' << r.inexact << ','
              << r.mean_frontier << ',' << r.max_frontier << ',' << r.risky_prunings << ','
              << r.mean_regret_s << '\n';
        }
        return true;
    }

} // namespace

int main(int argc, char **argv)
{
    int trials = 400;
    uint32_t nodes = 4;
    uint32_t out_degree = 3;
    int64_t headway = 900;
    std::string out_prefix = "risk-probe";
    bool csv = true;

    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        if (a == "--trials" && i + 1 < argc)
            trials = std::stoi(argv[++i]);
        else if (a == "--nodes" && i + 1 < argc)
            nodes = static_cast<uint32_t>(std::stoul(argv[++i]));
        else if (a == "--out-degree" && i + 1 < argc)
            out_degree = static_cast<uint32_t>(std::stoul(argv[++i]));
        else if (a == "--headway" && i + 1 < argc)
            headway = std::stoll(argv[++i]);
        else if (a == "--out-prefix" && i + 1 < argc)
            out_prefix = argv[++i];
        else if (a == "--no-csv")
            csv = false;
        else
        {
            std::fprintf(stderr,
                         "usage: routing_engine_risk_probe [--trials N] [--nodes N]\n"
                         "       [--out-degree N] [--headway S] [--out-prefix PATH] [--no-csv]\n");
            return 2;
        }
    }
    if (trials < 1 || nodes < 3 || out_degree < 1 || headway < 1)
    {
        std::fprintf(stderr, "risk_probe: parameters out of range\n");
        return 2;
    }

    RandomInstanceConfig base;
    base.num_nodes = nodes;
    base.out_degree = out_degree;
    base.delay_support = 4;
    base.delay_spread_s = 2 * headway;
    base.headway_s = headway;
    base.ride_s = 300;
    base.max_scenarios = 1 << 20;

    std::printf("═══════════════════════════════════════════════════════════════════\n");
    std::printf(" Risk-objective probe — which dominance orders survive a timetable\n");
    std::printf("═══════════════════════════════════════════════════════════════════\n");
    std::printf("\nNo feed is read and no delay data is used. Every instance below is\n"
                "synthetic and exactly reproducible; every claim is structural.\n");

    // ── 1. The counterexamples ───────────────────────────────────────────────
    std::printf("\n\n1. THE COUNTEREXAMPLES\n"
                "──────────────────────\n");

    {
        const LabelWitness w = scalar_label_witness();
        std::printf("\n  A scalar risk label cannot be accumulated at all.\n"
                    "  Two journeys agreeing EXACTLY on both numbers such a label carries.\n");
        print_law("route A", w.a, w.tail_count);
        print_law("route B", w.b, w.tail_count);
        std::printf("    ... then one more leg, and they agree on neither:\n");
        print_law("route A, one leg on", extend(w.a, w.leg), w.tail_count);
        print_law("route B, one leg on", extend(w.b, w.leg), w.tail_count);
        std::printf("    So no rule of any shape takes (mean, CVaR) to the extended\n"
                    "    (mean, CVaR). The obstacle is not that CVaR fails to add up.\n");
    }

    print_search_witness(
        "Pruning on (mean, CVaR) discards the prefix of the optimum.",
        "Route B is worse on both numbers at the interchange, and better on both after it.",
        scalar_pruning_witness());

    print_search_witness(
        "Pruning on CVaR at EVERY confidence level discards it too.",
        "Order by the objective, all of it, and the pruning is still wrong.",
        tail_average_pruning_witness());

    print_search_witness(
        "With correlated delays, stochastic dominance discards it as well.",
        "Route B is stochastically later, and better: it is late when its connection is.",
        correlated_delay_witness());

    {
        const ProxyWitness w = additive_proxy_witness();
        const auto journeys = enumerate_journeys(w.network, 0, w.departure_time_s, 2);
        const ArrivalLaw *arrival = nullptr;
        for (const Journey &j : journeys)
            if (j.legs == w.journey)
                arrival = &j.law;

        std::printf("\n  Summing per-leg CVaR is not a bound on the journey's CVaR.\n"
                    "  Coherence bounds a sum of random costs. An arrival is not one.\n");
        if (arrival)
        {
            std::printf("    arrival             %s\n", law_text(*arrival).c_str());
            std::printf("    true CVaR           %7.1f s\n", arrival->cvar(w.alpha));
        }
        const double proxy = additive_cvar_proxy(w.network, w.journey, w.departure_time_s, w.alpha);
        std::printf("    additive proxy      %7.1f s\n", proxy);
        if (arrival)
            std::printf("    the proxy under-states by %.0f s: a two-minute delay, and the\n"
                        "    bill is a whole headway.\n",
                        arrival->cvar(w.alpha) - proxy);
    }

    // ── 2. The sweeps ────────────────────────────────────────────────────────
    std::vector<SweepRow> rows;

    std::printf("\n\n2. HOW OFTEN AN UNSAFE ORDER ACTUALLY COSTS SOMETHING\n"
                "─────────────────────────────────────────────────────\n");
    std::printf("\n  %d random networks per row, each measured against exhaustive\n"
                "  enumeration over the same journey set. Disruption is the number of\n"
                "  seconds the whole network runs late on half of all scenarios: 0 is\n"
                "  the independent regime the sufficiency proof covers.\n\n",
                trials);

    std::vector<SweepRow> exactness;
    for (const int64_t disruption : {int64_t{0}, headway / 8, headway / 4, (3 * headway) / 8,
                                     headway / 2, (5 * headway) / 8, (3 * headway) / 4, headway})
    {
        RandomInstanceConfig cfg = base;
        cfg.disruption_s = disruption;
        const auto column = sweep_column(cfg, "exactness", "disruption_s", disruption, trials, 1);
        exactness.insert(exactness.end(), column.begin(), column.end());
    }
    print_exactness(exactness, trials);
    rows.insert(rows.end(), exactness.begin(), exactness.end());
    std::printf("\n  A disruption of exactly one headway leaves every timetable where it\n"
                "  was, so the last row must read like the first. It is the phase, not\n"
                "  the lateness, that costs the marginal its sufficiency.\n");

    // ── 3. Frontier cost ─────────────────────────────────────────────────────
    std::printf("\n\n3. WHAT THE SAFE ORDERS COST, AS THE DELAYS WIDEN\n"
                "─────────────────────────────────────────────────\n");
    std::printf("\n  Mean labels per reached node. Delays independent throughout.\n\n");

    std::vector<SweepRow> by_spread;
    for (const int64_t spread : {int64_t{0}, headway / 6, headway / 3, headway * 2 / 3, headway,
                                 headway * 2, headway * 4})
    {
        RandomInstanceConfig cfg = base;
        cfg.delay_spread_s = spread;
        const auto column = sweep_column(cfg, "frontier-vs-spread", "delay_spread_s", spread, trials, 1);
        by_spread.insert(by_spread.end(), column.begin(), column.end());
    }
    print_frontier(by_spread, "spread (s)");
    rows.insert(rows.end(), by_spread.begin(), by_spread.end());

    std::printf("\n\n4. AND AS THE DISTRIBUTION IS RESOLVED MORE FINELY\n"
                "──────────────────────────────────────────────────\n");
    std::printf("\n  The question that decides whether any of this scales: does the\n"
                "  frontier grow with the number of scenarios carried in a label?\n\n");

    std::vector<SweepRow> by_resolution;
    for (const int64_t support : {int64_t{2}, int64_t{3}, int64_t{4}, int64_t{5}, int64_t{6}, int64_t{8}})
    {
        RandomInstanceConfig cfg = base;
        cfg.delay_support = static_cast<uint32_t>(support);
        const auto column =
            sweep_column(cfg, "frontier-vs-resolution", "delay_support", support, trials, 1);
        by_resolution.insert(by_resolution.end(), column.begin(), column.end());
    }
    print_frontier(by_resolution, "atoms/leg");
    rows.insert(rows.end(), by_resolution.begin(), by_resolution.end());

    if (csv)
    {
        const std::string path = out_prefix + "-orders.csv";
        if (!write_csv(path, rows))
            return 1;
        std::printf("\n\nWrote %s (%zu rows).\n", path.c_str(), rows.size());
    }

    std::printf("\n");
    return 0;
}
