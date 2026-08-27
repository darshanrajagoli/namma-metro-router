#include <gtest/gtest.h>

#include "risk.hpp"

#include <cstdint>
#include <random>
#include <vector>

/**
 * @file test_risk.cpp
 * @brief Contracts for the risk-objective probe.
 *
 * WHAT THESE TESTS ARE FOR
 * ════════════════════════
 * Most of the file is not coverage. It is the argument of `include/risk.hpp`,
 * written as assertions, so that a claim in `docs/risk.md` cannot quietly stop
 * being true.
 *
 * The counterexamples are constructed by the library rather than here — see the
 * note above §6 of the header — so a test and the tool that prints the same
 * witness cannot come to disagree about what it shows.
 *
 * Two of these are worth more than the rest:
 *
 *   · `StochasticPruningMatchesEnumerationOnIndependentInstances` runs the
 *     pruned search and an exhaustive enumeration over the same journey set on
 *     randomised networks and requires them to agree exactly. That is the
 *     sufficiency half of the main result checked against something that cannot
 *     have made the same mistake, in the spirit of
 *     `RaptorVsEngine.UnrestrictedEngineAgreesExactlyOnEarliestArrival`.
 *
 *   · `StochasticDominanceIsNecessary` asserts the converse, constructively: for
 *     every pair the order does not relate, the library produces a legal leg
 *     that reverses them. Together these say the order is not a choice.
 *
 * The randomised tests use `std::mt19937` and plain modulo rather than
 * `std::uniform_int_distribution`, which is not specified to produce the same
 * values across standard library implementations. A randomised test that draws
 * different networks on a different toolchain is a test that fails somewhere
 * else than where it was written.
 */

using namespace namma_metro;

namespace
{

    /// Deterministic on every implementation, unlike the <random> distributions.
    struct Draws
    {
        std::mt19937 rng;
        explicit Draws(uint32_t seed) : rng(seed) {}
        int64_t below(int64_t n) { return n <= 0 ? 0 : static_cast<int64_t>(rng() % static_cast<uint32_t>(n)); }
        std::vector<int64_t> vec(std::size_t n, int64_t hi)
        {
            std::vector<int64_t> v(n);
            for (auto &x : v)
                x = below(hi);
            return v;
        }
    };

    /// A scenario space Ω = {prefix outcomes} × {leg outcomes}, indexed
    /// w = p·nd + d, so a law built by @ref product_prefix depends only on p and
    /// a delay built by @ref product_delay only on d. That is exact
    /// independence, which is the hypothesis the sufficiency lemma needs — not
    /// an approximation of it.
    ArrivalLaw product_prefix(const std::vector<int64_t> &prefix, std::size_t nd)
    {
        std::vector<int64_t> v;
        v.reserve(prefix.size() * nd);
        for (const int64_t p : prefix)
            for (std::size_t d = 0; d < nd; ++d)
                v.push_back(p);
        return ArrivalLaw(std::move(v));
    }

    std::vector<int64_t> product_delay(const std::vector<int64_t> &delay, std::size_t np)
    {
        std::vector<int64_t> v;
        v.reserve(delay.size() * np);
        for (std::size_t p = 0; p < np; ++p)
            for (const int64_t d : delay)
                v.push_back(d);
        return v;
    }

    StochasticLeg simple_leg(int64_t offset, int64_t headway, int64_t ride,
                             std::vector<int64_t> delay = {})
    {
        StochasticLeg leg;
        leg.from = 0;
        leg.to = 1;
        leg.offset_s = offset;
        leg.headway_s = headway;
        leg.ride_s = ride;
        leg.delay_s = std::move(delay);
        return leg;
    }

    /// The two journeys a SearchWitness offers out of the source, in leg order.
    ArrivalLaw at_interchange(const SearchWitness &w, std::size_t leg)
    {
        return extend(ArrivalLaw::certain(w.network.num_scenarios, w.departure_time_s),
                      w.network.legs[leg]);
    }

    int64_t optimum_at_destination(const SearchWitness &w, const PruningRule &rule)
    {
        RiskSearchConfig cfg;
        cfg.rule = rule;
        cfg.max_legs = 3;
        const auto r = risk_search(w.network, w.source, w.departure_time_s, cfg);
        const auto best = best_tail_total(r, w.destination, w.tail_count);
        EXPECT_TRUE(best.has_value());
        return best.value_or(-1);
    }

    int64_t truth_at_destination(const SearchWitness &w)
    {
        const auto all = enumerate_journeys(w.network, w.source, w.departure_time_s, 3);
        const auto best = best_tail_total(all, w.destination, w.tail_count);
        EXPECT_TRUE(best.has_value());
        return best.value_or(-1);
    }

} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// The risk functionals
// ═══════════════════════════════════════════════════════════════════════════

TEST(RiskProbe, CvarAtZeroConfidenceIsTheMean)
{
    const ArrivalLaw law({0, 10, 20, 90});
    EXPECT_DOUBLE_EQ(law.mean(), 30.0);
    EXPECT_DOUBLE_EQ(law.cvar(0.0), law.mean());
    EXPECT_EQ(law.total(), 120);
}

TEST(RiskProbe, CvarAtScenarioBoundariesIsTheTailAverage)
{
    // Rounding the tail to a whole scenario instead of integrating the quantile
    // function gives answers that look right and are not. At the boundaries the
    // two must coincide exactly, which is what makes the interpolation below
    // checkable at all.
    const ArrivalLaw law({0, 10, 20, 90});
    const std::size_t n = law.size();
    for (std::size_t m = 1; m <= n; ++m)
    {
        const double alpha = 1.0 - static_cast<double>(m) / static_cast<double>(n);
        EXPECT_NEAR(law.cvar(alpha),
                    static_cast<double>(law.tail_total(m)) / static_cast<double>(m), 1e-9)
            << "at m = " << m;
    }
}

TEST(RiskProbe, CvarBetweenBoundariesInterpolatesTheBoundaryScenario)
{
    // alpha = 0.6 over four scenarios puts 1.6 scenarios in the tail: all of the
    // worst one and 60% of the next. Rounding either way moves the answer by
    // tens of seconds on these instances, which is the difference between
    // catching a connection and missing it.
    const ArrivalLaw law({0, 10, 20, 90});
    EXPECT_NEAR(law.cvar(0.6), 63.75, 1e-9);
    EXPECT_DOUBLE_EQ(law.var(0.6), 20.0);
    EXPECT_DOUBLE_EQ(law.cvar(1.0), 90.0); // the tail closes on the worst day
}

TEST(RiskProbe, CvarIsMonotoneUnderStochasticDominance)
{
    // The other half of what makes stochastic dominance the right order: it is
    // no use pruning safely if the objective does not respect the order.
    Draws d(20260827u);
    for (int trial = 0; trial < 400; ++trial)
    {
        const ArrivalLaw a(d.vec(6, 3600));
        const ArrivalLaw b(d.vec(6, 3600));
        if (!dominates_stochastically(a, b))
            continue;
        for (double alpha : {0.0, 0.25, 0.5, 0.75, 0.9})
            EXPECT_LE(a.cvar(alpha), b.cvar(alpha) + 1e-9) << "alpha " << alpha;
    }
}

TEST(RiskProbe, CvarAtEveryLevelDeterminesTheDistribution)
{
    // This is why "carry more confidence levels" is not an escape from the
    // scalar impossibility: carrying all of them is carrying the distribution.
    Draws d(7u);
    for (int trial = 0; trial < 200; ++trial)
    {
        const ArrivalLaw law(d.vec(5, 5000));
        std::vector<int64_t> profile;
        for (std::size_t m = 0; m <= law.size(); ++m)
            profile.push_back(law.tail_total(m));
        EXPECT_EQ(sorted_from_tail_totals(profile), law.sorted());
    }

    // A profile whose differences increase is not the tail profile of anything.
    EXPECT_THROW(sorted_from_tail_totals({0, 10, 30}), std::invalid_argument);
}

// ═══════════════════════════════════════════════════════════════════════════
// The timetable
// ═══════════════════════════════════════════════════════════════════════════

TEST(RiskProbe, AnArrivalBeforeTheOffsetBoardsTheEarlierDeparture)
{
    // Service is periodic in both directions, so with a departure at 1000 and a
    // 600 s headway there is one at 400 as well. Reaching for the usual
    // (a + b - 1) / b ceiling idiom truncates the negative numerator toward
    // zero, returns the 1000 departure, and invents a ten-minute wait that the
    // timetable does not contain.
    const StochasticLeg leg = simple_leg(1000, 600, 0);
    EXPECT_EQ(extend_one(100, leg, 0), 400);
    EXPECT_EQ(extend_one(400, leg, 0), 400);
    EXPECT_EQ(extend_one(401, leg, 0), 1000);
    EXPECT_EQ(extend_one(1000, leg, 0), 1000);
    EXPECT_EQ(extend_one(1001, leg, 0), 1600);
}

TEST(RiskProbe, MissingTheConnectionCostsAHeadwayNotADelay)
{
    // The whole reason a risk objective is interesting here: the penalty for
    // being late is not proportional to how late you were.
    const StochasticLeg leg = simple_leg(660, 1800, 600);
    const ArrivalLaw before({600, 720}); // two minutes apart
    const ArrivalLaw after = extend(before, leg);
    EXPECT_EQ(after.by_scenario()[0], 1260);
    EXPECT_EQ(after.by_scenario()[1], 3060);
    EXPECT_EQ(after.by_scenario()[1] - after.by_scenario()[0], 1800); // one headway, from 120 s
}

// ═══════════════════════════════════════════════════════════════════════════
// The orders
// ═══════════════════════════════════════════════════════════════════════════

TEST(RiskProbe, TheOrdersFormAChain)
{
    // Statewise ⇒ stochastic ⇒ every tail average ⇒ (mean, one tail average).
    // Each step orders more pairs, so each prunes harder — which is exactly why
    // safety is lost going up and frontier size is paid going down.
    Draws d(1234u);
    for (int trial = 0; trial < 500; ++trial)
    {
        const ArrivalLaw a(d.vec(5, 2000));
        const ArrivalLaw b(d.vec(5, 2000));

        if (dominates_statewise(a, b))
        {
            EXPECT_TRUE(dominates_stochastically(a, b));
        }
        if (dominates_stochastically(a, b))
        {
            EXPECT_TRUE(dominates_in_all_tail_averages(a, b));
        }
        if (dominates_in_all_tail_averages(a, b))
        {
            for (std::size_t m = 1; m <= a.size(); ++m)
                EXPECT_TRUE(dominates_scalar(a, b, m));
        }
    }
}

TEST(RiskProbe, StochasticDominanceSurvivesExtension)
{
    // The sufficiency lemma in its smallest form: a non-decreasing departure map
    // and an INDEPENDENT delay both preserve the order. The scenario space is a
    // product, so the independence is exact rather than approximate.
    Draws d(99u);
    int ordered_pairs = 0;
    for (int trial = 0; trial < 600; ++trial)
    {
        const std::vector<int64_t> pa = d.vec(4, 1800);
        const std::vector<int64_t> pb = d.vec(4, 1800);
        const std::vector<int64_t> delay = d.vec(3, 900);

        const ArrivalLaw a = product_prefix(pa, delay.size());
        const ArrivalLaw b = product_prefix(pb, delay.size());
        if (!dominates_stochastically(a, b))
            continue;
        ++ordered_pairs;

        StochasticLeg leg = simple_leg(d.below(900), 900, 300, product_delay(delay, pa.size()));
        EXPECT_TRUE(dominates_stochastically(extend(a, leg), extend(b, leg)));
    }
    EXPECT_GT(ordered_pairs, 20) << "the draw produced almost no ordered pairs to test";
}

TEST(RiskProbe, StochasticDominanceIsNecessary)
{
    // The converse, constructively. Every pair the order leaves unrelated can be
    // reversed by an ordinary timetable, so no weaker order is safe — pruning
    // has to be done in a strictly stronger order than the objective it serves.
    Draws d(2718u);
    int reversals = 0;
    for (int trial = 0; trial < 600; ++trial)
    {
        const ArrivalLaw a(d.vec(5, 3600));
        const ArrivalLaw b(d.vec(5, 3600));

        const auto leg = separating_leg(a, b);
        if (dominates_stochastically(a, b))
        {
            EXPECT_FALSE(leg.has_value());
            continue;
        }
        ASSERT_TRUE(leg.has_value());
        EXPECT_GT(extend(a, *leg).total(), extend(b, *leg).total());
        ++reversals;
    }
    EXPECT_GT(reversals, 100);
}

// ═══════════════════════════════════════════════════════════════════════════
// The witnesses
// ═══════════════════════════════════════════════════════════════════════════

TEST(RiskProbe, NoAccumulationRuleForMeanAndCvar)
{
    // Two journeys that agree EXACTLY on both numbers a scalar risk label would
    // carry, and disagree on both after one more leg. So no rule of any shape —
    // additive or otherwise — can take (mean, CVaR) to the extended
    // (mean, CVaR). That is strictly stronger than "CVaR is not additive", and
    // it is why the answer is not a cleverer accumulation formula.
    const LabelWitness w = scalar_label_witness();
    ASSERT_EQ(w.a.size(), w.b.size());

    EXPECT_EQ(w.a.total(), w.b.total());
    EXPECT_EQ(w.a.tail_total(w.tail_count), w.b.tail_total(w.tail_count));

    const ArrivalLaw ax = extend(w.a, w.leg);
    const ArrivalLaw bx = extend(w.b, w.leg);
    EXPECT_NE(ax.total(), bx.total());
    EXPECT_NE(ax.tail_total(w.tail_count), bx.tail_total(w.tail_count));
}

TEST(RiskProbe, ScalarPruningDiscardsThePrefixOfTheOptimum)
{
    const SearchWitness w = scalar_pruning_witness();
    const ArrivalLaw a = at_interchange(w, 0);
    const ArrivalLaw b = at_interchange(w, 1);

    // At the interchange, the rule discards b on both of its numbers at once.
    EXPECT_TRUE(dominates(a, b, w.unsafe_rule));
    EXPECT_LT(a.total(), b.total());
    EXPECT_LT(a.tail_total(w.tail_count), b.tail_total(w.tail_count));

    // And b was the better journey.
    const ArrivalLaw ax = extend(a, w.network.legs[2]);
    const ArrivalLaw bx = extend(b, w.network.legs[2]);
    EXPECT_LT(bx.total(), ax.total());
    EXPECT_LT(bx.tail_total(w.tail_count), ax.tail_total(w.tail_count));

    const int64_t truth = truth_at_destination(w);
    EXPECT_GT(optimum_at_destination(w, w.unsafe_rule), truth);
    EXPECT_EQ(optimum_at_destination(w, w.safe_rule), truth);
}

TEST(RiskProbe, TailAverageDominanceAtEveryLevelStillDiscardsTheOptimum)
{
    // The result the file exists for. The discarded journey is no better at ANY
    // confidence level — order by the objective, all of it, and the pruning is
    // still wrong, because a next-departure map is not convex.
    const SearchWitness w = tail_average_pruning_witness();
    const ArrivalLaw a = at_interchange(w, 0);
    const ArrivalLaw b = at_interchange(w, 1);

    ASSERT_TRUE(dominates_in_all_tail_averages(a, b));
    for (std::size_t m = 1; m <= a.size(); ++m)
        EXPECT_LE(a.tail_total(m), b.tail_total(m)) << "at m = " << m;

    // Not stochastically ordered, which is precisely the gap being exploited.
    EXPECT_FALSE(dominates_stochastically(a, b));
    EXPECT_FALSE(dominates_stochastically(b, a));

    const int64_t truth = truth_at_destination(w);
    EXPECT_GT(optimum_at_destination(w, w.unsafe_rule), truth);
    EXPECT_EQ(optimum_at_destination(w, w.safe_rule), truth);
}

TEST(RiskProbe, CorrelatedDelaysCostTheMarginalItsSufficiency)
{
    // The honest boundary of the sufficiency result. The discarded journey is
    // stochastically LATER, and it is the better journey, because it is late
    // exactly on the day its connection is late. No property of the arrival
    // distribution alone can see that.
    const SearchWitness w = correlated_delay_witness();
    const ArrivalLaw a = at_interchange(w, 0);
    const ArrivalLaw b = at_interchange(w, 1);

    ASSERT_TRUE(dominates_stochastically(a, b));
    EXPECT_FALSE(dominates_statewise(a, b));
    EXPECT_FALSE(dominates_statewise(b, a));

    const int64_t truth = truth_at_destination(w);
    EXPECT_GT(optimum_at_destination(w, w.unsafe_rule), truth);
    EXPECT_EQ(optimum_at_destination(w, w.safe_rule), truth);
}

TEST(RiskProbe, AdditiveProxyIsNotABound)
{
    // Coherence gives CVaR(sum) <= sum of CVaRs, and it is tempting to read that
    // as "per-leg risk is conservative". A journey's arrival is not a sum of
    // random costs: the delay is two minutes and the bill is a headway.
    const ProxyWitness w = additive_proxy_witness();
    const auto journeys = enumerate_journeys(w.network, 0, w.departure_time_s, 2);

    const ArrivalLaw *arrival = nullptr;
    for (const Journey &j : journeys)
        if (j.legs == w.journey)
            arrival = &j.law;
    ASSERT_NE(arrival, nullptr);

    const double truth = arrival->cvar(w.alpha);
    const double proxy = additive_cvar_proxy(w.network, w.journey, w.departure_time_s, w.alpha);

    EXPECT_LT(proxy, truth) << "the proxy is meant to under-state here, not bound";
    EXPECT_GT(truth - proxy, 1500.0) << "and to under-state by most of a headway";
}

// ═══════════════════════════════════════════════════════════════════════════
// Search against the oracle
// ═══════════════════════════════════════════════════════════════════════════

TEST(RiskProbe, UnprunedSearchAgreesWithEnumeration)
{
    // The control. A search that prunes nothing cannot have lost anything, so if
    // this ever disagrees with the enumeration the disagreement is in how the
    // two range over journeys, not in a pruning rule.
    RandomInstanceConfig cfg;
    cfg.num_nodes = 4;
    cfg.out_degree = 3;
    cfg.delay_support = 3;
    cfg.delay_spread_s = 1800;
    for (uint64_t seed = 1; seed <= 40; ++seed)
    {
        cfg.seed = seed;
        const RandomInstance inst = make_random_instance(cfg);
        const auto m = measure_rule(inst, PruningRule{DominanceOrder::None, 1}, 3);
        EXPECT_TRUE(m.destination_reached) << "seed " << seed;
        EXPECT_TRUE(m.exact) << "seed " << seed;
        EXPECT_EQ(m.labels_pruned, 0u);
    }
}

TEST(RiskProbe, StochasticPruningMatchesEnumerationOnIndependentInstances)
{
    // Sufficiency, end to end, against an algorithm that shares none of the
    // pruning logic. Every confidence level at once: one search, checked at
    // every tail count the scenario set admits.
    RandomInstanceConfig cfg;
    cfg.num_nodes = 4;
    cfg.out_degree = 3;
    cfg.delay_support = 3;
    cfg.delay_spread_s = 1800;

    for (uint64_t seed = 1; seed <= 60; ++seed)
    {
        cfg.seed = seed;
        const RandomInstance inst = make_random_instance(cfg);

        RiskSearchConfig scfg;
        scfg.rule = PruningRule{DominanceOrder::FirstOrderStochastic, 1};
        scfg.max_legs = 3;
        const auto pruned = risk_search(inst.network, inst.source, inst.departure_time_s, scfg);
        const auto all = enumerate_journeys(inst.network, inst.source, inst.departure_time_s, 3);

        for (std::size_t m = 1; m <= inst.network.num_scenarios; ++m)
        {
            const auto found = best_tail_total(pruned, inst.destination, m);
            const auto truth = best_tail_total(all, inst.destination, m);
            ASSERT_TRUE(truth.has_value()) << "seed " << seed;
            ASSERT_TRUE(found.has_value()) << "seed " << seed;
            EXPECT_EQ(*found, *truth) << "seed " << seed << " at m = " << m;
        }
        EXPECT_EQ(pruned.unsupported_prunings, 0u);
    }
}

TEST(RiskProbe, StatewisePruningMatchesEnumerationUnderDisruption)
{
    // Where stochastic dominance stops being safe, statewise dominance still is,
    // and this is the sweep that says so rather than the single witness.
    RandomInstanceConfig cfg;
    cfg.num_nodes = 4;
    cfg.out_degree = 3;
    cfg.delay_support = 3;
    cfg.delay_spread_s = 1800;
    cfg.disruption_s = 450; // half a headway: the phase that hurts most

    for (uint64_t seed = 1; seed <= 60; ++seed)
    {
        cfg.seed = seed;
        const RandomInstance inst = make_random_instance(cfg);
        const auto safe = measure_rule(inst, PruningRule{DominanceOrder::Statewise, 1}, 3);
        EXPECT_TRUE(safe.exact) << "seed " << seed;
        EXPECT_EQ(safe.regret_tail_total, 0) << "seed " << seed;
    }
}

TEST(RiskProbe, ADisruptionOfAWholeHeadwayIsNoDisruptionAtAll)
{
    // A shift of exactly one headway leaves every timetable where it was, so the
    // correlated instance is the independent one with its scenario set doubled.
    // It is the check that the failures at other magnitudes are caused by the
    // disruption moving the connection's PHASE, and not by the extra coordinate
    // or by anything about how the scenario space is laid out.
    RandomInstanceConfig indep;
    indep.num_nodes = 4;
    indep.out_degree = 3;
    indep.delay_support = 3;
    indep.delay_spread_s = 1800;

    RandomInstanceConfig shifted = indep;
    shifted.disruption_s = indep.headway_s;

    for (uint64_t seed = 1; seed <= 25; ++seed)
    {
        indep.seed = shifted.seed = seed;
        const RandomInstance a = make_random_instance(indep);
        const RandomInstance b = make_random_instance(shifted);
        ASSERT_EQ(b.network.num_scenarios, 2 * a.network.num_scenarios);

        for (const auto order : {DominanceOrder::FirstOrderStochastic, DominanceOrder::Scalar})
        {
            const auto ma = measure_rule(a, PruningRule{order, 1}, 3);
            const auto mb = measure_rule(b, PruningRule{order, 1}, 3);
            EXPECT_EQ(ma.exact, mb.exact) << "seed " << seed;
            EXPECT_EQ(ma.max_frontier, mb.max_frontier) << "seed " << seed;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Guards
// ═══════════════════════════════════════════════════════════════════════════

TEST(RiskProbe, MalformedInputsAreRejected)
{
    EXPECT_THROW(ArrivalLaw(std::vector<int64_t>{}), std::invalid_argument);
    EXPECT_THROW(ArrivalLaw::certain(0, 0), std::invalid_argument);

    const ArrivalLaw law({0, 1, 2});
    EXPECT_THROW((void)law.tail_total(4), std::out_of_range);
    EXPECT_THROW((void)law.cvar(-0.1), std::invalid_argument);
    EXPECT_THROW((void)dominates_scalar(law, law, 0), std::invalid_argument);
    EXPECT_THROW((void)dominates_stochastically(law, ArrivalLaw({0, 1})), std::invalid_argument);

    StochasticLeg bad = simple_leg(0, 0, 0); // zero headway
    EXPECT_THROW((void)extend(law, bad), std::invalid_argument);

    StochasticLeg mismatched = simple_leg(0, 600, 0, {1, 2});
    EXPECT_THROW((void)extend(law, mismatched), std::invalid_argument);

    StochasticNetwork net;
    net.num_nodes = 2;
    net.num_scenarios = 3;
    net.legs = {simple_leg(0, 600, 300)};
    EXPECT_THROW((void)risk_search(net, 7, 0, RiskSearchConfig{}), std::invalid_argument);

    RandomInstanceConfig cfg;
    cfg.num_nodes = 8;
    cfg.delay_support = 6;
    cfg.max_scenarios = 64; // 6^7 is far beyond it
    EXPECT_THROW((void)make_random_instance(cfg), std::invalid_argument);
}
