#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

/**
 * @file risk.hpp
 * @brief Does dominance pruning survive a risk objective? A theory probe.
 *
 * THE QUESTION, AND WHERE IT COMES FROM
 * ═════════════════════════════════════
 * `docs/write-up.tex` §4 and the README both say the same thing, correctly, and
 * then stop: the engine's second objective is deterministic and linear, transit
 * delay distributions are right-skewed, so variance is the wrong risk measure
 * and **Conditional Value at Risk is the coherent choice**. Both then record
 * the extension as future work.
 *
 * This file is that future work, reduced to the one question that has to be
 * answered before any of it is worth building:
 *
 *     The engine is fast because it throws labels away. A label is discarded
 *     when another label at the same node is at least as good on every
 *     objective, and that is safe because the cost of a journey is the sum of
 *     the costs of its legs — worse now stays worse later.
 *
 *     CVaR is not additive along a path. **So what, exactly, breaks — and what
 *     is the weakest thing you can put in a label that still makes pruning
 *     safe?**
 *
 * The answer turns out not to be about CVaR at all, and that is the finding.
 *
 * WHAT THIS IS NOT
 * ════════════════
 * It is not a risk-aware router. It does not read a feed, it does not touch one
 * line of the engine, and **it uses no real delay data, because it needs none.**
 * Every claim below is structural: it is about which orders survive composition
 * with a timetable, and a structural claim cannot be made true or false by the
 * numbers you feed it. Inventing a delay distribution and reporting the routes
 * that fall out of it would repeat the mistake `docs/crowd-model.md` is the
 * repair to: a second objective that was a formula rather than a measurement,
 * and was constant across the entire city for it.
 *
 * The instances here are therefore small, synthetic, and exactly reproducible.
 * The measured quantities are counterexample existence, agreement with an
 * exhaustive oracle, and frontier size. None of them depends on a delay
 * distribution being realistic; all of them depend on the arithmetic being
 * exact, which is why everything below is integer arithmetic over a finite
 * scenario set.
 *
 * THE MODEL
 * ═════════
 * A **scenario set** Ω = {0 … n-1}, equally likely, carries the entire joint law
 * of every delay in the network. Every quantity is a deterministic function of
 * ω, so a distribution is a vector of integers, a mean is a sum, a CVaR is a
 * partial sum, and no comparison anywhere in this file goes through a float.
 *
 * That choice is doing real work, and not only for exactness. Writing the joint
 * law down explicitly is what makes **dependence between delays expressible at
 * all** — and dependence is where the interesting half of the answer lives (see
 * §6). A model that stored one marginal distribution per leg could not state
 * the counterexample that matters most.
 *
 * A **leg** is a periodic service plus two scenario-dependent perturbations:
 *
 *     ready[ω]  = arrival[ω] + min_transfer
 *     dep[ω]    = the earliest  offset + k·headway + shift[ω]  that is ≥ ready[ω]
 *     arrive[ω] = dep[ω] + ride + delay[ω]
 *
 * `delay` is this vehicle's own lateness on this ride. `shift` moves every
 * departure of the connecting service together — one line running late all day,
 * which is what makes a *missed connection* correlate with the thing that
 * caused it. Service is periodic and unbounded, so every journey completes in
 * every scenario and no risk measure has to cope with an infinite atom.
 *
 * The map t ↦ arrive is, for each fixed ω, a **non-decreasing step function**.
 * That single fact is the source of everything that follows. It is also exactly
 * the FIFO property the deterministic engine relies on
 * (`tests/test_fifo_violation.cpp`, `docs/write-up.tex` §2) — stated per
 * scenario instead of once. Read this file as the stochastic continuation of
 * that argument, not as a separate topic.
 *
 * THE FIVE ORDERS, AND WHERE SAFETY IS LOST
 * ═════════════════════════════════════════
 * A pruning rule is an order on labels. Five are implemented, and they form a
 * chain: each orders strictly more pairs than the one below it, so each prunes
 * strictly harder.
 *
 *     None  ⊂  Statewise  ⊂  FirstOrderStochastic  ⊂  AllTailAverages  ⊂  Scalar
 *     ←──────────────── safer                          prunes harder ─────────→
 *
 *   **Scalar** — dominance on (mean arrival, CVaR of arrival) at one confidence
 *     level. This is the engine's own rule, transplanted: two numbers per
 *     label, both must be no worse. **Unsafe.** Not because CVaR is
 *     incoherent — the mean is as additive as a quantity gets, and it fails
 *     too — but because `arrive` is a step function of `t`, so E[f(T)] is not a
 *     function of E[T]. **No scalar summary of the prefix is a sufficient
 *     statistic for its own future.** @ref scalar_label_witness exhibits two
 *     prefixes agreeing exactly on both numbers whose extensions differ in
 *     both, which rules out an accumulation rule of *any* form, not merely the
 *     additive one.
 *
 *   **AllTailAverages** — dominance in CVaR at *every* confidence level at once
 *     (the increasing convex order). This is the natural repair once the first
 *     one fails: if the objective is CVaR, order by CVaR, all of it. Carrying
 *     every level is equivalent to carrying the whole distribution
 *     (@ref sorted_from_tail_totals), so nothing is being summarised away.
 *     **Still unsafe** — and this is the result worth the file. The increasing
 *     convex order is preserved by non-decreasing *convex* maps, and a
 *     next-departure map is non-decreasing but emphatically not convex.
 *     @ref tail_average_pruning_witness is a three-scenario instance where the
 *     CVaR-dominant prefix is the wrong one to keep.
 *
 *   **FirstOrderStochastic** — the arrival distribution of one label is no later
 *     than the other's at every quantile. **Safe when delays are independent
 *     across legs, and not merely sufficient but necessary:**
 *
 *       · sufficiency — a non-decreasing map preserves first-order stochastic
 *         dominance, adding an independent delay preserves it, and every CVaR is
 *         monotone under it. So the surviving label is at least as good after
 *         any continuation, at *every* confidence level simultaneously. The
 *         pruned search is exact for a whole family of objectives at once, not
 *         for one chosen α.
 *
 *       · necessity — @ref separating_leg constructs, for any pair this order
 *         does *not* relate, a legal leg that reverses them in the mean. There
 *         is no weaker safe order.
 *
 *     Stated together: **pruning must use an order strictly stronger than the
 *     objective it serves.** Ordering by CVaR is not enough to optimise CVaR.
 *
 *   **Statewise** — no later in *every scenario*. Safe unconditionally, because
 *     a per-scenario inequality survives a per-scenario non-decreasing map
 *     whatever the delays are correlated with. It is what remains when the
 *     marginal stops being a sufficient statistic — see §6.
 *
 *   **None** — no pruning. The control. Present so that "the pruned search
 *     found the optimum" can be checked against a search that cannot have
 *     missed it, independently of the enumeration oracle.
 *
 * WHAT THE PRICE IS
 * ═════════════════
 * Safety is bought with frontier size, and the frontier is the whole reason the
 * engine is fast: on the measured feeds it holds one label at 96–100% of nodes
 * (README, Measured Behaviour §1). Scalar dominance is a total-ish order on two
 * numbers; stochastic dominance is a partial order on n-vectors, and statewise
 * dominance is weaker still. `tools/risk_probe.cpp` measures what that costs, as
 * a function of delay spread, headway and branching, because "it is exact" and
 * "it is affordable" are different questions and only one of them has been
 * settled by a proof.
 *
 * THE LIMITATION THAT MATTERS
 * ═══════════════════════════
 * Sufficiency of stochastic dominance needs the leg's delay to be independent of
 * how the passenger arrived. Real delays are not: a late feeder and a late
 * connection are late together, and a dispatcher may hold the connection
 * *because* the feeder is late. Under that correlation the marginal arrival
 * distribution is no longer a sufficient statistic for the future at all, and
 * stochastic dominance stops being safe — @ref correlated_delay_witness is a
 * three-scenario instance where the journey with the stochastically *later*
 * arrival is the better journey, because it is late exactly when its connection
 * is. Statewise dominance still holds there, at a further cost in frontier size.
 *
 * That is not a footnote. It is the honest boundary of the result, and it is
 * where the interesting work would start.
 *
 * WHAT IS NOT CLAIMED
 * ═══════════════════
 * No delay distribution here is asserted to resemble any real service. No route
 * produced by anything in this file is a recommendation. The instances are
 * small enough for exhaustive enumeration by design, so nothing here says what
 * happens at city scale beyond the frontier-growth measurement, which is a
 * trend on synthetic networks and is reported as one.
 *
 * See `docs/risk.md` for the write-up and the measured numbers,
 * `tests/test_risk.cpp` for the witnesses as assertions.
 */

namespace namma_metro
{

    // ═══════════════════════════════════════════════════════════════════════════
    // § 1.  Arrival laws
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * @brief The arrival time at one node under every scenario.
     *
     * Equally likely scenarios, so the empirical distribution is just this
     * vector and every functional of it is a partial sum. The ascending-sorted
     * copy and its suffix sums are computed once at construction: every
     * dominance test and every CVaR reads one of them, and sorting inside the
     * comparison instead made the frontier sweep unusably slow.
     *
     * Times are integers (seconds). Nothing in this file divides before it has
     * to, so `total()` and `tail_total()` are exact and are what the search and
     * the tests compare. @ref mean and @ref cvar exist for reporting.
     */
    class ArrivalLaw
    {
    public:
        ArrivalLaw() = default;

        /// @throws std::invalid_argument if @p by_scenario is empty.
        explicit ArrivalLaw(std::vector<int64_t> by_scenario);

        /// The degenerate law: arrival @p t in every scenario. This is what a
        /// query's source label is — a departure time is a fact, not a guess.
        [[nodiscard]] static ArrivalLaw certain(std::size_t num_scenarios, int64_t t);

        [[nodiscard]] std::size_t size() const noexcept { return by_scenario_.size(); }
        [[nodiscard]] bool empty() const noexcept { return by_scenario_.empty(); }

        [[nodiscard]] const std::vector<int64_t> &by_scenario() const noexcept { return by_scenario_; }

        /// Ascending. This *is* the empirical quantile function, sampled at the
        /// n scenario boundaries.
        [[nodiscard]] const std::vector<int64_t> &sorted() const noexcept { return sorted_; }

        /// Sum over scenarios: n times the mean, exactly. Two laws over the same
        /// scenario count compare on this without a division.
        [[nodiscard]] int64_t total() const noexcept { return total_; }

        /// Sum of the @p m latest arrivals: m times the CVaR at confidence
        /// 1 - m/n, exactly.
        /// @throws std::out_of_range if m > size().
        [[nodiscard]] int64_t tail_total(std::size_t m) const;

        [[nodiscard]] double mean() const;

        /**
         * @brief CVaR at confidence @p alpha — the mean of the worst
         *        (1 - alpha) fraction of arrivals.
         *
         * The convention matches `docs/write-up.tex` §4 and the README: alpha is
         * the confidence level and the tail is what lies above it, so
         * alpha = 0.95 asks about the worst 5% of days and alpha = 0 is the
         * plain mean.
         *
         * Computed by the Rockafellar–Uryasev integral of the quantile function,
         * which is the definition that stays correct when (1-alpha)·n is not a
         * whole number: the boundary scenario enters with partial weight rather
         * than being rounded in or out. At the boundaries themselves this agrees
         * exactly with tail_total(m)/m — pinned by
         * `RiskProbe.CvarAtScenarioBoundariesIsTheTailAverage`, which is there
         * because rounding instead of interpolating is the classic way to get a
         * plausible and wrong CVaR.
         *
         * @param alpha in [0, 1). At alpha → 1 the tail closes on the worst
         *        single scenario, and that is what is returned.
         * @throws std::invalid_argument if alpha is outside [0, 1] or the law is
         *         empty.
         */
        [[nodiscard]] double cvar(double alpha) const;

        /// Value at Risk: the arrival that @p alpha of scenarios come in under.
        /// Reported beside CVaR because the gap between them is the shape of the
        /// tail, and a VaR-only view is exactly the blindness CVaR exists to fix.
        [[nodiscard]] double var(double alpha) const;

        [[nodiscard]] bool operator==(const ArrivalLaw &rhs) const noexcept
        {
            return by_scenario_ == rhs.by_scenario_;
        }
        [[nodiscard]] bool operator!=(const ArrivalLaw &rhs) const noexcept { return !(*this == rhs); }

    private:
        std::vector<int64_t> by_scenario_;
        std::vector<int64_t> sorted_;      ///< ascending
        std::vector<int64_t> tail_suffix_; ///< tail_suffix_[m] = sum of the m largest
        int64_t total_ = 0;
    };

    /**
     * @brief Recover the sorted arrival vector from its CVaR profile.
     *
     * @p tail_totals must be the sequence m ↦ tail_total(m) for m = 0 … n.
     * Successive differences are the sorted values read from the top down, so
     * **knowing CVaR at every level is knowing the distribution.**
     *
     * This is not a utility; it is the proposition that closes the argument.
     * It is why "carry more CVaR levels" cannot be the escape from the scalar
     * impossibility — carrying all of them is carrying the distribution, and at
     * that point the question is which *order* to compare distributions with,
     * which is what §3 is about.
     *
     * @throws std::invalid_argument if the sequence is not a valid profile
     *         (non-empty, starting at 0, with non-increasing differences).
     */
    [[nodiscard]] std::vector<int64_t> sorted_from_tail_totals(const std::vector<int64_t> &tail_totals);

    // ═══════════════════════════════════════════════════════════════════════════
    // § 2.  Legs and networks
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * @brief One timetabled service between two nodes, with delay.
     *
     * Periodic and unbounded in both directions: a departure sits at
     * `offset_s + k·headway_s` for every integer k, so a passenger who arrives
     * before the offset waits for it rather than falling off the end of the
     * timetable. Unboundedness is a modelling convenience with a purpose — it
     * keeps every journey finite in every scenario, so no risk measure has to
     * be defined on an infinite atom and no result below is an artefact of how
     * unreachability was encoded.
     *
     * @ref shift_s and @ref delay_s are both per-scenario and they are not the
     * same thing:
     *
     *   - @ref delay_s is this ride's own lateness. It moves the arrival and
     *     nothing else.
     *   - @ref shift_s moves every departure of this service together — the
     *     line is running late today. It changes *whether the connection is
     *     caught*, and because it is indexed by the same ω as the delays that
     *     made the passenger late, it is how correlated disruption is written
     *     down. A model without it can only express independent delays, and
     *     would therefore have made §6's counterexample unsayable.
     *
     * Either vector may be empty, meaning identically zero.
     */
    struct StochasticLeg
    {
        uint32_t from = 0;
        uint32_t to = 0;
        int64_t headway_s = 600;      ///< > 0.
        int64_t offset_s = 0;         ///< One scheduled departure sits here.
        int64_t ride_s = 0;           ///< >= 0. In-vehicle time, before delay.
        int64_t min_transfer_s = 0;   ///< >= 0. Charged before boarding.
        std::vector<int64_t> shift_s; ///< Per scenario; empty = all zero.
        std::vector<int64_t> delay_s; ///< Per scenario; empty = all zero.
    };

    /// Arrival at @p leg.to given the arrival law at @p leg.from. Elementwise
    /// over scenarios, integer-exact.
    /// @throws std::invalid_argument on a non-positive headway, a negative ride
    ///         or transfer time, or a shift/delay vector whose length is neither
    ///         zero nor the scenario count.
    [[nodiscard]] ArrivalLaw extend(const ArrivalLaw &at_from, const StochasticLeg &leg);

    /// The same map for one scenario, exposed because the witnesses are easier
    /// to read as arithmetic than as vectors.
    [[nodiscard]] int64_t extend_one(int64_t arrival, const StochasticLeg &leg, std::size_t scenario);

    /**
     * @brief A network is a scenario count and a list of legs.
     *
     * Deliberately not a CSRGraph. This carries a joint delay law and a
     * per-scenario timetable, which `graph.hpp` has no concept of, and the whole
     * point of a probe is that it can be wrong about the engine without
     * disturbing it.
     */
    struct StochasticNetwork
    {
        uint32_t num_nodes = 0;
        std::size_t num_scenarios = 0;
        std::vector<StochasticLeg> legs;

        /// legs_from()[u] lists indices into @ref legs. Built on demand; small
        /// enough that caching it would be premature.
        [[nodiscard]] std::vector<std::vector<uint32_t>> legs_from() const;
    };

    // ═══════════════════════════════════════════════════════════════════════════
    // § 3.  The five orders
    // ═══════════════════════════════════════════════════════════════════════════

    enum class DominanceOrder
    {
        None,                 ///< Nothing is pruned. The control.
        Statewise,            ///< No later in every scenario. Always safe.
        FirstOrderStochastic, ///< No later at every quantile. Safe under independence.
        AllTailAverages,      ///< No worse in CVaR at every level. NOT safe.
        Scalar,               ///< No worse in (mean, CVaR at one level). NOT safe.
    };

    /// Which order to prune with, and — for @ref DominanceOrder::Scalar — the
    /// confidence level, expressed as the number of worst-case scenarios
    /// averaged. Given as a count rather than an alpha so that the rule the
    /// search applies is the same integer comparison the tests assert.
    struct PruningRule
    {
        DominanceOrder order = DominanceOrder::FirstOrderStochastic;
        std::size_t tail_count = 1;
    };

    /// @return true if @p a is at least as good as @p b under @p rule, so that
    ///         @p b may be discarded. Reflexive: every law dominates itself
    ///         under every order except None, which is what keeps duplicate
    ///         labels out of the frontier.
    [[nodiscard]] bool dominates(const ArrivalLaw &a, const ArrivalLaw &b, const PruningRule &rule);

    /// The four orders individually, for tests that need to state which one they
    /// mean. `dominates_statewise` requires equal scenario counts; the others
    /// require equal sizes because comparing distributions sampled at different
    /// resolutions is a question this file does not answer.
    [[nodiscard]] bool dominates_statewise(const ArrivalLaw &a, const ArrivalLaw &b);
    [[nodiscard]] bool dominates_stochastically(const ArrivalLaw &a, const ArrivalLaw &b);
    [[nodiscard]] bool dominates_in_all_tail_averages(const ArrivalLaw &a, const ArrivalLaw &b);
    [[nodiscard]] bool dominates_scalar(const ArrivalLaw &a, const ArrivalLaw &b, std::size_t tail_count);

    /**
     * @brief A leg that reverses @p a and @p b, or nullopt if none can exist.
     *
     * The necessity half of the main result. If @p a does not stochastically
     * dominate @p b then some quantile of @p a is later, and the returned leg is
     * a service whose single relevant departure sits exactly at that crossing:
     * everything at or below it catches that departure, everything above waits a
     * full headway. Under it, @p a's mean arrival becomes strictly *worse* than
     * @p b's — so a pruning rule that had discarded @p b would have discarded the
     * better journey.
     *
     * The returned leg is an ordinary @ref StochasticLeg with no delay and no
     * shift. Nothing exotic is required to break a rule weaker than stochastic
     * dominance: a timetable is enough.
     *
     * @return nullopt exactly when `dominates_stochastically(a, b)`.
     */
    [[nodiscard]] std::optional<StochasticLeg> separating_leg(const ArrivalLaw &a, const ArrivalLaw &b);

    // ═══════════════════════════════════════════════════════════════════════════
    // § 4.  Search, and the oracle it is checked against
    // ═══════════════════════════════════════════════════════════════════════════

    inline constexpr uint32_t kNoLabel = 0xFFFFFFFFu;
    inline constexpr uint32_t kNoLeg = 0xFFFFFFFFu;

    /// One label: a journey's arrival law at a node, plus enough to name the
    /// journey afterwards. Labels are never erased from
    /// @ref RiskSearchResult::labels, only removed from a frontier, so a pruned
    /// label's index stays valid for any child it already produced and the
    /// witnesses can print the path that was thrown away.
    struct RiskLabel
    {
        uint32_t node = 0;
        uint32_t legs_used = 0;
        ArrivalLaw law;
        uint32_t parent = kNoLabel;
        uint32_t via_leg = kNoLeg;
        bool pruned = false; ///< Dominated after insertion, or dominated on arrival.
    };

    struct RiskSearchConfig
    {
        PruningRule rule;

        /// Journeys are bounded by leg count rather than by simple-path-ness,
        /// which is what makes the search and @ref enumerate_journeys range over
        /// exactly the same set of journeys. A comparison against an oracle that
        /// searched a different space would prove nothing.
        uint32_t max_legs = 4;
    };

    struct RiskSearchResult
    {
        std::vector<RiskLabel> labels;                 ///< Every label ever created, in creation order.
        std::vector<std::vector<uint32_t>> frontier;   ///< Per node: surviving label indices.
        std::size_t labels_created = 0;
        std::size_t labels_pruned = 0;

        /// Prunings the sufficiency proof does not cover: the label that did the
        /// discarding did not stochastically dominate the label it discarded.
        /// Zero for None, Statewise and FirstOrderStochastic by construction; for
        /// the two unsafe orders it counts how often the rule took a risk, which
        /// is a different and much more sensitive quantity than how often the
        /// risk cost anything. Under correlated delays even a supported pruning
        /// is not covered — see §6 — so this counter understates there, by design
        /// rather than by accident.
        std::size_t unsupported_prunings = 0;

        std::size_t max_frontier = 0;                  ///< Largest frontier at any node, at the end.
        double mean_frontier = 0.0;                    ///< Over nodes with at least one label.

        /// Leg indices of the journey ending at @p label, source first.
        [[nodiscard]] std::vector<uint32_t> journey_of(uint32_t label) const;
    };

    /**
     * @brief Label-correcting search over arrival laws, pruning by @p cfg.rule.
     *
     * The shape is the engine's: expand a label along every outgoing leg, offer
     * the result to the destination's frontier, keep it if nothing there
     * dominates it and evict whatever it dominates. Only the label and the order
     * have changed.
     *
     * Termination is by the leg bound, not by a monotone key. That is deliberate
     * and it is the point: under a partial order on distributions there is no
     * scalar to settle on, so a priority queue has nothing to order and the
     * "settled" notion the engine relies on does not exist. What a real
     * implementation would cost is a separate question this probe does not
     * answer.
     *
     * @throws std::invalid_argument if the source is out of range or the network
     *         is malformed.
     */
    [[nodiscard]] RiskSearchResult risk_search(const StochasticNetwork &net,
                                               uint32_t source,
                                               int64_t departure_time_s,
                                               const RiskSearchConfig &cfg);

    /// One enumerated journey and where it gets you.
    struct Journey
    {
        uint32_t end_node = 0;
        std::vector<uint32_t> legs;
        ArrivalLaw law;
    };

    /**
     * @brief Every journey of at most @p max_legs legs from @p source. The oracle.
     *
     * Exhaustive over leg sequences, including ones that revisit a node, so that
     * it covers exactly what @ref risk_search covers. Exponential in
     * @p max_legs by construction — that is affordable only because the
     * instances are deliberately tiny, and it is the reason they are.
     */
    [[nodiscard]] std::vector<Journey> enumerate_journeys(const StochasticNetwork &net,
                                                          uint32_t source,
                                                          int64_t departure_time_s,
                                                          uint32_t max_legs);

    /// The best (smallest) tail total at @p node among a search's surviving
    /// labels, and among an enumeration's journeys. Integer, so "the pruned
    /// search found the optimum" is an exact comparison and not a tolerance.
    [[nodiscard]] std::optional<int64_t> best_tail_total(const RiskSearchResult &r,
                                                          uint32_t node,
                                                          std::size_t tail_count);
    [[nodiscard]] std::optional<int64_t> best_tail_total(const std::vector<Journey> &js,
                                                          uint32_t node,
                                                          std::size_t tail_count);

    // ═══════════════════════════════════════════════════════════════════════════
    // § 5.  The additive proxy, and why it is not a bound
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * @brief Schedule the journey, then add each leg's own CVaR of delay.
     *
     * This is the extension `docs/write-up.tex` proposes verbatim — replace the
     * linear crowd term with an empirical CVaR per edge — and it is the obvious
     * thing to reach for, because a sum is additive and an additive objective is
     * prunable by the machinery that already exists.
     *
     * It is also not the journey's CVaR, and the usual reassurance does not
     * apply. Coherence gives CVaR(ΣXᵢ) ≤ Σ CVaR(Xᵢ), so summing per-leg risk is
     * a conservative bound **on a sum of random costs**. A journey's arrival is
     * not a sum of random costs: a delay that causes a missed connection costs a
     * whole headway, not the delay. So the proxy can sit far *below* the true
     * CVaR as easily as above it, and its error is largest exactly where the
     * risk is — pinned by `RiskProbe.AdditiveProxyIsNotABound`.
     *
     * Kept, measured and reported rather than dismissed, because it is what an
     * implementer would build first and the size of its regret is the argument
     * against it.
     *
     * @param legs A journey, in order, as indices into `net.legs`.
     * @throws std::invalid_argument if the journey is not connected or a leg
     *         index is out of range.
     */
    [[nodiscard]] double additive_cvar_proxy(const StochasticNetwork &net,
                                             const std::vector<uint32_t> &legs,
                                             int64_t departure_time_s,
                                             double alpha);

    // ═══════════════════════════════════════════════════════════════════════════
    // § 6.  Witnesses
    // ═══════════════════════════════════════════════════════════════════════════
    //
    // Each counterexample is constructed here, once, and used by both
    // tests/test_risk.cpp and tools/risk_probe.cpp. They are not in the test
    // file because a witness that exists twice is a witness that can drift into
    // meaning two different things — the same reason tools/isochrone.cpp refuses
    // to carry its own copy of the accessibility loop, in a place where the
    // divergence would be harder to see.

    /// Two prefixes, one leg, one confidence level.
    struct LabelWitness
    {
        ArrivalLaw a;
        ArrivalLaw b;
        StochasticLeg leg;
        std::size_t tail_count = 1;
    };

    /**
     * @brief No accumulation rule exists for a scalar risk label.
     *
     * `a` and `b` agree **exactly** on mean arrival and on CVaR at
     * `tail_count`. After `leg` they differ in both. So there is no function
     * whatever — additive or otherwise — taking (mean, CVaR, the leg) to the
     * extended (mean, CVaR): the two numbers are not a sufficient statistic for
     * their own future.
     *
     * This is strictly stronger than the familiar observation that CVaR is not
     * additive, and it is why the answer to "does dominance pruning survive
     * CVaR" cannot be repaired by finding a cleverer accumulation formula.
     */
    [[nodiscard]] LabelWitness scalar_label_witness();

    /// A three-node instance: two journeys to an interchange, one connection out
    /// of it, and a confidence level at which the pruning rule under test
    /// discards the prefix of the optimum.
    struct SearchWitness
    {
        StochasticNetwork network;
        uint32_t source = 0;
        uint32_t interchange = 1;
        uint32_t destination = 2;
        int64_t departure_time_s = 0;
        std::size_t tail_count = 1;
        PruningRule unsafe_rule;   ///< Loses the optimum on this instance.
        PruningRule safe_rule;     ///< Finds it.
    };

    /// Scalar dominance discards the prefix of the optimum. The discarded
    /// journey is worse in mean *and* in CVaR at the interchange, and better in
    /// both at the destination.
    [[nodiscard]] SearchWitness scalar_pruning_witness();

    /// Dominance in CVaR at every confidence level discards the prefix of the
    /// optimum. The headline: ordering by the objective, at every level of the
    /// objective, is still not a safe way to prune for that objective.
    [[nodiscard]] SearchWitness tail_average_pruning_witness();

    /// With correlated delays, stochastic dominance discards the prefix of the
    /// optimum: the journey that is stochastically later is the better journey,
    /// because it is late exactly when its connection is. Statewise dominance
    /// keeps it.
    [[nodiscard]] SearchWitness correlated_delay_witness();

    /// A journey whose true CVaR the additive proxy under-states by most of a
    /// headway, because the proxy charges the delay and the passenger pays for
    /// the connection.
    struct ProxyWitness
    {
        StochasticNetwork network;
        std::vector<uint32_t> journey;
        int64_t departure_time_s = 0;
        double alpha = 0.5;
    };
    [[nodiscard]] ProxyWitness additive_proxy_witness();

    // ═══════════════════════════════════════════════════════════════════════════
    // § 7.  Random instances and measurement
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * @brief Parameters of a randomly generated probe instance.
     *
     * The scenario set is the **product** of the per-leg delay supports, so with
     * `independent_delays` every leg's delay is a function of its own coordinate
     * alone and the legs are exactly, not approximately, independent. That
     * exactness is what makes it legitimate to check the sufficiency proof
     * against these instances. It also costs: |Ω| = `delay_support` ^ |legs|, so
     * the generator caps the product and reports what it built.
     *
     * A non-zero `disruption_s` adds one extra coordinate, shared by every leg's
     * `shift_s`: on half of all scenarios the whole network runs late by that
     * many seconds. It is the mildest correlated disruption there is — every
     * journey is simply translated — and §6 says why even that is enough to cost
     * the marginal its sufficiency.
     */
    struct RandomInstanceConfig
    {
        uint32_t num_nodes = 5;
        uint32_t out_degree = 2;        ///< Legs out of each non-terminal node.
        uint32_t delay_support = 3;     ///< Delay atoms per leg.
        int64_t delay_spread_s = 600;   ///< Delays are drawn from [0, this].
        int64_t headway_s = 900;
        int64_t ride_s = 300;
        int64_t max_scenarios = 4096;   ///< Cap on the product scenario space.

        /// Seconds by which the whole network runs late on half of all
        /// scenarios. **0 means delays are independent across legs**, which is
        /// the hypothesis the stochastic-dominance result needs; anything else
        /// puts the instance in the regime where it does not hold.
        int64_t disruption_s = 0;

        uint64_t seed = 1;
    };

    struct RandomInstance
    {
        StochasticNetwork network;
        uint32_t source = 0;
        uint32_t destination = 0;
        int64_t departure_time_s = 0;
    };

    /**
     * @brief Build one instance. Deterministic in `cfg.seed` and in nothing else.
     *
     * The generator uses its own splitmix64 rather than `<random>`: the standard
     * distributions are not specified to produce the same values across library
     * implementations, so a figure in `docs/risk.md` generated with libstdc++
     * could not be reproduced on another toolchain. A seeded sweep that is not
     * reproducible elsewhere is not evidence.
     */
    [[nodiscard]] RandomInstance make_random_instance(const RandomInstanceConfig &cfg);

    /// What one pruning rule did on one instance, measured against exhaustive
    /// enumeration over the same journey set.
    struct RuleMeasurement
    {
        std::size_t max_frontier = 0;
        double mean_frontier = 0.0;
        std::size_t labels_created = 0;
        std::size_t labels_pruned = 0;
        std::size_t unsupported_prunings = 0;
        bool destination_reached = false;
        bool exact = false;             ///< Matched the enumeration optimum exactly.
        int64_t regret_tail_total = 0;  ///< Search optimum minus true optimum; 0 iff exact.
    };

    /// @throws std::invalid_argument if the instance is malformed.
    [[nodiscard]] RuleMeasurement measure_rule(const RandomInstance &instance,
                                               const PruningRule &rule,
                                               uint32_t max_legs);

} // namespace namma_metro
