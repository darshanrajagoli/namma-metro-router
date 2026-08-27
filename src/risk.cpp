#include "risk.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <string>

namespace namma_metro
{

    namespace
    {

        /// Ceiling division for a possibly negative numerator and a positive
        /// denominator.
        ///
        /// C++ integer division truncates toward zero, so `(-650) / 1800` is 0
        /// and `(a + b - 1) / b` — the usual ceiling idiom — silently returns
        /// the wrong answer for every negative numerator. That case is not
        /// exotic here: it is what a passenger who arrives BEFORE the offset
        /// looks like, which happens on the first leg of nearly every journey.
        /// Getting it wrong makes a departure appear one whole headway early,
        /// which is a plausible-looking timetable and a fictional one.
        [[nodiscard]] int64_t ceil_div(int64_t a, int64_t b)
        {
            if (a >= 0)
                return (a + b - 1) / b;
            return -((-a) / b);
        }

        void require(bool ok, const char *what)
        {
            if (!ok)
                throw std::invalid_argument(what);
        }

        void check_same_size(const ArrivalLaw &a, const ArrivalLaw &b)
        {
            require(!a.empty() && a.size() == b.size(),
                    "risk: laws compared across different scenario sets");
        }

        /// The leg's per-scenario perturbations, with the empty-vector shorthand
        /// resolved. Both vectors are allowed to be empty meaning "identically
        /// zero", because most legs in most witnesses have no shift and writing
        /// a vector of zeros in every construction obscured which leg was the
        /// one carrying the correlation.
        [[nodiscard]] int64_t at_or_zero(const std::vector<int64_t> &v, std::size_t i)
        {
            return v.empty() ? 0 : v[i];
        }

        void validate_leg(const StochasticLeg &leg, std::size_t n)
        {
            require(leg.headway_s > 0, "risk: leg headway must be positive");
            require(leg.ride_s >= 0, "risk: leg ride time must be non-negative");
            require(leg.min_transfer_s >= 0, "risk: leg transfer time must be non-negative");
            require(leg.shift_s.empty() || leg.shift_s.size() == n,
                    "risk: leg shift vector does not match the scenario count");
            require(leg.delay_s.empty() || leg.delay_s.size() == n,
                    "risk: leg delay vector does not match the scenario count");
        }

        void validate_network(const StochasticNetwork &net)
        {
            require(net.num_nodes > 0, "risk: network has no nodes");
            require(net.num_scenarios > 0, "risk: network has no scenarios");
            for (const auto &leg : net.legs)
            {
                require(leg.from < net.num_nodes && leg.to < net.num_nodes,
                        "risk: leg endpoint out of range");
                validate_leg(leg, net.num_scenarios);
            }
        }

    } // namespace

    // ═══════════════════════════════════════════════════════════════════════════
    // § 1.  ArrivalLaw
    // ═══════════════════════════════════════════════════════════════════════════

    ArrivalLaw::ArrivalLaw(std::vector<int64_t> by_scenario)
        : by_scenario_(std::move(by_scenario))
    {
        require(!by_scenario_.empty(), "risk: an arrival law needs at least one scenario");

        sorted_ = by_scenario_;
        std::sort(sorted_.begin(), sorted_.end());

        // tail_suffix_[m] = sum of the m largest. Built once, read by every
        // dominance test and every CVaR, so the search never sorts in a loop.
        tail_suffix_.assign(sorted_.size() + 1, 0);
        for (std::size_t m = 1; m <= sorted_.size(); ++m)
            tail_suffix_[m] = tail_suffix_[m - 1] + sorted_[sorted_.size() - m];

        total_ = tail_suffix_.back();
    }

    ArrivalLaw ArrivalLaw::certain(std::size_t num_scenarios, int64_t t)
    {
        require(num_scenarios > 0, "risk: an arrival law needs at least one scenario");
        return ArrivalLaw(std::vector<int64_t>(num_scenarios, t));
    }

    int64_t ArrivalLaw::tail_total(std::size_t m) const
    {
        if (m >= tail_suffix_.size())
            throw std::out_of_range("risk: tail count exceeds the scenario count");
        return tail_suffix_[m];
    }

    double ArrivalLaw::mean() const
    {
        require(!empty(), "risk: mean of an empty law");
        return static_cast<double>(total_) / static_cast<double>(size());
    }

    double ArrivalLaw::cvar(double alpha) const
    {
        require(!empty(), "risk: CVaR of an empty law");
        require(alpha >= 0.0 && alpha <= 1.0, "risk: CVaR confidence must lie in [0, 1]");

        const double n = static_cast<double>(size());

        // beta is the tail measured in scenarios. The whole point of computing
        // it this way rather than rounding to a scenario index is that a
        // confidence level almost never lands on a scenario boundary, and
        // rounding the boundary IN or OUT moves CVaR by a whole atom — which on
        // these instances is the difference between catching a connection and
        // missing it.
        const double beta = (1.0 - alpha) * n;

        // Degenerate tail: the confidence level has closed on the single worst
        // scenario. That is the essential supremum, and it is the honest limit
        // rather than a division by zero.
        constexpr double kEps = 1e-9;
        if (beta <= kEps)
            return static_cast<double>(sorted_.back());

        std::size_t k = static_cast<std::size_t>(std::floor(beta + kEps));
        if (k > size())
            k = size();
        double remainder = beta - static_cast<double>(k);
        if (remainder < kEps || k == size())
            remainder = 0.0;

        double sum = static_cast<double>(tail_total(k));
        if (remainder > 0.0)
            sum += remainder * static_cast<double>(sorted_[size() - k - 1]);

        return sum / beta;
    }

    double ArrivalLaw::var(double alpha) const
    {
        require(!empty(), "risk: VaR of an empty law");
        require(alpha >= 0.0 && alpha <= 1.0, "risk: VaR confidence must lie in [0, 1]");

        if (alpha <= 0.0)
            return static_cast<double>(sorted_.front());

        const double n = static_cast<double>(size());
        std::size_t idx = static_cast<std::size_t>(std::ceil(alpha * n - 1e-9));
        if (idx == 0)
            idx = 1;
        if (idx > size())
            idx = size();
        return static_cast<double>(sorted_[idx - 1]);
    }

    std::vector<int64_t> sorted_from_tail_totals(const std::vector<int64_t> &tail_totals)
    {
        require(tail_totals.size() >= 2, "risk: a tail-total profile needs at least one scenario");
        require(tail_totals.front() == 0, "risk: a tail-total profile must start at zero");

        const std::size_t n = tail_totals.size() - 1;
        std::vector<int64_t> descending(n);
        for (std::size_t m = 1; m <= n; ++m)
            descending[m - 1] = tail_totals[m] - tail_totals[m - 1];

        // The m-th difference is the m-th largest value, so the differences must
        // be non-increasing. If they are not, the input was not the tail profile
        // of any distribution and reconstructing something from it anyway would
        // manufacture a law nobody has.
        for (std::size_t i = 1; i < n; ++i)
            require(descending[i] <= descending[i - 1],
                    "risk: tail-total differences are not non-increasing");

        std::vector<int64_t> ascending(descending.rbegin(), descending.rend());
        return ascending;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // § 2.  Legs and networks
    // ═══════════════════════════════════════════════════════════════════════════

    int64_t extend_one(int64_t arrival, const StochasticLeg &leg, std::size_t scenario)
    {
        const int64_t shift = at_or_zero(leg.shift_s, scenario);
        const int64_t delay = at_or_zero(leg.delay_s, scenario);

        const int64_t ready = arrival + leg.min_transfer_s;
        const int64_t k = ceil_div(ready - shift - leg.offset_s, leg.headway_s);
        const int64_t departure = leg.offset_s + k * leg.headway_s + shift;

        return departure + leg.ride_s + delay;
    }

    ArrivalLaw extend(const ArrivalLaw &at_from, const StochasticLeg &leg)
    {
        require(!at_from.empty(), "risk: cannot extend an empty law");
        validate_leg(leg, at_from.size());

        std::vector<int64_t> out(at_from.size());
        for (std::size_t w = 0; w < at_from.size(); ++w)
            out[w] = extend_one(at_from.by_scenario()[w], leg, w);
        return ArrivalLaw(std::move(out));
    }

    std::vector<std::vector<uint32_t>> StochasticNetwork::legs_from() const
    {
        std::vector<std::vector<uint32_t>> out(num_nodes);
        for (uint32_t i = 0; i < legs.size(); ++i)
        {
            if (legs[i].from >= num_nodes)
                throw std::invalid_argument("risk: leg endpoint out of range");
            out[legs[i].from].push_back(i);
        }
        return out;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // § 3.  The five orders
    // ═══════════════════════════════════════════════════════════════════════════

    bool dominates_statewise(const ArrivalLaw &a, const ArrivalLaw &b)
    {
        check_same_size(a, b);
        for (std::size_t w = 0; w < a.size(); ++w)
            if (a.by_scenario()[w] > b.by_scenario()[w])
                return false;
        return true;
    }

    bool dominates_stochastically(const ArrivalLaw &a, const ArrivalLaw &b)
    {
        check_same_size(a, b);
        // Comparing the sorted vectors IS comparing the quantile functions: with
        // equally likely scenarios the i-th smallest arrival is the quantile at
        // (i+1)/n. No cumulative distribution has to be built.
        for (std::size_t i = 0; i < a.size(); ++i)
            if (a.sorted()[i] > b.sorted()[i])
                return false;
        return true;
    }

    bool dominates_in_all_tail_averages(const ArrivalLaw &a, const ArrivalLaw &b)
    {
        check_same_size(a, b);
        // m = size() is the mean, so this subsumes the mean comparison and there
        // is no separate one.
        for (std::size_t m = 1; m <= a.size(); ++m)
            if (a.tail_total(m) > b.tail_total(m))
                return false;
        return true;
    }

    bool dominates_scalar(const ArrivalLaw &a, const ArrivalLaw &b, std::size_t tail_count)
    {
        check_same_size(a, b);
        require(tail_count >= 1 && tail_count <= a.size(),
                "risk: tail count out of range for these laws");
        return a.total() <= b.total() && a.tail_total(tail_count) <= b.tail_total(tail_count);
    }

    bool dominates(const ArrivalLaw &a, const ArrivalLaw &b, const PruningRule &rule)
    {
        switch (rule.order)
        {
        case DominanceOrder::None:
            return false;
        case DominanceOrder::Statewise:
            return dominates_statewise(a, b);
        case DominanceOrder::FirstOrderStochastic:
            return dominates_stochastically(a, b);
        case DominanceOrder::AllTailAverages:
            return dominates_in_all_tail_averages(a, b);
        case DominanceOrder::Scalar:
            return dominates_scalar(a, b, rule.tail_count);
        }
        return false;
    }

    std::optional<StochasticLeg> separating_leg(const ArrivalLaw &a, const ArrivalLaw &b)
    {
        check_same_size(a, b);
        if (dominates_stochastically(a, b))
            return std::nullopt;

        // a is later than b at some quantile. Take the first such crossing and
        // put a departure exactly on b's value there: everything at or below it
        // boards, everything above waits a full headway.
        std::size_t crossing = 0;
        while (crossing < a.size() && a.sorted()[crossing] <= b.sorted()[crossing])
            ++crossing;
        const int64_t theta = b.sorted()[crossing];

        // The headway has to exceed the whole spread of both laws on both sides
        // of theta. Service is periodic and unbounded, so a headway that is
        // merely "large above theta" would let an early arrival board a
        // departure a period BELOW theta, which flattens the step this argument
        // depends on and quietly weakens the witness into a non-witness.
        const int64_t lo = std::min(a.sorted().front(), b.sorted().front());
        const int64_t hi = std::max(a.sorted().back(), b.sorted().back());
        const int64_t headway = std::max<int64_t>(std::max(hi - theta, theta - lo) + 1, 1);

        StochasticLeg leg;
        leg.headway_s = headway;
        leg.offset_s = theta;
        leg.ride_s = 0;
        leg.min_transfer_s = 0;
        return leg;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // § 4.  Search and oracle
    // ═══════════════════════════════════════════════════════════════════════════

    std::vector<uint32_t> RiskSearchResult::journey_of(uint32_t label) const
    {
        std::vector<uint32_t> legs;
        while (label != kNoLabel)
        {
            const RiskLabel &l = labels.at(label);
            if (l.via_leg == kNoLeg)
                break;
            legs.push_back(l.via_leg);
            label = l.parent;
        }
        std::reverse(legs.begin(), legs.end());
        return legs;
    }

    RiskSearchResult risk_search(const StochasticNetwork &net,
                                 uint32_t source,
                                 int64_t departure_time_s,
                                 const RiskSearchConfig &cfg)
    {
        validate_network(net);
        require(source < net.num_nodes, "risk: source node out of range");
        if (cfg.rule.order == DominanceOrder::Scalar)
            require(cfg.rule.tail_count >= 1 && cfg.rule.tail_count <= net.num_scenarios,
                    "risk: tail count out of range for this scenario set");

        const auto adjacency = net.legs_from();

        RiskSearchResult r;
        r.frontier.assign(net.num_nodes, {});

        RiskLabel start;
        start.node = source;
        start.legs_used = 0;
        start.law = ArrivalLaw::certain(net.num_scenarios, departure_time_s);
        r.labels.push_back(std::move(start));
        r.labels_created = 1;
        r.frontier[source].push_back(0);

        std::vector<uint32_t> work{0};

        while (!work.empty())
        {
            const uint32_t idx = work.back();
            work.pop_back();
            if (r.labels[idx].pruned)
                continue; // evicted after it was queued; its children are already accounted for
            if (r.labels[idx].legs_used >= cfg.max_legs)
                continue;

            const uint32_t node = r.labels[idx].node;
            const uint32_t legs_used = r.labels[idx].legs_used;

            for (const uint32_t leg_index : adjacency[node])
            {
                const StochasticLeg &leg = net.legs[leg_index];

                RiskLabel candidate;
                candidate.node = leg.to;
                candidate.legs_used = legs_used + 1;
                candidate.law = extend(r.labels[idx].law, leg);
                candidate.parent = idx;
                candidate.via_leg = leg_index;

                const uint32_t new_index = static_cast<uint32_t>(r.labels.size());
                r.labels.push_back(std::move(candidate));
                ++r.labels_created;

                // Offered to the destination's frontier exactly as the engine
                // offers a Label to a ParetoLabelSet: rejected if anything there
                // already dominates it, otherwise inserted and everything it
                // dominates is evicted.
                auto &frontier = r.frontier[leg.to];
                const ArrivalLaw &law = r.labels[new_index].law;

                bool rejected = false;
                for (const uint32_t held : frontier)
                {
                    if (dominates(r.labels[held].law, law, cfg.rule))
                    {
                        rejected = true;
                        if (!dominates_stochastically(r.labels[held].law, law))
                            ++r.unsupported_prunings;
                        break;
                    }
                }
                if (rejected)
                {
                    r.labels[new_index].pruned = true;
                    ++r.labels_pruned;
                    continue;
                }

                std::vector<uint32_t> kept;
                kept.reserve(frontier.size() + 1);
                for (const uint32_t held : frontier)
                {
                    if (dominates(law, r.labels[held].law, cfg.rule))
                    {
                        r.labels[held].pruned = true;
                        ++r.labels_pruned;
                        if (!dominates_stochastically(law, r.labels[held].law))
                            ++r.unsupported_prunings;
                    }
                    else
                    {
                        kept.push_back(held);
                    }
                }
                kept.push_back(new_index);
                frontier.swap(kept);

                work.push_back(new_index);
            }
        }

        std::size_t occupied = 0;
        std::size_t total_labels = 0;
        for (const auto &f : r.frontier)
        {
            r.max_frontier = std::max(r.max_frontier, f.size());
            if (!f.empty())
            {
                ++occupied;
                total_labels += f.size();
            }
        }
        r.mean_frontier = occupied == 0
                              ? 0.0
                              : static_cast<double>(total_labels) / static_cast<double>(occupied);
        return r;
    }

    std::vector<Journey> enumerate_journeys(const StochasticNetwork &net,
                                            uint32_t source,
                                            int64_t departure_time_s,
                                            uint32_t max_legs)
    {
        validate_network(net);
        require(source < net.num_nodes, "risk: source node out of range");

        const auto adjacency = net.legs_from();

        std::vector<Journey> out;
        Journey start;
        start.end_node = source;
        start.law = ArrivalLaw::certain(net.num_scenarios, departure_time_s);
        out.push_back(start);

        // Breadth-first over leg sequences rather than a recursion, so the depth
        // bound is a loop bound and a deep max_legs cannot overflow a stack in a
        // sweep that is otherwise entirely allocation-bound.
        std::vector<Journey> layer{out.front()};
        for (uint32_t depth = 0; depth < max_legs; ++depth)
        {
            std::vector<Journey> next;
            for (const Journey &j : layer)
            {
                for (const uint32_t leg_index : adjacency[j.end_node])
                {
                    Journey child;
                    child.legs = j.legs;
                    child.legs.push_back(leg_index);
                    child.end_node = net.legs[leg_index].to;
                    child.law = extend(j.law, net.legs[leg_index]);
                    next.push_back(std::move(child));
                }
            }
            if (next.empty())
                break;
            out.insert(out.end(), next.begin(), next.end());
            layer.swap(next);
        }
        return out;
    }

    std::optional<int64_t> best_tail_total(const RiskSearchResult &r,
                                           uint32_t node,
                                           std::size_t tail_count)
    {
        if (node >= r.frontier.size() || r.frontier[node].empty())
            return std::nullopt;
        std::optional<int64_t> best;
        for (const uint32_t idx : r.frontier[node])
        {
            const int64_t v = r.labels[idx].law.tail_total(tail_count);
            if (!best || v < *best)
                best = v;
        }
        return best;
    }

    std::optional<int64_t> best_tail_total(const std::vector<Journey> &js,
                                           uint32_t node,
                                           std::size_t tail_count)
    {
        std::optional<int64_t> best;
        for (const Journey &j : js)
        {
            if (j.end_node != node)
                continue;
            const int64_t v = j.law.tail_total(tail_count);
            if (!best || v < *best)
                best = v;
        }
        return best;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // § 5.  The additive proxy
    // ═══════════════════════════════════════════════════════════════════════════

    double additive_cvar_proxy(const StochasticNetwork &net,
                               const std::vector<uint32_t> &legs,
                               int64_t departure_time_s,
                               double alpha)
    {
        validate_network(net);

        int64_t scheduled = departure_time_s;
        uint32_t at = legs.empty() ? 0u : net.legs.at(legs.front()).from;
        double risk_sum = 0.0;

        for (const uint32_t leg_index : legs)
        {
            require(leg_index < net.legs.size(), "risk: leg index out of range");
            const StochasticLeg &leg = net.legs[leg_index];
            require(leg.from == at, "risk: journey is not connected");

            // The schedule is the network with every perturbation set to zero:
            // what the timetable promises. Riding it is what the proxy assumes.
            StochasticLeg on_time = leg;
            on_time.shift_s.clear();
            on_time.delay_s.clear();
            scheduled = extend_one(scheduled, on_time, 0);

            // This leg's own lateness against that schedule, as a distribution.
            std::vector<int64_t> lateness(net.num_scenarios);
            for (std::size_t w = 0; w < net.num_scenarios; ++w)
                lateness[w] = at_or_zero(leg.shift_s, w) + at_or_zero(leg.delay_s, w);
            risk_sum += ArrivalLaw(std::move(lateness)).cvar(alpha);

            at = leg.to;
        }

        return static_cast<double>(scheduled) + risk_sum;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // § 6.  Witnesses
    // ═══════════════════════════════════════════════════════════════════════════

    namespace
    {
        /// A leg out of the source that simply takes `ride` seconds plus its own
        /// per-scenario delay. Offset 0 with the query departing at 0 means the
        /// service is waiting, so the arrival law is exactly ride + delay and the
        /// witness can be read as arithmetic.
        [[nodiscard]] StochasticLeg approach(uint32_t from, uint32_t to, int64_t ride,
                                             std::vector<int64_t> delay)
        {
            StochasticLeg leg;
            leg.from = from;
            leg.to = to;
            leg.headway_s = 1800;
            leg.offset_s = 0;
            leg.ride_s = ride;
            leg.delay_s = std::move(delay);
            return leg;
        }

        [[nodiscard]] StochasticLeg connection(uint32_t from, uint32_t to, int64_t offset,
                                               int64_t headway, int64_t ride,
                                               std::vector<int64_t> shift = {})
        {
            StochasticLeg leg;
            leg.from = from;
            leg.to = to;
            leg.headway_s = headway;
            leg.offset_s = offset;
            leg.ride_s = ride;
            leg.shift_s = std::move(shift);
            return leg;
        }
    } // namespace

    LabelWitness scalar_label_witness()
    {
        LabelWitness w;
        // Four equally likely days. Both journeys arrive at the interchange with
        // the same mean and the same CVaR — they are indistinguishable to a
        // label that carries those two numbers.
        w.a = ArrivalLaw({5400, 5400, 6060, 6060});
        w.b = ArrivalLaw({5400, 5400, 5940, 6180});
        w.tail_count = 2;
        // The connection leaves at 6000 every half hour. It is the only thing
        // that happens next, and it separates them.
        w.leg = connection(1, 2, 6000, 1800, 0);
        return w;
    }

    SearchWitness scalar_pruning_witness()
    {
        SearchWitness w;
        w.network.num_nodes = 3;
        w.network.num_scenarios = 4;
        w.network.legs = {
            // A: usually punctual, occasionally very late.
            approach(0, 1, 5000, {0, 0, 0, 1500}),
            // B: never as late, but late more often. Worse on the mean AND on
            // CVaR at the interchange, so a scalar rule discards it there.
            approach(0, 1, 5000, {0, 0, 950, 1000}),
            // The connection at 6000 is what makes B the better journey: B is
            // never late enough to miss it, and A is.
            connection(1, 2, 6000, 1800, 600),
        };
        w.tail_count = 2;
        w.unsafe_rule = PruningRule{DominanceOrder::Scalar, 2};
        w.safe_rule = PruningRule{DominanceOrder::FirstOrderStochastic, 2};
        return w;
    }

    SearchWitness tail_average_pruning_witness()
    {
        SearchWitness w;
        w.network.num_nodes = 3;
        w.network.num_scenarios = 3;
        w.network.legs = {
            // X is no worse than Y in CVaR at EVERY confidence level, so the
            // increasing convex order — dominance in the objective itself,
            // everywhere — discards Y.
            approach(0, 1, 600, {0, 540, 600}),
            // Y's middle day is early enough to catch the connection. X's is not.
            approach(0, 1, 600, {0, 300, 840}),
            connection(1, 2, 960, 1800, 240),
        };
        w.tail_count = 2;
        w.unsafe_rule = PruningRule{DominanceOrder::AllTailAverages, 2};
        w.safe_rule = PruningRule{DominanceOrder::FirstOrderStochastic, 2};
        return w;
    }

    SearchWitness correlated_delay_witness()
    {
        SearchWitness w;
        w.network.num_nodes = 3;
        w.network.num_scenarios = 3;
        w.network.legs = {
            // X: punctual on the disrupted day, late on a normal one.
            approach(0, 1, 1500, {0, 0, 400}),
            // Y: late only on the disrupted day — the same day its connection is
            // late. Stochastically LATER than X, so stochastic dominance
            // discards it.
            approach(0, 1, 1500, {500, 0, 0}),
            // The connecting line runs 600 s late on scenario 0 and on time
            // otherwise. Y is late exactly then, so Y never misses it.
            connection(1, 2, 1550, 1800, 300, {600, 0, 0}),
        };
        w.tail_count = 1;
        w.unsafe_rule = PruningRule{DominanceOrder::FirstOrderStochastic, 1};
        w.safe_rule = PruningRule{DominanceOrder::Statewise, 1};
        return w;
    }

    ProxyWitness additive_proxy_witness()
    {
        ProxyWitness w;
        w.network.num_nodes = 3;
        w.network.num_scenarios = 2;
        w.network.legs = {
            // Two minutes late, half the time. That is the whole risk on the leg.
            approach(0, 1, 600, {0, 120}),
            // And it is enough to miss a connection with a half-hour headway.
            connection(1, 2, 660, 1800, 600),
        };
        w.journey = {0, 1};
        w.alpha = 0.5;
        return w;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // § 7.  Random instances and measurement
    // ═══════════════════════════════════════════════════════════════════════════

    namespace
    {
        /// splitmix64. Written out rather than taken from <random> because
        /// std::uniform_int_distribution is not specified to produce the same
        /// values across library implementations, so a seeded figure quoted in a
        /// document would not reproduce on another toolchain — and a number that
        /// only reproduces on one machine is not evidence.
        struct SplitMix64
        {
            uint64_t state;

            uint64_t next()
            {
                uint64_t z = (state += 0x9E3779B97F4A7C15ULL);
                z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
                z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
                return z ^ (z >> 31);
            }

            /// Uniform-ish on [0, n). The modulo bias is of order n / 2^64 —
            /// around 1e-16 for every range drawn here — and is not worth a
            /// rejection loop, which would make the stream depend on how many
            /// draws happened to be rejected and so on the parameters.
            int64_t below(int64_t n) { return n <= 0 ? 0 : static_cast<int64_t>(next() % static_cast<uint64_t>(n)); }
        };
    } // namespace

    RandomInstance make_random_instance(const RandomInstanceConfig &cfg)
    {
        require(cfg.num_nodes >= 2, "risk: an instance needs at least two nodes");
        require(cfg.out_degree >= 1, "risk: an instance needs at least one leg per node");
        require(cfg.delay_support >= 1, "risk: a leg needs at least one delay atom");
        require(cfg.headway_s > 0, "risk: headway must be positive");
        require(cfg.delay_spread_s >= 0, "risk: delay spread must be non-negative");
        require(cfg.disruption_s >= 0, "risk: disruption must be non-negative");

        // ── The scenario space ────────────────────────────────────────────────
        // One coordinate per LAYER, not per leg. A leg's delay is a function of
        // the coordinate of the node it departs from; every leg on a journey
        // reaching that node departs from a lower-numbered node, so it is a
        // function of strictly earlier coordinates. Delays are therefore exactly
        // independent of the prefix that meets them — which is the hypothesis the
        // stochastic-dominance result needs, and one coordinate per leg would
        // have made the scenario space exponential in the leg count for no
        // additional independence.
        const uint32_t layers = cfg.num_nodes - 1;
        int64_t scenarios = 1;
        for (uint32_t i = 0; i < layers; ++i)
        {
            scenarios *= static_cast<int64_t>(cfg.delay_support);
            require(scenarios <= cfg.max_scenarios,
                    "risk: the product scenario space exceeds max_scenarios; "
                    "reduce num_nodes or delay_support");
        }
        const bool independent = (cfg.disruption_s == 0);
        const int64_t shift_support = independent ? 1 : 2;
        scenarios *= shift_support;
        require(scenarios <= cfg.max_scenarios,
                "risk: the product scenario space exceeds max_scenarios; "
                "reduce num_nodes or delay_support");

        SplitMix64 rng{cfg.seed * 0x2545F4914F6CDD1DULL + 0x9E3779B97F4A7C15ULL};

        auto coordinate = [&](std::size_t w, uint32_t layer) {
            std::size_t stride = 1;
            for (uint32_t i = 0; i < layer; ++i)
                stride *= cfg.delay_support;
            return (w / stride) % cfg.delay_support;
        };
        const std::size_t shift_stride = static_cast<std::size_t>(scenarios / shift_support);

        RandomInstance inst;
        inst.network.num_nodes = cfg.num_nodes;
        inst.network.num_scenarios = static_cast<std::size_t>(scenarios);
        inst.source = 0;
        inst.destination = cfg.num_nodes - 1;
        inst.departure_time_s = 0;

        auto add_leg = [&](uint32_t from, uint32_t to) {
            StochasticLeg leg;
            leg.from = from;
            leg.to = to;
            leg.headway_s = cfg.headway_s;
            leg.offset_s = rng.below(cfg.headway_s);
            leg.ride_s = cfg.ride_s + rng.below(cfg.ride_s / 2 + 1);
            leg.min_transfer_s = 0;

            // Each leg gets its OWN delay atoms, read through its layer's
            // coordinate. Sharing one atom set across a layer — the first thing
            // tried — gave every parallel service on a link an identical delay
            // vector, so their arrival laws were translates of one another,
            // almost always comparable, and no pruning rule ever had a decision
            // to get wrong. The independence the proof needs is between a leg
            // and the PREFIX that meets it, not between two legs leaving the
            // same node, and only the former is what the per-layer coordinate
            // buys.
            std::vector<int64_t> leg_atoms(cfg.delay_support);
            for (auto &a : leg_atoms)
                a = rng.below(cfg.delay_spread_s + 1);

            leg.delay_s.resize(static_cast<std::size_t>(scenarios));
            for (std::size_t w = 0; w < leg.delay_s.size(); ++w)
                leg.delay_s[w] = leg_atoms[coordinate(w, from)];

            if (!independent)
            {
                leg.shift_s.assign(static_cast<std::size_t>(scenarios), 0);
                for (std::size_t w = 0; w < leg.shift_s.size(); ++w)
                    leg.shift_s[w] = (w / shift_stride) == 0 ? 0 : cfg.disruption_s;
            }
            inst.network.legs.push_back(std::move(leg));
        };

        for (uint32_t u = 0; u + 1 < cfg.num_nodes; ++u)
        {
            for (uint32_t d = 0; d < cfg.out_degree; ++d)
                add_leg(u, u + 1);
            if (u + 2 < cfg.num_nodes)
                add_leg(u, u + 2); // a skip service, so journeys differ in length too
        }

        return inst;
    }

    RuleMeasurement measure_rule(const RandomInstance &instance,
                                 const PruningRule &rule,
                                 uint32_t max_legs)
    {
        RiskSearchConfig cfg;
        cfg.rule = rule;
        cfg.max_legs = max_legs;

        const RiskSearchResult r =
            risk_search(instance.network, instance.source, instance.departure_time_s, cfg);
        const std::vector<Journey> all =
            enumerate_journeys(instance.network, instance.source, instance.departure_time_s, max_legs);

        const std::size_t m = rule.tail_count;
        const auto found = best_tail_total(r, instance.destination, m);
        const auto truth = best_tail_total(all, instance.destination, m);

        RuleMeasurement out;
        out.max_frontier = r.max_frontier;
        out.mean_frontier = r.mean_frontier;
        out.labels_created = r.labels_created;
        out.labels_pruned = r.labels_pruned;
        out.unsupported_prunings = r.unsupported_prunings;
        out.destination_reached = truth.has_value();
        out.exact = (found.has_value() == truth.has_value()) &&
                    (!truth.has_value() || *found == *truth);
        out.regret_tail_total =
            (found.has_value() && truth.has_value()) ? (*found - *truth) : 0;
        return out;
    }

} // namespace namma_metro
