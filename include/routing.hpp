#pragma once

#include "graph.hpp"
#include "arena_allocator.hpp"
#include <cstdint>
#include <vector>
#include <queue>
#include <functional>
#include <limits>
#include <memory>
#include <optional>

/**
 * @file routing.hpp
 * @brief Time-dependent bi-criteria Pareto-Dijkstra routing engine.
 *
 * Mathematical formulation:
 *   Graph G = (V, E) where V = transit stops, E = timetabled service legs.
 *   Edge weight: w(u, v, t) = arrival_time at v given departure at u at time t.
 *
 *   Objective: compute the non-dominated (time, crowd) frontier between a source
 *   node s and all reachable destinations. SCOPE NOTE: the engine expands one
 *   composite-optimal departure per link (the lambda-minimizer among the next k
 *   departures), so the returned frontier is the set of non-dominated trade-offs
 *   ACROSS ROUTE CHOICES at a fixed lambda — not the full frontier over every
 *   possible boarding on a multi-departure link. See tests/test_pareto_oracle.cpp.
 *
 *   Markowitz analogy:
 *     Scalarized objective: min E[travel_time] + λ·σ[crowd_weight]
 *     where λ is the user's risk-aversion coefficient (wait tolerance).
 *     This is isomorphic to Markowitz mean-variance: min σ² - μ/λ.
 *     Limitation: transit delays are right-skewed (bounded below by 0,
 *     unbounded above), so variance symmetrically penalises early arrivals.
 *     Semi-variance or CVaR would be economically superior. See write-up.tex.
 */

namespace namma_metro
{

    // ═══════════════════════════════════════════════════════════════════════════
    // § 1.  Routing Label
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * @brief A Pareto label representing one non-dominated path to a node.
     *
     * Each node in the graph may maintain multiple labels forming its Pareto
     * frontier. The algorithm terminates when no label can be improved.
     *
     * Memory: allocated exclusively from ArenaAllocator. Never heap-allocated.
     */
    struct Label
    {
        uint32_t node;         ///< Destination node index this label is associated with.
        uint32_t arrival_time; ///< Total travel time in seconds from source departure.
        uint32_t crowd_cost;   ///< Cumulative crowd_weight + penalty along path.
        uint32_t predecessor;  ///< Node index of the previous hop (for path reconstruction).
                               ///< UINT32_MAX for source label.

        /// Priority queue ordering: min-heap on arrival_time (primary), crowd_cost (secondary).
        bool operator>(const Label &rhs) const noexcept
        {
            if (arrival_time != rhs.arrival_time)
                return arrival_time > rhs.arrival_time;
            return crowd_cost > rhs.crowd_cost;
        }
    };

    static_assert(sizeof(Label) == 16,
                  "Label must be 16 bytes to fit 4 labels per 64-byte cache line.");

    // ═══════════════════════════════════════════════════════════════════════════
    // § 2.  ParetoLabelSet
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * @brief Maintains the non-dominated (time, crowd) Pareto frontier for one node.
     *
     * Invariants (must hold after every call to insert_and_dominate):
     *   1. All labels are sorted ascending by arrival_time.
     *   2. crowd_cost is strictly DECREASING across the sorted sequence.
     *      (Earlier arrival time → higher crowd cost; later arrival time → lower
     *       crowd cost.  This follows from Pareto non-dominance: if labels were
     *       sorted by time ascending and crowd were non-decreasing, the right-hand
     *       label would be dominated on BOTH dimensions and should have been
     *       pruned during insertion.)
     *
     * Complexity:
     *   insert_and_dominate: O(log k) for binary search + O(k) worst-case pruning,
     *   where k = |labels_|. In practice k is very small (≤ 5 for transit graphs).
     *
     * Memory: labels_ holds raw pointers into the ArenaAllocator. The arena's
     * lifetime must exceed this object's lifetime.
     */
    class ParetoLabelSet
    {
    public:
        /**
         * @brief Attempt to insert @p new_label into the Pareto frontier.
         *
         * ═══════════════════════════════════════════════════════════════════════
         * CORE COMPONENT — correctness-critical Pareto dominance logic.
         * Implemented in src/routing.cpp following the three-step protocol below.
         * ═══════════════════════════════════════════════════════════════════════
         *
         * Three-step protocol:
         *
         * STEP 1 — O(log k) Binary Search:
         *   Use std::lower_bound on labels_ (sorted by arrival_time) to find the
         *   insertion point `it` for new_label.arrival_time. This is the first
         *   position where labels_[it]->arrival_time >= new_label->arrival_time.
         *
         *   CRITICAL: labels_ is std::vector<Label*>. You MUST supply an explicit
         *   comparator — without one, lower_bound compares raw pointer addresses
         *   (arbitrary heap locations), not arrival_time values. The binary search
         *   will silently land at the wrong position and the entire Pareto set will
         *   be corrupted, producing plausible-looking but incorrect output.
         *
         *   Correct call:
         *     auto it = std::lower_bound(
         *         labels_.begin(), labels_.end(),
         *         new_label->arrival_time,
         *         [](const Label* l, uint32_t t){ return l->arrival_time < t; });
         *
         *   Wrong (DO NOT write this — compares addresses, not times):
         *     auto it = std::lower_bound(labels_.begin(), labels_.end(), new_label);
         *
         * STEP 2 — O(1) Backward Dominance Check:
         *   If `it` is not the beginning (it != labels_.begin()), examine the
         *   immediately preceding label `prev = *(it - 1)`.
         *   If prev->arrival_time <= new_label->arrival_time AND
         *      prev->crowd_cost  <= new_label->crowd_cost,
         *   then `new_label` is dominated. Deallocate it from the arena and
         *   return false immediately.
         *
         * STEP 2b — Same-Time Dominance Check (REQUIRED — handles exact duplicates):
         *   Step 2 only inspects the label to the LEFT of `it`.  When lower_bound
         *   lands exactly ON a label with the same arrival_time (common when the
         *   same train is relaxed twice), `it` is not past begin, so Step 2 is
         *   skipped.  Without this check, Step 3 would erase the existing label
         *   and insert the duplicate, incorrectly returning `true`.
         *
         *   Add this immediately before Step 3:
         *     if (it != labels_.end()  &&
         *         (*it)->arrival_time == new_label->arrival_time &&
         *         (*it)->crowd_cost   <= new_label->crowd_cost) {
         *         arena.deallocate(new_label);
         *         return false;  // new_label dominated: same time, equal/worse crowd
         *     }
         *
         * STEP 3 — Amortized O(1) Forward Pruning:
         *   Starting from `it`, scan rightward. While the iterator is valid and
         *   (*it)->crowd_cost >= new_label->crowd_cost, the label at *it is
         *   dominated by new_label (same or better time AND same or better crowd).
         *   Deallocate each dominated label back to the arena, collect the
         *   iterators, then call labels_.erase(range_begin, range_end).
         *   After Steps 2 and 2b, any label at `it` with equal arrival_time will
         *   have strictly higher crowd_cost, so Step 3 correctly evicts it.
         *
         * After pruning, insert new_label at the correct sorted position.
         * Return true to signal a successful, non-dominated insertion.
         *
         * Mathematical justification:
         *   The Pareto invariant is: sorted by arrival_time ASCENDING, crowd_cost
         *   strictly DECREASING (earlier arrival = higher crowd; later = lower).
         *   Step 2 rejects labels dominated by a left neighbour.
         *   Step 2b rejects exact duplicates and same-time inferior labels.
         *   Step 3 evicts all right-side labels that new_label now dominates.
         *   Together these restore the strict-decreasing-crowd invariant — the
         *   core correctness argument for Pareto-Dijkstra label management.
         *
         * @param new_label   Pointer to a Label allocated from ArenaAllocator.
         *                    Ownership transfers to this set. Deallocated here
         *                    if dominated.
         * @param arena       Reference to the allocator for deallocation of
         *                    dominated labels.
         * @return            true  = new_label was inserted (Pareto-non-dominated).
         *                    false = new_label was dominated and has been freed.
         */
        bool insert_and_dominate(Label *new_label, ArenaAllocator<Label> &arena);

        // Implemented in src/routing.cpp following the three-step protocol above.

        [[nodiscard]] bool empty() const noexcept { return labels_.empty(); }
        [[nodiscard]] std::size_t size() const noexcept { return labels_.size(); }
        [[nodiscard]] const std::vector<Label *> &labels() const noexcept { return labels_; }

        /// Release all labels back to arena (called when resetting between queries).
        void clear(ArenaAllocator<Label> &arena)
        {
            for (Label *l : labels_)
                arena.deallocate(l);
            labels_.clear();
        }

        /// Wipe the labels_ vector WITHOUT returning each label to the arena.
        /// ONLY safe to call AFTER arena_->reset() has bulk-freed all labels.
        /// Used by ParetoDijkstra::run() to avoid redundant per-label deallocate
        /// calls when the arena is being reset as a whole.
        void unsafe_clear_after_arena_reset() noexcept
        {
            labels_.clear();
        }

    private:
        std::vector<Label *> labels_; ///< Sorted ascending by arrival_time. Invariant: crowd_cost strictly DECREASING.
    };

    // ═══════════════════════════════════════════════════════════════════════════
    // § 3.  Bounded-Wait Lookahead — CORE COMPONENT
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * @brief Configuration parameters for the Bounded-Wait Lookahead policy.
     *
     * The lookahead replaces strict "board-next-available-train" with a policy
     * that evaluates k upcoming departures within a W_max-second window and
     * selects the one minimizing composite cost = travel_time + penalty.
     *
     * This is necessary to preserve the FIFO property under synthetic crowd
     * penalties. See docs/write-up.tex for the mathematical proof.
     */
    struct LookaheadConfig
    {
        uint32_t k_departures = 5;     ///< Number of upcoming departures to evaluate.
        uint32_t W_max_seconds = 1800; ///< Maximum waiting time (default: 30 minutes).
        float lambda = 1.0f;           ///< Crowd-aversion coefficient (Markowitz analogy).
    };

    /**
     * @brief Select the optimal departure from node u to node v given current time.
     *
     * ═══════════════════════════════════════════════════════════════════════════
     * CORE COMPONENT — Bounded-Wait Lookahead departure selection.
     * Implemented in src/routing.cpp.
     * ═══════════════════════════════════════════════════════════════════════════
     *
     * Mathematical context:
     *   FIFO property: t1 + w(u,v,t1) ≤ t2 + w(u,v,t2) for all t1 ≤ t2.
     *   A strict next-train policy violates FIFO when crowd penalties drop
     *   precipitously after morning peak: a later departure may have lower
     *   penalty, causing earlier-departure-later-arrival, breaking Dijkstra's
     *   optimality subpath property.
     *
     *   The Bounded-Wait Lookahead resolves this by:
     *     For each of the next k departures of service (u→v) with
     *     departure_time in [current_time, current_time + W_max]:
     *       composite_cost(e) = e.travel_time + lambda * e.crowd_weight + e.penalty
     *     Return the edge e* minimizing composite_cost(e).
     *
     *   Proof that this preserves FIFO:
     *     The penalty derivative constraint d/dt(penalty) ≥ -1 ensures that
     *     any waiting benefit is bounded by the waiting cost, preventing the
     *     "later departure, earlier arrival" violation.
     *
     * @param graph         The CSR graph (for edge iteration).
     * @param config        Lookahead configuration (k, W_max, lambda).
     * @param current_time  Current time in seconds past midnight.
     * @param u             Source node index.
     * @param v             Destination node index.
     * @return              The optimal Edge to board, or std::nullopt if no
     *                      valid departure exists within the W_max window.
     *
     * Implementation hints:
     *   1. Use graph.edges_of(u) to get the [begin, end) edge range.
     *   2. Since edges are sorted by departure_time within each source node
     *      (guaranteed by GraphBuilder), use std::lower_bound to find the
     *      first edge with departure_time >= current_time.
     *   3. Iterate forward, counting only edges where `e->destination == v`
     *      toward the k_departures budget. Skip edges to other destinations
     *      without incrementing the counter. Break when:
     *        (a) the k budget (edges-to-v) is exhausted, OR
     *        (b) e->departure_time > current_time + W_max_seconds.
     *      IMPORTANT: the CSR stores ALL edges from u sorted by departure_time,
     *      not just edges to v. Failing to skip non-v edges will exhaust k
     *      early and miss valid departures to v on multi-destination nodes.
     *   4. For each candidate edge (where e->destination == v), compute:
     *        composite_cost = (float)e->travel_time
     *                       + config.lambda * (float)e->crowd_weight
     *                       + (float)e->penalty
     *      Use float arithmetic: lambda is float and mixing with uint32_t needs
     *      explicit casts to avoid implicit narrowing.
     *      DO NOT include wait time (departure_time - current_time) in the
     *      composite. Wait is bounded by W_max_seconds and does not affect which
     *      departure minimises composite — it would only bias toward earlier
     *      trains regardless of crowd/travel tradeoffs.  Tests 5 and 6 in
     *      test_fifo.cpp will FAIL if you include wait.
     *   5. Return std::nullopt if no edges fall within the window.
     */
    std::optional<Edge> select_optimal_departure(
        const CSRGraph &graph,
        const LookaheadConfig &config,
        uint32_t current_time,
        uint32_t u,
        uint32_t v);
    // Implemented in src/routing.cpp.

    // ═══════════════════════════════════════════════════════════════════════════
    // § 4.  Pareto-Dijkstra Engine
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * @brief Result of a Pareto-Dijkstra query.
     */
    struct QueryResult
    {
        /// For each node v, the set of Pareto-optimal (arrival_time, crowd_cost) labels.
        /// Indexed by node id. May be empty for unreachable nodes.
        std::vector<ParetoLabelSet> pareto_sets;

        /// Query wall-clock latency in nanoseconds (populated by benchmark harness).
        uint64_t latency_ns = 0;
    };

    /**
     * @brief Multi-label correcting Pareto-Dijkstra router.
     *
     * Computes the non-dominated (time, crowd) frontier from a source node to all
     * reachable destinations (across route choices at a fixed lambda; see the
     * scope note in the file header).
     *
     * Algorithm sketch:
     *   pq = min-heap ordered by (arrival_time, crowd_cost)
     *   Push source label (arrival_time=departure_time, crowd_cost=0) to pq.
     *   while pq non-empty:
     *     pop Label L
     *     // Lazy-deletion filter: skip L if an already-settled label at L.node
     *     //   strictly dominates it (bi-criteria). See src/routing.cpp.
     *     for each edge e in graph.edges_of(L.node):
     *       optimal = select_optimal_departure(graph, config, L.arrival_time, L.node, e.destination)
     *       if optimal has value:
     *         new_label = arena.allocate(); placement-new Label{...}
     *         if pareto_sets[e.destination].insert_and_dominate(new_label, arena):
     *           pq.push(*new_label)
     *
     * Complexity: O(k·|E|·(k + log(k·|V|))) per query, where k is the maximum
     * Pareto frontier size at any node. Decomposed:
     *   - Heap operations: O(k·|E|) insertions each costing O(log(k·|V|))
     *     → O(k·|E|·log(k·|V|))
     *   - insert_and_dominate pruning: O(k) worst-case per call, called O(k·|E|) times
     *     → O(k²·|E|)
     *   - Full bound: O(k·|E|·(k + log(k·|V|)))
     * For BMRCL: k≤16, |E|≈300, |V|≈60 → ~76,800 pruning ops + ~62,400 heap ops.
     * The pruning term dominates at BMRCL scale.
     * Note: the single-label Dijkstra bound O((E+V)log V) does NOT apply here —
     * in the label-correcting setting each node may be settled up to k times.
     */
    class ParetoDijkstra
    {
    public:
        // ArenaAllocator<Label> at default capacity = 65536 × 16 bytes = 1 MB.
        // Held via unique_ptr rather than by value: as a value member it would sit
        // on the stack of whatever thread constructs ParetoDijkstra. The main
        // thread (8 MB stack) survives that, but a worker thread or a CI container
        // with a smaller stack would overflow silently at construction.
        explicit ParetoDijkstra(const CSRGraph &graph, LookaheadConfig config = {})
            : graph_(graph), config_(config), arena_(std::make_unique<ArenaAllocator<Label>>())
        {
            // Reserve dest_scratch_ to the maximum out-degree in the graph.
            // A fixed reserve of 64 would trigger reallocation on high-frequency
            // interchange nodes (Kempegowda/Majestic can have >100 departure edges).
            // Any reallocation inside the hot routing loop violates the zero-allocation invariant.
            // One O(V) pass at construction prevents all hot-path allocations.
            uint32_t max_deg = 0;
            for (uint32_t u = 0; u < graph.num_nodes; ++u)
                max_deg = std::max(max_deg, graph.out_degree(u));
            dest_scratch_.reserve(std::max(max_deg, static_cast<uint32_t>(64)));
        }

        /**
         * @brief Run Pareto-Dijkstra from @p source_node at @p departure_time.
         *
         * @param source_node     Origin stop index.
         * @param departure_time  Journey start time in seconds past midnight.
         * @return                QueryResult with Pareto frontiers for all nodes.
         */
        QueryResult run(uint32_t source_node, uint32_t departure_time);

        /// Allocation-free hot path: fills @p out in place. When the SAME QueryResult
        /// is reused across calls, every per-node frontier vector retains its capacity,
        /// so the timed routing loop performs zero heap allocations (Label objects come
        /// from the arena). Prefer this overload in benchmarks and servers.
        void run(uint32_t source_node, uint32_t departure_time, QueryResult &out);

        /// Reset arena between queries (preserves allocated capacity).
        void reset_arena() { arena_->reset(); }

        /// Pre-fault arena pages before benchmarking (call once at startup).
        void prefault_arena() { arena_->prefault(); }

    private:
        const CSRGraph &graph_;
        LookaheadConfig config_;
        std::unique_ptr<ArenaAllocator<Label>> arena_; ///< heap-allocated ~1MB

        using MinHeap = std::priority_queue<Label, std::vector<Label>, std::greater<Label>>;

        // A member, not a local, so the underlying std::vector retains its heap
        // capacity across queries — a local would allocate on every call. pq_ is
        // always empty at the end of run(), since the Dijkstra loop exits only
        // when the queue is drained; run() asserts that invariant on entry rather
        // than draining defensively.
        MinHeap pq_;

        // dest_scratch_: unique-destination scratch buffer; reserved at construction
        // to the graph's max out-degree and cleared (not reallocated) during routing.
        //
        // The per-node Pareto frontiers live in the caller-owned QueryResult passed to
        // run(..., QueryResult&). Reusing one QueryResult across queries keeps those
        // vectors' capacity, so no separate working-set member is needed and the timed
        // loop stays allocation-free. (best_time_ was removed in v8: written every run()
        // but never read — the lazy-deletion filter uses the frontier labels directly.)
        std::vector<uint32_t> dest_scratch_;        ///< unique-dest work buffer
    };

} // namespace namma_metro
