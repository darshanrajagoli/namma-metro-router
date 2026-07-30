#include "routing.hpp"
#include <cassert>
#include <limits>
#include <algorithm>

namespace namma_metro
{

    // ═══════════════════════════════════════════════════════════════════════════
    // ParetoLabelSet::insert_and_dominate
    // ═══════════════════════════════════════════════════════════════════════════

    bool ParetoLabelSet::insert_and_dominate(Label *new_label, ArenaAllocator<Label> &arena)
    {
        // Sorted-vector Pareto insertion. Invariant preserved on return:
        //   labels_ is sorted ascending by arrival_time, with secondary_cost strictly
        //   DECREASING across the sequence (earlier arrival = higher crowd cost).
        // Contract: on a false return, new_label has already been deallocated.
        // See include/routing.hpp §2 for the derivation and
        // tests/test_dominance.cpp for the correctness contract.

        // ── STEP 1: O(log k) binary search for the insertion point. ──────────
        // The explicit comparator is mandatory: labels_ holds Label* and the
        // default lower_bound would order by raw pointer address, not by
        // arrival_time, silently corrupting the frontier.
        auto it = std::lower_bound(
            labels_.begin(), labels_.end(),
            new_label->arrival_time,
            [](const Label *l, uint32_t t) { return l->arrival_time < t; });

        // ── STEP 2: O(1) backward dominance check. ───────────────────────────
        // The left neighbour (*(it-1)) has a strictly smaller arrival_time. If
        // its secondary_cost is also <=, it dominates new_label on both criteria.
        if (it != labels_.begin())
        {
            const Label *prev = *(it - 1);
            if (prev->arrival_time <= new_label->arrival_time &&
                prev->secondary_cost <= new_label->secondary_cost)
            {
                arena.deallocate(new_label);
                return false; // dominated by the left neighbour
            }
        }

        // ── STEP 2b: same-arrival-time dominance check. ──────────────────────
        // lower_bound can land directly ON a label with an equal arrival_time
        // (e.g. the same train relaxed twice). That label sits to the RIGHT of
        // (it-1), so Step 2 misses it. If the existing label's secondary_cost is
        // equal or better, new_label is dominated.
        if (it != labels_.end() &&
            (*it)->arrival_time == new_label->arrival_time &&
            (*it)->secondary_cost <= new_label->secondary_cost)
        {
            arena.deallocate(new_label);
            return false; // same arrival time, equal or worse crowd
        }

        // ── STEP 3: amortised O(1) forward pruning. ──────────────────────────
        // Every label from `it` rightward whose secondary_cost is >= new_label's is
        // now dominated by new_label (equal-or-smaller arrival_time AND
        // equal-or-smaller secondary_cost; at least one is strictly smaller after
        // Steps 2/2b). Free each, then erase the contiguous run in one call.
        auto prune_end = it;
        while (prune_end != labels_.end() &&
               (*prune_end)->secondary_cost >= new_label->secondary_cost)
        {
            arena.deallocate(*prune_end);
            ++prune_end;
        }
        // erase() returns an iterator to the element that followed the removed
        // range — precisely the sorted position at which new_label belongs.
        auto insert_pos = labels_.erase(it, prune_end);
        labels_.insert(insert_pos, new_label);

        return true; // inserted: new_label is Pareto-non-dominated
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // select_optimal_departure
    // ═══════════════════════════════════════════════════════════════════════════

    std::optional<Edge> select_optimal_departure(
        const CSRGraph &graph,
        const LookaheadConfig &config,
        uint32_t current_time,
        uint32_t u,
        uint32_t v)
    {
        // Bounded-Wait Lookahead. Among the next k departures of service (u -> v)
        // that leave within [current_time, current_time + W_max_seconds], return
        // the one minimising the composite objective. This preserves FIFO under
        // the penalty derivative bound d/dt(penalty) >= -1.
        // See include/routing.hpp §3 and tests/test_fifo.cpp for the contract.

        const auto [edge_begin, edge_end] = graph.edges_of(u);

        // Edges from u are sorted by departure_time (GraphBuilder guarantees it),
        // so a binary search finds the first boardable departure in O(log deg(u)).
        // Boarding requires departure_time >= current_time, so a train leaving at
        // exactly current_time is catchable.
        const Edge *first = std::lower_bound(
            edge_begin, edge_end, current_time,
            [](const Edge &e, uint32_t t) { return e.departure_time < t; });

        const uint64_t window_end =
            static_cast<uint64_t>(current_time) + config.W_max_seconds;

        const Edge *best = nullptr;
        float best_composite = 0.0f;
        uint32_t seen_to_v = 0; // candidates to v evaluated against the k budget

        for (const Edge *e = first; e != edge_end; ++e)
        {
            // Past the bounded-wait window: because the range is sorted by
            // departure_time, every subsequent edge is also outside it.
            if (e->departure_time > window_end)
                break;

            // The CSR range holds ALL departures from u, not just those to v.
            // Edges to other destinations are skipped WITHOUT consuming the k
            // budget; otherwise a busy interchange could exhaust k before a
            // single edge to v is examined.
            if (e->destination != v)
                continue;

            // Composite objective, choosing between departures on the SAME link.
            //
            // Two regimes:
            //
            //  (a) Arrival-minimising — used when the second objective cannot
            //      discriminate between the candidates, so the only thing left to
            //      optimise is when the passenger actually gets there. This covers
            //      lambda == 0 (the crowd term vanishes) and the TransferCount
            //      objective (every service edge carries secondary_weight = 0, so
            //      the crowd term is identically zero for every candidate).
            //      Folding departure_time in is what makes this minimise ARRIVAL
            //      rather than ride DURATION.
            //
            //      Getting this wrong is not subtle in its effect: under
            //      TransferCount the (b) formula reduces to "shortest ride", which
            //      picks a later, faster train that arrives strictly later while
            //      saving zero transfers — measurably worse on every axis. The
            //      trade-off under TransferCount lives in the Pareto frontier
            //      across ROUTES, not in the choice of departure on one link.
            //
            //  (b) Duration-vs-crowd — the crowd-aware regime, where deliberately
            //      waiting for an emptier train is the entire point, so neither
            //      the wait nor the arrival time enters the cost.
            const bool arrival_minimising =
                (config.lambda == 0.0f) ||
                (graph.second_objective == SecondObjective::TransferCount);

            float composite;
            if (arrival_minimising)
            {
                composite = static_cast<float>(e->departure_time)
                          + static_cast<float>(e->travel_time)
                          + static_cast<float>(e->penalty);
            }
            else
            {
                composite = static_cast<float>(e->travel_time)
                          + config.lambda * static_cast<float>(e->secondary_weight)
                          + static_cast<float>(e->penalty);
            }

            if (best == nullptr || composite < best_composite)
            {
                best = e;
                best_composite = composite;
            }

            // k counts only edges to v: stop after evaluating the k-th candidate.
            if (++seen_to_v >= config.k_departures)
                break;
        }

        if (best == nullptr)
            return std::nullopt;
        return *best;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // ParetoDijkstra::run — Multi-label correcting Pareto-Dijkstra
    // ═══════════════════════════════════════════════════════════════════════════

    void ParetoDijkstra::run(uint32_t source_node, uint32_t departure_time, QueryResult &out)
    {
        assert(source_node < graph_.num_nodes);

        // arena_->reset() bulk-frees all labels from the previous query in O(1).
        // This MUST happen before we touch the frontier vectors below.
        arena_->reset();

        // Reuse the caller's buffers. resize() allocates only on the FIRST call with a
        // given `out`; on later calls (size already == num_nodes) it is skipped and each
        // frontier vector is cleared in place, retaining its capacity — so the timed loop
        // performs ZERO std::vector allocations (and zero Label allocs, via the arena).
        //
        // Safe to wipe labels_ vectors directly: arena_->reset() above already freed the
        // backing Label memory in bulk. Per-label deallocate is unnecessary.
        if (out.pareto_sets.size() != graph_.num_nodes)
            out.pareto_sets.resize(graph_.num_nodes);
        for (auto &ps : out.pareto_sets)
        {
            ps.unsafe_clear_after_arena_reset(); // clears labels_ vector, no arena call
        }

        // Short alias for the frontier buffer, used throughout the loop below.
        auto &result_pareto = out.pareto_sets;

        // There is deliberately no per-node best-arrival-time cache here. In a
        // single-objective Dijkstra such a cache drives the stale-label test, but
        // in the bi-criteria setting the lazy-deletion filter below must consult
        // the full frontier (result_pareto[node].labels()) — a scalar best-time
        // would wrongly discard labels that trade arrival time for lower crowd.
        // Maintaining one would cost O(V) per query and answer no question.

        // pq_ is always empty at the end of run(): the Dijkstra loop exits only
        // when pq_ is drained. Assert the invariant rather than draining
        // defensively, which would be a guaranteed-zero-iteration loop.
        assert(pq_.empty() && "pq_ must be empty at start of run(); previous run() did not drain");

        // ── Initialize source label ───────────────────────────────────────────
        Label *src_label = arena_->allocate();
        new (src_label) Label{source_node, departure_time, 0, std::numeric_limits<uint32_t>::max()};

        // Insert the source label into its Pareto set FIRST, then push a value
        // copy onto the priority queue. Insertion into an empty set is always
        // non-dominated, so it must succeed. The ordering matters: the
        // lazy-deletion filter below reads the frontier, so a label present in
        // the queue but absent from the frontier would be indistinguishable from
        // a stale one.
        const bool src_ok =
            result_pareto[source_node].insert_and_dominate(src_label, *arena_);
        assert(src_ok && "source label inserted into an empty set must succeed");
        (void)src_ok; // silence unused-variable warning under -DNDEBUG
        pq_.push(*src_label);

        // ── Main Dijkstra loop ────────────────────────────────────────────────
        while (!pq_.empty())
        {
            Label current = pq_.top();
            pq_.pop();

            // ── Lazy deletion: skip stale popped labels. ──────────────────────
            // std::priority_queue has no decrease-key, so superseded labels stay
            // in the heap. This is Pareto-Dijkstra, so "stale" means STRICTLY
            // dominated by an already-settled label at this node — NOT merely
            // "worse arrival_time". The single-objective test
            //   if (current.arrival_time > best_time[node]) continue;
            // would wrongly discard Pareto-optimal labels that trade a later
            // arrival for a lower crowd cost.
            //
            // The strict clause (at least one dimension strictly better) is
            // mandatory: the source label is inserted into its Pareto set before
            // being pushed, so a non-strict (<=,<=) test would let it dominate
            // itself on the first pop and the engine would return empty frontiers
            // for every query. Cost is O(k) per pop, k = per-node frontier size.
            bool dominated = false;
            for (const Label *s : result_pareto[current.node].labels())
            {
                if (s->arrival_time <= current.arrival_time &&
                    s->secondary_cost <= current.secondary_cost &&
                    (s->arrival_time < current.arrival_time ||
                     s->secondary_cost < current.secondary_cost))
                {
                    dominated = true;
                    break;
                }
            }
            if (dominated)
                continue;

            // ── Edge relaxation ───────────────────────────────────────────────
            auto [edge_begin, edge_end] = graph_.edges_of(current.node);

            // The CSR stores one edge per timetabled departure, NOT one per
            // neighbour. A node with 20 daily departures to the same destination
            // would otherwise trigger 20 select_optimal_departure() calls all
            // returning the same result, costing O(freq²) redundant arena allocs
            // and insert_and_dominate calls. Collect unique destination ids first.
            //
            // dest_scratch_ is a member reserved to the graph's maximum out-degree
            // at construction, not a fresh std::vector per expansion: that would
            // be one malloc+free per node pop on the hot path.
            dest_scratch_.clear(); // clears size, preserves capacity — no alloc
            {
                uint32_t prev_dest = UINT32_MAX;
                for (const Edge *e = edge_begin; e != edge_end; ++e)
                {
                    if (e->destination != prev_dest)
                    {
                        dest_scratch_.push_back(e->destination);
                        prev_dest = e->destination;
                    }
                }
            }
            // Note: edges are sorted by departure_time within each source node
            // (graph_builder guarantees this). Destinations may not be contiguous,
            // so we must de-duplicate across the full edge range.
            // For correctness, sort and unique:
            std::sort(dest_scratch_.begin(), dest_scratch_.end());
            dest_scratch_.erase(
                std::unique(dest_scratch_.begin(), dest_scratch_.end()),
                dest_scratch_.end());

            for (uint32_t dest : dest_scratch_)
            {
                // select_optimal_departure's k_departures budget counts only edges
                // to this specific destination — it filters by destination before
                // counting, so a busy interchange cannot exhaust k on edges bound
                // elsewhere before examining a single edge to `dest`.
                // See routing.hpp §3.
                auto opt_edge = select_optimal_departure(
                    graph_, config_, current.arrival_time, current.node, dest);

                if (!opt_edge.has_value())
                    continue;

                const uint32_t new_arrival = opt_edge->departure_time + opt_edge->travel_time;
                const uint32_t new_crowd = current.secondary_cost + opt_edge->secondary_weight + opt_edge->penalty;

                Label *new_label = arena_->allocate();
                new (new_label) Label{
                    dest,
                    new_arrival,
                    new_crowd,
                    current.node};

                if (result_pareto[dest].insert_and_dominate(new_label, *arena_))
                {
                    pq_.push(*new_label);
                }
                // If dominated: insert_and_dominate already freed new_label
            }

            // ── Transfer relaxation ───────────────────────────────────────────
            // Transfers are always available: the passenger arrives at
            // current.arrival_time and may start walking immediately, so there is
            // no departure to select and no bounded-wait window to respect. That
            // also makes them trivially FIFO — travel_time is a constant, so
            // t1 <= t2 implies t1 + w <= t2 + w for every pair of arrival times.
            //
            // The range is empty on feeds built without a transfer layer, so this
            // loop costs one pointer comparison in that case.
            const uint32_t transfer_cost =
                (graph_.second_objective == SecondObjective::TransferCount) ? 1u : 0u;

            auto [tr_begin, tr_end] = graph_.transfers_of(current.node);
            for (const TransferEdge *t = tr_begin; t != tr_end; ++t)
            {
                const uint32_t new_arrival = current.arrival_time + t->travel_time;
                const uint32_t new_secondary = current.secondary_cost + transfer_cost;

                Label *new_label = arena_->allocate();
                new (new_label) Label{
                    t->destination,
                    new_arrival,
                    new_secondary,
                    current.node};

                if (result_pareto[t->destination].insert_and_dominate(new_label, *arena_))
                {
                    pq_.push(*new_label);
                }
            }
        }

        // ── FM8 ADVERSARIAL AUDIT — predecessor chain and arena lifetime ──────
        // predecessor field in Label stores current.node (the NODE INDEX of the
        // prior hop), NOT a Label* pointer and NOT an arena slot index.
        // Path reconstruction: follow pred chain C.pred=B_id → B.pred=A_id →
        // UINT32_MAX (source sentinel). Terminates in ≤ |V| steps (non-negative
        // edge weights guarantee strictly non-increasing arrival_time along the
        // chain; zero-weight cycles are impossible with min travel_time=1s).
        //
        // ARENA LIFETIME WARNING: QueryResult.pareto_sets contains Label* pointers
        // into arena_. These pointers become dangling on the NEXT call to run()
        // because arena_->reset() is the first statement in run().
        //
        // Safe usage pattern:
        //   auto result = router.run(src, dep);
        //   // Consume result.pareto_sets HERE, before the next router.run() call.
        //   // Do NOT call router.reset_arena() explicitly — run() already does it.
        //
        // If you need to keep a result alive across multiple run() calls, deep-copy
        // the Label VALUES (not pointers) before the next run():
        //   std::vector<Label> my_labels;
        //   for (const Label* l : result.pareto_sets[v].labels())
        //       my_labels.push_back(*l);  // copy by value before arena reset
        //
        // LABEL-LEVEL RESOLUTION GAP: predecessor stores node ID, not a pointer
        // to which specific Pareto label at that node was the predecessor. If a
        // node has multiple Pareto labels (e.g., 3 labels at B: fast/crowded,
        // mid/mid, slow/uncrowded), a label at C with pred=B_id does not encode
        // which of B's 3 labels was used. Full path reconstruction requires
        // matching by arrival_time compatibility — ambiguous on wide frontiers.
        // For this build (penalty=0, simple graphs), this is acceptable.
        // A production implementation should store a Label* or arena index.

        // ── Done ──────────────────────────────────────────────────────────────
        // `out` is filled in place. When the caller reuses the same QueryResult across
        // queries (see the by-value wrapper below and main_bench.cpp), the timed loop is
        // allocation-free end to end: Label objects come from the arena, and every
        // per-node frontier vector retains its capacity via the in-place clear above.
    }

    // Convenience by-value overload: allocates a fresh QueryResult per call, so prefer
    // the output-parameter overload above on hot paths (benchmarks, servers).
    QueryResult ParetoDijkstra::run(uint32_t source_node, uint32_t departure_time)
    {
        QueryResult out;
        run(source_node, departure_time, out);
        return out;
    }

} // namespace namma_metro
