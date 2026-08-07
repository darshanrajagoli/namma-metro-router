#include <gtest/gtest.h>
#include "routing.hpp"
#include "arena_allocator.hpp"
#include <vector>

/**
 * @file test_dominance.cpp
 * @brief Google Test suite for ParetoLabelSet::insert_and_dominate().
 *
 * PURPOSE:
 *   These tests define the exact correctness contract for the Pareto label
 *   insertion and dominance pruning logic in
 *   ParetoLabelSet::insert_and_dominate() (src/routing.cpp). Each failure
 *   message names the step of the three-step protocol in routing.hpp §2 that
 *   the failure implicates.
 *
 * MATHEMATICAL CONTEXT:
 *   Label (t, c) dominates (t', c') iff t ≤ t' AND c ≤ c'.
 *   A Pareto frontier is the minimal set of labels where no label dominates
 *   any other. Maintained as a vector sorted ascending by t, with strictly
 *   DECREASING c (earlier arrival = higher crowd; a non-decreasing crowd
 *   would mean the right-hand label is dominated and must be pruned).
 *
 * INVARIANTS verified by these tests:
 *   1. After any insert, labels_ is sorted ascending by arrival_time.
 *   2. After any insert, secondary_cost is strictly DECREASING across labels_.
 *   3. Memory is correctly returned to the arena when a label is dominated.
 *   4. insert_and_dominate returns true iff the label was NOT dominated.
 */

using namespace namma_metro;

// ─────────────────────────────────────────────────────────────────────────
// Helper: create a Label on the arena
// ─────────────────────────────────────────────────────────────────────────
static Label* make_label(ArenaAllocator<Label>& arena,
                          uint32_t time, uint32_t crowd,
                          uint32_t node = 0) {
    Label* l = arena.allocate();
    new(l) Label{node, time, crowd, /*predecessor=*/UINT32_MAX};
    return l;
}

// ─────────────────────────────────────────────────────────────────────────
// Test fixture
// ─────────────────────────────────────────────────────────────────────────
class ParetoSetTest : public ::testing::Test {
protected:
    ArenaAllocator<Label> arena;
    ParetoLabelSet pset;

    void TearDown() override {
        pset.clear(arena);
        EXPECT_EQ(arena.used_count(), 0u)
            << "Memory leak detected: not all labels returned to arena";
        EXPECT_TRUE(arena.check_invariant())
            << "Arena invariant violated after test teardown";
    }
};

// ─────────────────────────────────────────────────────────────────────────
// Test 1: Insert into empty set — must always succeed
// ─────────────────────────────────────────────────────────────────────────
TEST_F(ParetoSetTest, InsertIntoEmptySet_Succeeds) {
    Label* l = make_label(arena, /*time=*/100, /*crowd=*/50);

    bool inserted = pset.insert_and_dominate(l, arena);

    EXPECT_TRUE(inserted) << "Insertion into empty set must always succeed";
    EXPECT_EQ(pset.size(), 1u);
    EXPECT_EQ(pset.labels()[0]->arrival_time, 100u);
    EXPECT_EQ(pset.labels()[0]->secondary_cost,   50u);
}

// ─────────────────────────────────────────────────────────────────────────
// Test 2: Non-dominated label — both criteria better
// (t=80, c=30) vs existing (t=100, c=50): new label dominates → inserted,
// existing label must be pruned and freed.
// ─────────────────────────────────────────────────────────────────────────
TEST_F(ParetoSetTest, BetterOnBothCriteria_DominatesExisting) {
    Label* existing = make_label(arena, 100, 50);
    pset.insert_and_dominate(existing, arena);

    Label* better = make_label(arena, 80, 30); // strictly dominates existing
    bool inserted = pset.insert_and_dominate(better, arena);

    EXPECT_TRUE(inserted) << "Label dominating all existing labels must be inserted";
    EXPECT_EQ(pset.size(), 1u)
        << "The dominated (100, 50) label must have been pruned from the set";
    EXPECT_EQ(pset.labels()[0]->arrival_time, 80u);
    EXPECT_EQ(pset.labels()[0]->secondary_cost,   30u);

    // Arena should hold exactly 1 label (existing was freed during pruning)
    EXPECT_EQ(arena.used_count(), 1u)
        << "Dominated label must be deallocated back to arena";
}

// ─────────────────────────────────────────────────────────────────────────
// Test 3: Dominated new label — existing is strictly better
// Existing (80, 30). New (100, 50): t=100 ≥ 80 AND c=50 ≥ 30.
// New label is dominated → must be rejected and freed.
// ─────────────────────────────────────────────────────────────────────────
TEST_F(ParetoSetTest, DominatedNewLabel_RejectedAndFreed) {
    Label* dominant = make_label(arena, 80, 30);
    pset.insert_and_dominate(dominant, arena);

    size_t arena_before = arena.used_count(); // = 1

    Label* dominated = make_label(arena, 100, 50);
    bool inserted = pset.insert_and_dominate(dominated, arena);

    EXPECT_FALSE(inserted) << "Dominated label must NOT be inserted";
    EXPECT_EQ(pset.size(), 1u) << "Set size must remain 1";
    EXPECT_EQ(arena.used_count(), arena_before)
        << "Rejected label must be deallocated, restoring arena count to "
        << arena_before;
}

// ─────────────────────────────────────────────────────────────────────────
// Test 4: Pareto trade-off — incomparable labels, both survive
// (80, 50): fast but crowded
// (100, 20): slow but comfortable
// Neither dominates the other → both must be in the Pareto set.
// ─────────────────────────────────────────────────────────────────────────
TEST_F(ParetoSetTest, ParetoIncomparable_BothSurvive) {
    Label* fast_crowded = make_label(arena, 80, 50);
    Label* slow_comfy   = make_label(arena, 100, 20);

    pset.insert_and_dominate(fast_crowded, arena);
    bool inserted = pset.insert_and_dominate(slow_comfy, arena);

    EXPECT_TRUE(inserted) << "Non-dominated (100, 20) must be inserted";
    EXPECT_EQ(pset.size(), 2u) << "Both non-dominated labels must coexist";

    // Verify sorted order: time-ascending
    EXPECT_LT(pset.labels()[0]->arrival_time, pset.labels()[1]->arrival_time)
        << "Labels must remain sorted ascending by arrival_time";

    // Verify the Pareto invariant: sorted by time ascending means crowd DECREASING.
    // (80, 50) comes first (faster but more crowded).
    // (100, 20) comes second (slower but comfortable).
    // crowd must go 50 → 20, i.e. labels[0]->crowd > labels[1]->crowd.
    // If labels[0]->crowd <= labels[1]->crowd, the right-hand label is dominated
    // and should have been pruned. Check your Step 3 forward pruning logic.
    EXPECT_GT(pset.labels()[0]->secondary_cost, pset.labels()[1]->secondary_cost)
        << "Pareto invariant broken: sorted by arrival_time ascending, "
           "secondary_cost must be strictly DECREASING. "
           "Expected labels[0]->crowd(50) > labels[1]->crowd(20).";
}

// ─────────────────────────────────────────────────────────────────────────
// Test 5: Chain insertion preserves sorted order
// Insert labels in reverse time order: (300,10), (200,30), (100,60)
// After all insertions, must be sorted ascending: (100,60),(200,30),(300,10)
// ─────────────────────────────────────────────────────────────────────────
TEST_F(ParetoSetTest, ReverseOrderInsertions_SortedCorrectly) {
    pset.insert_and_dominate(make_label(arena, 300, 10), arena);
    pset.insert_and_dominate(make_label(arena, 200, 30), arena);
    pset.insert_and_dominate(make_label(arena, 100, 60), arena);

    ASSERT_EQ(pset.size(), 3u) << "All three incomparable labels should survive";

    EXPECT_EQ(pset.labels()[0]->arrival_time, 100u);
    EXPECT_EQ(pset.labels()[1]->arrival_time, 200u);
    EXPECT_EQ(pset.labels()[2]->arrival_time, 300u);
}

// ─────────────────────────────────────────────────────────────────────────
// Test 6: Bulk forward pruning
// Existing Pareto set: {(100,60), (200,40), (300,20), (400,10)}
// Insert new label (150, 5): t=150 > 100 so not dominated by (100,60).
// New label has crowd=5 which is ≤ crowd of (200,40),(300,20),(400,10).
// Since new.time=150 ≤ their times AND new.crowd=5 ≤ their crowds,
// all three right-side labels are dominated and must be pruned.
// Final set: {(100,60), (150,5)}
// ─────────────────────────────────────────────────────────────────────────
TEST_F(ParetoSetTest, BulkForwardPruning_RemovesMultipleLabels) {
    pset.insert_and_dominate(make_label(arena, 100, 60), arena);
    pset.insert_and_dominate(make_label(arena, 200, 40), arena);
    pset.insert_and_dominate(make_label(arena, 300, 20), arena);
    pset.insert_and_dominate(make_label(arena, 400, 10), arena);

    ASSERT_EQ(pset.size(), 4u) << "All 4 labels should form initial Pareto set";

    // Now insert (150, 5) — dominates (200,40), (300,20), (400,10)
    Label* sweeper = make_label(arena, 150, 5);
    bool inserted = pset.insert_and_dominate(sweeper, arena);

    EXPECT_TRUE(inserted);
    ASSERT_EQ(pset.size(), 2u)
        << "Expected 2 labels after bulk pruning: {(100,60), (150,5)}. "
           "Your implementation may be missing the forward pruning loop (Step 3).";

    EXPECT_EQ(pset.labels()[0]->arrival_time, 100u);
    EXPECT_EQ(pset.labels()[0]->secondary_cost,    60u);
    EXPECT_EQ(pset.labels()[1]->arrival_time, 150u);
    EXPECT_EQ(pset.labels()[1]->secondary_cost,    5u);

    EXPECT_EQ(arena.used_count(), 2u)
        << "3 pruned labels must have been deallocated to arena";
}

// ─────────────────────────────────────────────────────────────────────────
// Test 7: Exact duplicate — should be treated as dominated (not inserted)
// (100, 50) already in set. Insert another (100, 50).
// Second insertion has t ≥ existing AND c ≥ existing → dominated.
// ─────────────────────────────────────────────────────────────────────────
TEST_F(ParetoSetTest, ExactDuplicate_SecondIsRejected) {
    pset.insert_and_dominate(make_label(arena, 100, 50), arena);
    bool inserted = pset.insert_and_dominate(make_label(arena, 100, 50), arena);

    EXPECT_FALSE(inserted) << "Exact duplicate is dominated and must be rejected";
    EXPECT_EQ(pset.size(), 1u);
    EXPECT_EQ(arena.used_count(), 1u);
}

// ─────────────────────────────────────────────────────────────────────────
// Test 8: No memory leak after clear()
// ─────────────────────────────────────────────────────────────────────────
TEST_F(ParetoSetTest, Clear_ReturnsAllLabelsToArena) {
    for (int i = 0; i < 5; ++i) {
        pset.insert_and_dominate(
            make_label(arena, 100 + i * 50, 100 - i * 20), arena
        );
    }

    std::size_t before_clear = arena.used_count();
    pset.clear(arena);

    EXPECT_EQ(pset.size(), 0u);
    EXPECT_EQ(arena.used_count(), 0u)
        << "clear() must return all " << before_clear << " labels to arena";
    EXPECT_TRUE(arena.check_invariant());
}

// ─────────────────────────────────────────────────────────────────────────
// Test 9: Same arrival_time, NEW has WORSE crowd → must be rejected (Step 2b)
//
// This is the case that would PASS a broken implementation missing Step 2b:
//   lower_bound lands on the existing label (100,30). it != begin is false
//   (or the existing label IS at begin), so Step 2 is skipped. Without Step 2b,
//   Step 3's condition (*it)->secondary_cost >= new_label->secondary_cost becomes
//   30 >= 50 = false, so Step 3 does not prune. The new worse label gets
//   inserted BEFORE the existing better label, violating the Pareto invariant.
// ─────────────────────────────────────────────────────────────────────────
TEST_F(ParetoSetTest, SameTime_WorseCrowd_IsRejectedByStep2b) {
    pset.insert_and_dominate(make_label(arena, 100, 30), arena); // existing: (100,30)

    bool inserted = pset.insert_and_dominate(make_label(arena, 100, 50), arena); // (100,50)

    EXPECT_FALSE(inserted)
        << "Same arrival_time with worse crowd must be rejected (Step 2b). "
           "If true: Step 2b is missing. The set now contains {(100,50),(100,30)}, "
           "violating the strictly-decreasing-crowd invariant at equal times.";
    EXPECT_EQ(pset.size(), 1u)
        << "Set must still contain exactly 1 label after rejecting the worse-crowd duplicate";
    ASSERT_FALSE(pset.labels().empty());
    EXPECT_EQ(pset.labels()[0]->arrival_time, 100u);
    EXPECT_EQ(pset.labels()[0]->secondary_cost,   30u)
        << "The surviving label must be the BETTER one (crowd=30), not the worse (crowd=50)";
    EXPECT_EQ(arena.used_count(), 1u)
        << "Rejected label must be deallocated — arena leak detected";
}

// ─────────────────────────────────────────────────────────────────────────
// Test 10: New label dominates ALL existing labels → set clears entirely
//
// Existing set: {(200,50),(300,40),(400,30)}.
// Insert (100,5) which has strictly earlier time AND strictly lower crowd
// than all three existing labels. Forward pruning (Step 3) must walk the
// entire vector and evict all three.
// ─────────────────────────────────────────────────────────────────────────
TEST_F(ParetoSetTest, DominatesAllExisting_EntireSetCleared) {
    pset.insert_and_dominate(make_label(arena, 200, 50), arena);
    pset.insert_and_dominate(make_label(arena, 300, 40), arena);
    pset.insert_and_dominate(make_label(arena, 400, 30), arena);
    ASSERT_EQ(pset.size(), 3u) << "Setup: three non-dominated labels in set";
    ASSERT_EQ(arena.used_count(), 3u);

    bool inserted = pset.insert_and_dominate(make_label(arena, 100, 5), arena);

    EXPECT_TRUE(inserted)
        << "(100,5) dominates all existing labels — must be inserted";
    EXPECT_EQ(pset.size(), 1u)
        << "All three existing labels must have been pruned by (100,5)";
    ASSERT_FALSE(pset.labels().empty());
    EXPECT_EQ(pset.labels()[0]->arrival_time, 100u);
    EXPECT_EQ(pset.labels()[0]->secondary_cost,   5u);
    EXPECT_EQ(arena.used_count(), 1u)
        << "All three evicted labels must be deallocated — arena leak detected";
    EXPECT_TRUE(arena.check_invariant());
}

// ─────────────────────────────────────────────────────────────────────────
// Test 11: Insert order commutativity
//
// Two Pareto-incomparable labels A=(100,80) and B=(200,10) inserted in
// order {A, B} must produce the same final set as {B, A}.
// Verifies no asymmetric bug in binary search or forward pruning that
// depends on insertion order.
// ─────────────────────────────────────────────────────────────────────────
TEST_F(ParetoSetTest, InsertOrderCommutativity_ABequalsBA) {
    // Order A then B
    ParetoLabelSet pset_ab;
    pset_ab.insert_and_dominate(make_label(arena, 100, 80), arena);
    pset_ab.insert_and_dominate(make_label(arena, 200, 10), arena);

    // Order B then A
    ParetoLabelSet pset_ba;
    pset_ba.insert_and_dominate(make_label(arena, 200, 10), arena);
    pset_ba.insert_and_dominate(make_label(arena, 100, 80), arena);

    ASSERT_EQ(pset_ab.size(), 2u) << "A-B order: expected 2 non-dominated labels";
    ASSERT_EQ(pset_ba.size(), 2u) << "B-A order: expected 2 non-dominated labels";

    // Both sets must contain the same labels (sorted by arrival_time)
    EXPECT_EQ(pset_ab.labels()[0]->arrival_time, 100u);
    EXPECT_EQ(pset_ab.labels()[0]->secondary_cost,   80u);
    EXPECT_EQ(pset_ab.labels()[1]->arrival_time, 200u);
    EXPECT_EQ(pset_ab.labels()[1]->secondary_cost,   10u);

    EXPECT_EQ(pset_ba.labels()[0]->arrival_time, pset_ab.labels()[0]->arrival_time);
    EXPECT_EQ(pset_ba.labels()[0]->secondary_cost,   pset_ab.labels()[0]->secondary_cost);
    EXPECT_EQ(pset_ba.labels()[1]->arrival_time, pset_ab.labels()[1]->arrival_time);
    EXPECT_EQ(pset_ba.labels()[1]->secondary_cost,   pset_ab.labels()[1]->secondary_cost);

    // Cleanup — clear both sets manually to satisfy TearDown arena invariant
    pset_ab.clear(arena);
    pset_ba.clear(arena);
}
