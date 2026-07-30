#include <gtest/gtest.h>
#include "arena_allocator.hpp"
#include "routing.hpp" // for Label (16 bytes — satisfies the >= sizeof(void*) requirement)
#include <set>
#include <vector>

/**
 * @file test_arena.cpp
 * @brief Google Test suite for ArenaAllocator<T, Capacity>.
 *
 * PURPOSE:
 *   Locks the contract of the arena-backed free-list allocator that makes the
 *   routing loop allocation-free. The Pareto-Dijkstra engine
 *   relies on three guarantees verified here:
 *     1. allocate()/deallocate() are O(1) and maintain used_count() exactly.
 *     2. deallocate() pushes the slot onto a free list that the NEXT allocate()
 *        pops first — recycling dominated Label memory without growing the bump
 *        pointer. (This is what keeps a long query inside the fixed arena.)
 *     3. The invariant used_count + free_list_length == bump_index holds after
 *        every operation, and exhaustion throws rather than corrupting memory.
 *
 * These tests are independent of the routing logic — they pass or fail purely
 * on the allocator implementation in arena_allocator.hpp.
 */

using namespace namma_metro;

// ─────────────────────────────────────────────────────────────────────────
// Fixture — a default-capacity arena (1 MB of Label slots), as used in routing
// ─────────────────────────────────────────────────────────────────────────
class ArenaTest : public ::testing::Test {
protected:
    ArenaAllocator<Label> arena;

    void TearDown() override {
        EXPECT_TRUE(arena.check_invariant())
            << "Arena invariant (used + free == bump) violated after test";
    }
};

// ─────────────────────────────────────────────────────────────────────────
// Test 1: A freshly constructed arena is empty and consistent
// ─────────────────────────────────────────────────────────────────────────
TEST_F(ArenaTest, FreshArena_IsEmpty) {
    EXPECT_EQ(arena.used_count(), 0u);
    EXPECT_EQ(arena.bump_index(), 0u);
    EXPECT_EQ(arena.free_list_length(), 0u);
    EXPECT_EQ(arena.capacity(), ARENA_DEFAULT_CAPACITY);
    EXPECT_TRUE(arena.check_invariant());
}

// ─────────────────────────────────────────────────────────────────────────
// Test 2: allocate() returns a usable, owned, zeroed slot and bumps the count
// ─────────────────────────────────────────────────────────────────────────
TEST_F(ArenaTest, Allocate_ReturnsZeroedOwnedSlot) {
    Label* p = arena.allocate();

    ASSERT_NE(p, nullptr) << "allocate() must never return null";
    EXPECT_TRUE(arena.owns(p)) << "Returned pointer must lie inside the arena";
    EXPECT_EQ(arena.used_count(), 1u);
    EXPECT_EQ(arena.bump_index(), 1u);

    // Contract: the slot is zeroed before being handed back.
    EXPECT_EQ(p->node, 0u);
    EXPECT_EQ(p->arrival_time, 0u);
    EXPECT_EQ(p->secondary_cost, 0u);
    EXPECT_EQ(p->predecessor, 0u);

    arena.deallocate(p);
}

// ─────────────────────────────────────────────────────────────────────────
// Test 3: deallocate() decrements used_count and pushes onto the free list,
// WITHOUT rewinding the bump pointer.
// ─────────────────────────────────────────────────────────────────────────
TEST_F(ArenaTest, Deallocate_PushesToFreeList) {
    Label* p = arena.allocate();
    ASSERT_EQ(arena.bump_index(), 1u);

    arena.deallocate(p);

    EXPECT_EQ(arena.used_count(), 0u);
    EXPECT_EQ(arena.free_list_length(), 1u);
    EXPECT_EQ(arena.bump_index(), 1u)
        << "deallocate() must not rewind the bump pointer — the slot is recycled "
           "via the free list, not by decrementing the bump index";
    EXPECT_TRUE(arena.check_invariant());
}

// ─────────────────────────────────────────────────────────────────────────
// Test 4: The free list is LIFO and is consumed before fresh bump memory.
// Allocate one slot, free it, allocate again → must reuse the SAME address and
// leave the bump pointer unchanged.
// ─────────────────────────────────────────────────────────────────────────
TEST_F(ArenaTest, FreeListRecycling_ReusesSlotBeforeBumping) {
    Label* first = arena.allocate();
    arena.deallocate(first);

    Label* recycled = arena.allocate();

    EXPECT_EQ(recycled, first)
        << "allocate() must pop the free list before advancing the bump pointer";
    EXPECT_EQ(arena.bump_index(), 1u)
        << "Reusing a freed slot must not grow the bump pointer";
    EXPECT_EQ(arena.used_count(), 1u);

    arena.deallocate(recycled);
}

// ─────────────────────────────────────────────────────────────────────────
// Test 5: Many allocations are distinct; freeing all returns to a clean state.
// This is the core property the Pareto frontier relies on: every live Label is
// a distinct slot, and the count is exact.
// ─────────────────────────────────────────────────────────────────────────
TEST_F(ArenaTest, BulkAllocate_AllSlotsDistinct_ThenFreed) {
    constexpr std::size_t N = 1000;
    std::vector<Label*> ptrs;
    ptrs.reserve(N);
    std::set<Label*> unique;

    for (std::size_t i = 0; i < N; ++i) {
        Label* p = arena.allocate();
        ptrs.push_back(p);
        unique.insert(p);
    }

    EXPECT_EQ(unique.size(), N) << "Every allocation must return a distinct slot";
    EXPECT_EQ(arena.used_count(), N);
    EXPECT_EQ(arena.bump_index(), N);

    for (Label* p : ptrs) arena.deallocate(p);

    EXPECT_EQ(arena.used_count(), 0u);
    EXPECT_EQ(arena.free_list_length(), N)
        << "All freed slots must sit on the free list";
    EXPECT_TRUE(arena.check_invariant());
}

// ─────────────────────────────────────────────────────────────────────────
// Test 6: A free/realloc cycle keeps memory bounded.
// Free all N, then allocate N again. The bump pointer must NOT grow past N —
// the second wave is served entirely from the free list. This is exactly the
// behaviour that keeps a heavy routing query inside the fixed 1 MB arena.
// ─────────────────────────────────────────────────────────────────────────
TEST_F(ArenaTest, FreeThenReallocate_DoesNotGrowBumpPointer) {
    constexpr std::size_t N = 500;
    std::vector<Label*> wave1;
    for (std::size_t i = 0; i < N; ++i) wave1.push_back(arena.allocate());
    ASSERT_EQ(arena.bump_index(), N);

    for (Label* p : wave1) arena.deallocate(p);
    ASSERT_EQ(arena.free_list_length(), N);

    std::vector<Label*> wave2;
    for (std::size_t i = 0; i < N; ++i) wave2.push_back(arena.allocate());

    EXPECT_EQ(arena.bump_index(), N)
        << "Re-allocation after a full free must be served from the free list, "
           "not by advancing the bump pointer (memory must stay bounded)";
    EXPECT_EQ(arena.used_count(), N);
    EXPECT_EQ(arena.free_list_length(), 0u);

    for (Label* p : wave2) arena.deallocate(p);
}

// ─────────────────────────────────────────────────────────────────────────
// Test 7: reset() is an O(1) logical wipe — all counters return to zero.
// ─────────────────────────────────────────────────────────────────────────
TEST_F(ArenaTest, Reset_RestoresPristineState) {
    for (int i = 0; i < 50; ++i) (void)arena.allocate();
    ASSERT_GT(arena.used_count(), 0u);
    ASSERT_GT(arena.bump_index(), 0u);

    arena.reset();

    EXPECT_EQ(arena.used_count(), 0u);
    EXPECT_EQ(arena.bump_index(), 0u);
    EXPECT_EQ(arena.free_list_length(), 0u);
    EXPECT_TRUE(arena.check_invariant());

    // The arena is fully usable again after reset.
    Label* p = arena.allocate();
    EXPECT_EQ(arena.bump_index(), 1u);
    arena.deallocate(p);
}

// ─────────────────────────────────────────────────────────────────────────
// Test 8: The invariant holds through an interleaved alloc/free sequence.
// ─────────────────────────────────────────────────────────────────────────
TEST_F(ArenaTest, Invariant_HoldsUnderInterleavedOps) {
    std::vector<Label*> live;

    auto do_alloc = [&] { live.push_back(arena.allocate()); };
    auto do_free  = [&] {
        if (!live.empty()) {
            arena.deallocate(live.back());
            live.pop_back();
        }
    };

    // A deterministic but irregular pattern of allocations and frees.
    const char pattern[] = "AAAFAAFFAFAAAFFFAFA";
    for (char c : pattern) {
        if (c == 'A') do_alloc(); else do_free();
        ASSERT_TRUE(arena.check_invariant())
            << "Invariant broke mid-sequence at op '" << c << "'";
    }

    for (Label* p : live) arena.deallocate(p);
    EXPECT_EQ(arena.used_count(), 0u);
}

// ─────────────────────────────────────────────────────────────────────────
// Test 9: owns() distinguishes arena memory from foreign pointers.
// ─────────────────────────────────────────────────────────────────────────
TEST_F(ArenaTest, Owns_RejectsForeignPointers) {
    Label* p = arena.allocate();
    EXPECT_TRUE(arena.owns(p));

    Label stack_label{}; // not from this arena
    EXPECT_FALSE(arena.owns(&stack_label))
        << "A pointer outside the arena's storage must not be reported as owned";

    arena.deallocate(p);
}

// ─────────────────────────────────────────────────────────────────────────
// Test 10: prefault() touches pages without disturbing allocator state.
// ─────────────────────────────────────────────────────────────────────────
TEST_F(ArenaTest, Prefault_DoesNotDisturbState) {
    arena.prefault(); // must be safe to call on a fresh arena

    EXPECT_EQ(arena.used_count(), 0u);
    EXPECT_EQ(arena.bump_index(), 0u);
    EXPECT_TRUE(arena.check_invariant());

    // Still fully functional after pre-faulting.
    Label* p = arena.allocate();
    EXPECT_TRUE(arena.owns(p));
    arena.deallocate(p);
}

// ─────────────────────────────────────────────────────────────────────────
// Test 11: Exhausting a small arena throws std::bad_alloc (no silent overflow).
// Uses an explicit tiny capacity so the boundary is cheap to hit.
// ─────────────────────────────────────────────────────────────────────────
TEST(ArenaCapacityTest, Exhaustion_ThrowsBadAlloc) {
    ArenaAllocator<Label, 4> tiny;

    Label* a = tiny.allocate();
    Label* b = tiny.allocate();
    Label* c = tiny.allocate();
    Label* d = tiny.allocate();
    EXPECT_EQ(tiny.used_count(), 4u);
    EXPECT_EQ(tiny.bump_index(), 4u);

    EXPECT_THROW((void)tiny.allocate(), std::bad_alloc)
        << "Allocating past Capacity must throw std::bad_alloc, not overflow";

    // After freeing one slot, allocation succeeds again via the free list.
    tiny.deallocate(b);
    Label* e = tiny.allocate();
    EXPECT_EQ(e, b) << "The recovered allocation must reuse the freed slot";
    EXPECT_EQ(tiny.bump_index(), 4u);

    tiny.deallocate(a);
    tiny.deallocate(c);
    tiny.deallocate(d);
    tiny.deallocate(e);
    EXPECT_TRUE(tiny.check_invariant());
}
