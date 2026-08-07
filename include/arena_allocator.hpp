#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cassert>
#include <stdexcept>
#include <array>
#ifdef NAMMA_BASELINE_HEAP_ALLOC
#include <vector>   // live-pointer tracking, baseline builds only
#endif

/**
 * @file arena_allocator.hpp
 * @brief Arena-backed free-list allocator for routing Label objects.
 *
 * ╔══════════════════════════════════════════════════════════════╗
 * ║  WHY AN ARENA ALLOCATOR — determinism over the OS heap     ║
 * ╠══════════════════════════════════════════════════════════════╣
 * ║  The general-purpose heap (malloc/new) must serve any size,  ║
 * ║  track free blocks, resist fragmentation and stay thread-    ║
 * ║  safe. That generality costs time, and occasionally costs    ║
 * ║  a page fault or a syscall — so its tail is unpredictable.   ║
 * ║  It is NOT slow in the common case: glibc's tcache serves a  ║
 * ║  16-byte request in a few nanoseconds, which is exactly why  ║
 * ║  the measurement below comes out negative on one workload.   ║
 * ║                                                              ║
 * ║  MEASURED, not assumed: on the workload where the Pareto     ║
 * ║  frontier genuinely branches (BART + transfers, peak 280     ║
 * ║  labels/query) the arena beats new/delete by ~31% at p50.    ║
 * ║  On the workload where it does not (Namma, 82 labels per     ║
 * ║  query) it is ~5% SLOWER than glibc's tcache. The arena's    ║
 * ║  payoff scales with allocation volume; the optimisation      ║
 * ║  only pays once the algorithm it supports does real work.    ║
 * ║  See tools/ab.py and the README's Measured Behaviour.        ║
 * ║                                                              ║
 * ║  This arena:                                                 ║
 * ║    • Pre-allocates all memory at startup (O(1) amortized)   ║
 * ║    • Free-list recycles dominated Label slots instantly      ║
 * ║    • Gives O(1) allocate and O(1) deallocate                ║
 * ║    • Keeps Label allocation off the OS heap in routing  ║
 * ║                                                              ║
 * ║  A standard technique in latency-sensitive systems code.     ║
 * ╚══════════════════════════════════════════════════════════════╝
 *
 * Architecture:
 *
 *   ┌──────────────────────────────────────────────────────┐
 *   │                    arena_[CAPACITY]                   │
 *   │  ╔════╦════╦════╦════╦════╦════╦════╦════╦════╗      │
 *   │  ║ L0 ║ L1 ║ L2 ║ L3 ║ L4 ║ L5 ║ L6 ║ L7 ║...║      │
 *   │  ╚════╩════╩════╩════╩════╩════╩════╩════╩════╝      │
 *   │         ↑                   ↑                          │
 *   │     bump_ptr_           free_list_head_               │
 *   │     (next fresh)       (singly-linked, O(1) pop)      │
 *   └──────────────────────────────────────────────────────┘
 *
 * On allocate():
 *   1. If free_list non-empty: pop head → O(1), zero the slot, return.
 *   2. Else: advance bump_ptr_ → O(1), return.
 *   3. If bump_ptr_ == capacity: throw (arena exhausted — increase capacity).
 *
 * On deallocate(ptr):
 *   1. Reinterpret the slot as a FreeNode (stores next pointer in first 8 bytes).
 *   2. Push to head of free_list → O(1).
 *
 * Invariant (checked by check_invariant(), in SLOT COUNTS not bytes):
 *   used_count_ + free_list_length() == bump_ptr_
 * i.e. every slot ever handed out is either live or sitting on the free list.
 */

/**
 * NAMMA_BASELINE_HEAP_ALLOC — opt-in A/B baseline (never defined in normal builds)
 * ------------------------------------------------------------------------------
 * Defining this macro swaps the arena's three hot methods for the allocation
 * strategy a straightforward implementation would use: one `operator new` per
 * Label, freed in bulk between queries. It exists so the arena's benefit can be
 * *measured* rather than asserted — see the `routing_engine_baseline` target in
 * CMakeLists.txt and the A/B table in README.md.
 *
 * The comparison is deliberately generous to the baseline:
 *   - deallocate() is a NO-OP under this macro, so the baseline is charged for
 *     allocation but never for reclaiming dominated labels. The arena pays that
 *     cost (free-list push) on every dominated label and still wins.
 *   - The live-pointer vector that makes bulk reclamation possible retains its
 *     capacity across resets, so after warm-up each record is a pointer store,
 *     not an allocation.
 * The measured gap is therefore a LOWER bound on what the arena is worth.
 *
 * The macro is never set for the shipping binary, the test suite, or CI, so the
 * optimized code path carries no runtime branch for it.
 */

namespace namma_metro {

/// Default arena capacity: 65536 Label objects (~1 MB for the 16-byte Label).
/// Sizing rule: capacity >= |V| * k_max, where k_max is the worst-case Pareto
/// frontier size per node. Measured peak use is 280 live labels in a single query
/// (BART, 103 platforms, transfer objective) and 82 on the Namma feed — roughly
/// 230x headroom. Deliberately oversized: exhaustion throws rather than corrupting.
/// Increase to 1<<20 for exhaustive stress tests or a network orders of magnitude
/// larger (national rail at |V| ~ 8,000 would need 128,000 slots).
static constexpr std::size_t ARENA_DEFAULT_CAPACITY = 1u << 16; // 65536

/**
 * @brief A typed, fixed-capacity arena allocator with O(1) free-list recycling.
 *
 * @tparam T         The object type to allocate (must be trivially destructible
 *                   or the caller is responsible for explicit destructor calls).
 * @tparam Capacity  Maximum number of T objects the arena can hold.
 *
 * Thread safety: NOT thread-safe. One arena per thread in parallel workloads.
 */
template <typename T, std::size_t Capacity = ARENA_DEFAULT_CAPACITY>
class ArenaAllocator {
public:
    static_assert(sizeof(T) >= sizeof(void*),
        "T must be large enough to store a free-list next pointer (>= 8 bytes on x64).");

    // ── Construction ──────────────────────────────────────────────────────

    ArenaAllocator() noexcept : bump_ptr_(0), free_list_head_(nullptr), used_count_(0) {
        // Zero-initialize the arena. This also pre-faults pages if the arena
        // is a local variable; for heap-allocated arenas, call prefault().
        std::memset(arena_.data(), 0, sizeof(arena_));
    }

#ifdef NAMMA_BASELINE_HEAP_ALLOC
    /// Baseline builds only. reset() frees the previous query's labels, so without
    /// this the final query's labels would still be outstanding at teardown and
    /// LeakSanitizer would flag them. The real arena needs no destructor: its
    /// storage is a member array.
    ~ArenaAllocator() { for (T* p : live_) delete p; }
#endif

    // Non-copyable, non-movable (fixed address required for free-list pointers)
    ArenaAllocator(const ArenaAllocator&) = delete;
    ArenaAllocator& operator=(const ArenaAllocator&) = delete;
    ArenaAllocator(ArenaAllocator&&) = delete;
    ArenaAllocator& operator=(ArenaAllocator&&) = delete;

    // ── Core API ──────────────────────────────────────────────────────────

    /**
     * @brief Allocate one T-sized slot from the arena. O(1).
     *
     * Priority:
     *   1. Recycle from free-list head (reclaims dominated Label memory).
     *   2. Advance bump pointer into fresh arena memory.
     *
     * The returned pointer is zeroed before being returned.
     * The caller is responsible for constructing T via placement new:
     *   T* p = arena.allocate();
     *   new(p) T{...};
     *
     * @return  Pointer to an uninitialized T-sized slot. Never null.
     * @throws  std::bad_alloc if arena is exhausted.
     */
    [[nodiscard]] T* allocate() {
#ifdef NAMMA_BASELINE_HEAP_ALLOC
        // Baseline: straight to the OS allocator, one call per Label.
        T* slot = new T();          // value-initialized, matching the memset below
        live_.push_back(slot);      // amortized O(1); capacity retained across resets
        ++used_count_;
        return slot;
#else
        T* slot = nullptr;

        if (free_list_head_ != nullptr) {
            // ── Path 1: recycle from free list ────────────────────────────
            slot = reinterpret_cast<T*>(free_list_head_);
            free_list_head_ = free_list_head_->next;
        } else {
            // ── Path 2: bump allocator ────────────────────────────────────
            if (bump_ptr_ >= Capacity) {
                throw std::bad_alloc();
            }
            slot = &arena_[bump_ptr_++];
        }

        std::memset(slot, 0, sizeof(T));
        ++used_count_;
        return slot;
#endif
    }

    /**
     * @brief Return a slot to the arena's free list. O(1).
     *
     * The slot's memory is reused to store the free-list linkage pointer.
     * The caller must NOT use @p ptr after calling deallocate().
     *
     * @param ptr  Pointer previously returned by allocate(). Must not be null.
     *             Must point into this arena's internal storage.
     */
    void deallocate(T* ptr) noexcept {
        assert(ptr != nullptr);
#ifdef NAMMA_BASELINE_HEAP_ALLOC
        // Intentionally a no-op: dominated labels are reclaimed in bulk by reset()
        // at the start of the next query. This under-charges the baseline on
        // purpose — see the macro's note at the top of this file.
        (void)ptr;
#else
        assert(owns(ptr) && "deallocate(): pointer not owned by this arena");

        // Reinterpret the memory as a FreeNode to store the linkage.
        FreeNode* node = reinterpret_cast<FreeNode*>(ptr);
        node->next = free_list_head_;
        free_list_head_ = node;
        --used_count_;
#endif
    }

    // ── Reset ─────────────────────────────────────────────────────────────

    /**
     * @brief Reset the entire arena in O(1) (logical reset only).
     *
     * Does NOT zero memory. After reset(), all previously allocated pointers
     * are invalidated. Suitable for clearing between independent routing queries.
     */
    void reset() noexcept {
#ifdef NAMMA_BASELINE_HEAP_ALLOC
        // Baseline: O(n) teardown, one operator delete per Label allocated during
        // the previous query. This is the cost the arena's O(1) reset replaces.
        for (T* p : live_) delete p;
        live_.clear();              // retains capacity for the next query
        used_count_ = 0;
#else
        bump_ptr_       = 0;
        free_list_head_ = nullptr;
        used_count_     = 0;
#endif
    }

    // ── Memory pre-faulting ───────────────────────────────────────────────

    /**
     * @brief Force Linux kernel to map all arena virtual pages to physical RAM.
     *
     * Without pre-faulting, the first access to each 4 KB page generates a
     * soft page fault, causing an OS interrupt. On a 2 MB arena this produces
     * ~512 page faults, inflating p99 latency measurements by orders of
     * magnitude. Call prefault() once, before executing any warmup queries.
     *
     * Mechanism: writes a zero byte to every 4096th byte, touching each page.
     * This is the canonical technique used in HFT system initialization.
     */
    void prefault() noexcept {
#ifdef NAMMA_BASELINE_HEAP_ALLOC
        // Nothing to pre-fault: the baseline has no pre-allocated slab, which is
        // precisely one of the costs being measured — its pages are faulted in
        // on demand, inside the timed region.
        return;
#else
        constexpr std::size_t PAGE_SIZE = 4096;
        volatile uint8_t* raw = reinterpret_cast<volatile uint8_t*>(arena_.data());
        for (std::size_t i = 0; i < sizeof(arena_); i += PAGE_SIZE) {
            raw[i] = 0;
        }
#endif
    }

    // ── Diagnostics ───────────────────────────────────────────────────────

    [[nodiscard]] std::size_t capacity()    const noexcept { return Capacity; }
    [[nodiscard]] std::size_t used_count()  const noexcept { return used_count_; }
    [[nodiscard]] std::size_t bump_index()  const noexcept { return bump_ptr_; }

    [[nodiscard]] std::size_t free_list_length() const noexcept {
        std::size_t count = 0;
        const FreeNode* n = free_list_head_;
        while (n) { ++count; n = n->next; }
        return count;
    }

    /**
     * @brief Verify allocator invariant: used + free == bump_ptr.
     * @return true if the invariant holds.
     */
    [[nodiscard]] bool check_invariant() const noexcept {
        return (used_count_ + free_list_length()) == bump_ptr_;
    }

    /**
     * @brief Returns true if @p ptr was allocated from this arena.
     */
    [[nodiscard]] bool owns(const T* ptr) const noexcept {
        return ptr >= arena_.data() && ptr < arena_.data() + Capacity;
    }

private:
    /// Singly-linked free list node (reuses T's memory slot).
    struct FreeNode {
        FreeNode* next; ///< Next free slot, or nullptr if tail.
    };
    static_assert(sizeof(FreeNode) <= sizeof(T),
        "T must be large enough to store a FreeNode (pointer).");

    alignas(64) std::array<T, Capacity> arena_; ///< 64-byte aligned for AVX2 friendliness
    std::size_t bump_ptr_;       ///< Index of next fresh slot in arena_
    FreeNode*   free_list_head_; ///< Head of the recycled-slot free list
    std::size_t used_count_;     ///< Currently live (not freed) allocations

#ifdef NAMMA_BASELINE_HEAP_ALLOC
    /// Baseline builds only: pointers handed out since the last reset(), so they
    /// can be freed in bulk. Absent from the shipping binary entirely.
    std::vector<T*> live_;
#endif
};

} // namespace namma_metro
