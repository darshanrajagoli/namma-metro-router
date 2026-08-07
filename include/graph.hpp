#pragma once

#include <cstdint>
#include <vector>
#include <cassert>
#include <string>
#include <unordered_map>

/**
 * @file graph.hpp
 * @brief Compressed Sparse Row (CSR) graph for time-dependent transit routing.
 *
 * ARCHITECTURAL INVARIANT:
 *   The graph MUST be stored in CSR format. Never use std::vector<std::vector<Edge>>.
 *   Nested dynamic arrays cause pointer-chasing that destroys L1 cache coherency.
 *   On the Dell G15 5535 used here (AMD Ryzen 5 7640HS, Zen 4): L1d = 32 KB per
 *   core, L2 = 1 MB per core, L3 = 16 MB shared across 6 cores.
 *   With CSR, all outgoing edges of a node are packed contiguously:
 *     edges of node u = edge_data[ offset[u] .. offset[u+1] )
 *   This guarantees sequential prefetching during Dijkstra relaxation.
 *
 * Memory layout:
 *   offset[]    : (num_nodes + 1) uint32_t values  → 4*(N+1) bytes
 *   edge_data[] : num_edges * sizeof(Edge)          → 20*E bytes
 *
 * Cache math (measured Namma Metro feed — Purple, Green and Yellow lines):
 *   The CSR stores one edge per TIMETABLED DEPARTURE, not one per track segment, so
 *   |E| is driven by service frequency rather than station count. 82 stations at a
 *   5-minute headway over an 05:00-23:00 service day yield 35,588 edges:
 *     35,588 edges × 20 bytes = 711,760 bytes ≈ 695 KB.
 *   That does NOT fit the 32 KB L1d, but it is resident in the 1 MB L2 of a Zen 4
 *   core, and each node's range is traversed sequentially so the
 *   hardware prefetcher stays engaged. Only the working set of a single query — the
 *   visited nodes' frontier vectors and the arena slots in use — stays L1-resident.
 */

namespace namma_metro {

// ═══════════════════════════════════════════════════════════════════════════
// § 1.  Edge struct — exactly 20 bytes, zero padding, largest-field-first
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief A directed, time-stamped, bi-criteria edge in the transit graph.
 *
 * Field ordering: largest primitive first to guarantee
 * zero implicit padding. All fields are uint32_t (4 bytes each); total = 20 bytes.
 *
 * Interpretation:
 *   An edge (u → destination) is traversable if boarding at u at time t
 *   with departure_time ≥ t. The passenger arrives at destination at time
 *   departure_time + travel_time.
 *
 * Bi-criteria objective:
 *   Minimize total travel_time and minimize cumulative secondary_weight + penalty.
 *   The Pareto-Dijkstra algorithm maintains the full non-dominated frontier.
 *
 * FIFO constraint:
 *   The penalized weight function w'(u,v,t) = travel_time + penalty(u,v,t)
 *   must satisfy d/dt(penalty) ≥ -1 to preserve FIFO. This is enforced by
 *   the Bounded-Wait Lookahead (see routing.hpp).
 */
struct Edge {
    uint32_t destination;    ///< Node index of the head vertex in the CSR graph.
                             ///< Maps to StopRecord dense index via stop_index_map.

    uint32_t departure_time; ///< Absolute seconds-past-midnight at which this
                             ///< service departs node u (source of this edge).
                             ///< Boarding requires current_time ≤ departure_time.

    uint32_t travel_time;    ///< Journey duration in seconds from u to destination.
                             ///< Arrival = departure_time + travel_time.

    uint32_t secondary_weight; ///< Weight contributed to the SECOND Pareto objective.
                             ///< Its meaning is chosen at graph-build time via
                             ///< SecondObjective:
                             ///<   CrowdExposure — synthetic crowd density in [0,1000]
                             ///<                   (0 = empty carriage, 1000 = crush load)
                             ///<   TransferCount — always 0 on a service edge; only
                             ///<                   TransferEdge contributes.

    uint32_t penalty;        ///< Reserved for a time-dependent wait surcharge in the
                             ///< composite objective. Currently always 0 — see the
                             ///< note in graph_builder.cpp and "Known Limitations"
                             ///< in README.md.
};

// Compile-time memory layout enforcement
static_assert(sizeof(Edge) == 20,
    "Edge struct must be exactly 20 bytes. Check field ordering for padding.");
static_assert(alignof(Edge) == 4,
    "Edge struct alignment must be 4 bytes for CSR array packing.");

/**
 * @brief A walk between two platforms of the same physical station.
 *
 * Transfer edges are stored in their OWN CSR adjacency, deliberately kept apart
 * from `edge_data`:
 *
 *   - Service edges are sorted by `departure_time` and searched with
 *     std::lower_bound. A transfer has no departure time — it is available
 *     whenever the passenger arrives — so folding it into that array would
 *     require a sentinel time, and any sentinel can collide with a real
 *     midnight (00:00:00 == 0) departure. A separate array has no such
 *     ambiguity.
 *   - It also leaves the service-edge hot path byte-for-byte unchanged.
 *
 * FIFO: `travel_time` here is a constant independent of arrival time, so
 * t1 <= t2 implies t1 + w <= t2 + w. Transfer edges satisfy FIFO trivially and
 * need no Bounded-Wait treatment.
 */
struct TransferEdge {
    uint32_t destination;  ///< Node index of the platform being walked to.
    uint32_t travel_time;  ///< Minimum transfer time in seconds (GTFS
                           ///< transfers.txt min_transfer_time, or the
                           ///< normaliser's default when the feed omits the pair).
};

static_assert(sizeof(TransferEdge) == 8,
    "TransferEdge must be exactly 8 bytes.");

/// What the second Pareto objective measures. Fixed when the graph is built.
enum class SecondObjective : uint8_t {
    CrowdExposure, ///< Accumulate Edge::secondary_weight from the crowd model.
    TransferCount  ///< Accumulate 1 per transfer edge; service edges contribute 0.
};

// ═══════════════════════════════════════════════════════════════════════════
// § 2.  CSRGraph — the complete graph representation
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Immutable Compressed Sparse Row transit graph.
 *
 * Construction (done once at startup by GraphBuilder):
 *   1. Sort all StopTimeRecord entries by trip_id, then stop_sequence.
 *   2. For consecutive stops (i, i+1) within a trip, create directed Edge.
 *   3. Group edges by source node index.
 *   4. Compute prefix-sum to build offset[].
 *   5. Fill edge_data[] in CSR order.
 *
 * Query (repeated at routing time — must be allocation-free):
 *   for (uint32_t i = offset[u]; i < offset[u+1]; ++i) {
 *       const Edge& e = edge_data[i];  // L1 cache hit guaranteed
 *       // relax e ...
 *   }
 *
 * Thread safety: read-only after construction → fully thread-safe.
 */
struct CSRGraph {
    uint32_t num_nodes = 0;    ///< Total node count (== num unique stops)
    uint32_t num_edges = 0;    ///< Total directed edge count

    /// CSR adjacency data. edge_data[offset[u]..offset[u+1]) = outgoing edges of u.
    std::vector<Edge>     edge_data;

    /// CSR row pointer. Length = num_nodes + 1. offset[num_nodes] == num_edges.
    std::vector<uint32_t> offset;

    uint32_t num_transfers = 0; ///< Total directed transfer-edge count.

    /// Which objective `secondary_weight` / TransferEdge feed. Set by GraphBuilder.
    SecondObjective second_objective = SecondObjective::CrowdExposure;

    /// Separate CSR adjacency for platform-to-platform transfers.
    /// transfer_data[transfer_offset[u]..transfer_offset[u+1]) = transfers out of u.
    /// Empty (and transfer_offset all-zero) on feeds built without transfers.
    std::vector<TransferEdge> transfer_data;
    std::vector<uint32_t>     transfer_offset;

    // ── Accessors ──────────────────────────────────────────────────────────

    /**
     * @brief Return the contiguous range of outgoing edges for node @p u.
     * @return (begin_ptr, end_ptr) pair into edge_data. Caller must not store
     *         these pointers beyond the lifetime of this CSRGraph.
     */
    [[nodiscard]] std::pair<const Edge*, const Edge*> edges_of(uint32_t u) const noexcept {
        assert(u < num_nodes);
        return {
            edge_data.data() + offset[u],
            edge_data.data() + offset[u + 1]
        };
    }

    [[nodiscard]] uint32_t out_degree(uint32_t u) const noexcept {
        return offset[u + 1] - offset[u];
    }

    /**
     * @brief Return the contiguous range of outgoing transfer edges for node @p u.
     *
     * Returns an empty range when the graph was built without transfers, so
     * callers need no special-casing.
     */
    [[nodiscard]] std::pair<const TransferEdge*, const TransferEdge*>
    transfers_of(uint32_t u) const noexcept {
        assert(u < num_nodes);
        if (transfer_offset.empty())
            return {nullptr, nullptr};
        return {
            transfer_data.data() + transfer_offset[u],
            transfer_data.data() + transfer_offset[u + 1]
        };
    }

    [[nodiscard]] bool has_transfers() const noexcept { return num_transfers > 0; }

    [[nodiscard]] bool empty() const noexcept { return num_nodes == 0; }

    /// Memory footprint in bytes (useful for cache-fit verification)
    [[nodiscard]] size_t memory_bytes() const noexcept {
        return sizeof(uint32_t)     * offset.size()
             + sizeof(Edge)         * edge_data.size()
             + sizeof(uint32_t)     * transfer_offset.size()
             + sizeof(TransferEdge) * transfer_data.size();
    }
};

// ═══════════════════════════════════════════════════════════════════════════
// § 3.  GraphBuilder — constructs CSRGraph from parsed GTFS data
// ═══════════════════════════════════════════════════════════════════════════

// Forward declarations
struct StopTimeRecord;
struct StopRecord;
struct TransferRecord;

/**
 * @brief Constructs a CSRGraph from sanitized GTFS stop-time data.
 *
 * This is intentionally separated from CSRGraph itself: immutability of
 * the graph after construction is a hard invariant.
 *
 * Usage:
 *   GTFSParser parser("./data");
 *   parser.load_agency(); parser.load_stops(); // ...
 *   CSRGraph g = GraphBuilder::build(parser.stop_times(), parser.stops().size());
 */
class GraphBuilder {
public:
    /**
     * @brief Build CSRGraph from sanitized GTFS stop-time records.
     *
     * @param stop_times     Fully interpolated, FK-validated stop time records.
     * @param num_stops      Total number of unique stops (= num_nodes in result).
     * @param stop_index_map Optional stop_id → dense uint32_t index mapping from
     *                       GTFSParser::stop_index_map(). When provided, this
     *                       mapping is used exclusively — GraphBuilder does NOT
     *                       rebuild its own local index, which would diverge from
     *                       the parser's ordering and silently corrupt all queries
     *                       that use stop IDs from the parser as source/destination.
     *                       Pass nullptr to use the fallback local index (CI/tests only).
     * @return               Immutable CSRGraph ready for query execution.
     *
     * @pre   No UINT32_MAX sentinel values remain in stop_times (interpolation done).
     * @post  result.offset[result.num_nodes] == result.num_edges.
     * @post  For all u: result.edge_data[offset[u]..offset[u+1]) sorted by departure_time.
     */
    static CSRGraph build(
        const std::vector<StopTimeRecord>& stop_times,
        uint32_t num_stops,
        const std::unordered_map<std::string, uint32_t>* stop_index_map = nullptr
    );

    /**
     * @brief Build a CSRGraph with a transfer layer and a chosen second objective.
     *
     * @param stop_times     As above.
     * @param num_stops      As above.
     * @param stop_index_map As above. REQUIRED (non-null) when @p transfers is
     *                       non-empty: transfer records name stops by id, and
     *                       resolving them against a locally-rebuilt index that
     *                       disagrees with the parser's would silently wire
     *                       transfers between the wrong platforms.
     * @param transfers      Platform-to-platform transfer records. Records naming
     *                       an unknown stop are skipped and counted, not fatal.
     * @param objective      What the second Pareto dimension measures. Under
     *                       TransferCount every service edge gets
     *                       secondary_weight = 0 and each transfer contributes 1.
     *
     * @post result.transfer_offset.size() == num_nodes + 1 when transfers exist.
     * @post result.transfer_offset[num_nodes] == result.num_transfers.
     */
    static CSRGraph build_with_transfers(
        const std::vector<StopTimeRecord>& stop_times,
        uint32_t num_stops,
        const std::unordered_map<std::string, uint32_t>* stop_index_map,
        const std::vector<TransferRecord>& transfers,
        SecondObjective objective
    );

private:
    GraphBuilder() = delete;
};

} // namespace namma_metro
