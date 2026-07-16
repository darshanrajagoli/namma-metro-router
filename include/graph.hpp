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
 * ARCHITECTURAL MANDATE (CLAUDE.md §4):
 *   The graph MUST be stored in CSR format. Never use std::vector<std::vector<Edge>>.
 *   Nested dynamic arrays cause pointer-chasing that destroys L1 cache coherency.
 *   On a Dell G15 with a 12th-gen Intel CPU, L1d = 48 KB per core, L2 = 1.25 MB.
 *   With CSR, all outgoing edges of a node are packed contiguously:
 *     edges of node u = edge_data[ offset[u] .. offset[u+1] )
 *   This guarantees sequential prefetching during Dijkstra relaxation.
 *
 * Memory layout:
 *   offset[]    : (num_nodes + 1) uint32_t values  → 4*(N+1) bytes
 *   edge_data[] : num_edges * sizeof(Edge)          → 20*E bytes
 *
 * Cache math (for Namma Metro Purple+Green lines, ~500 edges):
 *   500 edges × 20 bytes = 10,000 bytes = ~10 KB → fits entirely in L1d.
 */

namespace namma_metro {

// ═══════════════════════════════════════════════════════════════════════════
// § 1.  Edge struct — exactly 20 bytes, zero padding, largest-field-first
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief A directed, time-stamped, bi-criteria edge in the transit graph.
 *
 * Field ordering follows CLAUDE.md §5: largest primitive first to guarantee
 * zero implicit padding. All fields are uint32_t (4 bytes each); total = 20 bytes.
 *
 * Interpretation:
 *   An edge (u → destination) is traversable if boarding at u at time t
 *   with departure_time ≥ t. The passenger arrives at destination at time
 *   departure_time + travel_time.
 *
 * Bi-criteria objective:
 *   Minimize total travel_time and minimize cumulative crowd_weight + penalty.
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

    uint32_t crowd_weight;   ///< Synthetic crowd density score in [0, 1000].
                             ///< 0 = empty carriage; 1000 = crush load.
                             ///< Source: IUDX real-time AFC gate data or synthetic
                             ///< Gaussian model parameterised on Namma Metro ridership.

    uint32_t penalty;        ///< Bounded-Wait Lookahead penalty injected by
                             ///< select_optimal_departure() in routing.hpp.
                             ///< Encodes waiting cost in the composite objective.
                             ///< 0 = board immediately (ideal FIFO case).
};

// Compile-time memory layout enforcement (CLAUDE.md §4)
static_assert(sizeof(Edge) == 20,
    "Edge struct must be exactly 20 bytes. Check field ordering for padding.");
static_assert(alignof(Edge) == 4,
    "Edge struct alignment must be 4 bytes for CSR array packing.");

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
 * Query (repeated at routing time — must be allocation-free per CLAUDE.md §3):
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

    [[nodiscard]] bool empty() const noexcept { return num_nodes == 0; }

    /// Memory footprint in bytes (useful for cache-fit verification)
    [[nodiscard]] size_t memory_bytes() const noexcept {
        return sizeof(uint32_t) * offset.size()
             + sizeof(Edge)     * edge_data.size();
    }
};

// ═══════════════════════════════════════════════════════════════════════════
// § 3.  GraphBuilder — constructs CSRGraph from parsed GTFS data
// ═══════════════════════════════════════════════════════════════════════════

// Forward declarations
struct StopTimeRecord;
struct StopRecord;

/**
 * @brief Constructs a CSRGraph from sanitized GTFS stop-time data.
 *
 * This is intentionally separated from CSRGraph itself: immutability of
 * the graph after construction is a hard invariant (CLAUDE.md §3).
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

private:
    GraphBuilder() = delete;
};

} // namespace namma_metro
