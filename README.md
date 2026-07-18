# Namma Metro C++ Routing Engine

![CI](https://github.com/darshanrajagoli/namma-metro-router/actions/workflows/cpp-ci.yml/badge.svg)
![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)

> **This project models crowd-aware transit routing as a weighted-sum
> scalarization of a bi-criteria (travel-time vs. crowd-exposure) objective:**
> **min travel_time + λ·crowd**, where λ is a user-supplied crowd-aversion
> coefficient that sweeps out the trade-off between a fast route and an uncrowded
> one. This is *structurally analogous* to Markowitz mean-variance portfolio
> selection (λ playing the role of the risk-aversion knob) — but the analogy is
> **informal**: this objective is *linear* in the crowd term, not quadratic in a
> variance/covariance term, and no variance is computed. See `docs/write-up.tex` §5
> for the precise correspondence and its stated limits.

A cache-optimized, multi-label-correcting Pareto-Dijkstra routing engine in modern
C++17. Routing `Label` objects are served from a fixed-capacity arena (no per-label
heap allocation in the routing loop), and the graph is held in Compressed Sparse Row
format for cache-friendly traversal. It ingests GTFS transit feeds — the bundled
demo uses a synthetic feed; the `scripts/` normalizer and builder ingest real feeds
(e.g. the real Namma Metro topology with a modelled timetable — see **GTFS Data**).
Built to demonstrate systems-level C++ and applied optimization.

---

## Benchmark Results

> Benchmarks collected on isolated core 3 with taskset, frequency scaling
> disabled, Turbo Boost off, arena pre-faulted. Serialization via CPUID+RDTSC/RDTSCP+CPUID.
> Note: if benchmarked under WSL2, MSR frequency locking is unavailable (Hyper-V blocks
> WRMSR) — measurements may include Turbo Boost variance; annotate results accordingly.

| Metric | Latency |
|--------|---------|
| p50    | 511 ns  |
| p95    | 1183 ns |
| p99    | 1593 ns |

> These figures are from the bundled **10-node synthetic** GTFS feed (289 reachable
> query pairs, 10,000 queries) measured under WSL2, where MSR frequency locking is
> unavailable. They demonstrate the engine's order of magnitude; replace them with a
> real BMRCL-feed run on a frequency-locked core for headline numbers (see the
> "GTFS Data" section).

**Hardware:** Dell G15 5520 (Intel Core i7-12700H, 16GB DDR5), WSL2 Ubuntu 24.04,
GCC 13, `-O3 -march=native`.

---

## Mathematical Formulation

### Graph Model

Let **G = (V, E)** where:
- **V** = set of Namma Metro stops (nodes), |V| ≈ 60
- **E** = set of directed timetabled service legs; each edge *e = (u, v, t)* carries
  weight **w(u, v, t)** = seconds to traverse from *u* to *v* departing at time *t*

### FIFO Property

Time-dependent edge weights must satisfy the **First-In-First-Out** constraint:

```
t₁ + w(u, v, t₁) ≤ t₂ + w(u, v, t₂)   for all t₁ ≤ t₂
```

This guarantees that Dijkstra's optimality subpath property holds. Synthetic crowd
penalties violate FIFO when they drop precipitously after morning peak. The
**Bounded-Wait Lookahead** policy resolves this by enforcing the penalty derivative
constraint: `d/dt(penalty) ≥ -1`.

### Pareto Dominance

A label **(t, c)** dominates **(t', c')** iff `t ≤ t' ∧ c ≤ c'` with at least one
strict inequality. Each node keeps its non-dominated `(t, c)` frontier in a
sorted-vector label set with O(log k) binary-search insertion and O(1) amortized
forward pruning.

> **Scope note (important, and tested):** the engine expands **one** composite-optimal
> departure per link — the λ-minimizer among the next *k* departures — so the frontier
> it returns is the set of non-dominated trade-offs **across route choices** at a fixed
> λ, *not* the full frontier over every possible boarding on a multi-departure link.
> This is a deliberate, documented restriction; see `tests/test_pareto_oracle.cpp`
> (`MultiDeparture_*`) for the exact semantics and `docs/write-up.tex` for the theory.

### Markowitz Analogy (and its Limitations)

The scalarized objective `min E + λσ[crowd]` is isomorphic to Markowitz
mean-variance optimization. However, transit delay distributions are **right-skewed**
(bounded below by zero, unbounded above), so variance symmetrically penalizes both
early and late arrivals. **Semi-variance or Conditional Value at Risk (CVaR)** would
be economically superior risk proxies. See `docs/write-up.tex` §4 for a formal
discussion.

---

## Architecture

### Arena-Allocated Label Routing

Routing `Label` objects are served from a fixed-capacity `ArenaAllocator<Label>`
backed by a contiguous heap array; dominated labels are recycled through an O(1)
intrusive free list. **No `Label` object is heap-allocated during the routing loop.**
The priority queue and the destination-scratch buffer are `ParetoDijkstra` members
that retain their capacity across queries.

```
Label (routing-object) malloc / new during routing:            0   (arena-served)
Priority-queue / scratch-buffer allocations per query:         0   (members, capacity retained)
Per-node frontier-vector allocations per query (reused buffer): 0   (capacity retained across calls)
```

> **Allocation note.** The routing loop is allocation-free on the hot path. Use
> `run(src, dep, QueryResult& out)`: it fills `out` in place, and reusing the same
> `QueryResult` across queries keeps every per-node frontier vector's capacity, so after
> the first call no `std::vector` grows and no `operator new` runs inside the timed region
> (`Label` objects always come from the arena). The benchmark uses this overload. The
> convenience by-value overload `QueryResult run(src, dep)` allocates a fresh result per
> call and is intended for one-off queries, not the hot path.

### Compressed Sparse Row Graph

```
edge_data[offset[u] .. offset[u+1])  =  outgoing edges of node u
```

All edges of a node are contiguous in memory, enabling sequential cache-line
prefetching during Dijkstra relaxation. At ~500 edges × 20 bytes, the full
Namma Metro graph occupies **~10 KB**, fitting entirely in the 48 KB L1d cache.

### Priority Queue Memory Bound

The `ParetoDijkstra` min-heap holds at most one entry per label ever pushed. The
theoretical worst-case bound is:

```
max_heap_entries ≤ |V| × k × D
```

where `|V|` is the number of nodes (~60–100 for BMRCL), `k` is the maximum number
of non-dominated Pareto labels retained per node (bounded by arena capacity ÷ |V|,
practically ≤ 16), and `D` is the maximum number of distinct departure times per
origin node across the timetable (≤ ~200 for a full-day BMRCL schedule).

For BMRCL parameters: 100 × 16 × 200 = **320,000 heap entries** worst-case.
Each `Label` is 16 bytes (4 × uint32_t), so worst-case heap memory ≈ **5 MB** —
well within WSL2's default 8 GB memory assignment. In practice the heap stays
under 1,000 entries for typical peak-hour queries because lazy deletion prunes
stale labels before they propagate.

This bound is important when scaling to larger networks (e.g., full Indian Railways
GTFS with ~8,000 stations): the k × D factor must be re-evaluated and the arena
capacity adjusted upward accordingly.

### Edge Memory Layout (20 bytes, zero padding)

```cpp
struct Edge {                 // exactly 20 bytes, static_assert enforced
    uint32_t destination;     // 4B
    uint32_t departure_time;  // 4B
    uint32_t travel_time;     // 4B
    uint32_t crowd_weight;    // 4B
    uint32_t penalty;         // 4B
};
```

---

## Project Structure

```
namma-metro-router/
├── include/
│   ├── gtfs_parser.hpp      # GTFS structs + FK-validated parser
│   ├── graph.hpp            # CSR graph + Edge struct (20B)
│   ├── arena_allocator.hpp  # Arena-backed free-list allocator
│   ├── routing.hpp          # Pareto-Dijkstra 
│   └── benchmark.hpp        # CPUID+RDTSC cycle-accurate timing
├── src/
│   ├── gtfs_parser.cpp      # GTFS ingestion + geometric interpolation
│   ├── graph_builder.cpp    # CSR construction from stop-time records
│   ├── routing.cpp          # Dijkstra engine  
│   ├── benchmark.cpp        # TSC calibration + percentile computation
│   └── main_bench.cpp       # Benchmark harness entry point
├── tests/                   # Google Test suite (8 files, 65 tests)
├── scripts/
│   ├── stabilize_cpu.sh     # Governor=performance, disable Turbo, BD PROCHOT
│   └── enable_gmode.sh      # Dell G-Mode maximum fan control
└── docs/write-up.tex        # Formal mathematical write-up (FIFO, dominance, Markowitz analogy)
```

---

## Build & Run

### Prerequisites (WSL2 Ubuntu 24.04)

```bash
# Keep repository on the native ext4 filesystem (NOT /mnt/c/) for 3–5× faster builds
cd ~/projects   # NOT /mnt/c/Users/...

# Install toolchain
sudo apt-get update
sudo apt-get install -y cmake ninja-build gcc-13 g++-13 \
    linux-tools-common linux-tools-generic msr-tools \
    acpi-call-dkms texlive-full

# Clone
git clone https://github.com/darshanrajagoli/namma-metro-router.git
cd namma-metro-router
```

### Build

```bash
# Release (benchmarking)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -G Ninja
cmake --build build --parallel

# Debug with ASan (development)
cmake -S . -B build_debug -DCMAKE_BUILD_TYPE=Debug -G Ninja
cmake --build build_debug --parallel
```

### Test

```bash
cmake --build build_debug --target namma_metro_tests
cd build_debug && ctest --output-on-failure
```

### Benchmark

```bash
# 1. Stabilize CPU (once per session)
sudo bash scripts/stabilize_cpu.sh

# 2. Optional: maximum fan speed during sustained benchmarking
sudo bash scripts/enable_gmode.sh enable

# 3. Run
taskset -c 3 ./build/routing_engine_benchmark ./data
```

### GTFS Data

Download from [India Urban Data Exchange (IUDX)](https://iudx.org.in/) or
[BMRCL open data portal](https://english.bmrc.co.in/). Place files in `data/`:

```
data/
├── agency.txt
├── stops.txt
├── routes.txt
├── trips.txt
└── stop_times.txt
```

---
## Core Components

The correctness-critical routines of the engine, each covered by dedicated unit tests:

| Component | File | Mathematical Concept |
|-----------|------|---------------------|
| `select_optimal_departure()` | `src/routing.cpp` | Bounded-Wait Lookahead + FIFO preservation |
| `ParetoLabelSet::insert_and_dominate()` | `src/routing.cpp` | Binary search + dominance pruning |
| Lazy deletion filter in Dijkstra loop | `src/routing.cpp` | Stale label suppression |

Run `ctest` from the build directory — all 65 tests pass under AddressSanitizer +
UndefinedBehaviorSanitizer.

---

## License

MIT — see LICENSE.
