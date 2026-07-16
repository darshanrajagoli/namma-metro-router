# Namma Metro C++ Routing Engine

![CI](https://github.com/darshanrajagoli/namma-metro-router/actions/workflows/cpp-ci.yml/badge.svg)
![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)

> **This project formulates time-dependent transit routing as a mean-variance
> optimization problem analogous to Markowitz portfolio theory.** The bi-criteria
> objective — minimizing both travel time and crowd exposure — is scalarized as
> **min E[travel_time] + λ·σ[crowd_weight]**, where λ is a user-supplied
> risk-aversion coefficient. This shares the identical mathematical structure as
> an investor minimizing variance for a given expected return, where λ serves as
> the risk-aversion coefficient along the efficient frontier.

A production-grade, zero-allocation, cache-optimized multi-label
correcting Pareto-Dijkstra routing engine operating on real Namma Metro (BMRCL)
GTFS data. Built to demonstrate systems-level C++ and applied mathematical
optimization relevant to quantitative finance infrastructure.

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

A label **(t, c)** dominates **(t', c')** if and only if `t ≤ t' ∧ c ≤ c'`.
The engine maintains the full non-dominated Pareto frontier per node via a
sorted-vector label set with O(log k) binary-search insertion and O(1) amortized
forward pruning.

### Markowitz Analogy (and its Limitations)

The scalarized objective `min E + λσ[crowd]` is isomorphic to Markowitz
mean-variance optimization. However, transit delay distributions are **right-skewed**
(bounded below by zero, unbounded above), so variance symmetrically penalizes both
early and late arrivals. **Semi-variance or Conditional Value at Risk (CVaR)** would
be economically superior risk proxies. See `docs/write-up.tex` §4 for a formal
discussion.

---

## Architecture

### Zero-Allocation Label Routing

All `Label` objects are allocated from a fixed-capacity `ArenaAllocator<Label>`
backed by a contiguous heap array. Dominated labels are recycled via an O(1)
intrusive free list. Working buffers (`pareto_sets`, priority queue,
destination scratch) are promoted to private members of `ParetoDijkstra` and
pre-allocated at construction — `run()` resets them in-place without calling
`malloc` or `operator new` for any Label object.

```
Label malloc / new calls during routing:         0
Working-buffer malloc / new calls per query:     0  (pre-allocated at construction)
QueryResult vector-shell allocation per query:   1  (~2.4 KB for BMRCL, at function exit)
```

> **Note:** `QueryResult` is returned by value (by-move from `result_pareto_`).
> `std::move` leaves the source vector with capacity zero; the subsequent `resize()`
> allocates one small buffer for the vector shell (~`num_nodes × 24` bytes).
> This allocation occurs **after** the RDTSCP measurement closes and does not appear
> in the reported latency. Label allocations remain strictly zero.
> For fully allocation-free operation, use an output-parameter API overload.

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
├── docs/write-up.tex        # Formal mathematical proof document
├── CLAUDE.md                # AI session architecture directives
└── project_state.md         # Multi-session progress tracker
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
