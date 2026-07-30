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
> variance/covariance term, and no variance is computed. See `docs/write-up.tex` §4
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

> **Measurement rig.** Pinned to core 3 with `taskset`, arena pre-faulted before the
> timed region, TSC calibrated at startup, and each sample bracketed by
> CPUID+RDTSC / RDTSCP+CPUID so neither the read nor the surrounding work is
> reordered across it. RDTSCP and InvariantTSC are both verified present at startup.
>
> **What is *not* controlled:** these numbers were taken under WSL2, where MSR
> frequency locking is unavailable (Hyper-V blocks `WRMSR`), so
> `scripts/stabilize_cpu.sh` cannot pin the governor or disable Turbo Boost. Turbo
> variance is therefore present. Running the same binary on native Linux with that
> script applied removes it; the run-to-run spread below is the honest bound in the
> meantime.

| Metric | Latency |
|--------|---------|
| p50    | 139.2 µs |
| p95    | 206.1 µs |
| p99    | 272.5 µs |

> **Workload:** real Namma Metro station topology — **82 stations, 35,588 timetabled
> edges** (695 KB CSR) across the Purple, Green and Yellow lines — with a *modelled*
> 5-min-headway timetable (BMRCL publishes no open timetable). Each query is a
> **single-source, all-destinations** bi-criteria search returning the non-dominated
> time-vs-crowd frontier at every reachable station, not a point-to-point lookup.
> 10,000 queries drawn from 500 pre-screened reachable (origin, departure-time) pairs,
> plus 1,000 warm-up queries.
>
> Measured under WSL2, where MSR frequency locking is unavailable (Hyper-V blocks
> WRMSR), so Turbo Boost variance is present. Figures are the **median of six runs**;
> across those runs p50 spanned 137.6–142.7 µs, p95 187.2–222.8 µs, and p99
> 255.1–305.7 µs. Occasional multi-millisecond `max` samples are hypervisor
> preemption, not routing cost — the harness prints a warning when
> `max / p99 > 10` so a contaminated run cannot be published silently.

### Second feed: BART (fully published timetable)

The Namma Metro figures use a modelled timetable. To measure against a network whose
schedule is *also* real, the same binary was run on BART's published GTFS feed, ingested
through `scripts/normalize_gtfs.py` with no engine changes:

| Metric | Latency | |
|--------|---------|---|
| p50 | 62.3 µs | 50 stations, 35,134 timetabled edges (686 KB CSR) |
| p95 | 96.0 µs | 12 routes, 2,689 trips, 40,575 stop_time rows |
| p99 | 136.3 µs | **0 rows dropped** — FK-clean ingest |

Median of five runs (p50 spanned 61.9–63.4 µs, p95 87.8–101.1 µs, p99 131.9–157.0 µs).
BART carries a comparable edge count to the Namma Metro feed but fewer stations (50 vs
82), and lands proportionally faster — consistent with per-node frontier work being the
term that scales with `|V|`.

> **What is real in each feed.** BART contributes a real topology *and* a real published
> timetable. Namma Metro contributes a real topology with a modelled 5-min headway.
> **In both cases `crowd_weight` is synthetic** — it comes from the Gaussian
> time-of-day model in `graph_builder.cpp`, since neither operator publishes occupancy
> data. The second objective is therefore a plausible model, not measured crowding, on
> every feed this engine has been run against.

<details>
<summary>Bundled 10-node synthetic feed, for scale</summary>

The demo feed that ships in `scripts/generate_synthetic_gtfs.py` measures
p50 511 ns / p95 1183 ns / p99 1593 ns. It is a toy graph — quoted only to show the
fixed overhead of the harness itself, not as a result.

</details>

**Hardware:** Dell G15 5520 (Intel Core i7-12700H, 16GB DDR5), WSL2 Ubuntu 24.04,
GCC 13, `-O3 -march=native`.

---

## Mathematical Formulation

### Graph Model

Let **G = (V, E)** where:
- **V** = set of Namma Metro stops (nodes), |V| = 82 on the current BMRCL topology
  (Purple, Green and Yellow lines, platform stops collapsed onto parent stations)
- **E** = set of directed timetabled service legs; each edge *e = (u, v, t)* carries
  weight **w(u, v, t)** = seconds to traverse from *u* to *v* departing at time *t*

### FIFO Property

Time-dependent edge weights must satisfy the **First-In-First-Out** constraint:

```
t₁ + w(u, v, t₁) ≤ t₂ + w(u, v, t₂)   for all t₁ ≤ t₂
```

This guarantees that Dijkstra's optimality subpath property holds. Synthetic crowd
penalties violate FIFO when they drop precipitously after morning peak. The
**Bounded-Wait Lookahead** policy resolves this by enforcing the derivative
constraint `d/dt(cost) ≥ -1` on the time-varying term: cost cannot fall faster than
one unit per second, so waiting can never beat departing by more than the wait itself.

In this build the binding term is `crowd_weight`, whose Gaussian peak model has
max |d/dt| ≈ 0.18 — comfortably inside the bound, and checked at graph-build time
under `NDEBUG`-off builds (`src/graph_builder.cpp`) as well as in
`tests/test_fifo_invariant.cpp`. The `penalty` field is a second time-varying term the
same bound would govern, but it is currently always 0 (see **Known Limitations**), so
it neither contributes to the objective nor exercises the constraint.

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

The objective `min travel_time + λ·crowd` is a **weighted-sum scalarization**, and λ
plays the same role Markowitz's risk-aversion coefficient does: sweeping it traces out
a trade-off curve. That is where the correspondence ends — this objective is
deterministic and *linear* in the crowd term, whereas Markowitz minimizes a quadratic
form over a covariance matrix of random returns. There is no probability distribution,
no variance and no covariance here, so the objective is **not** isomorphic to
mean-variance optimization and is not written with σ notation.

Two further limits worth stating plainly:

- **λ-scalarization recovers only the convex hull** of a Pareto frontier. A frontier
  with a non-convex kink has points that *no* single λ selects. This is exactly why the
  engine is multi-label-correcting — it maintains the frontier at each node directly
  rather than relying on the scalarization to enumerate it.
- If crowding were modelled stochastically, **variance would be the wrong risk measure**:
  transit delay distributions are right-skewed (bounded below by zero, unbounded above),
  so variance penalizes early arrivals as heavily as late ones. **Semi-variance or
  Conditional Value at Risk (CVaR)** are the economically appropriate proxies; CVaR is
  additionally coherent in the sense of Artzner et al., which variance is not.

See `docs/write-up.tex` §4 for the formal statement of all three points.

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
prefetching during Dijkstra relaxation.

The CSR stores **one edge per timetabled departure**, not one per neighbour, so its
size is driven by service frequency rather than by station count. At a 5-min headway
over an 05:00–23:00 service day the 82-station network yields **35,588 edges ×
20 bytes ≈ 695 KB** — too large for a 48 KB L1d, but resident in the 1.25 MB L2 of an
Alder Lake P-core, and traversed sequentially within each node's range so the hardware
prefetcher stays engaged. Only the *working set* of a single query — the visited nodes'
frontier vectors and the arena slots in use — stays L1-resident.

### Priority Queue Memory Bound

The `ParetoDijkstra` min-heap holds at most one entry per label ever pushed. The
theoretical worst-case bound is:

```
max_heap_entries ≤ |V| × k × D
```

where `|V|` is the number of nodes (82 on the measured feed), `k` is the maximum
number of non-dominated Pareto labels retained per node (practically ≤ 16), and `D`
is the maximum number of distinct departure times per origin node across the
timetable (216 at a 5-min headway over an 05:00–23:00 service day).

Taking round numbers: 100 × 16 × 200 = **320,000 heap entries** worst-case. Each
`Label` is 16 bytes (4 × uint32_t), so worst-case heap memory ≈ **5 MB**. In practice
the heap stays far below that for typical peak-hour queries, because lazy deletion
prunes stale labels before they propagate.

> **The arena is the binding constraint, not the heap.** `ARENA_DEFAULT_CAPACITY` is
> 65,536 labels (1 MB), well under the 320,000-label worst case. The arena does not
> silently overrun: `allocate()` **throws** when the bump pointer reaches capacity,
> so exhaustion surfaces as an exception rather than as a corrupted frontier. The
> `|V| × k` sizing rule leaves plenty of slack at metro scale (82 × 16 = 1,312 slots
> used of 65,536); it would bind only on a network an order of magnitude larger, at
> which point `k × D` must be re-derived and the capacity raised to match.

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
├── tests/                   # Google Test suite (8 files, 68 tests)
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

The engine reads five GTFS files (`agency.txt`, `stops.txt`, `routes.txt`, `trips.txt`,
`stop_times.txt`) from a data directory. The bundled demo uses a small synthetic feed; for
a real network, use the helpers in `scripts/`:

```bash
# Real Namma Metro topology (public-domain station/line/distance data) + a modelled
# 5-min-headway timetable (BMRCL publishes no open timetable).
# This is the exact sequence that produced the Benchmark Results table above:
mkdir -p raw_namma && curl -L -o raw_namma/bengaluru_metro_network.csv \
  https://raw.githubusercontent.com/Vinayak-Chinchakhandi/Bengaluru-Metro-Network-Dataset/main/bengaluru_metro_network.csv
python3 scripts/build_namma_metro_gtfs.py raw_namma/bengaluru_metro_network.csv data
taskset -c 3 ./build/routing_engine_benchmark ./data   # expect: Nodes: 82 | Edges: 35588

# ...or normalize a real published GTFS feed into the engine's layout. BART is the
# reference case (real topology AND real timetable) and produced the second table above.
# NOTE: the normalizer takes an UNZIPPED directory, not the .zip.
sudo apt-get install -y unzip                     # if not already present
curl -L -o bart.zip https://www.bart.gov/dev/schedules/google_transit.zip
unzip -o -q bart.zip -d raw_bart
python3 scripts/normalize_gtfs.py raw_bart data_bart
taskset -c 3 ./build/routing_engine_benchmark ./data_bart
```

> **Expected normalizer output for BART** — if your numbers differ materially, the feed
> format has changed:
> `routes kept: 12 | trips kept: 2689 | stops kept: 50 | stop_times kept: 37823`,
> then `Nodes: 50 | Edges: 35134` and `Dropped (bad FK): 0` from the engine.

> The C++ parser reads GTFS columns **positionally**; `normalize_gtfs.py` rewrites a real
> feed (which orders columns by header name) into that layout, keeps rail routes, and
> collapses platform stops onto their parent station. If **0 rows load**, the benchmark
> prints a prominent warning and falls back to a 10-node synthetic graph (so a failed load
> can't masquerade as a real result).

---
## Core Components

The correctness-critical routines of the engine, each covered by dedicated unit tests:

| Component | File | Mathematical Concept |
|-----------|------|---------------------|
| `select_optimal_departure()` | `src/routing.cpp` | Bounded-Wait Lookahead + FIFO preservation |
| `ParetoLabelSet::insert_and_dominate()` | `src/routing.cpp` | Binary search + dominance pruning |
| Lazy deletion filter in Dijkstra loop | `src/routing.cpp` | Stale label suppression |

Run `ctest` from the build directory — all 68 tests pass under AddressSanitizer +
UndefinedBehaviorSanitizer.

---

## Known Limitations

Stated up front rather than discovered later. Each is a deliberate scope boundary, and
each has a test or a runtime message pinning the current behaviour.

| Limitation | Effect | Where it's pinned |
|---|---|---|
| **Bounded-wait horizon.** `select_optimal_departure` considers only departures within `W_max_seconds` of arrival (default 1800 s). If the next service to a neighbour is further out, that neighbour is reported unreachable rather than "reachable after a long wait". | On a dense feed (≤30 min headway) no effect. On a sparse or late-night feed, reachable stations can be missed. Widen the window via `LookaheadConfig::W_max_seconds`. | `tests/test_fifo.cpp` (`BoundedWait_*`) |
| **One composite-optimal departure per link.** The engine expands the λ-minimizer among the next *k* departures rather than branching on every departure. | The returned frontier is the set of non-dominated trade-offs **across route choices** at a fixed λ, not the full per-boarding frontier. | `tests/test_pareto_oracle.cpp` (`MultiDeparture_*`) |
| **Crowd is a function of time of day only.** `crowd_weight` comes from a Gaussian peak model; it does not vary by station or by segment, and `penalty` is currently always 0. | The second objective is real and drives dominance, but carries less information than a position-dependent model would. | `tests/test_fifo_invariant.cpp` |
| **Calendars and `frequencies.txt` are parsed but not enforced.** All trips are treated as active on every service day. | Correct for the single-service-pattern metro feeds targeted here; wrong for a feed with weekday/weekend variants. | Runtime `[GTFS INFO]` line on every load |
| **No transfer edges.** Interchange stations are merged into a single node when within 250 m, so line changes cost zero time instead of a realistic walk-and-wait. | Understates journey time at interchanges. Requires `transfers.txt` or a BMRCL interchange table. | `src/graph_builder.cpp` (documented, with the ordering invariant a future implementor needs) |
| **Positional GTFS parser.** The C++ parser reads columns by position, not by header name. | Real feeds must pass through `scripts/normalize_gtfs.py` first. A feed that fails to load triggers a loud banner and a 10-node fallback rather than a silent bad result. | `tests/test_gtfs_parser.cpp` |

---

## License

MIT — see LICENSE.
