# Namma Metro C++ Routing Engine

![CI](https://github.com/darshanrajagoli/namma-metro-router/actions/workflows/cpp-ci.yml/badge.svg)
![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)

> **This project models crowd-aware transit routing as a weighted-sum
> scalarization of a bi-criteria (travel-time vs. crowd-exposure) objective:**
> **min travel_time + λ·crowd**, where λ is a crowd-aversion coefficient. This is
> *structurally analogous* to Markowitz mean-variance portfolio selection (λ playing
> the role of the risk-aversion knob) — but the analogy is **informal**: the
> objective is *linear* in the crowd term, not quadratic in a variance/covariance
> term, and no variance is computed. See `docs/write-up.tex` §4.
>
> The second objective is **selectable at graph-build time** (`SecondObjective`):
>
> | Mode | Second dimension | Where it bites |
> |---|---|---|
> | `TransferCount` | number of platform changes | **BART: a real trade-off at 71% of reached nodes** |
> | `CrowdExposure` | modelled crowd exposure | Namma Metro: measurably inert — see below |
>
> **Measured caveat, stated up front:** the crowd objective is *degenerate on the
> feeds measured here*. Crowd depends only on time of day and is identical for every
> edge, so no spatial trade-off exists: λ acts as an on/off switch (λ=0.5 and λ=1000
> give identical routes) and the frontier holds one label at 96–100% of nodes. The
> **transfer objective on BART is the working demonstration** — frontier sizes up to
> 5, a genuine time-vs-changes trade-off at 71% of nodes. Both results, and the root
> cause, are in [Measured Behaviour](#measured-behaviour).

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
> **What is *not* controlled:** these numbers were taken under WSL2, which exposes no
> `cpufreq` sysfs and where Hyper-V blocks `WRMSR`, so `scripts/stabilize_cpu.sh` cannot
> pin the governor or disable Precision Boost. Boost variance is therefore present.
>
> **And on this hardware that script would not fully fix it on native Linux either.** It
> was written for an Intel machine. Its boost control targets
> `/sys/devices/system/cpu/intel_pstate/no_turbo`, which does not exist on AMD — the
> `acpi-cpufreq` fallback is the path that actually applies — and its BD PROCHOT step
> writes MSR `0x1FC` (`MSR_POWER_CTL`), which is **Intel-architectural and undefined on
> AMD**, so that write faults on Zen 4. Governor pinning and the fallback boost control
> do work on bare metal; the BD PROCHOT step never has on this CPU. The run-to-run
> spread below is the honest bound.

| Metric | Latency | Observed range (9 runs) |
|--------|---------|---|
| p50    | **148 µs** | 147–153 µs |
| p95    | 237 µs | 229–265 µs |
| p99    | 299 µs | 276–369 µs |

Equivalently **≈ 6,800 single-source all-destinations queries/sec** on one core, or
**≈ 1.8 µs per destination solved** (148 µs ÷ 82 stations).

> ⚠️ **Run on AC power.** On battery, Windows/WSL2 caps the package power and every
> figure here roughly **doubles** (measured on one binary: p50 139 µs on AC → 271 µs on
> battery) with no other change — same
> binary, same flags, all cores uniformly slower. This dwarfs the run-to-run spread
> above, so a battery run is not comparable to anything in this table. Verify with
> `(Get-CimInstance Win32_Battery).BatteryStatus` on Windows: `2` = AC.

> **Workload:** real Namma Metro station topology — **82 stations, 35,588 timetabled
> edges** (695 KB CSR) across the Purple, Green and Yellow lines — with a *modelled*
> 5-min-headway timetable (BMRCL publishes no open timetable). Each query is a
> **single-source, all-destinations** bi-criteria search returning the non-dominated
> time-vs-crowd frontier at every reachable station, not a point-to-point lookup.
> 10,000 queries drawn from 500 pre-screened reachable (origin, departure-time) pairs,
> plus 1,000 warm-up queries.
>
> Measured under WSL2, where MSR frequency locking is unavailable (Hyper-V blocks
> WRMSR), so Precision Boost variance is present. Figures are medians over 9 consecutive
> AC-power runs with the observed range beside them; tail percentiles vary more than
> p50, which is why nothing here is quoted to more than three significant figures.
>
> **Session-to-session variance is larger than within-session variance.** Across every
> AC run taken while developing this, Namma p50 spanned roughly 131–165 µs depending on
> the machine's thermal state and background load. Treat these numbers as accurate to
> about ±10%, and re-run the whole set in one sitting before comparing anything.
> Occasional multi-millisecond `max` samples are hypervisor preemption, not routing
> cost; the harness warns when `max / p99 > 10` so a contaminated run cannot be
> published silently.

### Second feed: BART (fully published timetable, with transfer layer)

The Namma Metro figures use a modelled timetable. To measure against a network whose
schedule is *also* real, the same binary was run on BART's published GTFS feed, ingested
through `scripts/normalize_gtfs.py` with no engine changes. This is the configuration
that exercises the **transfer objective**: platforms stay separate nodes and changing
line costs real time.

| Metric | Latency | Range (9 runs) | |
|--------|---------|---|---|
| p50 | **54 µs** | 52–60 µs | 103 platforms, 35,134 timetabled edges + 120 transfers |
| p95 | 89 µs | 85–112 µs | 12 routes, 2,689 trips, 40,628 rows, 688 KB |
| p99 | 117 µs | 113–171 µs | **0 rows dropped** — FK-clean ingest |

≈ **18,500 queries/sec**, or ≈ **0.52 µs per destination** across 103 platforms.

<details>
<summary>Same feed with platforms collapsed (no transfer layer)</summary>

Collapsing platforms onto their parent station gives 50 nodes and no transfers, so line
changes are free and instantaneous — the pre-transfer model. It measures p50 57 µs /
p95 90 µs / p99 138 µs (median of 3).

Note the transfer graph is *faster in absolute terms* while solving twice as many
destinations (103 vs 50). Collapsing concentrates every line's departures onto one
node, so each node carries ~700 edges against ~340 when platforms are separate, and the
per-expansion destination-deduplication scan is proportional to a node's edge count.
Both figures come from the same binary and the same 35,134 service edges.

</details>

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

**Hardware:** Dell G15 5535 (AMD Ryzen 5 7640HS, 16GB DDR5), WSL2 Ubuntu 24.04,
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

This guarantees that Dijkstra's optimality subpath property holds. The
**Bounded-Wait Lookahead** addresses one way it can fail — a time-varying cost term
that drops faster than the clock — by requiring `d/dt(cost) ≥ -1`, so waiting can
never beat departing by more than the wait itself. In this build the binding term is
`crowd_weight`, whose Gaussian model has max |d/dt| ≈ 0.15 (maximal at |t−peak| = σ,
where it equals 900·e^(−1/2)/3600; it is zero *at* the peak). That is comfortably
inside the bound, and it is checked at graph-build time under `NDEBUG`-off builds
(`src/graph_builder.cpp`) and in `tests/test_fifo_invariant.cpp`.

> **⚠ That derivative bound is not sufficient, and the engine does not preserve FIFO
> in general. This is measured, not theorised — see `tests/test_fifo_violation.cpp`,
> which constructs each failure and asserts the current behaviour.**
>
> Three independent mechanisms break it:
>
> 1. **The composite ignores arrival time.** Under `CrowdExposure` with λ > 0,
>    `select_optimal_departure` minimises `travel + λ·crowd + penalty` — an expression
>    with no `departure_time` term. With per-link travel times that differ between
>    departures, a later query can pick a faster train and arrive *earlier*.
>    Worked case: X departs 100 riding 400s (crowd 5, composite 405, arrives 500);
>    Y departs 110 riding 50s (crowd 400, composite 450, arrives 160). Querying at
>    t=100 arrives at **500**; querying at t=105 arrives at **160**.
> 2. **The `k_departures` budget truncates.** Only the first *k* candidates to a
>    destination are examined, and that set slides with query time.
> 3. **The `W_max` window truncates.** Same argument for `[t, t + W_max]`.
>
> Mechanisms 2 and 3 fire **even in the arrival-minimising regime** (λ = 0, or
> `TransferCount`), where the composite *is* the arrival time — so switching selection
> to minimise arrival, the obvious fix for mechanism 1, would not restore FIFO.
>
> **Why it does not bite on the feeds measured here.** `build_namma_metro_gtfs.py`
> derives travel time from distance ÷ average speed, so τ is identical for every
> departure on a link; arrival is then monotone in the chosen departure and the chosen
> departure cannot move backwards, so FIFO holds. The BART configuration avoids
> mechanism 1 separately by running under `TransferCount`.
> `FIFOViolation.ConstantPerLinkTravelTime_ArrivalIsMonotone` is the control.
>
> **The consequence is real, not cosmetic.** FIFO underpins *consistency under
> extension*: if label L₁ dominates L₂ at a node, extending both should preserve that.
> `docs/write-up.tex` §2 previously claimed this holds trivially for non-negative edge
> weights — but that argument assumes both labels are extended by the *same* edge, and
> they are not, because the edge is chosen as a function of arrival time.
> `FIFOViolation.DominatedPredecessorPruning_HidesAReachableDestination` exhibits a
> graph where the dominated label at an intermediate node was the only one that could
> catch the onward service, so **a genuinely reachable destination is reported
> unreachable**.

The `penalty` field is a second time-varying term the same bound would govern, but it
is currently always 0 (see **Known Limitations**), so it neither contributes to the
objective nor exercises the constraint.

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
20 bytes ≈ 695 KB** — too large for a 32 KB L1d, but resident in the 1 MB L2 of a Zen 4
core, and traversed sequentially within each node's range so the hardware
prefetcher stays engaged. Only the *working set* of a single query — the visited nodes'
frontier vectors and the arena slots in use — stays L1-resident.

### Priority Queue Memory Bound

The `ParetoDijkstra` min-heap holds at most one entry per label ever pushed. The
theoretical worst-case bound is:

```
max_heap_entries ≤ |V| × k × D
```

where `|V|` is the number of nodes (82 on the measured feed), `k` is the maximum
number of non-dominated Pareto labels retained per node, and `D` is the maximum number
of distinct departure times per origin node across the timetable (216 at a 5-min
headway over an 05:00–23:00 service day).

Taking round numbers with a worst-case `k = 16`: 100 × 16 × 200 = **320,000 heap
entries**. Each `Label` is 16 bytes (4 × uint32_t), so worst-case heap memory ≈ **5 MB**.

> **How loose that bound is, measured.** The observed maximum frontier size on any feed
> is **k = 5** (BART under the transfer objective; k = 1 everywhere on the Namma feed),
> and peak surviving labels in a single query is **280** — against an arena sized for
> 65,536, i.e. roughly **230× headroom**. The theoretical 320,000-entry bound is about
> **1,100× above** what the workload actually uses; see
> [Measured Behaviour](#measured-behaviour) §1.
> The arena does not silently overrun regardless: `allocate()` **throws** when the bump
> pointer reaches capacity, so exhaustion surfaces as an exception rather than as a
> corrupted frontier. The capacity would only bind on a network orders of magnitude
> larger, at which point `k × D` must be re-derived.

### Transfer Layer

Transfer edges model the walk between two platforms of one physical station — the cost
of changing line. They live in a **separate CSR adjacency** (`transfer_offset` /
`transfer_data`), not in `edge_data`:

```
transfer_data[transfer_offset[u] .. transfer_offset[u+1])  =  transfers out of u
```

The separation is deliberate. Service edges are sorted by `departure_time` and located
with `std::lower_bound`; a transfer has **no departure time** — it is available the
moment the passenger arrives — so folding it into that array would need a sentinel time,
and every candidate sentinel collides with a real departure (`0` is a legitimate
00:00:00). A separate array removes the ambiguity, leaves the service-edge binary search
untouched, and costs one pointer comparison on feeds that have no transfers.

`TransferEdge` is 8 bytes (`destination`, `travel_time`). Transfers are trivially FIFO:
`travel_time` is constant, so `t₁ ≤ t₂ ⟹ t₁ + w ≤ t₂ + w` — no Bounded-Wait treatment
is needed and none is applied.

> **Prior art — worth knowing before you cite this.** Time-versus-transfers is *the*
> classic bi-criteria formulation in transit routing, not a novel objective.
> **RAPTOR** (Delling, Pajor & Werneck, 2012) optimises exactly this pair, and gets
> transfer count structurally free by working in rounds — round *k* = journeys using at
> most *k* trips — with no graph and no dominance machinery at all. **Connection Scan**
> (Dibbelt et al., 2013) and **Transfer Patterns** (Bast et al., 2010) are the other
> standard approaches. This engine implements the older *time-dependent graph*
> formulation instead: transfers as explicit edges, multi-label-correcting Dijkstra with
> Pareto dominance. That is a legitimate and well-documented approach, and it is the
> right one here because the graph, arena and CSR layout are what the project is
> demonstrating — but RAPTOR would very likely be faster for this specific objective.

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
├── tests/                   # Google Test suite (10 files, 85 tests)
├── tools/                   # Measurement harnesses — see "Measured Behaviour"
│   ├── diag.cpp             # Frontier-size distribution + lambda sensitivity
│   ├── ab.py                # Interleaved arena-vs-heap A/B
│   └── scale.sh             # Latency vs timetabled-edge count sweep
├── scripts/
│   ├── normalize_gtfs.py    # Any real GTFS feed → this engine's positional layout
│   ├── build_namma_metro_gtfs.py  # Real BMRCL topology + modelled timetable
│   ├── generate_synthetic_gtfs.py # 10-station demo feed (used by CI)
│   ├── stabilize_cpu.sh     # Governor=performance, boost off. Written for Intel —
│   │                        # its BD PROCHOT step does not apply on AMD (see below)
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
`stop_times.txt`) from a data directory, plus an optional `transfers.txt` — when present,
the second objective becomes transfer count. The bundled demo uses a small synthetic
feed; for a real network, use the helpers in `scripts/`:

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

# --transfers keeps platforms as separate nodes and generates transfers.txt, which
# switches the second objective to transfer count. This produced the BART table above.
python3 scripts/normalize_gtfs.py raw_bart data_bart_tr --transfers
taskset -c 3 ./build/routing_engine_benchmark ./data_bart_tr

# ...or omit --transfers to collapse platforms onto stations (line changes then cost
# nothing, and the second objective stays crowd exposure):
python3 scripts/normalize_gtfs.py raw_bart data_bart
taskset -c 3 ./build/routing_engine_benchmark ./data_bart
```

> **Expected output for BART `--transfers`** — if your numbers differ materially, the
> feed format has changed:
> `transfers: 120 emitted (25 from feed, 95 defaulted to 120s)`,
> `stops kept: 103`, `trips kept: 2689`, then from the engine
> `Nodes: 103 | Edges: 35134 | Transfers: 120`, `Second objective: transfer count`,
> and `Dropped (bad FK): 0`.
> Without `--transfers`: `stops kept: 50`, `Nodes: 50 | Edges: 35134 | Transfers: 0`.

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
| Transfer relaxation in Dijkstra loop | `src/routing.cpp` | Time-independent edges, trivially FIFO |
| Transfer CSR construction | `src/graph_builder.cpp` | Separate adjacency, FK-validated |

Run `ctest` from the build directory — all **85 tests** pass under AddressSanitizer +
UndefinedBehaviorSanitizer.

---

## Measured Behaviour

Claims elsewhere in this README are design intent. This section is what
instrumentation actually reports. Every number below is reproducible with the tools in
`tools/`; two of the three results contradict what the design predicted, and are
recorded here rather than quietly dropped.

### 1. Frontier size: inert under crowd, load-bearing under transfers

`tools/diag.cpp` runs 300 reachable (origin, departure) pairs per feed and records the
frontier size at every reached node.

| Feed | objective | k=1 | k=2 | k=3 | k≥4 | **k>1** | max k | mean k |
|---|---|---|---|---|---|---|---|---|
| **BART + transfers** | TransferCount | 28.65% | 46.99% | 22.40% | 1.95% | **71.35%** | **5** | **1.977** |
| Namma Metro | CrowdExposure | **100.00%** | 0% | 0% | 0% | **0.00%** | 1 | 1.000 |
| BART collapsed | CrowdExposure | 96.09% | 3.91% | 0% | 0% | 3.91% | 2 | 1.039 |
| Bundled synthetic | CrowdExposure | 96.14% | 3.86% | 0% | 0% | 3.86% | 2 | 1.039 |

**This is the difference the transfer objective makes.** Under `TransferCount` on BART,
**71% of reached nodes carry a genuine trade-off** — an earlier arrival that costs a
change versus a later one that avoids it — and frontiers reach 5 labels. The
multi-label-correcting machinery (forward pruning in `insert_and_dominate`, the
bi-criteria lazy-deletion filter) is doing real work on every query.

Under `CrowdExposure` it is not. On the Namma feed the frontier **never** holds a
trade-off: 24,600 reached-node observations, every one a single label. Same code, same
tests — the difference is entirely in whether the data offers a choice.

Peak surviving labels in one query rises from 82 (crowd, one per node) to **280**
(transfers), still far below the 65,536-slot arena. The `k ≤ 16` figure quoted above
remains a worst-case bound; the observed maximum is 5.

### 2. λ is a binary switch, not a continuous knob

Same harness, sweeping λ and comparing each destination against the λ=0 (time-only)
optimum:

| λ | Namma: arrivals differ | BART: arrivals differ |
|---|---|---|
| 0 | — (reference) | — (reference) |
| 0.5 | 29.63% | 68.90% |
| 1 | 29.63% | 68.79% |
| 5 | 29.63% | 68.45% |
| 50 | 29.63% | 68.47% |
| 1000 | 29.63% | 68.54% |

On Namma, every λ > 0 gives **byte-identical** results — a 2000× change in the
crowd-aversion coefficient changes nothing. BART shows a fraction of a percent of
movement, which is noise-level. The mean arrival-time delta at λ>0 on Namma is
**+1800 s**, exactly `W_max_seconds`: with any crowd aversion at all, the policy always
waits the full lookahead window for the least-crowded departure.

**Root cause.** Under `CrowdExposure`, `secondary_weight` is a pure function of
`departure_time`, identical for every edge in the network (`synthetic_crowd_weight` in
`graph_builder.cpp`). There is no *spatial* variation, so no genuine "route A is faster,
route B is emptier" choice exists for the frontier to represent. And because crowd values
(~100–1000) dwarf travel times (~100–300 s), for any λ > 0 the composite is
crowd-dominated and the argmin is λ-independent.

**Under `TransferCount`, λ correctly has no effect at all** — 0.00% at every λ including
1000. That is not a defect: every service edge carries `secondary_weight = 0`, so λ has
nothing to scale. The trade-off lives in the Pareto frontier *across routes*, not in the
choice of departure on one link. `select_optimal_departure` therefore switches to
arrival-minimising selection in this mode, pinned by
`TransferRouting.DepartureSelectionMinimisesArrivalUnderTransferCount` — the
duration-minimising formula would pick a later train that arrives later while saving no
transfers, which is worse on both objectives at once.

Making `secondary_weight` position-dependent would make the crowd framing literally true
as well. It is not in this build; the transfer objective is the working demonstration.

### 3. The arena's payoff scales with allocation volume

`tools/ab.py` builds a second binary (`-DBUILD_BASELINE=ON`) in which every `Label`
comes from `operator new` and every query returns its result by value, then runs the two
**interleaved** so clock drift hits both arms equally. Medians of 7 pairs, AC power:

| Feed | peak labels / query | p50 | p95 | p99 |
|---|---|---|---|---|
| **BART + transfers** | **280** | **+30.8%** | **+24.6%** | **+18.4%** |
| Namma Metro | 82 | −5.3% | −3.8% | −1.1% |

*(positive = arena faster than the `new`/`delete` baseline)*

**The arena is worth 25–31% on the transfer workload and nothing on the crowd one**, and
the reason is visible in finding 1. Under `CrowdExposure` a query allocates ~82 labels —
one per node — so allocation is a rounding error and glibc's tcache serves 16-byte
requests in a few nanoseconds. Under `TransferCount` the frontier genuinely branches,
peak labels rise to **280**, and the allocator moves onto the critical path where the
arena's O(1) bump-and-free-list beats `malloc`.

This is worth stating precisely because the naive version of the claim — "arena
allocation makes it faster" — was **false on the workload originally measured**. The
optimisation only pays once the algorithm it supports is doing real work. Both halves of
that are reproducible with `python3 tools/ab.py <feed> 7 build`.

The baseline binary passes the routing, graph, parser and transfer suites under
ASan/UBSan; the failures are confined to `ArenaTest.*` / `ArenaCapacityTest.*`, which
assert arena internals — bump index, free-list length, slot ownership, exhaustion — that a
heap allocator does not have. The two arms therefore compute the same thing.

### 4. Scaling is near-linear in timetabled edges

`tools/scale.sh` holds the topology fixed at 82 stations and varies service frequency,
which varies |E| alone:

| Headway | Edges | CSR size | p50 | ns / edge |
|---|---|---|---|---|
| 900 s | 11,972 | 234 KB | 42.7 µs | 3.56 |
| 600 s | 17,876 | 349 KB | 64.7 µs | 3.62 |
| 300 s | 35,588 | 695 KB | 137.5 µs | 3.86 |
| 120 s | 88,724 | 1.69 MB | 407.9 µs | 4.60 |
| 60 s | 177,284 | 3.38 MB | 897.9 µs | 5.07 |

Across a **14.8× range** of edge counts, latency grows 21.1× — an empirical exponent of
**|E|^1.13**, i.e. near-linear with a mild log/cache term. Per-edge cost rises only 42%
even as the CSR grows from 234 KB (L2-resident) to 3.38 MB (well past the 1 MB L2),
which is the sequential-access property the CSR layout was chosen for.

---

## Known Limitations

Stated up front rather than discovered later. Each is a deliberate scope boundary, and
each has a test or a runtime message pinning the current behaviour.

| Limitation | Effect | Where it's pinned |
|---|---|---|
| **Bounded-wait horizon.** `select_optimal_departure` considers only departures within `W_max_seconds` of arrival (default 1800 s). If the next service to a neighbour is further out, that neighbour is reported unreachable rather than "reachable after a long wait". | On a dense feed (≤30 min headway) no effect. On a sparse or late-night feed, reachable stations can be missed. Widen the window via `LookaheadConfig::W_max_seconds`. | `tests/test_fifo.cpp` (`BoundedWait_*`) |
| **FIFO is not preserved in general.** Three mechanisms break it: the crowd composite contains no `departure_time` term, and both the `k_departures` budget and the `W_max` window truncate a candidate set that slides with query time. The last two fire even when selection minimises arrival. | A later query can return an *earlier* arrival. Downstream, consistency under extension fails, so a Pareto-dominated label at an intermediate node can be the only one able to catch an onward service — **a reachable destination can be reported unreachable**. Does not fire on the measured feeds: Namma has constant per-link travel time by construction, BART runs under `TransferCount`. | `tests/test_fifo_violation.cpp` (5 cases, incl. the control and the end-to-end consequence) |
| **One composite-optimal departure per link.** The engine expands the λ-minimizer among the next *k* departures rather than branching on every departure. | The returned frontier is the set of non-dominated trade-offs **across route choices** at a fixed λ, not the full per-boarding frontier. | `tests/test_pareto_oracle.cpp` (`MultiDeparture_*`) |
| **Crowd is a function of time of day only.** `crowd_weight` comes from a Gaussian peak model, identical for every edge; it does not vary by station or segment, and `penalty` is always 0. | **Measured consequence:** the second objective is effectively inert — frontiers hold one label at 96–100% of nodes and λ is a binary switch. See [Measured Behaviour](#measured-behaviour) §1–2. Position-dependent crowd is the fix. | `tools/diag.cpp`, `tests/test_fifo_invariant.cpp` |
| **Calendars and `frequencies.txt` are parsed but not enforced.** All trips are treated as active on every service day. | Correct for the single-service-pattern metro feeds targeted here; wrong for a feed with weekday/weekend variants. | Runtime `[GTFS INFO]` line on every load |
| **Transfers require a feed that distinguishes platforms.** The transfer layer is built from `transfers.txt` over platform-level nodes. BART has both; the Namma Metro builder merges interchanges within 250 m into one node, so line changes there still cost zero time. | Journey time is understated at Namma interchanges. BART is unaffected. Fixing it needs BMRCL platform-level stop data, which is not published. | `tests/test_transfers.cpp` (12 cases) |
| **Feed `transfers.txt` files are sparse.** BART's has 34 rows covering 11 stations, while 36 further stations have several platforms and no row at all. | `normalize_gtfs.py --transfers` therefore *generates* every ordered same-station platform pair, using the feed's `min_transfer_time` where present (25 pairs) and `--default-transfer` (120 s) elsewhere (95 pairs). Copying the file verbatim would leave 36 stations with no way to change platform, silently disconnecting the graph. | `scripts/normalize_gtfs.py` |
| **Positional GTFS parser.** The C++ parser reads columns by position, not by header name. | Real feeds must pass through `scripts/normalize_gtfs.py` first. A feed that fails to load triggers a loud banner and a 10-node fallback rather than a silent bad result. | `tests/test_gtfs_parser.cpp` |

---

## License

MIT — see LICENSE.
