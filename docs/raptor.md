# RAPTOR, and the comparison this repository owed itself

`include/raptor.hpp` · `src/raptor.cpp` · `tools/raptor_bench.cpp` · `tests/test_raptor.cpp`

---

## Why

The README carried a sentence that was admirably honest and strategically
useless:

> *RAPTOR would very likely be faster for this specific objective.*

That is a guess about the central design decision of the whole project. This
replaces it with a measurement, and in doing so RAPTOR turns out to earn its
place here three separate times:

1. **As a speed baseline.** Same feed, same query set, same machine, same
   timing harness, interleaved so drift hits both arms equally.
2. **As an exactness oracle.** RAPTOR scans every trip of every route it
   reaches. It has no lookahead window and no departure-selection heuristic, so
   its arrival times are exact — which makes them a yardstick the shipped
   engine's bounded-wait lookahead can finally be measured against on *real
   timetables* rather than on hand-built fixtures.
3. **As the ground truth for the research question.** Round *k* of RAPTOR is,
   by construction, the earliest arrival using at most *k* trips. So the exact
   (arrival time, number of trips) Pareto frontier falls out of the round
   structure with no dominance machinery at all. That is what makes the
   multi-feed study a statement about *networks* rather than about this
   project's search heuristics. See [study.md](study.md).

Reference: Delling, Pajor and Werneck, *Round-Based Public Transit Routing*,
ALENEX 2012 / Transportation Science 49(3), 2015.

---

## What is implemented

The base algorithm (Algorithm 1 of the paper) with footpaths and local pruning,
plus the preprocessing that makes it correct on real feeds.

**Not** implemented, deliberately:

| Omitted | Why |
|---|---|
| Target pruning | This engine answers one-to-all queries. There is no target. |
| rRAPTOR (range queries) | Not needed; the study samples departure times explicitly. |
| McRAPTOR (extra criteria) | The comparison is against a *bi*-criteria engine. |
| Any preprocessing-based acceleration | The point is a like-for-like comparison, not the fastest possible RAPTOR. |

---

## Making the comparison mean something

A speed comparison is worthless unless both algorithms solve the same problem on
the same network. Four things had to be made identical, and one difference is
deliberate and is itself a finding.

### 1. Identical segment admission

`RaptorBuilder` replicates `GraphBuilder`'s edge-admission rules exactly. A
segment `stop[i] -> stop[i+1]` is admissible only when neither time is the
interpolation sentinel, both stop ids resolve, `pickup_type[i] != 1`,
`drop_off_type[i+1] != 1`, and `arrival[i+1] >= departure[i]`; travel time is
clamped to a one-second minimum, so the effective arrival is
`max(arrival[i+1], departure[i] + 1)`.

A trip containing an inadmissible segment is **split**, not dropped — in the
graph model the surviving segments remain traversable, so dropping the whole
trip would route the two engines over different networks.

### 2. Routes, and the non-overtaking split

RAPTOR's "route" is a maximal set of trips serving exactly the same ordered stop
sequence **and not overtaking one another**. The second half is not decoration:
the query finds the earliest catchable trip by binary search over trip index,
which is only valid if departure time at a given stop is monotone in that index.

Real feeds contain express services that leave later and arrive earlier. The
builder therefore splits a stop pattern into as many routes as it takes, by
first-fit into bins where every trip is pointwise no later than the next.
`RaptorTimetable::routes_split_for_overtaking` reports how many extra routes
that cost. Without the split the search silently returns a trip that is not the
earliest — no crash, no warning, just wrong answers.

### 3. Footpaths must be transitively closed — and real feeds are not

RAPTOR relaxes footpaths once per round, which is only correct when the footpath
relation is transitively closed. It is tempting to assume a generated
`transfers.txt` satisfies that: `normalize_gtfs.py --transfers` emits every
ordered platform pair of a station, so the graph is *complete* within each
station.

**Complete is not closed.** BART's feed supplies real `min_transfer_time` values
for some platform pairs and the normaliser defaults the rest, and at several
stations the direct time between two platforms is longer than walking via a
third. One pass then misses the cheaper two-leg walk.

This was not caught by reasoning about it. It was caught by
`tools/study.cpp`'s correctness gate, which reported that the Pareto engine —
which chains transfers for free through its priority queue — had beaten the
"exact" oracle on 121 observations. `RaptorBuilder` now takes the transitive
closure of the footpath relation at build time (a small Dijkstra per node over
the footpath graph, whose components are the platforms of one station), reports
how many footpaths that added, and `transfers_are_transitively_closed()` remains
available so the tools verify rather than assume.

Closure is also what keeps the two models identical: the graph model can already
walk A → B → C, so its effective walk time is `min(direct, via)`.

### 4. The one difference, which is a finding

The graph model has no notion of *which vehicle you are on*. Arriving at a node,
a passenger may board any departure from it at no cost. Its transfer count
therefore counts **platform walks**, whereas RAPTOR's round index counts
**trips**. The two agree whenever changing service requires changing platform,
and differ by one for every same-platform interchange.

So the engine's second objective systematically **under-counts changes wherever
two routes share a platform**. `tools/study.cpp` reports both numbers side by
side and never averages them together.

---

## Correctness: how it is actually established

`tests/test_raptor.cpp`, and the strongest test is not a hand-written
expectation.

**Agreement with the other implementation.** With the Pareto engine's lookahead
window opened up (`k_departures = UINT32_MAX`, `W_max = 172800`, `lambda = 0`)
neither algorithm is allowed a heuristic shortcut, and they are solving an
identical problem. `RaptorVsEngine.UnrestrictedEngineAgreesExactlyOnEarliestArrival`
runs both over 25 randomised networks — irregular headways, so arrival is not
monotone in departure for trivial reasons — and asserts the earliest arrival
matches at every node of every query. Two unrelated algorithms agreeing on
thousands of answers is worth more than any number of assertions written by the
same person who wrote the code.

**The one-sided property that makes it an oracle.**
`RaptorVsEngine.ShippedLookaheadIsNeverEarlierThanTheOracle` asserts that with
the *shipped* settings the engine may be later than RAPTOR but never earlier. If
that direction ever inverts, either the engine is finding journeys that do not
exist or RAPTOR is missing some, and every optimality-gap number measured with
it is void.

The rest covers the preprocessing, where the subtle failures live: overtaking
patterns, trips with one unusable segment, the one-second clamp, the footpath
closure including the case where a direct walk is slower than the detour, the
round cap being visible when it binds, and scratch hygiene across queries.

`tools/raptor_bench.cpp` runs the same gate on the *actual feed* before it will
print a single timing, and exits non-zero rather than report a comparison
between an algorithm and a wrong algorithm.

---

## Running it

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -G Ninja
cmake --build build --target routing_engine_raptor_bench

taskset -c 3 ./build/routing_engine_raptor_bench ./feeds/norm/bart
```

Three arms, interleaved one query each per iteration:

| arm | what it is |
|---|---|
| `engine-shipped` | the Pareto engine as `main_bench.cpp` runs it (k=5, W=1800, λ=1.0) |
| `engine-timeopt` | the same engine with λ=0, so departure selection minimises arrival — isolates the cost of the second objective from the cost of the search |
| `raptor` | rounds over routes |

It also prints the cost of an empty timed region. On a virtualised host the
serialising `CPUID` in the harness can be a VM exit costing microseconds, which
would swamp a fast RAPTOR query; printing it lets the reader judge whether the
comparison has any resolution at all.

Cross-feed results are collected by [study.md](study.md); this tool is the
single-feed deep dive.

---

## What came out

**On BART** (103 platforms, 1,021 trips, 120 footpaths; run of 2026-08-09,
pinned to core 3, AMD Ryzen 5 7640HS, 3,000 paired samples):

```
correctness gate : 4,120 arrivals over 40 one-to-all queries — identical
engine-shipped   : p50  66,014 ns   p95 110,456   p99 160,861   647 ns/destination
engine-timeopt   : p50  60,318 ns   p95 107,821   p99 169,215   591 ns/destination
raptor           : p50  17,544 ns   p95  27,936   p99  49,798   172 ns/destination
memory           : engine 275 KB vs raptor 126 KB (2.19x)
```

RAPTOR is **3.76× faster at p50** here, on 2.19× less memory. An unpinned rerun
of the same command minutes later gave 3.91× — which is the size of the machine
noise these numbers carry, and the reason the sweep is pinned.

**Across all 38 feeds** ([study.md](study.md)): faster on 34, median **2.91×**,
range 0.05× to 27.92×. It is *slower* on four — Renfe, SNCF Intercités, Ireland
and Metra — all sparse wide-area rail, where scanning a whole route to reach one
marked stop costs more than a label-correcting search that settles a handful of
labels and stops. That is the crossover this comparison existed to find.

As an oracle, across the whole sweep: **3,528,510 arrivals compared, zero cases
of the engine beating it.** The bounded-wait lookahead cost nothing at all on 17
of 38 feeds and up to a 13.4% suboptimality rate with a 65-minute mean gap on the
German national feed.

The preprocessing was not idle either. Across the sweep it created **25 extra
routes** because trips on a stop pattern overtook one another, and made **13,881
trip splits** at segments neither engine can traverse. Both counts are per
occurrence, not per trip: one trip with two bad segments contributes two splits.

## Reading the output honestly

- **The arms return different second objectives.** `engine-shipped` returns a
  frontier over (arrival, platform walks); RAPTOR returns one over (arrival,
  trips). The tool prints this next to the numbers rather than in a document
  nobody opens.
- **Latency comparisons are only valid within one run.** Interleaving cancels
  drift *within* a feed. Comparing a number from one sitting against one from
  another compares thermal states, not algorithms.
- **The memory figures are structures, not resident set.** `graph.memory_bytes()`
  against `tt.memory_bytes()`, both excluding per-query scratch.
