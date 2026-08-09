# When does multi-objective transit routing actually matter?

An empirical study across the world's published transit feeds.

`tools/study.cpp` · `include/topology.hpp` · `scripts/feeds.json` ·
`scripts/fetch_feeds.py` · `scripts/prefilter_gtfs.py` · `scripts/run_study.py` ·
`scripts/analyze_study.py` · `tests/test_topology.cpp`

---

## The question

Multi-objective (Pareto) transit routing is universally assumed necessary. The
machinery it needs — frontier maintenance, dominance pruning, label-correcting
revisits, stale-label filtering — is not free; every serious transit-routing
system pays for it.

Nobody seems to have checked how often that cost buys anything.

This project found out by accident. `tools/diag.cpp` printed the frontier size
at every reached node and, on Bangalore's Namma Metro, the answer was **one,
every time**: 24,600 observations, not a single genuine choice. On San
Francisco's BART the identical code found a real trade-off at 71% of nodes.

Same algorithm, opposite answers, depending only on the shape of the network.

> **Across real transit networks, what fraction of origin–destination pairs
> actually admits a genuine time-versus-changes trade-off, and which structural
> property of a network predicts it?**

If a structural metric predicts it, the result is a cheap test an agency or a
systems engineer can run on a network to decide whether multi-objective routing
is worth building at all.

## Why every feed is measured twice

The obvious approach — run `tools/diag.cpp` on many feeds — answers a narrower
question than it appears to. `diag` reports the frontier **this engine finds**,
and this engine expands one composite-optimal departure per link chosen from the
next *k* within a bounded window. A small frontier could mean the network has no
trade-off, or it could mean the search never looked. Those are different
findings and a study cannot conflate them.

| | what it measures | why it is here |
|---|---|---|
| **RAPTOR** | the exact (arrival, number of trips) Pareto frontier | Round *k* is by construction the earliest arrival using at most *k* trips, so the frontier falls out of the round structure with no dominance machinery and no heuristic. **This is the answer to the research question** — a property of the network, not of any implementation. |
| **the engine** | the frontier the shipped Pareto-Dijkstra finds, with its default lookahead | The answer to "what does this implementation see", which is a different and also interesting number. |
| **the difference** | how often, and by how much, the engine's earliest arrival is later than the true one | Isolates the cost of the bounded-wait heuristic on real timetables. λ is set to 0 so departure selection is trying to minimise arrival and the window is the only thing in the way. |

Every feed also passes a **correctness gate**: the engine must never report an
arrival *earlier* than the oracle. If it does, one of the two implementations is
wrong, the row is marked, `tools/study.cpp` exits non-zero and
`scripts/analyze_study.py` prints the feed in bold as unusable. This is not
decoration — it fired on BART during development and caught a real bug in the
RAPTOR footpath handling. See [raptor.md](raptor.md).

## The structural metrics

`include/topology.hpp`. All standard, none invented for this project.

**Cyclomatic number** (circuit rank) `μ = L − S + C`: the number of independent
cycles, and **zero exactly when the network is a forest**. This is the direct
formalisation of "is there more than one way to go", and it is what the tree
hypothesis actually predicts.

Alongside it: Kansky's **alpha** (meshedness), **beta** and **gamma** indices;
the **degree distribution** (termini, through stations, junctions); **interchange
density**; **route overlap** — the fraction of links carried by more than one
line, which is the *other* way an alternative can exist; and **service
frequency** as a median headway.

Three definitional decisions do real work:

- **The unit is the station, not the platform.** A feed normalised with
  `--transfers` keeps platforms as separate nodes, so every interchange is a
  little clique joined by transfer edges. Measuring topology on platforms would
  count those cliques as cycles and report a mesh where the city has a tree —
  an artefact of the encoding masquerading as a property of the network. So
  platforms are merged into stations (connected components of the transfer
  graph) first.

- **A "line" is not a `route_id`.** Most feeds publish each direction as its own
  `route_id` — BART's twelve are six lines, there and back — and counting them
  directly made *every* station on any line look like an interchange, which put
  BART's interchange density at exactly 1.000. Two `route_id`s serving the same
  **set** of stations are therefore treated as one line.

- **Headway is per directed *platform* link.** The two directions between a pair
  of stations use different platforms; merging them interleaves their departures
  and halves the apparent headway. BART came out at a median of 0 seconds, which
  is not a headway, it is an artefact.

Each of these was a wrong number first, and each has a test named after the
failure in `tests/test_topology.cpp`.

## The feeds

`scripts/feeds.json`: 38 published static GTFS feeds reachable by anonymous
HTTP — no credentials, no API keys, no scraping — across North America, Europe,
the Middle East, South America and Australasia. A second list records the feeds
that are **not** reachable, with the reason, so a reader can tell "excluded"
from "unobtainable without an account". Delhi is the most painful of those: it
has genuine parallel routes and would be the single most valuable Indian
addition.

Bengaluru is included and flagged everywhere it appears: real station topology
and coordinates from an open dataset, but a **modelled** timetable, because
BMRCL publishes no open GTFS.

Three normalisation decisions matter for comparability, all in
`scripts/prefilter_gtfs.py`:

**Rail modes only.** The unit of comparison is a city's fixed-guideway network,
so a regional feed contributes its rail layer and nothing else. Basic
`route_type` 0/1/2/5/7/12 plus the extended ranges 100–199, 400–499, 900–999 and
1400–1499. Trolleybus (11) is excluded — it is a bus, whatever the older
normaliser's comment said — and the extended ranges matter: Entur's Norwegian
feed types every service in the extended space, so a basic-only filter reports
it as having no rail service at all and the feed silently drops out.

**One service day, not all of them.** A GTFS feed describes a whole timetable
period: weekday, Saturday, Sunday and holiday variants of the same train all sit
in `trips.txt`, distinguished only by `service_id`, and the C++ parser does not
filter on service day. Loading a feed whole puts every variant into the graph at
once. BART carried **three** trips departing the same platform at the same
second, which tripled the edge count and drove the measured headway to literally
zero. Worse than a constant factor across a study, because feeds differ in how
many patterns they publish. So trips are filtered to a single service date,
resolving `calendar.txt` and `calendar_dates.txt` properly, defaulting to the
weekday with the most active trips. The chosen date is recorded per feed in
`feeds/feeds.lock.json`.

**Checksums.** Agencies republish feeds continuously at the same URL with no
version. Every download is hashed and pinned. See
[reproducibility.md](reproducibility.md).

## Running it

```bash
# 1. acquire, pin and normalise every feed (a few GB, one pass)
python3 scripts/fetch_feeds.py --workdir feeds

# 2. build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -G Ninja
cmake --build build -j"$(nproc)"

# 3. run the study, pinned so the latency columns are comparable across feeds
python3 scripts/run_study.py --workdir feeds --build build --pin-core 3

# 4. the answer
python3 scripts/analyze_study.py feeds/study-results.csv
python3 scripts/analyze_study.py feeds/study-results.csv --markdown > results/study-results.md
```

Or all of it at once with `bash scripts/reproduce.sh`.

Each feed runs as its own process with its own timeout. A feed that crashes,
hangs or exhausts the router's arena becomes a **row with a status**, not a
missing row — a study that silently omits the feeds it could not process is
reporting a biased sample.

## Reading `study-results.csv`

One row per feed. Groups of columns:

| prefix | what it is |
|---|---|
| — | `slug`, `status`, graph shape, memory, RAPTOR structure, `footpaths_closed` |
| (topology) | every field of `TopologyMetrics`, in `topology_csv_header()` order |
| `rt_` | RAPTOR's exact frontier: observations, reached, k=1, k>1, the rate, max and mean size, deepest round used |
| `en_` | the shipped engine's own frontier over the same query set |
| `ag_` | agreement: compared, equal, later, **earlier (must be 0)**, unreached, and the gap distribution in seconds |
| `t_` | interleaved timing for both arms, plus the cost of an empty timed region |

`scripts/analyze_study.py` turns that into: the ranking, Spearman and Pearson
correlations against every structural metric, the forest hypothesis as a
contingency table, the lookahead cost table, and the head-to-head timing.

**Spearman is the headline** because the trade-off rate is a proportion bounded
at 0 and 1 with a large mass at 0, and several structural metrics are heavily
skewed — one feed has forty times the stations of another. Pearson on those is
dominated by the extremes; Spearman asks the question actually being asked.

The p-values are from a t approximation and are labelled as such wherever they
are printed. They bound sampling noise. They say nothing about **selection**:
these are the networks whose agencies publish open data, not a random sample of
the world's transit systems.

## What came out

Run of **2026-08-09**: 38 feeds, all `ok`, 4,415,126 reached (origin, destination)
observations, 3,528,510 arrivals compared against the oracle, **0** cases of the
engine beating it, 0 queries lost to the round cap. Feed checksums in
`feeds/feeds.lock.json`. Regenerate with `bash scripts/reproduce.sh`.

**1. The trade-off rate varies enormously, and it is not noise.** From 0.00% to
51.55% of reached observations. Bengaluru at 0.00% is not an outlier; it is one
end of a spectrum.

**2. The cyclomatic number is the strongest predictor of thirteen tested.**
Spearman **+0.844**, p < 0.0001. Junction fraction (+0.736), links per station
(+0.734) and number of lines (+0.722) follow. Median headway is the only metric
with no relationship at all (+0.110, p = 0.51) — *how often* the service runs
does not predict whether there is a choice to make; *what shape* the network is
does.

**3. The forest hypothesis holds, and is necessary but not sufficient.** All
three networks whose station graph is a forest — Calgary, Namma Metro, Sound
Transit — score exactly **0.00%**. But so do three networks that *do* have
cycles: Rome (μ=1), Atlanta (μ=1) and Santiago (μ=13). A cycle somewhere in the
network is not the same as a cycle that any passenger's journey can use. The
clean statement is one-directional: **a forest guarantees no trade-off; cycles
do not guarantee one.**

**4. The engine's own frontier is uncorrelated with the exact one.** Across the
22 feeds where the engine uses its transfer-count objective, Spearman between
its trade-off rate and RAPTOR's is **−0.063** — no relationship whatsoever, in
either direction. Warsaw: exact 44.98%, engine 0.00% (the feed has two transfer
edges, so there is nothing to count). Sound Transit: exact 0.00%, engine 37.19%
(a platform walk scores as a trade-off on a network with one path between any
two points). This is the result that justifies measuring every feed twice, and
it is a finding about this project's own instrument.

**5. The bounded-wait lookahead is free on half the feeds and expensive on the
rest.** Exactly optimal on 17 of 38. Worst case Germany: later on 13.4% of
comparisons, mean gap 65 minutes, and 562,817 destinations it does not reach at
all that RAPTOR does. The pattern is service density — a 30-minute window is
generous on a metro and useless on regional rail.

**6. RAPTOR wins, except where it does not.** Faster on 34 of 38, median
**2.91×** (range 0.05× to 27.92×). The interesting part is the crossover: it is
**slower** on Renfe (0.05×), SNCF Intercités (0.09×), Ireland (0.45×) and Metra
(0.63×) — every one sparse, wide-area rail. RAPTOR scans a whole route once any
stop on it is marked; a label-correcting search on a graph that thin settles a
handful of labels and stops. Note also the `unreached` column for those feeds:
part of the engine's win there is work it did not do.

The README's old concession — *"RAPTOR would very likely be faster for this
specific objective"* — is now a measured statement with a regime attached.

## Honest limits

- **The sample is not random.** Agencies that publish open GTFS skew rich,
  Western and rail-heavy. Every conclusion is conditional on that.
- **One service day per feed, chosen automatically.** A representative full
  weekday, not an average over the year.
- **Rail only.** A city whose alternatives are bus routes will look
  alternative-free here. That is a scope decision, stated, not a finding.
- **Latency columns are machine-dependent** and only comparable across feeds if
  the whole sweep ran in one sitting on a pinned core. The structural and
  frontier columns are deterministic given the same feeds.
- **The engine's second objective is not RAPTOR's.** It counts platform walks;
  RAPTOR counts vehicles. They coincide only where changing service requires
  changing platform. The two are reported side by side and never averaged
  together — and the divergence between them is one of the study's more
  interesting outputs, not a nuisance.
