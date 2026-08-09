# Accessibility surfaces: what the engine was computing all along

`include/accessibility.hpp` · `src/accessibility.cpp` · `tools/isochrone.cpp` ·
`tests/test_accessibility.cpp`

---

## The reframing

This repository has been presenting its output as a latency table — p50, p95,
p99 for a single-source, all-destinations query. That is a benchmark of the
implementation, and it hides what the query actually computes.

A one-to-all bi-criteria search returns, for one origin and one departure time,
the non-dominated ways to reach **every other station simultaneously**. That is
not a routing result. It is an **accessibility surface** with its trade-off
structure preserved instead of collapsed to a scalar.

Transport planning has used accessibility for decades — *how much of the city
can you reach in 45 minutes* — and almost always computes it from travel time
alone. That quietly asserts every 45-minute journey is equivalent. They are not:
45 minutes direct is not 45 minutes with two changes, and the difference falls
hardest on the people for whom an interchange is a barrier rather than an
inconvenience — carrying shopping, travelling with a small child, elderly, or
using a wheelchair.

So compute the same thing at each change budget separately:

```
reachable(origin, T, c) = stations reachable within T seconds
                          using at most c changes
```

and look at the **gap** between `c = 0` and `c = 2`. That is the part of the
city which is nominally accessible and practically is not. It is invisible to
every standard accessibility metric.

## Why RAPTOR computes this and the Pareto engine does not

RAPTOR's round *k* **is** "at most *k* trips", so the whole surface is one query
per (origin, departure): take the round-*k* layer of `tau` and threshold it.

The Pareto engine's second objective counts *platform walks* rather than
vehicles, which is the same number only when every interchange requires changing
platform (see [raptor.md](raptor.md)). For a claim about passengers, "how many
vehicles must I board" is the honest quantity, so this module is built on
RAPTOR.

## Two modes

### One origin — an isochrone with a change budget

```bash
mkdir -p results
./build/routing_engine_isochrone feeds/norm/namma-metro \
    --origin KGWA --at 08:00 --out-prefix results/majestic --labels
```

`--origin` takes a **`stop_id`**, not a station name. `KGWA` is Nadaprabhu
Kempegowda (Majestic) in the feed `scripts/build_namma_metro_gtfs.py` produces;
`cut -d, -f1,2 feeds/norm/<slug>/stops.txt` lists them. An id the feed does not
contain is refused with a non-zero exit rather than silently routed from
somewhere else.

Writes `results/majestic-isochrone.csv` and `.svg`. For every station: the
earliest arrival with 0 changes, with at most 1, with at most 2, and the column
that matters — **how much longer the journey is for someone who will not
change**, or `unreachable_without_changing` where there is no such journey at
all. The second is a stronger statement than any number.

### The whole network — a per-station score

```bash
mkdir -p results
./build/routing_engine_isochrone feeds/norm/bart \
    --out-prefix results/accessibility-bart
```

For every station, the mean number of stations reachable within 30/45/60 minutes
at each change budget, averaged over six departure times across the day, plus
the gap. Writes `-surface.csv` and `-surface.svg`.

Options: `--thresholds 1800,2700,3600`, `--max-changes 2`, `--labels`.

## Decisions worth knowing about

**Stations, not platforms.** Two platforms of one station are one place a
passenger can get to. Counting them separately would inflate every reachability
number precisely at the interchanges the analysis is about, so both modes work
on the station graph from `compute_topology()` — the same graph the structural
metrics describe, obtained through its `StationGraph` out-parameter rather than
recomputed, so the map and the metrics cannot disagree about what the network
is.

**Unproductive departures still count in the denominator.** Means are over
*every* sampled departure, including those with no service. Dividing by only the
productive ones would flatter a station whose service stops at 20:00 by
pretending the evening does not exist, and accessibility is precisely a claim
about the whole day. `Accessibility.DeparturesWithoutServiceStillCountInTheDenominator`
pins this.

**One scale for both map axes.** An equirectangular projection with longitude
scaled by cos(mean latitude) — accurate enough at city scale, and no projection
dependency. Fitting each axis independently would make a north–south line and an
east–west line of the same length look different, which misrepresents the only
thing a transit map is for. `AccessibilityMap.ProjectionUsesOneScaleForBothAxes`
recovers the drawn coordinates and checks the aspect ratio.

**The SVG is self-contained and escaped.** Station names come from third-party
feeds and contain ampersands, angle brackets and apostrophes; unescaped, they
produce a file no viewer will open. The renderer writes its own background so it
looks the same wherever it is opened.

## What came out

Whole-network surfaces, six departures across the day, 45-minute budget, run of
2026-08-09:

| network | reachable with 0 changes | with ≤2 changes | the gap |
|---|---|---|---|
| Namma Metro (82 stations) | 23.6 | 42.2 | **+18.5** |
| BART (50 stations) | 18.8 | 22.7 | +3.9 |
| NYC Subway (496 stations) | 30.8 | 73.1 | **+42.3** |

Mean stations per origin. Read the gap column, not the totals: on Bengaluru's
metro, being willing to change lines **nearly doubles** how much of the network
is within 45 minutes. On BART it adds a fifth. On the New York subway it more
than doubles it.

That difference is the point. A passenger for whom an interchange is a real
barrier loses very little of BART and a great deal of New York — and the
standard accessibility metric, which is a single travel time, cannot say so
because it has already averaged the two cases together.

A single isochrone makes the same point sharper. From Nadaprabhu Kempegowda
(Majestic) at 08:00, all 81 other stations on Namma Metro are reachable — but
**14 of them only by changing.** For anyone who cannot, one in six of the
network simply is not there.

## What is not claimed

Counting reachable **stations** is not counting reachable jobs, schools or
people. A proper accessibility study weights destinations by what is at them,
which needs land-use and demographic data joined to the network and is a data
project of its own.

Station counts are the network-only version of the measure: defensible,
reproducible from the feed alone, and labelled as what it is everywhere it
appears. Nothing here is a claim about transport equity; it is an instrument
that a claim about transport equity could be built on.
