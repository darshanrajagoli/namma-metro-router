# A crowd model made of measurements

`include/crowd_model.hpp` · `src/crowd_model.cpp` · `tools/crowd_study.cpp` ·
`scripts/fetch_ridership.py` · `scripts/station-aliases-bmrcl.csv` ·
`tests/test_crowd_model.cpp`

---

## The problem this fixes

The engine's original second objective is a Gaussian in time of day:

```
crowd(t) = 10 + 90 · exp( −(t − 08:00)² / (2 · 3600²) )
```

It is a function of one variable, and that variable is the same for every edge
in the city. Two different routes between the same pair of stations, departing
at the same second, get the same crowd cost. The second Pareto dimension
therefore has nothing to discriminate — and `tools/diag.cpp` measured exactly
that: a single-label frontier at 96–100% of nodes.

The machinery was never broken. **The objective was constant.**

## The data

BMRCL's station-wise hourly entry counts, obtained under the Right to
Information Act and republished as open data.

| | |
|---|---|
| Dataset | `github.com/Vonter/bmrcl-ridership-hourly` → `data/station-hourly.csv.zip` |
| Upstream | `data.opencity.in/dataset/bmrcl-station-wise-ridership-data` |
| Origin | BMRCL, via RTI |
| Shape | `Date;Hour;Station;Ridership` — semicolon-separated |
| Coverage | 83 stations × 24 hours × 48 dates (2025-08-01 … 2025-09-30) |
| Fetch | `python3 scripts/fetch_ridership.py --out data-ridership` |

`fetch_ridership.py` records a SHA-256 and reports loudly when the upstream
refreshes. It does not fail on a moved checksum — the upstream is a living
dataset — but numbers either side of a move are not directly comparable, and it
says so.

This is a **station × hour** surface. That is the whole point: it varies in
space as well as time, which is the minimum needed for a route-choice trade-off
to be representable at all.

## What is being claimed, and what is not

Station entries are **not** train occupancy. A passenger entering at station A
loads the train from A onwards, not at A alone; the count says nothing about
direction, and nothing about how full the train already was.

Treating entries as a proxy for the crowd a passenger meets *boarding at that
station* is a modelling assumption, stated in the header of `crowd_model.hpp`
and repeated here. It is better than "crowding is identical everywhere in the
city", which is what it replaces. It is worse than a load-profile model built
from origin–destination flows — which the same dataset would support, and which
is left as future work. No tool here describes it as a measurement of occupancy.

## Name matching, deliberately conservative

The ridership file names stations the way the operator does; the feed names them
the way the topology dataset does. A matcher that is too eager silently
attributes one station's passengers to another, producing a plausible crowd
field that is wrong in a way no aggregate statistic reveals.

So: case folded, punctuation to spaces, runs of spaces collapsed, a trailing
`metro station` / `station` removed — and nothing else. Words like *Road* and
*Cross* are **not** stripped, because "Mysuru Road" is not "Mysuru".

Everything that then fails to match is reported, not smoothed over. On the Namma
Metro feed eight names genuinely differ (transliteration variants, mostly) and
are listed explicitly in `scripts/station-aliases-bmrcl.csv`. That file is
pipe-separated, because two of the names it exists to fix contain commas.

One station — BMRCL's *Electronic City* — has **no** counterpart in the topology
dataset at all. It stays unmatched. Inventing a mapping would attribute a real
station's passengers to a different one.

## FIFO safety, by construction and then verified

Raw hourly buckets are a step function. A step *down* of size *d* at an hour
boundary means the weight falls by *d* in one second, which violates the
`d/dt(crowd) ≥ −1` bound the whole consistency argument rests on
(`graph.hpp`, `routing.hpp` §3).

So the buckets are read as samples at the **centre** of each hour and
interpolated linearly, bounding the derivative by
`(max adjacent difference) / 3600`. At the default scale of 1000 that is at most
0.28 per second.

That is an argument about a continuous function, and the graph holds sampled
departures — so `apply_hourly_crowd()` **measures** the realised worst slope
between consecutive departures on every link and reports it. A safety property
nobody has watched fail is a safety property nobody has tested, so
`CrowdModel.InterpolationKeepsTheFifoDerivativeBoundAndStepsDoNot` asserts both
that the interpolated model is safe and that the un-interpolated one is
measurably worse.

## Why this is additive rather than a change to the builder

`apply_hourly_crowd()` rewrites `secondary_weight` on an **already-built**
`CSRGraph`. `src/graph_builder.cpp` is untouched. Two consequences:

- the shipped build path and every existing measurement are unaffected;
- both crowd models can be run over the same feed **in the same process**,
  which is what makes `tools/crowd_study.cpp` an A/B rather than two runs.

`gaussian_crowd_weight()` is a copy of the model inside `graph_builder.cpp`,
kept deliberately so the baseline cannot drift when the other file is edited —
and pinned to it by
`CrowdModel.GaussianCopyMatchesTheWeightsGraphBuilderActuallyProduces`, which
builds a real graph and compares every edge.

## Running it

```bash
python3 scripts/fetch_ridership.py --out data-ridership

./build/routing_engine_crowd_study feeds/norm/namma-metro \
    --ridership data-ridership/station-hourly.csv \
    --aliases   scripts/station-aliases-bmrcl.csv
```

It reports, for both models on the same feed and the same query set: the
frontier size distribution, the mean accumulated crowd, and a λ sweep against
the λ=0 route.

## The prediction, stated before measuring

A spatially varying objective is **necessary** for a route-choice trade-off. It
is not **sufficient**.

On a network whose station graph is a forest there is exactly one path between
any pair of stations, so there is no second route for any second objective to
prefer — whatever it measures. Namma Metro's station graph has a cyclomatic
number of **zero**, which `tools/study.cpp` and `tools/crowd_study.cpp` both
report. So:

- the frontier should stay single-label, because the topology forbids anything
  else;
- λ should now change the chosen **departure** differently at different
  stations, which the time-only model could not do;
- the crowd cost accumulated along a journey should become a real number that
  differs between origins, which it previously could not.

If the first prediction fails, the tree argument is wrong — and that would be
the more interesting outcome. Either way it is measured. The tool prints its own
reading of the result, including the cyclomatic number, so the conclusion cannot
be quietly reversed after the fact.

## What came out

Run of 2026-08-09, Namma Metro feed, 300 screened query pairs, ridership
averaged over 48 dates (2025-08-01 … 2025-09-30):

```
stops matched   : 82 of 82 (100%) with the alias file; 74 of 82 without it
unmatched       : one ridership station, "Electronic City", which the topology
                  dataset does not contain — left unmatched, not invented
peak cell       : 3,324 entries in one station-hour
weights         : max 998, mean 146  (the Gaussian model spans 100..1000)
FIFO            : steepest fall 0.0967 per second — bound is 1.0, holds
```

**The frontier stays single-label at every one of 23,340 reached observations,
under both models.** Predicted, and for the predicted reason: Namma Metro's
station graph has a cyclomatic number of 0. With one path between any two
stations there is no alternative route for any second objective to prefer.

**What the measured model does change is real and large.** Against the λ=0
route, the Gaussian model changes the chosen departure at 29.6% of observations;
the measured model changes it at **79.9%**. The time-only model could not
discriminate between stations at all, so its only lever was the time of day; the
measured one prices the same minute differently at Majestic and at Jnanabharathi,
and the router responds.

Two further observations worth stating because they are unflattering:

- **λ is an on/off switch, not a dial.** Every λ from 0.5 to 1000 produces
  identical output under both models. Per-link travel time is constant across
  departures, so once λ > 0 the argmin of `travel + λ·crowd` is decided by crowd
  alone and the magnitude cannot matter. The README's older description of λ as
  behaving like a switch is confirmed here across a much wider sweep.
- **The crowd-averse route is drastically slower.** Under the measured model the
  chosen journey arrives on average **5,401 seconds later** than the time-optimal
  one, because the composite objective deliberately excludes waiting cost and
  the router will use the full 30-minute window at every link to find an emptier
  train. That is the policy behaving exactly as specified, and it is a strong
  argument that the specification wants a waiting term before anyone treats its
  output as a recommendation.
