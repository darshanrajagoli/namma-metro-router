# The research artifacts

Six pieces of work were added to this repository on top of the routing engine.
They are separate: separate sources, separate binaries, separate tests, separate
documents. Each one answers a different question, and each can be read, run and
judged on its own.

Start with [study.md](study.md). It is the one with a research question in it.

| | What it is | Read | Build target |
|---|---|---|---|
| **1** | **The multi-feed study.** Across 38 real transit networks worldwide: how often does a genuine time-versus-changes trade-off exist, and what predicts it? | [study.md](study.md) | `routing_engine_study` |
| **2** | **RAPTOR.** A second, independent implementation — used as a speed baseline, as an exactness oracle, and as the ground truth for (1). | [raptor.md](raptor.md) | `routing_engine_raptor_bench` |
| **3** | **The reproducibility artifact.** Docker image, checksum-pinned feeds, one command that regenerates every figure. | [reproducibility.md](reproducibility.md) | `scripts/reproduce.sh` |
| **4** | **A crowd model made of measurements.** BMRCL's station-hourly ridership, obtained under RTI, replacing a Gaussian that was constant across the whole city. | [crowd-model.md](crowd-model.md) | `routing_engine_crowd_study` |
| **5** | **Accessibility surfaces.** The same query, presented as what it actually computes: what is reachable from here, by when, with how many changes — and the gap between those. | [accessibility.md](accessibility.md) | `routing_engine_isochrone` |
| **6** | **Where should one new interchange go?** The inverse question: the engine as the inner loop of a search over network modifications, ranking every walkable connection the network does not already have by what it measurably buys. | [interchange.md](interchange.md) | `routing_engine_interchange_search` |

## How they fit together

They are not five unrelated additions. (2) is what makes (1) a statement about
networks rather than about this engine's search heuristics, because it measures
the trade-off exactly instead of measuring what a bounded-wait lookahead happens
to find. (5) is built on (2) for the same reason: RAPTOR's round index counts
vehicles, which is the quantity a passenger cares about, while the engine's
second objective counts platform walks. (4) is the repair to the original
framing that (1) then explains — a crowd model constant in space cannot produce
a route-choice trade-off, and on a network whose station graph is a forest
nothing can. (3) is what makes any of it checkable by someone else. (6) turns (5) around: once
you can measure trade-off structure you can ask what would change it, and the
answer for Bengaluru — that no walkable interchange exists at all, the nearest
unjoined pair being 1283 m apart and redundant with the interchange it would
duplicate — is a sharper statement about the city than the surface alone.

## New files

```
include/raptor.hpp          src/raptor.cpp            tests/test_raptor.cpp
include/topology.hpp        src/topology.cpp          tests/test_topology.cpp
include/crowd_model.hpp     src/crowd_model.cpp       tests/test_crowd_model.cpp
include/accessibility.hpp   src/accessibility.cpp     tests/test_accessibility.cpp
include/interchange.hpp     src/interchange.cpp       tests/test_interchange.cpp

tools/study.cpp             the multi-feed measurement, one feed in, one CSV row out
tools/raptor_bench.cpp      the focused head-to-head, with a correctness gate
tools/isochrone.cpp         accessibility surfaces, as CSV and self-contained SVG
tools/crowd_study.cpp       the crowd-model A/B, both models in one process
tools/interchange_search.cpp  rank candidate new interchanges by measured gain

scripts/feeds.json                  38 feeds, plus the ones that are unobtainable and why
scripts/fetch_feeds.py              download, checksum-pin, prefilter, normalise
scripts/prefilter_gtfs.py           rail modes, one service day, streaming
scripts/run_study.py                drive the study across every feed
scripts/analyze_study.py            rankings, correlations, the forest test, the head-to-head
scripts/fetch_ridership.py          the BMRCL RTI dataset, pinned
scripts/station-aliases-bmrcl.csv   the eight names that genuinely differ
scripts/reproduce.sh                every result, one command

Dockerfile  .dockerignore  CITATION.cff  .zenodo.json
```

**Not one line of pre-existing `include/`, `src/`, `tests/` or `tools/` code was
changed.** Four files that already existed were edited, all of them
infrastructure rather than engine:

| File | Edit |
|---|---|
| `CMakeLists.txt` | the new sources and the five new targets |
| `.gitignore` | the study's working directory and the new output types, with `feeds/feeds.lock.json` deliberately kept |
| `.gitattributes` | `Dockerfile text eol=lf` — the repo is cloned with `core.autocrlf=true`, and a Dockerfile whose `RUN` continuations end in CRLF fails to build |
| `.github/workflows/cpp-ci.yml` | one step smoke-testing `prefilter_gtfs.py`, matching the step that already covers `normalize_gtfs.py`. The Python here decides the study's mode filter and service day, and `ctest` cannot reach either. |

That constraint was deliberate. The engine's measurements — the arena A/B, the
scaling study, the FIFO probe, the latency figures — all still describe exactly
the code they were taken from.

## The test suite

85 cases before, **156** after, all green under AddressSanitizer and UBSan.

The 71 new ones are not coverage padding. The strongest of them,
`RaptorVsEngine.UnrestrictedEngineAgreesExactlyOnEarliestArrival`, runs two
unrelated algorithms over randomised networks and asserts they agree on every
arrival at every node — which is worth more than any expectation written by
whoever wrote the implementation. Several others are named after something that
was actually wrong during development: an interchange density of exactly 1.000,
a headway deflated by merging two directions of one track, an "exact" oracle that
the engine beat, and a tool that carried its own copy of a library loop so the
tests covered code nothing ran.

The newest of them, `InterchangeSearch.AFootpathNeverReducesReachability`, pins an
invariant rather than a number: adding a footpath can only ever add an option, and
the transitive closure can only ever shorten a walk, so no proposed interchange may
make any measure worse. A regression in the closure, the platform-to-station merge
or the round accounting would show up there first.
