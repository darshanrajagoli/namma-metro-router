# Reproducing every number in this repository

`Dockerfile` · `scripts/reproduce.sh` · `scripts/fetch_feeds.py` ·
`scripts/feeds.json` · `CITATION.cff` · `.zenodo.json`

---

## One command

```bash
docker build -t namma-metro-router .
docker run --rm -v "$PWD/results:/work/results" namma-metro-router
```

Or, on a machine with a compiler and Python 3:

```bash
bash scripts/reproduce.sh                  # everything
bash scripts/reproduce.sh --quick          # 6 feeds, fewer queries, ~5 minutes
bash scripts/reproduce.sh --pin-core 3     # pin, for comparable latency numbers
bash scripts/reproduce.sh --skip-fetch     # reuse an existing feeds/ directory
```

Ten steps, each refusing to continue on a failure that would make the next one
meaningless:

1. build, Release and Debug
2. the test suite under AddressSanitizer and UBSan
3. the risk probe: which dominance orders survive a timetable — reads no feed
4. fetch, checksum-pin and normalise every feed in `scripts/feeds.json`
5. the multi-feed study, one row per feed
6. the analysis: rankings, correlations, the forest test, the head-to-head
7. the focused RAPTOR comparison on one feed, with its correctness gate
8. the crowd-model A/B against measured ridership
9. the accessibility surfaces, as CSV and SVG
10. where one new interchange would pay most, on each feed that has candidates

Everything lands in `results/`. Nothing is written outside `results/` and
`feeds/`.

**Step 2 comes before step 5 on purpose.** The study's headline numbers come
from comparing two implementations against each other. If the tests are red, the
comparison is between two unknown quantities and the study is worthless — so the
tests gate the study rather than running afterwards as a formality.

**Step 3 comes before step 4 on purpose too.** The risk probe is the one artifact
that reads no feed at all, and step 4 is a several-gigabyte download of 38
third-party feeds that agencies move without notice. Run last it would be out of
reach of anyone offline or behind a failed fetch; run third, a reader with no
network still reproduces one complete result. It costs seventeen seconds, and it
is deliberately *not* shortened by `--quick`, because a different trial count
would print numbers that no longer match the ones [risk.md](risk.md) quotes.

**Step 10 is skipped for New York under `--quick`.** It is the only one of the
three feeds with walkable candidates, and evaluating all 712 of them costs a full
RAPTOR preprocess each — about five minutes single-threaded. Namma Metro and BART
still run, including the stretched 2500 m radius that produces the Bengaluru
result, because on those two the search is nearly instant.

## Why this matters more than it sounds

SEA and ALENEX run artifact-evaluation tracks. A reviewer who can re-run an
experiment does not have to take the author's word for anything — including, in
this case, who typed which line of the implementation. In empirical work the
contribution *is* the experimental design and the finding, and both are
checkable here.

## What cannot be reproduced, stated up front

### The inputs

Transit agencies republish their feeds continuously, at the same URL, with no
version and no archive. This project has already been bitten by that once: BART
added roughly 40% more service between two runs and every published figure
moved.

So `feeds/feeds.lock.json` records, per feed, the SHA-256 of the exact archive
the numbers came from, its size, the `Last-Modified` and `ETag` the server
returned, when it was fetched, and which service day was selected.

```bash
python3 scripts/fetch_feeds.py --workdir feeds --verify
```

checks a local copy against those pins and fails on any mismatch. A re-fetch
that finds different bytes prints `[PIN MOVED]` with both checksums and carries
on — because a data refresh is normal and failing the build over it would be
worse — but it says, unmissably, that results either side of it are not
comparable.

What the lock file cannot do is let someone re-download the identical bytes
months later. Agencies do not keep old feeds. That is a property of the domain,
not of the tooling. Closing the gap fully means archiving the feed archives
themselves alongside the results, which is what a Zenodo deposit is for:
`.zenodo.json` and `CITATION.cff` are in place for it.

### The latencies

They depend on the host CPU, its thermal state, and whether the machine is on
mains power. Measured on this project's own hardware, p50 roughly **doubles** on
battery — larger than every algorithmic effect the repository has measured.

The structural, frontier and optimality-gap results are deterministic given the
same feeds. Those are the results the study's conclusions rest on. The timing
columns are labelled as machine-dependent everywhere they appear, and
`tools/study.cpp` and `tools/raptor_bench.cpp` both print the cost of an *empty*
timed region so a reader can tell whether the comparison had any resolution at
all on that host.

That split is measured, not assumed. Run the study twice and diff everything
except the timing block, which is columns 63–71 (`t_samples` through
`t_ratio_p50`):

```bash
python3 scripts/run_study.py --workdir feeds --build build --pin-core 3 --out a.csv
python3 scripts/run_study.py --workdir feeds --build build --pin-core 3 --out b.csv
diff <(cut -d, -f1-62,72 a.csv) <(cut -d, -f1-62,72 b.csv) \
  && echo "deterministic"
```

On this machine that produced **2,394 identical cells out of 2,394** — 38 feeds
across the 63 non-timing columns. Over the same two sweeps the engine's p50 moved
by a median of 4.2% and at most 19.3%. Anything quoted from the timing columns
carries that error bar; nothing else does.

## Determinism, and where it was nearly lost

Reproducibility is not only about inputs. Several places in this pipeline would
silently produce different answers run to run if left alone, and each is pinned:

| Where | What would drift | How it is pinned |
|---|---|---|
| `normalize_gtfs.py` | Python randomises string hashing per process, so iterating a set of stop ids emits `stops.txt` in a different order every run — renumbering the entire graph | stops are emitted in sorted order |
| `graph_builder.cpp` | `std::sort` is not stable and ties on departure time are everywhere; a different order changes which of two tied labels survives | the comparator is a total order |
| `RaptorBuilder` | pattern ids and trip order within a route | patterns keyed in a `std::map`, trips ordered by their full time vector with `trip_id` as the final tie-break |
| `prefilter_gtfs.py` | which service day is chosen | the weekday with the most active trips, ties to the earliest date — a deterministic function of the feed |
| query sampling | which origins and departure times are measured | fixed seed, reported in the output |

## The feed manifest

`scripts/feeds.json` lists 37 published static GTFS feeds reachable by anonymous
HTTP, plus Bengaluru — which is not a GTFS feed but an open topology dataset
with a modelled timetable, marked as such in every table — and a second list of
feeds that are **not** reachable, with the reason. That second
list is part of the artifact, not an omission: a reader can tell the difference
between "this network was excluded" and "this network could not be obtained
without an account".

The mode filter (rail only, basic and extended `route_type`), the service-day
selection, and the nested-archive handling all live in `scripts/prefilter_gtfs.py`
and are documented in its header. Licences are **not** asserted — each entry
carries a `source_page` to check before redistributing anything, and the
pipeline downloads at run time rather than vendoring feeds into the repository.

## Provenance of the one non-GTFS input

The ridership data behind the crowd model comes from BMRCL under the Right to
Information Act, republished as open data. `scripts/fetch_ridership.py` pins it
the same way and writes `ridership.lock.json` next to it. See
[crowd-model.md](crowd-model.md).
