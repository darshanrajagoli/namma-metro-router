# Where should one new interchange go?

`docs/accessibility.md` measures what a network gives its passengers. This is the
design question that follows from it, and it is the one a transport authority
actually has to answer:

> Given money for exactly **one** new interchange, where does it go?

The engine becomes the inner loop of a search over network modifications. Every
candidate is proposed, built, measured against the unmodified network, and ranked
by what it actually delivered — not by which stations look close together on a
map.

Run of **27 August 2026**. All three feeds are byte-identical to the pins in
`feeds/feeds.lock.json` — same SHA-256, same auto-selected service day — so these
numbers sit on exactly the inputs the multi-feed study was computed from. The
baselines below reproduce the published surface figures to two decimal places,
which is the check that the pipeline is measuring the same thing.

---

## What a candidate is

A walking connection between two stations that are physically close but which the
network does not currently join — a new footpath, in GTFS terms a pair of
`transfers.txt` records.

That is deliberately the cheapest possible intervention, and the only one this
data can honestly support. Building new track would need run times, alignment and
cost data that no feed carries. "These two stations are 274 m apart and there is
no way to change between them" is fully determined by `stops.txt`.

Excluded from the candidate set:

- pairs a vehicle already runs between — a footpath there is a walk beside a ride,
  not an interchange;
- pairs the feed already joins by transfer;
- anything beyond the walking radius, 800 m by default, the conventional planning
  figure for a transfer walk.

Walk time is modelled from straight-line distance at 1.2 m/s, the standard
pedestrian planning speed, with a 60-second floor covering egress and access.

---

## Two traps, both of which would have produced plausible nonsense

**1 — The station set must not move.** `compute_topology()` merges platforms into
stations by taking connected components of the transfer graph. Add a transfer
between two distinct stations and recompute, and those two stations become *one*
station: the "after" network has fewer stations than the "before" network, and
every count is measured against a different denominator. The comparison would be
meaningless and would look fine.

So the station graph is computed **once**, from the unmodified feed, and is an
input to the search rather than something it derives. Origins, destinations and
station identity are fixed across every evaluation. Only the footpath layer moves.

**2 — The timetable is rebuilt, not patched.** It is tempting to copy the
`RaptorTimetable` and splice two edges into `transfer_data`, since the routes and
trips plainly do not change. That would be wrong. RAPTOR relaxes footpaths once
per round and is correct only when the footpath relation is transitively **closed**,
so a new edge has to be chained against every existing one — the closure in
`src/raptor.cpp`, and the BART bug it was written for. Re-running
`RaptorBuilder::build()` gets the closure right by construction and keeps exactly
one implementation of it in the repository.

It costs a full preprocess per candidate. That is the correct trade: 712
candidates on the New York subway take 4 minutes 43 seconds single-threaded.

---

## What is ranked, and what is only reported

The ranking key is **Δreach**: the change in the mean number of stations reachable
within the threshold using the full change budget. That is how much more of the
city opens up, and it is what a planner is buying.

**Δgap** is reported beside it and is *not* ranked on, because its sign is not a
verdict. An interchange that widens the gap has delivered its gain only to
passengers willing and able to change; one that narrows it has delivered gain to
everyone. Both can be good buys. Collapsing them into a single score would be
exactly the averaging-away the multi-feed study spent its length arguing against,
so both are carried to the output and the reader decides.

That decision earned itself immediately — see Bengaluru below.

---

## What came out

| | Namma Metro | BART | NYC Subway |
|---|---|---|---|
| stations | 82 | 50 | 496 |
| cyclomatic number | **0** (a forest) | 2 | 88 |
| interchange density | 0.024 | 0.720 | 0.647 |
| reachable in 45 min, 0 changes | 23.61 | 18.79 | 30.80 |
| reachable in 45 min, ≤2 changes | 42.16 | 22.68 | 73.07 |
| the gap | 18.55 | 3.88 | 42.27 |
| **candidates within 800 m** | **0** | **0** | **712** |
| of those, improving reachability | — | — | 702 |
| nearest unjoined pair | 1283 m | 1235 m | 17 m |

**The first result is that the intervention is not always available, and whether
it is turns out to be a property of the network rather than a detail.** On two of
these three systems there is no pair of unconnected stations within walking
distance at all. A footpath interchange is simply not a thing you can buy in
Bengaluru or the Bay Area.

---

## Bengaluru: the tree cannot be fixed with a footpath

Namma Metro's station graph is a forest — cyclomatic number zero, exactly one
route between any pair of stations, and a trade-off rate of 0.00% in the
multi-feed study. A tree is also, in principle, the network where a single new
connection changes the most, because the first cycle you add is the first choice
any passenger has ever had.

There is nothing to add. **The closest pair of stations with no service between
them is 1283 m apart: Sir M Visvesvaraya Central College and Chickpete.** At
1.2 m/s that is an eighteen-minute walk, which is not an interchange.

And the detail that makes it sharp. Reading the two lines in the feed:

```
Purple Line:  ... VDSA  VSWA  KGWA  SRCS ...
Green Line:   ... SPGD  KGWA  CKPE  KRMT ...
```

`KGWA` is Nadaprabhu Kempegowda (Majestic), the network's only real interchange.
`VSWA` is Central College, immediately adjacent to it on the Purple Line. `CKPE`
is Chickpete, immediately adjacent to it on the Green Line. **The cheapest
conceivable second connection in Bengaluru is a walk between the two stations that
already flank the interchange it would duplicate** — which is why it delivers
exactly nothing.

Stretching the radius to 2500 m, well past any honest definition of a transfer
walk, admits 91 candidates. **Not one of them improves reachability at 45 minutes
by any amount.** The walk costs more than it saves: eighteen minutes is 40% of the
budget.

Nineteen of the 91 do move a number — they *reduce* the gap, by up to 0.93
stations. That is not an accessibility gain and it should not be reported as one.
It happens because RAPTOR counts vehicles, so a passenger who walks between two
stations has made no "change", and a journey that used to need one now formally
needs none. **A twenty-minute walk is a larger barrier than a cross-platform
interchange for precisely the passengers the gap measure exists to describe** —
someone with luggage, a child, or a wheelchair. The number is real; the reading
"this helps people who cannot change" is not. It is recorded here rather than in
the headline for that reason.

**The finding, stated plainly: Bengaluru's lack of route choice is not a missing
footpath. It is a missing line.** No walkable intervention exists, and the
cheapest one that does exist is redundant with what is already there. That is a
statement about a fast-growing megacity's network, obtained by measuring rather
than asserting, and it is a stronger result than a ranked list would have been.

---

## New York: 712 opportunities, and the top one is famous

| rank | station A | station B | metres | Δreach | Δgap |
|---|---|---|---|---|---|
| 1 | Times Sq-42 St | 42 St-Bryant Pk | 274 | **+12.31** | +11.68 |
| 2 | 34 St-Penn Station | 34 St-Herald Sq | 282 | +12.10 | +11.48 |
| 3 | Nevins St | DeKalb Av | 288 | +10.56 | +10.08 |
| 4 | Christopher St-Stonewall | W 4 St-Wash Sq | 236 | +10.43 | +9.93 |
| 5 | 50 St | 7 Av | 225 | +9.90 | +9.52 |

The best single footpath in New York takes the mean number of stations reachable
in 45 minutes from **73.07 to 85.37 — a 16.8% increase from one connection.**

The top two are real gaps that a New Yorker would recognise: Times Square and
Bryant Park are three blocks apart with no free transfer between them, and the
same is true of Penn Station and Herald Square. The method found them without
being told anything about the city.

Note the shape of the gain. The winner adds +12.31 stations at the full change
budget but only **+0.62** with no changes at all, so Δgap rises by +11.68. This is
an intervention that pays passengers who can change, and barely touches those who
cannot. That is not an argument against building it. It is an argument for
reporting both numbers, which is why the ranking never collapses them.

---

## A side effect: the method audits the feed

97 of the 712 New York candidates join two stations **with the same name** — among
them 59 St-Columbus Circle to 59 St-Columbus Circle, seventeen metres apart.

Those are not places to spend money. They are one station complex whose
`transfers.txt` does not join all of its platforms, so the complex arrives in the
data as two separate stations and the search dutifully proposes connecting a
station to itself. The interchange already exists; the missing thing is a data
record. They are held out of the ranking and reported separately, because a feed
gap of that size distorts every reachability number computed from the feed.

**The heuristic is suggestive, not decisive, and it fails in both directions.**
New York has several genuinely distinct stations called "14 St". More importantly,
it misses gaps between differently-named halves of a real complex. Bleecker St and
Broadway-Lafayette St have been connected by a free transfer since 2012. The feed
disagrees:

```
$ grep -E '^(637[NS]|D21[NS]),' transfers.txt
637N,637S,120        # Bleecker St, platform to platform
637S,637N,120
D21N,D21S,120        # Broadway-Lafayette St, platform to platform
D21S,D21N,120
```

Each station joins its own two platforms. Neither joins the other. So
Bleecker–Broadway-Lafayette appears at rank 10 of the proposals, and it should not:
that interchange was built fourteen years ago.

**This is the honest limit of the whole exercise.** From the feed alone the method
cannot distinguish *"this interchange does not exist"* from *"this interchange
exists and the feed omits it."* Every New York proposal needs checking against the
world before it means anything. Bengaluru's null result does not have this problem,
because the claim there is about distance rather than about connection.

---

## What is not claimed

- **Reachable stations, not reachable jobs or people.** The same limit
  `docs/accessibility.md` states, for the same reason: weighting destinations by
  what is at them needs land-use and demographic data joined to the network, and
  that is a data project of its own.
- **The walk is a straight line.** It knows nothing about roads, level changes,
  rivers, or whether a walkway could physically be built between two points.
  Straight-line distance under-states every real walk, so the model is if anything
  generous to candidates.
- **A footpath is not free of effort.** The model charges walking time but does not
  charge it as a "change", which is the right accounting for RAPTOR and the wrong
  accounting for a passenger who cannot walk 1200 m. See Bengaluru above.
- **Nothing here says where to build anything.** It ranks candidates on one clearly
  stated criterion, on data that is known to be incomplete in at least one
  direction.

---

## Running it

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target routing_engine_interchange_search -j

# the whole search, defaults: 800 m radius, 45-minute threshold, <=2 changes
./build/routing_engine_interchange_search <feed_dir> --labels

# stretch the radius to diagnose a network with no walkable candidates
./build/routing_engine_interchange_search <feed_dir> --max-walk 2500 --top 15
```

Options: `--max-walk METRES`, `--walk-speed MPS`, `--threshold SECONDS`,
`--max-changes N`, `--top N`, `--out-prefix PATH`, `--labels`.

Outputs `<prefix>-candidates.csv` — every candidate ranked, with `same_name` and
the full before/after/delta columns — and `<prefix>-sites.svg`, the network with
each station coloured by the best gain any candidate touching it achieved. The
proposed link is deliberately **not** drawn: the renderer draws every link
identically, and a proposed footpath painted like existing track would be a
picture of a network that does not exist.

Contracts are in `tests/test_interchange.cpp` (19 cases). The one worth knowing
about is `AFootpathNeverReducesReachability`: a footpath only ever adds an option,
and the transitive closure only ever shortens a walk, so no candidate may make any
measure worse. If a change to the closure, the station merge or the round
accounting breaks that, it is the test that says so first.
