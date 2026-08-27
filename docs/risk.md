# Does dominance pruning survive a risk objective?

`include/risk.hpp` · `src/risk.cpp` · `tools/risk_probe.cpp` · `tests/test_risk.cpp`

---

The README and `docs/write-up.tex` §4 both say the same thing, correctly, and
then stop. The engine's second objective is deterministic and linear; transit
delay distributions are right-skewed, so variance is the wrong risk measure;
**Conditional Value at Risk is the coherent choice**. Both then record the
extension as future work.

This is that future work, reduced to the one question that has to be answered
before any of it is worth building:

> The engine is fast because it throws labels away. A label is discarded when
> another label at the same node is no worse on every objective, and that is safe
> because the cost of a journey is the sum of the costs of its legs — worse now
> stays worse later. **CVaR is not additive along a path. So what exactly breaks,
> and what is the weakest thing a label can carry that still makes pruning safe?**

The answer turns out not to be about CVaR, and that is the finding.

Run of **27 August 2026**. Nothing here reads a feed and nothing here uses delay
data, by design — see [what is not claimed](#what-is-not-claimed). Every instance
is synthetic and generated from a stated seed, so every number below is
reproducible with one command and no downloads.

---

## The model

A **scenario set** Ω = {0 … n−1}, equally likely, carries the whole joint law of
every delay in the network. Everything is a deterministic function of the
scenario, so a distribution is a vector of integers, a mean is a sum, a CVaR is a
partial sum, and no comparison anywhere goes through a float.

A **leg** is a periodic service plus two per-scenario perturbations:

```
ready[ω]  = arrival[ω] + min_transfer
dep[ω]    = the earliest  offset + k·headway + shift[ω]  that is ≥ ready[ω]
arrive[ω] = dep[ω] + ride + delay[ω]
```

`delay` is this ride's own lateness. `shift` moves every departure of the
connecting service together — one line running late all day — and it is what
makes a missed connection *correlate* with the thing that caused it.

For each fixed scenario the map from arrival to arrival is a **non-decreasing
step function**, and that single fact drives everything below. It is also exactly
the FIFO property the deterministic engine already relies on
(`tests/test_fifo_violation.cpp`, `docs/write-up.tex` §2), stated per scenario
instead of once. This is the stochastic continuation of that argument, not a new
subject.

---

## Five orders, and where safety is lost

A pruning rule is an order on labels. Five are implemented. They form a chain —
each orders strictly more pairs than the one below, so each prunes strictly
harder, and `RiskProbe.TheOrdersFormAChain` asserts the inclusions on randomised
pairs.

| Order | A label carries | Prunes | Safe? |
|---|---|---|---|
| **none** | — | nothing | trivially |
| **statewise** | the arrival in every scenario | no later in *every scenario* | **always** |
| **stochastic** | the arrival distribution | no later at *every quantile* | **only if delays are independent of the prefix** |
| **all tail averages** | the arrival distribution | no worse in CVaR at *every* confidence level | **no** |
| **scalar** | mean and CVaR at one level | no worse on both numbers | **no** |

The bottom row is the engine's own rule transplanted: two numbers per label, both
must be no worse. The rows above it are the successive repairs, and the result is
that the repair has to go two rungs further up than it looks like it should.

---

## 1 — A scalar risk label cannot be accumulated at all

Two journeys arriving at an interchange over four equally likely days:

```
route A   [5400 5400 6060 6060]    mean 5730.0    CVaR 6060.0
route B   [5400 5400 5940 6180]    mean 5730.0    CVaR 6060.0
```

They agree **exactly** on both numbers a scalar risk label would carry. One more
leg — a connection at 6000 with a half-hour headway — and they agree on neither:

```
route A   [6000 6000 7800 7800]    mean 6900.0    CVaR 7800.0
route B   [6000 6000 6000 7800]    mean 6450.0    CVaR 6900.0
```

So there is **no function of any shape** taking (mean, CVaR, the next leg) to the
extended (mean, CVaR). This is strictly stronger than the familiar observation
that CVaR is not additive: no cleverer accumulation formula can exist, because
the two numbers are not a sufficient statistic for their own future.

And the cause is not CVaR. The mean is as additive as a quantity gets, and it
fails here too, because arrival is a *step function* of arrival — `E[f(T)]` is
not a function of `E[T]`. **It is scalarisation that breaks, not coherence.**

Pinned by `RiskProbe.NoAccumulationRuleForMeanAndCvar`, and the corresponding
search failure by `RiskProbe.ScalarPruningDiscardsThePrefixOfTheOptimum`: route B
is worse on *both* numbers at the interchange and better on *both* at the
destination, so the rule discards the prefix of the optimum and returns a CVaR of
7500 s where the answer is 6600 s.

---

## 2 — Ordering by CVaR at *every* level is still not enough

This is the result the artifact exists for.

If the objective is CVaR, the natural repair is to order by CVaR — all of it, at
every confidence level at once. That is not a summary: knowing CVaR at every
level *is* knowing the distribution, since successive tail totals differ by the
sorted values (`RiskProbe.CvarAtEveryLevelDeterminesTheDistribution`). Nothing is
being thrown away.

It is still wrong.

```
                             route A              route B
at the interchange     [600 1140 1200]      [600 900 1440]
  CVaR, worst 1                  1200.0               1440.0
  CVaR, worst 2                  1170.0               1170.0
  CVaR, worst 3 (mean)            980.0                980.0
```

Route A is no worse at every level, so the rule discards B. After one connection:

```
at the destination    [1200 3000 3000]     [1200 1200 3000]
  CVaR, worst 2                  3000.0               2100.0
```

B was the better journey by 900 seconds.

The reason is exact and general. Dominance in all tail averages is the
*increasing convex order*, which is preserved by non-decreasing **convex** maps.
A next-departure map is non-decreasing and emphatically not convex — it is a
staircase. So the order does not survive the one operation a router performs.

**Pruning has to be done in an order strictly stronger than the objective it
serves.** Ordering by CVaR is not enough to optimise CVaR.

---

## 3 — First-order stochastic dominance is the right order, and it is the *only* one

**Sufficient.** If one label's arrival distribution is no later than another's at
every quantile then it stays that way after any continuation: a non-decreasing
map preserves the order, adding an independent delay preserves it, and every
CVaR is monotone under it. So the surviving label is at least as good after any
continuation, **at every confidence level simultaneously** — one search, exact
for the whole family of objectives rather than for one chosen α.

Checked, not asserted: `StochasticPruningMatchesEnumerationOnIndependentInstances`
runs the pruned search and an exhaustive enumeration over the same journey set on
60 randomised networks and requires them to agree at *every* tail count the
scenario set admits.

**Necessary.** For any pair the order does not relate, `separating_leg()`
constructs an ordinary timetable that reverses them in the mean: put a departure
exactly at the crossing, and everything above it waits a full headway.
`RiskProbe.StochasticDominanceIsNecessary` builds that leg for every unrelated
pair it draws and checks the reversal. Nothing weaker is safe.

Taken together, over the family {CVaR at every α}, **dominance pruning is safe if
and only if the order is first-order stochastic dominance.** It is not a design
choice.

---

## 4 — Correlated delays cost the marginal its sufficiency

Sufficiency needed the leg's delay to be independent of how the passenger
arrived. Real delays are not: a late feeder and a late connection are late
together.

Three days. The connecting line runs 600 s late on day 0 and on time otherwise.

```
                          route A              route B
at the interchange  [1500 1500 1900]     [2000 1500 1500]
  stochastically              earlier               later    -> B is discarded
at the destination  [2450 1850 3650]     [2450 1850 1850]
  CVaR, worst 1                3650.0              2450.0
```

Route B is stochastically *later* and is the better journey, because it is late
exactly on the day its connection is late, so it never misses it. Route A is
punctual on the disrupted day and late on a normal one, and misses.

No property of the arrival distribution alone can see that. In this regime the
marginal is not a sufficient statistic at all, and the safe order drops to
statewise dominance — no later in every scenario — which is weaker still and
therefore prunes less.

`RiskProbe.CorrelatedDelaysCostTheMarginalItsSufficiency` pins the witness;
`RiskProbe.StatewisePruningMatchesEnumerationUnderDisruption` sweeps 60 disrupted
networks and finds statewise pruning exact on all of them.

---

## 5 — Summing per-leg CVaR is not a bound

`docs/write-up.tex` proposes exactly one extension: replace the linear crowd term
with an empirical CVaR per edge. It is the obvious move, because a sum is
additive and an additive objective is prunable by machinery that already exists.

Coherence appears to make it safe — CVaR(ΣXᵢ) ≤ Σ CVaR(Xᵢ), so per-leg risk looks
conservative. **A journey's arrival is not a sum of random costs.** A delay that
causes a missed connection costs a whole headway, not the delay:

```
arrival           [1260 3060]
true CVaR          3060.0 s
additive proxy     1380.0 s      under-states by 1680 s
```

Two minutes of lateness, and the bill is half an hour. The proxy sits far *below*
the true risk, and its error is largest exactly where the risk is.
`RiskProbe.AdditiveProxyIsNotABound`.

---

## How often it actually costs something

400 random networks per row, each measured against exhaustive enumeration over
the same journey set; the count is the number of networks on which the rule
returned something worse than the true optimum. Disruption is the number of
seconds the whole network runs late on half of all scenarios, against a 900 s
headway. Zero is the independent regime the sufficiency proof covers.

| disruption | statewise | stochastic | all tail averages | scalar |
|---|---|---|---|---|
| 0 s | 0/400 | 0/400 | 0/400 | 0/400 |
| 112 s | 0/400 | 1/400 | 3/400 | 3/400 |
| 225 s | 0/400 | 3/400 | 6/400 | 7/400 |
| 337 s | 0/400 | 8/400 | 12/400 | 13/400 |
| **450 s** (half a headway) | 0/400 | **10/400** | **14/400** | **16/400** |
| 562 s | 0/400 | 4/400 | 10/400 | 10/400 |
| 675 s | 0/400 | 3/400 | 9/400 | 9/400 |
| 900 s (one headway) | 0/400 | 0/400 | 0/400 | 0/400 |

**Under independent delays the unsafe rules never lost the optimum**, on any of
these 400 networks, despite taking 2.38 prunings per network (scalar) and 2.23
(all tail averages) that the sufficiency proof does not cover. The failure needs the two arrival laws to cross
in a particular alignment with a departure boundary, and a random timetable
rarely provides one. That is the same shape of answer as the FIFO probe's
(README, Measured Behaviour §5): **proved possible, and latent rather than
frequent.**

Correlated disruption provides the alignment systematically, because the
disruption moves the boundary. And note the last row: a disruption of exactly one
headway leaves every departure where it was, and the failure rate returns to
zero. **It is the phase that costs the marginal its sufficiency, not the
lateness** — `RiskProbe.ADisruptionOfAWholeHeadwayIsNoDisruptionAtAll` pins that,
and it is what rules out the extra scenario coordinate as the cause.

The disruption model is the mildest imaginable: half the days, everything runs
late by the same amount, so every journey is simply translated. That is enough.

---

## What it costs

Mean labels per reached node, delays independent, 400 networks per row.

| delay spread | none | statewise | stochastic | all tail averages | scalar |
|---|---|---|---|---|---|
| 0 s | 11.75 | 1.00 | 1.00 | 1.00 | 1.00 |
| 150 s | 11.75 | 1.26 | 1.14 | 1.05 | 1.04 |
| 300 s | 11.75 | 1.50 | 1.25 | 1.09 | 1.08 |
| 600 s | 11.75 | 1.95 | 1.41 | 1.14 | 1.12 |
| 900 s | 11.75 | 2.25 | 1.51 | 1.20 | 1.17 |
| 1800 s | 11.75 | 3.07 | 1.67 | 1.23 | 1.20 |
| 3600 s | 11.75 | 4.63 | 1.85 | 1.25 | 1.21 |

**The safe order is affordable.** At a delay spread of twice the headway,
stochastic dominance holds 1.67 labels per node against the unsafe scalar rule's
1.20 — about 40% more, for exactness at every confidence level at once. Set
against the engine's own frontier behaviour on real feeds — one label at 96–100%
of nodes — that is a real cost and not a prohibitive one.

The question that decides whether the direction scales is different: does the
frontier grow with how finely the delay distribution is resolved? A label carries
the whole distribution, so this is the axis that would sink it.

| atoms per leg | scenarios | none | statewise | stochastic | all tail averages | scalar |
|---|---|---|---|---|---|---|
| 2 | 8 | 11.75 | 1.85 | 1.39 | 1.14 | 1.13 |
| 3 | 27 | 11.75 | 2.54 | 1.58 | 1.17 | 1.15 |
| 4 | 64 | 11.75 | 3.07 | 1.67 | 1.23 | 1.20 |
| 5 | 125 | 11.75 | 3.57 | 1.71 | 1.23 | 1.18 |
| 6 | 216 | 11.75 | 4.05 | 1.73 | 1.25 | 1.20 |
| 8 | 512 | 11.75 | 5.03 | 1.75 | 1.27 | 1.20 |

**Stochastic dominance saturates and statewise dominance does not.** A 64-fold
increase in resolution — 8 scenarios to 512 — costs the stochastic frontier 26%
and the statewise frontier 172%, still climbing.

That is the practical answer, and it splits along exactly the line the theory
does:

- **Independent delays.** Carry the distribution, prune by stochastic dominance.
  Exact for every confidence level at once, about 40% more labels per node, and
  the cost barely responds to resolution. This is buildable.
- **Correlated delays.** The safe order is statewise, and its cost grows with the
  resolution of the joint law — which is precisely the thing you would need to
  resolve finely to model correlated disruption in the first place. **That is the
  open problem, and it is where the interesting work would start.**

---

## What this means for the engine

Nothing changes in `src/routing.cpp`. Not one line of the engine was touched, and
no measurement it has published is affected — the engine's timetable is
deterministic, where its dominance rule is correct and where FIFO is the whole of
the story.

What changes is the shape of the extension the write-up gestures at. Adding a
risk objective is not a matter of swapping the second cost term:

- the label stops being two integers and becomes a distribution;
- the frontier stops being a sorted vector with an invariant and becomes an
  antichain in a partial order;
- the priority queue loses its key, because a partial order has nothing to settle
  on — the search here terminates on a leg bound rather than on a monotone key,
  and what a real implementation would cost is a question this probe does not
  answer;
- and the per-edge CVaR the write-up proposes is neither the objective nor a
  bound on it.

`docs/write-up.tex` §4 has been corrected to point here.

---

## What is not claimed

- **No real delay data is used, and none is needed.** Every claim is structural —
  which orders survive composition with a timetable — and a structural claim
  cannot be made true by realistic numbers or false by unrealistic ones. Inventing
  a delay distribution and reporting the routes that fall out of it would repeat
  the mistake `docs/crowd-model.md` is the repair to.
- **No route here is a recommendation.** The instances are three to five nodes,
  chosen so that exhaustive enumeration is affordable, because a claim of
  exactness is worth only as much as the oracle behind it.
- **The frontier growth is a trend on synthetic networks**, not a projection to a
  city feed. It says how the cost responds to delay spread and to distribution
  resolution, on networks whose structure was generated rather than measured.
- **The independence hypothesis is a modelling choice, not a fact about
  transit.** It is stated because the proof needs it, and section 4 exists
  because reality does not supply it.
- **A finite scenario set is an approximation of a delay law**, and every number
  here is exact *for that set*. Nothing is claimed about continuous
  distributions.

---

## Running it

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target routing_engine_risk_probe -j

./build/routing_engine_risk_probe                    # ~17 s at the defaults
./build/routing_engine_risk_probe --trials 50        # the counterexamples, quickly
```

Options: `--trials N`, `--nodes N`, `--out-degree N`, `--headway S`,
`--out-prefix PATH`, `--no-csv`.

The counterexamples run in milliseconds and need no arguments. Writes
`<prefix>-orders.csv`: one row per (sweep, parameter value, order), with the
inexactness count, the frontier statistics, the number of prunings the
sufficiency proof does not cover, and the mean regret in seconds.

Contracts are in `tests/test_risk.cpp` (20 cases). The counterexamples are
constructed by `include/risk.hpp` and used by both the tool and the tests, so the
witness the probe prints and the witness the suite asserts cannot come to mean
two different things.
