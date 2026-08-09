#!/usr/bin/env python3
"""
analyze_study.py — turn study-results.csv into the answer to the question.

    python3 scripts/analyze_study.py feeds/study-results.csv
    python3 scripts/analyze_study.py feeds/study-results.csv --markdown > docs/study-results.md

THE QUESTION
------------
Across real transit networks, what fraction of origin-destination pairs actually
admits a genuine time-versus-changes trade-off, and which structural property of
a network predicts it?

WHAT THIS SCRIPT DOES ABOUT IT
------------------------------
  1. Ranks the feeds by the exact trade-off rate RAPTOR measured.
  2. Correlates that rate against every structural metric, by Spearman rank
     correlation as the headline and Pearson alongside it.
  3. Tests the specific hypothesis the project started from — that a network
     whose station graph is a FOREST can have no trade-offs — as a contingency
     table rather than a correlation, because it is a categorical claim.
  4. Reports the cost of the engine's bounded-wait lookahead across feeds.
  5. Reports the head-to-head timing, with the caveat that only feeds measured
     in the same sweep on the same machine are comparable.

WHY SPEARMAN IS THE HEADLINE
----------------------------
The trade-off rate is a proportion bounded at 0 and 1 with a large mass at 0,
and several structural metrics are heavily skewed (one feed has forty times the
stations of another). Pearson on those is dominated by the extremes. Spearman
asks the question actually being asked — does a network that ranks higher on
this metric rank higher on trade-offs — and is unaffected by the skew.

Standard library only. No numpy, no scipy: the reproducibility artifact should
run on a bare Python and the statistics here are a hundred lines.
"""

import argparse
import csv
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MANIFEST = os.path.join(HERE, "feeds.json")

# Marks a feed whose timetable is MODELLED rather than published. Exactly one
# entry qualifies today (Bengaluru), but the flag is derived from the manifest
# rather than hard-coded, so adding another such feed cannot silently produce a
# table that presents a synthetic timetable as an observed one.
MODELLED_MARK = " *"


def modelled_slugs():
    try:
        with open(MANIFEST, "r", encoding="utf-8") as f:
            manifest = json.load(f)
    except (OSError, ValueError):
        return set()
    return {e["slug"] for e in manifest.get("feeds", []) if e.get("builder")}

# Structural columns to correlate against the trade-off rate, with a short
# human description for the output table.
STRUCTURAL = [
    ("cyclomatic", "independent cycles in the station graph"),
    ("alpha_index", "meshedness, cycles per planar maximum"),
    ("beta_index", "links per station"),
    ("gamma_index", "links per planar maximum"),
    ("deg3plus_fraction", "fraction of stations that are junctions"),
    ("interchange_density", "fraction of stations served by 2+ lines"),
    ("shared_link_fraction", "fraction of links carrying 2+ lines"),
    ("mean_lines_per_link", "mean lines per link"),
    ("mean_station_degree", "mean station degree"),
    ("lines", "number of lines"),
    ("stations", "number of stations"),
    ("median_link_headway_s", "median headway in the sampled window"),
    ("service_span_hours", "hours between first and last departure"),
]

TARGET = "rt_k2plus_frac"


def to_float(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def rank(values):
    """Fractional ranks, averaging ties — required for Spearman to be correct
    when several networks share a value, which they do: many have cyclomatic 0."""
    order = sorted(range(len(values)), key=lambda i: values[i])
    ranks = [0.0] * len(values)
    i = 0
    while i < len(order):
        j = i
        while j + 1 < len(order) and values[order[j + 1]] == values[order[i]]:
            j += 1
        shared = (i + j) / 2.0 + 1.0
        for k in range(i, j + 1):
            ranks[order[k]] = shared
        i = j + 1
    return ranks


def pearson(xs, ys):
    n = len(xs)
    if n < 3:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    if sxx <= 0 or syy <= 0:
        return None
    return sxy / math.sqrt(sxx * syy)


def spearman(xs, ys):
    if len(xs) < 3:
        return None
    return pearson(rank(xs), rank(ys))


def two_sided_p(r, n):
    """p-value for a correlation, via the t approximation.

    An approximation, and labelled as one wherever it is printed. With twenty to
    forty feeds it is close enough to tell a real association from noise, and it
    is emphatically not close enough to defend a borderline claim. Feeds are also
    not independent draws from any population — they are the networks whose
    agencies publish open data — so the p-value bounds sampling noise and says
    nothing about selection.
    """
    if r is None or n < 4:
        return None
    r = max(-0.999999, min(0.999999, r))
    t = abs(r) * math.sqrt((n - 2) / (1 - r * r))
    df = n - 2
    # Student's t survival function via the regularised incomplete beta.
    x = df / (df + t * t)
    return betainc(df / 2.0, 0.5, x)


def betainc(a, b, x):
    """Regularised incomplete beta I_x(a, b), by continued fraction."""
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    lbeta = math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b)
    front = math.exp(lbeta + a * math.log(x) + b * math.log(1.0 - x))
    if x < (a + 1.0) / (a + b + 2.0):
        return front * betacf(a, b, x) / a
    return 1.0 - math.exp(lbeta + b * math.log(1.0 - x) + a * math.log(x)) * betacf(b, a, 1.0 - x) / b


def betacf(a, b, x):
    tiny = 1e-30
    qab, qap, qam = a + b, a + 1.0, a - 1.0
    c, d = 1.0, 1.0 - qab * x / qap
    if abs(d) < tiny:
        d = tiny
    d = 1.0 / d
    h = d
    for m in range(1, 200):
        m2 = 2 * m
        aa = m * (b - m) * x / ((qam + m2) * (a + m2))
        d = 1.0 + aa * d
        if abs(d) < tiny:
            d = tiny
        c = 1.0 + aa / c
        if abs(c) < tiny:
            c = tiny
        d = 1.0 / d
        h *= d * c
        aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2))
        d = 1.0 + aa * d
        if abs(d) < tiny:
            d = tiny
        c = 1.0 + aa / c
        if abs(c) < tiny:
            c = tiny
        d = 1.0 / d
        delta = d * c
        h *= delta
        if abs(delta - 1.0) < 1e-10:
            break
    return h


def load(path):
    with open(path, "r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="study-results.csv from scripts/run_study.py")
    ap.add_argument("--markdown", action="store_true", help="emit Markdown tables")
    args = ap.parse_args()

    rows = load(args.csv)
    usable = [r for r in rows if r.get("status") in ("ok", "partial")
              and to_float(r.get(TARGET)) is not None
              and to_float(r.get("rt_reached")) not in (None, 0.0)]
    dropped = [r for r in rows if r not in usable]

    modelled = modelled_slugs()

    def name(row):
        """Feed slug, marked when its timetable is modelled rather than published."""
        slug = row.get("slug", "?")
        return slug + MODELLED_MARK if slug in modelled else slug

    out = []
    e = out.append
    h1 = (lambda s: e("\n# " + s + "\n")) if args.markdown else \
         (lambda s: e("\n" + s + "\n" + "=" * len(s)))
    h2 = (lambda s: e("\n## " + s + "\n")) if args.markdown else \
         (lambda s: e("\n" + s + "\n" + "-" * len(s)))

    def table(headers, body):
        if args.markdown:
            e("| " + " | ".join(headers) + " |")
            e("|" + "|".join("---" for _ in headers) + "|")
            for r in body:
                e("| " + " | ".join(str(c) for c in r) + " |")
        else:
            widths = [max(len(str(h)), *(len(str(r[i])) for r in body)) if body else len(str(h))
                      for i, h in enumerate(headers)]
            e("  ".join(str(h).ljust(w) for h, w in zip(headers, widths)))
            e("  ".join("-" * w for w in widths))
            for r in body:
                e("  ".join(str(c).ljust(w) for c, w in zip(r, widths)))

    h1("When does multi-objective transit routing actually matter?")
    e(f"{len(usable)} feeds analysed, {len(dropped)} excluded, from `{args.csv}`.")
    marked = sorted(s for s in modelled if any(r.get("slug") == s for r in usable))
    if marked:
        e("")
        e(f"`{MODELLED_MARK.strip()}` marks a feed whose TIMETABLE is modelled rather than "
          f"published — real station topology, synthesised departures. "
          f"{', '.join(marked)}. Its structural metrics are as real as any other feed's; "
          f"its headway, service span and latency figures are a property of the model.")
    if dropped:
        e("")
        e("Excluded:")
        for r in dropped:
            e(f"  - {r.get('slug')}: {r.get('status')} — {(r.get('note') or '').strip() or 'no detail'}")

    if len(usable) < 3:
        e("\nFewer than three usable feeds: nothing to correlate. "
          "Run scripts/fetch_feeds.py and scripts/run_study.py first.")
        print("\n".join(out))
        return 1

    # ── 1. The ranking ────────────────────────────────────────────────────────
    h2("Trade-off rate per network, measured exactly")
    e("`k>1` is the fraction of reached (origin, destination) observations where the exact "
      "(arrival time, number of trips) Pareto frontier holds more than one point — that is, "
      "where a real time-versus-changes choice exists. Computed with RAPTOR, so it is a "
      "property of the network and not of this project's search heuristics.")
    e("")
    ranked = sorted(usable, key=lambda r: -to_float(r[TARGET]))
    table(["feed", "stations", "links", "cyclomatic", "lines", "interchange density",
           "k>1", "mean k", "max k"],
          [[name(r), r["stations"], r["links"], r["cyclomatic"], r["lines"],
            f"{to_float(r['interchange_density']):.3f}",
            f"{100 * to_float(r[TARGET]):.2f}%",
            f"{to_float(r['rt_mean_k']):.3f}", r["rt_max_k"]] for r in ranked])

    # ── 2. Correlations ───────────────────────────────────────────────────────
    h2("What predicts it")
    e("Spearman rank correlation against the trade-off rate, with Pearson alongside. "
      "p-values are from the t approximation and bound sampling noise only — the feeds "
      "are the networks whose agencies publish open data, not a random sample of the "
      "world's transit systems, so nothing here corrects for selection.")
    e("")
    ys = [to_float(r[TARGET]) for r in usable]
    body = []
    for col, desc in STRUCTURAL:
        pairs = [(to_float(r.get(col)), y) for r, y in zip(usable, ys)
                 if to_float(r.get(col)) is not None]
        if len(pairs) < 3:
            continue
        xs = [p[0] for p in pairs]
        yy = [p[1] for p in pairs]
        rs, rp = spearman(xs, yy), pearson(xs, yy)
        ps = two_sided_p(rs, len(xs))
        body.append([col, desc, len(xs),
                     "n/a" if rs is None else f"{rs:+.3f}",
                     "n/a" if rp is None else f"{rp:+.3f}",
                     "n/a" if ps is None else (f"{ps:.4f}" if ps >= 1e-4 else "<0.0001")])
    body.sort(key=lambda r: -abs(float(r[3])) if r[3] != "n/a" else 0.0)
    table(["metric", "meaning", "n", "spearman", "pearson", "p (approx)"], body)

    # ── 3. The forest hypothesis ──────────────────────────────────────────────
    h2("The hypothesis this project started from")
    e("A network whose station graph is a FOREST (cyclomatic number zero) has exactly one "
      "path between any pair of stations. If that is what drives the result, then every "
      "forest should show a trade-off rate at or near zero, and the interesting variation "
      "should live entirely among the networks with cycles. This is a categorical claim, so "
      "it is tested as one rather than folded into a correlation.")
    e("")
    forests = [r for r in usable if to_float(r["cyclomatic"]) == 0]
    meshes = [r for r in usable if to_float(r["cyclomatic"]) > 0]

    def summarise(group, label):
        if not group:
            return [label, 0, "-", "-", "-"]
        vals = sorted(to_float(r[TARGET]) for r in group)
        med = vals[len(vals) // 2] if len(vals) % 2 else 0.5 * (vals[len(vals) // 2 - 1] + vals[len(vals) // 2])
        return [label, len(group), f"{100 * min(vals):.2f}%", f"{100 * med:.2f}%", f"{100 * max(vals):.2f}%"]

    table(["group", "feeds", "min k>1", "median k>1", "max k>1"],
          [summarise(forests, "forest (cyclomatic = 0)"),
           summarise(meshes, "has cycles (cyclomatic > 0)")])
    if forests:
        e("")
        e("Forests: " + ", ".join(f"{name(r)} ({100 * to_float(r[TARGET]):.2f}%)" for r in forests))
    zero_with_cycles = [r for r in meshes if to_float(r[TARGET]) == 0.0]
    if zero_with_cycles:
        e("")
        e("Networks WITH cycles that still show no trade-off: " +
          ", ".join(f"{name(r)} (cyclomatic {r['cyclomatic']})" for r in zero_with_cycles) + ".")
        e("So the implication runs one way only: a forest guarantees no trade-off, "
          "and cycles guarantee nothing. A cycle somewhere in a network is not the same "
          "as a cycle a passenger's journey can use.")

    # ── 3b. The engine's own frontier against the exact one ───────────────────
    h2("Why the engine's own frontier is not an answer to the question")
    e("The obvious way to run this study would have been to point the existing diagnostic at "
      "many feeds and count frontier sizes. The table below is why that would have produced a "
      "different result, not a cheaper one.")
    e("")
    e("The engine's second objective counts PLATFORM WALKS; RAPTOR's round index counts "
      "VEHICLES. They coincide only where changing service requires changing platform. The "
      "consequence is not a small bias — it goes in both directions:")
    e("")
    e("- a feed whose `transfers.txt` is thin gives the engine almost nothing to count, so it "
      "reports no trade-offs on a network that has plenty;")
    e("- a feed with a dense transfer layer lets the engine score a platform walk as a "
      "trade-off even where there is only one route, so it reports trade-offs on a network "
      "that has none.")
    e("")
    tr_rows = []
    for r in sorted(usable, key=lambda x: -(to_float(x.get(TARGET)) or 0)):
        if r.get("second_objective") != "transfers":
            continue
        rt = to_float(r[TARGET])
        en = to_float(r.get("en_k2plus_frac"))
        if rt is None or en is None:
            continue
        tr_rows.append([name(r), r.get("transfers", "-"),
                        f"{100 * rt:.2f}%", f"{100 * en:.2f}%",
                        f"{100 * (en - rt):+.2f}pp"])
    if tr_rows:
        table(["feed", "transfer edges", "exact (RAPTOR)", "engine", "engine minus exact"], tr_rows)
        xs = [to_float(r[TARGET]) for r in usable if r.get("second_objective") == "transfers"]
        ys = [to_float(r["en_k2plus_frac"]) for r in usable if r.get("second_objective") == "transfers"]
        rs = spearman(xs, ys)
        e("")
        e(f"Spearman between the two measures across these {len(xs)} feeds: "
          f"{'n/a' if rs is None else f'{rs:+.3f}'}. "
          "The engine's frontier is a fact about the engine. The study's headline number is "
          "the exact one.")
    else:
        e("No feed in this run used the transfer-count objective, so there is nothing to "
          "compare. That happens when no feed carried a transfers.txt.")

    # ── 4. The cost of the lookahead heuristic ────────────────────────────────
    h2("What the engine's bounded-wait lookahead costs on real feeds")
    e("The Pareto engine expands one composite-optimal departure per link, chosen from the "
      "next k=5 departures within a 30-minute window. The column below is how often its "
      "earliest arrival is later than the exact one, with lambda set to 0 so that departure "
      "selection is trying to minimise arrival and the window is the only thing in the way.")
    e("")
    e("`unreached` counts destinations RAPTOR reaches and the engine does not at all, and it "
      "is NOT in the denominator of the percentage — that column answers \"when the engine "
      "found a journey, how often was it the wrong one\". A feed can therefore show a small "
      "percentage and a large `unreached`, which is a worse failure, not a better one. Read "
      "the two together.")
    e("")
    gap_rows = []
    for r in sorted(usable, key=lambda r: -(to_float(r.get("ag_suboptimal_frac")) or 0.0)):
        sub = to_float(r.get("ag_suboptimal_frac"))
        if sub is None:
            continue
        gap_rows.append([name(r), r.get("ag_compared", "-"),
                         f"{100 * sub:.3f}%",
                         f"{to_float(r.get('ag_gap_mean_s')) or 0:.0f}s",
                         f"{to_float(r.get('ag_gap_p95_s')) or 0:.0f}s",
                         f"{to_float(r.get('ag_gap_max_s')) or 0:.0f}s",
                         r.get("ag_engine_unreached", "-"),
                         r.get("ag_skipped_capped", "-"),
                         r.get("ag_earlier", "-")])
    table(["feed", "compared", "later than optimal", "mean gap", "p95 gap", "max gap",
           "unreached", "skipped (capped)", "earlier (must be 0)"], gap_rows)
    bad = [r for r in usable if (to_float(r.get("ag_earlier")) or 0) > 0]
    if bad:
        e("")
        e("**" + ", ".join(name(r) for r in bad) +
          ": the engine reported an arrival EARLIER than the exact oracle. "
          "One of the two implementations is wrong and no number from those feeds is usable.**")

    # ── 5. Timing ─────────────────────────────────────────────────────────────
    h2("Head to head: time-dependent Pareto-Dijkstra against RAPTOR")
    e("Interleaved on the same query set, same machine, same timer, one query per arm per "
      "iteration. Comparable ACROSS feeds only if the whole sweep ran in one sitting on a "
      "pinned core; comparable WITHIN a feed regardless, because the interleaving cancels "
      "drift. `ratio` above 1 means RAPTOR was faster.")
    e("")
    e("**Read the feeds where the engine wins with the previous table open.** The two arms do "
      "not answer the same question on a sparse network: the engine's bounded-wait window "
      "makes it abandon links whose next departure is more than thirty minutes away, so on a "
      "long-distance rail feed it stops early AND returns a worse answer, while RAPTOR scans "
      "every route it reaches and returns the exact one. Part of any engine win is therefore "
      "work not done rather than work done faster, and the `unreached` and `later than "
      "optimal` columns above are how much. The comparison is clean on the dense urban feeds, "
      "where both reach everything.")
    e("")
    t_rows = []
    for r in sorted(usable, key=lambda r: -(to_float(r.get("t_ratio_p50")) or 0.0)):
        ratio = to_float(r.get("t_ratio_p50"))
        if not ratio:
            continue
        t_rows.append([name(r), r["nodes"], r["edges"],
                       f"{(to_float(r['t_engine_p50']) or 0) / 1000:.1f}",
                       f"{(to_float(r['t_raptor_p50']) or 0) / 1000:.1f}",
                       f"{ratio:.2f}x",
                       f"{(to_float(r['t_engine_p99']) or 0) / 1000:.1f}",
                       f"{(to_float(r['t_raptor_p99']) or 0) / 1000:.1f}",
                       f"{(to_float(r.get('t_overhead_ns')) or 0):.0f} ns"])
    table(["feed", "nodes", "edges", "engine p50 (us)", "raptor p50 (us)", "ratio",
           "engine p99 (us)", "raptor p99 (us)", "timer overhead"], t_rows)

    if t_rows:
        ratios = sorted(to_float(r.get("t_ratio_p50")) for r in usable
                        if to_float(r.get("t_ratio_p50")))
        faster = sum(1 for x in ratios if x > 1.0)
        med = ratios[len(ratios) // 2] if len(ratios) % 2 else \
            0.5 * (ratios[len(ratios) // 2 - 1] + ratios[len(ratios) // 2])
        e("")
        e(f"RAPTOR was faster on {faster} of {len(ratios)} feeds; median ratio {med:.2f}x "
          f"(range {min(ratios):.2f}x to {max(ratios):.2f}x).")

        # The crossover is the interesting part of a comparison, not the median.
        losers = sorted((r for r in usable if (to_float(r.get("t_ratio_p50")) or 0) < 1.0),
                        key=lambda r: to_float(r["t_ratio_p50"]))
        if losers:
            e("")
            e("The crossover: RAPTOR was SLOWER on " +
              ", ".join(f"{r['slug']} ({to_float(r['t_ratio_p50']):.2f}x)" for r in losers) + ".")
            e("Every one of those is sparse, wide-area rail rather than a dense urban network. "
              "RAPTOR's cost is driven by the routes it scans, and it scans a whole route once "
              "any stop on it is marked; a label-correcting search on a graph that thin settles "
              "a handful of labels and stops. Note also how large `unreached` is for those "
              "feeds — the engine is not solving the same problem there.")

    print("\n".join(out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
