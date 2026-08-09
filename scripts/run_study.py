#!/usr/bin/env python3
"""
run_study.py — run tools/study on every normalised feed and collect one CSV.

    python3 scripts/run_study.py --workdir feeds --build build
    python3 scripts/run_study.py --workdir feeds --build build --only bart,cta-chicago
    python3 scripts/run_study.py --workdir feeds --build build --pin-core 3

Reads   <workdir>/norm/<slug>/          (produced by scripts/fetch_feeds.py)
Writes  <workdir>/study-results.csv     one row per feed, plus
        <workdir>/reports/<slug>.txt    the full human-readable report

WHY A RUNNER RATHER THAN A SHELL LOOP
-------------------------------------
Three things a loop gets wrong and this does not:

  * A feed that crashes, hangs or exhausts the router's arena must not end the
    sweep. Each feed runs as its own process with its own timeout, and a failure
    becomes a row with a status, not a missing row. A study that silently omits
    the feeds it could not process is reporting a biased sample.
  * Every row must have the same columns in the same order. The header comes
    from the binary itself (`--header`), so the two cannot drift.
  * The order in which feeds run must not change the numbers. Each feed is a
    fresh process, so nothing carries over.

TIMING CAVEAT, STATED HERE BECAUSE IT IS EASY TO FORGET
-------------------------------------------------------
The latency columns are only comparable across feeds if every feed was measured
under the same conditions. Use --pin-core to keep the sweep on one core, run it
on AC power, and do not use the machine while it runs. The project has already
measured what happens otherwise: on battery, p50 roughly doubles. The structural
and frontier columns are unaffected by any of that; only the timing columns are.
"""

import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)


def log(msg):
    print(msg, flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--workdir", default="feeds")
    ap.add_argument("--build", default="build", help="cmake build directory")
    ap.add_argument("--only", default="", help="comma-separated slugs")
    ap.add_argument("--skip", default="", help="comma-separated slugs")
    ap.add_argument("--queries", type=int, default=200)
    ap.add_argument("--timed", type=int, default=0, help="0 = calibrate to a time budget")
    ap.add_argument("--seed", type=int, default=123)
    ap.add_argument("--timeout", type=int, default=1800, help="seconds per feed")
    ap.add_argument("--pin-core", type=int, default=-1,
                    help="taskset the study to this core (recommended for the timing columns)")
    ap.add_argument("--out", default="", help="output CSV (default <workdir>/study-results.csv)")
    args = ap.parse_args()

    workdir = os.path.abspath(args.workdir)
    norm_root = os.path.join(workdir, "norm")
    if not os.path.isdir(norm_root):
        log(f"ERROR: {norm_root} does not exist. Run scripts/fetch_feeds.py first.")
        return 2

    binary = os.path.join(os.path.abspath(args.build), "routing_engine_study")
    if not os.path.exists(binary):
        log(f"ERROR: {binary} not found. Build it:\n"
            f"  cmake -S . -B {args.build} -DCMAKE_BUILD_TYPE=Release && "
            f"cmake --build {args.build} --target routing_engine_study")
        return 2

    only = {s for s in args.only.split(",") if s}
    skip = {s for s in args.skip.split(",") if s}
    slugs = sorted(d for d in os.listdir(norm_root)
                   if os.path.isdir(os.path.join(norm_root, d)))
    slugs = [s for s in slugs if (not only or s in only) and s not in skip]
    if not slugs:
        log("no feeds to run")
        return 1

    header = subprocess.run([binary, "--header"], capture_output=True, text=True,
                            check=True).stdout.strip()

    reports = os.path.join(workdir, "reports")
    os.makedirs(reports, exist_ok=True)
    out_path = args.out or os.path.join(workdir, "study-results.csv")

    rows = []
    for i, slug in enumerate(slugs, 1):
        feed = os.path.join(norm_root, slug)
        cmd = []
        if args.pin_core >= 0:
            cmd += ["taskset", "-c", str(args.pin_core)]
        cmd += [binary, feed, "--slug", slug,
                "--queries", str(args.queries), "--seed", str(args.seed)]
        if args.timed:
            cmd += ["--timed", str(args.timed)]

        log(f"[{i}/{len(slugs)}] {slug}")
        t0 = time.time()
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=args.timeout)
            stdout, stderr, rc = proc.stdout, proc.stderr, proc.returncode
        except subprocess.TimeoutExpired:
            stdout, stderr, rc = "", f"timed out after {args.timeout}s", -1

        with open(os.path.join(reports, slug + ".txt"), "w", encoding="utf-8") as f:
            f.write(stdout)
            if stderr:
                f.write("\n--- stderr ---\n")
                f.write(stderr)

        row = next((ln[4:] for ln in stdout.splitlines() if ln.startswith("CSV:")), None)
        if row is None:
            # No row at all means the binary died before it could report. Emit a
            # padded row so the feed is visible in the results as a failure.
            pad = "," * (header.count(",") - 2)
            reason = (stderr.strip().splitlines() or ["no output"])[-1][:200]
            reason = reason.replace(",", " ").replace('"', " ")
            row = f"{slug},crashed{pad},{reason}"
            log(f"      CRASHED (rc={rc}) — {reason}")
        else:
            status = row.split(",")[1]
            log(f"      {status} in {time.time() - t0:.1f}s")
        if rc == 3:
            log("      !! the engine reported an arrival EARLIER than the exact oracle. "
                "Investigate before using any number from this feed.")
        rows.append(row)

    with open(out_path, "w", encoding="utf-8", newline="\n") as f:
        f.write(header + "\n")
        for r in rows:
            f.write(r + "\n")

    log("")
    log(f"wrote {out_path} ({len(rows)} rows)")
    log(f"reports in {reports}/")
    log("next: python3 scripts/analyze_study.py " + out_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
