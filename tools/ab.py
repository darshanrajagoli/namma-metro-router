#!/usr/bin/env python3
"""Interleaved A/B: arena + output-parameter API vs a plain heap allocator.

Builds nothing itself. Configure with -DBUILD_BASELINE=ON first:

    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_BASELINE=ON
    cmake --build build
    python3 tools/ab.py ./data 7 build

The two binaries are run ALTERNATELY rather than in two blocks, so any clock
drift during the session (thermal, Turbo, hypervisor scheduling) lands on both
arms equally. Only the medians of each arm are compared.

Run on AC power: on battery every figure roughly doubles. See README.md.
"""
import os
import re
import statistics
import subprocess
import sys

FEED = sys.argv[1] if len(sys.argv) > 1 else "./data"
PAIRS = int(sys.argv[2]) if len(sys.argv) > 2 else 7
BUILD = sys.argv[3] if len(sys.argv) > 3 else "build"

OPT = os.path.join(BUILD, "routing_engine_benchmark")
BASE = os.path.join(BUILD, "routing_engine_baseline")

for path in (OPT, BASE):
    if not os.path.exists(path):
        sys.exit(f"missing {path}\n"
                 f"configure with: cmake -S . -B {BUILD} "
                 f"-DCMAKE_BUILD_TYPE=Release -DBUILD_BASELINE=ON")

PAT = {k: re.compile(rf"{k}\s+:\s+([\d.]+)") for k in ("p50", "p95", "p99", "Max")}


def run(binary):
    out = subprocess.run([binary, FEED], capture_output=True, text=True,
                         timeout=600).stdout
    vals = {}
    for key, pat in PAT.items():
        m = pat.search(out)
        if not m:
            sys.exit(f"could not parse {key} from {binary}")
        vals[key] = float(m.group(1))
    return vals


opt_runs, base_runs = [], []
for i in range(PAIRS):
    o, b = run(OPT), run(BASE)          # interleaved, in this order, every pair
    opt_runs.append(o)
    base_runs.append(b)
    print(f"  pair {i+1}: opt p50={o['p50']:>9.0f}  p99={o['p99']:>9.0f}   "
          f"base p50={b['p50']:>9.0f}  p99={b['p99']:>9.0f}")

print(f"\nfeed: {FEED}   ({PAIRS} interleaved pairs, medians below)")
print(f"{'metric':>6} {'arena+outparam':>16} {'heap baseline':>15} {'delta':>10}")
for k in ("p50", "p95", "p99", "Max"):
    mo = statistics.median(r[k] for r in opt_runs)
    mb = statistics.median(r[k] for r in base_runs)
    delta = (mb - mo) / mo * 100
    print(f"{k:>6} {mo:>13.0f} ns {mb:>12.0f} ns {delta:>+8.1f}%   "
          f"{'arena wins' if delta > 0 else 'baseline wins'}")
