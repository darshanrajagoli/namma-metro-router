#!/usr/bin/env bash
# Scaling sweep: hold the topology fixed at 82 stations and vary service
# frequency, which varies |E| alone. Isolates how query latency scales with
# timetabled-edge count, independent of station count.
#
#   bash tools/scale.sh [network_csv] [build_dir]
#
# Needs the Namma network CSV (see README "GTFS Data") and a Release build.
# Run on AC power -- on battery every figure roughly doubles.
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO" || exit 1

CSV="${1:-raw_namma/bengaluru_metro_network.csv}"
BUILD="${2:-build}"
BIN="$BUILD/routing_engine_benchmark"

[ -f "$CSV" ] || { echo "missing network CSV: $CSV" >&2; exit 1; }
[ -x "$BIN" ] || { echo "missing $BIN -- build Release first" >&2; exit 1; }

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

printf '%10s %8s %10s %12s %12s\n' headway_s nodes edges p50_ns ns_per_edge
for h in 900 600 300 120 60; do
  d="$TMP/feed_$h"
  mkdir -p "$d"
  python3 scripts/build_namma_metro_gtfs.py "$CSV" "$d" --headway "$h" >/dev/null 2>&1
  out=$(taskset -c 3 "$BIN" "$d" 2>/dev/null)
  n=$(printf '%s' "$out" | grep -oP 'Nodes: \K[0-9]+')
  e=$(printf '%s' "$out" | grep -oP 'Edges: \K[0-9]+')
  p=$(printf '%s' "$out" | awk '/p50 +:/{print $4}')
  awk -v h="$h" -v n="$n" -v e="$e" -v p="$p" \
    'BEGIN{printf "%10s %8s %10s %12.0f %12.3f\n", h, n, e, p, p/e}'
done
