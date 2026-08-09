#!/usr/bin/env bash
#
# reproduce.sh — every result this project publishes, from one command.
#
#   bash scripts/reproduce.sh                 # everything
#   bash scripts/reproduce.sh --quick         # 6 feeds, fewer queries (~5 min)
#   bash scripts/reproduce.sh --skip-fetch    # reuse an existing feeds/ directory
#   bash scripts/reproduce.sh --pin-core 3    # pin, for comparable latency numbers
#
# Steps, in order, each one refusing to continue on a failure that would make
# the next step meaningless:
#
#   1. build (Release and Debug)
#   2. the test suite under AddressSanitizer and UBSan
#   3. fetch and normalise every feed in scripts/feeds.json, pinning checksums
#   4. the multi-feed study, one row per feed
#   5. the analysis: rankings, correlations, the forest test, the head-to-head
#   6. the focused RAPTOR comparison on one feed, with its correctness gate
#   7. the crowd-model A/B against measured ridership
#   8. the accessibility surface, as CSV and SVG
#
# Outputs land in results/. Nothing is written outside results/ and feeds/.
#
# WHY STEP 2 COMES BEFORE STEP 4
# The study's headline numbers are produced by comparing two implementations
# against each other. If the test suite is red, the comparison is between two
# unknown quantities and the study is worthless. So the tests gate the study
# rather than running after it as a formality.

set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

QUICK=0
SKIP_FETCH=0
PIN_CORE=""
QUICK_FEEDS="bart,namma-metro,cta-chicago,nyc-subway,hsl-helsinki,marta-atlanta"

while [ $# -gt 0 ]; do
  case "$1" in
    --quick)      QUICK=1 ;;
    --skip-fetch) SKIP_FETCH=1 ;;
    --pin-core)   PIN_CORE="${2:-}"; shift ;;
    -h|--help)    sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done

mkdir -p results feeds
# Absolute, because one step below runs from inside the build directory and a
# relative "../results/..." would land somewhere else entirely if that directory
# is a symlink — which silently split the log in two the first time this ran.
LOG="$REPO/results/reproduce.log"
: > "$LOG"

step()  { printf '\n\033[1m== %s ==\033[0m\n' "$*" | tee -a "$LOG"; }
note()  { printf '%s\n' "$*" | tee -a "$LOG"; }
die()   { printf '\n\033[31mFAILED: %s\033[0m\n' "$*" | tee -a "$LOG"; exit 1; }

note "namma-metro-router — full reproduction"
note "repository : $REPO"
note "started    : $(date -u +%Y-%m-%dT%H:%M:%SZ)"
note "host       : $(uname -srm)"
note "compiler   : $(c++ --version | head -1)"
note "python     : $(python3 --version)"
[ "$QUICK" = 1 ] && note "mode       : QUICK (subset of feeds, fewer queries)"

# ── 1. Build ──────────────────────────────────────────────────────────────────
step "1/8  Build"
if [ ! -x build/routing_engine_study ]; then
  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -G Ninja >>"$LOG" 2>&1 \
    && cmake --build build -j"$(nproc)" >>"$LOG" 2>&1 \
    || die "release build (see $LOG)"
fi
if [ ! -x build_debug/namma_metro_tests ]; then
  cmake -S . -B build_debug -DCMAKE_BUILD_TYPE=Debug -G Ninja >>"$LOG" 2>&1 \
    && cmake --build build_debug -j"$(nproc)" >>"$LOG" 2>&1 \
    || die "debug build (see $LOG)"
fi
note "ok"

# ── 2. Tests ──────────────────────────────────────────────────────────────────
step "2/8  Test suite (AddressSanitizer + UBSan)"
if (cd build_debug && ctest --output-on-failure >>"$LOG" 2>&1); then
  note "$(grep -E '[0-9]+% tests passed' "$LOG" | tail -1)"
else
  die "tests are red; nothing downstream would be trustworthy (see $LOG)"
fi

# ── 3. Feeds ──────────────────────────────────────────────────────────────────
step "3/8  Transit feeds"
if [ "$SKIP_FETCH" = 1 ]; then
  note "skipped (--skip-fetch); verifying what is already here"
  python3 scripts/fetch_feeds.py --workdir feeds --verify 2>&1 | tee -a "$LOG"
else
  ONLY=""
  [ "$QUICK" = 1 ] && ONLY="--only $QUICK_FEEDS"
  # A feed that 404s or moves is expected and must not end the run: the manifest
  # is a snapshot of what was reachable on one day, and agencies change URLs.
  python3 scripts/fetch_feeds.py --workdir feeds $ONLY 2>&1 | tee -a "$LOG"
  note "(feeds that failed are listed above and are excluded from the study, "
  note " visibly, rather than silently omitted)"
fi
[ -d feeds/norm ] || die "no normalised feeds; cannot continue"
note "normalised feeds: $(ls feeds/norm | wc -l)"

# ── 4. The study ──────────────────────────────────────────────────────────────
step "4/8  Multi-feed study"
PIN_ARG=""
[ -n "$PIN_CORE" ] && PIN_ARG="--pin-core $PIN_CORE"
QUERIES=200
STUDY_ONLY=""
if [ "$QUICK" = 1 ]; then
  QUERIES=60
  # --quick must narrow the STUDY too, not only the fetch. With --skip-fetch it
  # would otherwise study every feed already on disk, which is the opposite of
  # quick and makes the flag's documented runtime a lie.
  STUDY_ONLY="--only $QUICK_FEEDS"
fi
python3 scripts/run_study.py --workdir feeds --build build \
        --queries "$QUERIES" $PIN_ARG $STUDY_ONLY --out results/study-results.csv 2>&1 | tee -a "$LOG"
[ -s results/study-results.csv ] || die "study produced no results"

# ── 5. Analysis ───────────────────────────────────────────────────────────────
step "5/8  Analysis"
python3 scripts/analyze_study.py results/study-results.csv --markdown \
        > results/study-results.md 2>>"$LOG" || die "analysis failed (see $LOG)"
python3 scripts/analyze_study.py results/study-results.csv | tee -a "$LOG"
note "wrote results/study-results.md"

# ── 6. RAPTOR head to head ────────────────────────────────────────────────────
step "6/8  RAPTOR vs the Pareto engine, one feed, with the correctness gate"
BENCH_FEED=feeds/norm/bart
if [ -d "$BENCH_FEED" ]; then
  RUNNER=""
  [ -n "$PIN_CORE" ] && RUNNER="taskset -c $PIN_CORE"
  $RUNNER ./build/routing_engine_raptor_bench "$BENCH_FEED" \
      > results/raptor-vs-engine-bart.txt 2>&1
  tail -20 results/raptor-vs-engine-bart.txt | tee -a "$LOG"
  note "wrote results/raptor-vs-engine-bart.txt"
else
  note "skipped: $BENCH_FEED not present"
fi

# ── 7. Crowd model ────────────────────────────────────────────────────────────
step "7/8  Crowd model: synthetic Gaussian vs measured ridership"
if python3 scripts/fetch_ridership.py --out data-ridership >>"$LOG" 2>&1; then
  RIDERSHIP="data-ridership/station-hourly.csv"
  if [ -d feeds/norm/namma-metro ] && [ -f "$RIDERSHIP" ]; then
    ./build/routing_engine_crowd_study feeds/norm/namma-metro \
        --ridership "$RIDERSHIP" \
        --aliases scripts/station-aliases-bmrcl.csv \
        > results/crowd-model-namma.txt 2>&1
    sed -n '/Pareto frontier/,$p' results/crowd-model-namma.txt | head -20 | tee -a "$LOG"
    note "wrote results/crowd-model-namma.txt"
  else
    note "skipped: need feeds/norm/namma-metro and the ridership file"
  fi
else
  note "skipped: ridership download failed (see $LOG)"
fi

# ── 8. Accessibility ──────────────────────────────────────────────────────────
step "8/8  Accessibility surfaces"
for slug in namma-metro bart; do
  [ -d "feeds/norm/$slug" ] || continue
  ./build/routing_engine_isochrone "feeds/norm/$slug" \
      --out-prefix "results/accessibility-$slug" --labels \
      > "results/accessibility-$slug.txt" 2>&1 \
    && note "wrote results/accessibility-$slug-surface.{csv,svg}" \
    || note "accessibility failed for $slug (see results/accessibility-$slug.txt)"
done

step "Done"
note "finished   : $(date -u +%Y-%m-%dT%H:%M:%SZ)"
note ""
note "results/"
ls -1 results/ | sed 's/^/  /' | tee -a "$LOG"
note ""
note "The headline answer is in results/study-results.md."
note "Feed checksums are in feeds/feeds.lock.json — quote them alongside any number."
