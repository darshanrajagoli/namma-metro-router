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
#   3. the risk probe: which dominance orders survive a timetable — no feed
#   4. fetch and normalise every feed in scripts/feeds.json, pinning checksums
#   5. the multi-feed study, one row per feed
#   6. the analysis: rankings, correlations, the forest test, the head-to-head
#   7. the focused RAPTOR comparison on one feed, with its correctness gate
#   8. the crowd-model A/B against measured ridership
#   9. the accessibility surface, as CSV and SVG
#  10. where one new interchange would pay most
#
# Outputs land in results/. Nothing is written outside results/ and feeds/.
#
# WHY STEP 2 COMES BEFORE STEP 5
# The study's headline numbers are produced by comparing two implementations
# against each other. If the test suite is red, the comparison is between two
# unknown quantities and the study is worthless. So the tests gate the study
# rather than running after it as a formality.
#
# WHY STEP 3 COMES BEFORE STEP 4
# It is the one artifact that reads no feed, and step 4 is a several-gigabyte
# download of 38 third-party feeds that agencies move without notice. Run last,
# it would be out of reach of anyone offline or behind a failed fetch. Run here,
# a reader with no network still reproduces one complete result, and it costs
# seventeen seconds to say so.

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
    # Print the header block by finding where it ends rather than by line
    # number: the previous fixed '2,30p' silently began truncating --help the
    # moment the step list grew past it, which is the kind of breakage nothing
    # reports.
    -h|--help)    sed -n '2,/^$/p' "$0"; exit 0 ;;
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

# Put an interchange run's ANSWER in the log, not just the name of the file it
# went to. Two shapes to handle, and both are results: a ranked table when the
# network has walkable candidates, and a flat refusal when it has none — which is
# what Namma Metro and BART return, and what docs/interchange.md reports as a
# property of those networks rather than as a run that found nothing.
show_interchange() {
  if grep -q '^=== Top' "$1" 2>/dev/null; then
    sed -n '/^=== Top/,$p' "$1" | head -22 | tee -a "$LOG"
  else
    tail -8 "$1" | tee -a "$LOG"
  fi
}

note "namma-metro-router — full reproduction"
note "repository : $REPO"
note "started    : $(date -u +%Y-%m-%dT%H:%M:%SZ)"
note "host       : $(uname -srm)"
note "compiler   : $(c++ --version | head -1)"
note "python     : $(python3 --version)"
[ "$QUICK" = 1 ] && note "mode       : QUICK (subset of feeds, fewer queries)"

# ── 1. Build ──────────────────────────────────────────────────────────────────
step "1/10  Build"
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
step "2/10  Test suite (AddressSanitizer + UBSan)"
if (cd build_debug && ctest --output-on-failure >>"$LOG" 2>&1); then
  note "$(grep -E '[0-9]+% tests passed' "$LOG" | tail -1)"
else
  die "tests are red; nothing downstream would be trustworthy (see $LOG)"
fi

# ── 3. Risk probe ─────────────────────────────────────────────────────────────
step "3/10  Risk objective: which dominance orders survive a timetable"
# Deliberately NOT reduced under --quick. It takes seventeen seconds at the
# published settings, and running it at any other trial count would print numbers
# that do not match the ones docs/risk.md quotes — which is most of the reason
# for it being in here at all.
if ./build/routing_engine_risk_probe --out-prefix results/risk-probe \
      > results/risk-probe.txt 2>&1; then
  sed -n '/^2\. HOW OFTEN/,/^3\. WHAT/p' results/risk-probe.txt | head -22 | tee -a "$LOG"
  note "wrote results/risk-probe.txt and results/risk-probe-orders.csv"
else
  note "risk probe failed (see results/risk-probe.txt)"
fi

# ── 4. Feeds ──────────────────────────────────────────────────────────────────
step "4/10  Transit feeds"
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

# ── 5. The study ──────────────────────────────────────────────────────────────
step "5/10  Multi-feed study"
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

# ── 6. Analysis ───────────────────────────────────────────────────────────────
step "6/10  Analysis"
python3 scripts/analyze_study.py results/study-results.csv --markdown \
        > results/study-results.md 2>>"$LOG" || die "analysis failed (see $LOG)"
python3 scripts/analyze_study.py results/study-results.csv | tee -a "$LOG"
note "wrote results/study-results.md"

# ── 7. RAPTOR head to head ────────────────────────────────────────────────────
step "7/10  RAPTOR vs the Pareto engine, one feed, with the correctness gate"
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

# ── 8. Crowd model ────────────────────────────────────────────────────────────
step "8/10  Crowd model: synthetic Gaussian vs measured ridership"
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

# ── 9. Accessibility ──────────────────────────────────────────────────────────
step "9/10  Accessibility surfaces"
for slug in namma-metro bart; do
  [ -d "feeds/norm/$slug" ] || continue
  ./build/routing_engine_isochrone "feeds/norm/$slug" \
      --out-prefix "results/accessibility-$slug" --labels \
      > "results/accessibility-$slug.txt" 2>&1 \
    && note "wrote results/accessibility-$slug-surface.{csv,svg}" \
    || note "accessibility failed for $slug (see results/accessibility-$slug.txt)"
done

# ── 10. Interchange search ────────────────────────────────────────────────────
step "10/10  Where one new interchange would pay most"
# On Namma Metro and BART the 800 m planning radius returns no candidates at all,
# and that null IS the published result — docs/interchange.md reports it as a
# property of those networks rather than as a run that found nothing.
for slug in namma-metro bart; do
  [ -d "feeds/norm/$slug" ] || continue
  note ""
  note "-- $slug, 800 m --"
  if ./build/routing_engine_interchange_search "feeds/norm/$slug" \
        --out-prefix "results/interchange-$slug" --labels \
        > "results/interchange-$slug.txt" 2>&1; then
    show_interchange "results/interchange-$slug.txt"
  else
    note "interchange search failed for $slug (see results/interchange-$slug.txt)"
  fi
done
# The Bengaluru finding is the stretched radius, not the null above: 91 candidates
# at 2500 m, and not one of them improves reachability at 45 minutes.
if [ -d feeds/norm/namma-metro ]; then
  note ""
  note "-- namma-metro, stretched to 2500 m (past any honest transfer walk) --"
  if ./build/routing_engine_interchange_search feeds/norm/namma-metro \
        --max-walk 2500 --top 15 --out-prefix results/interchange-namma-metro-2500 \
        > results/interchange-namma-metro-2500.txt 2>&1; then
    show_interchange "results/interchange-namma-metro-2500.txt"
  else
    note "stretched-radius search failed (see results/interchange-namma-metro-2500.txt)"
  fi
fi
# New York is the only one of the three with walkable candidates, and where the
# ranked table in docs/interchange.md comes from. 712 candidates, one full RAPTOR
# preprocess each: about five minutes single-threaded, so --quick leaves it out.
if [ "$QUICK" = 0 ] && [ -d feeds/norm/nyc-subway ]; then
  note ""
  note "-- nyc-subway, 800 m: 712 candidates, roughly five minutes single-threaded --"
  if ./build/routing_engine_interchange_search feeds/norm/nyc-subway \
        --top 15 --out-prefix results/interchange-nyc-subway --labels \
        > results/interchange-nyc-subway.txt 2>&1; then
    show_interchange "results/interchange-nyc-subway.txt"
  else
    note "interchange search failed for nyc-subway (see results/interchange-nyc-subway.txt)"
  fi
fi

step "Done"
note "finished   : $(date -u +%Y-%m-%dT%H:%M:%SZ)"
note ""
note "results/"
ls -1 results/ | sed 's/^/  /' | tee -a "$LOG"
note ""
note "The headline answer is in results/study-results.md."
note "Feed checksums are in feeds/feeds.lock.json — quote them alongside any number."
