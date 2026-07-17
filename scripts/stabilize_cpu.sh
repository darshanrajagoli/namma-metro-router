#!/usr/bin/env bash
# =============================================================================
# stabilize_cpu.sh — Namma Metro Routing Engine Benchmark Environment Setup
#
# PURPOSE:
#   Configure the Dell G15 CPU for cycle-accurate RDTSC benchmarking.
#   Run this script BEFORE every benchmark session.
#
# REQUIRED HARDWARE: Dell G15 laptop (Intel 12th/13th gen, WSL2/Ubuntu 24.04)
# REQUIRED PACKAGES: linux-tools-common, linux-tools-generic, msr-tools
#
# INSTALL DEPENDENCIES (run once):
#   sudo apt-get update && sudo apt-get install -y \
#       linux-tools-common linux-tools-generic msr-tools
#
# USAGE:
#   sudo bash scripts/stabilize_cpu.sh
#   taskset -c 3 ./build/routing_engine_benchmark    # core 3: matches main_bench.cpp
#
# MATHEMATICAL JUSTIFICATION:
#   RDTSC increments at the invariant TSC frequency (fixed nominal clock).
#   If Turbo Boost is active, the CPU core frequency decouples from TSC,
#   rendering raw tick counts meaningless for wall-clock latency estimation.
#   With Turbo disabled and governor=performance, TSC ticks directly equal
#   nominal_GHz × nanoseconds, giving sub-nanosecond timing resolution.
# =============================================================================

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log()  { echo -e "${GREEN}[BENCH]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN] ${NC} $1"; }
die()  { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

# ─── Root check ────────────────────────────────────────────────────────────
[[ $EUID -eq 0 ]] || die "This script must be run as root: sudo bash $0"

# ─── 1. Set performance governor on core 3 ─────────────────────────────────
log "Setting CPU frequency governor to 'performance' on core 3..."

if ! command -v cpupower &>/dev/null; then
    die "cpupower not found. Run: sudo apt-get install linux-tools-generic"
fi

cpupower -c 3 frequency-set -g performance
log "  ✓ Governor = performance (core 3)"

# Optionally set all cores (prevents thermal migration to hotter cores)
NCORES=$(nproc)
for i in $(seq 0 $((NCORES - 1))); do
    cpupower -c "$i" frequency-set -g performance 2>/dev/null || true
done
log "  ✓ Governor = performance (all $NCORES cores)"

# ─── 2. Disable Intel Turbo Boost ──────────────────────────────────────────
#
# WHY THIS IS NON-NEGOTIABLE:
#   Turbo Boost raises core frequency above the base clock for short bursts.
#   The TSC increments at the BASE clock rate, not the boosted rate.
#   During a 10μs routing query, the CPU may be at 4.7 GHz while TSC
#   assumes 2.3 GHz, making measured latency appear 2× too fast.
#   This makes p99 benchmarks completely unreliable for professional use.
#
log "Disabling Intel Turbo Boost..."

TURBO_PATH="/sys/devices/system/cpu/intel_pstate/no_turbo"
if [[ -f "$TURBO_PATH" ]]; then
    echo 1 > "$TURBO_PATH"
    log "  ✓ Turbo Boost disabled (intel_pstate)"
else
    warn "intel_pstate not found. Trying acpi-cpufreq path..."
    BOOST_PATH="/sys/devices/system/cpu/cpufreq/boost"
    if [[ -f "$BOOST_PATH" ]]; then
        echo 0 > "$BOOST_PATH"
        log "  ✓ Turbo Boost disabled (acpi-cpufreq)"
    else
        warn "Could not disable Turbo Boost. Benchmarks may be inaccurate."
    fi
fi

# ─── 3. Disable BD PROCHOT signal (prevents thermal throttle to 800 MHz) ────
#
# BD PROCHOT (Bi-Directional Processor Hot) is a signal from external power
# management ICs to the CPU. On Dell G15, third-party adapters or sensor
# anomalies frequently trigger spurious BD PROCHOT, forcibly downclocking
# the CPU to 800 MHz during benchmarks.
#
# MSR 0x1FC = MSR_POWER_CTL
# Bit 0 of MSR_POWER_CTL controls BD PROCHOT. Clearing it prevents external
# sources from triggering the signal.
#
# WARNING: This modifies CPU hardware state. Reboot to restore defaults.
# Do NOT run on a system under sustained thermal load without monitoring temps.
#
log "Disabling BD PROCHOT via MSR 0x1FC..."

if ! command -v wrmsr &>/dev/null; then
    warn "wrmsr not found. Run: sudo apt-get install msr-tools"
    warn "Skipping BD PROCHOT disable. Benchmarks may throttle."
else
    # Load MSR kernel module
    modprobe msr 2>/dev/null || warn "msr module already loaded or unavailable"

    # Read current value of MSR_POWER_CTL (0x1FC)
    CURRENT_VAL=$(rdmsr -p 3 0x1FC 2>/dev/null || echo "0")
    log "  Current MSR_POWER_CTL (0x1FC) = 0x${CURRENT_VAL}"

    # Clear bit 0 (BD PROCHOT enable) — write back with bit 0 cleared
    # We use a safe fixed value that preserves other bits at their reset state.
    # For Intel 12th gen (Alder Lake): reset value of 0x1FC = 0x4005F
    # With BD PROCHOT disabled: 0x4005E
    wrmsr -p 3 0x1FC 0x4005E
    log "  ✓ BD PROCHOT disabled on core 3 (MSR 0x1FC = 0x4005E)"
fi

# ─── 4. Verify current CPU frequency ───────────────────────────────────────
log "Verifying CPU frequency configuration..."

CURRENT_FREQ=$(cat /sys/devices/system/cpu/cpu3/cpufreq/scaling_cur_freq 2>/dev/null || echo "unknown")
CURRENT_GOV=$(cat /sys/devices/system/cpu/cpu3/cpufreq/scaling_governor 2>/dev/null || echo "unknown")

echo ""
echo "══════════════════════════════════════════════"
echo "  Benchmark Environment Status"
echo "══════════════════════════════════════════════"
echo "  Core 3 Governor : $CURRENT_GOV"
echo "  Core 3 Frequency: $CURRENT_FREQ kHz"
echo "  Turbo Boost     : $(cat $TURBO_PATH 2>/dev/null && echo 'DISABLED' || echo 'status unknown')"
echo "══════════════════════════════════════════════"
echo ""
log "CPU environment stabilized. Run benchmarks with:"
echo ""
echo "  taskset -c 3 ./build/routing_engine_benchmark"
echo ""
warn "To restore defaults (re-enable Turbo, reset governor): reboot or run:"
echo "  echo 0 | sudo tee /sys/devices/system/cpu/intel_pstate/no_turbo"
echo "  sudo cpupower -c 3 frequency-set -g powersave"
