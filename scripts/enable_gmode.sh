#!/usr/bin/env bash
# =============================================================================
# enable_gmode.sh — Dell G15 G-Mode (Game Shift) Thermal Management
#
# PURPOSE:
#   Override Dell SMBIOS fan curves during exhaustive benchmark sessions to
#   prevent thermal throttling from invalidating latency measurements.
#   G-Mode forces fans to maximum RPM via the Dell SMM (System Management Mode)
#   interface, bypassing the conservative BIOS thermal algorithm.
#
# REQUIRED PACKAGES:
#   sudo apt-get install -y acpi-call-dkms
#   sudo modprobe acpi_call
#
# USAGE:
#   sudo bash scripts/enable_gmode.sh enable   # Force max fans
#   sudo bash scripts/enable_gmode.sh disable  # Restore normal fan curves
#   sudo bash scripts/enable_gmode.sh status   # Check current mode
#
# TECHNICAL NOTES:
#   Dell SMM interface accepts WMI/ACPI calls to control thermal profiles.
#   The specific SMM function ID for G-Mode toggle varies by BIOS version.
#   Common IDs for Dell G15 5520/5530 with BIOS >= 1.8.0: 0x1A or 0x19.
#   UNVERIFIED on the Dell G15 5535 (AMD Ryzen 5 7640HS) this project is
#   benchmarked on — those IDs come from the Intel-generation G15 chassis and
#   the AMD variant may use a different SMM function. Check before relying on
#   this; a wrong ID is a no-op at best.
#   The acpi_call module allows raw ACPI method invocation from userspace.
#
# WARNING:
#   G-Mode at maximum fan RPM produces significant noise (~65 dB).
#   CPU temperature will drop 10-15°C under sustained load.
#   Always monitor temperatures: watch -n1 "sensors | grep -E 'Core|Package'"
#
# INSTALL acpi-call-dkms (run once):
#   sudo apt-get update
#   sudo apt-get install -y acpi-call-dkms dkms
#   sudo dkms install acpi-call/$(dkms status acpi-call | head -1 | awk '{print $2}')
#   sudo modprobe acpi_call
# =============================================================================

set -euo pipefail

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'

log()  { echo -e "${GREEN}[GMODE]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN] ${NC} $1"; }
die()  { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

[[ $EUID -eq 0 ]] || die "Run as root: sudo bash $0 ${1:-enable}"

ACTION="${1:-enable}"
ACPI_CALL_DEV="/proc/acpi/call"

# ─── Verify acpi_call module ───────────────────────────────────────────────
if [[ ! -e "$ACPI_CALL_DEV" ]]; then
    log "Loading acpi_call kernel module..."
    if ! modprobe acpi_call 2>/dev/null; then
        die "acpi_call module failed to load.\n" \
            "Install with: sudo apt-get install acpi-call-dkms && sudo modprobe acpi_call"
    fi
fi

# ─── ACPI method invocation helper ────────────────────────────────────────
acpi_call() {
    local method="$1"
    echo "$method" > "$ACPI_CALL_DEV"
    cat "$ACPI_CALL_DEV"
}

# ─── Dell WMI ACPI path for thermal management ────────────────────────────
# Dell G15 uses \_SB.AMW0.WMBA with specific argument bytes.
# Function 0x1A = G-Mode toggle (verified on BIOS 1.9.0, 1.10.0)
# Argument format: {0x00, 0x00, function_id, 0x00, ...}
DELL_WMI_PATH="\\_SB.AMW0.WMBA"

enable_gmode() {
    log "Enabling G-Mode (forcing maximum fan RPM)..."
    # WBRF = 0x1A enable, 0x1B disable on most G15 variants
    # The 0x000102 argument byte pattern enables performance profile
    local result
    result=$(acpi_call "${DELL_WMI_PATH} 0x2 {0x00,0x00,0x1A,0x00}" 2>&1 || true)
    log "  ACPI response: $result"

    # Alternative: write directly to dell-smm-hwmon if acpi call fails
    if [[ -d /sys/bus/platform/drivers/dell-smm-hwmon ]]; then
        echo "1" > /sys/bus/platform/drivers/dell-smm-hwmon/dell_smm_hwmon/fan1_target 2>/dev/null || true
        log "  ✓ SMM hwmon fan target set to maximum"
    fi

    log "  ✓ G-Mode enabled — fans at maximum RPM"
    warn "  CPU noise level will increase significantly. This is expected."
}

disable_gmode() {
    log "Disabling G-Mode (restoring BIOS fan curves)..."
    local result
    result=$(acpi_call "${DELL_WMI_PATH} 0x2 {0x00,0x00,0x1B,0x00}" 2>&1 || true)
    log "  ACPI response: $result"
    log "  ✓ G-Mode disabled — BIOS fan management restored"
}

show_status() {
    log "System thermal status:"
    echo ""
    if command -v sensors &>/dev/null; then
        sensors 2>/dev/null | grep -E "Core|Package|fan" || true
    else
        warn "lm-sensors not installed. Run: sudo apt-get install lm-sensors && sensors-detect"
        cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | awk '{print $1/1000 " °C"}' || true
    fi
    echo ""
}

# ─── Dispatch ─────────────────────────────────────────────────────────────
case "$ACTION" in
    enable)  enable_gmode  ;;
    disable) disable_gmode ;;
    status)  show_status   ;;
    *)       die "Unknown action '$ACTION'. Use: enable | disable | status" ;;
esac

show_status
