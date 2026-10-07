#!/usr/bin/env bash
#
# make_gps_sim_file.sh - build a GPS Sim TX replay file for the PortaPack Mayhem.
#
# The "GPS Sim TX" app (firmware/application/external/gpssim) is a plain baseband
# replayer: it streams a .C8 file (signed 8-bit interleaved I/Q) straight to the
# transmitter at the sample rate named in the companion .TXT metadata file. It does
# not synthesise anything itself, so the GPS L1 C/A signal has to be generated off
# the device. This script drives osqzss/gps-sdr-sim to do that and writes both the
# .C8 and the .TXT the app expects.
#
# Defaults target Hong Kong International Airport (VHHH / HKG) so a receiver's
# geofence or no-fly-zone logic can be exercised on the bench.
#
# Usage:
#   ./tools/make_gps_sim_file.sh                        # HKIA, 300 s
#   ./tools/make_gps_sim_file.sh -d 600 -o /media/sd/GPS/HKIA.C8
#   ./tools/make_gps_sim_file.sh --lat 51.4706 --lon -0.4619 --alt 25 -o LHR.C8
#
set -euo pipefail

# --- Hong Kong International Airport, aerodrome reference point -------------
# 22 18 32 N, 113 54 52 E (ICAO AIP). Aerodrome elevation is 9 m above mean sea
# level; gps-sdr-sim wants height above the WGS84 ellipsoid, and the geoid sits
# roughly 2 m above the ellipsoid in the Hong Kong area, hence 11 m.
LAT=22.308889
LON=113.914444
ALT=11.0

DURATION=300                 # seconds
SAMPLE_RATE=2600000          # must match what the metadata advertises
CENTER_FREQ=1575420000       # GPS L1
OUT=""
EPHEMERIS=""
START_DATE=""
RETIME=1
SIM_BIN="${GPS_SDR_SIM:-}"
WORKDIR="${TMPDIR:-/tmp}/gps-sdr-sim-build"

usage() { sed -n '2,/^set -euo/p' "$0" | sed 's/^# \{0,1\}//;$d'; exit "${1:-0}"; }

while [ $# -gt 0 ]; do
    case "$1" in
        --lat)        LAT="$2"; shift 2 ;;
        --lon)        LON="$2"; shift 2 ;;
        --alt)        ALT="$2"; shift 2 ;;
        -d|--duration) DURATION="$2"; shift 2 ;;
        -s|--rate)    SAMPLE_RATE="$2"; shift 2 ;;
        -o|--out)     OUT="$2"; shift 2 ;;
        -e|--ephemeris) EPHEMERIS="$2"; shift 2 ;;
        -t|--date)    START_DATE="$2"; shift 2 ;;
        --no-retime)  RETIME=0; shift ;;
        -h|--help)    usage 0 ;;
        *) echo "unknown option: $1" >&2; usage 1 ;;
    esac
done

[ -n "$OUT" ] || OUT="$(dirname "$0")/../sdcard/GPS/HKIA.C8"

# The app's file picker filters on ".C8" and the firmware derives the metadata
# path by swapping the extension for ".TXT" (see metadata_file.cpp), so keep the
# extensions exactly as the firmware spells them.
case "$OUT" in
    *.C8) ;;
    *) echo "error: output must end in .C8 (the app's file picker filters on it)" >&2; exit 1 ;;
esac
META="${OUT%.C8}.TXT"

# --- locate or build gps-sdr-sim -------------------------------------------
if [ -z "$SIM_BIN" ]; then
    if command -v gps-sdr-sim >/dev/null 2>&1; then
        SIM_BIN="$(command -v gps-sdr-sim)"
    elif [ -x "$WORKDIR/gps-sdr-sim/gps-sdr-sim" ]; then
        SIM_BIN="$WORKDIR/gps-sdr-sim/gps-sdr-sim"
    else
        echo ">> building gps-sdr-sim in $WORKDIR"
        mkdir -p "$WORKDIR"
        [ -d "$WORKDIR/gps-sdr-sim/.git" ] || \
            git clone --depth 1 https://github.com/osqzss/gps-sdr-sim.git "$WORKDIR/gps-sdr-sim"
        ( cd "$WORKDIR/gps-sdr-sim" && gcc -O3 -o gps-sdr-sim gpssim.c -lm )
        SIM_BIN="$WORKDIR/gps-sdr-sim/gps-sdr-sim"
    fi
fi
echo ">> gps-sdr-sim: $SIM_BIN"

# --- ephemeris --------------------------------------------------------------
# Any RINEX 2/3 GPS navigation file works. gps-sdr-sim ships a sample
# (brdc0010.22n); combined with -T below it is retimed to the scenario start, so
# the receiver reports a current date even though the orbits are the sample's.
# For orbits that match the real sky, download today's broadcast ephemeris
# (BRDC*_MN.rnx from CDDIS/IGS) and pass it with -e, plus --no-retime.
if [ -z "$EPHEMERIS" ]; then
    for cand in "$WORKDIR/gps-sdr-sim/brdc0010.22n" "$(dirname "$SIM_BIN")/brdc0010.22n"; do
        [ -f "$cand" ] && { EPHEMERIS="$cand"; break; }
    done
fi
[ -n "$EPHEMERIS" ] && [ -f "$EPHEMERIS" ] || {
    echo "error: no RINEX navigation file found; pass one with -e <file>" >&2; exit 1; }
echo ">> ephemeris:   $EPHEMERIS"

[ -n "$START_DATE" ] || START_DATE="$(date -u +%Y/%m/%d,%H:%M:%S)"

# --- size sanity check ------------------------------------------------------
BYTES=$(( DURATION * SAMPLE_RATE * 2 ))
printf '>> output:      %s  (%s s, %s MiB)\n' "$OUT" "$DURATION" "$(( BYTES / 1048576 ))"
if [ "$BYTES" -ge 4294967295 ]; then
    echo "   warning: larger than 4 GiB - needs an exFAT card, FAT32 cannot hold it" >&2
fi

mkdir -p "$(dirname "$OUT")"

# gps-sdr-sim keeps the -e and -o paths in fixed 100-byte buffers (MAX_CHAR in
# gpssim.h) and strcpy()s into them unchecked, so a long absolute path aborts it
# with "buffer overflow detected". Run from the output directory and pass bare
# filenames, staging the ephemeris alongside when its own path is too long.
OUT_DIR="$(cd "$(dirname "$OUT")" && pwd)"
OUT_BASE="$(basename "$OUT")"
EPH_ABS="$(cd "$(dirname "$EPHEMERIS")" && pwd)/$(basename "$EPHEMERIS")"

EPH_ARG="$EPH_ABS"
STAGED=""
if [ "${#EPH_ABS}" -ge 95 ]; then
    STAGED="$OUT_DIR/.gpssim-eph.rnx"
    cp "$EPH_ABS" "$STAGED"
    EPH_ARG=".gpssim-eph.rnx"
fi
cleanup() { [ -n "$STAGED" ] && rm -f "$STAGED"; }
trap cleanup EXIT

ARGS=( -e "$EPH_ARG" -l "$LAT,$LON,$ALT" -b 8 -s "$SAMPLE_RATE"
       -d "$DURATION" -t "$START_DATE" -o "$OUT_BASE" )
[ "$RETIME" -eq 1 ] && ARGS+=( -T "$START_DATE" )

( cd "$OUT_DIR" && "$SIM_BIN" "${ARGS[@]}" )

# --- metadata sidecar -------------------------------------------------------
# read_metadata_file() needs center_frequency and sample_rate or it rejects the
# file and the app silently falls back to its built-in defaults. CRLF matches
# what the firmware's own write_metadata_file() emits.
{
    printf 'center_frequency=%s\r\n' "$CENTER_FREQ"
    printf 'sample_rate=%s\r\n'      "$SAMPLE_RATE"
    printf 'latitude=%s\r\n'         "$LAT"
    printf 'longitude=%s\r\n'        "$LON"
} > "$META"

echo ">> wrote $OUT"
echo ">> wrote $META"
echo
echo "Copy both files into /GPS/ on the PortaPack SD card."
