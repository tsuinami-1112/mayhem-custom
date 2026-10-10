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
#   ./tools/make_gps_sim_file.sh --fetch                # real orbits, auto-downloaded
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
FETCH=0
USER_DATE=0
SIM_BIN="${GPS_SDR_SIM:-}"
WORKDIR="${TMPDIR:-/tmp}/gps-sdr-sim-build"
CACHE_DIR="${GPS_EPH_CACHE:-${XDG_CACHE_HOME:-$HOME/.cache}/portapack-gps-sim}"
# NOAA CORS carries the IGS-merged daily broadcast ephemeris as brdc<ddd>0.<yy>n.gz,
# openly and without a login (CDDIS needs an Earthdata account). The merged file is
# published roughly half a day after the day it covers.
EPH_MIRROR="${GPS_EPH_MIRROR:-https://geodesy.noaa.gov/corsdata/rinex}"
MAX_LOOKBACK=7

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
        -t|--date)    START_DATE="$2"; USER_DATE=1; shift 2 ;;
        --no-retime)  RETIME=0; shift ;;
        --fetch)      FETCH=1; RETIME=0; shift ;;
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

[ -n "$START_DATE" ] || START_DATE="$(date -u +%Y/%m/%d,%H:%M:%S)"

# --- ephemeris --------------------------------------------------------------
# Any RINEX 2/3 GPS navigation file works, and the choice only affects whether the
# simulated sky matches the real one. The signal is self-consistent either way:
# gps-sdr-sim encodes the same ephemeris into the nav message that it used to
# compute the pseudoranges, so the receiver always solves to the requested
# position regardless of how stale the orbits are.
#
#   default      gps-sdr-sim's bundled sample, retimed to the scenario start with
#                -T. Correct date, orbits that do not match the real sky. Works
#                forever, needs no network.
#   --fetch      the real merged broadcast ephemeris for the scenario day, cached
#                under $CACHE_DIR. Real sky, but only usable for start times
#                inside that day (see below).
#   -e <file>    your own RINEX nav file.
#
# A daily file holds each satellite's ephemeris at roughly 2-hour intervals and
# gps-sdr-sim walks forward through them mid-run (up to EPHEM_ARRAY_SIZE = 15
# sets), but it refuses to start unless the scenario time falls within an hour of
# one of them - about a 28-hour window per file. Outside it you get
# "ERROR: Invalid start time" with the usable tmin/tmax printed.
if [ "$FETCH" -eq 1 ]; then
    command -v gunzip >/dev/null 2>&1 || { echo "error: --fetch needs gunzip" >&2; exit 1; }
    date -u -d "2020-01-01" +%j >/dev/null 2>&1 || {
        echo "error: --fetch needs GNU date (coreutils)" >&2; exit 1; }

    mkdir -p "$CACHE_DIR"
    [ "$USER_DATE" -eq 0 ] || START_DATE="$(date -u -d \
        "$(printf '%s' "$START_DATE" | tr ',' ' ' | tr '/' '-')" +%Y/%m/%d,%H:%M:%S)"
    want_day="${START_DATE%%,*}"
    eff_day=""
    src=""

    for back in $(seq 0 "$MAX_LOOKBACK"); do
        day="$(date -u -d "${want_day//\//-} -${back} day" +%Y-%m-%d)"
        name="brdc$(date -u -d "$day" +%j)0.$(date -u -d "$day" +%y)n"

        if [ -s "$CACHE_DIR/$name" ]; then
            EPHEMERIS="$CACHE_DIR/$name"; eff_day="$day"; src="cached"; break
        fi
        if curl -fsS --retry 2 -o "$CACHE_DIR/$name.gz" \
                "$EPH_MIRROR/$(date -u -d "$day" +%Y)/$(date -u -d "$day" +%j)/$name.gz" 2>/dev/null; then
            gunzip -f "$CACHE_DIR/$name.gz"
            EPHEMERIS="$CACHE_DIR/$name"; eff_day="$day"; src="downloaded"; break
        fi
        rm -f "$CACHE_DIR/$name.gz"
    done

    [ -n "$eff_day" ] || {
        echo "error: no broadcast ephemeris found within $MAX_LOOKBACK days of $want_day" >&2
        echo "       mirror: $EPH_MIRROR" >&2; exit 1; }
    echo ">> ephemeris:   $EPHEMERIS ($src, real orbits)"

    # The daily file for the current day is published incrementally, so its newest
    # epoch usually trails real time by a couple of hours - the download can
    # succeed and the requested start still fall past the end of it. Read the
    # epochs out of the file rather than assuming the whole day is covered.
    # In a RINEX 2 nav epoch line, fields 2-6 are yy mm dd hh mm; field 7 can run
    # into a negative clock term, so seconds are taken as 00 (epochs are on the
    # minute). Timestamps are fixed-width, so sort and string compare are safe.
    TOCS="$(awk '/END OF HEADER/{h=1;next}
                 h && substr($0,1,2) ~ /[0-9]/ {
                     printf "20%02d/%02d/%02d,%02d:%02d:00\n", $2,$3,$4,$5,$6 }' \
            "$EPHEMERIS" | sort -u)"
    FIRST_TOC="$(printf '%s\n' "$TOCS" | head -1)"
    LAST_TOC="$(printf '%s\n' "$TOCS" | tail -1)"
    [ -n "$LAST_TOC" ] || {
        echo "error: no ephemeris epochs parsed from $EPHEMERIS" >&2; exit 1; }
    echo ">> epochs:      $FIRST_TOC .. $LAST_TOC UTC"

    if [ "$USER_DATE" -eq 0 ]; then
        # No specific time asked for, so use the freshest orbits in the file.
        START_DATE="$LAST_TOC"
    elif [ "$START_DATE" \> "$LAST_TOC" ]; then
        echo ">> note:        $START_DATE is past this file's last epoch; using $LAST_TOC."
        START_DATE="$LAST_TOC"
    elif [ "$START_DATE" \< "$FIRST_TOC" ]; then
        echo ">> note:        $START_DATE is before this file's first epoch; using $FIRST_TOC."
        START_DATE="$FIRST_TOC"
    fi
fi

if [ -z "$EPHEMERIS" ]; then
    for cand in "$WORKDIR/gps-sdr-sim/brdc0010.22n" "$(dirname "$SIM_BIN")/brdc0010.22n"; do
        [ -f "$cand" ] && { EPHEMERIS="$cand"; break; }
    done
    [ -n "$EPHEMERIS" ] && echo ">> ephemeris:   $EPHEMERIS (bundled sample, retimed)"
fi
[ -n "$EPHEMERIS" ] && [ -f "$EPHEMERIS" ] || {
    echo "error: no RINEX navigation file found; pass one with -e <file>" >&2; exit 1; }
echo ">> start time:  $START_DATE UTC"

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
# start_time is the UTC a receiver will display for the first sample. gps-sdr-sim
# treats -t as GPS time and encodes an 18 s leap-second count in the nav message,
# so the receiver shows 18 s less than the scenario time. Done as epoch arithmetic:
# "<date> -18 seconds" is ambiguous to GNU date, which reads -18 as a UTC-18 zone
# and a bare "seconds" as +1 s. Falls back to the uncorrected value without GNU date.
START_UTC="$START_DATE"
if EPOCH="$(date -u -d "$(printf '%s' "$START_DATE" | tr ',' ' ' | tr '/' '-')" +%s 2>/dev/null)"; then
    START_UTC="$(date -u -d "@$((EPOCH - 18))" +%Y/%m/%d,%H:%M:%S)"
fi
{
    printf 'center_frequency=%s\r\n' "$CENTER_FREQ"
    printf 'sample_rate=%s\r\n'      "$SAMPLE_RATE"
    printf 'format=c8\r\n'
    printf 'gnss=GPS\r\n'
    printf 'start_time=%s\r\n'       "$START_UTC"
    printf 'latitude=%s\r\n'         "$LAT"
    printf 'longitude=%s\r\n'        "$LON"
    printf 'altitude=%s\r\n'         "$ALT"
} > "$META"

echo ">> wrote $OUT"
echo ">> wrote $META"
echo
echo "Copy both files into /GPS/ on the PortaPack SD card."
