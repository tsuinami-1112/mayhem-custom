#!/usr/bin/env python3
"""Build a Multi-GNSS Sim replay file for the PortaPack Mayhem.

Drives one signal generator per constellation, mixes their output at equal
per-satellite power, quantises to one of the sample formats in FORMAT.md and
writes the .C8/.C2/.C1 plus its .TXT sidecar.

    ./make_gnss_sim_file.py --gnss gps,gal -d 300 -o /mnt/d/GPS/HKIA.C8

Defaults target Hong Kong International Airport. Run with --help for options.

Both upstream generators are fetched and built on first use, into
~/.cache/portapack-gps-sim/tools. Neither needs a USRP or libuhd; see
ensure_galileo() for how the Galileo one is decoupled from it.
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

import pack

# Hong Kong International Airport (VHHH), aerodrome reference point.
# 22 18 32 N, 113 54 52 E. Aerodrome elevation is 9 m above mean sea level and
# the generators want height above the WGS84 ellipsoid, which the geoid sits
# roughly 2 m below here.
HKIA = (22.308889, 113.914444, 11.0)

SAMPLE_RATE = 2_600_000          # what the firmware and both generators use
L1 = 1_575_420_000
CHUNK_SAMPLES = 1 << 20          # 1 Mi complex samples per mixing pass

GPS_REPO = "https://github.com/osqzss/gps-sdr-sim.git"
GAL_REPO = "https://github.com/harshadms/galileo-sdr-sim.git"
EPH_MIRROR = os.environ.get("GPS_EPH_MIRROR", "https://geodesy.noaa.gov/corsdata/rinex")
CACHE = Path(os.environ.get("GPS_EPH_CACHE",
                            Path(os.environ.get("XDG_CACHE_HOME",
                                                Path.home() / ".cache")) / "portapack-gps-sim"))

# A satellite line from either generator: PRN, azimuth, elevation, range, ...
SAT_LINE = re.compile(r"^\s*(\d+)\s+([\d.]+)\s+([\d.-]+)\s+([\d.]+)")


def log(msg):
    print(f">> {msg}", flush=True)


def run(cmd, **kw):
    kw.setdefault("check", True)
    return subprocess.run(cmd, **kw)


# --------------------------------------------------------------------------
# Building the generators
# --------------------------------------------------------------------------

def ensure_gps(tools):
    """osqzss/gps-sdr-sim. Emits int16 I/Q with -b 16 at 2.6 MS/s by default."""
    binary = tools / "gps-sdr-sim" / "gps-sdr-sim"
    if binary.exists():
        return binary
    src = tools / "gps-sdr-sim"
    if not (src / ".git").exists():
        log(f"cloning gps-sdr-sim into {src}")
        run(["git", "clone", "--depth", "1", GPS_REPO, str(src)])
    log("building gps-sdr-sim")
    run(["gcc", "-O3", "-o", "gps-sdr-sim", "gpssim.c", "-lm"], cwd=src)
    return binary


# Pinned: upstream moves (its 2026-10-06 commit changed the E1 composite sign, the
# carrier start and the I/NAV words), and every change here was verified against
# exactly this revision.
GAL_COMMIT = "bdccd7f67d98d206f07aecc728fc80d28ad73d47"

# set_scenario_start_time() in src/gnss-time.cpp, as upstream has it. eph1 is
# `vector<ephem_t> eph1[MAX_SAT]`, one vector per satellite, but the loop indexes
# it [set][satellite]: it reads past the end of every vector and never shifts the
# real records, so -T to a date far from the file's own leaves zero satellites
# visible and the generator exits 0 with a file of pure noise.
GAL_RETIME_BROKEN = """            for (int sv = 0; sv < MAX_SAT; sv++)
            {
                for (int i = 0; i < neph; i++)
                {
                    if (eph1[i][sv].vflg == 1)
                    {
                        gtmp = incGalTime(eph1[i][sv].toc, dsec);
                        gal2date(&gtmp, &ttmp);
                        eph1[i][sv].toc = gtmp;
                        eph1[i][sv].t = ttmp;

                        gtmp = incGalTime(eph1[i][sv].toe, dsec);
                        eph1[i][sv].toe = gtmp;
                    }
                }
            }
"""
GAL_RETIME_FIXED = """            // Patched by make_gnss_sim_file.py: eph1 is one vector per satellite.
            for (int sv = 0; sv < MAX_SAT; sv++)
            {
                for (ephem_t &e : eph1[sv])
                {
                    if (e.vflg == 1)
                    {
                        gtmp = incGalTime(e.toc, dsec);
                        gal2date(&gtmp, &ttmp);
                        e.toc = gtmp;
                        e.t = ttmp;

                        gtmp = incGalTime(e.toe, dsec);
                        e.toe = gtmp;
                    }
                }
            }
"""

# incGalTime() in the same file never carries seconds over into the week, so a
# -T shift of years leaves toe.sec at ~1e8 with the old week. subGalTime() still
# gets the difference right, but satpos() works from toe.sec alone (tk = t -
# toe.sec, with one +-604800 wrap), so every satellite is propagated years off
# its orbit and none is above the horizon. gps-sdr-sim's incGPSTime() normalises.
GAL_INC_BROKEN = """galtime_t incGalTime(galtime_t g, double dt)
{
    g.sec = g.sec + dt;
    return g;
}
"""
GAL_INC_FIXED = """galtime_t incGalTime(galtime_t g, double dt)
{
    // Patched by make_gnss_sim_file.py: carry whole weeks, as gps-sdr-sim does.
    g.sec = g.sec + dt;
    double weeks = floor(g.sec / SECONDS_IN_WEEK);
    g.week += (int)weeks;
    g.sec -= weeks * SECONDS_IN_WEEK;
    return g;
}
"""


def patch_galileo_source(src):
    """Apply the source changes galileo-sdr-sim needs here. Returns True if any
    file changed, so a stale binary can be rebuilt."""
    changed = False

    # Its sample rate is a compile-time constant, 2.484 MS/s, which has to become
    # 2.6 MS/s to mix with GPS. The constant only feeds `delt = 1/TX_SAMPLERATE`
    # and the phase accumulators, so retuning it is safe.
    constants = src / "include" / "constants.h"
    text = constants.read_text()
    patched = text.replace("#define TX_SAMPLERATE 2484000.0",
                           f"#define TX_SAMPLERATE {float(SAMPLE_RATE)}")
    patched = patched.replace("const float SAMP_RATE = 2484000.0;",
                              f"const float SAMP_RATE = {float(SAMPLE_RATE)};")
    if patched != text:
        constants.write_text(patched)
        log(f"retuned galileo-sdr-sim to {SAMPLE_RATE} Sa/s")
        changed = True
    if "2484000" in patched:
        sys.exit(f"error: {constants} still carries the 2.484 MS/s rate somewhere new")

    # The two -T retiming fixes described at GAL_RETIME_BROKEN and GAL_INC_BROKEN.
    timing = src / "src" / "gnss-time.cpp"
    text = timing.read_text()
    for broken, fixed, what in ((GAL_RETIME_BROKEN, GAL_RETIME_FIXED, "ephemeris retiming"),
                                (GAL_INC_BROKEN, GAL_INC_FIXED, "week carry")):
        if broken in text:
            text = text.replace(broken, fixed)
            log(f"patched galileo-sdr-sim {what}")
            changed = True
        elif fixed not in text:
            sys.exit(f"error: {timing} does not look like galileo-sdr-sim {GAL_COMMIT[:7]}; "
                     f"the {what} patch does not apply")
    if changed:
        timing.write_text(text)
    return changed


def ensure_galileo(tools):
    """harshadms/galileo-sdr-sim at GAL_COMMIT, with three changes applied on
    first build: the sample rate and the -T fix (patch_galileo_source), and no
    UHD dependency.

    It includes <uhd/usrp/multi_usrp.hpp> from structures.h for two types it
    parks in a struct, even on the file-sink path, which would otherwise make
    libuhd-dev a build dependency for a program that never touches a USRP. A
    small stand-in header supplies those types, usrp.cpp is left out and
    init_usrp() is stubbed. Output of the two builds was compared byte-for-byte
    and is identical.
    """
    binary = tools / "galileo-build" / "galileo-sdr-sim"
    src = tools / "galileo-sdr-sim"

    if not (src / ".git").exists():
        log(f"cloning galileo-sdr-sim {GAL_COMMIT[:7]} into {src}")
        src.mkdir(parents=True, exist_ok=True)
        run(["git", "init", "-q", str(src)])
        run(["git", "-C", str(src), "remote", "add", "origin", GAL_REPO])
    head = subprocess.run(["git", "-C", str(src), "rev-parse", "HEAD"],
                          capture_output=True, text=True).stdout.strip()
    if not re.fullmatch(r"[0-9a-f]{40}", head):
        head = ""  # a repository with no commits yet echoes "HEAD"
    if head != GAL_COMMIT:
        if head:
            log(f"moving galileo-sdr-sim from {head[:7]} to {GAL_COMMIT[:7]}")
            run(["git", "-C", str(src), "checkout", "-q", "--", "."])
        run(["git", "-C", str(src), "fetch", "-q", "--depth", "1", "origin", GAL_COMMIT])
        run(["git", "-C", str(src), "-c", "advice.detachedHead=false",
             "checkout", "-q", "FETCH_HEAD"])
        binary.unlink(missing_ok=True)

    if patch_galileo_source(src) and binary.exists():
        log("galileo-sdr-sim source changed; rebuilding")
        binary.unlink()
    if binary.exists():
        return binary

    build = tools / "galileo-build"
    build.mkdir(parents=True, exist_ok=True)
    shim = build / "uhdshim" / "uhd" / "usrp"
    shim.mkdir(parents=True, exist_ok=True)
    (shim / "multi_usrp.hpp").write_text(UHD_SHIM)
    (build / "usrp_stub.cpp").write_text(
        '#include "galileo-sdr.h"\nvoid init_usrp(usrp_conf_t, sim_t*) {}\n')

    sources = [str(p) for p in sorted((src / "src").glob("*.cpp")) if p.name != "usrp.cpp"]
    log("building galileo-sdr-sim (this takes a few minutes)")
    run(["g++", "-std=c++17", "-O2", "-w",
         "-I", str(build / "uhdshim"), "-I", str(src / "include"),
         *sources, "usrp_stub.cpp", "-o", "galileo-sdr-sim",
         "-lboost_system", "-lboost_thread", "-lboost_program_options",
         "-lpthread", "-lncurses", "-ltinfo", "-lcurl", "-lz", "-lm"],
        cwd=build)
    return binary


UHD_SHIM = '''/* Stand-in for the UHD headers, written by make_gnss_sim_file.py.
 *
 * galileo-sdr-sim's structures.h includes <uhd/usrp/multi_usrp.hpp> for two
 * types it stores in a struct, even on the file-sink path, so building it
 * normally needs libuhd-dev -- a large install for a program that never talks to
 * a USRP. Supplying just those types, leaving usrp.cpp out of the build and
 * stubbing init_usrp() avoids it. Output was verified byte-identical to a build
 * against the real headers.
 *
 * structures.h also says `using namespace std;` and then uses bare map/vector/
 * queue, relying on the real UHD header to have pulled them in.
 */
#pragma once
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <vector>

namespace uhd {

struct tx_metadata_t {
    bool start_of_burst = false;
    bool end_of_burst = false;
    bool has_time_spec = false;
    double time_spec = 0.0;
};

struct tx_streamer {
    using sptr = std::shared_ptr<tx_streamer>;
    /* main.cpp calls this on the USRP path. init_usrp() is stubbed so the
     * pointer stays null and that path is never taken with -U 1, but it still
     * has to compile. */
    template <typename BuffT>
    size_t send(const BuffT&, size_t nsamps, const tx_metadata_t&, double) {
        return nsamps;
    }
};

}  // namespace uhd
'''


# --------------------------------------------------------------------------
# Ephemeris
# --------------------------------------------------------------------------

def rinex_epochs(path):
    """Sorted unique 'YYYY/MM/DD,hh:mm:00' epochs in a RINEX 2 nav file.

    Fields 2-6 of an epoch line are yy mm dd hh mm; field 7 runs into the clock
    term when it is negative, and epochs fall on the minute, so seconds are 00.
    """
    out = set()
    header = False
    for line in path.read_text(errors="replace").splitlines():
        if not header:
            header = "END OF HEADER" in line
            continue
        if line[:2].strip().isdigit():
            f = line.split()
            if len(f) >= 6:
                try:
                    out.add("20%02d/%02d/%02d,%02d:%02d:00"
                            % tuple(int(x) for x in f[1:6]))
                except ValueError:
                    pass
    return sorted(out)


# Where a retimed scenario is anchored in the bundled Galileo sample. -T maps the
# scenario start onto the file's first record. The sample is one station's log
# (sbf2rin), so at its start it only holds records for the few satellites that
# station was tracking then, and a retimed scenario sees 1 satellite from Hong
# Kong; by 16:00 the file covers the constellation and the same scenario sees 7.
# Records before this epoch are dropped so the anchor lands there.
GAL_SAMPLE_ANCHOR = (2022, 2, 20, 16, 0)


def trimmed_gal_sample(tools):
    """The bundled 20feb2022.rnx without the records before GAL_SAMPLE_ANCHOR."""
    src = tools / "galileo-sdr-sim" / "rinex_files" / "20feb2022.rnx"
    out = CACHE / "gal_sample_from_1600.rnx"
    if out.exists() and out.stat().st_mtime >= src.stat().st_mtime:
        return out
    lines = src.read_text().splitlines(keepends=True)
    for i, line in enumerate(lines):
        if "END OF HEADER" in line:
            header, body = lines[:i + 1], lines[i + 1:]
            break
    else:
        sys.exit(f"error: {src} has no END OF HEADER")
    records, kept = [], 0
    for line in body:
        m = re.match(r"^E\d\d (\d{4}) (\d\d) (\d\d) (\d\d) (\d\d)", line)
        if m:
            records.append([line, tuple(int(x) for x in m.groups()) >= GAL_SAMPLE_ANCHOR])
        elif records:
            records[-1][0] += line
    with out.open("w") as f:
        f.writelines(header)
        for text, keep in records:
            if keep:
                f.write(text)
                kept += 1
    if not kept:
        sys.exit(f"error: no records at or after {GAL_SAMPLE_ANCHOR} in {src}")
    log(f"galileo sample: {kept} of {len(records)} records kept, from "
        f"{'/'.join(f'{v:02d}' for v in GAL_SAMPLE_ANCHOR[:3])},{GAL_SAMPLE_ANCHOR[3]:02d}:{GAL_SAMPLE_ANCHOR[4]:02d}")
    return out


def fetch_gps_ephemeris(want_day, lookback=7):
    """Daily merged broadcast ephemeris from NOAA CORS (open, no login).

    Returns (path, epochs). The file for the day in progress is published
    incrementally, so its newest epoch trails real time by a couple of hours.
    """
    import datetime
    CACHE.mkdir(parents=True, exist_ok=True)
    day = datetime.datetime.strptime(want_day, "%Y/%m/%d").date()
    for back in range(lookback + 1):
        d = day - datetime.timedelta(days=back)
        name = f"brdc{d.timetuple().tm_yday:03d}0.{d.year % 100:02d}n"
        local = CACHE / name
        if local.exists() and local.stat().st_size:
            log(f"ephemeris: {local} (cached)")
            return local, rinex_epochs(local)
        url = f"{EPH_MIRROR}/{d.year}/{d.timetuple().tm_yday:03d}/{name}.gz"
        gz = CACHE / f"{name}.gz"
        try:
            run(["curl", "-fsS", "--retry", "2", "-o", str(gz), url])
        except subprocess.CalledProcessError:
            gz.unlink(missing_ok=True)
            continue
        run(["gunzip", "-f", str(gz)])
        log(f"ephemeris: {local} (downloaded)")
        return local, rinex_epochs(local)
    sys.exit(f"error: no GPS ephemeris within {lookback} days of {want_day} at {EPH_MIRROR}")


# --------------------------------------------------------------------------
# Running a generator
# --------------------------------------------------------------------------

def run_generator(name, binary, ephemeris, work, args, retime):
    """Run one generator inside `work` with bare filenames.

    Both generators strcpy their -e and -o arguments into fixed buffers --
    MAX_CHAR is 100 in gps-sdr-sim and 120 in galileo-sdr-sim -- so a long
    absolute path aborts them with "buffer overflow detected". Everything is
    therefore staged into the working directory and referred to by name.
    """
    eph_local = f"{name}.rnx"
    shutil.copy(ephemeris, work / eph_local)
    out_local = f"{name}.bin"
    binary_local = work / f"gen_{name}"
    shutil.copy(binary, binary_local)
    binary_local.chmod(0o755)

    cmd = [f"./gen_{name}", "-e", eph_local, "-o", out_local,
           "-l", f"{args.lat},{args.lon},{args.alt}",
           "-d", str(args.duration), "-t", args.date]
    if retime:
        cmd += ["-T", args.date]
    if name == "gps":
        cmd += ["-b", "16", "-s", str(SAMPLE_RATE)]
    else:
        cmd += ["-U", "1", "-b", "1"]

    log(f"generating {name}")
    proc = subprocess.run(cmd, cwd=work, capture_output=True, text=True)
    produced = work / out_local
    if proc.returncode != 0 or not produced.exists() or not produced.stat().st_size:
        sys.stderr.write(proc.stderr[-2000:] + "\n")
        sys.exit(f"error: {name} generator failed")

    sats = []
    for line in proc.stderr.splitlines():
        m = SAT_LINE.match(line)
        if m:
            sats.append((int(m.group(1)), float(m.group(3))))
    if not sats:
        sys.exit(f"error: {name} reported no satellites; refusing to mix an empty stream")

    log(f"  {name}: {len(sats)} satellites, "
        f"elevation {min(e for _, e in sats):.0f}-{max(e for _, e in sats):.0f} deg")
    return produced, sats


def stream_power(path, sample_bytes=4, probe_samples=2_000_000):
    """Mean |z|^2 of a prefix of an int16 I/Q file."""
    raw = np.fromfile(path, dtype="<i2", count=2 * probe_samples)
    if raw.size < 2:
        sys.exit(f"error: {path} is too short to measure")
    z = raw[0::2].astype(np.float32) + 1j * raw[1::2].astype(np.float32)
    return float(np.mean(np.abs(z) ** 2))


# --------------------------------------------------------------------------
# Mixing and packing
# --------------------------------------------------------------------------

def mix(streams, out_path, fmt, cn0, seed=1):
    """Mix int16 streams to `fmt`, equalising per-satellite power.

    Each generator already models path loss and elevation within its own
    constellation, so only the level *between* constellations needs matching.
    Equalising total power would be wrong when the constellations have different
    satellite counts, so it is done per satellite.

    The post-mix level is known analytically from the measured powers, the scale
    factors and the noise level, so the quantiser's scale and thresholds are
    computed once and applied to every chunk. Deriving them per chunk would let
    them drift with the signal.
    """
    target = 1.0  # arbitrary per-satellite power; only ratios matter
    plan = []
    for s in streams:
        per_sat = s["power"] / len(s["sats"])
        plan.append({**s, "scale": np.sqrt(target / per_sat)})

    signal_power = sum(target * len(s["sats"]) for s in plan)

    noise_sigma = 0.0
    if cn0:
        noise_power = target / (10 ** (cn0 / 10) / SAMPLE_RATE)
        noise_sigma = np.sqrt(noise_power / 2)
        total = signal_power + noise_power
        log(f"noise: C/N0 {cn0:.0f} dB-Hz per satellite "
            f"({10 * np.log10(noise_power / signal_power):+.1f} dB vs total signal)")
    else:
        total = signal_power

    c8_scale = 32.0 / np.sqrt(total)
    c2_sigma = np.sqrt(total / 2.0)

    n = min(s["path"].stat().st_size // 4 for s in plan)
    n -= n % 4  # c1 packs four samples to a byte
    log(f"mixing {n:,} samples ({n / SAMPLE_RATE:.1f} s) to {fmt}")

    rng = np.random.default_rng(seed)
    handles = [open(s["path"], "rb") for s in plan]
    try:
        with open(out_path, "wb") as dst:
            done = 0
            while done < n:
                count = min(CHUNK_SAMPLES, n - done)
                acc = np.zeros(count, dtype=np.complex64)
                for h, s in zip(handles, plan):
                    raw = np.frombuffer(h.read(count * 4), dtype="<i2")
                    if raw.size < 2 * count:
                        sys.exit("error: a generator output ended early")
                    acc += s["scale"] * (raw[0::2].astype(np.float32)
                                         + 1j * raw[1::2].astype(np.float32))
                if noise_sigma:
                    acc += (rng.normal(0, noise_sigma, count)
                            + 1j * rng.normal(0, noise_sigma, count)).astype(np.complex64)

                if fmt == "c8":
                    dst.write(pack.pack_c8(acc, scale=c8_scale))
                elif fmt == "c2":
                    dst.write(pack.pack_c2(acc, sigma=c2_sigma))
                else:
                    dst.write(pack.pack_c1(acc))
                done += count
    finally:
        for h in handles:
            h.close()
    return n


def write_sidecar(path, fmt, gnss, args, cn0):
    """CRLF, as the firmware's own write_metadata_file() emits."""
    # start_time is the UTC the receiver will display. Both generators treat -t as
    # GPS time and the nav message carries an 18 s leap-second count, so the
    # receiver reads 18 s earlier than the scenario time.
    import datetime
    t = datetime.datetime.strptime(args.date, "%Y/%m/%d,%H:%M:%S") - datetime.timedelta(seconds=18)
    lines = [
        f"center_frequency={L1}",
        f"sample_rate={SAMPLE_RATE}",
        f"format={fmt}",
        f"gnss={','.join(gnss)}",
        f"start_time={t.strftime('%Y/%m/%d,%H:%M:%S')}",
        f"latitude={args.lat}",
        f"longitude={args.lon}",
        f"altitude={args.alt}",
        f"cn0={cn0:.0f}",
    ]
    path.write_bytes(("\r\n".join(lines) + "\r\n").encode())


# --------------------------------------------------------------------------

NAMES = {"gps": "GPS", "gal": "GAL"}


def main():
    import datetime

    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--lat", type=float, default=HKIA[0])
    p.add_argument("--lon", type=float, default=HKIA[1])
    p.add_argument("--alt", type=float, default=HKIA[2])
    p.add_argument("-d", "--duration", type=int, default=300, help="seconds (default 300)")
    p.add_argument("--gnss", default="gps", help="comma list of gps,gal (default gps)")
    p.add_argument("--format", default="c8", choices=("c8", "c2", "c1"))
    p.add_argument("-o", "--out", help="output file; extension must match --format")
    p.add_argument("-t", "--date", help="scenario start, YYYY/MM/DD,hh:mm:ss (default now)")
    p.add_argument("--cn0", type=float, default=None,
                   help="add noise at this per-satellite C/N0 in dB-Hz; "
                        "default 0 for c8, 45 for c2/c1 (c1 needs it)")
    p.add_argument("--fetch", action="store_true",
                   help="download real GPS ephemeris instead of retiming the bundled sample")
    p.add_argument("--gps-eph", help="GPS RINEX nav file")
    p.add_argument("--gal-eph", help="Galileo RINEX nav file")
    p.add_argument("--keep-temp", action="store_true")
    args = p.parse_args()

    gnss = [g.strip().lower() for g in args.gnss.split(",") if g.strip()]
    for g in gnss:
        if g not in NAMES:
            p.error(f"unknown constellation {g!r}; choose from {', '.join(NAMES)}")
    if not gnss:
        p.error("--gnss needs at least one constellation")

    if args.cn0 is None:
        args.cn0 = 0.0 if args.format == "c8" else 45.0
    if args.format == "c1" and not args.cn0:
        p.error("c1 needs noise: hard limiting a noiseless sum lets the strongest "
                "satellite capture the limiter. Pass --cn0.")

    suffix = pack.SUFFIXES[args.format]
    want_ext = pack.EXTENSIONS[args.format]
    out = Path(args.out) if args.out else Path(__file__).resolve().parents[2] / "sdcard" / "GPS" / f"HKIA{suffix}{want_ext}"
    if out.suffix.upper() != want_ext:
        p.error(f"--format {args.format} wants a {want_ext} file, got {out.suffix}")

    if not args.date:
        args.date = datetime.datetime.now(datetime.timezone.utc).strftime("%Y/%m/%d,%H:%M:%S")
    # Retiming (-T) shifts an ephemeris file's epochs onto the scenario date so a
    # stale sample still simulates a sky. It is decided per constellation: only an
    # ephemeris known to cover the scenario date (a --fetch download) is used as
    # is. The bundled Galileo sample is always retimed; a --fetch run must not
    # switch that off, or galileo-sdr-sim rejects today's date against 2022 orbits.
    retime = {g: True for g in gnss}

    tools = CACHE / "tools"
    tools.mkdir(parents=True, exist_ok=True)

    builders = {"gps": ensure_gps, "gal": ensure_galileo}
    binaries = {g: builders[g](tools) for g in gnss}

    eph = {}
    if "gps" in gnss:
        if args.gps_eph:
            eph["gps"] = Path(args.gps_eph)
        elif args.fetch:
            path, epochs = fetch_gps_ephemeris(args.date.split(",")[0])
            eph["gps"] = path
            retime["gps"] = False
            if epochs and args.date > epochs[-1]:
                log(f"note: {args.date} is past this ephemeris ({epochs[-1]}); using that")
                args.date = epochs[-1]
        else:
            eph["gps"] = tools / "gps-sdr-sim" / "brdc0010.22n"
    if "gal" in gnss:
        eph["gal"] = Path(args.gal_eph) if args.gal_eph else trimmed_gal_sample(tools)

    for g, path in eph.items():
        if not Path(path).exists():
            sys.exit(f"error: {g} ephemeris not found: {path}")

    log(f"start time:  {args.date} UTC")
    log("orbits:      " + ", ".join(
        f"{NAMES[g]} {'real' if not retime[g] else 'sample, retimed'}" for g in gnss))
    log(f"position:    {args.lat}, {args.lon}, {args.alt} m")
    log(f"output:      {out}")

    work = Path(tempfile.mkdtemp(prefix="gnsssim-"))
    try:
        streams = []
        for g in gnss:
            path, sats = run_generator(g, binaries[g], eph[g], work, args, retime[g])
            streams.append({"name": g, "path": path, "sats": sats,
                            "power": stream_power(path)})

        out.parent.mkdir(parents=True, exist_ok=True)
        samples = mix(streams, out, args.format, args.cn0)
        write_sidecar(out.with_suffix(".TXT"), args.format,
                      [NAMES[g] for g in gnss], args, args.cn0)
    finally:
        if args.keep_temp:
            log(f"temp kept at {work}")
        else:
            shutil.rmtree(work, ignore_errors=True)

    log(f"wrote {out} ({out.stat().st_size:,} bytes, {samples / SAMPLE_RATE:.1f} s)")
    log(f"wrote {out.with_suffix('.TXT')}")
    print()
    print(f"Copy both files into /GPS/ on the PortaPack SD card.")


if __name__ == "__main__":
    main()
