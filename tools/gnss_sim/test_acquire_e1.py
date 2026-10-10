#!/usr/bin/env python3
"""Checks that a generated Galileo E1 file really carries acquirable E1B signal.

    ./test_acquire_e1.py path/to/gal.bin

Searches all 50 E1B PRNs and compares the result against what the generator said
it put in the file. Three things have to hold, and the second and third are the
ones that make this more than a tautology:

  the satellites that should be there acquire;
  the 44 that should not be there do not, with a clear margin;
  the detection metric tracks elevation.

That last one is the strongest evidence of correctness. Code and BOC(1,1)
modulation errors, a wrong code phase convention or a broken Doppler search all
tend to give either nothing at all or a scatter of lookalike peaks; getting six
right PRNs ordered by elevation out of a 50-PRN search is hard to do by accident.
"""

import sys
import time

import numpy as np
from scipy.stats import spearmanr

import acquire_e1

# What the generator printed for /tmp/.../galout/gal.bin: Hong Kong Int'l Airport
# (22.308889 N, 113.914444 E), 2022/02/20 12:00:00. PRN -> elevation in degrees.
EXPECTED = {10: 25.6, 12: 45.0, 24: 26.6, 26: 11.5, 31: 70.9, 33: 47.4}

FS = 2_600_000
PERIODS = 25  # 4 ms each; 104 ms of signal read, 100 ms integrated

FAILURES = []


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  [{detail}]" if detail else ""))
    if not ok:
        FAILURES.append(name)


path = sys.argv[1] if len(sys.argv) > 1 else None
if not path:
    print(__doc__)
    sys.exit(2)

spc = int(round(FS * acquire_e1.CODE_PERIOD))
iq = acquire_e1.read_i16(path, samples=(PERIODS + 1) * spc)
print(f"{path}: {iq.size} samples = {iq.size / FS * 1000:.0f} ms at {FS/1e6:.3f} MS/s")

print(f"\nSearching all {acquire_e1.NUM_PRNS} E1B PRNs, +-5 kHz Doppler")
t0 = time.time()
# threshold=0 so every PRN comes back and the absent ones give us the
# false-alarm distribution to judge the threshold against.
res = acquire_e1.acquire(iq, FS, periods=PERIODS, threshold=0.0)
print(f"  {time.time() - t0:.1f} s")

metric = {prn: m for prn, _, _, m in res}
info = {prn: (f, ph) for prn, f, ph, _ in res}
threshold = 8.0  # acquire.py's default; the margin check below justifies it here
found = sorted(p for p, m in metric.items() if m > threshold)

print(f"\nDetections above the threshold of {threshold}")
print(f"  {'PRN':>4} {'elev':>6} {'doppler':>9} {'phase':>7} {'metric':>8}")
for prn in sorted(found, key=lambda p: -metric[p]):
    elev = f"{EXPECTED[prn]:.1f}" if prn in EXPECTED else "-"
    f, ph = info[prn]
    print(f"  {prn:>4} {elev:>6} {f:>+9.0f} {ph:>7} {metric[prn]:>8.2f}")

hit = [p for p in EXPECTED if p in found]
missed = sorted(set(EXPECTED) - set(found))
extra = sorted(set(found) - set(EXPECTED))

print()
check(f"at least 4 of the {len(EXPECTED)} expected PRNs acquire",
      len(hit) >= 4, f"{len(hit)} found, missed {missed}" if missed
      else f"all {len(hit)} found")
check("no PRN outside the expected set acquires", not extra,
      f"false positives {extra}" if extra else "none")

# The margin is what makes "no PRN outside the set" mean anything: if the 44
# absent PRNs were sitting just under the threshold the check would be luck.
absent = [m for p, m in metric.items() if p not in EXPECTED]
worst_hit = min(metric[p] for p in hit) if hit else 0.0
loudest_absent = max(absent)
check("the weakest real detection beats the loudest absent PRN by 3x",
      worst_hit > 3 * loudest_absent,
      f"weakest real {worst_hit:.2f} vs loudest absent {loudest_absent:.2f} "
      f"(ratio {worst_hit / max(loudest_absent, 1e-9):.1f}x), "
      f"threshold {threshold}")
check(f"the threshold sits inside that gap",
      loudest_absent < threshold < worst_hit,
      f"{loudest_absent:.2f} < {threshold} < {worst_hit:.2f}")

# Elevation drives received power, so the metric should rise with it. Rank
# correlation, not a fit: the metric is a peak-over-median confidence and the
# generator's own antenna/path model is not reproduced here, so only the
# ordering is meaningful -- and PRNs within a degree of each other may swap.
elevs = [EXPECTED[p] for p in hit]
metrics = [metric[p] for p in hit]
rho = float(spearmanr(elevs, metrics).statistic)
print(f"\n  elevation {['%.1f' % e for e in elevs]}")
print(f"  metric    {['%.1f' % m for m in metrics]}")
check("the metric ordering tracks elevation (Spearman > 0.5)", rho > 0.5,
      f"rho = {rho:.3f}")

# And the loudest satellite really is the highest one, which is the single most
# falsifiable prediction in the ground truth.
top_metric = max(hit, key=lambda p: metric[p])
top_elev = max(hit, key=lambda p: EXPECTED[p])
check("the strongest detection is the highest satellite",
      top_metric == top_elev,
      f"strongest PRN {top_metric}, highest PRN {top_elev} "
      f"({EXPECTED[top_elev]:.1f} deg)")

# Internal consistency, using no ground truth at all: carrier Doppler and code
# rate come from the same satellite motion, so a satellite seen at Doppler f_d
# has its code phase drift by -(f_d / 1575.42 MHz) * fs samples per second. Over
# a 2.5 s gap that is -9 samples at +2250 Hz and +8 at -1875 Hz -- opposite
# signs, several samples apart. Nothing in the acquisition search couples the
# Doppler axis to the code-phase axis, so if the two agree they are both
# measuring a real satellite rather than a search artefact.
GAP_S = 2.5
print(f"\nRe-acquiring the same PRNs {GAP_S} s later, to check that code phase")
print("drifts by what the measured Doppler predicts")
iq2 = acquire_e1.read_i16(path, samples=(PERIODS + 1) * spc,
                          offset_samples=int(GAP_S * FS))
res2 = acquire_e1.acquire(iq2, FS, prns=found, periods=PERIODS, threshold=0.0)
later = {prn: (f, ph) for prn, f, ph, _ in res2}

print(f"  {'PRN':>4} {'doppler':>9} {'predicted':>10} {'observed':>9} {'error':>7}")
drift_err = []
for prn in sorted(found):
    f, ph0 = info[prn]
    f2, ph1 = later[prn]
    predicted = -(f / acquire_e1.CARRIER_FREQ) * FS * GAP_S
    observed = ph1 - ph0
    drift_err.append(abs(observed - predicted))
    print(f"  {prn:>4} {f:>+9.0f} {predicted:>+10.2f} {observed:>+9d} "
          f"{observed - predicted:>+7.2f}")

check("Doppler unchanged over the gap for every PRN",
      all(abs(later[p][0] - info[p][0]) <= FS / (2 * spc) for p in found),
      "within one 125 Hz bin")
check("code-phase drift matches the measured Doppler to within 1.5 samples",
      max(drift_err) < 1.5, f"worst error {max(drift_err):.2f} samples")

print()
if FAILURES:
    print(f"{len(FAILURES)} check(s) failed: {FAILURES}")
    sys.exit(1)
print("all checks passed")
