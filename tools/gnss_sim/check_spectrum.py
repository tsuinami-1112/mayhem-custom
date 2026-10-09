#!/usr/bin/env python3
"""Reproduce the in-band power and quantiser figures quoted in FORMAT.md.

The power spectral density of a BPSK(n) ranging code is Tc*sinc^2(f*Tc), and of a
BOC(1,1) code (Galileo E1-B/C, BeiDou B1C) is that times tan^2(pi*f*Tc/2). The
fraction of each signal's power that survives a given passband is the integral of
its PSD over that band divided by the total.

    ./check_spectrum.py
"""

import numpy as np

CHIP = 1.023e6  # C/A and E1 chip rate; B1I is 2x this
L1 = 1_575_420_000
B1I = 1_561_098_000
WIDE_CENTRE = (L1 + B1I) // 2  # 1_568_259_000


def inband(chip_rate, boc, lo, hi):
    """Fraction of signal power falling in [lo, hi) Hz, relative to its centre."""
    tc = 1.0 / chip_rate
    f = np.linspace(-120e6, 120e6, 4_800_001)
    psd = tc * np.sinc(f * tc) ** 2
    if boc:
        psd = np.nan_to_num(psd * np.tan(np.pi * f * tc / 2) ** 2)
    return psd[(f >= lo) & (f < hi)].sum() / psd.sum()


def db(x):
    return 10 * np.log10(x)


SIGNALS = [
    # name, chip rate, BOC?, centre frequency
    ("GPS / QZSS L1 C/A", CHIP, False, L1),
    ("Galileo E1 / BeiDou B1C", CHIP, True, L1),
    ("BeiDou B1I", 2 * CHIP, False, B1I),
]

print("In-band power, centred on each signal (narrow and mid modes)")
for name, chip, boc, _ in SIGNALS:
    cells = []
    for bw in (2.6e6, 10e6):
        p = inband(chip, boc, -bw / 2, bw / 2)
        cells.append(f"{bw/1e6:>4.1f} MS/s {100*p:5.1f}% ({db(p):+5.2f} dB)")
    print(f"  {name:26s} " + " | ".join(cells))

print(f"\n20 MS/s wide mode, centred on {WIDE_CENTRE/1e6:.3f} MHz")
for name, chip, boc, fc in SIGNALS:
    lo = WIDE_CENTRE - 10e6 - fc
    hi = WIDE_CENTRE + 10e6 - fc
    p = inband(chip, boc, lo, hi)
    print(f"  {name:26s} {100*p:5.1f}% ({db(p):+5.2f} dB)   offset {(fc-WIDE_CENTRE)/1e6:+.3f} MHz")

# Quantiser levels from FORMAT.md. Both are scaled for an RMS magnitude near 32,
# which is what gps-sdr-sim -b 8 produces, so changing format does not change the
# RF level you set with the attenuators.
print("\nQuantiser output levels (target |z| ~= 32)")
rng = np.random.default_rng(1)
n = 4_000_000
sigma = 1.0
levels = np.array([-36, -12, 12, 36])
i = levels[np.digitize(rng.standard_normal(n), [-sigma, 0.0, sigma])]
q = levels[np.digitize(rng.standard_normal(n), [-sigma, 0.0, sigma])]
print(f"  C2  per-axis RMS {np.sqrt((i**2).mean()):5.2f}   |z| RMS {np.sqrt((i**2+q**2).mean()):5.2f}")
print(f"  C1  per-axis RMS {23.0:5.2f}   |z| RMS {23*np.sqrt(2):5.2f}  (constant)")

print("\nQuantisation loss against infinite precision")
print(f"  1-bit hard limit  {db(2/np.pi):+.2f} dB")
print("  2-bit, threshold +-1 sigma   about -0.6 dB")

print("\nNoise, C/N0 = 45 dB-Hz")
cn0 = 10 ** 4.5
for fs in (2.6e6, 10e6, 20e6):
    print(f"  at {fs/1e6:>4.1f} MS/s: one satellite {db(cn0/fs):+5.1f} dB, ten {db(10*cn0/fs):+5.1f} dB vs in-band noise")
