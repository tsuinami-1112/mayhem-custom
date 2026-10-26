#!/usr/bin/env python3
"""Checks the sample formats against FORMAT.md, and that signal survives them.

    ./test_pack.py [path/to/HKIA.C8]

The last test is the one that matters: pack a real GPS signal into each format,
unpack it the way the firmware will, and confirm the satellites still acquire.
"""

import sys
import numpy as np

import acquire
import pack

FAILURES = []


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  [{detail}]" if detail else ""))
    if not ok:
        FAILURES.append(name)


print("Bit layout, against the diagrams in FORMAT.md")

# c1: four samples per byte, earliest in the low bits, I before Q.
# Make sample 0 = (+,-), 1 = (-,+), 2 = (+,+), 3 = (-,-)
iq = np.array([1 - 1j, -1 + 1j, 1 + 1j, -1 - 1j], dtype=np.complex64)
b = pack.pack_c1(iq)[0]
expect = (1 << 0) | (0 << 1) | (0 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (0 << 6) | (0 << 7)
check("c1 packs 4 samples per byte, earliest in low bits", b == expect, f"0x{b:02X} vs 0x{expect:02X}")

# c2: two samples per byte, 2 bits each, I in the low half of its nibble.
iq = np.array([3 - 3j, -1 + 1j], dtype=np.complex64)  # sigma=2 -> codes 3,0 and 1,2
b = pack.pack_c2(iq)[0]
got = [(b >> 0) & 3, (b >> 2) & 3, (b >> 4) & 3, (b >> 6) & 3]
check("c2 packs 2 samples per byte as I0,Q0,I1,Q1", got == [3, 0, 1, 2], f"codes {got}")

print("\nLevels and RMS, against the table in FORMAT.md")
rng = np.random.default_rng(7)
n = 400_000
sig = rng.normal(0, 10, n) + 1j * rng.normal(0, 10, n)

for fmt, want_rms in (("c8", 32.0), ("c2", 31.9), ("c1", 32.5)):
    u = pack.unpack(pack.PACKERS[fmt](sig), fmt)
    rms = np.sqrt(np.mean(np.abs(u) ** 2))
    check(f"{fmt} RMS magnitude ~= {want_rms}", abs(rms - want_rms) < 1.0, f"{rms:.2f}")
    levels = set(np.unique(u.real).astype(int)) | set(np.unique(u.imag).astype(int))
    if fmt == "c1":
        check("c1 uses only +-23", levels <= {-23, 23}, str(sorted(levels)))
    elif fmt == "c2":
        check("c2 uses only +-12, +-36", levels <= {-36, -12, 12, 36}, str(sorted(levels)))

print("\nThe unpacker and the firmware's lookup tables describe the same mapping")
t1, t2 = pack.firmware_tables()
# Feed every byte through unpack() and compare against the table it should equal.
every = bytes(range(256))
u1 = pack.unpack(every, "c1")
ok1 = all(
    u1[byte * 4 + k].real == t1[byte, k, 0] and u1[byte * 4 + k].imag == t1[byte, k, 1]
    for byte in range(256) for k in range(4)
)
check("c1 table matches unpack() for all 256 bytes", ok1)
u2 = pack.unpack(every, "c2")
ok2 = all(
    u2[byte * 2 + k].real == t2[byte, k, 0] and u2[byte * 2 + k].imag == t2[byte, k, 1]
    for byte in range(256) for k in range(2)
)
check("c2 table matches unpack() for all 256 bytes", ok2)

print("\nShort reads decode to silence, not a DC tone")
# A zero input byte is the lowest level on both axes in the packed formats, which
# is why proc_mgnss_sim zero-fills its OUTPUT buffer rather than its input.
for fmt, lowest in (("c2", -36), ("c1", -23)):
    z = pack.unpack(bytes([0]), fmt)
    check(f"{fmt}: a zero input byte is NOT silence (it is {lowest})",
          np.all(z.real == lowest) and np.all(z.imag == lowest), str(z[0]))

print("\nSignal survives each format (acquisition after pack and unpack)")
path = sys.argv[1] if len(sys.argv) > 1 else None
if not path:
    print("  SKIP  no .C8 given; pass one to run this")
else:
    # Compare each format's acquisition metric against C8's on the same noisy
    # input. Absolute detection is not the question -- quantisation loss is -- and
    # the metric is a peak-over-median confidence, so a power loss of X dB shows up
    # as roughly X/2 dB here. Predicted: -0.6 dB for c2, -1.96 dB for c1.
    fs = 2_600_000
    clean = acquire.read_c8(path, samples=fs // 10)  # 100 ms
    base = acquire.acquire(clean, fs)
    prns = sorted(p for p, _, _, _ in base)
    print(f"  reference (unpacked .C8, no noise): {prns}")
    check("reference file acquires at least 4 satellites", len(prns) >= 4,
          f"{len(prns)} found")

    rng = np.random.default_rng(3)
    noisy = acquire.add_noise(clean, 55.0, fs, rng, n_sats=len(prns))
    usable = (noisy.size // 4) * 4
    metrics = {}
    for fmt in ("c8", "c2", "c1"):
        u = pack.unpack(pack.PACKERS[fmt](noisy[:usable]), fmt)
        r = acquire.acquire(u, fs, prns=prns, threshold=0.0)
        metrics[fmt] = np.array([m for _, _, _, m in r])
        print(f"  {fmt}: " + " ".join(f"{m:.1f}" for m in metrics[fmt]))

    check("c8 round trip still detects every reference satellite",
          bool(np.all(metrics["c8"] > 6.0)), f"min {metrics['c8'].min():.1f}")

    for fmt, floor, predicted in (("c2", 0.90, 0.6), ("c1", 0.78, 1.96)):
        ratio = float(np.mean(metrics[fmt] / metrics["c8"]))
        loss_db = -20 * np.log10(ratio)
        check(f"{fmt} keeps >= {floor:.0%} of c8's metric "
              f"(predicted ~{predicted} dB power loss)",
              ratio >= floor, f"ratio {ratio:.3f} = {loss_db:.2f} dB amplitude")

print()
if FAILURES:
    print(f"{len(FAILURES)} check(s) failed: {FAILURES}")
    sys.exit(1)
print("all checks passed")
