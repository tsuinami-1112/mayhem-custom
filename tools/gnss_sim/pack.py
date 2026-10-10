"""Reference packers and unpackers for the Multi-GNSS Sim sample formats.

These are the authority the firmware's lookup tables in
firmware/baseband/proc_mgnss_sim.cpp must agree with. The levels and bit layout
are specified in FORMAT.md; run test_pack.py to check a round trip.
"""

import numpy as np

# int8 output levels. Chosen so every format lands near an RMS magnitude of 32,
# which is what gps-sdr-sim -b 8 produces, so changing format does not change the
# RF level set by the attenuators.
LEVELS_C1 = np.array([-23, 23], dtype=np.int8)
LEVELS_C2 = np.array([-36, -12, 12, 36], dtype=np.int8)

SAMPLE_RATES = {"c8": 2_600_000, "c2": 10_000_000, "c1": 20_000_000}
EXTENSIONS = {"c8": ".C8", "c2": ".C2", "c1": ".C1"}
SUFFIXES = {"c8": "", "c2": "_2B", "c1": "_1B"}


def _quantise(x, thresholds):
    """Map a real-valued array to quantiser codes via np.digitize."""
    return np.digitize(x, thresholds).astype(np.uint8)


def pack_c8(iq, scale=None):
    """Complex array -> signed 8-bit interleaved I,Q (two bytes per sample).

    `scale` multiplies before rounding. Pass it when packing a long file in
    chunks: deriving it from each chunk separately would let the quantiser drift
    with the signal, so the caller computes one value for the whole file.
    """
    if scale is None:
        scale = 32.0 / max(np.sqrt(np.mean(np.abs(iq) ** 2)), 1e-12)
    out = np.empty(iq.size * 2, dtype=np.int8)
    out[0::2] = np.clip(np.rint(iq.real * scale), -127, 127)
    out[1::2] = np.clip(np.rint(iq.imag * scale), -127, 127)
    return out.tobytes()


def pack_c2(iq, sigma=None):
    """Complex array -> 2-bit I and Q, two complex samples per byte.

    bit    7 6   5 4   3 2   1 0
         [ Q1 ] [ I1 ] [ Q0 ] [ I0 ]

    `sigma` sets the quantiser thresholds at +/-sigma. Pass it when chunking, for
    the same reason as pack_c8's `scale`.
    """
    if iq.size % 2:
        raise ValueError("c2 needs an even sample count")
    if sigma is None:
        sigma = max(np.sqrt(np.mean(np.concatenate([iq.real, iq.imag]) ** 2)), 1e-12)
    thr = [-sigma, 0.0, sigma]
    ci, cq = _quantise(iq.real, thr), _quantise(iq.imag, thr)
    out = (
        (ci[0::2] << 0) | (cq[0::2] << 2) | (ci[1::2] << 4) | (cq[1::2] << 6)
    ).astype(np.uint8)
    return out.tobytes()


def pack_c1(iq):
    """Complex array -> 1-bit I and Q, four complex samples per byte.

    bit   7    6    5    4    3    2    1    0
         [Q3] [I3] [Q2] [I2] [Q1] [I1] [Q0] [I0]
    """
    if iq.size % 4:
        raise ValueError("c1 needs a sample count that is a multiple of 4")
    ci = (iq.real >= 0).astype(np.uint8)
    cq = (iq.imag >= 0).astype(np.uint8)
    out = np.zeros(iq.size // 4, dtype=np.uint8)
    for k in range(4):
        out |= ci[k::4] << (2 * k)
        out |= cq[k::4] << (2 * k + 1)
    return out.tobytes()


PACKERS = {"c8": pack_c8, "c2": pack_c2, "c1": pack_c1}


def unpack(data, fmt):
    """Bytes -> complex array of int8 levels. Mirrors proc_mgnss_sim's tables."""
    b = np.frombuffer(data, dtype=np.uint8)
    if fmt == "c8":
        s = b.view(np.int8)
        return s[0::2].astype(np.float32) + 1j * s[1::2].astype(np.float32)
    if fmt == "c2":
        i = np.empty(b.size * 2, dtype=np.int8)
        q = np.empty(b.size * 2, dtype=np.int8)
        for k in range(2):
            i[k::2] = LEVELS_C2[(b >> (4 * k)) & 3]
            q[k::2] = LEVELS_C2[(b >> (4 * k + 2)) & 3]
        return i.astype(np.float32) + 1j * q.astype(np.float32)
    if fmt == "c1":
        i = np.empty(b.size * 4, dtype=np.int8)
        q = np.empty(b.size * 4, dtype=np.int8)
        for k in range(4):
            i[k::4] = LEVELS_C1[(b >> (2 * k)) & 1]
            q[k::4] = LEVELS_C1[(b >> (2 * k + 1)) & 1]
        return i.astype(np.float32) + 1j * q.astype(np.float32)
    raise ValueError(f"unknown format {fmt!r}")


def firmware_tables():
    """Rebuild proc_mgnss_sim.cpp's two tables, to diff against the C++ source."""
    t1 = np.zeros((256, 4, 2), dtype=np.int8)
    t2 = np.zeros((256, 2, 2), dtype=np.int8)
    for byte in range(256):
        for k in range(4):
            t1[byte, k] = (LEVELS_C1[(byte >> (2 * k)) & 1],
                           LEVELS_C1[(byte >> (2 * k + 1)) & 1])
        for k in range(2):
            t2[byte, k] = (LEVELS_C2[(byte >> (4 * k)) & 3],
                           LEVELS_C2[(byte >> (4 * k + 2)) & 3])
    return t1, t2
