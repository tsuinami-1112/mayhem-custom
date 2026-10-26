"""GPS L1 C/A acquisition, used to prove a generated file really carries signal.

This is deliberately independent of the generators: it builds the Gold codes from
the IS-GPS-200 polynomials and correlates, so agreeing with gps-sdr-sim's channel
list is evidence both are right rather than evidence they share a bug.
"""

import numpy as np

# G2 phase-selector taps per PRN, IS-GPS-200 Table 3-Ia.
G2_TAPS = {
    1: (2, 6), 2: (3, 7), 3: (4, 8), 4: (5, 9), 5: (1, 9), 6: (2, 10),
    7: (1, 8), 8: (2, 9), 9: (3, 10), 10: (2, 3), 11: (3, 4), 12: (5, 6),
    13: (6, 7), 14: (7, 8), 15: (8, 9), 16: (9, 10), 17: (1, 4), 18: (2, 5),
    19: (3, 6), 20: (4, 7), 21: (5, 8), 22: (6, 9), 23: (1, 3), 24: (4, 6),
    25: (5, 7), 26: (6, 8), 27: (7, 9), 28: (8, 10), 29: (1, 6), 30: (2, 7),
    31: (3, 8), 32: (4, 9),
}

CODE_LENGTH = 1023
CODE_PERIOD = 1e-3


def ca_code(prn):
    """One period of the PRN's C/A code as +-1."""
    g1 = np.ones(10, dtype=np.int8)
    g2 = np.ones(10, dtype=np.int8)
    t1, t2 = G2_TAPS[prn]
    out = np.zeros(CODE_LENGTH, dtype=np.int8)
    for i in range(CODE_LENGTH):
        out[i] = g1[9] ^ (g2[t1 - 1] ^ g2[t2 - 1])
        f1 = g1[2] ^ g1[9]
        f2 = g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9]
        g1 = np.roll(g1, 1); g1[0] = f1
        g2 = np.roll(g2, 1); g2[0] = f2
    return 1 - 2 * out.astype(np.float32)


def acquire(iq, fs, prns=range(1, 33), coherent_ms=10, doppler=5000, step=250,
            threshold=8.0, freq_offset=0.0):
    """Search iq for each PRN. Returns [(prn, doppler_hz, code_phase, metric)].

    metric is the correlation peak over the median of the search plane, so it is a
    detection confidence rather than a C/N0 estimate. freq_offset shifts the search
    centre, for files where the signal is not at baseband zero (wide mode).
    """
    spc = int(round(fs * CODE_PERIOD))  # samples per code period
    need = spc * coherent_ms
    if iq.size < need:
        raise ValueError(f"need {need} samples at {fs/1e6:.3f} MS/s, got {iq.size}")

    idx = (np.arange(spc) * CODE_LENGTH / spc).astype(int)
    t = np.arange(spc) / fs
    freqs = np.arange(-doppler, doppler + 1, step) + freq_offset

    found = []
    for prn in prns:
        code_f = np.conj(np.fft.fft(ca_code(prn)[idx]))
        best = 0.0
        best_f = 0.0
        best_phase = 0
        noise = []
        for f in freqs:
            acc = np.zeros(spc)
            mixer = np.exp(-2j * np.pi * f * t)
            for k in range(coherent_ms):
                seg = iq[k * spc:(k + 1) * spc] * mixer
                acc += np.abs(np.fft.ifft(np.fft.fft(seg) * code_f))
            noise.append(np.median(acc))
            peak = acc.max()
            if peak > best:
                best, best_f, best_phase = peak, f, int(acc.argmax())
        metric = best / max(np.median(noise), 1e-12)
        if metric > threshold:
            found.append((prn, best_f, best_phase, metric))
    return found


def read_c8(path, samples=None):
    """Read a .C8 file as a complex array."""
    count = -1 if samples is None else 2 * samples
    raw = np.fromfile(path, dtype=np.int8, count=count)
    return raw[0::2].astype(np.float32) + 1j * raw[1::2].astype(np.float32)


def add_noise(iq, cn0_dbhz, fs, rng, n_sats=1):
    """Add white Gaussian noise putting each satellite at cn0_dbhz.

    iq holds the sum of n_sats satellites, so one satellite's carrier power is
    mean(|iq|^2)/n_sats. Noise power across the sampled band is then
    C / (10^(cn0/10) / fs). Pass the real satellite count or the noise floor comes
    out n_sats times too high.
    """
    carrier = np.mean(np.abs(iq) ** 2) / max(n_sats, 1)
    noise_power = carrier / (10 ** (cn0_dbhz / 10) / fs)
    sigma = np.sqrt(noise_power / 2)
    return iq + rng.normal(0, sigma, iq.size) + 1j * rng.normal(0, sigma, iq.size)
