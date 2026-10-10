"""Galileo E1B acquisition, used to prove a generated file really carries signal.

Companion to acquire.py (GPS L1 C/A), same shape and same peak-over-median
metric. Like that module it is deliberately independent of the generators: the
codes come from the published ICD tables (e1b_codes.txt, see
extract_e1b_codes.py) and the BOC(1,1) modulation is rebuilt here from the
constants in the ICD, so agreeing with a generator's channel list is evidence
both are right rather than evidence they share a bug.

Three things about E1B make it more than acquire.py with different codes:

  BOC(1,1).  Every code chip is multiplied by a 1.023 MHz square subcarrier, so
  each chip becomes two half-chips of opposite sign. The replica must carry that
  or the correlation collapses -- a plain BPSK replica sees the split spectrum as
  noise.

  One data symbol per code period.  E1B runs 250 symbols/s and the code period
  is 4 ms, so there is exactly one symbol per period and the correlation sign can
  flip from one period to the next. Integration is therefore coherent over one
  period at most, then non-coherent (magnitudes) across periods.

  Symbol boundaries sit inside a fixed window.  This is the subtle one. acquire.py
  slices fixed 1 ms windows and correlates circularly, which is fine for GPS
  because 20 C/A periods share a bit. Do that at 4 ms here and every window
  straddles a symbol boundary at the signal's code phase: the circular
  correlation adds a (tau/T) piece carrying the previous symbol to a (1-tau/T)
  piece carrying the current one, and when the symbol flips those cancel to
  |2*tau/T - 1|. The loss is up to 6 dB and depends on each PRN's code phase,
  which is exactly the thing that would scramble a metric-vs-elevation ordering.
  So instead we correlate an 8 ms window against a 4 ms replica zero-padded to
  8 ms. Lag tau of that circular correlation is then an honest sliding
  correlation over the 4 ms of signal starting at tau, and at the true code phase
  that span holds exactly one symbol. Only lags 0..SPC-1 are kept; windows
  advance by one code period so a given lag tracks consecutive code periods.
"""

import os

import numpy as np

try:  # scipy's pocketfft takes a worker count; numpy's does not.
    from scipy.fft import fft as _fft, ifft as _ifft

    def _FFT(a, axis=-1):
        return _fft(a, axis=axis, workers=os.cpu_count() or 1)

    def _IFFT(a, axis=-1):
        return _ifft(a, axis=axis, workers=os.cpu_count() or 1)
except ImportError:  # pragma: no cover
    def _FFT(a, axis=-1):
        return np.fft.fft(a, axis=axis)

    def _IFFT(a, axis=-1):
        return np.fft.ifft(a, axis=axis)

CHIP_RATE = 1.023e6          # chips/s
CODE_LENGTH = 4092           # chips per primary code period
CODE_PERIOD = 0.004          # s  (= CODE_LENGTH / CHIP_RATE)
SUBCARRIER_RATE = 1.023e6    # Hz, BOC(1,1): one subcarrier cycle per chip
SYMBOL_RATE = 250.0          # E1B symbols/s -- one per code period
NUM_PRNS = 50

CODES_TXT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "e1b_codes.txt")

_CHIPS = None


def _load_chips(path=CODES_TXT):
    """(50, 4092) int8 of +-1, row 0 = PRN 1. Cached."""
    global _CHIPS
    if _CHIPS is not None:
        return _CHIPS
    rows = [l.strip() for l in open(path) if l.strip() and not l.startswith("#")]
    if len(rows) != NUM_PRNS:
        raise ValueError(f"{path}: {len(rows)} codes, expected {NUM_PRNS}")
    out = np.empty((NUM_PRNS, CODE_LENGTH), dtype=np.int8)
    for i, hexstr in enumerate(rows):
        if len(hexstr) * 4 != CODE_LENGTH:
            raise ValueError(f"{path} line {i + 1}: {len(hexstr)} hex digits")
        nib = np.frombuffer(bytes.fromhex("0" + hexstr), dtype=np.uint8)
        bits = np.unpackbits(nib)[4:]  # drop the padding nibble we prepended
        # 0 -> +1, 1 -> -1. The absolute polarity is a per-PRN sign that the
        # magnitude metric cannot see, so a convention flip here is harmless.
        out[i] = 1 - 2 * bits.astype(np.int8)
    _CHIPS = out
    return out


def e1b_code(prn):
    """One period of the PRN's E1B primary code as +-1, CODE_LENGTH chips."""
    if not 1 <= prn <= NUM_PRNS:
        raise ValueError(f"PRN {prn} outside 1..{NUM_PRNS}")
    return _load_chips()[prn - 1].astype(np.float32)


def boc11_replica(prn, fs):
    """The PRN's BOC(1,1) waveform, one code period, sampled at fs.

    Each chip becomes two half-chips (+1, -1) -- that is the 1.023 MHz square
    subcarrier -- then the 2*CODE_LENGTH half-chip sequence is sampled by
    nearest-earlier index, the same way acquire.py samples its C/A chips.
    """
    chips = e1b_code(prn)
    half = np.repeat(chips, 2) * np.tile(np.array([1.0, -1.0], np.float32), CODE_LENGTH)
    spc = int(round(fs * CODE_PERIOD))
    idx = (np.arange(spc) * half.size / spc).astype(int)
    return half[idx]


def acquire(iq, fs, prns=range(1, NUM_PRNS + 1), periods=25, doppler=5000,
            step=None, threshold=8.0, freq_offset=0.0, half_sample=True):
    """Search iq for each E1B PRN. Returns [(prn, doppler_hz, code_phase, metric)].

    metric is the correlation peak over the median of the search plane, as in
    acquire.py, so it is a detection confidence rather than a C/N0 estimate.
    code_phase is in samples, within one 4 ms code period. freq_offset shifts the
    search centre for files whose signal is not at baseband zero.

    periods is the number of 4 ms code periods summed non-coherently; it needs
    periods+1 periods of samples because the windows are 8 ms and overlap. 25
    (100 ms) is the default: code Doppler at 5 kHz drifts the code phase by only
    ~0.8 samples over that span, so the peak does not smear across lags.

    step defaults to half an FFT bin (fs/(2*SPC) = 125 Hz at 2.6 MS/s). The
    coherent integration is one 4 ms code period, so the Doppler response is
    sinc(f_err * 0.004) with its first null at 250 Hz: acquire.py's 250 Hz step
    would leave a worst-case 125 Hz error, i.e. sinc(0.5) = -3.9 dB, which is
    enough to lose a weak satellite. 125 Hz leaves at most 62.5 Hz, sinc(0.25)
    = -0.9 dB. It is also exactly the bin spacing of the 8 ms FFT (fs/(2*SPC)),
    so every Doppler is a whole-bin roll of a spectrum computed once -- the whole
    Doppler search costs no extra forward FFTs.
    """
    spc = int(round(fs * CODE_PERIOD))            # 10400 at 2.6 MS/s
    nfft = 2 * spc                                # 8 ms window
    need = (periods + 1) * spc
    if iq.size < need:
        raise ValueError(f"need {need} samples at {fs/1e6:.3f} MS/s, got {iq.size}")

    bin_hz = fs / nfft
    if step is None:
        step = bin_hz
    # Mixing by exp(-2j*pi*f*t) shifts the spectrum, and for f a whole number of
    # bins that is just a roll -- so quantise the requested step onto the bin
    # grid and do the Doppler search with np.roll instead of 2*periods*nfreq
    # time-domain mixes and forward FFTs.
    bins_per_step = max(1, int(round(step / bin_hz)))
    max_bin = int(np.floor(doppler / bin_hz))
    offset_bins = int(round(freq_offset / bin_hz))
    shifts = np.arange(-max_bin, max_bin + 1, bins_per_step) + offset_bins

    # periods overlapping 8 ms windows, one forward FFT each.
    starts = np.arange(periods) * spc
    win = np.empty((periods, nfft), dtype=np.complex64)
    for k, s in enumerate(starts):
        win[k] = iq[s:s + nfft]
    spec = _FFT(win, axis=1)                      # (periods, nfft)

    # Replica spectra, zero-padded from 4 ms to 8 ms so lag tau is a true sliding
    # correlation over the 4 ms starting at tau rather than a circular one.
    # half_sample adds a second replica sampled half a sample later: BOC(1,1) has
    # a 3-per-chip autocorrelation slope and one sample is 0.39 chip at 2.6 MS/s,
    # so a worst-case half-sample misalignment costs ~7.7 dB and varies per PRN;
    # taking the better of two half-sample-apart replicas bounds that at ~3 dB.
    phases = (0.0, 0.5) if half_sample else (0.0,)

    found = []
    for prn in prns:
        chips = e1b_code(prn)
        half = np.repeat(chips, 2) * np.tile(np.array([1.0, -1.0], np.float32),
                                            CODE_LENGTH)
        best = 0.0
        best_f = 0.0
        best_phase = 0
        noise = []
        for ph in phases:
            idx = ((np.arange(spc) + ph) * half.size / spc).astype(int) % half.size
            rep = np.zeros(nfft, dtype=np.complex64)
            rep[:spc] = half[idx]
            code_f = np.conj(_FFT(rep))
            for m in shifts:
                # roll(spec, m) along the frequency axis == mixing down by
                # -m*bin_hz; we want a mix of -f, so the search frequency is
                # f = m*bin_hz.
                acc = np.abs(_IFFT(np.roll(spec, -m, axis=1) * code_f,
                                   axis=1))[:, :spc].sum(axis=0)
                noise.append(np.median(acc))
                peak = float(acc.max())
                if peak > best:
                    best, best_f, best_phase = peak, m * bin_hz, int(acc.argmax())
        metric = best / max(float(np.median(noise)), 1e-12)
        if metric > threshold:
            found.append((prn, best_f, best_phase, metric))
    return found


def read_i16(path, samples=None, offset_samples=0):
    """Read an interleaved signed-16-bit I,Q file as a complex array."""
    count = -1 if samples is None else 2 * samples
    raw = np.fromfile(path, dtype="<i2", count=count,
                      offset=4 * offset_samples)
    return (raw[0::2].astype(np.float32) + 1j * raw[1::2].astype(np.float32))


def main():  # pragma: no cover
    import argparse
    import time

    p = argparse.ArgumentParser(description="Galileo E1B acquisition check")
    p.add_argument("path")
    p.add_argument("--fs", type=float, default=2_600_000)
    p.add_argument("--periods", type=int, default=25)
    p.add_argument("--prns", default="", help="e.g. 10,12,24 (default: all 50)")
    p.add_argument("--threshold", type=float, default=8.0)
    p.add_argument("--skip-ms", type=float, default=0.0)
    p.add_argument("--no-half-sample", action="store_true")
    a = p.parse_args()

    prns = ([int(x) for x in a.prns.split(",")] if a.prns
            else range(1, NUM_PRNS + 1))
    spc = int(round(a.fs * CODE_PERIOD))
    iq = read_i16(a.path, samples=(a.periods + 1) * spc,
                  offset_samples=int(a.skip_ms * 1e-3 * a.fs))
    t0 = time.time()
    found = acquire(iq, a.fs, prns=prns, periods=a.periods,
                    threshold=a.threshold, half_sample=not a.no_half_sample)
    dt = time.time() - t0
    print(f"{len(iq)/a.fs*1000:.0f} ms of signal, {a.periods} code periods, "
          f"{dt:.1f} s")
    print(f"{'PRN':>4} {'doppler':>9} {'phase':>7} {'metric':>8}")
    for prn, f, ph, m in sorted(found, key=lambda r: -r[3]):
        print(f"{prn:>4} {f:>+9.0f} {ph:>7} {m:>8.2f}")


if __name__ == "__main__":  # pragma: no cover
    main()
