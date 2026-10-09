# Multi-GNSS Sim file formats

The Multi-GNSS Sim app replays a baseband I/Q file from `/GPS/` on the SD card. It
does no signal synthesis: everything is decided when the file is generated on a PC
by `tools/make_gnss_sim_file.py`. This document is the contract between that
generator and the firmware (`firmware/baseband/proc_mgnss_sim.cpp` for the sample
formats, `firmware/application/external/mgnss_sim/` for the metadata).

Anything here that the generator and the firmware disagree about produces a file
that transmits noise, so both sides are tested against the reference packers in
`tools/gnss_sim/pack.py`.

## Byte-rate budget

Every format reads **~5 MB/s** off the SD card. That is the one fixed quantity: the
M0 side (`ReplayThread` -> `FileReader` -> 16 KiB buffers) is identical in all
three cases and identical to what GPS Sim already does today. Only the M4's
interpretation of the bytes changes.

| Format | Bits per I or Q | Samples per byte | Sample rate | SD rate |
|---|---|---|---|---|
| `.C8` | 8 | 0.5 | 2,600,000 | 5.20 MB/s |
| `.C2` | 2 | 2 | 10,000,000 | 5.00 MB/s |
| `.C1` | 1 | 4 | 20,000,000 | 5.00 MB/s |

20 MS/s is the radio's ceiling: `max283x::filter::bandwidths` tops out well above
it, but `MAX2837`'s usable TX path and the 20 MHz SPI clock
(`hackrf_hal.hpp:49-50`) put the practical limit there.

## Sample packing

In every format the **earliest sample occupies the least significant bits**, so
unpacking is a right-shift. I precedes Q within a sample.

### `.C8` — unchanged from GPS Sim

Signed 8-bit interleaved: `I0 Q0 I1 Q1 ...`, two bytes per complex sample, no
header. Byte-for-byte what `gps-sdr-sim -b 8` emits, which is why GPS Sim could
replay it with a `memcpy`.

### `.C2` — two complex samples per byte

```
bit    7 6   5 4   3 2   1 0
     [ Q1 ] [ I1 ] [ Q0 ] [ I0 ]
```

2-bit code to int8 level:

| Code | 0 | 1 | 2 | 3 |
|---|---|---|---|---|
| Level | -36 | -12 | +12 | +36 |

Quantiser thresholds sit at `{-sigma, 0, +sigma}` of the pre-quantisation signal.
That is close to the Lloyd-Max optimum for a Gaussian input (0.98 sigma, level
ratio 3.335 against our 3.0) and costs about 0.6 dB against infinite precision.

### `.C1` — four complex samples per byte

```
bit   7    6    5    4    3    2    1    0
     [Q3] [I3] [Q2] [I2] [Q1] [I1] [Q0] [I0]
```

1-bit code to int8 level: `0 -> -23`, `1 -> +23`.

Hard limiting costs the classic 2 dB (pi/2) against infinite precision.

### Level choice

All three formats are scaled to the same RMS magnitude, ~32 LSB, which is what
`gps-sdr-sim -b 8` produces (measured 32.14 on a reference run). Switching format
therefore does not change your RF level and you do not have to re-tune attenuators
between tests.

* `.C1`: `|z| = 23 * sqrt(2) = 32.5`, constant by construction.
* `.C2`: per-axis RMS 22.56, so `|z| = 31.91` (measured over 4e6 samples).

### M4 unpacking

Both packed formats unpack through a 256-entry byte lookup table built at startup
in RAM (the processor is heap-allocated by `main()`), not stored in the image:
`.C1` needs 256 x 4 complex samples (2 KiB), `.C2` 256 x 2 (1 KiB). Keeping them out of the image matters because an external
app's code plus its baseband image share the 40 KiB `m4_code` region on the
LPC4320 (HackRF One); it is 64 KiB only on the LPC4330 (HackRF Pro).

Per 2048-sample `execute()` block at 20 MS/s the M4 has ~102 us, about 20,000
cycles at 200 MHz, to expand 512 input bytes. A table lookup plus a 8-byte store
per input byte fits with room to spare, but the figure is measured during bring-up
rather than assumed.

## Short reads

`StreamOutput::read` returns fewer bytes than asked when the SD card has not kept
up ("Hole in transmission" in `stream_output.cpp`). GPS Sim leaves the rest of the
TX buffer holding whatever samples were there from an earlier block, which
transmits stale signal and tells nobody.

`proc_mgnss_sim` instead **zero-fills the remainder and increments an underrun
counter**, reported to the app in `GNSSSimStatsMessage`. Zero fill is a gap in
carrier rather than a phase discontinuity, which a tracking loop recovers from more
cleanly, and the counter means a lost lock on the bench is explained instead of
guessed at.

## Centre frequency

| Constellations | Centre | Why |
|---|---|---|
| Without BeiDou B1I | 1,575,420,000 | GPS/QZSS L1 C/A, Galileo E1 and BeiDou B1C all sit here |
| With BeiDou B1I | 1,568,259,000 | Midway between B1I (1,561,098,000) and L1; needs the 20 MHz span, so `.C1` only |

In wide mode each component is frequency-shifted by the generator: L1 signals to
`+7.161 MHz`, B1I to `-7.161 MHz`. In-band power retained:

| Signal | At 2.6 MS/s | At 10 MS/s | 20 MS/s wide |
|---|---|---|---|
| GPS / QZSS L1 C/A | 91.2% (-0.40 dB) | 98.1% (-0.09 dB) | 98.1% (-0.08 dB) |
| Galileo E1 / BeiDou B1C | 80.2% (-0.96 dB) | 93.7% (-0.28 dB) | 93.1% (-0.31 dB) |
| BeiDou B1I | n/a | n/a | 95.6% (-0.20 dB) |

Figures are the signals' power spectra -- `sinc^2` for BPSK(1), `sinc^2 tan^2`
for BOC(1,1) -- integrated numerically over the passband; reproduce with
`tools/gnss_sim/check_spectrum.py`.

GLONASS L1 (1,598-1,606 MHz) cannot be reached from any single centre that also
covers L1, so it is out of scope on this hardware.

## Noise

`.C1` requires added noise. Hard limiting a sum of noiseless satellites lets the
strongest capture the limiter and the rest disappear; real 1-bit receivers work
because thermal noise dominates the input. The generator therefore adds white
Gaussian noise scaled so each satellite lands at a chosen C/N0, default
45 dB-Hz (a normal clear-sky value), before quantising.

At 20 MS/s, 45 dB-Hz puts each satellite about 28 dB below the in-band noise, so
noise dominates as it must. A side effect is that the file no longer has the
unnaturally clean, multipath-free, uniform-C/N0 look of a raw `gps-sdr-sim` output.

Noise is on by default for `.C2` and `.C1`, and off for `.C8` so that the existing
GPS Sim behaviour is unchanged; `--cn0` overrides either way.

## Metadata sidecar

Same basename, uppercase `.TXT`, CRLF line endings, `key=value`:

```
center_frequency=1575420000
sample_rate=2600000
format=c8
gnss=GPS,GAL
start_time=2026/10/09,12:00:00
latitude=22.308889
longitude=113.914444
altitude=11.0
cn0=0
```

`center_frequency` and `sample_rate` are read by the shared
`read_metadata_file()`, which **rejects the file if either is missing** and leaves
the app on its built-in defaults. The rest are read by a small app-local scan, so
`capture_metadata` (shared with Capture, Replay and the rest) does not grow.

Both readers ignore keys they do not know, which is what lets one sidecar format
serve the old app and the new one.

* `format`: `c8` | `c2` | `c1`. Absent means `c8`, so a hand-written or
  firmware-written sidecar still works.
* `gnss`: comma list from `GPS`, `QZSS`, `GAL`, `BDS` (B1I), `BDSC` (B1C).
  Display only; the app never infers behaviour from it.
* `start_time`: UTC, the instant the first sample represents. The app adds elapsed
  playback to it and displays the result, so you can see at a glance the time your
  receiver ought to be reporting. **Baked in at generation**, and it steps back at
  every loop.
* `cn0`: dB-Hz, or `0` when no noise was added.

## File naming

The firmware derives the sidecar path by replacing the extension
(`metadata_file.cpp: replace_extension(".TXT")`), so **two data files with the same
basename would share one `.TXT`**. `HKIA.C8` and `HKIA.C1` together would leave the
old GPS Sim app reading `sample_rate=20000000` for its 2.6 MS/s file and
transmitting garbage.

The generator therefore gives each format a distinct basename:

| Format | Data | Sidecar | Plays in |
|---|---|---|---|
| `.C8` | `HKIA.C8` | `HKIA.TXT` | GPS Sim **and** Multi-GNSS Sim |
| `.C2` | `HKIA_2B.C2` | `HKIA_2B.TXT` | Multi-GNSS Sim only |
| `.C1` | `HKIA_1B.C1` | `HKIA_1B.TXT` | Multi-GNSS Sim only |

GPS Sim's picker filters on `.C8`, so the packed files never appear there. The
`.C*` wildcard used by Replay, IQ Trim and Remote resolves through
`is_cxx_capture_file()`, which matches only `.C8`, `.CU8` and `.C16`
(`file.cpp:545-548`), so packed files cannot be mistaken for 8-bit captures there
either.

## M0 <-> M4 messages

Two new IDs, appended before `MAX` (so existing IDs keep their values):

* `GNSSSimConfig = 119`, app to M4: which sample format to unpack. Sample rate
  keeps using the existing `SampleRateConfigMessage`.
* `GNSSSimStats = 120`, M4 to app: bytes consumed and the underrun count.
  `TXProgressMessage` carries only `progress` and `done` with no spare field, and
  widening a message every TX app receives to serve one app would be the wrong
  trade.
