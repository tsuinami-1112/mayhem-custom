# GPS Sim TX files

The **GPS Sim TX** app is a baseband replayer, not a signal synthesiser. It streams
a `.C8` file from this folder straight to the transmitter and does no GPS maths of
its own, so the L1 C/A waveform has to be generated on a PC first.

## File format

| | |
|---|---|
| Data file | `<NAME>.C8` — signed 8-bit interleaved I/Q (I,Q,I,Q…), no header |
| Metadata  | `<NAME>.TXT` — same basename, uppercase `.TXT` |
| Sample rate | 2,600,000 Sa/s |
| Centre frequency | 1,575,420,000 Hz (GPS L1) |

The app's file picker only lists `.C8`, and the firmware finds the metadata by
swapping the extension for `.TXT`. Without a readable `.TXT` the app silently falls
back to its built-in defaults, so always ship the pair together.

`.TXT` contents (CRLF line endings, as the firmware writes them):

```
center_frequency=1575420000
sample_rate=2600000
latitude=22.308889
longitude=113.914444
```

Only `center_frequency` and `sample_rate` are read by the app; `latitude` and
`longitude` are there to document what the file simulates.

## Included

`HKIA.TXT` — metadata for Hong Kong International Airport (VHHH/HKG), aerodrome
reference point 22°18′32″N 113°54′52″E. The matching `HKIA.C8` is ~1.5 GiB for
5 minutes, so it is not kept in the repository. Generate it with:

```sh
./tools/make_gps_sim_file.sh -o /path/to/sdcard/GPS/HKIA.C8
```

That script rewrites `HKIA.TXT` as well, so the pair always stays consistent. Run
it with `--lat`/`--lon`/`--alt` for other locations, and `--help` for all options.

## Which ephemeris

The generator needs a RINEX navigation file. That choice does **not** affect
positional accuracy: gps-sdr-sim encodes the same ephemeris into the simulated nav
message that it used to compute the pseudoranges, so the solution is
self-consistent and the receiver lands on the requested coordinates however stale
the orbits are. It only decides whether the simulated sky matches the real one,
which matters if the receiver cross-references a stored almanac or AGPS data.

| | Orbits | Reported time | Network |
|---|---|---|---|
| default | sample, retimed via `-T` | now | none |
| `--fetch` | real | ~2-3 h behind now | yes, cached |

`--fetch` pulls the IGS-merged daily broadcast ephemeris from NOAA CORS
(`brdc<ddd>0.<yy>n.gz`, open, no login) into `~/.cache/portapack-gps-sim` and
reuses it on later runs. Override the location with `GPS_EPH_CACHE` and the mirror
with `GPS_EPH_MIRROR`.

Two things about daily files are worth knowing. gps-sdr-sim will not start unless
the scenario time falls within an hour of one of the file's ephemeris epochs, so a
given file covers roughly a 28-hour window and refuses anything outside it. And
the file for the day in progress is published incrementally, so its newest epoch
trails real time by a couple of hours. The script reads the epochs out of the file
and picks the newest one, or clamps an explicit `-t` into range and says so.

## Notes

* Size is `duration × 5.2 MB/s`. FAT32 caps a single file at 4 GiB (≈826 s); the
  firmware also supports exFAT if you need longer.
* The app loops the file when **Loop** is ticked, but a loop steps GPS time
  backwards, so a receiver will usually drop lock and re-acquire at each wrap.
  Generate the longest file you can live with rather than relying on short loops.
* Transmit into a shielded enclosure or over coax with attenuation. GPS L1 is a
  protected aeronautical band — radiating it over the air is illegal in most
  jurisdictions and interferes with everything nearby.
