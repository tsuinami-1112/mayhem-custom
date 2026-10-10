# Clean-slate setup: this fork's firmware + a fresh SD card

For a **HackRF One + PortaPack** (LPC4320), built on **Windows + WSL**. Everything
below is read out of this tree; every behavioural claim cites `file:line` so you can
check it. Anything not verifiable from the repo is marked and collected in
[§11](#11-not-verified-from-this-repo).

Target state: firmware **and** `/APPS/*.ppma` from the *same* build of this branch, a
freshly formatted card holding the repo's `sdcard/` template, and an HKIA GNSS file in
`/GPS/`.

> **Why start clean.** The flashed firmware and the `.ppma` external apps on the card
> are version-locked to each other ([§5](#5-the-firmware--sd-card-coupling)). Carrying
> over a card written by a different firmware gives you apps that are silently absent
> or dead on arrival. Reformatting also clears the other half of the problem: a
> half-written multi-gigabyte file left by an interrupted copy
> ([§10](#10-troubleshooting)).

---

## 1. What you need on the PC

Work inside the **WSL filesystem**, e.g. `~/mayhem-custom`, not `/mnt/c/...`.
`/mnt/c` is a drvfs mount: slow for a toolchain extract and a full build, and it is the
checkout that `.gitattributes` was added to defend against — see the commit message in
`.gitattributes:1-11` and commit `6bb43dc`.

### 1.1 apt packages

`./mbt deps` installs the build set (`mbt:72-74`):

```sh
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  git tar wget dfu-util cmake make python3 bzip2 lz4 curl hackrf \
  python3-setuptools python3-yaml binutils ccache ninja-build
```

The ARM toolchain is **not** an apt package: `mbt` downloads GCC `9.2.1` into
`./armbin` and prepends it to `PATH` (`mbt:98-114`, `:118-126`, `:921`, via
`scripts/toolchain/mbtenv.sh`).
A different compiler version builds but prints a mismatch warning
(`firmware/CMakeLists.txt:46-49,97-104`).

For the GNSS generators in [§7](#7-generate-and-install-the-hkia-gnss-files) you need
more, and **`mbt deps` does not install these**:

```sh
sudo apt-get install -y gcc g++ python3-numpy gzip
```

`tools/make_gps_sim_file.sh:88-92` compiles gps-sdr-sim with `gcc`;
`tools/gnss_sim/make_gnss_sim_file.py:74` does the same and `:121` builds
galileo-sdr-sim with `g++ -std=c++17`; the script imports `numpy`
(`make_gnss_sim_file.py:26`) and shells out to `curl`/`gunzip` for `--fetch`
(`:219,:223`).

### 1.2 Line endings

Git for Windows defaults to `core.autocrlf=true`, which rewrites shebang scripts to
CRLF; the kernel then looks for an interpreter literally named `bash\r` and the build
dies. This repo ships `.gitattributes` forcing LF on `*.sh`, `*.py` and `mbt`
(`.gitattributes:11-13`), so a **fresh** clone is safe.

A checkout made **before** commit `6bb43dc` still has CRLF on disk. Re-normalise it:

```sh
cd ~/mayhem-custom
git config core.autocrlf false
git rm -r --cached . -q
git reset --hard HEAD
# sanity check: must print nothing
file mbt tools/make_gps_sim_file.sh tools/gnss_sim/*.py | grep CRLF
```

### 1.3 Clone

```sh
git clone --recurse-submodules <your-fork-url> ~/mayhem-custom
cd ~/mayhem-custom
./mbt doctor          # reports missing host tools, does not build
```

`./mbt build` syncs submodules by default (`mbt:89-93`, `mbt:194`); `--no-submodule-sync`
skips it.

---

## 2. Build the firmware

```sh
cd ~/mayhem-custom
./mbt build --device hackrf-one
```

`hackrf-one` passes **no extra CMake args** — it is the default memory/flash layout
(`mbt:361-366`). For contrast, `hackrf-pro` adds `-DBOARD=PRALINE` and `portarf` adds
`-DFLASH_MB_SIZE=2 -DFLASH_MB_LIMIT_SIZE=2` (`mbt:366-372`). Device aliases accepted:
`1|one|hackrf|hackrf-one` (`mbt:138-140`).

`mbt` records the device in `build/.mbt_device` and wipes `build/` if you switch
devices (`mbt:436-446`), so you cannot accidentally mix PRALINE objects into a
HackRF One image.

### 2.1 Outputs

The `ppfw` target is in `ALL` (`firmware/CMakeLists.txt:157-160`), so a plain build
produces the package too.

| Path | What it is |
|---|---|
| `build/firmware/portapack-mayhem-firmware.bin` | Bare SPI-flash image, exactly 1 MiB. **Firmware only — no external apps.** For `hackrf_spiflash`. (`firmware/CMakeLists.txt:36-37,83-89`) |
| `build/firmware/portapack-mayhem_OCI.ppfw.tar` | The on-device update package. (`firmware/CMakeLists.txt:38,122-136`) |
| `build/firmware/application/*.ppma` | External apps, one per app in `EXTAPPLIST` (`firmware/application/external/external.cmake:424`) |
| `build/firmware/standalone/*/*.ppmp` | Standalone apps (`pacman`, `digitalrain`) |
| `build/hackrf/firmware/hackrf_usb/hackrf_usb.dfu` | HackRF USB firmware as a DFU image — the unbrick payload ([§9](#9-recovery-dfu-unbrick)) |

`mbt` prints the first three paths itself at `mbt:489-492`.

### 2.2 What is inside the `.ppfw.tar`

Two top-level directories, built at `firmware/CMakeLists.txt:125-132`:

```
FIRMWARE/portapack-mayhem_<VERSION_NOHASH>.bin
APPS/*.ppma        (from build/firmware/application/)
APPS/*.ppmp        (from build/firmware/standalone/*/)
```

`VERSION_NOHASH` is literally `dev` unless you set `VERSION_STRING`
(`CMakeLists.txt:82-85`), so a normal developer build gives
`FIRMWARE/portapack-mayhem_dev.bin`. Verify before copying:

```sh
tar -tf build/firmware/portapack-mayhem_OCI.ppfw.tar | grep '^FIRMWARE/'
# FIRMWARE/portapack-mayhem_dev.bin
```

To stamp a version, use the **environment variable** — `CMakeLists.txt:66` reads
`$ENV{VERSION_STRING}`:

```sh
VERSION_STRING=hkia-1 ./mbt build --device hackrf-one
```

`mbt:223` advertises `--cmake-arg=-DVERSION_STRING=my-build` instead. That looks wrong:
`CMakeLists.txt:66` unconditionally assigns `VERSION` from the environment, which would
discard a `-D` cache entry of that name. **Flagged, not tested** — use the env var.

### 2.3 The 40 KiB budget (why this board is tighter than a HackRF Pro)

An external app's M0 code and the baseband image it carries both live in
`portapack::memory::map::m4_code`, and the sum has to fit
(`firmware/tools/export_external_apps.py:100-103`). That region is:

| Board | `M4_CODE_SIZE` | Where |
|---|---|---|
| HackRF One (LPC4320) | **40960** (40 KiB) | `firmware/chibios/os/ports/GCC/ARMCMx/rules.cmake:149` |
| HackRF Pro / PRALINE (LPC4330) | 65536 (64 KiB) | `rules.cmake:124` |

The build hard-fails with a byte-exact message if an app overruns it
(`export_external_apps.py:106-115,161,192`). One app's *link* region is 32 KiB
(`firmware/tools/external_app_info.py:33`), which is the separate, smaller cap on app
code alone. So: a 40 KiB failure on this board is not a bug, and a `.ppma` built for a
HackRF Pro may simply not fit here.

---

## 3. Format the SD card

### 3.1 Filesystem

| | FAT32 | exFAT |
|---|---|---|
| Supported by this firmware | yes | **yes** — `_FS_EXFAT 1`, `firmware/common/ffconf.h:192` |
| Max single file | **4 GiB − 1** | effectively unlimited |
| Max GNSS file duration at 5.2 MB/s | ≈826 s (`sdcard/GPS/README.md`) | not the limit |

exFAT needs LFN support, which is on (`_USE_LFN 3`, `_MAX_LFN 255`, `ffconf.h:93-94`).

**Recommendation:** FAT32 if every file stays under 4 GiB — that covers the 300 s
default (1.56 GB, [§7](#7-generate-and-install-the-hkia-gnss-files)) and anything up to
~13 minutes. exFAT only if you need longer than that in one file. Note that the old
firmware may not have had exFAT enabled, which is a reason to prefer FAT32 if you ever
want to move the card back.

Hard constraints from `ffconf.h`:

* **512-byte sectors only** — `_MIN_SS` and `_MAX_SS` are both `512` (`ffconf.h:157-158`).
  A card or formatter using 4096-byte logical sectors will not mount. (Standard SD cards
  report 512; this bites "advanced format" USB adapters.)
* **One volume, no multi-partition** — `_VOLUMES 1` (`ffconf.h:138`),
  `_MULTI_PARTITION 0` (`ffconf.h:149`). Use a single primary partition.
* **The firmware cannot format** — `_USE_MKFS 0` (`ffconf.h:41`). Formatting is a PC job.

### 3.2 Doing it

You cannot format removable media from WSL2; do it from Windows. Open an
**Administrator** Command Prompt, with `D:` standing in for your card's letter:

```bat
rem FAT32, 32 KiB clusters, full (non-quick) format
format D: /FS:FAT32 /A:32K /V:PORTAPACK

rem or exFAT
format D: /FS:exFAT /A:32K /V:PORTAPACK
```

A **full** (non-quick) format is worth the wait once: it surfaces a dying card now
rather than as a read error 20 minutes into a transmit.

On cluster size: nothing in this repo prescribes one. What is defensible from the code
is that replay reads the file in 16 KiB blocks (`ReplayThread` config, `read_size{16384}`
at `firmware/application/external/gpssim/gps_sim_app.hpp:66` and `mgnss_sim_app.hpp:73`), FatFs walks the cluster chain to serve them, and larger clusters
mean fewer FAT lookups per block and a smaller FAT. 32 KiB is Windows' own default for
volumes this size. **The specific figure is not repo-verified** — see
[§11](#11-not-verified-from-this-repo).

### 3.3 Mounting it in WSL

WSL2 auto-mounts fixed drives only; removable media usually needs it by hand:

```sh
sudo mkdir -p /mnt/d
sudo mount -t drvfs D: /mnt/d
```

**General WSL knowledge, not from this repo.** Keep the drive letter straight — copying
1.5 GB to the wrong volume is the single most common way to lose an afternoon here.

---

## 4. Flash

Two routes. Prefer (a).

> **Where the files live.** You build in WSL (`~/mayhem-custom`). The Windows side only
> ever needs the `flashing\` directory of the *Windows* checkout. Stage build artefacts
> into `flashing\firmware\` from WSL first (commands below), then run the Windows tools
> with paths relative to `flashing\`. Do **not** point the Windows commands at
> `C:\...\mayhem-custom\build\` — you never built there.

### 4a. On-device Flash Utility — **preferred**

Preferred because the `.ppfw.tar` carries firmware *and* external apps, and the
unpacker writes the apps before the flash reboots the device — so the two halves can
never drift ([§5](#5-the-firmware--sd-card-coupling)).

The Flash Utility is a **built-in** app, not a `.ppma`
(`firmware/application/ui_navigation.cpp:120`), so it works no matter what state
`/APPS/` is in. That is what breaks the chicken-and-egg.

1. Put the package on the card as `/FIRMWARE/portapack-mayhem_OCI.ppfw.tar`:

   ```sh
   mkdir -p /mnt/d/FIRMWARE
   cp build/firmware/portapack-mayhem_OCI.ppfw.tar /mnt/d/FIRMWARE/
   sync
   ```

   Then **Eject the drive in Windows** before pulling the card.

2. On the device: main menu → **Utilities** → **Flash Utility**
   (`ui_navigation.cpp:94,120`).

   It lists only `/FIRMWARE`, and only two patterns: `*.bin` in red and `*.tar` in cyan
   (`ui_flash_utility.cpp:109-110`). It also creates `/APPS` and `/FIRMWARE` on entry
   if missing (`:89-90`).

3. Select the `.tar`, confirm the "This will replace your current firmware" prompt
   (`:116-125`).

What then happens, in order:

| Step | Code |
|---|---|
| Untar, showing each filename | `ui_flash_utility.cpp:137-158` |
| Each member written root-relative → `/APPS/...`, `/FIRMWARE/...` | `untar.hpp:66,138-139` (`_FS_RPATH 0`, so a bare name is from the root) |
| Each `.ppma` checksummed as it is written; a bad one is **deleted** and the whole untar aborts — files already written stay, so the card is left half-updated | `untar.hpp:141,166-183` |
| The extracted `.bin` is remembered as the flash target | `untar.hpp:127-129` |
| Image validated: size ≤ `FLASH_ROM_SIZE` (1 MiB here) and `HACKRFFW` checksum | `ui_flash_utility.cpp:37-79`, `ui_flash_utility.hpp:35-37` |
| On failure: `BAD FIRMWARE FILE OR W/R ERR` for 5 s, no flash | `ui_flash_utility.cpp:160-166` |
| Otherwise the M4 flashes it and the device reboots | `:167-179` |

Next time you open the Flash Utility, the extracted `portapack-mayhem_dev.bin` will
also be listed in red — the untar leaves it in `/FIRMWARE` (`:109`).

**Caveat:** step 3 runs on your *current* firmware. If the firmware now on the device
is old or stripped enough that its Flash Utility lists only `*.bin`, the `.tar` will not
appear. Use route (b) for the first flash, then route (a) from then on.

### 4b. USB via `hackrf_spiflash`

> **This route flashes firmware only.** `portapack-mayhem-firmware.bin` contains no
> external apps (`firmware/CMakeLists.txt:83-89` — it is `application.bin` +
> `baseband.img`). The `/APPS/*.ppma` on the card stay whatever they were, i.e. **stale**,
> and after reboot the new firmware's `VERSION_MD5` will not match them, so your
> external apps go grey or vanish ([§5](#5-the-firmware--sd-card-coupling)). You must
> copy them separately — see 4c.

The device is on **Windows**, so run the flasher on Windows. (Reaching a USB device
from WSL2 needs `usbipd-win`; not repo-verified, see [§11](#11-not-verified-from-this-repo).)

**Drivers**, once: run `flashing\driver\dpinst.exe` as Administrator. It installs
`hackrf_one.inf` and `lpc_dfu.inf` — the latter is what makes Windows recognise the DFU
device `1fc9:000c` in [§9](#9-recovery-dfu-unbrick).

**Stage the image**, from WSL:

```sh
WINFLASH=/mnt/c/Users/seant/mayhem-custom/flashing
mkdir -p "$WINFLASH/firmware"
cp build/firmware/portapack-mayhem-firmware.bin "$WINFLASH/firmware/firmware_hackrf.bin"
```

`firmware_hackrf.bin` is the name `:set_firmware` expects for device 1
(`mayhem_flasher.bat:208-209`), so one copy serves both the direct command and the menu.

**Direct command** (what the batch file ultimately runs, `mayhem_flasher.bat:241`):

```bat
cd C:\Users\seant\mayhem-custom\flashing
utils\hackrf_spiflash.exe -R -w firmware\firmware_hackrf.bin
```

If the PortaPack is attached, put it in **HackRF mode first**: main menu → **HackRF**
(`ui_navigation.cpp:1033`). The batch file says the same at `mayhem_flasher.bat:60-66`,
and falls back to writing `hackrf\r\n` to the device's COM port if the first flash fails
(`:133-182`, `:242-251`).

**Via the batch menu** — note the filenames it insists on. `:set_firmware`
(`mayhem_flasher.bat:208-211`):

| Menu choice | File it opens |
|---|---|
| 1. HackRF One / PortaPack | `firmware\firmware_hackrf.bin` |
| 2. PortaRF | `firmware\firmware_portarf.bin` |
| 3. HackRF Pro / PortaPack | `firmware\firmware_hpro.bin` |

**`flashing\firmware\` does not exist in this repo** — the batch file ships inside
release archives that provide it. Without it you get
`ERROR: The firmware file ... was not found` and a pointer to release.hackrf.app
(`:227-235`). The staging step above creates it.

With that in place, run `flashing\mayhem_flasher.bat` → `1` → `1`. `:do_flash` invokes
`utils/hackrf_spiflash.exe -R -w "%FIRMWARE%"` (`:239-241`).

### 4c. After route (b): copy the external apps

```sh
mkdir -p /mnt/d/APPS
cp build/firmware/application/*.ppma    /mnt/d/APPS/
cp build/firmware/standalone/*/*.ppmp   /mnt/d/APPS/
sync
```

Same file set the package carries (`firmware/CMakeLists.txt:130-131`). Eject in Windows
before removing the card. There is also an automated path —
`firmware/tools/copy_external_apps.sh`, reachable as the `program-external-apps` CMake
target (`firmware/CMakeLists.txt:115-120`) — but it drives the device's **SD over USB**
app and looks for `/dev/disk/by-id/*Portapack*part1` on a Linux host
(`copy_external_apps.sh:38-50`), which will not resolve under WSL. Copy by hand.

---

## 5. The firmware ↔ SD card coupling

### 5.1 What is coupled, exactly

A `.ppma` is not a self-contained program. It is a chunk of the firmware's own
`application.elf`, carved out per app and **rewritten for the address it will run at**
(`export_external_apps.py:75-94,146`). Its pointers into the main firmware are whatever
that one link produced. Run it against a different build and those addresses are wrong.

The firmware therefore gates every `.ppma` on the card, in
`firmware/application/ui_external_items_menu_loader.cpp`:

| Gate | Code | Result if it fails |
|---|---|---|
| `header_version != CURRENT_HEADER_VERSION` (`0x00000003`, `firmware/common/external_app.hpp:30`) | `:196-197` | `continue` — **the app does not appear at all**, no message |
| `app_version != VERSION_MD5` | `:199` | Tile appears **grey**, icon `bitmap_sd_card_error`, and selecting it shows **"The .ppma file in your APPS folder is outdated. Please update your SD Card content."** (`:218-228`) |
| Run-time checksum ≠ `EXT_APP_EXPECTED_CHECKSUM` (0) after loading | `:350-351` | **"The .ppma file in your APPS folder can't be read. Please update your SD Card content."** (`:211-214`) |

So **yes — the loader does validate both `app_version`/`VERSION_MD5` and
`header_version`**, and it reads them out of the first `sizeof(application_information_t)`
bytes of the file (`:187-191`).

`VERSION_MD5` is the first 8 hex digits of the md5 of `VERSION`
(`CMakeLists.txt:88-92`), and `VERSION` is `git log -n 1 --format=%h` unless
`VERSION_STRING` is set (`:66-80`). It is compiled into both halves from the same value
(`firmware/application/CMakeLists.txt:433`, and each app's
`/*.app_version = */ VERSION_MD5` — e.g.
`firmware/application/external/mgnss_sim/main.cpp:39`).

**Consequence: every commit changes `VERSION_MD5` and invalidates every `.ppma` already
on the card.** This is not a release-to-release thing. Rebuild on a new commit, flash,
and your apps go grey until you copy the new `.ppma` files too.

### 5.2 What is *not* coupled

* **`.ppmp` standalone apps** (`pacman`, `digitalrain`) are not MD5-bound. They are
  gated on `header_version > CURRENT_STANDALONE_APPLICATION_API_VERSION` only — a
  `<=` check, so older ones still load (`:250`) — and they relocate themselves at load
  time by scanning for addresses in `0xADB10000..+64K` (`:381-387`).
* **No separate app-vs-baseband mismatch is possible.** The baseband image is appended
  *inside* the `.ppma` and located by `m4_app_offset` in its header
  (`export_external_apps.py:178-190`; loader `:292-333`). Multi-GNSS Sim's baseband is
  tagged `PMGS` (`mgnss_sim/main.cpp:80`, `firmware/baseband/CMakeLists.txt:627`), GPS
  Sim's is `PGPS` (`gpssim/main.cpp:80`) — but you never handle those as files.
* **Everything else on the card is plain data**: the `sdcard/` template, `/GPS/*.C8`,
  `/FREQMAN`, settings. No version check, no ABI.

### 5.3 Does this explain the "File read error." they saw? **No.**

This matters, so be precise about it. A stale or foreign `.ppma` produces one of three
outcomes listed in 5.1: *silently absent*, *"outdated"*, or *"can't be read"*. **None of
them is "File read error."**

`"File read error."` is raised only from inside a *running* GNSS app, about the **data
file**, never about the `.ppma`:

| Site | Trigger |
|---|---|
| `gps_sim_app.cpp:93` ← `:49-55` | `File::open` on the selected `.C8` failed, right after you pick it in the browser |
| `gps_sim_app.cpp:93` ← `:110-114` | `FileReader::open` failed when you pressed play |
| `gps_sim_app.cpp:93` ← `:150-156` | `ReplayThread` returned `READ_ERROR` mid-transmit (`replay_thread.cpp:97,114`) |
| `mgnss_sim_app.cpp:258` ← `:128-132`, `:278-281`, `:323-328` | same three, for Multi-GNSS Sim |

All of them bottom out in `f_open`/`f_read` returning a non-`FR_OK` `FRESULT`, passed
straight through as `File::Error` (`firmware/application/file.cpp:36-52,82-90`).

Two further points:

* The app that showed that modal was the **old firmware's own** GPS Sim, because only
  the `.C8` was copied, not `/APPS`. The traces above are *this* repo's code. The old
  firmware's code is not in this tree and cannot be checked — flagged.
* So a clean start fixes the symptom not by resolving an ABI mismatch, but because it
  replaces the card contents: fresh format, freshly written file, verified copy
  ([§7](#7-generate-and-install-the-hkia-gnss-files)). The ABI coupling is the reason to
  flash firmware and apps *together* — a different, equally real problem you would hit
  next.

### 5.4 How starting clean avoids the coupling problem

One build, one package, one write:

```
./mbt build --device hackrf-one
        │
        ├─ portapack-mayhem_OCI.ppfw.tar ──► /FIRMWARE/ ──► Flash Utility
        │                                                      │
        │                              unpacks APPS/*.ppma ────┤ same VERSION_MD5
        └─ (nothing else needed)       then flashes the .bin ──┘
```

Both halves carry the same `VERSION_MD5` by construction, and the device writes the
apps before it reboots into the firmware that checks them.

---

## 6. Populate the card

All names are uppercase and come from `firmware/application/file_path.cpp:25-61`.

### 6.1 Copy the template

```sh
cp -rv sdcard/. /mnt/d/
sync
```

The template is `sdcard/` in this repo: 25 directories, no files at the root. It does
**not** contain `APPS/` or `FIRMWARE/` — those come from the build or are created by the
device.

### 6.2 Directory map

"Device creates" = an explicit `ensure_directory(<named dir>)` call found in
`firmware/application/`.

| Directory | In `file_path.cpp` | In `sdcard/` template | Device creates | Note |
|---|---|---|---|---|
| `APPS` | `:27` | **no** | yes — `ui_flash_utility.cpp:89` | **External apps live here.** From the build or the `.ppfw.tar`. |
| `FIRMWARE` | `:35` | **no** | yes — `ui_flash_utility.cpp:90` | Where the Flash Utility looks |
| `GPS` | `:37` | yes (`HKIA.TXT`, `README.md`) | yes — `gps_sim_app.cpp:189`, `mgnss_sim_app.cpp:369` | **GNSS sim files go here** |
| `ADSB` | `:25` | yes | — | |
| `AIS` | `:26` | yes | — | |
| `APRS` | `:28` | no | — | |
| `AUDIO` | `:29` | no | — | |
| `BLERX` / `BLETX` | `:30-31` | `BLETX` only | — | |
| `CAPTURES` | `:32` (also `repeat_rec_path`, `:42`) | no | — | |
| `CVSFILES` | `:33` | yes | — | |
| `DEBUG` | `:34` | no | yes | |
| `EPIRB` | `:61` | yes | — | |
| `FREQMAN` | `:36` | yes | — | |
| `HOPPER` | `:52` | yes | yes | |
| `KEELOQKEYS` / `KEELOQREMOTES` | `:58-59` | yes | yes | |
| `LOGS` | `:38` | no | — | |
| `LOOKINGGLASS` | `:39` | yes | — | |
| `MACADDRESS` | `:56` | yes | — | |
| `OOKFILES` | `:51` | yes | yes | |
| `PLAYLIST` | `:40` | yes | yes | |
| `REMOTES` | `:41` | yes | yes | |
| `SAMPLES` | `:43` | yes | — | |
| `SCREENSHOTS` | `:44` | no | yes | |
| `SECPLUS` | `:60` | no | yes | |
| `SETTINGS` | `:45` | no | yes | per-app settings persist here |
| `SPECTRUM` | `:46` | yes | yes | |
| `SPLASH` | `:47` | yes | — | `splash.bmp` at the root is also read (`:57`) |
| `SSTV` | `:48` | yes | yes (`SSTV/RX`) | |
| `SUBGHZ` | `:53` | yes | — | |
| `TPMS` | `:54` | no | yes | |
| `WATERFALLS` | `:55` | yes | yes | |
| `WAV` | `:49` | yes | — | |
| `WHIPCALC` | `:50` | yes | — | |
| `BLESPAM` | **no** | yes | — | path hardcoded: `external/blespam/ui_blespam.cpp:338` |
| `OSM` | **no** | yes | — | path hardcoded: `ui/ui_geomap.cpp:582`, `ui_geomap.hpp:128` |
| `BMP` | **no** | no | yes | `ensure_directory("/BMP")` |

The device creating a directory on demand is not a reason to skip the template: the
template ships *content* (frequency lists, waterfall palettes, splash images) that
nothing recreates.

### 6.3 Add the apps

Route (a) in [§4](#4-flash) writes `/APPS` for you. If you used route (b), do 4c now.

```sh
ls /mnt/d/APPS/*.ppma | wc -l   # ~100 for a full build
ls /mnt/d/APPS/mgnss_sim.ppma /mnt/d/APPS/gpssim.ppma
```

---

## 7. Generate and install the HKIA GNSS files

Both generators clone and build upstream simulators on first use into
`~/.cache/portapack-gps-sim` (override with `GPS_EPH_CACHE`). First run takes a while;
later runs reuse it.

> **Pass `-o` explicitly.** With no `-o`, both default to the repo's own
> `sdcard/GPS/HKIA.C8` (`make_gps_sim_file.sh:68`,
> `tools/gnss_sim/make_gnss_sim_file.py:419`) — and `sdcard/GPS/` is **not** gitignored,
> so you get a 1.5 GB untracked file in your checkout.

> **Legal.** GPS L1 is a protected aeronautical band. Transmit into a shielded enclosure
> or over attenuated coax. See the warning in `sdcard/GPS/README.md`.

### 7.1 GPS only — the shell script

```sh
cd ~/mayhem-custom
./tools/make_gps_sim_file.sh -d 300 -o ~/gnss/HKIA.C8
```

* Defaults are the HKIA aerodrome reference point: `22.308889, 113.914444, 11.0 m`
  (`make_gps_sim_file.sh:27-29`). Override with `--lat/--lon/--alt`.
* `-d` seconds (default 300), `-s` sample rate (default 2,600,000), `-t`
  `YYYY/MM/DD,hh:mm:ss` start, `--fetch` for real orbits, `-e` your own RINEX
  (`:52-67`).
* Output **must** end in `.C8` or it refuses (`:74-78`).
* It writes `${OUT%.C8}.TXT` alongside with `center_frequency`, `sample_rate`,
  `latitude`, `longitude`, CRLF (`:222-232`).

### 7.2 Multi-constellation — the Python script

```sh
cd ~/mayhem-custom/tools/gnss_sim
./make_gnss_sim_file.py --gnss gps,gal -d 300 -o ~/gnss/HKIA.C8
```

Run it from `tools/gnss_sim` — it does `import pack` from its own directory
(`make_gnss_sim_file.py:28`).

* `--gnss` comma list of `gps,gal`, default `gps` (`:390`).
* `--format c8|c2|c1`, default `c8` (`:391`). The extension must match the format or it
  errors (`:420-421`), and each format gets a distinct basename so two data files never
  share one sidecar: `c8`→`""`, `c2`→`_2B`, `c1`→`_1B` (`tools/gnss_sim/pack.py:17-18`).
* `--cn0` noise: defaults to 0 for `c8`, 45 dB-Hz for `c2`/`c1`; `c1` **requires**
  noise (`:412-415`).
* Sidecar gets the full key set — `format`, `gnss`, `start_time`, `altitude`, `cn0`
  (`:360-374`).

> **Only `.C8` is selectable on the device today.** Both apps' file pickers filter on
> `".C8"` (`gps_sim_app.cpp:188`, `mgnss_sim_app.cpp:368`, and the comment at
> `mgnss_sim_app.cpp:362-367` explains why). Generating `.C2`/`.C1` is possible; playing
> them is not yet. See `tools/gnss_sim/FORMAT.md`.

### 7.3 The pair must travel together

The firmware derives the sidecar path by swapping the extension for `.TXT`
(`firmware/application/metadata_file.cpp:38-40`). So:

* **Same basename, same directory, uppercase `.TXT`.** `HKIA.C8` + `HKIA.TXT` in `/GPS/`.
* `read_metadata_file` rejects the sidecar unless **both** `center_frequency` and
  `sample_rate` parse non-zero (`metadata_file.cpp:110-111`) — and rejection is
  **silent**: the app keeps its built-in defaults. A missing `.TXT` is not an error.
* Two data files with one basename would share one sidecar — hence the `_2B`/`_1B`
  suffixes.

### 7.4 Copy to the card, and verify

This is the step that bites. A 1.5 GB write through drvfs is slow, and Windows does not
flush the volume until you eject it.

```sh
SRC=~/gnss/HKIA.C8
DST=/mnt/d/GPS

mkdir -p "$DST"
cp -v "$SRC" "${SRC%.C8}.TXT" "$DST"/
sync

# 1. sizes match?
ls -l "$SRC" "$DST/HKIA.C8"
# 300 s x 2,600,000 Sa/s x 2 bytes = 1,560,000,000

# 2. byte-identical? (slow, but this is the check that matters)
cmp "$SRC" "$DST/HKIA.C8" && echo COPY-OK

# 3. sidecar present and sane?
cat -A "$DST/HKIA.TXT" | head
```

Then **Eject the drive in Windows** (not just `umount`) before pulling the card. `sync`
in WSL does not guarantee the Windows volume cache is written —
[§11](#11-not-verified-from-this-repo).

---

## 8. First-run verification

Work down the list. Each row's failure mode tells you which of the previous sections to
revisit.

| # | Check | Where | Pass | Fail means |
|---|---|---|---|---|
| 1 | Firmware version | Top-left info bar, 9 characters (`ui_navigation.hpp:374-376`, `VERSION_STRING`) | Matches `git rev-parse --short HEAD` of what you built | Old firmware still flashed — redo [§4](#4-flash) |
| 2 | Flash integrity | Same bar shows **`FLASH ERR`** if the image checksum fails (`ui_navigation.cpp:679-693`) | No `FLASH ERR` | Bad or truncated flash — reflash, then [§9](#9-recovery-dfu-unbrick) |
| 3 | Card mounted | Open **Utilities → File Manager** | Directories from [§6](#6-populate-the-card) listed | `EMPTY SD CARD!` (`apps/ui_fileman.cpp:320`) or nothing → not mounted. 512-byte sectors? single partition? ([§3.1](#31-filesystem)) |
| 4 | App present | Main menu → **Transmit** (`ui_navigation.cpp:88`); externals are appended by `add_external_items(..., app_location_t::TX, ...)` (`:929`) | Green tile **`Multi-GNSS Sim`** (name and colour from `mgnss_sim/main.cpp:41,76-77`) | **Absent** → `/APPS/mgnss_sim.ppma` missing, or `header_version` mismatch (`loader:196-197`). **Grey with an SD-error icon** → `VERSION_MD5` mismatch (`loader:199,218-228`) — firmware and apps are from different builds, redo [§4c](#4c-after-route-b-copy-the-external-apps) |
| 5 | App launches | Select the tile | The view opens | Modal "…can't be read" → corrupt `.ppma`, checksum failed at load (`loader:350-351`). Re-copy. |
| 6 | File loads | **Open file** → the picker opens in `/GPS` (`mgnss_sim_app.cpp:368-370`) → pick `HKIA.C8` | Header fields populate | `File read error.` → [§10](#10-troubleshooting) |
| 7 | Sidecar was read | Header lines `GNSS  GPS   fmt C8` and `POS   22.3089 113.9144` (`mgnss_sim_app.cpp:184-191`; `latitude`/`longitude` *are* parsed — `metadata_file.cpp:34-35,100-103`, `metadata_file.hpp:32-33` — and shown to 4 decimals) | Both populated | `GNSS -` / `POS -` → the `.TXT` was not read. Silent by design (`metadata_file.cpp:110-111`); check the basename and that `center_frequency` and `sample_rate` both parse non-zero. |
| 8 | Frequency | Frequency field | **1575.420 MHz** (from `center_frequency=1575420000`) | Default would also be 1,575,420,000 (`gps_sim_app.hpp:57`, `mgnss_sim_app.hpp:66`) — so this cannot distinguish "sidecar read" from "sidecar ignored". Use row 7 for that. |
| 9 | Sample rate | `text_sample_rate` (`mgnss_sim_app.cpp:171`) | **`2.6MHz`** — `unit_auto_scale(2600000, 3, 1)` scales to 2.6 and appends `'M'` from `unit_prefix` (`string_format.cpp:410-436`, `string_format.hpp:42`), then the app appends `Hz` | Same caveat as row 8 |
| 10 | Duration | `text_duration` (`:176-177`) | **`5m0s`** for the 300 s default — `to_string_time_ms(300000)` renders `<min>m<sec>s` (`string_format.cpp:236-251`) | Shorter → the file is truncated. Re-check [§7.4](#74-copy-to-the-card-and-verify) |
| 11 | Format/extension agree | — | No modal | `format= in the .TXT does not match the extension.` (`:154-157`) → hand-edited sidecar, or a `.C8` carrying `format=c1` |
| 12 | Transmit | Press play | `DROPS 0` stays at 0 (`:241-249`); `TIME` counts up from `start_time` (`:211-239`) | `DROPS` climbing → the card is not keeping up at ~5.2 MB/s. Short reads are **underruns, not errors** — zero-filled and counted (`FORMAT.md` "Short reads") |

On GPS Sim rather than Multi-GNSS Sim, the tile is **`GPSSim`** (`gpssim/main.cpp:41`),
and it has no GNSS/POS/DROPS lines — so there is nothing on screen that proves the
sidecar was read, because its built-in defaults already equal the HKIA values
(`gps_sim_app.hpp:56-60`). That is a reason to verify on Multi-GNSS Sim.

---

## 9. Recovery: DFU unbrick

Use this when `hackrf_spiflash` can no longer talk to the device, or it will not boot.
It loads HackRF firmware into **RAM** over DFU — nothing is written to flash — giving
you a device that `hackrf_spiflash` can reach again.

Button sequence, from `mayhem_flasher.bat:85-87` (repeated at `:106-108`):

1. Hold **DFU** and **RESET** together.
2. Release **RESET** first (the button closest to the edge).
3. Then release **DFU**.

Stage both payloads from WSL first — `hackrf_usb.dfu` is **produced by your build**:

```sh
WINFLASH=/mnt/c/Users/seant/mayhem-custom/flashing
mkdir -p "$WINFLASH/firmware"
cp build/hackrf/firmware/hackrf_usb/hackrf_usb.dfu "$WINFLASH/firmware/"
cp build/firmware/portapack-mayhem-firmware.bin "$WINFLASH/firmware/firmware_hackrf.bin"
```

Then, on Windows (`:266-267` for the DFU call, `:216-220` for the filename it expects):

```bat
cd C:\Users\seant\mayhem-custom\flashing
utils\dfu-util-static.exe --device 1fc9:000c --download firmware\hackrf_usb.dfu
rem wait ~5 s for the device to re-enumerate, then:
utils\hackrf_spiflash.exe -R -w firmware\firmware_hackrf.bin
```

(`CMakeLists.txt:101` names it `hackrf_usb.dfu`; `:set_dfu_file` looks for
`firmware\hackrf_usb.dfu` at `mayhem_flasher.bat:216-220`.)

If Windows does not see device `1fc9:000c`, install `flashing\driver\lpc_dfu.inf` —
`dpinst.exe` as Administrator does both drivers.

The batch file's menu does both steps in one go: `flashing\mayhem_flasher.bat` → device
`1` → action **`2`** ("Flash DFU then Mayhem", `:99-130`). Action `3` is DFU alone
(`:77-96`).

**Last resort:** action `4` restores factory HackRF firmware from
`firmware\hackrf_usb.bin` and **removes PortaPack support** (`mayhem_flasher.bat:185-201`).
The build produces that file too — `CMakeLists.txt:103` names it, and it is at
`build/hackrf/firmware/hackrf_usb/hackrf_usb.bin` (confirmed present, 57,820 bytes).
Stage it the same way:

```sh
cp build/hackrf/firmware/hackrf_usb/hackrf_usb.bin \
   /mnt/c/Users/seant/mayhem-custom/flashing/firmware/
```

---

## 10. Troubleshooting

### 10.1 `File read error.`

Raised from `gps_sim_app.cpp:93` / `mgnss_sim_app.cpp:258`, by one of three callers
([§5.3](#53-does-this-explain-the-file-read-error-they-saw-no)). **When** the modal
appears narrows it a lot:

* **Immediately on picking the file** → `File::open` failed → `f_open` returned a
  non-`FR_OK` (`file.cpp:36-52`).
* **On pressing play** → `FileReader::open` failed → same `f_open`
  (`io_file.hpp:41-43`; note `File::open` defaults to read-only, `file.hpp:334`, so a
  write-protect switch is not it).
* **Partway into a transmit** → `f_read` failed → `READ_ERROR`
  (`replay_thread.cpp:97,114`).

Candidate causes, ordered for someone who just copied a 1.5 GB file to a card from WSL:

| # | Cause | Why it is likely here | Symptom shape | Fix |
|---|---|---|---|---|
| 1 | **Incomplete or unflushed copy.** The directory entry exists (so the picker lists the file) but the cluster chain is inconsistent, because the card was removed before Windows flushed, or `cp` was interrupted. | 1.5 GB through drvfs takes minutes; `sync` in WSL does not flush the Windows volume cache; people pull the card. | Either on pick (`FR_INT_ERR`/`FR_DISK_ERR` from `f_open`) **or** mid-replay | `chkdsk D: /F` in Windows, reformat ([§3](#3-format-the-sd-card)), re-copy, then **`cmp`** and **Eject** ([§7.4](#74-copy-to-the-card-and-verify)) |
| 2 | **Card out of space**, leaving a short file | 1.5 GB onto a card already holding the old firmware's data | Loads, but `text_duration` is short, then `END_OF_FILE` early. Not strictly this modal — but the same root cause as #1 and worth ruling out together | `df -h /mnt/d`; reformat |
| 3 | **Marginal card / genuine I/O error.** `f_read` returns `FR_DISK_ERR`. | Sustained 5.2 MB/s reads for 5 minutes will find a dying card. Note this is a *hard* error: a card merely too slow produces underruns (`DROPS`), not this modal (`FORMAT.md` "Short reads") | Usually mid-replay, at a repeatable offset | Full (non-quick) format; if it recurs, replace the card |
| 4 | **Wrong file or wrong place.** `FR_NO_FILE`. | The picker opens in `/GPS` (`mgnss_sim_app.cpp:368-370`) but you can navigate away; extension matching is case-insensitive (`ui_fileman.cpp:180`, `path_iequal`), so `hkia.c8` *does* list | On pick, always | Confirm the path in the File Manager |
| 5 | **`FR_NOT_ENOUGH_CORE` on open.** With `_USE_LFN 3` every path operation heap-allocates an LFN work buffer — `(255+1)*2 + MAXDIRB(255)` ≈ 1.1 KiB — via `chHeapAlloc` (`chibios-portapack/os/various/fatfs_bindings/fatfs_syscall.c:72-75`, `ext/fatfs/src/ff.c:578`), and returns `FR_NOT_ENOUGH_CORE` if it cannot. `rules.cmake:140` notes the M0 heap is down to ~15 KiB free by the time an app opens. | Low probability — the allocation is small — but it is a real path to an open failure, not slowness | On pick, intermittently | Reboot the device; fewer apps deep in the nav stack |
| 6 | **The old firmware's own code path.** The modal they saw came from the firmware that was flashed at the time, which is not in this tree. | Unknowable | — | Resolved by [§4](#4-flash): flash this fork, then re-test |

**Not causes** — stop chasing these:

* A missing or malformed `.TXT`. `read_metadata_file` fails **silently** and the app
  keeps its defaults (`metadata_file.cpp:110-111`).
* A stale or foreign `.ppma`. That gives "outdated" / "can't be read" / an absent tile
  ([§5.1](#51-what-is-coupled-exactly)) — different strings.
* A "locked" file. `_FS_LOCK 0` (`ffconf.h:210`), so FatFs tracks no open-file state.
* exFAT being unsupported. It is supported (`ffconf.h:192`); an unmountable card gives
  `EMPTY SD CARD!` / no mount (`sd_card.cpp:38-61`), not this modal.

### 10.2 Everything else

| Symptom | Likely cause | Fix |
|---|---|---|
| `FLASH ERR` in the info bar | Firmware checksum mismatch (`ui_navigation.cpp:679-693`) | Reflash; if it persists, [§9](#9-recovery-dfu-unbrick) |
| `BAD FIRMWARE FILE OR W/R ERR` in the Flash Utility | `valid_firmware_file` failed: file > `FLASH_ROM_SIZE` (1 MiB here), or the `HACKRFFW` checksum is wrong (`ui_flash_utility.cpp:37-79`). A PRALINE build is 4 MiB and will trip the size check. | Use the `hackrf-one` build |
| `.tar` not listed in the Flash Utility | It only lists `/FIRMWARE` with `*.bin` and `*.tar` (`:109-110`); or the *current* firmware's utility predates `.tar` support | Check the path; else flash route (b) once |
| Untar appears to finish but an app is missing | A `.ppma` failed its checksum during untar and was deleted, aborting the rest (`untar.hpp:166-183`) | Re-copy the `.ppfw.tar` and verify its size; retry |
| Tile absent from **Transmit** | `.ppma` not on card, or `header_version != 3` (`loader:196-197`, `external_app.hpp:30`) | Copy `/APPS` from the matching build |
| Tile grey, "outdated" modal | `VERSION_MD5` mismatch (`loader:199,218-228`) | [§4c](#4c-after-route-b-copy-the-external-apps) |
| `format= in the .TXT does not match the extension.` | Sidecar `format=` disagrees with the extension (`mgnss_sim_app.cpp:154-157`) | Regenerate; do not hand-edit |
| `Not a GNSS sim file.` | Extension is not `.C8`/`.C2`/`.C1` (`mgnss_sim_app.cpp:150-153`) | Rename / regenerate |
| `DROPS` climbing during transmit | Card not sustaining ~5 MB/s. Zero-filled and counted, not an error (`FORMAT.md`) | Faster card; a larger cluster size may help ([§3.2](#32-doing-it), unverified) |
| Receiver re-acquires at every loop | A loop steps GPS time backwards — inherent (`sdcard/GPS/README.md`) | Generate a longer file instead of looping |
| `hackrf_spiflash` "Device not found" | PortaPack not in HackRF mode, or no driver | Main menu → **HackRF** (`ui_navigation.cpp:1033`); run `flashing\driver\dpinst.exe` |
| Build dies with `env: 'bash\r': No such file or directory` | CRLF checkout | [§1.2](#12-line-endings) |
| `application <x> does not fit in the 40960 byte m4_code region` | App code + baseband exceed the LPC4320 budget (`export_external_apps.py:106-115`) | Expected on this board; see [§2.3](#23-the-40-kib-budget-why-this-board-is-tighter-than-a-hackrf-pro) |

---

## 11. Not verified from this repo

Stated above where relevant; collected here so nothing is taken on trust.

1. **Windows `format` and `chkdsk` command lines** ([§3.2](#32-doing-it),
   [§10.1](#101-file-read-error)) — general Windows knowledge. Nothing in this repo
   prescribes how to format a card.
2. **Cluster / allocation-unit size.** No repo guidance exists. The 32 KiB suggestion is
   Windows' own default, and the 16 KiB replay block size (`gps_sim_app.hpp:66`) is the
   only related fact the code supplies. The performance claim is reasoning, not
   measurement.
3. **WSL removable-drive mounting** (`mount -t drvfs D: /mnt/d`) and the fact that
   `sync` does not flush the Windows volume cache ([§3.3](#33-mounting-it-in-wsl),
   [§7.4](#74-copy-to-the-card-and-verify)) — general WSL behaviour.
4. **`usbipd-win` for reaching the HackRF from WSL2** ([§4b](#4b-usb-via-hackrf_spiflash))
   — not referenced anywhere in this tree. The Windows-side route is what the repo's own
   tooling assumes.
5. **The previously flashed firmware.** Its code is not in this tree. Specifically
   unverifiable: whether its Flash Utility lists `.tar`
   ([§4a](#4a-on-device-flash-utility--preferred)), and which of its code paths produced
   `File read error.` ([§5.3](#53-does-this-explain-the-file-read-error-they-saw-no),
   row 6 of [§10.1](#101-file-read-error)).
6. **`--cmake-arg=-DVERSION_STRING=...`** is advertised at `mbt:223`, but
   `CMakeLists.txt:66` reads `$ENV{VERSION_STRING}` and assigns `VERSION`
   unconditionally, which looks like it would discard a `-D` of that name. **Not tested**
   — the env-var form is the one documented here ([§2.2](#22-what-is-inside-the-ppfwtar)).
7. **No build was run while writing this.** Output paths, the 1,048,576-byte firmware
   size, the 102-entry tar and `FIRMWARE/portapack-mayhem_dev.bin` were read from a
   pre-existing `build/` tree in this checkout and cross-checked against
   `firmware/CMakeLists.txt`. Nothing on the device itself was exercised.
8. **The exact on-screen strings in [§8](#8-first-run-verification)** (`5m0s`, `2.6MHz`,
   `22.3089 113.9144`) were derived by reading `string_format.cpp:236-251,410-436` and
   `string_format.hpp:42` and hand-evaluating them for the 300 s HKIA defaults, not by
   looking at a device. The field *positions* and the code paths are verified; a digit of
   rounding could differ.
9. **FatFs `FRESULT` → cause mappings** in [§10.1](#101-file-read-error) are inferred
   from how `file.cpp:36-52,82-90` passes the raw `FRESULT` through as `File::Error`.
   The firmware does not display the numeric code anywhere, so you cannot read the exact
   `FRESULT` off the device to confirm which one fired.
