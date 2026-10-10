#!/usr/bin/env python3
"""Pulls the Galileo E1B primary codes out of gnss-sdr's Galileo_E1.h into e1b_codes.txt.

    ./extract_e1b_codes.py /path/to/Galileo_E1.h [e1b_codes.txt]

Run once; acquire_e1.py reads the .txt. The header is GPL-3.0, so none of it is
copied here -- only GALILEO_E1_B_PRIMARY_CODE's numeric values, which are the
published Galileo OS SIS ICD memory codes and not anybody's source code.

The array is char[50][1024] of C string literals, one per PRN (index 0 = PRN 1),
each 1023 hexadecimal digits split over several adjacent literals that the C
compiler concatenates. 1023 digits x 4 bits = the 4092 chips of one code period.
The output keeps that same hex form, one PRN per line, so it can be diffed
straight against the ICD tables or any other source.
"""

import re
import sys

NUM_CODES = 50
HEX_DIGITS = 1023

ARRAY = "GALILEO_E1_B_PRIMARY_CODE[GALILEO_E1_NUMBER_OF_CODES][1024]"


def parse_header(path):
    """Return the 50 hex code strings, PRN 1 first."""
    lines = open(path, encoding="utf-8", errors="replace").read().splitlines()
    try:
        start = next(i for i, l in enumerate(lines) if ARRAY in l)
    except StopIteration:
        raise SystemExit(f"{path}: no {ARRAY} in this header")

    codes = []
    cur = ""
    for line in lines[start + 1:]:
        if line.strip() == "};":
            break
        # Adjacent literals on this line belong to the entry being built.
        for m in re.finditer(r'"([0-9A-Fa-f]*)"', line):
            cur += m.group(1)
        # A comma after the last literal ends the entry. (The final entry in the
        # array has no trailing comma, so anything left over is flushed below.)
        if "," in line.rsplit('"', 1)[-1]:
            codes.append(cur)
            cur = ""
    if cur:
        codes.append(cur)

    if len(codes) != NUM_CODES:
        raise SystemExit(f"expected {NUM_CODES} codes, parsed {len(codes)}")
    for i, c in enumerate(codes):
        if len(c) != HEX_DIGITS:
            raise SystemExit(f"PRN {i + 1}: {len(c)} hex digits, expected {HEX_DIGITS}")
    if len(set(codes)) != NUM_CODES:
        raise SystemExit("codes are not all distinct -- parse is wrong")
    return [c.upper() for c in codes]


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else "e1b_codes.txt"
    codes = parse_header(src)
    with open(dst, "w") as f:
        f.write("# Galileo E1B primary codes, one PRN per line, PRN 1 first.\n")
        f.write("# 1023 hex digits = 4092 chips, 4 chips per digit, MSB first.\n")
        f.write("# Values from the Galileo OS SIS ICD, as tabulated in gnss-sdr's\n")
        f.write("# Galileo_E1.h (GALILEO_E1_B_PRIMARY_CODE); extracted by\n")
        f.write("# extract_e1b_codes.py, which see.\n")
        for c in codes:
            f.write(c + "\n")
    print(f"wrote {dst}: {len(codes)} codes x {HEX_DIGITS} hex digits")


if __name__ == "__main__":
    main()
