#!/usr/bin/env python3
"""packrle.py -- the PackRLE codec (src/PackRLE.h) in Python, for whatever makes pictures for a device.

A host that sends a picture to a NanoGFX device (an album cover to an e-paper panel, an icon to
a gauge) encodes it here and the device decodes it with `prle_image_decode_flat`. The format is
the C header's, bit for bit -- see its comment for the grammar:

* a row is a nibble stream, MSB first; nibbles 0x0..0xE are one pixel of that palette index;
  0xF escapes a run token [c][n2][n1][n0] of N = 12-bit pixels of colour c;
* runs are emitted only at N >= 6 (a token is 5 nibbles), so no row is ever longer than the flat
  packed row, PRLE_STRIDE(w) = ceil(w / 2) bytes;
* an IMAGE is its rows back to back, each prefixed with its byte length (uint16, little endian).

    >>> rows = [[0] * 20, [1, 0] * 10]
    >>> blob = encode_image(rows)
    >>> decode_image(blob, 20, 2) == rows
    True

Standard library only, so it can be copied into a project that has no third-party packages.
"""
from __future__ import annotations

MIN_RUN = 6
MAX_RUN = 4095


def stride(width: int) -> int:
    return (width + 1) >> 1


class _Nibbles:
    def __init__(self) -> None:
        self.out = bytearray()
        self.odd = False

    def put(self, v: int) -> None:
        v &= 0x0F
        if self.odd:
            self.out[-1] |= v
        else:
            self.out.append(v << 4)
        self.odd = not self.odd


def encode_row(row: list[int]) -> bytes:
    """One row of palette indices (0..14) as a PackRLE stream."""
    w = _Nibbles()
    i, n = 0, len(row)
    while i < n:
        c = row[i]
        if not 0 <= c <= 14:
            raise ValueError(f"palette index {c} is outside 0..14 (0xF is the escape)")
        j = i + 1
        while j < n and row[j] == c:
            j += 1
        left = j - i
        while left > 0:
            seg = min(left, MAX_RUN)
            if seg >= MIN_RUN:
                for v in (0xF, c, seg >> 8, seg >> 4, seg):
                    w.put(v)
            else:
                for _ in range(seg):
                    w.put(c)
            left -= seg
        i = j
    return bytes(w.out)


def decode_row(data: bytes, width: int) -> list[int]:
    """A PackRLE stream back to `width` palette indices. ValueError when it is malformed."""
    nibbles = [v for b in data for v in (b >> 4, b & 0x0F)]
    out: list[int] = []
    k = 0
    while len(out) < width:
        if k >= len(nibbles):
            raise ValueError("truncated row")
        v = nibbles[k]; k += 1
        if v != 0xF:
            out.append(v)
            continue
        if k + 4 > len(nibbles):
            raise ValueError("truncated run")
        c, n = nibbles[k], (nibbles[k + 1] << 8) | (nibbles[k + 2] << 4) | nibbles[k + 3]
        k += 4
        if c == 0xF or n == 0 or len(out) + n > width:
            raise ValueError("bad run")
        out.extend([c] * n)
    return out


def encode_image(rows: list[list[int]]) -> bytes:
    """Rows of palette indices as a PackRLE image (length-prefixed rows)."""
    blob = bytearray()
    for row in rows:
        stream = encode_row(row)
        blob += len(stream).to_bytes(2, "little") + stream
    return bytes(blob)


def decode_image(blob: bytes, width: int, height: int) -> list[list[int]]:
    rows, k = [], 0
    for _ in range(height):
        if k + 2 > len(blob):
            raise ValueError("truncated image")
        n = int.from_bytes(blob[k:k + 2], "little"); k += 2
        if n > stride(width) or k + n > len(blob):
            raise ValueError("bad row length")
        rows.append(decode_row(blob[k:k + n], width)); k += n
    return rows


if __name__ == "__main__":
    import doctest
    import random
    doctest.testmod()
    rnd = random.Random(1)
    for trial in range(2000):
        wdt = rnd.randint(1, 700)
        row = []
        while len(row) < wdt:
            row += [rnd.randint(0, 14)] * rnd.choice((1, 1, 2, 5, 6, 7, 40, 5000))
        row = row[:wdt]
        enc = encode_row(row)
        assert len(enc) <= stride(wdt), (wdt, len(enc))
        assert decode_row(enc, wdt) == row
    print("packrle: ok")
