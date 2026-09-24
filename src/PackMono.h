// ============================================================================
//  PackMono.h — a packed 4-bit canvas, out to a 1-bit panel (e-paper, mono LCD/OLED).
//
//  A mono panel has two colours and a PackCanvas has fifteen. PackMono keeps the
//  canvas's palette meaningful on one bit: each palette index gets an INK LEVEL
//  (0 = paper, 16 = solid ink), and export() turns the level into a 4x4 ordered
//  (Bayer) dither. Index 0 at level 0 and index 1 at level 16 are the plain
//  paper-and-ink case; the indices in between are greys — a bar's track, a plot's
//  fill, a dimmed label — that stay flat, compressible palette colours on the canvas
//  and only become dots on the way out.
//
//  Output: MSB-first rows (x = 0 is bit 7 of byte 0), (W+7)/8 bytes per row, which is
//  what UC8179 / SSD16xx / most mono controllers take. `inkBit` picks the bit value
//  for ink (1 for most e-paper "black" RAM, 0 where the controller wants white = 0),
//  `mirrorX` flips each row for panels mounted the other way round.
//
//  Speed: a per-(row phase, x phase) LUT turns one canvas byte (2 px) into 2 output
//  bits, so a row is W/2 table lookups — an 800x480 frame is ~190k lookups.
//
//  Dirty rows: diffRows() compares two exported frames and returns the band of rows
//  that differ — the partial-refresh window of an e-paper panel, or nothing at all.
//  Comparing the 1-bit output (not the canvas) is what makes a redraw that lands on
//  the same dots cost nothing, which is the common case for a ticking clock or bar.
// ============================================================================
#pragma once
#include <stdint.h>
#include <string.h>
#include "PackCanvas.h"

class PackMono {
public:
  // Bayer 4x4, values 0..15: pixel is ink when level > threshold.
  static constexpr uint8_t BAYER4[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};

  PackMono() {
    memset(level, 0, sizeof level);
    level[1] = 16;                       // index 1 = ink, everything else paper until set
    rebuild();
  }

  // Ink level of palette index i, 0 (paper) .. 16 (solid). Rebuilds the tables.
  void setLevel(uint8_t index, uint8_t inkLevel) {
    if (index > 15) return;
    level[index] = inkLevel > 16 ? 16 : inkLevel;
    rebuild();
  }
  uint8_t getLevel(uint8_t index) const { return index > 15 ? 0 : level[index]; }

  static size_t rowBytes(int w) { return (size_t)((w + 7) >> 3); }

  // Export canvas rows [y0, y1) into `out` (row y at out + y * rowBytes(W)). The canvas
  // must be packed4 with an even width; rows are flattened first (the raw-access contract).
  void exportRows(PackCanvas &c, uint8_t *out, int y0, int y1, bool inkBit = true,
                  bool mirrorX = false) {
    const int W = c.width(), H = c.height();
    if (y0 < 0) y0 = 0;
    if (y1 > H) y1 = H;
    if (y0 >= y1 || (W & 1)) return;
    c.flatten(y0, y1);
    const size_t stride = c.lineSlotBytes(), ob = rowBytes(W);
    const uint8_t *raw = c.rawBuffer();
    const uint8_t flip = inkBit ? 0x00 : 0xFF;
    for (int y = y0; y < y1; y++) {
      const uint8_t *src = raw + (size_t)y * stride;
      uint8_t *dst = out + (size_t)y * ob;
      const uint8_t *l0 = lut[y & 3][0], *l1 = lut[y & 3][1];
      // 8 pixels = 4 canvas bytes = 1 output byte; x phase alternates 0/2 inside a byte group.
      int full = W >> 3;
      for (int i = 0; i < full; i++) {
        const uint8_t *s = src + i * 4;
        dst[i] = (uint8_t)(((l0[s[0]] << 6) | (l1[s[1]] << 4) | (l0[s[2]] << 2) | l1[s[3]]) ^ flip);
      }
      int rem = W & 7;                   // tail pixels (W not a multiple of 8): pad with paper
      if (rem) {
        uint8_t b = 0;
        for (int k = 0; k < rem; k += 2) {
          uint8_t two = ((k >> 1) & 1) ? l1[src[full * 4 + (k >> 1)]] : l0[src[full * 4 + (k >> 1)]];
          b |= (uint8_t)(two << (6 - k));
        }
        dst[full] = (uint8_t)(b ^ (flip & (uint8_t)(0xFF << (8 - rem))));
      }
      if (mirrorX) mirrorRow(dst, W);
    }
  }

  // Export only byte columns [xb0, xb1) (8 px each) of canvas rows [y0, y1); every other byte of
  // `out` is left as it was. For a screen whose unchanged parts were exported before (a ticking
  // cell redrawn into an otherwise kept canvas): bit-identical to exportRows over that rectangle,
  // at a fraction of the cost. No mirrorX (a mirrored row maps columns elsewhere: use exportRows).
  void exportRect(PackCanvas &c, uint8_t *out, int xb0, int xb1, int y0, int y1, bool inkBit = true) {
    const int W = c.width(), H = c.height(), nb = (int)rowBytes(W);
    if (y0 < 0) y0 = 0;
    if (y1 > H) y1 = H;
    if (xb0 < 0) xb0 = 0;
    if (xb1 > nb) xb1 = nb;
    if (y0 >= y1 || xb0 >= xb1 || (W & 1)) return;
    c.flatten(y0, y1);
    const size_t stride = c.lineSlotBytes();
    const uint8_t *raw = c.rawBuffer();
    const uint8_t flip = inkBit ? 0x00 : 0xFF;
    const int full = W >> 3, rem = W & 7;
    const int fb1 = xb1 < full ? xb1 : full;          // whole output bytes; the tail byte apart
    for (int y = y0; y < y1; y++) {
      const uint8_t *src = raw + (size_t)y * stride;
      uint8_t *dst = out + (size_t)y * (size_t)nb;
      const uint8_t *l0 = lut[y & 3][0], *l1 = lut[y & 3][1];
      for (int i = xb0; i < fb1; i++) {
        const uint8_t *s = src + i * 4;
        dst[i] = (uint8_t)(((l0[s[0]] << 6) | (l1[s[1]] << 4) | (l0[s[2]] << 2) | l1[s[3]]) ^ flip);
      }
      if (rem && xb1 > full) {                        // the tail byte (W not a multiple of 8)
        uint8_t b = 0;
        for (int k = 0; k < rem; k += 2) {
          uint8_t two = ((k >> 1) & 1) ? l1[src[full * 4 + (k >> 1)]] : l0[src[full * 4 + (k >> 1)]];
          b |= (uint8_t)(two << (6 - k));
        }
        dst[full] = (uint8_t)(b ^ (flip & (uint8_t)(0xFF << (8 - rem))));
      }
    }
  }

  // The band of rows where frames a and b differ: returns false when they are identical,
  // else [*y0, *y1) covers every differing row.
  static bool diffRows(const uint8_t *a, const uint8_t *b, int w, int h, int *y0, int *y1) {
    const size_t ob = rowBytes(w);
    int first = -1, last = -1;
    for (int y = 0; y < h; y++) {
      if (memcmp(a + (size_t)y * ob, b + (size_t)y * ob, ob) != 0) {
        if (first < 0) first = y;
        last = y;
      }
    }
    if (first < 0) return false;
    *y0 = first;
    *y1 = last + 1;
    return true;
  }

private:
  uint8_t level[16];
  // lut[y&3][xphase][byte] -> 2 bits (bit1 = even pixel, bit0 = odd pixel), 1 = ink.
  // xphase 0 covers x&3 in {0,1}, phase 1 covers {2,3}.
  uint8_t lut[4][2][256];

  void rebuild() {
    for (int ry = 0; ry < 4; ry++)
      for (int ph = 0; ph < 2; ph++)
        for (int v = 0; v < 256; v++) {
          uint8_t hi = (uint8_t)(v >> 4), lo = (uint8_t)(v & 15);
          uint8_t a = level[hi] > BAYER4[ry][ph * 2] ? 1 : 0;
          uint8_t b = level[lo] > BAYER4[ry][ph * 2 + 1] ? 1 : 0;
          lut[ry][ph][v] = (uint8_t)((a << 1) | b);
        }
  }

  static uint8_t rev8(uint8_t b) {
    b = (uint8_t)((b & 0xF0) >> 4 | (b & 0x0F) << 4);
    b = (uint8_t)((b & 0xCC) >> 2 | (b & 0x33) << 2);
    return (uint8_t)((b & 0xAA) >> 1 | (b & 0x55) << 1);
  }
  static void mirrorRow(uint8_t *row, int w) {
    // Byte-reverse + bit-reverse, then shift out the pad when w is not a multiple of 8.
    const int n = (int)rowBytes(w);
    for (int i = 0, j = n - 1; i <= j; i++, j--) {
      uint8_t a = rev8(row[i]), b = rev8(row[j]);
      row[i] = b;
      row[j] = a;
    }
    const int pad = n * 8 - w;
    if (pad) {
      for (int i = 0; i < n; i++)
        row[i] = (uint8_t)((row[i] << pad) | (i + 1 < n ? row[i + 1] >> (8 - pad) : 0));
    }
  }
};
