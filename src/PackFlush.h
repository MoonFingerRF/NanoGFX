// ============================================================================
//  PackFlush — dirty-band frame flushing for per-line encoded frames (NanoGFX).
//
//  Panels with persistent GRAM (SPI/QSPI TFTs and AMOLEDs) only need the rows
//  that CHANGED since the last flush. PackFlush keeps an "on-glass" copy of each
//  row's ENCODED PackRLE stream — comparing compressed bytes, typically 10-50x
//  smaller than pixels — memcmps each incoming row against it, coalesces dirty
//  rows into bands (small clean gaps are absorbed rather than paying a window
//  switch per fragment), and hands each band to a caller-supplied push callback
//  (set the panel's address window, decode, write).
//
//  Frame format: row y's stream lives at comp + y*slot holding lineLen[y] valid
//  bytes — exactly what PackCanvas::encodeFrame() emits.
//
//  invalidate() forces the next flush() to push the whole frame. Call it when
//  the glass content is unknown (boot) or when identical encoded bytes would no
//  longer decode to identical pixels (e.g. the decode palette changed).
//
//  The caller owns the glass buffers (height*slot bytes + height uint16s), so
//  they can live wherever is cheap — e.g. ESP32 PSRAM. All comparisons read
//  only the encoded streams; the glass copy is updated for pushed rows only,
//  keeping the bookkeeping exact even across partial flushes.
//
//  Worst case (every row changed) degrades to exactly one full-frame band.
//  Part of NanoGFX; MIT license.
// ============================================================================
#pragma once
#include <stdint.h>
#include <string.h>

class PackFlush {
public:
  // glass: height*slot bytes; glassLen: height entries. Caller-allocated.
  void begin(uint8_t *glass, uint16_t *glassLen, int height, size_t slot) {
    g = glass; gl = glassLen; h = height; sl = slot; valid = false;
  }

  // Next flush() pushes everything (boot, decode-palette change, panel reset).
  void invalidate() { valid = false; }

  // Flush one frame. pushBand(y0, y1) must write rows [y0, y1] inclusive to the
  // panel. gap = clean rows absorbed into a band (trade a little extra wire for
  // fewer window switches; 4-16 is typical). Returns the number of rows pushed.
  template <typename PushBand>
  int flush(const uint8_t *comp, const uint16_t *lineLen, int gap, PushBand pushBand) {
    if (!g || !gl) { pushBand(0, h - 1); return h; }   // no glass state: always full
    if (!valid) {                                      // unknown glass: full + adopt
      pushBand(0, h - 1);
      memcpy(g, comp, (size_t)h * sl);
      memcpy(gl, lineLen, (size_t)h * sizeof(uint16_t));
      valid = true;
      return h;
    }
    int pushed = 0, bandStart = -1, lastDirty = -1;
    for (int y = 0; y < h; y++) {
      const uint8_t *ln = comp + (size_t)y * sl;
      bool dirty = lineLen[y] != gl[y] ||
                   memcmp(ln, g + (size_t)y * sl, lineLen[y]) != 0;
      if (dirty) {
        memcpy(g + (size_t)y * sl, ln, lineLen[y]);
        gl[y] = lineLen[y];
        if (bandStart < 0) bandStart = y;
        lastDirty = y;
      } else if (bandStart >= 0 && y - lastDirty > gap) {
        pushBand(bandStart, lastDirty);
        pushed += lastDirty + 1 - bandStart;
        bandStart = -1;
      }
    }
    if (bandStart >= 0) {
      pushBand(bandStart, lastDirty);
      pushed += lastDirty + 1 - bandStart;
    }
    return pushed;
  }

private:
  uint8_t  *g = nullptr;
  uint16_t *gl = nullptr;
  int       h = 0;
  size_t    sl = 0;
  bool      valid = false;
};
