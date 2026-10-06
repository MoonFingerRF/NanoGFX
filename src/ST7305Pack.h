// ============================================================================
//  ST7305Pack.h — 1-bit canvas rows -> the ST7305 reflective LCD's frame-memory
//  layout (Waveshare ESP32-S3-RLCD-4.2, 300 x 400 mono). Pure C++, host-tested
//  (tests/st7305_host.cpp); the panel driver is ST7305.h.
//
//  The controller's RAM is NOT row-major. The glass is 300 columns x 400 lines in
//  its native (portrait) sense, and one RAM byte covers a 4-column x 2-line cell:
//
//      byte = RAM[(line >> 1) * 75 + (col >> 2)]
//      bit  = 7 - ((col & 3) * 2 + (line & 1))          (MSB = left, upper)
//
//  i.e. a byte is the 2-bit (upper, lower) pairs of four columns, left to right.
//  A RAM "page" (RASET unit) is one line pair = 75 bytes; a CASET unit is 3 bytes
//  (12 columns), XS..XE = 0x12..0x2A for the 25 units of this glass.
//
//  Input: MSB-first 1-bit rows (x = 0 is bit 7 of byte 0, `stride` bytes per row),
//  1 = INK — what PackMono::exportRows() and the art dither produce. The ST7305 as
//  Waveshare initialises it (INVON) shows a 0 bit as black, so ink is written as 0.
//
//  Orientations (how the logical canvas sits on the glass):
//    PORTRAIT        300 x 400, native: canvas (x, y) = glass (col x, line y)
//    LANDSCAPE       400 x 300, Waveshare's landscape (their LVGL demos):
//                    canvas (x, y) = glass (col 299 - y, line x)
//    PORTRAIT_180 / LANDSCAPE_180: the same turned half a turn.
//
//  Cost (host, -O2, per full 15 KB frame): two table lookups per output byte in
//  portrait, one 4x4 transpose of 2-bit cells per output byte pair in landscape —
//  well under a millisecond on an ESP32-S3 either way (measured on the panel by the
//  RLCD firmware's self-test).
//
//  Dirty windows: packed frames are compared page by page (line pairs) and CASET
//  unit by unit; see rectOfChange(). MIT license.
// ============================================================================
#pragma once
#include <stdint.h>
#include <string.h>

namespace st7305 {

constexpr int GLASS_COLS = 300;          // native columns (portrait width)
constexpr int GLASS_LINES = 400;         // native lines (portrait height)
constexpr int PAGE_BYTES = GLASS_COLS / 4;   // 75: one line pair
constexpr int PAGES = GLASS_LINES / 2;       // 200
constexpr int FRAME_BYTES = PAGE_BYTES * PAGES;   // 15000
constexpr int UNIT_BYTES = 3;            // one CASET unit = 12 columns
constexpr int UNITS = PAGE_BYTES / UNIT_BYTES;    // 25
constexpr uint8_t XS0 = 0x12;            // CASET of unit 0 on this glass
constexpr uint8_t YS0 = 0x00;            // RASET of page 0

enum Orientation : uint8_t { PORTRAIT = 0, LANDSCAPE = 1, PORTRAIT_180 = 2, LANDSCAPE_180 = 3 };

inline int canvasWidth(Orientation o) { return (o & 1) ? GLASS_LINES : GLASS_COLS; }
inline int canvasHeight(Orientation o) { return (o & 1) ? GLASS_COLS : GLASS_LINES; }
inline int canvasStride(Orientation o) { return (canvasWidth(o) + 7) >> 3; }

namespace detail {
// abcd -> a0b0c0d0 (the upper line's 4 bits in a cell byte's even-from-top positions)
static const uint8_t SPREAD4[16] = {0x00, 0x02, 0x08, 0x0A, 0x20, 0x22, 0x28, 0x2A,
                                    0x80, 0x82, 0x88, 0x8A, 0xA0, 0xA2, 0xA8, 0xAA};
inline uint8_t rev8(uint8_t b) {
  b = (uint8_t) ((b & 0xF0) >> 4 | (b & 0x0F) << 4);
  b = (uint8_t) ((b & 0xCC) >> 2 | (b & 0x33) << 2);
  return (uint8_t) ((b & 0xAA) >> 1 | (b & 0x55) << 1);
}
// Transpose a 4x4 matrix of 2-bit cells held as 4 bytes (row r = byte r, cell c = bits 7-2c..6-2c):
// out byte c = cell c of rows 0..3, row 0 in the top bits.
inline uint32_t transpose2x4(uint32_t w) {
  // w = r0<<24 | r1<<16 | r2<<8 | r3; cell (r, c) sits at bits 30-8r-2c (+1). A recursive block
  // transpose in two delta swaps: cells within each 2x2 block ((0,1) <-> (1,0), 6 bits apart),
  // then the off-diagonal 2x2 blocks (12 bits apart).
  uint32_t t;
  t = (w ^ (w >> 6)) & 0x00CC00CCu;  w ^= t ^ (t << 6);
  t = (w ^ (w >> 12)) & 0x0000F0F0u; w ^= t ^ (t << 12);
  return w;
}
}  // namespace detail

// Pack a whole canvas (canvasWidth x canvasHeight, 1 = ink) into a 15000-byte frame.
inline void pack(const uint8_t *rows, int stride, Orientation o, uint8_t *frame) {
  using namespace detail;
  if (o == PORTRAIT || o == PORTRAIT_180) {
    const bool flip = o == PORTRAIT_180;
    for (int p = 0; p < PAGES; p++) {
      // glass lines 2p, 2p+1 <- canvas rows (flip: 399-2p, 398-2p, each row mirrored)
      const int ya = flip ? GLASS_LINES - 1 - 2 * p : 2 * p;
      const int yb = flip ? ya - 1 : ya + 1;
      const uint8_t *ra = rows + ya * stride, *rb = rows + yb * stride;
      uint8_t *out = frame + p * PAGE_BYTES;
      for (int c = 0; c < PAGE_BYTES; c++) {
        // glass columns 4c..4c+3 <- canvas x (flip: 299-4c .. 296-4c)
        uint8_t na, nb;
        if (!flip) {
          const int xb = c >> 1;
          const int sh = (c & 1) ? 0 : 4;
          na = (uint8_t) ((ra[xb] >> sh) & 0x0F);
          nb = (uint8_t) ((rb[xb] >> sh) & 0x0F);
        } else {
          // canvas x = 299-4c-k for k = 0..3 (k = 0 leftmost on the glass); 300 = 37*8+4, so
          // gather bit by bit (the rare case; 180 is a mounting choice, still < 1 ms)
          na = nb = 0;
          for (int k = 0; k < 4; k++) {
            const int x = GLASS_COLS - 1 - 4 * c - k;
            na = (uint8_t) (na << 1 | ((ra[x >> 3] >> (7 - (x & 7))) & 1));
            nb = (uint8_t) (nb << 1 | ((rb[x >> 3] >> (7 - (x & 7))) & 1));
          }
        }
        out[c] = (uint8_t) ~(SPREAD4[na] | (SPREAD4[nb] >> 1));
      }
    }
    return;
  }
  // LANDSCAPE: glass line L = canvas x (flip: 399 - x); glass column C = 299 - y (flip: y).
  // A cell byte (page p, byte c) = columns 4c..4c+3 = canvas rows y = 299-4c-k (k = 0..3), each
  // contributing its x = 2p, 2p+1 pair. Four canvas rows' byte xb hold the pairs of pages
  // 4xb..4xb+3: a 4x4 transpose of 2-bit cells gives the four cell bytes at once.
  const bool flip = o == LANDSCAPE_180;
  const int W = GLASS_LINES;          // 400 canvas columns, 50 bytes
  for (int c = 0; c < PAGE_BYTES; c++) {
    const uint8_t *r[4];
    for (int k = 0; k < 4; k++) {
      const int y = flip ? 4 * c + k : GLASS_COLS - 1 - 4 * c - k;
      r[k] = rows + y * stride;
    }
    for (int xb = 0; xb < W / 8; xb++) {
      uint8_t b[4];
      if (!flip) {
        for (int k = 0; k < 4; k++) b[k] = r[k][xb];
      } else {
        // glass line L = 399 - x: canvas byte (49 - xb) read mirrored covers lines 8xb..8xb+7
        for (int k = 0; k < 4; k++) b[k] = rev8(r[k][W / 8 - 1 - xb]);
      }
      // two pages per nibble-pair: byte xb covers canvas x 8xb..8xb+7 = pages 4xb..4xb+3
      uint32_t w = (uint32_t) b[0] << 24 | (uint32_t) b[1] << 16 | (uint32_t) b[2] << 8 | b[3];
      w = transpose2x4(w);
      const int p0 = xb * 4;
      frame[(p0 + 0) * PAGE_BYTES + c] = (uint8_t) ~(w >> 24);
      frame[(p0 + 1) * PAGE_BYTES + c] = (uint8_t) ~(w >> 16);
      frame[(p0 + 2) * PAGE_BYTES + c] = (uint8_t) ~(w >> 8);
      frame[(p0 + 3) * PAGE_BYTES + c] = (uint8_t) ~w;
    }
  }
}

// Reference packer: one pixel at a time, straight from the layout comment (the test oracle).
inline void packReference(const uint8_t *rows, int stride, Orientation o, uint8_t *frame) {
  memset(frame, 0xFF, FRAME_BYTES);
  const int W = canvasWidth(o), H = canvasHeight(o);
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++) {
      if (!((rows[y * stride + (x >> 3)] >> (7 - (x & 7))) & 1)) continue;
      int col, line;
      switch (o) {
        case PORTRAIT: col = x; line = y; break;
        case PORTRAIT_180: col = GLASS_COLS - 1 - x; line = GLASS_LINES - 1 - y; break;
        case LANDSCAPE: col = GLASS_COLS - 1 - y; line = x; break;
        default: col = y; line = GLASS_LINES - 1 - x; break;
      }
      const int idx = (line >> 1) * PAGE_BYTES + (col >> 2);
      const int bit = 7 - ((col & 3) * 2 + (line & 1));
      frame[idx] &= (uint8_t) ~(1u << bit);
    }
}

// A window in RAM units: pages [p0, p1] (line pairs) x CASET units [u0, u1], inclusive.
struct Window {
  int p0 = 0, p1 = -1, u0 = 0, u1 = -1;
  bool empty() const { return p1 < p0 || u1 < u0; }
  int bytes() const { return empty() ? 0 : (p1 - p0 + 1) * (u1 - u0 + 1) * UNIT_BYTES; }
  static Window full() { Window w; w.p0 = 0; w.p1 = PAGES - 1; w.u0 = 0; w.u1 = UNITS - 1; return w; }
};

// The smallest window covering every byte where `a` and `b` differ (empty when equal).
inline Window rectOfChange(const uint8_t *a, const uint8_t *b) {
  Window w;
  int u0 = UNITS, u1 = -1;
  for (int p = 0; p < PAGES; p++) {
    const uint8_t *ra = a + p * PAGE_BYTES, *rb = b + p * PAGE_BYTES;
    if (!memcmp(ra, rb, PAGE_BYTES)) continue;
    if (w.p1 < w.p0) w.p0 = p;
    w.p1 = p;
    int lo = 0, hi = PAGE_BYTES - 1;
    while (ra[lo] == rb[lo]) lo++;
    while (ra[hi] == rb[hi]) hi--;
    if (lo / UNIT_BYTES < u0) u0 = lo / UNIT_BYTES;
    if (hi / UNIT_BYTES > u1) u1 = hi / UNIT_BYTES;
  }
  if (w.p1 >= w.p0) { w.u0 = u0; w.u1 = u1; }
  return w;
}

// Copy a window's bytes out of a full frame into `out` (contiguous, page by page): what RAMWR
// takes after CASET/RASET select that window. Returns the byte count.
inline int gather(const uint8_t *frame, const Window &w, uint8_t *out) {
  if (w.empty()) return 0;
  const int run = (w.u1 - w.u0 + 1) * UNIT_BYTES;
  for (int p = w.p0; p <= w.p1; p++) {
    memcpy(out, frame + p * PAGE_BYTES + w.u0 * UNIT_BYTES, (size_t) run);
    out += run;
  }
  return w.bytes();
}

}  // namespace st7305
