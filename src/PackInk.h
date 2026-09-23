// ============================================================================
//  PackInk.h — 1-bit "ink" drawing: coverage rasterisers, textures, paint modes.
//
//  PackCanvas thinks in palette colours; a 1-bit e-paper picture with character
//  needs something else: an intensity turned into dots in a chosen WAY (ordered
//  dither, white noise, noise gated by smooth patches, value-noise clouds,
//  hatching, halftone dots), shapes that COVER what is under them or only add
//  ink, cut holes or flip pixels, masks and clips — and results that are exactly
//  reproducible on a laptop. This header is that layer:
//
//    InkBits      a 1-bit bitmap, MSB-first rows, 1 = ink (PackMono/UC8179 order)
//    InkCoverage  the set of pixels one shape covers (+ pen-stamp count)
//    fill/outline rect & ellipse, even-odd polygon, Bresenham lines stamped with a
//    round pen, quadratic/cubic curves, arcs, sine waves, spirals, the classic 5x7
//    font at any integer scale in four rotations
//    inked()      the per-pixel texture rule; paint() applies a coverage to a bitmap
//    blit()       an InkBits onto any Adafruit_GFX-style canvas (e.g. PackCanvas)
//
//  DETERMINISM IS THE CONTRACT. Integer arithmetic only, floor division and
//  positive modulo spelled out (fdiv/pmod), one 32-bit hash (lowbias32), one
//  copied sine table. A Python reference of exactly these rules exists (the
//  Nostalgia hub's memory_proxy/art/ink.py) and tests/ink_host.cpp checks pinned
//  numbers from it, so a picture previewed on a laptop is the picture on glass.
//
//  No heap: the caller owns every buffer. Header-only, C++11, no Arduino needed.
// ============================================================================
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#ifndef PROGMEM
#define PROGMEM   // not an AVR/ESP Arduino build: the font table is ordinary const data
#endif
#include "PackFont5x7.h"

namespace packink {

// ---------------------------------------------------------------- integer helpers
inline int32_t fdiv(int32_t a, int32_t b) {           // floor(a / b), b != 0
  int32_t q = a / b;
  if ((a % b != 0) && ((a < 0) != (b < 0))) q--;
  return q;
}
inline int64_t fdiv64(int64_t a, int64_t b) {
  int64_t q = a / b;
  if ((a % b != 0) && ((a < 0) != (b < 0))) q--;
  return q;
}
inline int32_t pmod(int32_t a, int32_t m) {           // a mod m in [0, m), m > 0
  int32_t r = a % m;
  return r < 0 ? r + m : r;
}
inline uint32_t isqrt64(uint64_t n) {                  // floor(sqrt(n))
  uint64_t x = 0, bit = (uint64_t)1 << 62;
  while (bit > n) bit >>= 2;
  while (bit) {
    if (n >= x + bit) { n -= x + bit; x = (x >> 1) + bit; }
    else x >>= 1;
    bit >>= 2;
  }
  return (uint32_t)x;
}
inline int32_t imin(int32_t a, int32_t b) { return a < b ? a : b; }
inline int32_t imax(int32_t a, int32_t b) { return a > b ? a : b; }
inline int32_t iabs(int32_t a) { return a < 0 ? -a : a; }

inline uint32_t mix(uint32_t x) {                      // lowbias32
  x ^= x >> 16; x *= 0x7FEB352Du; x ^= x >> 15; x *= 0x846CA68Bu; x ^= x >> 16;
  return x;
}
inline uint32_t hash4(int32_t a, int32_t b, int32_t c, int32_t d) {
  uint32_t h = mix((uint32_t)a + 0x9E3779B9u);
  h = mix(h ^ (uint32_t)b);
  h = mix(h ^ (uint32_t)c);
  return mix(h ^ (uint32_t)d);
}

// round(sin(deg) * 1024), deg = 0..359
static const int16_t SIN1024[360] = {
    0, 18, 36, 54, 71, 89, 107, 125, 143, 160, 178, 195, 213, 230, 248, 265, 282, 299, 316, 333,
    350, 367, 384, 400, 416, 433, 449, 465, 481, 496, 512, 527, 543, 558, 573, 587, 602, 616, 630, 644,
    658, 672, 685, 698, 711, 724, 737, 749, 761, 773, 784, 796, 807, 818, 828, 839, 849, 859, 868, 878,
    887, 896, 904, 912, 920, 928, 935, 943, 949, 956, 962, 968, 974, 979, 984, 989, 994, 998, 1002, 1005,
    1008, 1011, 1014, 1016, 1018, 1020, 1022, 1023, 1023, 1024, 1024, 1024, 1023, 1023, 1022, 1020, 1018, 1016, 1014, 1011,
    1008, 1005, 1002, 998, 994, 989, 984, 979, 974, 968, 962, 956, 949, 943, 935, 928, 920, 912, 904, 896,
    887, 878, 868, 859, 849, 839, 828, 818, 807, 796, 784, 773, 761, 749, 737, 724, 711, 698, 685, 672,
    658, 644, 630, 616, 602, 587, 573, 558, 543, 527, 512, 496, 481, 465, 449, 433, 416, 400, 384, 367,
    350, 333, 316, 299, 282, 265, 248, 230, 213, 195, 178, 160, 143, 125, 107, 89, 71, 54, 36, 18,
    0, -18, -36, -54, -71, -89, -107, -125, -143, -160, -178, -195, -213, -230, -248, -265, -282, -299, -316, -333,
    -350, -367, -384, -400, -416, -433, -449, -465, -481, -496, -512, -527, -543, -558, -573, -587, -602, -616, -630, -644,
    -658, -672, -685, -698, -711, -724, -737, -749, -761, -773, -784, -796, -807, -818, -828, -839, -849, -859, -868, -878,
    -887, -896, -904, -912, -920, -928, -935, -943, -949, -956, -962, -968, -974, -979, -984, -989, -994, -998, -1002, -1005,
    -1008, -1011, -1014, -1016, -1018, -1020, -1022, -1023, -1023, -1024, -1024, -1024, -1023, -1023, -1022, -1020, -1018, -1016, -1014, -1011,
    -1008, -1005, -1002, -998, -994, -989, -984, -979, -974, -968, -962, -956, -949, -943, -935, -928, -920, -912, -904, -896,
    -887, -878, -868, -859, -849, -839, -828, -818, -807, -796, -784, -773, -761, -749, -737, -724, -711, -698, -685, -672,
    -658, -644, -630, -616, -602, -587, -573, -558, -543, -527, -512, -496, -481, -465, -449, -433, -416, -400, -384, -367,
    -350, -333, -316, -299, -282, -265, -248, -230, -213, -195, -178, -160, -143, -125, -107, -89, -71, -54, -36, -18};

inline int32_t sin1024(int32_t deg) { return SIN1024[pmod(deg, 360)]; }
inline int32_t cos1024(int32_t deg) { return SIN1024[pmod(deg + 90, 360)]; }
inline void polar(int32_t cx, int32_t cy, int32_t r, int32_t deg, int32_t *x, int32_t *y) {
  *x = cx + fdiv(r * cos1024(deg) + 512, 1024);
  *y = cy + fdiv(r * sin1024(deg) + 512, 1024);
}

static const uint8_t BAYER4[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};

// ---------------------------------------------------------------- bitmaps
// MSB-first rows, (w+7)/8 bytes each, 1 = ink.
struct InkBits {
  int16_t w = 0, h = 0, stride = 0;
  uint8_t *bits = nullptr;
  InkBits() {}
  InkBits(int16_t w_, int16_t h_, uint8_t *buf) : w(w_), h(h_), stride((int16_t)((w_ + 7) >> 3)), bits(buf) {}
  static size_t bytesFor(int w, int h) { return (size_t)((w + 7) >> 3) * (size_t)h; }
  void clear() { memset(bits, 0, bytesFor(w, h)); }
  bool get(int x, int y) const {
    if (x < 0 || y < 0 || x >= w || y >= h) return false;
    return (bits[y * stride + (x >> 3)] >> (7 - (x & 7))) & 1;
  }
  void put(int x, int y, bool ink) {
    uint8_t &b = bits[y * stride + (x >> 3)];
    const uint8_t m = (uint8_t)(0x80 >> (x & 7));
    b = ink ? (uint8_t)(b | m) : (uint8_t)(b & ~m);
  }
  void flip(int x, int y) { bits[y * stride + (x >> 3)] ^= (uint8_t)(0x80 >> (x & 7)); }
  // Pixels x0..x1 inclusive on row y, clipped.
  void span(int y, int x0, int x1) {
    if (y < 0 || y >= h) return;
    if (x0 < 0) x0 = 0;
    if (x1 >= w) x1 = w - 1;
    if (x0 > x1) return;
    uint8_t *row = bits + y * stride;
    int a = x0 >> 3, b = x1 >> 3;
    uint8_t ma = (uint8_t)(0xFF >> (x0 & 7)), mb = (uint8_t)(0xFF << (7 - (x1 & 7)));
    if (a == b) { row[a] |= (uint8_t)(ma & mb); return; }
    row[a] |= ma;
    for (int i = a + 1; i < b; i++) row[i] = 0xFF;
    row[b] |= mb;
  }
  uint32_t count() const {
    uint32_t n = 0;
    const size_t total = bytesFor(w, h);
    for (size_t i = 0; i < total; i++) {
      uint8_t v = bits[i];
      while (v) { v &= (uint8_t)(v - 1); n++; }
    }
    return n;
  }
  void orWith(const InkBits &o) {
    const size_t total = bytesFor(w, h);
    for (size_t i = 0; i < total; i++) bits[i] |= o.bits[i];
  }
};

// The pixels one shape covers, and how many pen stamps it took (the render budget counts both).
struct InkCoverage : InkBits {
  uint32_t stamps = 0;
  InkCoverage() {}
  InkCoverage(int16_t w_, int16_t h_, uint8_t *buf) : InkBits(w_, h_, buf) {}
  void reset() { clear(); stamps = 0; }
};

// ---------------------------------------------------------------- shapes
inline int32_t ellipseHalf(int32_t rx, int32_t ry, int32_t dy) {
  if (ry == 0) return rx;
  return (int32_t)isqrt64((uint64_t)(((int64_t)rx * rx * ((int64_t)ry * ry - (int64_t)dy * dy)) /
                                     ((int64_t)ry * ry)));
}

inline void fillRect(InkCoverage &c, int32_t x, int32_t y, int32_t w, int32_t h) {
  for (int32_t yy = imax(y, 0); yy < imin(y + h, c.h); yy++) c.span(yy, x, x + w - 1);
}

inline void outlineRect(InkCoverage &c, int32_t x, int32_t y, int32_t w, int32_t h, int32_t s) {
  if (2 * s >= w || 2 * s >= h) { fillRect(c, x, y, w, h); return; }
  for (int32_t yy = imax(y, 0); yy < imin(y + h, c.h); yy++) {
    if (yy < y + s || yy >= y + h - s) c.span(yy, x, x + w - 1);
    else { c.span(yy, x, x + s - 1); c.span(yy, x + w - s, x + w - 1); }
  }
}

inline void fillEllipse(InkCoverage &c, int32_t cx, int32_t cy, int32_t rx, int32_t ry) {
  for (int32_t dy = -ry; dy <= ry; dy++) {
    const int32_t y = cy + dy;
    if (y < 0 || y >= c.h) continue;
    const int32_t hw = ellipseHalf(rx, ry, dy);
    c.span(y, cx - hw, cx + hw);
  }
}

inline void outlineEllipse(InkCoverage &c, int32_t cx, int32_t cy, int32_t rx, int32_t ry, int32_t s) {
  const int32_t irx = rx - s, iry = ry - s;
  if (irx < 0 || iry < 0) { fillEllipse(c, cx, cy, rx, ry); return; }
  for (int32_t dy = -ry; dy <= ry; dy++) {
    const int32_t y = cy + dy;
    if (y < 0 || y >= c.h) continue;
    const int32_t hw = ellipseHalf(rx, ry, dy);
    if (-iry <= dy && dy <= iry) {
      const int32_t ih = ellipseHalf(irx, iry, dy);
      c.span(y, cx - hw, cx - ih - 1);
      c.span(y, cx + ih + 1, cx + hw);
    } else {
      c.span(y, cx - hw, cx + hw);
    }
  }
}

// Even-odd scanline fill; `pts` is x0,y0,x1,y1,... (n points), n <= MAX_POLY.
static const int MAX_POLY = 160;
inline void fillPoly(InkCoverage &c, const int32_t *pts, int n) {
  if (n < 3 || n > MAX_POLY) return;
  int32_t ymin = pts[1], ymax = pts[1];
  for (int i = 1; i < n; i++) { ymin = imin(ymin, pts[2 * i + 1]); ymax = imax(ymax, pts[2 * i + 1]); }
  ymin = imax(ymin, 0);
  ymax = imin(ymax, c.h);
  int32_t xs[MAX_POLY];
  for (int32_t y = ymin; y < ymax; y++) {
    int k = 0;
    for (int i = 0; i < n; i++) {
      const int j = (i + 1) % n;
      const int32_t x0 = pts[2 * i], y0 = pts[2 * i + 1], x1 = pts[2 * j], y1 = pts[2 * j + 1];
      if (y0 == y1) continue;
      if (imin(y0, y1) <= y && y < imax(y0, y1))
        xs[k++] = x0 + (int32_t)fdiv64((int64_t)(y - y0) * (x1 - x0), y1 - y0);
    }
    for (int a = 1; a < k; a++) {                     // insertion sort: k is small
      int32_t v = xs[a];
      int b = a - 1;
      while (b >= 0 && xs[b] > v) { xs[b + 1] = xs[b]; b--; }
      xs[b + 1] = v;
    }
    for (int i = 0; i + 1 < k; i += 2)
      if (xs[i + 1] > xs[i]) c.span(y, xs[i], xs[i + 1] - 1);
  }
}

// The round pen: 1 = a pixel, 2 = a 2x2 square, >= 3 = a disc of radius s/2.
inline void stamp(InkCoverage &c, int32_t x, int32_t y, int32_t s) {
  c.stamps++;
  if (s <= 1) { c.span(y, x, x); return; }
  if (s == 2) { c.span(y, x, x + 1); c.span(y + 1, x, x + 1); return; }
  const int32_t r = s / 2;
  for (int32_t dy = -r; dy <= r; dy++) {
    const int32_t hw = ellipseHalf(r, r, dy);
    c.span(y + dy, x - hw, x + hw);
  }
}

// Bresenham, both ends stamped.
inline void line(InkCoverage &c, int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t s) {
  const int32_t dx = iabs(x1 - x0), dy = -iabs(y1 - y0);
  const int32_t sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
  int32_t err = dx + dy;
  for (;;) {
    stamp(c, x0, y0, s);
    if (x0 == x1 && y0 == y1) return;
    const int32_t e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

// An open polyline drawn point by point: each point after the first draws a segment; a
// polyline that only ever got one point stamps it. `close()` joins the last point to the first.
struct PolyPen {
  InkCoverage &c;
  int32_t s, fx = 0, fy = 0, lx = 0, ly = 0, n = 0;
  PolyPen(InkCoverage &c_, int32_t s_) : c(c_), s(s_) {}
  void to(int32_t x, int32_t y) {
    if (n == 0) { fx = x; fy = y; }
    else line(c, lx, ly, x, y, s);
    lx = x; ly = y; n++;
  }
  void end() { if (n == 1) stamp(c, lx, ly, s); }
  void close() { if (n > 2) line(c, lx, ly, fx, fy, s); else end(); }
};

inline int32_t curveSegments(int32_t length) { return imax(4, imin(32, length / 6)); }

// Point sources. Each calls out(x, y) for every point, in order.
template <class F>
inline void quadPoints(int32_t x1, int32_t y1, int32_t cx, int32_t cy, int32_t x2, int32_t y2, F out) {
  const int32_t n = curveSegments(iabs(cx - x1) + iabs(cy - y1) + iabs(x2 - cx) + iabs(y2 - cy));
  const int64_t nn = (int64_t)n * n;
  for (int32_t i = 0; i <= n; i++) {
    const int64_t a = n - i;
    out((int32_t)fdiv64(a * a * x1 + 2 * a * i * cx + (int64_t)i * i * x2 + nn / 2, nn),
        (int32_t)fdiv64(a * a * y1 + 2 * a * i * cy + (int64_t)i * i * y2 + nn / 2, nn));
  }
}

template <class F>
inline void cubicPoints(int32_t x1, int32_t y1, int32_t ax, int32_t ay, int32_t bx, int32_t by, int32_t x2,
                        int32_t y2, F out) {
  const int32_t n = curveSegments(iabs(ax - x1) + iabs(ay - y1) + iabs(bx - ax) + iabs(by - ay) +
                                  iabs(x2 - bx) + iabs(y2 - by));
  const int64_t n3 = (int64_t)n * n * n;
  for (int32_t i = 0; i <= n; i++) {
    const int64_t a = n - i, ii = i;
    out((int32_t)fdiv64(a * a * a * x1 + 3 * a * a * ii * ax + 3 * a * ii * ii * bx + ii * ii * ii * x2 + n3 / 2, n3),
        (int32_t)fdiv64(a * a * a * y1 + 3 * a * a * ii * ay + 3 * a * ii * ii * by + ii * ii * ii * y2 + n3 / 2, n3));
  }
}

// Degrees, 0 = +x, 90 = +y (down), sweeping from a0 up to a1 (at most one turn).
template <class F>
inline void arcPoints(int32_t cx, int32_t cy, int32_t r, int32_t a0, int32_t a1, F out) {
  while (a1 < a0) a1 += 360;
  if (a1 - a0 > 360) a1 = a0 + 360;
  const int32_t step = r < 24 ? 10 : r < 96 ? 5 : 3;
  int32_t x, y;
  for (int32_t a = a0; a < a1; a += step) { polar(cx, cy, r, a, &x, &y); out(x, y); }
  polar(cx, cy, r, a1, &x, &y);
  out(x, y);
}

template <class F>
inline void wavePoints(int32_t x, int32_t y, int32_t w, int32_t amp, int32_t period, int32_t phase, F out) {
  period = imax(4, period);
  w = imax(0, w);
  int32_t px = x;
  for (;;) {
    const int32_t deg = phase + fdiv((px - x) * 360, period);
    out(px, y + fdiv(amp * sin1024(deg) + 512, 1024));
    if (px >= x + w) return;
    px = imin(px + 2, x + w);
  }
}

template <class F>
inline void spiralPoints(int32_t cx, int32_t cy, int32_t r, int32_t turns, F out) {
  const int32_t total = imax(1, turns) * 360;
  int32_t x, y;
  for (int32_t d = 0; d <= total; d += 10) {
    polar(cx, cy, (int32_t)fdiv64((int64_t)r * d, total), d, &x, &y);
    out(x, y);
  }
}

enum Align : uint8_t { LEFT = 0, CENTER = 1, RIGHT = 2 };

// The classic 5x7 font (PackFont5x7.h), `size` pixels per font pixel, rotated clockwise by
// `turn` (0/90/180/270) about (x, y), which is the top-left corner at turn 0.
inline void text(InkCoverage &c, int32_t x, int32_t y, int32_t size, const char *s, size_t len, int32_t turn,
                 Align align) {
  if (size < 1) size = 1;
  const int32_t n = (int32_t)len;
  const int32_t width = n ? n * 6 * size - size : 0;
  const int32_t ax = align == LEFT ? 0 : align == CENTER ? -(width / 2) : -width;
  for (int32_t i = 0; i < n; i++) {
    int32_t code = (uint8_t)s[i];
    if (code < 32 || code > 126) code = 63;
    for (int32_t col = 0; col < 5; col++) {
      const uint8_t bits = PACKFONT_5X7[code * 5 + col];
      for (int32_t row = 0; row < 7; row++) {
        if (!((bits >> row) & 1)) continue;
        const int32_t u0 = (i * 6 + col) * size + ax, v0 = row * size;
        if (turn == 0) fillRect(c, x + u0, y + v0, size, size);
        else if (turn == 90) fillRect(c, x - v0 - size, y + u0, size, size);
        else if (turn == 180) fillRect(c, x - u0 - size, y - v0 - size, size, size);
        else fillRect(c, x + v0, y - u0 - size, size, size);
      }
    }
  }
}

// ---------------------------------------------------------------- textures and paint
enum Texture : uint8_t { FLAT = 0, NOISE, GATED, CLOUD, HATCH, LINES, VLINES, CROSS, DOTS, TEXTURE_COUNT };
enum Mode : uint8_t { COVER = 0, GLAZE, ERASE, INVERT, MODE_COUNT };

struct Pen {
  uint8_t texture = FLAT;
  uint8_t level = 16;      // 0 = paper .. 16 = solid ink
  uint8_t param = 6;       // texture scale, 2..64
  uint8_t mode = COVER;
  int32_t seed = 0;
};

inline int32_t valueNoise(int32_t xl, int32_t yl, int32_t s, int32_t seed) {
  const int32_t gx = fdiv(xl, s), gy = fdiv(yl, s);
  const int32_t fx = xl - gx * s, fy = yl - gy * s;
  const int32_t v00 = (int32_t)(hash4(seed, gx, gy, 0x51ED) & 255), v10 = (int32_t)(hash4(seed, gx + 1, gy, 0x51ED) & 255);
  const int32_t v01 = (int32_t)(hash4(seed, gx, gy + 1, 0x51ED) & 255), v11 = (int32_t)(hash4(seed, gx + 1, gy + 1, 0x51ED) & 255);
  return (v00 * (s - fx) * (s - fy) + v10 * fx * (s - fy) + v01 * (s - fx) * fy + v11 * fx * fy) / (s * s);
}

// Is the (layer-local) pixel (xl, yl) ink under this pen?
inline bool inked(const Pen &p, int32_t xl, int32_t yl) {
  const int32_t L = p.level;
  if (L <= 0) return false;
  const int32_t P = p.param < 2 ? 2 : p.param;
  switch (p.texture) {
    case FLAT: return L > BAYER4[yl & 3][xl & 3];
    case NOISE: return (int32_t)(hash4(p.seed, xl, yl, 0) & 15) < L;
    case GATED:
      return valueNoise(xl, yl, P, p.seed) >= 128 && (int32_t)(hash4(p.seed ^ 0x5BD1E995, xl, yl, 0) & 15) < L;
    case CLOUD: return ((valueNoise(xl, yl, P, p.seed) * L) >> 8) > BAYER4[yl & 3][xl & 3];
    default: break;
  }
  const int32_t w = (L * P + 15) >> 4;
  switch (p.texture) {
    case HATCH: return pmod(xl + yl, P) < w;
    case LINES: return pmod(yl, P) < w;
    case VLINES: return pmod(xl, P) < w;
    case CROSS: return pmod(xl + yl, P) < w || pmod(xl - yl, P) < w;
    case DOTS: {
      const int32_t dx = 2 * pmod(xl, P) + 1 - P, dy = 2 * pmod(yl, P) + 1 - P;
      return (dx * dx + dy * dy) * 16 < L * 2 * P * P;
    }
    default: return false;
  }
}

// Apply one shape's coverage to `dst`. Only pixels inside the clip rectangle [cx0,cx1) x [cy0,cy1)
// and (when `mask` is given) inside it -- or outside it with `maskOutside` -- are touched.
// Textures read layer-local coordinates (x - mx, y - my), so a moved layer carries them along.
inline void paint(InkBits &dst, const InkCoverage &cov, const Pen &pen, int32_t mx, int32_t my, int32_t cx0 = -32768,
                  int32_t cy0 = -32768, int32_t cx1 = 32767, int32_t cy1 = 32767, const InkBits *mask = nullptr,
                  bool maskOutside = false) {
  const int32_t y0 = imax(0, cy0), y1 = imin(cov.h, cy1);
  const int32_t x0 = imax(0, cx0), x1 = imin(cov.w, cx1);
  for (int32_t y = y0; y < y1; y++) {
    const uint8_t *row = cov.bits + y * cov.stride;
    for (int32_t x = x0; x < x1; x++) {
      if (!((row[x >> 3] >> (7 - (x & 7))) & 1)) continue;
      if (mask && mask->get(x, y) == maskOutside) continue;
      if (pen.mode == ERASE) { dst.put(x, y, false); continue; }
      const bool ink = inked(pen, x - mx, y - my);
      if (pen.mode == COVER) dst.put(x, y, ink);
      else if (!ink) continue;
      else if (pen.mode == GLAZE) dst.put(x, y, true);
      else dst.flip(x, y);
    }
  }
}

// An InkBits onto any canvas with drawFastHLine(x, y, w, colour) -- PackCanvas, GFXcanvas8, ...
// Runs of ink and paper become one call each.
template <class Canvas>
inline void blit(Canvas &canvas, int x, int y, const InkBits &b, uint16_t inkColour, uint16_t paperColour) {
  for (int yy = 0; yy < b.h; yy++) {
    int start = 0;
    bool cur = b.get(0, yy);
    for (int xx = 1; xx <= b.w; xx++) {
      const bool v = xx < b.w ? b.get(xx, yy) : !cur;
      if (v != cur) {
        canvas.drawFastHLine((int16_t)(x + start), (int16_t)(y + yy), (int16_t)(xx - start), cur ? inkColour : paperColour);
        start = xx;
        cur = v;
      }
    }
  }
}

}  // namespace packink
