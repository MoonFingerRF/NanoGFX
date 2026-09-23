// PackInk host test: every rule checked against numbers from the Python reference of the same
// rules (the Nostalgia hub's memory_proxy/art/ink.py), so the e-paper panel and the hub's preview
// draw the same pixels. Build and run:
//
//   c++ -std=c++17 -O2 -Wall -Wextra -I../src ink_host.cpp -o /tmp/ink_host && /tmp/ink_host
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "PackInk.h"

using namespace packink;

static int failures = 0;
#define CHECK_EQ(got, want)                                                                            \
  do {                                                                                                 \
    long long g_ = (long long)(got), w_ = (long long)(want);                                           \
    if (g_ != w_) { std::printf("FAIL %s:%d %s = %lld, want %lld\n", __FILE__, __LINE__, #got, g_, w_); failures++; } \
  } while (0)

static const int W = 64, H = 48;
static uint8_t buf[(W / 8) * H], out[(W / 8) * H];

static InkCoverage fresh() {
  InkCoverage c(W, H, buf);
  c.reset();
  return c;
}

static uint64_t fnv64(const uint8_t *p, size_t n) {
  uint64_t h = 0xCBF29CE484222325ull;
  for (size_t i = 0; i < n; i++) { h ^= p[i]; h *= 0x100000001B3ull; }
  return h;
}

int main() {
  // the integer helpers the whole contract rests on
  CHECK_EQ(fdiv(-7, 2), -4);
  CHECK_EQ(fdiv(7, -2), -4);
  CHECK_EQ(fdiv(-8, 2), -4);
  CHECK_EQ(pmod(-1, 6), 5);
  CHECK_EQ(isqrt64(99), 9);
  CHECK_EQ(isqrt64(100), 10);
  CHECK_EQ(mix(1), 1753845952u);
  CHECK_EQ(hash4(1, 2, 3, 4), 4028510986u);
  CHECK_EQ(hash4(-1, -2, 3, 4), 1444262145u);
  CHECK_EQ(valueNoise(-7, 13, 12, 99), 141);
  CHECK_EQ(sin1024(90), 1024);
  CHECK_EQ(cos1024(-90), 0);

  // coverage: pixel counts (and pen stamps) of every rasteriser
  { auto c = fresh(); fillRect(c, -3, 5, 20, 9); CHECK_EQ(c.count(), 153); }
  { auto c = fresh(); outlineRect(c, 4, 4, 50, 30, 3); CHECK_EQ(c.count(), 444); }
  { auto c = fresh(); fillEllipse(c, 30, 20, 25, 14); CHECK_EQ(c.count(), 1091); }
  { auto c = fresh(); outlineEllipse(c, 30, 20, 25, 14, 4); CHECK_EQ(c.count(), 436); }
  { auto c = fresh(); const int32_t p[] = {2, 2, 60, 10, 40, 45, 20, 20, 5, 40}; fillPoly(c, p, 5); CHECK_EQ(c.count(), 1368); }
  { auto c = fresh(); line(c, -5, 3, 70, 44, 3); CHECK_EQ(c.count(), 192); CHECK_EQ(c.stamps, 76); }
  { auto c = fresh(); PolyPen pen(c, 2); quadPoints(0, 40, 30, -20, 63, 40, [&](int32_t x, int32_t y) { pen.to(x, y); }); pen.end();
    CHECK_EQ(c.count(), 200); CHECK_EQ(c.stamps, 106); }
  { auto c = fresh(); PolyPen pen(c, 1); cubicPoints(0, 0, 70, 10, -10, 40, 63, 47, [&](int32_t x, int32_t y) { pen.to(x, y); }); pen.end();
    CHECK_EQ(c.count(), 89); CHECK_EQ(c.stamps, 120); }
  { auto c = fresh(); PolyPen pen(c, 1); arcPoints(32, 24, 20, 200, -20, [&](int32_t x, int32_t y) { pen.to(x, y); }); pen.end();
    CHECK_EQ(c.count(), 43); CHECK_EQ(c.stamps, 56); }
  { auto c = fresh(); PolyPen pen(c, 2); wavePoints(-4, 24, 70, -10, 23, 45, [&](int32_t x, int32_t y) { pen.to(x, y); }); pen.end();
    CHECK_EQ(c.count(), 296); CHECK_EQ(c.stamps, 165); }
  { auto c = fresh(); PolyPen pen(c, 1); spiralPoints(32, 24, 22, 3, [&](int32_t x, int32_t y) { pen.to(x, y); }); pen.end();
    CHECK_EQ(c.count(), 189); CHECK_EQ(c.stamps, 296); }
  { auto c = fresh(); text(c, 32, 24, 2, "Hi~", 3, 90, CENTER); CHECK_EQ(c.count(), 124); }
  { auto c = fresh(); text(c, 60, 40, 1, "ink!", 4, 180, RIGHT); CHECK_EQ(c.count(), 6); }
  { auto c = fresh(); text(c, 3, 40, 1, "up", 2, 270, LEFT); CHECK_EQ(c.count(), 25); }

  // textures: ink pixels over the 64x48 field at level 7, seed 5, shifted by (3, -2)
  const struct { uint8_t t, p; int want; } tex[] = {
      {FLAT, 6, 1344}, {NOISE, 6, 1318}, {GATED, 12, 705}, {CLOUD, 12, 524}, {HATCH, 6, 1536},
      {LINES, 6, 1536}, {VLINES, 6, 1488}, {CROSS, 6, 2224}, {DOTS, 6, 2032}};
  for (const auto &t : tex) {
    Pen pen;
    pen.texture = t.t; pen.param = t.p; pen.level = 7; pen.seed = 5;
    int n = 0;
    for (int y = 0; y < H; y++)
      for (int x = 0; x < W; x++) n += inked(pen, x - 3, y + 2);
    CHECK_EQ(n, t.want);
  }

  // paint modes, layered: cloud cover, an erased hole, an inverted hatched ellipse, glazed text
  InkBits dst(W, H, out);
  dst.clear();
  auto layer = [&](uint8_t texture, uint8_t param, uint8_t level, int32_t seed, uint8_t mode, int32_t mx, int32_t my) {
    Pen pen;
    pen.texture = texture; pen.param = param; pen.level = level; pen.seed = seed; pen.mode = mode;
    InkCoverage c(W, H, buf);
    paint(dst, c, pen, mx, my);
  };
  { auto c = fresh(); fillRect(c, 0, 0, W, H); } layer(CLOUD, 10, 9, 3, COVER, 0, 0);
  { auto c = fresh(); fillEllipse(c, 20, 20, 15, 12); } layer(FLAT, 6, 0, 0, ERASE, 0, 0);
  { auto c = fresh(); fillEllipse(c, 40, 26, 18, 10); } layer(HATCH, 5, 8, 0, INVERT, 2, 1);
  { auto c = fresh(); text(c, 4, 30, 2, "ab", 2, 0, LEFT); } layer(NOISE, 6, 12, 9, GLAZE, 0, 0);
  CHECK_EQ(fnv64(out, sizeof out), 0x0bc979e2e1642bb2ull);

  // a mask: only inside a circle, and only inside a clip rectangle
  {
    static uint8_t mbuf[(W / 8) * H];
    InkCoverage m(W, H, mbuf);
    m.reset();
    fillEllipse(m, 32, 24, 10, 10);
    dst.clear();
    auto c = fresh();
    fillRect(c, 0, 0, W, H);
    Pen pen;
    paint(dst, c, pen, 0, 0, 0, 0, 32, 48, &m, false);
    uint32_t half = 0;
    for (int y = 0; y < H; y++) for (int x = 0; x < 32; x++) half += m.get(x, y);
    CHECK_EQ(half > 100, 1);
    CHECK_EQ(dst.count(), half);                                  // exactly the disc's left half
    dst.clear();
    paint(dst, c, pen, 0, 0, -32768, -32768, 32767, 32767, &m, true);
    CHECK_EQ(dst.count(), (uint32_t)(W * H) - m.count());
  }

  if (failures) { std::printf("%d failure(s)\n", failures); return 1; }
  std::printf("PackInk host test: all checks passed\n");
  return 0;
}
