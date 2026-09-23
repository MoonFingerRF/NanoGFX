// Host test for the standalone (no Adafruit_GFX) build: NanoGFXBase, PackMono, PackRLE images,
// and the UC8179 state machine against a simulated bus.
//   c++ -std=c++17 -O1 -Wall -Wextra -I src tests/host_standalone.cpp -o /tmp/ngfx_host && /tmp/ngfx_host
// Optionally pass a PackRLE image made by tools/packrle.py:  ngfx_host image.bin W H  (checks
// that the C decoder and re-encoder agree with the Python encoder byte for byte).
#define NANOGFX_STANDALONE 1
#include "NanoGFX.h"
#include "PackMono.h"
#include "UC8179.h"
#include "EInkGhost.h"
#include <stdio.h>
#include <vector>

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)

static std::vector<uint8_t> mem(size_t n) { return std::vector<uint8_t>(n, 0); }

static void test_canvas_and_text() {
  PackCanvas c(64, 32, false);
  c.packed4 = true;
  auto buf = mem(PackCanvas::bufBytes(64, 32, true));
  c.useBuffer(buf.data());
  c.fillScreen(0);
  c.fillRect(2, 3, 10, 4, 1);
  CHECK(c.getPixel(2, 3) == 1 && c.getPixel(11, 6) == 1 && c.getPixel(12, 6) == 0);
  int16_t x1, y1; uint16_t w, h;
  c.setTextSize(1);
  c.getTextBounds("AB", 0, 0, &x1, &y1, &w, &h);
  CHECK(w == 12 && h == 8);                       // classic font: 6 px advance, 8 px line
  c.setCursor(0, 20); c.setTextColor(1); c.print("I");
  bool any = false;
  for (int y = 20; y < 28; y++) for (int x = 0; x < 6; x++) any |= c.getPixel(x, y) == 1;
  CHECK(any);
}

static void test_mono_export() {
  const int W = 40, H = 8;
  PackCanvas c(W, H, false);
  c.packed4 = true;
  auto buf = mem(PackCanvas::bufBytes(W, H, true));
  c.useBuffer(buf.data());
  c.fillScreen(0);
  c.fillRect(0, 0, 8, 8, 1);                      // solid ink
  c.fillRect(8, 0, 8, 8, 3);                      // grey index 3
  PackMono m;
  m.setLevel(3, 8);                               // 50 %: half the Bayer cells
  auto out = mem(PackMono::rowBytes(W) * H);
  m.exportRows(c, out.data(), 0, H);
  CHECK(out[0] == 0xFF && out[2] == 0x00);        // ink byte, paper byte
  int ones = 0;
  for (int y = 0; y < 4; y++) for (int b = 0; b < 8; b++) ones += (out[y * 5 + 1] >> b) & 1;
  CHECK(ones == 16);                              // 50 % of a 4x8 block
  // naive reference for every pixel
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++) {
      uint8_t idx = c.getPixel(x, y);
      bool ink = m.getLevel(idx) > PackMono::BAYER4[y & 3][x & 3];
      bool got = (out[y * 5 + (x >> 3)] >> (7 - (x & 7))) & 1;
      if (ink != got) { CHECK(ink == got); return; }
    }
  auto mir = mem(PackMono::rowBytes(W) * H);
  m.exportRows(c, mir.data(), 0, H, true, true);
  CHECK(mir[4] == 0xFF && mir[0] == 0x00);        // mirrored: the ink block is now on the right
  auto inv = mem(PackMono::rowBytes(W) * H);
  m.exportRows(c, inv.data(), 0, H, false);
  CHECK(inv[0] == 0x00 && inv[2] == 0xFF);
  int y0, y1;
  CHECK(!PackMono::diffRows(out.data(), out.data(), W, H, &y0, &y1));
  auto other = out;
  other[3 * 5 + 2] ^= 0x10;
  other[5 * 5 + 0] ^= 0x01;
  CHECK(PackMono::diffRows(out.data(), other.data(), W, H, &y0, &y1) && y0 == 3 && y1 == 6);
}

static void test_prle_image(const char *path, int w, int h) {
  // Round trip in C first.
  std::vector<uint8_t> idx((size_t)w * h);
  for (size_t i = 0; i < idx.size(); i++) idx[i] = (uint8_t)((i / 7 + ((int)(i % w) > w / 2)) % 3);
  std::vector<uint8_t> enc(h * (2 + PRLE_STRIDE(w)));
  size_t n = prle_image_encode_idx8(idx.data(), w, h, enc.data());
  std::vector<uint8_t> flat(h * PRLE_STRIDE(w));
  CHECK(prle_image_decode_flat(enc.data(), n, w, h, flat.data()));
  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++) {
      uint8_t b = flat[y * PRLE_STRIDE(w) + (x >> 1)];
      if (((x & 1) ? (b & 15) : (b >> 4)) != idx[y * w + x]) { CHECK(false); return; }
    }
  CHECK(!prle_image_decode_flat(enc.data(), n - 1, w, h, flat.data()));   // truncated -> false
  if (!path) return;
  FILE *f = fopen(path, "rb");
  if (!f) { CHECK(false); return; }
  std::vector<uint8_t> py(1 << 20);
  size_t pn = fread(py.data(), 1, py.size(), f);
  fclose(f);
  std::vector<uint8_t> pflat(h * PRLE_STRIDE(w));
  CHECK(prle_image_decode_flat(py.data(), pn, w, h, pflat.data()));
  std::vector<uint8_t> id8((size_t)w * h);
  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++) {
      uint8_t b = pflat[y * PRLE_STRIDE(w) + (x >> 1)];
      id8[y * w + x] = (x & 1) ? (b & 15) : (b >> 4);
    }
  std::vector<uint8_t> re(h * (2 + PRLE_STRIDE(w)));
  size_t rn = prle_image_encode_idx8(id8.data(), w, h, re.data());
  CHECK(rn == (size_t)pn && memcmp(re.data(), py.data(), pn) == 0);   // Python == C, byte for byte
}

// ---- UC8179 against a simulated bus --------------------------------------------------
struct SimBus {
  uint32_t t = 0, busyUntil = 0;
  std::vector<uint8_t> cmds;
  size_t dataBytes = 0;
  void command(uint8_t c) {
    cmds.push_back(c);
    if (c == 0x04) busyUntil = t + 50;
    if (c == 0x12) busyUntil = t + 900;
    if (c == 0x02) busyUntil = t + 20;
  }
  void data(const uint8_t *, size_t n) { dataBytes += n; }
  bool busy() { return (int32_t)(busyUntil - t) > 0; }
  void reset(bool) {}
  void delayMs(uint32_t ms) { t += ms; }
  uint32_t millis() { return t; }
};

static void test_uc8179() {
  SimBus bus;
  UC8179<SimBus> epd(bus);
  epd.begin();
  static uint8_t a[48000], b[48000];
  CHECK(epd.startFull(a));
  CHECK(!epd.startFull(a));                      // busy: refused, not queued
  int guard = 0;
  while (!epd.isIdle() && guard++ < 10000) { bus.t += 5; epd.poll(); }
  CHECK(epd.isIdle() && !epd.faulted());
  CHECK(bus.dataBytes >= 48000);
  bus.dataBytes = 0;
  CHECK(epd.startPartial(b, a, 100, 140));
  while (!epd.isIdle() && guard++ < 20000) { bus.t += 5; epd.poll(); }
  CHECK(bus.dataBytes >= 2u * 40 * 100 && bus.dataBytes < 2u * 41 * 100 + 64);  // only the window, twice
  CHECK(epd.partialsSinceFull() == 1);
  CHECK(!epd.powered());
  // A narrow window sends only its own bytes; staying powered skips the rails.
  bus.dataBytes = 0;
  epd.stayPowered(true);
  CHECK(epd.startPartial(b, a, 10, 20, false, 4, 6));
  while (!epd.isIdle() && guard++ < 30000) { bus.t += 5; epd.poll(); }
  CHECK(bus.dataBytes >= 2u * 10 * 2 && bus.dataBytes < 2u * 10 * 2 + 64);
  CHECK(epd.powered() && epd.lastTiming().off == 0);
  size_t cmds = bus.cmds.size();
  CHECK(epd.startPartial(b, a, 10, 20, false, 4, 6));
  bool sawPowerOn = false;
  while (!epd.isIdle() && guard++ < 40000) { bus.t += 5; epd.poll(); }
  for (size_t i = cmds; i < bus.cmds.size(); i++) sawPowerOn |= bus.cmds[i] == 0x04;
  CHECK(!sawPowerOn);
  CHECK(epd.powerDown());
  while (!epd.isIdle() && guard++ < 50000) { bus.t += 5; epd.poll(); }
  CHECK(!epd.powered());
  epd.stayPowered(false);
  bool sawWindow = false;
  for (uint8_t c : bus.cmds) sawWindow |= c == 0x90;
  CHECK(sawWindow);
  // A panel that never lets go of BUSY is a fault, not a hang.
  CHECK(epd.startFull(a));
  bus.busyUntil = bus.t + 100000;
  while (!epd.isIdle() && guard++ < 40000) { bus.t += 50; bus.busyUntil = bus.t + 100000; epd.poll(); }
  CHECK(epd.faulted());
}

static void test_ghost() {
  const int W = 100, H = 20;                     // row bytes, rows
  std::vector<uint8_t> glass(W * H, 0), frame(W * H, 0);
  EInkGhost g(H, W, 32);
  // one tick in a small box: a PARTIAL window around exactly the change
  frame[5 * W + 40] = 0xF0; frame[6 * W + 42] = 0x01;
  EInkGhost::Plan p = g.plan(glass.data(), frame.data(), false, true);
  CHECK(p.kind == EInkGhost::PARTIAL && p.y0 == 5 && p.y1 == 7 && p.xb0 == 40 && p.xb1 == 43);
  g.done(p); glass = frame;
  CHECK(g.wear(5) == 1 && g.wear(4) == 0);
  // ambient ticks wear by 1; past the budget the same change is a CLEAN of that box
  int cleans = 0, partials = 0;
  for (int i = 0; i < 40; i++) {
    frame[5 * W + 40] ^= 0xFF;
    p = g.plan(glass.data(), frame.data(), false, true);
    if (p.kind == EInkGhost::CLEAN) { cleans++; CHECK(p.y0 == 5 && p.xb0 <= 40 && p.xb1 >= 41); }
    else partials++;
    g.done(p); glass = frame;
  }
  CHECK(cleans == 1 && partials == 39);
  // an event on a third-worn row cleans while it redraws anyway
  for (int i = 0; i < 12; i++) { frame[5 * W + 40] ^= 0xFF; p = g.plan(glass.data(), frame.data(), false, true); g.done(p); glass = frame; }
  frame[5 * W + 41] = 0xAA;
  p = g.plan(glass.data(), frame.data(), false, false);
  CHECK(p.kind == EInkGhost::CLEAN);
  g.done(p); glass = frame;
  CHECK(g.wear(5) == 0);
  // nothing changed: nothing to do; forced: full
  CHECK(g.plan(glass.data(), frame.data()).kind == EInkGhost::NONE);
  CHECK(g.plan(glass.data(), frame.data(), true).kind == EInkGhost::FULL);
  // a quiet screen finds its worn region
  for (int i = 0; i < 10; i++) { frame[15 * W + 7] ^= 0xFF; p = g.plan(glass.data(), frame.data(), false, true); g.done(p); glass = frame; }
  EInkGhost::Plan q;
  CHECK(g.idleClean(&q) && q.y0 == 15 && q.y1 == 16 && q.xb0 == 7 && q.xb1 == 8);
  g.done(q);
  CHECK(!g.idleClean(&q));
  g.setTemperature(5.0f);
  CHECK(g.budget() == 16);
}

int main(int argc, char **argv) {
  test_canvas_and_text();
  test_mono_export();
  test_prle_image(argc > 3 ? argv[1] : nullptr, argc > 3 ? atoi(argv[2]) : 37, argc > 3 ? atoi(argv[3]) : 11);
  test_uc8179();
  test_ghost();
  printf(fails ? "host_standalone: %d FAILED\n" : "host_standalone: ok\n", fails);
  return fails ? 1 : 0;
}
