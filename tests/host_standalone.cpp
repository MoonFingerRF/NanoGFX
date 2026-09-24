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
  // exportRect: the rectangle as exportRows makes it, every other byte untouched
  for (int inkBit = 0; inkBit < 2; inkBit++) {
    auto whole = mem(PackMono::rowBytes(W) * H);
    m.exportRows(c, whole.data(), 0, H, inkBit != 0);
    auto rect = mem(PackMono::rowBytes(W) * H);
    for (auto &b : rect) b = 0x5A;
    m.exportRect(c, rect.data(), 1, 3, 2, 7, inkBit != 0);
    bool ok = true;
    for (int y = 0; y < H; y++)
      for (int b = 0; b < 5; b++) {
        const bool in = b >= 1 && b < 3 && y >= 2 && y < 7;
        ok &= in ? rect[y * 5 + b] == whole[y * 5 + b] : rect[y * 5 + b] == 0x5A;
      }
    CHECK(ok);
    m.exportRect(c, rect.data(), 0, 99, 0, 99, inkBit != 0);   // clamped: the whole frame
    CHECK(rect == whole);
  }
  {   // a width that is not a multiple of 8: the tail byte
    PackCanvas t(36, 5, false);
    t.packed4 = true;
    auto tb = mem(PackCanvas::bufBytes(36, 5, true));
    t.useBuffer(tb.data());
    t.fillScreen(0);
    t.fillRect(30, 0, 6, 5, 3);
    t.fillRect(20, 1, 9, 2, 1);
    auto whole = mem(PackMono::rowBytes(36) * 5), rect = mem(PackMono::rowBytes(36) * 5);
    m.exportRows(t, whole.data(), 0, 5);
    m.exportRect(t, rect.data(), 2, 5, 0, 5);
    bool ok = true;
    for (int y = 0; y < 5; y++)
      for (int b = 0; b < 5; b++) ok &= b >= 2 ? rect[y * 5 + b] == whole[y * 5 + b] : rect[y * 5 + b] == 0;
    CHECK(ok);
  }
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
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> log;   // every command with its data
  void command(uint8_t c) {
    log.push_back({c, {}});
    cmds.push_back(c);
    if (c == 0x04) busyUntil = t + 50;
    if (c == 0x12) busyUntil = t + 900;
    if (c == 0x02) busyUntil = t + 20;
  }
  void data(const uint8_t *p, size_t n) {
    dataBytes += n;
    if (!log.empty() && log.back().second.size() < 256) log.back().second.insert(log.back().second.end(), p, p + (n > 256 ? 256 : n));
  }
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
  const int W = 100, H = 24;                     // row bytes, rows (3 cell rows of 8)
  std::vector<uint8_t> glass(W * H, 0), frame(W * H, 0);
  EInkGhost g(H, W, 32);                         // 32 full toggles a cell
  auto toggleCell = [&](int cy, int b) { for (int r = 0; r < 8; r++) frame[(cy * 8 + r) * W + b] ^= 0xFF; };
  // one small change: a PARTIAL window around exactly the change
  frame[5 * W + 40] = 0xF0; frame[6 * W + 42] = 0x01;
  EInkGhost::Plan p = g.plan(glass.data(), frame.data(), false, true);
  CHECK(p.kind == EInkGhost::PARTIAL && p.y0 == 5 && p.y1 == 7 && p.xb0 == 40 && p.xb1 == 43);
  g.done(p); glass = frame;
  CHECK(g.wear(5) == 0 && g.headroom(0, 8, 40, 41) == 31);   // 4 of 64 pixels: under 1 toggle
  // a whole cell flipped each time wears it by 1; past the budget it is a CLEAN of whole cells
  int cleans = 0, partials = 0;
  for (int i = 0; i < 40; i++) {
    toggleCell(0, 40);
    p = g.plan(glass.data(), frame.data(), false, true);
    if (p.kind == EInkGhost::CLEAN) { cleans++; CHECK(p.y0 == 0 && p.y1 == 8 && p.xb0 <= 40 && p.xb1 >= 41); }
    else partials++;
    g.done(p); glass = frame;
  }
  CHECK(cleans == 1 && partials == 39);
  // early clean is off by default: an event on a worn cell is a partial ...
  for (int i = 0; i < 12; i++) { toggleCell(0, 40); p = g.plan(glass.data(), frame.data(), false, true); g.done(p); glass = frame; }
  frame[5 * W + 40] ^= 0x0F;                     // an event on the worn cell
  CHECK(g.plan(glass.data(), frame.data(), false, false).kind == EInkGhost::PARTIAL);
  // ... and with it on, a third-worn cell is cleaned while it is redrawn anyway
  g.setEarlyClean(true);
  p = g.plan(glass.data(), frame.data(), false, false);
  CHECK(p.kind == EInkGhost::CLEAN && p.y0 == 0 && p.y1 == 8);
  g.done(p); glass = frame;
  CHECK(g.headroom(0, 8, 40, 41) == 32);
  g.setEarlyClean(false);
  // a progress bar: one pixel further each tick flips each pixel once -- 1/8 of a toggle per cell
  for (int x = 0; x < 160; x++) {
    frame[18 * W + 10 + x / 8] |= (uint8_t) (0x80 >> (x % 8));
    p = g.plan(glass.data(), frame.data(), false, true);
    CHECK(p.kind == EInkGhost::PARTIAL);
    g.done(p); glass = frame;
  }
  CHECK(g.wear(18) == 0 && g.headroom(16, 24, 10, 30) == 31 && g.lastHeadroom() >= 2000);
  // nothing changed: nothing to do; forced: full
  CHECK(g.plan(glass.data(), frame.data()).kind == EInkGhost::NONE);
  CHECK(g.plan(glass.data(), frame.data(), true).kind == EInkGhost::FULL);
  // a quiet screen finds its worn region (cells >= a quarter of the budget)
  EInkGhost::Plan q;
  CHECK(!g.idleClean(&q));
  for (int i = 0; i < 10; i++) { toggleCell(2, 7); p = g.plan(glass.data(), frame.data(), false, true); g.done(p); glass = frame; }
  CHECK(g.idleClean(&q) && q.y0 == 16 && q.y1 == 24 && q.xb0 == 7 && q.xb1 == 8 && g.maxWear() == 10);
  CHECK(g.lastHeadroom() == 22);                 // (32 - 10) more like the last one
  g.done(q);
  CHECK(!g.idleClean(&q));
  g.setTemperature(5.0f);
  CHECK(g.budget() == 16);
  g.setBudget(360);
  CHECK(g.budget() == 180);                      // still cold: half
  g.setTemperature(22.0f);
  CHECK(g.budget() == 360);
}

// The fast register-LUT path: the datasheet's table lengths, the research's safety rules S1-S4.
static void test_fast() {
  SimBus bus;
  UC8179<SimBus> epd(bus);
  epd.begin();
  static uint8_t a[48000], b[48000];
  bus.log.clear();
  CHECK(!epd.startFast(b, a, 0, 128, 2, 32, 0) && !epd.startFast(b, a, 0, 128, 2, 32, 61));   // bounds
  CHECK(epd.startFast(b, a, 40, 168, 2, 32, 25));
  auto find = [&](uint8_t c) -> const std::vector<uint8_t> * {
    for (auto &e : bus.log) if (e.first == c) return &e.second;
    return nullptr;
  };
  auto psr = find(0x00), vdcs = find(0x82), cdi = find(0x50);
  CHECK(psr && psr->size() == 1 && (*psr)[0] == 0x3F);
  CHECK(vdcs && vdcs->size() == 1 && (*vdcs)[0] == 0x26);                     // S3
  CHECK(cdi && cdi->size() == 2 && (*cdi)[0] == 0x39);                        // border -> LUTBD
  const uint8_t lens[6] = {60, 42, 60, 60, 60, 42};
  for (int k = 0; k < 6; k++) { auto t = find(0x20 + k); CHECK(t && t->size() == lens[k]); }
  auto zero = [](const std::vector<uint8_t> *t) { for (uint8_t v : *t) if (v) return false; return true; };
  CHECK(zero(find(0x21)) && zero(find(0x24)) && zero(find(0x25)));             // S1: WW, KK, BD
  auto kw = find(0x22), wk = find(0x23), vc = find(0x20);
  CHECK((*kw)[0] == 0x80 && (*wk)[0] == 0x40);                                 // VDL / VDH
  for (int i = 1; i < 60; i++) CHECK((*kw)[i] == (*wk)[i]);                    // S2: mirror timing
  CHECK((*kw)[1] == 25 && (*kw)[5] == 1 && (*vc)[0] == 0x00 && (*vc)[1] == 25); // VCOM at VCOM_DC
  for (auto &e : bus.log) if (e.first >= 0x20 && e.first <= 0x25)
    for (size_t i = 0; i < e.second.size(); i += 6) for (int ph = 0; ph < 4; ph++)
      CHECK(((e.second[i] >> (6 - 2 * ph)) & 3) != 3);                         // never VDHR / float
  int guard = 0;
  while (!epd.isIdle() && guard++ < 20000) { bus.t += 5; epd.poll(); }
  CHECK(epd.isIdle() && epd.fastActive());
  // a VCOM sweep setting: the code is clamped, the all-zero VCOM table has no frames
  epd.fastVcom(0x05, 1);
  CHECK(epd.fastVcomCode() == 0x0A && epd.fastVcomTable() == 1);
  bus.log.clear();
  CHECK(epd.startFast(b, a, 40, 168, 2, 32, 13));
  CHECK((*find(0x82))[0] == 0x0A && zero(find(0x20)) && (*find(0x22))[1] == 13);
  while (!epd.isIdle() && guard++ < 30000) { bus.t += 5; epd.poll(); }
  epd.fastVcom(0x26, 0);
  // S4: the next OTP refresh starts from a hardware reset: PSR 0x1F before any refresh command
  bus.log.clear();
  CHECK(epd.startPartial(b, a, 0, 8));
  CHECK(!epd.fastActive());
  bool sawPsr = false, psrOtp = false;
  for (auto &e : bus.log) {
    if (e.first == 0x00 && !sawPsr) { sawPsr = true; psrOtp = e.second.size() == 1 && e.second[0] == 0x1F; }
    CHECK(!(e.first >= 0x20 && e.first <= 0x25));                             // no LUT on the OTP path
  }
  CHECK(sawPsr && psrOtp);
  bool powerOn = false;
  for (auto &e : bus.log) if (e.first == 0x04) powerOn = true;
  CHECK(powerOn);                                                              // the reset took the rails down
  while (!epd.isIdle() && guard++ < 40000) { bus.t += 5; epd.poll(); }
  epd.startFast(b, a, 40, 168, 2, 32, 25);
  while (!epd.isIdle() && guard++ < 60000) { bus.t += 5; epd.poll(); }
  bus.log.clear();
  CHECK(epd.startFull(a) && !epd.fastActive());
  CHECK(!bus.log.empty() && bus.log[0].first == 0x01);                         // begin() came first
  while (!epd.isIdle() && guard++ < 70000) { bus.t += 5; epd.poll(); }
  // leaving fast with the rails still up: power off (0x02) BEFORE the reset, unless disabled
  for (int fix = 1; fix >= 0; fix--) {
    epd.offBeforeReset(fix == 1);
    epd.stayPowered(true);
    CHECK(epd.startFast(b, a, 40, 168, 2, 32, 10));
    while (!epd.isIdle() && guard++ < 90000) { bus.t += 5; epd.poll(); }
    CHECK(epd.powered());
    bus.log.clear();
    CHECK(epd.startPartial(b, a, 0, 8) && !epd.fastActive());
    CHECK(!bus.log.empty() && bus.log[0].first == (fix ? 0x02 : 0x01));
    while (!epd.isIdle() && guard++ < 99000) { bus.t += 5; epd.poll(); }
    epd.stayPowered(false);
    epd.powerDown();
    while (!epd.isIdle() && guard++ < 99999) { bus.t += 5; epd.poll(); }
  }
  epd.offBeforeReset(true);
  // the null refresh: VCOM driven at VCOM_DC, every source LUT zero, no pixel data change
  while (!epd.isIdle() && guard++ < 99999) { bus.t += 5; epd.poll(); }
  bus.log.clear();
  const uint32_t rails = epd.railCycles();
  CHECK(epd.startNull(a, 0, 8, 0, 1, 4));
  auto n20 = find(0x20), n22 = find(0x22), n23 = find(0x23);
  CHECK(n20 && (*n20)[0] == 0x00 && (*n20)[1] == 4 && (*n20)[5] == 1);
  CHECK(zero(n22) && zero(n23) && zero(find(0x21)) && zero(find(0x24)) && zero(find(0x25)));
  CHECK(epd.railCycles() == rails + (epd.powered() ? 0 : 1));
}

int main(int argc, char **argv) {
  test_canvas_and_text();
  test_mono_export();
  test_prle_image(argc > 3 ? argv[1] : nullptr, argc > 3 ? atoi(argv[2]) : 37, argc > 3 ? atoi(argv[3]) : 11);
  test_uc8179();
  test_fast();
  test_ghost();
  printf(fails ? "host_standalone: %d FAILED\n" : "host_standalone: ok\n", fails);
  return fails ? 1 : 0;
}
