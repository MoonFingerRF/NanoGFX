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

static void test_factory_scan() {
  SimBus bus;
  UC8179<SimBus> epd(bus);
  epd.begin();
  static uint8_t a[48000], b[48000];
  auto complete = [&]() {
    int guard = 0;
    while (!epd.isIdle() && guard++ < 5000) { bus.t += 5; epd.poll(); }
    CHECK(epd.isIdle() && !epd.faulted());
  };
  auto scan = [&]() -> int {
    for (auto i = bus.log.rbegin(); i != bus.log.rend(); ++i)
      if (i->first == 0x90 && i->second.size() == 9) return i->second[8];
    return -1;
  };
  auto vcomWritten = [&]() {
    for (const auto &i : bus.log) if (i.first == 0x82) return true;
    return false;
  };
  CHECK(epd.partialScanOutside());
  bus.log.clear();
  CHECK(epd.startPartial(b, a, 44, 124, false, 3, 63)); complete();
  CHECK(scan() == 1 && !vcomWritten());
  bus.log.clear();
  epd.partialScanOutside(false);
  CHECK(!epd.partialScanOutside() && bus.log.empty());  // setter never writes voltage/registers
  CHECK(epd.startPartial(b, a, 44, 124, false, 3, 63)); complete();
  CHECK(scan() == 0 && !vcomWritten());
  bus.log.clear();
  CHECK(epd.startFull(b)); complete();
  CHECK(scan() == -1 && !vcomWritten());                // full refresh unaffected
  bus.log.clear();
  CHECK(epd.startFast(b, a, 44, 124, 3, 63, 4)); complete();
  CHECK(scan() == 1);                                  // validated FAST scan unchanged
  bus.log.clear();
  CHECK(epd.startPartial(b, a, 44, 124, false, 3, 63)); complete();
  CHECK(scan() == 0 && !vcomWritten());                  // factory selection survives FAST reset
  epd.partialScanOutside(true);
  bus.log.clear();
  CHECK(epd.startPartial(b, a, 44, 124, false, 3, 63)); complete();
  CHECK(scan() == 1 && !vcomWritten());                  // explicit rollback
}

static void test_factory_power_bytes() {
  std::vector<std::pair<uint8_t,std::vector<uint8_t>>> reference;
  static uint8_t a[48000],b[48000]; memset(b,0x55,sizeof(b));
  for(bool down:{false,true}) {
    SimBus bus; UC8179<SimBus> epd(bus);epd.begin();epd.stayPowered(!down);
    bus.cmds.clear();bus.log.clear();bus.dataBytes=0;
    for(int n=0;n<3;++n) {
      CHECK(epd.startPartial(b,a,40,60,false,3,11));
      for(int guard=0;guard<2000 && !epd.isIdle();++guard){bus.t+=5;epd.poll();}
      CHECK(epd.isIdle() && !epd.faulted());
      CHECK(down ? !epd.powered() && epd.lastTiming().off>0 : epd.powered() && epd.lastTiming().off==0);
    }
    int pon=0,pof=0;std::vector<std::pair<uint8_t,std::vector<uint8_t>>> content;
    for(auto &entry:bus.log) {
      pon+=entry.first==0x04;pof+=entry.first==0x02;
      if(entry.first!=0x04 && entry.first!=0x02 && entry.first!=0x71) content.push_back(entry);
      CHECK(entry.first!=0x82 && !(entry.first>=0x20 && entry.first<=0x25));
      if(entry.first==0xE0) CHECK(entry.second==std::vector<uint8_t>{0x02});
      if(entry.first==0xE5) CHECK(entry.second==std::vector<uint8_t>{0x6E});
    }
    CHECK(pon==(down?3:1) && pof==(down?3:0));
    if(!down)reference=content;else CHECK(content==reference);  // exact windows/payloads/LUT/bin unchanged
  }
}

static void test_idle_power() {
  SimBus bus; UC8179<SimBus> epd(bus); epd.begin();
  bus.cmds.clear(); bus.dataBytes = 0;
  CHECK(epd.powerUpWithoutRefresh());
  CHECK(bus.cmds == std::vector<uint8_t>{0x04});
  CHECK(!epd.powerUpWithoutRefresh());
  for (int i=0; i<100 && !epd.isIdle(); ++i) { bus.t+=10; epd.poll(); }
  CHECK(epd.isIdle() && epd.powered() && !epd.faulted());
  CHECK(bus.dataBytes == 0 && epd.partialsSinceFull()==0);
  for (auto c:bus.cmds) CHECK(c==0x04 || c==0x71);
  bus.cmds.clear(); CHECK(epd.powerUpWithoutRefresh() && bus.cmds.empty());
  epd.powerDown(); CHECK(bus.cmds == std::vector<uint8_t>{0x02});
  for(int i=0;i<100 && !epd.isIdle();++i){bus.t+=10;epd.poll();}
  CHECK(!epd.powered());
  bus.cmds.clear();bus.busyUntil=bus.t+100;
  CHECK(!epd.powerUpWithoutRefresh() && bus.cmds.empty());
  bus.t+=100; CHECK(epd.powerUpWithoutRefresh());
  bus.busyUntil=bus.t+20000;bus.t+=13000;epd.poll();
  CHECK(epd.faulted() && bus.cmds.back()==0x02);
  bus.t+=30;epd.poll();
  CHECK(epd.isIdle() && epd.faulted() && !epd.powered());
  bus.cmds.clear(); CHECK(!epd.powerUpWithoutRefresh() && bus.cmds.empty());
  bus.t+=20000;epd.begin();static uint8_t a[48000],b[48000];
  CHECK(epd.startFast(b,a,44,124,3,63,4));
  for(int i=0;i<2000 && !epd.isIdle();++i){bus.t+=10;epd.poll();}
  bus.cmds.clear(); CHECK(!epd.powerUpWithoutRefresh() && bus.cmds.empty());
  epd.leaveFast();CHECK(epd.startNull(a,0,8,0,1,4));
  for(int i=0;i<2000 && !epd.isIdle();++i){bus.t+=10;epd.poll();}
  bus.cmds.clear(); CHECK(!epd.powerUpWithoutRefresh() && bus.cmds.empty());
}

static void test_old_sync() {
  struct CaptureBus : SimBus {
    std::vector<uint8_t> old;
    bool autoBusy = false;
    void data(const uint8_t *p, size_t n) {
      SimBus::data(p, n);
      if (!log.empty() && log.back().first == 0x10) {
        old.insert(old.end(), p, p + n);
        if (autoBusy && old.size() >= 48000) busyUntil = t + 900;
      }
    }
  } bus;
  UC8179<CaptureBus> epd(bus);
  epd.begin();
  std::vector<uint8_t> glass(48000), next(48000);
  for (size_t i = 0; i < glass.size(); ++i) glass[i] = (uint8_t)(i * 37 + i / 100);
  auto complete = [&]() {
    int guard = 0;
    while (!epd.isIdle() && guard++ < 5000) { bus.t += 5; epd.poll(); }
    CHECK(epd.isIdle() && !epd.faulted());
  };
  auto clear = [&]() { bus.log.clear(); bus.cmds.clear(); bus.old.clear(); bus.dataBytes = 0; };
  for (bool inv : {false, true}) {
    epd.invert(inv);
    clear();
    auto count = epd.partialsSinceFull();
    auto last = epd.lastRefreshMs();
    auto powered = epd.powered();
    CHECK(epd.syncOld(glass.data()));
    CHECK(bus.cmds == std::vector<uint8_t>({0x91, 0x90, 0x10, 0x92}));
    CHECK(bus.log[1].second == std::vector<uint8_t>({0,0,3,31,0,0,1,223,1}));
    CHECK(bus.old.size() == glass.size());
    for (size_t i = 0; i < glass.size(); ++i) CHECK(bus.old[i] == (inv ? (uint8_t)~glass[i] : glass[i]));
    CHECK(epd.isIdle() && epd.powered() == powered && epd.partialsSinceFull() == count && epd.lastRefreshMs() == last);
  }
  clear(); CHECK(!epd.syncOld(nullptr) && bus.cmds.empty());
  bus.busyUntil = bus.t + 100;
  clear(); CHECK(!epd.syncOld(glass.data()) && bus.cmds.empty());
  bus.t += 100;
  epd.stayPowered(true);
  CHECK(epd.startPartial(next.data(), glass.data(), 44, 124, false, 3, 63));
  clear(); CHECK(!epd.syncOld(glass.data()) && bus.cmds.empty());
  complete();
  CHECK(epd.powered());
  clear(); CHECK(epd.syncOld(glass.data()) && epd.powered());  // no power change while rails up
  epd.partialScanOutside(false);
  clear(); CHECK(epd.startPartial(next.data(), glass.data(), 44, 124, false, 3, 63));
  CHECK(bus.log[0].first == 0x50); complete();
  bool narrow = false;
  for (const auto &entry : bus.log) if (entry.first == 0x90) narrow = entry.second == std::vector<uint8_t>({0,24,1,247,0,44,0,123,0});
  CHECK(narrow);
  clear(); CHECK(epd.syncOld(glass.data()));
  CHECK(bus.log[1].second[8] == 0);
  CHECK(epd.startFast(next.data(), glass.data(), 44,124,3,63,4)); complete();
  clear(); CHECK(!epd.syncOld(glass.data()) && bus.cmds.empty());
  CHECK(epd.startPartial(next.data(), glass.data(),44,124,false,3,63)); complete();
  clear(); bus.autoBusy = true;
  CHECK(epd.syncOld(glass.data()));
  CHECK(epd.isIdle() && epd.controllerBusy());  // software idle alone is NOT no-refresh proof
  clear(); CHECK(!epd.syncOld(glass.data()) && bus.cmds.empty());
  bus.autoBusy = false; bus.t += 1000;
  CHECK(epd.startPartial(next.data(), glass.data(),44,124,false,3,63));
  bus.busyUntil = bus.t + 100000; bus.t += 13000; epd.poll();
  CHECK(epd.faulted());
  clear(); CHECK(!epd.syncOld(glass.data()) && bus.cmds.empty());
}

// NEUTRAL partial (2026-10-01): exact bytes, DC balance per transition, zero-field tables, scan,
// VCOM clamp, bounds, and the S4 exit back to the vendor registers.
static void test_neutral() {
  typedef UC8179<SimBus> E;
  // the tables themselves
  for (int n = 0; n <= 40; n++) {
    uint8_t c[60], ww[42], kw[60], wk[60], kk[60], bd[42];
    const bool ok = E::neutralTables((uint8_t) n, c, ww, kw, wk, kk, bd);
    CHECK(ok == (n >= E::NEUTRAL_MIN_FRAMES && n <= E::NEUTRAL_MAX_FRAMES));
    if (!ok) continue;
    CHECK(E::lutNetFrames(kw, 60) == 0 && E::lutNetFrames(wk, 60) == 0);        // N2 per transition
    CHECK(E::lutAllZeroLevel(c, 60) && E::lutAllZeroLevel(ww, 42) && E::lutAllZeroLevel(kk, 60) &&
          E::lutAllZeroLevel(bd, 42));                                           // N1, N3
    // K->W ends at VDL (to white) after a VDH pre-phase; W->K is its mirror (N2)
    CHECK(kw[0] == 0x40 && kw[6] == 0x80 && wk[0] == 0x80 && wk[6] == 0x40);
    // every table runs the same two groups of n frames, once, then the end condition (all zero)
    const uint8_t *all[6] = {c, ww, kw, wk, kk, bd};
    const size_t len[6] = {60, 42, 60, 60, 60, 42};
    for (int t = 0; t < 6; t++) {
      CHECK(all[t][1] == n && all[t][7] == n && all[t][5] == 1 && all[t][11] == 1);
      for (int i = 2; i <= 4; i++) CHECK(all[t][i] == 0 && all[t][6 + i] == 0);
      for (size_t i = 12; i < len[t]; i++) CHECK(all[t][i] == 0);
      for (size_t g = 0; g < len[t]; g += 6) CHECK((all[t][g] & 0xC0) != 0xC0);   // never 11b
    }
    // total drive time per transition: 2n frames, |V| = 15 V, never longer than 60 frames
    CHECK(2 * n <= 60);
  }
  // the arithmetic checker itself catches an unbalanced or VDHR table
  {
    uint8_t t[60];
    E::fastLut(t, 60, E::LVL_TO_WHITE, 10);
    CHECK(E::lutNetFrames(t, 60) == -10);          // the FAST single phase: balanced only as a pair
    t[0] = 0xC0;
    CHECK(E::lutNetFrames(t, 60) == INT32_MIN);
    E::fastLut(t, 60, 0x00, 10);
    CHECK(E::lutAllZeroLevel(t, 60));
    t[0] = 0x40;
    CHECK(!E::lutAllZeroLevel(t, 60));
  }
  SimBus bus;
  E epd(bus);
  epd.begin();
  static uint8_t prev[48000], next[48000];
  for (size_t i = 0; i < sizeof next; i++) next[i] = (uint8_t) (i * 7);
  for (size_t i = 0; i < sizeof prev; i++) prev[i] = (uint8_t) (i * 13);
  auto complete = [&]() {
    int guard = 0;
    while (!epd.isIdle() && guard++ < 5000) { bus.t += 5; epd.poll(); }
    CHECK(epd.isIdle() && !epd.faulted());
  };
  static const std::vector<uint8_t> none(9, 0xEE);   // "not sent": never equal to a real payload
  auto find = [&](uint8_t c) -> const std::vector<uint8_t> * {
    for (auto &e : bus.log) if (e.first == c) return &e.second;
    return &none;
  };
  auto has = [&](uint8_t c) { return find(c) != &none; };
  // bounds and refusals send nothing
  bus.log.clear();
  CHECK(!epd.startNeutral(next, prev, 40, 60, 3, 11, 3));
  CHECK(!epd.startNeutral(next, prev, 40, 60, 3, 11, 31));
  CHECK(!epd.startNeutral(next, nullptr, 40, 60, 3, 11, 10));
  CHECK(!epd.startNeutral(next, prev, 60, 60, 3, 11, 10));
  CHECK(bus.log.empty());
  CHECK(epd.neutralVcomCode() == 0x0A);
  // the exact command order of one neutral partial, rails down afterwards
  epd.stayPowered(false);
  CHECK(epd.startNeutral(next, prev, 40, 60, 3, 11, 10));
  CHECK(!epd.startNeutral(next, prev, 40, 60, 3, 11, 10));          // busy: refused
  complete();
  CHECK(epd.neutralActive() && epd.fastActive() && !epd.powered() && epd.neutralCount() == 1);
  std::vector<uint8_t> order;
  for (auto &e : bus.log) if (e.first != 0x71) order.push_back(e.first);
  const std::vector<uint8_t> want = {0x00, 0x82, 0x50, 0xE0, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x04,
                                     0x91, 0x90, 0x10, 0x13, 0x12, 0x92, 0x02};
  CHECK(order == want);
  CHECK(*find(0x00) == std::vector<uint8_t>{0x3F});
  CHECK(*find(0x82) == std::vector<uint8_t>{0x0A});
  CHECK(*find(0x50) == (std::vector<uint8_t>{0x39, 0x07}));
  CHECK(*find(0xE0) == std::vector<uint8_t>{0x00});
  {
    uint8_t c[60], ww[42], kw[60], wk[60], kk[60], bd[42];
    CHECK(E::neutralTables(10, c, ww, kw, wk, kk, bd));
    CHECK(*find(0x20) == std::vector<uint8_t>(c, c + 60));
    CHECK(*find(0x21) == std::vector<uint8_t>(ww, ww + 42));
    CHECK(*find(0x22) == std::vector<uint8_t>(kw, kw + 60));
    CHECK(*find(0x23) == std::vector<uint8_t>(wk, wk + 60));
    CHECK(*find(0x24) == std::vector<uint8_t>(kk, kk + 60));
    CHECK(*find(0x25) == std::vector<uint8_t>(bd, bd + 42));
  }
  // window 24..87 x 40..59, PT_SCAN default 1; OLD = prev's window, NEW = next's window (invert off)
  CHECK(*find(0x90) == (std::vector<uint8_t>{0x00, 24, 0x00, 87, 0x00, 40, 0x00, 59, 0x01}));
  {
    std::vector<uint8_t> wantOld, wantNew;
    for (int y = 40; y < 60 && wantOld.size() < 256; y++)
      for (int b = 3; b < 11 && wantOld.size() < 256; b++) {
        wantOld.push_back(prev[y * 100 + b]);
        wantNew.push_back(next[y * 100 + b]);
      }
    CHECK(*find(0x10) == wantOld && *find(0x13) == wantNew);
  }
  for (auto &e : bus.log) CHECK(e.first != 0xE5 && e.first != 0x2A && e.first != 0x01 && e.first != 0x06);
  // window-only scan for NEUTRAL; FAST keeps its validated 1; factory follows the setting
  epd.partialScanOutside(false);
  bus.log.clear();
  epd.stayPowered(true);
  CHECK(epd.startNeutral(prev, next, 40, 60, 3, 11, 10)); complete();
  CHECK((*find(0x90))[8] == 0x00 && has(0x04) && !has(0x02) && epd.powered());
  bus.log.clear();
  CHECK(epd.startNeutral(next, prev, 40, 60, 3, 11, 10)); complete();
  CHECK(!has(0x04) && !has(0x02));                                 // rails kept up: no PON, no POF
  bus.log.clear();
  CHECK(epd.startFast(prev, next, 40, 60, 3, 11, 10)); complete();
  CHECK((*find(0x90))[8] == 0x01 && !epd.neutralActive());
  bus.log.clear();
  CHECK(epd.startNeutral(next, prev, 40, 60, 3, 11, 10)); complete();
  CHECK((*find(0x90))[8] == 0x00 && epd.neutralActive());
  // VCOM clamp: never more negative than 0x0A; less negative allowed on the neutral path only
  epd.neutralVcom(0x26);
  CHECK(epd.neutralVcomCode() == 0x0A);
  epd.neutralVcom(0x04);
  CHECK(epd.neutralVcomCode() == 0x04);
  bus.log.clear();
  CHECK(epd.startNeutral(prev, next, 40, 60, 3, 11, 10)); complete();
  CHECK(*find(0x82) == std::vector<uint8_t>{0x04});
  epd.neutralVcom(0x0A);
  // S4 exit: a factory partial after NEUTRAL powers off, resets, loads the vendor registers
  bus.log.clear();
  CHECK(epd.startPartial(next, prev, 40, 60, false, 3, 11)); complete();
  CHECK(!epd.fastActive() && !epd.neutralActive());
  CHECK(bus.log.size() > 2 && bus.log[0].first == 0x02);              // POF before the reset
  CHECK(*find(0x00) == std::vector<uint8_t>{0x1F});    // vendor PSR again
  CHECK((*find(0x90))[8] == 0x00);                                    // factory follows the setting
  for (auto &e : bus.log) CHECK(e.first != 0x82);                     // no VDCS on the OTP path
  epd.partialScanOutside(true);
  // a stuck BUSY during NEUTRAL is a fault
  CHECK(epd.startNeutral(next, prev, 40, 60, 3, 11, 10));
  int guard = 0;
  while (!epd.isIdle() && guard++ < 40000) { bus.t += 50; bus.busyUntil = bus.t + 100000; epd.poll(); }
  CHECK(epd.faulted());
}

// PAIR partial (2026-10-01): one-direction drive per transition, balanced over the K->W / W->K pair;
// zero-field tables as NEUTRAL; exact bytes; bounded per-pixel running DC over any history.
static void test_pair() {
  typedef UC8179<SimBus> E;
  for (int n = 0; n <= 40; n++)
    for (int sh = 0; sh <= 12; sh++) {
      uint8_t c[60], ww[42], kw[60], wk[60], kk[60], bd[42];
      const bool ok = E::pairTables((uint8_t) n, (uint8_t) sh, c, ww, kw, wk, kk, bd);
      CHECK(ok == (n >= E::NEUTRAL_MIN_FRAMES && n <= E::NEUTRAL_MAX_FRAMES && sh <= E::PAIR_MAX_SHAKE && sh < n));
      if (!ok) continue;
      CHECK(E::pairBalanced(kw, wk));
      CHECK(E::lutNetFrames(kw, 60) == sh - n && E::lutNetFrames(wk, 60) == n - sh);   // one direction each
      CHECK(E::lutAllZeroLevel(c, 60) && E::lutAllZeroLevel(ww, 42) && E::lutAllZeroLevel(kk, 60) &&
            E::lutAllZeroLevel(bd, 42));
      // the drive phase is the last group that runs: K->W ends on VDL (white), W->K on VDH (black)
      const int g = sh ? 6 : 0;
      CHECK(kw[g] == E::LVL_TO_WHITE && wk[g] == E::LVL_TO_BLACK && kw[g + 1] == n && wk[g + 1] == n);
      if (sh) CHECK(kw[0] == E::LVL_TO_BLACK && wk[0] == E::LVL_TO_WHITE && kw[1] == sh && wk[1] == sh);
      const uint8_t *all[6] = {c, ww, kw, wk, kk, bd};
      const size_t len[6] = {60, 42, 60, 60, 60, 42};
      for (int t = 0; t < 6; t++) {                       // identical timing in every table
        CHECK(all[t][g + 1] == n && all[t][g + 5] == 1);
        if (sh) CHECK(all[t][1] == sh && all[t][5] == 1);
        for (size_t i = (size_t) g + 6; i < len[t]; i++) CHECK(all[t][i] == 0);
        for (size_t q = 0; q < len[t]; q += 6) CHECK((all[t][q] & 0xC0) != 0xC0);
      }
      // per-pixel running DC over a long random history: transitions alternate, so the running
      // sum never leaves {0, n - sh} (started white) -- one transition at most
      uint32_t r = 12345u + (uint32_t) (n * 31 + sh);
      bool black = false;
      int32_t sum = 0;
      for (int step = 0; step < 4000; step++) {
        r = r * 1664525u + 1013904223u;
        const bool want = (r >> 16) & 1;
        if (want != black) { sum += black ? E::lutNetFrames(kw, 60) : E::lutNetFrames(wk, 60); black = want; }
        CHECK(sum == (black ? n - sh : 0));
      }
    }
  // pairBalanced refuses: an uneven pair, both the same way, a backwards pair, VDHR
  {
    uint8_t kw[60], wk[60];
    E::twoPhaseLut(kw, 60, 0, 0, E::LVL_TO_WHITE, 10);
    E::twoPhaseLut(wk, 60, 0, 0, E::LVL_TO_BLACK, 11);
    CHECK(!E::pairBalanced(kw, wk));
    E::twoPhaseLut(wk, 60, 0, 0, E::LVL_TO_WHITE, 10);
    CHECK(!E::pairBalanced(kw, wk));
    E::twoPhaseLut(wk, 60, 0, 0, E::LVL_TO_BLACK, 10);
    CHECK(E::pairBalanced(kw, wk) && !E::pairBalanced(wk, kw));
    wk[0] = 0xC0;
    CHECK(!E::pairBalanced(kw, wk));
  }
  SimBus bus;
  E epd(bus);
  epd.begin();
  static uint8_t prev[48000], next[48000];
  for (size_t i = 0; i < sizeof next; i++) next[i] = (uint8_t) (i * 7);
  for (size_t i = 0; i < sizeof prev; i++) prev[i] = (uint8_t) (i * 13);
  auto complete = [&]() {
    int guard = 0;
    while (!epd.isIdle() && guard++ < 5000) { bus.t += 5; epd.poll(); }
    CHECK(epd.isIdle() && !epd.faulted());
  };
  static const std::vector<uint8_t> none(9, 0xEE);
  auto find = [&](uint8_t c) -> const std::vector<uint8_t> * {
    for (auto &e : bus.log) if (e.first == c) return &e.second;
    return &none;
  };
  bus.log.clear();
  CHECK(!epd.startPair(next, prev, 40, 60, 3, 11, 3));
  CHECK(!epd.startPair(next, prev, 40, 60, 3, 11, 31));
  CHECK(!epd.startPair(next, prev, 40, 60, 3, 11, 10, 10));
  CHECK(!epd.startPair(next, prev, 40, 60, 3, 11, 20, 11));
  CHECK(!epd.startPair(next, nullptr, 40, 60, 3, 11, 10));
  CHECK(bus.log.empty());
  epd.stayPowered(false);
  CHECK(epd.startPair(next, prev, 40, 60, 3, 11, 20, 2));
  complete();
  CHECK(epd.pairActive() && epd.neutralActive() && epd.pairCount() == 1 && epd.neutralCount() == 0);
  std::vector<uint8_t> order;
  for (auto &e : bus.log) if (e.first != 0x71) order.push_back(e.first);
  CHECK(order == (std::vector<uint8_t>{0x00, 0x82, 0x50, 0xE0, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x04,
                                       0x91, 0x90, 0x10, 0x13, 0x12, 0x92, 0x02}));
  CHECK(*find(0x82) == std::vector<uint8_t>{0x0A} && *find(0x00) == std::vector<uint8_t>{0x3F});
  CHECK(*find(0x50) == (std::vector<uint8_t>{0x39, 0x07}));
  {
    uint8_t c[60], ww[42], kw[60], wk[60], kk[60], bd[42];
    CHECK(E::pairTables(20, 2, c, ww, kw, wk, kk, bd));
    CHECK(*find(0x20) == std::vector<uint8_t>(c, c + 60) && *find(0x21) == std::vector<uint8_t>(ww, ww + 42));
    CHECK(*find(0x22) == std::vector<uint8_t>(kw, kw + 60) && *find(0x23) == std::vector<uint8_t>(wk, wk + 60));
    CHECK(*find(0x24) == std::vector<uint8_t>(kk, kk + 60) && *find(0x25) == std::vector<uint8_t>(bd, bd + 42));
  }
  CHECK((*find(0x90))[8] == 0x01);
  for (auto &e : bus.log) CHECK(e.first != 0xE5 && e.first != 0x2A && e.first != 0x01 && e.first != 0x06);
  epd.partialScanOutside(false);
  bus.log.clear();
  CHECK(epd.startPair(prev, next, 40, 60, 3, 11, 20)); complete();
  CHECK((*find(0x90))[8] == 0x00);
  bus.log.clear();
  CHECK(epd.startNeutral(next, prev, 40, 60, 3, 11, 10)); complete();
  CHECK(!epd.pairActive() && epd.neutralActive() && (*find(0x90))[8] == 0x00);
  bus.log.clear();
  CHECK(epd.startFast(prev, next, 40, 60, 3, 11, 10)); complete();
  CHECK(!epd.pairActive() && (*find(0x90))[8] == 0x01);
  epd.partialScanOutside(true);
  epd.stayPowered(true);                              // rails up: the exit must power off first
  bus.log.clear();
  CHECK(epd.startPair(next, prev, 40, 60, 3, 11, 20)); complete();
  CHECK(epd.powered());
  bus.log.clear();
  CHECK(epd.startFull(prev)); complete();
  CHECK(!epd.pairActive() && !epd.fastActive() && bus.log[0].first == 0x02);         // S4 exit fence
}

// OTP COPY (2026-10-01): a factory KW table set, sent byte for byte (padded to the register
// lengths), with only VDCS/CDI/EVS from the caller; every refusal rule sends nothing.
static void test_copy() {
  typedef UC8179<SimBus> E;
  // the shape of the panel's partial: 3 + 15 + 5 + 5, then 6 + 3 frames
  const uint8_t c[42] = {0x00, 3, 15, 5, 5, 1, 0x00, 6, 3, 0, 0, 1};
  const uint8_t ww[42] = {0x00, 3, 15, 5, 5, 1, 0x00, 6, 3, 0, 0, 1};
  const uint8_t kw[42] = {0x2A, 3, 15, 5, 5, 1, 0xA0, 6, 3, 0, 0, 1};
  const uint8_t wk[42] = {0x15, 3, 15, 5, 5, 1, 0x00, 6, 3, 0, 0, 1};
  const uint8_t kk[42] = {0x40, 3, 15, 5, 5, 1, 0x00, 6, 3, 0, 0, 1};
  CHECK(E::copyFrames(c) == 37 && E::copyTablesOk(c, ww, kw, wk, kk));
  SimBus bus;
  E epd(bus);
  epd.begin();
  static uint8_t prev[48000], next[48000];
  for (size_t i = 0; i < sizeof next; i++) next[i] = (uint8_t) (i * 7);
  auto complete = [&]() {
    int guard = 0;
    while (!epd.isIdle() && guard++ < 5000) { bus.t += 5; epd.poll(); }
    CHECK(epd.isIdle() && !epd.faulted());
  };
  static const std::vector<uint8_t> none(9, 0xEE);
  auto find = [&](uint8_t cm) -> const std::vector<uint8_t> * {
    for (auto &e : bus.log) if (e.first == cm) return &e.second;
    return &none;
  };
  // refusals send nothing
  bus.log.clear();
  {
    uint8_t bad[42];
    memcpy(bad, kw, 42); bad[0] = 0xEA;                   // phase 0 = 11b (VDHR)
    CHECK(!epd.startOtpCopy(next, prev, 40, 60, 3, 11, c, ww, bad, wk, kk, 0x12));
    memcpy(bad, c, 42); bad[0] = 0x40;                    // VCOM AC (VCOMH)
    CHECK(!epd.startOtpCopy(next, prev, 40, 60, 3, 11, bad, ww, kw, wk, kk, 0x12));
    memcpy(bad, c, 42); bad[0] = 0xC0;                    // VCOM floating
    CHECK(!epd.startOtpCopy(next, prev, 40, 60, 3, 11, bad, ww, kw, wk, kk, 0x12));
    memcpy(bad, wk, 42); bad[2] = 16;                     // timing differs from LUTC
    CHECK(!epd.startOtpCopy(next, prev, 40, 60, 3, 11, c, ww, kw, bad, kk, 0x12));
    uint8_t lc[42] = {0x00, 100, 0, 0, 0, 1, 0x00, 21, 0, 0, 0, 1}, lk[42] = {0x40, 100, 0, 0, 0, 1, 0x80, 21, 0, 0, 0, 1};
    CHECK(!epd.startOtpCopy(next, prev, 40, 60, 3, 11, lc, lc, lk, lk, lc, 0x12));   // 121 frames
    CHECK(!epd.startOtpCopy(next, prev, 40, 60, 3, 11, c, ww, kw, wk, kk, 0x09));   // VCOM -0.55
    CHECK(!epd.startOtpCopy(next, prev, 40, 60, 3, 11, c, ww, kw, wk, kk, 0x27));   // VCOM -2.05
    CHECK(!epd.startOtpCopy(next, prev, 40, 60, 3, 11, c, ww, kw, wk, kk, 0x12, 0x11));   // CDI
    CHECK(!epd.startOtpCopy(next, prev, 40, 60, 3, 11, c, ww, kw, wk, kk, 0x12, 0x19, 0x01)); // EVS
    CHECK(!epd.startOtpCopy(next, prev, 40, 60, 3, 11, c, ww, kw, wk, nullptr, 0x12));
  }
  CHECK(bus.log.empty());
  // the exact bytes
  epd.stayPowered(false);
  CHECK(epd.startOtpCopy(next, prev, 40, 60, 3, 11, c, ww, kw, wk, kk, 0x26, 0x19, 0x03));
  complete();
  CHECK(epd.copyActive() && epd.neutralActive() && !epd.pairActive() && epd.copyCount() == 1);
  std::vector<uint8_t> order;
  for (auto &e : bus.log) if (e.first != 0x71) order.push_back(e.first);
  CHECK(order == (std::vector<uint8_t>{0x00, 0x82, 0x50, 0x52, 0xE0, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x04,
                                       0x91, 0x90, 0x10, 0x13, 0x12, 0x92, 0x02}));
  CHECK(*find(0x82) == std::vector<uint8_t>{0x26} && *find(0x50) == (std::vector<uint8_t>{0x19, 0x07}));
  CHECK(*find(0x52) == std::vector<uint8_t>{0x03} && *find(0x00) == std::vector<uint8_t>{0x3F});
  auto pad = [](const uint8_t *t, size_t n) { std::vector<uint8_t> v(t, t + 42); v.resize(n, 0); return v; };
  CHECK(*find(0x20) == pad(c, 60) && *find(0x21) == pad(ww, 42) && *find(0x22) == pad(kw, 60));
  CHECK(*find(0x23) == pad(wk, 60) && *find(0x24) == pad(kk, 60) && *find(0x25) == std::vector<uint8_t>(42, 0));
  for (auto &e : bus.log) CHECK(e.first != 0xE5 && e.first != 0x2A && e.first != 0x01 && e.first != 0x06 && e.first != 0x30);
  CHECK((*find(0x90))[8] == 0x01);
  // EVS 0 is not written; scan follows the setting; the next NEUTRAL clears the copy flag
  epd.partialScanOutside(false);
  bus.log.clear();
  CHECK(epd.startOtpCopy(prev, next, 40, 60, 3, 11, c, ww, kw, wk, kk, 0x12)); complete();
  CHECK(find(0x52) == &none && (*find(0x90))[8] == 0x00 && *find(0x82) == std::vector<uint8_t>{0x12});
  CHECK(epd.startNeutral(next, prev, 40, 60, 3, 11, 10)); complete();
  CHECK(!epd.copyActive() && epd.neutralActive());
  epd.partialScanOutside(true);
  // S4 exit fence before any OTP refresh
  epd.stayPowered(true);
  CHECK(epd.startOtpCopy(prev, next, 40, 60, 3, 11, c, ww, kw, wk, kk, 0x12)); complete();
  bus.log.clear();
  CHECK(epd.startPartial(next, prev, 40, 60, false, 3, 11)); complete();
  CHECK(!epd.copyActive() && !epd.fastActive() && bus.log[0].first == 0x02 && *find(0x00) == std::vector<uint8_t>{0x1F});
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
  test_factory_scan();
  test_old_sync();
  test_idle_power();
  test_factory_power_bytes();
  test_neutral();
  test_pair();
  test_copy();
  test_ghost();
  printf(fails ? "host_standalone: %d FAILED\n" : "host_standalone: ok\n", fails);
  return fails ? 1 : 0;
}
