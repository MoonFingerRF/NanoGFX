// Host test of SSD1677.h against a simulated controller: the RAM window and address counters
// (data entry x+, y+), both RAMs, BUSY, and the rotation mapping. Checks, for every rotation:
//   * a full refresh puts the logical frame into both RAMs at the documented RAM coordinates
//     (upright landscape row y at RAM row 479 - y; portrait = Waveshare's ROTATE_270);
//   * a partial writes old/new in its window and leaves old == new everywhere afterwards;
//   * random partial windows never write outside the byte-widened window.
// Build: c++ -std=c++17 -O1 -Wall -fsanitize=address,undefined -I../src ssd1677_host.cpp -o ssd1677_host
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "SSD1677.h"

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

struct Sim {
  uint8_t glass[480][100];      // what the glass shows: mode 1 (0xF7) copies 0x24, mode 2 (0xFF) drives old != new
  uint8_t ram[2][480][100];     // [0] = 0x24 BW (new), [1] = 0x26 RED (old)
  int xs = 0, xe = 799, ys = 0, ye = 479, xc = 0, yc = 0;
  int cmdReg = -1, argN = 0, target = -1;
  uint8_t args[8];
  int busyLeft = 0;
  uint32_t now = 0;
  int refreshes = 0, lastUpdate = -1, lastCtrl = -1;
  bool wroteOutside = false, wroteOld = false, forbidden = false;
  int lastCmd = -1, swresets = 0;
  // DEEP SLEEP (0x10 0x01), as measured on the glass 2026-10-03: a controller woken by a bare hardware reset
  // runs no update (BUSY stays high for good); EPD_3IN97_Init's SWRESET after the reset is what wakes it.
  bool asleep = false, resetSinceSleep = false, cmdWhileAsleep = false;
  int winRx0 = 0, winRx1 = 800, winRy0 = 0, winRy1 = 480;   // the window the test allows

  void command(uint8_t c) {
    if (asleep && !resetSinceSleep) cmdWhileAsleep = true;   // it does not listen until reset
    if (asleep && c == 0x12) asleep = false;                 // the SWRESET after the reset: awake
    if (asleep && c == 0x20) { busyLeft = 1 << 30; }         // a bare reset: BUSY high for good
    cmdReg = c; argN = 0; target = -1; lastCmd = c;
    if (c == 0x21 || c == 0x32 || c == 0x03 || c == 0x04 || c == 0x2C || c == 0x37) forbidden = true;
    if (c == 0x26) wroteOld = true;
    if (c == 0x24) target = 0;
    if (c == 0x26) target = 1;
    if (c == 0x20) {
      refreshes++; busyLeft = 3;
      for (int y = 0; y < 480; y++)
        for (int x = 0; x < 100; x++) {
          if (lastUpdate == 0xF7) glass[y][x] = ram[0][y][x];
          else if (lastUpdate == 0xFF)   // only bits where old and new differ are driven, to new
            glass[y][x] = (uint8_t) ((glass[y][x] & ~(ram[0][y][x] ^ ram[1][y][x])) | (ram[0][y][x] & (ram[0][y][x] ^ ram[1][y][x])));
        }
    }
    if (c == 0x12) { swresets++; }
    if (c == 0x12) { xs = 0; xe = 799; ys = 0; ye = 479; xc = yc = 0; }
  }
  void data(const uint8_t *p, size_t n) {
    for (size_t i = 0; i < n; i++) {
      if (target >= 0) {
        if (xc / 8 < 100 && yc < 480) {
          if (xc < winRx0 || xc >= winRx1 || yc < winRy0 || yc >= winRy1) wroteOutside = true;
          ram[target][yc][xc / 8] = p[i];
        }
        xc += 8;
        if (xc > xe) { xc = xs; yc++; if (yc > ye) yc = ys; }
        continue;
      }
      if (argN < 8) args[argN] = p[i];
      argN++;
      if (cmdReg == 0x44 && argN == 4) { xs = args[0] | args[1] << 8; xe = args[2] | args[3] << 8; }
      if (cmdReg == 0x45 && argN == 4) { ys = args[0] | args[1] << 8; ye = args[2] | args[3] << 8; }
      if (cmdReg == 0x4E && argN == 2) xc = args[0] | args[1] << 8;
      if (cmdReg == 0x4F && argN == 2) yc = args[0] | args[1] << 8;
      if (cmdReg == 0x22 && argN == 1) lastUpdate = args[0];
      if (cmdReg == 0x10 && argN == 1 && (args[0] & 3)) { asleep = true; resetSinceSleep = false; }
      if (cmdReg == 0x21 && argN == 1) lastCtrl = args[0];
    }
  }
  bool busy() { if (busyLeft > 0) { busyLeft--; return true; } return false; }
  void reset(bool level) { if (!level && asleep) resetSinceSleep = true; }
  void delayMs(uint32_t ms) { now += ms; }
  uint32_t millis() { return now += 1; }
};

static int ramBit(const Sim &s, int reg, int rx, int ry) { return (s.ram[reg][ry][rx / 8] >> (7 - (rx & 7))) & 1; }
static int frameBit(const std::vector<uint8_t> &f, int rb, int lx, int ly) { return (f[(size_t) ly * rb + lx / 8] >> (7 - (lx & 7))) & 1; }

// Where the header says logical (lx, ly) lives in RAM.
static void expected(int rot, int lx, int ly, int *rx, int *ry) {
  switch (rot) {
    case 1: *rx = ly; *ry = lx; break;
    case 2: *rx = 799 - lx; *ry = ly; break;
    case 3: *rx = 799 - ly; *ry = 479 - lx; break;
    default: *rx = lx; *ry = 479 - ly;
  }
}

static void run(Sim &s, SSD1677<Sim> &d) {
  for (int i = 0; i < 1000 && !d.isIdle(); i++) { d.poll(); s.now += 10; }
}

int main() {
  srand(7);
  for (int rot = 0; rot < 4; rot++) {
    Sim s;
    memset(s.ram, 0x55, sizeof s.ram);
    SSD1677<Sim> d(s);
    d.rotation((SSD1677<Sim>::Rotation) rot);
    d.begin();
    const int W = d.width(), H = d.height(), rb = d.rowBytes();
    CHECK((rot & 1) ? (W == 480 && H == 800) : (W == 800 && H == 480), "dims rot %d", rot);
    std::vector<uint8_t> a((size_t) rb * H), b((size_t) rb * H);
    for (auto &v : a) v = (uint8_t) rand();
    // full
    CHECK(d.startFull(a.data()), "startFull");
    run(s, d);
    CHECK(d.isIdle() && !d.faulted(), "full idle");
    CHECK(s.lastUpdate == 0xF7 && s.lastCtrl == -1, "full update bytes %02X %02X", s.lastUpdate, s.lastCtrl);
    CHECK(s.lastCmd == 0x20, "full ends at its activation (no deep sleep: partials need an awake controller)");
    int bad = 0;
    for (int ly = 0; ly < H; ly++)
      for (int lx = 0; lx < W; lx++) {
        int rx, ry;
        expected(rot, lx, ly, &rx, &ry);
        const int want = !frameBit(a, rb, lx, ly);   // ink 1 -> RAM 0
        bad += ramBit(s, 0, rx, ry) != want;
        bad += ramBit(s, 1, rx, ry) != want;
      }
    CHECK(bad == 0, "rot %d full: %d wrong bits", rot, bad);
    // partials: random windows, glass tracking, old == new afterwards, nothing outside the window
    std::vector<uint8_t> glass = a;
    for (int t = 0; t < 40; t++) {
      b = glass;
      int y0 = rand() % H, y1 = y0 + 1 + rand() % (H - y0);
      int xb0 = rand() % rb, xb1 = xb0 + 1 + rand() % (rb - xb0);
      for (int y = y0; y < y1; y++)
        for (int x = xb0; x < xb1; x++) b[(size_t) y * rb + x] = (uint8_t) rand();
      int rx0, rx1, ry0, ry1;
      d.ramWindow(xb0 * 8, xb1 * 8, y0, y1, &rx0, &rx1, &ry0, &ry1);
      s.winRx0 = rx0; s.winRx1 = rx1; s.winRy0 = ry0; s.winRy1 = ry1;
      s.wroteOutside = s.wroteOld = false;
      const int sw = s.swresets;
      d.stayPowered(t & 1);
      CHECK(d.startPartial(b.data(), glass.data(), y0, y1, false, xb0, xb1), "startPartial");
      run(s, d);
      // 2026-10-03: a repeat pass and the final sync rewrite both RAMs over the whole frame on purpose (old == new
      // outside the window, so nothing there is driven); the erase checks below still prove the window.
      CHECK(s.lastUpdate == 0xFF, "partial update byte");
      CHECK(s.swresets == sw, "partial sent SWRESET (Waveshare: hardware reset only)");
      CHECK(s.wroteOld, "rot %d partial %d: the old RAM was not kept equal to the glass", rot, t);
      CHECK(s.lastCmd != 0x10, "partial ends without deep sleep");
      CHECK(!d.powered(), "rails down after a partial");
      glass = b;
      // the old RAM still holds the base image: the controller, not the driver, owns it after a partial
      bad = 0;
      for (int ly = 0; ly < H; ly++)
        for (int lx = 0; lx < W; lx++) {
          int rx, ry;
          expected(rot, lx, ly, &rx, &ry);
          bad += ramBit(s, 0, rx, ry) != !frameBit(glass, rb, lx, ly);
        }
      CHECK(bad == 0, "rot %d partial %d: %d wrong bits", rot, t, bad);
      bad = 0;   // the GLASS model: every bit that changed was driven, both ways, across successive partials
      for (int ly = 0; ly < H; ly++)
        for (int lx = 0; lx < W; lx++) {
          int rx, ry;
          expected(rot, lx, ly, &rx, &ry);
          bad += ((s.glass[ry][rx / 8] >> (7 - (rx & 7))) & 1) != !frameBit(glass, rb, lx, ly);
        }
      CHECK(bad == 0, "rot %d partial %d: %d glass bits not driven", rot, t, bad);
    }
    if (d.powered()) { CHECK(d.powerDown(), "powerDown"); run(s, d); }  // never up now
    CHECK(!d.powered() && d.isIdle(), "rails down");
    CHECK(!s.forbidden, "rot %d: a register LUT / voltage / 0x21 command was sent (OTP only)", rot);
    // THE ERASE BUG (owner 2026-10-03): white page, partial draws a black block, a second partial takes it
    // back to white -- the second must drive those pixels (black -> white), not see white -> white.
    {
      std::vector<uint8_t> w((size_t) rb * H, 0), k = w;
      for (int y = 10; y < 30; y++) for (int x = 2; x < 6; x++) k[(size_t) y * rb + x] = 0xFF;
      CHECK(d.startFull(w.data()), "erase: full"); run(s, d);
      CHECK(d.startPartial(k.data(), w.data(), 10, 30, false, 2, 6), "erase: draw"); run(s, d);
      CHECK(d.startPartial(w.data(), k.data(), 10, 30, false, 2, 6), "erase: un-draw"); run(s, d);
      int left = 0;
      for (int ly = 0; ly < H; ly++)
        for (int lx = 0; lx < W; lx++) {
          int rx, ry;
          expected(rot, lx, ly, &rx, &ry);
          left += !((s.glass[ry][rx / 8] >> (7 - (rx & 7))) & 1);
        }
      CHECK(left == 0, "rot %d erase: %d black pixels left on the glass", rot, left);
    }
    // DEEP SLEEP between updates (battery mode): sleep(), then a partial wakes it with EPD_3IN97_Init first and
    // draws correctly; a second sleep() is refused while asleep; a partial without a sleep sends no SWRESET.
    {
      std::vector<uint8_t> w((size_t) rb * H, 0), k = w;
      for (int y = 40; y < 80; y++) for (int x = 1; x < 5; x++) k[(size_t) y * rb + x] = 0xF0;
      CHECK(d.startFull(w.data()), "sleep: full"); run(s, d);
      CHECK(d.sleep() && d.asleep() && s.asleep && s.lastCmd == 0x10, "sleep: 0x10 sent");
      CHECK(!d.sleep(), "sleep: refused when already asleep");
      const int sw = s.swresets, wk = (int) d.wakes();
      CHECK(d.startPartial(k.data(), w.data(), 40, 80, false, 1, 5), "sleep: partial accepted"); run(s, d);
      CHECK(d.isIdle() && !d.faulted() && !d.asleep() && !s.asleep, "sleep: woke and finished (faulted %d)", d.faulted());
      CHECK(s.swresets == sw + 1 && (int) d.wakes() == wk + 1, "sleep: one EPD_Init (SWRESET) for the wake");
      CHECK(!s.cmdWhileAsleep, "sleep: a command reached the sleeping controller before its reset");
      int bad = 0;
      for (int ly = 0; ly < H; ly++)
        for (int lx = 0; lx < W; lx++) {
          int rx, ry;
          expected(rot, lx, ly, &rx, &ry);
          bad += ((s.glass[ry][rx / 8] >> (7 - (rx & 7))) & 1) != !frameBit(k, rb, lx, ly);
        }
      CHECK(bad == 0, "rot %d sleep: %d wrong glass bits after the woken partial", rot, bad);
      const int sw2 = s.swresets;
      CHECK(d.startPartial(w.data(), k.data(), 40, 80, false, 1, 5), "awake partial"); run(s, d);
      CHECK(s.swresets == sw2 && !d.faulted(), "awake partial: no SWRESET");
      // a full out of deep sleep wakes through its own EPD_Init
      CHECK(d.sleep(), "sleep again"); CHECK(d.startFull(w.data()), "full from sleep"); run(s, d);
      CHECK(!d.faulted() && !d.asleep() && !s.asleep, "full from sleep: woke");
    }
    // the UC8179 bench interface is refused
    CHECK(!d.startFast(a.data(), a.data(), 0, 1, 0, 1, 4) && !d.startNeutral(a.data(), a.data(), 0, 1, 0, 1, 4),
          "bench refused");
    // a stuck BUSY is a fault, not a hang
    s.busyLeft = 1 << 30;
    d.startFull(a.data());
    for (int i = 0; i < 100000 && !d.isIdle(); i++) { s.now += 1; d.poll(); }
    CHECK(d.faulted(), "busy timeout faults");
    s.busyLeft = 0;
  }
  printf(fails ? "ssd1677_host: %d FAILED\n" : "ssd1677_host: all passed\n", fails);
  return fails ? 1 : 0;
}
