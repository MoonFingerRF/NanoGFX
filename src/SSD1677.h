// ============================================================================
//  SSD1677.h — non-blocking driver for 800x480 SSD1677 e-paper (Waveshare
//  ESP32-S3-ePaper-3.97 / its 3.97" glass, GoodDisplay GDEM0397T81), full and
//  windowed partial refresh, with the logical frame in any of four rotations.
//
//  SAME SHAPE AS UC8179.h, so a caller written for one drives the other: begin(),
//  startFull(), startPartial(frame, prev, y0, y1, clean, xb0, xb1), poll(), isIdle(),
//  stayPowered()/powerDown(), faulted(), lastTiming(). The UC8179's bench-only waveform
//  experiments (startFast / startNeutral / startPair / startOtpCopy / startNull) exist
//  here only as refusals (they return false and touch nothing): this controller runs
//  the panel's own OTP waveforms and nothing else.
//
//  WHERE THE REGISTER SEQUENCE COMES FROM (2026-10-02):
//    * Waveshare's demo for this board (github.com/waveshareteam/ESP32-S3-ePaper-3.97,
//      ESP-IDF/08_.../components/epaper_port/epaper_port.c): SWRESET 0x12, temperature
//      sensor 0x18=0x80, booster 0x0C {AE C7 C3 C0 80}, driver output 0x01 {479, 0x02},
//      full update 0x22=0xF7, partial update 0x22=0xFF, deep sleep 0x10=0x01.
//    * GxEPD2's GxEPD2_397_GDEM0397T81 (the same 3.97" 800x480 SSD1677 glass): display
//      update control 0x21 {0x40,0x00} for a full refresh (RED RAM bypassed as 0) and
//      {0x00,0x00} for a partial (RED RAM = the old picture), partial 0x22=0xFC with the
//      rails left up, power off 0x22=0x83, and -- the part that makes partials right --
//      writing the new picture into BOTH RAMs after a partial ("writeImageAgain").
//
//  HOW A PARTIAL WORKS ON THIS CONTROLLER, and why it differs from the UC8179: the
//  SSD1677 refreshes the WHOLE glass every time; "partial" is the OTP mode-2 waveform,
//  which drives a pixel only where the BW RAM (0x24, new) and the RED RAM (0x26, old)
//  differ. So the RAM window is only where data is written -- every pixel outside it is
//  driven by the same-state waveform, and must therefore have old == new. This driver
//  keeps that invariant: before the refresh it writes the caller's `prev` window to 0x26
//  and `frame` to 0x24; after BUSY drops it writes `frame` to 0x26 too. A full refresh
//  writes `frame` to both.
//
//  GLASS CARE (the 7.5" panel's greying lessons, satellite/components/nostalgia_panel):
//  only the vendor's balanced OTP waveforms are used (no register LUTs), the border is
//  driven by the LUT (0x3C = 0x01, never floated: a floating border drifts grey on a
//  panel whose rails stay up), the rails come down after a run of updates (powerDown()),
//  and the full refresh is the caller's to ration (EInkGhost), not a habit.
//
//  ROTATION: the controller's RAM is 800 sources (x) by 480 gates (y), and on this glass
//  the gates are reversed: an upright landscape row y lives at RAM row 479 - y (GxEPD2:
//  "gates are reversed on this display"). The caller's frame is LOGICAL -- landscape
//  800x480 or portrait 480x800 -- and the driver maps it while sending:
//      LANDSCAPE      rx = lx,        ry = 479 - ly
//      PORTRAIT       rx = ly,        ry = lx          (Waveshare's own UI: Paint ROTATE_270)
//      LANDSCAPE_180  rx = 799 - lx,  ry = ly
//      PORTRAIT_180   rx = 799 - ly,  ry = 479 - lx
//  Data entry mode is 0x03 (x and y increment) and every window is given in RAM
//  coordinates; a logical window that lands on a RAM x range not on a byte boundary is
//  widened to whole bytes (the rows are sent from the caller's full frame, so widening
//  only resends what is already there).
//
//  FRAMES are 1 bit per pixel, MSB-first, logical width / 8 bytes per row, bit 1 = INK
//  (the PackMono default). The controller's RAM is 1 = white, so ink is sent as 0;
//  invert(true) flips that for a glass that comes out negative.
//
//  Bus: the same small type UC8179.h takes (command, data, busy, reset, delayMs, millis).
//  BUSY is HIGH while the controller works.
// ============================================================================
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <initializer_list>

template <class Bus>
class SSD1677 {
public:
  static constexpr int PW = 800, PH = 480, PROW = PW / 8;   // the controller's RAM: sources x gates
  static constexpr uint32_t BUSY_TIMEOUT_MS = 15000;        // a full refresh is ~2-4 s; this is a fault
  enum Rotation : uint8_t { LANDSCAPE = 0, PORTRAIT = 1, LANDSCAPE_180 = 2, PORTRAIT_180 = 3 };
  enum class Step : uint8_t { IDLE, SEND, REFRESH_WAIT, OFF_WAIT };
  struct Timing { uint16_t power, send, refresh, off; };

  // Constants the UC8179 bench code names; nothing here uses them (see the header).
  static constexpr uint8_t FAST_VCOM_DC = 0x26, FAST_VCOM_MIN = 0x0A, FAST_CDI = 0x39, FAST_MAX_FRAMES = 60;
  static constexpr uint8_t NEUTRAL_MIN_FRAMES = 4, NEUTRAL_MAX_FRAMES = 30, PAIR_MAX_SHAKE = 10;

  explicit SSD1677(Bus &bus) : bus_(bus) {}

  void rotation(Rotation r) { rot_ = r; }
  Rotation rotation() const { return rot_; }
  int width() const { return (rot_ & 1) ? PH : PW; }    // logical
  int height() const { return (rot_ & 1) ? PW : PH; }
  int rowBytes() const { return width() / 8; }

  void invert(bool on) { invert_ = on; }

  // Hard reset + register setup. Blocking (~50 ms). Call once, and again after a fault.
  void begin() {
    initRegisters(false);
    step_ = Step::IDLE;
    faulted_ = false;
    powered_ = false;
  }

private:
  void hardReset() {
    bus_.reset(true);  bus_.delayMs(20);
    bus_.reset(false); bus_.delayMs(2);
    bus_.reset(true);  bus_.delayMs(20);
    waitIdle(1000);
  }
  // Waveshare's EPD_3IN97_Init, with data entry x+/y+ (our windows are in RAM coordinates).
  void initRegisters(bool) {
    hardReset();
    cmd(0x12);                                   // SWRESET
    bus_.delayMs(10);
    waitIdle(1000);
    cmd(0x18, {0x80});                           // internal temperature sensor
    cmd(0x0C, {0xAE, 0xC7, 0xC3, 0xC0, 0x80});   // booster soft start (vendor)
    cmd(0x01, {(uint8_t) ((PH - 1) & 0xFF), (uint8_t) ((PH - 1) >> 8), 0x02});   // 480 gates
    cmd(0x3C, {BORDER});                         // border follows the LUT: driven, never floated
    cmd(0x11, {0x03});                           // data entry: x+, y+ (windows in RAM coordinates)
    busyAfterBegin_ = !waitIdle(1000);
  }

public:

  bool isIdle() const { return step_ == Step::IDLE; }
  Step step() const { return step_; }
  bool faulted() const { return faulted_; }
  bool controllerBusy() { return bus_.busy(); }
  uint32_t partialsSinceFull() const { return partials_; }
  uint32_t lastRefreshMs() const { return lastMs_; }
  const Timing &lastTiming() const { return timing_; }
  uint32_t railCycles() const { return railCycles_; }
  // Whether BUSY was seen high during the last refresh: a refresh that never raised BUSY did not run
  // (a controller asleep or not answering looks exactly like a very fast one otherwise).
  bool sawBusy() const { return sawBusy_; }
  // BUSY right after the last begin()'s SWRESET wait (true = still busy: the wait timed out).
  bool busyAfterBegin() const { return busyAfterBegin_; }

  void stayPowered(bool on) { stay_ = on; }
  bool powered() const { return powered_; }
  // Take the rails down now if a partial left them up. Non-blocking (poll() finishes it).
  bool powerDown() {
    if (step_ != Step::IDLE || !powered_) return false;
    count_ = false;
    cmd(0x22, {0x83});                           // disable analog, disable OSC
    cmd(0x20);
    enter(Step::OFF_WAIT, SETTLE_MS);
    return true;
  }

  // Queue a full refresh of the logical `frame`. Returns false if a refresh is running.
  bool startFull(const uint8_t *frame) {
    if (step_ != Step::IDLE || !frame) return false;
    full_ = true; count_ = true; frame_ = frame; prev_ = nullptr;
    lx0_ = 0; lx1_ = width(); ly0_ = 0; ly1_ = height();
    started_ = bus_.millis();
    enter(Step::SEND, 0);
    return true;
  }

  // Queue a partial refresh of logical rows [y0, y1), byte columns [xb0, xb1) of `frame`, where
  // `prev` is what is on the glass now (both whole logical frames). `clean` asks for the full
  // waveform in that window: this controller has no windowed full drive (every update covers the
  // whole glass), so a clean is a full refresh of the whole frame.
  bool startPartial(const uint8_t *frame, const uint8_t *prev, int y0, int y1, bool clean = false,
                    int xb0 = 0, int xb1 = 1 << 14) {
    if (step_ != Step::IDLE || !frame || !prev) return false;
    if (clean) return startFull(frame);
    const int rb = rowBytes();
    if (y0 < 0) y0 = 0;
    if (y1 > height()) y1 = height();
    if (xb0 < 0) xb0 = 0;
    if (xb1 > rb) xb1 = rb;
    if (y0 >= y1 || xb0 >= xb1) return false;
    full_ = false; count_ = true; frame_ = frame; prev_ = prev;
    lx0_ = xb0 * 8; lx1_ = xb1 * 8; ly0_ = y0; ly1_ = y1;
    started_ = bus_.millis();
    enter(Step::SEND, 0);
    return true;
  }

  // BENCH: set the old (RED) RAM to `glass` without a refresh.
  bool syncOld(const uint8_t *glass) {
    if (step_ != Step::IDLE || faulted_ || !glass || bus_.busy()) return false;
    lx0_ = 0; lx1_ = width(); ly0_ = 0; ly1_ = height();
    sendWindow(0x26, glass);
    return true;
  }

  // Advance the running refresh by at most one step. Cheap when idle or still busy.
  void poll() {
    if (step_ == Step::IDLE) return;
    const uint32_t now = bus_.millis();
    if ((int32_t) (now - notBefore_) < 0) return;
    if (bus_.busy()) {
      if (step_ == Step::REFRESH_WAIT) sawBusy_ = true;
      if (now - stepStart_ > BUSY_TIMEOUT_MS) {
        faulted_ = true;
        step_ = Step::IDLE;
      }
      return;
    }
    switch (step_) {
      case Step::SEND: {
        const uint32_t t0 = bus_.millis();
        timing_.power = 0;
        if (full_) {
          initRegisters(true);                   // reset + SWRESET + the vendor's init (EPD_3IN97_Init)
          sendWindow(0x26, frame_);              // old := new, so the next partial compares to it
          sendWindow(0x24, frame_);
          cmd(0x21, {0x40, 0x00});               // full: RED RAM bypassed as 0
          cmd(0x22, {0xF7});                     // clock+analog on, temp, LUT mode 1, display, off
          if (!powered_) railCycles_++;
        } else {
          // MEASURED 2026-10-02 (webcam): a mode-2 update issued on the full-refresh register set (0x01
          // interlaced 480 gates, 0x21) ran its 0.5 s waveform and changed NOTHING on this glass, while
          // Waveshare's EPD_3IN97_Display_Partial -- hardware reset (registers back to their power-on
          // values, RAM kept), 0x18 0x80, 0x3C 0x80, window, 0x24, 0x22 0xFF, 0x20 -- drew. So a partial
          // is exactly that sequence.
          hardReset();
          cmd(0x18, {0x80});
          cmd(0x3C, {0x80});
          cmd(0x11, {0x03});
          sendWindow(0x26, prev_);               // what is on the glass
          sendWindow(0x24, frame_);              // what should be
          cmd(0x22, {0xFF});                     // LUT mode 2, then analog and OSC off
          if (!powered_) railCycles_++;
        }
        cmd(0x20);                               // master activation
        sawBusy_ = false;
        timing_.send = (uint16_t) (bus_.millis() - t0);
        enter(Step::REFRESH_WAIT, SETTLE_MS);
        break;
      }
      case Step::REFRESH_WAIT:
        timing_.refresh = (uint16_t) (now - stepStart_);
        timing_.off = 0;
        powered_ = false;                        // both sequences end with the analog and OSC off
        if (!full_) {
          const uint32_t t0 = bus_.millis();
          sendWindow(0x26, frame_);              // old := new in the window: old == new everywhere
          timing_.send = (uint16_t) (timing_.send + (bus_.millis() - t0));
        }
        finish(now);
        break;
      case Step::OFF_WAIT:
        powered_ = false;
        timing_.off = (uint16_t) (now - stepStart_);
        step_ = Step::IDLE;
        break;
      default:
        step_ = Step::IDLE;
    }
  }

  // Deep sleep: lowest power; needs begin() to wake. Only when idle.
  void sleep() {
    if (step_ != Step::IDLE) return;
    cmd(0x10, {0x01});
    powered_ = false;
  }

  // ---- the UC8179's bench interface, refused (see the header) ---------------------------------
  bool startFast(const uint8_t *, const uint8_t *, int, int, int, int, uint8_t) { return false; }
  bool startNull(const uint8_t *, int, int, int, int, uint8_t) { return false; }
  bool startNeutral(const uint8_t *, const uint8_t *, int, int, int, int, uint8_t) { return false; }
  bool startPair(const uint8_t *, const uint8_t *, int, int, int, int, uint8_t, uint8_t = 0) { return false; }
  bool startOtpCopy(const uint8_t *, const uint8_t *, int, int, int, int, const uint8_t *, const uint8_t *,
                    const uint8_t *, const uint8_t *, const uint8_t *, uint8_t, uint8_t = 0x19, uint8_t = 0) {
    return false;
  }
  bool powerUpWithoutRefresh() { return false; }
  void leaveFast() {}
  void offBeforeReset(bool) {}
  void fastVcom(uint8_t, uint8_t = 0) {}
  uint8_t fastVcomCode() const { return FAST_VCOM_MIN; }
  uint8_t fastVcomTable() const { return 0; }
  void factoryVcom(int) {}
  void neutralVcom(uint8_t) {}
  uint8_t neutralVcomCode() const { return FAST_VCOM_MIN; }
  bool fastActive() const { return false; }
  bool neutralActive() const { return false; }
  bool pairActive() const { return false; }
  bool copyActive() const { return false; }
  uint32_t neutralCount() const { return 0; }
  uint32_t pairCount() const { return 0; }
  uint32_t copyCount() const { return 0; }
  void partialScanOutside(bool on) { scanOutside_ = on; }
  bool partialScanOutside() const { return scanOutside_; }
  void partialTemperature(uint8_t t) { partialTemp_ = t; }
  uint8_t partialTemperature() const { return partialTemp_; }

  // ---- the mapping, public for the host test ----------------------------------------------------
  // RAM (rx, ry) -> logical (lx, ly).
  void ramToLogical(int rx, int ry, int *lx, int *ly) const {
    switch (rot_) {
      case PORTRAIT:      *lx = ry;            *ly = rx;            break;
      case LANDSCAPE_180: *lx = PW - 1 - rx;   *ly = ry;            break;
      case PORTRAIT_180:  *lx = PH - 1 - ry;   *ly = PW - 1 - rx;   break;
      default:            *lx = rx;            *ly = PH - 1 - ry;   break;
    }
  }
  // The RAM window that covers logical [lx0, lx1) x [ly0, ly1): x in whole bytes.
  void ramWindow(int lx0, int lx1, int ly0, int ly1, int *rx0, int *rx1, int *ry0, int *ry1) const {
    int a0, a1, b0, b1;   // RAM x range, RAM y range (half-open)
    switch (rot_) {
      case PORTRAIT:      a0 = ly0;           a1 = ly1;           b0 = lx0;           b1 = lx1;           break;
      case LANDSCAPE_180: a0 = PW - lx1;      a1 = PW - lx0;      b0 = ly0;           b1 = ly1;           break;
      case PORTRAIT_180:  a0 = PW - ly1;      a1 = PW - ly0;      b0 = PH - lx1;      b1 = PH - lx0;      break;
      default:            a0 = lx0;           a1 = lx1;           b0 = PH - ly1;      b1 = PH - ly0;      break;
    }
    *rx0 = a0 & ~7;
    *rx1 = (a1 + 7) & ~7;
    *ry0 = b0;
    *ry1 = b1;
  }
  // One RAM row's bytes [rx0, rx1) from a logical frame, in the controller's polarity.
  void ramRow(const uint8_t *src, int ry, int rx0, int rx1, uint8_t *out) const {
    const int rb = rowBytes();
    const uint8_t flip = invert_ ? 0x00 : 0xFF;     // logical ink 1 -> RAM 0 (black)
    if (rot_ == LANDSCAPE) {                        // a RAM row is a logical row: a copy
      const uint8_t *row = src + (size_t) (PH - 1 - ry) * rb + rx0 / 8;
      for (int i = 0; i < (rx1 - rx0) / 8; i++) out[i] = (uint8_t) (row[i] ^ flip);
      return;
    }
    for (int rx = rx0, i = 0; rx < rx1; rx += 8, i++) {
      uint8_t v = 0;
      for (int k = 0; k < 8; k++) {
        int lx, ly;
        ramToLogical(rx + k, ry, &lx, &ly);
        v = (uint8_t) ((v << 1) | ((src[(size_t) ly * rb + (lx >> 3)] >> (7 - (lx & 7))) & 1));
      }
      out[i] = (uint8_t) (v ^ flip);
    }
  }

private:
  Bus &bus_;
  Step step_ = Step::IDLE;
  static constexpr uint32_t SETTLE_MS = 5;      // BUSY takes a moment to assert after a command
  static constexpr uint8_t BORDER = 0x01;       // 0x3C: border waveform = LUT1 (driven)
  Rotation rot_ = LANDSCAPE;
  bool full_ = true, count_ = false, faulted_ = false, invert_ = false;
  bool stay_ = false, powered_ = false, scanOutside_ = true, sawBusy_ = false, busyAfterBegin_ = false;
  uint8_t partialTemp_ = 0x6E;
  Timing timing_{0, 0, 0, 0};
  const uint8_t *frame_ = nullptr, *prev_ = nullptr;
  int lx0_ = 0, lx1_ = PW, ly0_ = 0, ly1_ = PH;
  uint32_t notBefore_ = 0, stepStart_ = 0, started_ = 0, lastMs_ = 0, partials_ = 0, railCycles_ = 0;
  uint8_t line_[PROW * 10];

  void cmd(uint8_t c) { bus_.command(c); }
  void cmd(uint8_t c, std::initializer_list<uint8_t> d) {
    bus_.command(c);
    uint8_t tmp[8];
    size_t n = 0;
    for (uint8_t b : d) if (n < sizeof tmp) tmp[n++] = b;
    bus_.data(tmp, n);
  }
  bool waitIdle(uint32_t ms) {                  // begin() only: the one place a block is acceptable
    for (uint32_t t = 0; bus_.busy(); t += 5) {
      if (t >= ms) return false;
      bus_.delayMs(5);
    }
    return true;
  }
  void finish(uint32_t now) {
    if (count_) {
      partials_ = full_ ? 0 : partials_ + 1;
      lastMs_ = now - started_;
    }
    step_ = Step::IDLE;
  }
  void enter(Step s, uint32_t settleMs) {
    step_ = s;
    stepStart_ = bus_.millis();
    notBefore_ = stepStart_ + settleMs;
  }
  // Write the current logical window of `src` into RAM `reg` (0x24 new / 0x26 old).
  void sendWindow(uint8_t reg, const uint8_t *src) {
    int rx0, rx1, ry0, ry1;
    ramWindow(lx0_, lx1_, ly0_, ly1_, &rx0, &rx1, &ry0, &ry1);
    cmd(0x44, {(uint8_t) (rx0 & 0xFF), (uint8_t) (rx0 >> 8), (uint8_t) ((rx1 - 1) & 0xFF), (uint8_t) ((rx1 - 1) >> 8)});
    cmd(0x45, {(uint8_t) (ry0 & 0xFF), (uint8_t) (ry0 >> 8), (uint8_t) ((ry1 - 1) & 0xFF), (uint8_t) ((ry1 - 1) >> 8)});
    cmd(0x4E, {(uint8_t) (rx0 & 0xFF), (uint8_t) (rx0 >> 8)});
    cmd(0x4F, {(uint8_t) (ry0 & 0xFF), (uint8_t) (ry0 >> 8)});
    cmd(reg);
    const size_t n = (size_t) (rx1 - rx0) / 8;
    size_t fill = 0;
    for (int ry = ry0; ry < ry1; ry++) {
      if (fill + n > sizeof line_) { bus_.data(line_, fill); fill = 0; }
      ramRow(src, ry, rx0, rx1, line_ + fill);
      fill += n;
    }
    if (fill) bus_.data(line_, fill);
  }
};
