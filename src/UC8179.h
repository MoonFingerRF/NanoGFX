// ============================================================================
//  UC8179.h — non-blocking driver for 800x480 UC8179 e-paper (Waveshare 7.5" V2,
//  GoodDisplay GDEY075T7, Seeed reTerminal E1001), full and windowed partial refresh.
//
//  WHY NON-BLOCKING: an e-paper refresh is seconds of BUSY. A driver that waits in a
//  loop stalls everything else on the core for that long — buttons, network, the next
//  frame. Here every wait is a state: start*() queues a refresh, poll() advances it by
//  at most one step and returns at once, and isIdle() says when the glass is done.
//  The only blocking calls are begin() (reset + register setup, ~60 ms) and the SPI
//  data pushes (48 KB at 10-20 MHz, a few tens of ms).
//
//  TRANSPORT-AGNOSTIC: the chip is driven through a small Bus type the caller
//  supplies, so the same driver runs on Arduino SPI, ESP-IDF spi_master, ESPHome's
//  SPIDevice or a host simulator:
//
//    struct Bus {
//      void command(uint8_t c);                  // DC low, one byte
//      void data(const uint8_t *p, size_t n);    // DC high, n bytes
//      bool busy();                              // true while the panel is busy
//      void reset(bool high);                    // drive RST
//      void delayMs(uint32_t ms);                // only used by begin()
//      uint32_t millis();
//    };
//
//  FRAMES are 1 bit per pixel, MSB-first, 100 bytes per row, bit 1 = INK (the
//  PackMono default). The driver inverts where the controller wants white = 1.
//
//  REFRESH KINDS
//   - full:    the panel's own full waveform (the flashing one) — clears ghosting.
//              Register sequence as ESPHome's `7.50inv2`, which is the one measured to
//              draw on the E1001's glass.
//   - partial: the fast "only changed pixels" waveform on a row window [y0, y1).
//              Sequence as Waveshare's EPD_7IN5_V2 partial demo (0x50 A9/07, 0xE0 02,
//              0xE5 6E, 0x91, 0x90 window), with one addition: the PREVIOUS frame's
//              window is written to DTM1 (0x10) every time, so a partial straight after
//              a full refresh (which leaves DTM1 stale) still compares against what is
//              really on the glass. Needs the caller's previous frame.
//  Partial refreshes accumulate ghosting; call full every N partials (the caller's
//  policy — the driver counts them in partialsSinceFull()).
// ============================================================================
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <initializer_list>

template <class Bus>
class UC8179 {
public:
  static constexpr int W = 800, H = 480, ROW = W / 8;
  static constexpr uint32_t BUSY_TIMEOUT_MS = 12000;   // a full refresh is ~4 s; this is a fault

  enum class Step : uint8_t { IDLE, POWER_WAIT, REFRESH_WAIT, OFF_WAIT };
  // Where the last refresh spent its time, ms: rails up, SPI, waveform, rails down.
  struct Timing { uint16_t power, send, refresh, off; };

  explicit UC8179(Bus &bus) : bus_(bus) {}

  // Which way round this glass is. The controller's two data polarities (normal for full
  // refreshes, 0x50 DDX=01 for partial ones) are fixed, but panels and bring-up code disagree
  // about which bit is ink; measured on the reTerminal E1001 (camera, 2026-09-23): the
  // ESPHome `7.50inv2` convention draws it white-on-black, so that board sets invert(true).
  void invert(bool on) { invert_ = on; }

  // Hard reset + register setup. Blocking (~300 ms). Call once, and again after a timeout.
  // The controller is BUSY for a while after RST rises and ignores commands until it lets go,
  // so the reset is long (the Waveshare/ESPHome 200 ms) and every step waits for BUSY: a panel
  // that silently dropped its panel-setting and resolution writes still refreshes -- showing
  // whatever its RAM held before -- which looks exactly like "the data never arrived".
  void begin() {
    bus_.reset(true);  bus_.delayMs(20);
    bus_.reset(false); bus_.delayMs(200);
    bus_.reset(true);  bus_.delayMs(20);
    waitIdle(1000);
    cmd(0x01, {0x07, 0x07, 0x3F, 0x3F});   // power setting: VGH/VGL, VDH/VDL 15 V
    cmd(0x06, {0x17, 0x17, 0x28, 0x17});   // booster soft start
    cmd(0x00, {0x1F});                     // panel setting: KW mode, LUT from OTP
    cmd(0x61, {0x03, 0x20, 0x01, 0xE0});   // resolution 800 x 480
    cmd(0x15, {0x00});                     // dual SPI off
    cmd(0x60, {0x22});                     // TCON
    waitIdle(1000);
    step_ = Step::IDLE;
    faulted_ = false;
  }

  bool isIdle() const { return step_ == Step::IDLE; }
  Step step() const { return step_; }
  bool faulted() const { return faulted_; }
  uint32_t partialsSinceFull() const { return partials_; }
  uint32_t lastRefreshMs() const { return lastMs_; }   // duration of the last completed refresh
  const Timing &lastTiming() const { return timing_; }

  // STAY POWERED: skip the power-off after a refresh (and so the power-on before the next), for a
  // run of frequent updates -- a ticking track position. Rails up/down cost ~100-200 ms of every
  // refresh; the caller turns this off (and calls powerDown()) when the run ends, because the
  // controller's charge pumps should not sit on for hours.
  void stayPowered(bool on) { stay_ = on; }
  bool powered() const { return powered_; }
  // Take the rails down now if they were left up. Non-blocking (poll() finishes it).
  bool powerDown() {
    if (step_ != Step::IDLE || !powered_) return false;
    full_ = false; clean_ = false; count_ = false;
    cmd(0x02);
    enter(Step::OFF_WAIT, 0);
    return true;
  }

  // Queue a full refresh of `frame` (W*H/8 bytes). Returns false if a refresh is running.
  bool startFull(const uint8_t *frame) {
    if (step_ != Step::IDLE) return false;
    full_ = true; clean_ = false; count_ = true; frame_ = frame; prev_ = nullptr; y0_ = 0; y1_ = H;
    xb0_ = 0; xb1_ = ROW;
    cmd(0xE0, {0x00});                     // cascade off: the real temperature picks the LUT
    cmd(0x92);                             // leave partial mode, if we were in it
    cmd(0x50, {CDI_FULL, 0x07});           // VCOM/data interval; border driven (see CDI_*)
    powerOn();
    return true;
  }

  // Queue a partial refresh of rows [y0, y1) of `frame`, where `prev` holds what is on the
  // glass now (both full frames; only the window is sent). Returns false if busy.
  //
  // `clean` = the same window, driven with the panel's FULL waveform (the one the internal
  // temperature sensor picks) instead of the fast one: it flashes, but only inside the window,
  // and it clears the ghosting that fast partial updates leave behind in those rows. This is
  // what lets a caller clean one busy band (a ticking progress bar) without flashing the rest.
  //
  // `xb0`/`xb1` narrow the window to byte columns [xb0, xb1) (8 pixels each; the controller's
  // horizontal window is byte-aligned): a ticking progress bar sends and drives only itself.
  bool startPartial(const uint8_t *frame, const uint8_t *prev, int y0, int y1, bool clean = false,
                    int xb0 = 0, int xb1 = ROW) {
    if (step_ != Step::IDLE || !prev) return false;
    if (y0 < 0) y0 = 0;
    if (y1 > H) y1 = H;
    if (xb0 < 0) xb0 = 0;
    if (xb1 > ROW) xb1 = ROW;
    if (y0 >= y1 || xb0 >= xb1) return false;
    full_ = false; clean_ = clean; count_ = true; frame_ = frame; prev_ = prev; y0_ = y0; y1_ = y1;
    xb0_ = xb0; xb1_ = xb1;
    cmd(0x50, {CDI_PARTIAL, 0x07});        // data polarity 1 = ink, new->old copy, border driven
    if (clean) {
      cmd(0xE0, {0x00});                   // cascade off: the temperature-selected full LUT
    } else {
      cmd(0xE0, {0x02});                   // cascade on:
      cmd(0xE5, {0x6E});                   //   the fast partial waveform
    }
    powerOn();
    return true;
  }

  // Advance the running refresh by at most one step. Cheap when idle or still busy.
  void poll() {
    if (step_ == Step::IDLE) return;
    const uint32_t now = bus_.millis();
    if ((int32_t)(now - notBefore_) < 0) return;
    if (bus_.busy()) {
      if (now - stepStart_ > BUSY_TIMEOUT_MS) { faulted_ = true; step_ = Step::IDLE; return; }
      if (now - lastStatus_ >= 10) { cmd(0x71); lastStatus_ = now; }  // V2 wants status polls
      return;
    }
    switch (step_) {
      case Step::POWER_WAIT: {
        powered_ = true;
        timing_.power = (uint16_t)(now - started_);
        const uint32_t t0 = bus_.millis();
        sendAndRefresh();
        timing_.send = (uint16_t)(bus_.millis() - t0);
        enter(Step::REFRESH_WAIT, SETTLE_MS);
        break;
      }
      case Step::REFRESH_WAIT:
        timing_.refresh = (uint16_t)(now - stepStart_);
        if (!full_) cmd(0x92);             // out of partial mode
        if (stay_ && !full_) {             // a run of updates: leave the rails up
          timing_.off = 0;
          finish(now);
          break;
        }
        cmd(0x02);                         // power off (keeps registers and RAM)
        enter(Step::OFF_WAIT, 0);
        break;
      case Step::OFF_WAIT:
        powered_ = false;
        timing_.off = (uint16_t)(now - stepStart_);
        finish(now);
        break;
      default:
        step_ = Step::IDLE;
    }
  }

  // Deep sleep: lowest power; needs begin() to wake. Only when idle.
  void sleep() {
    if (step_ != Step::IDLE) return;
    cmd(0x02);
    cmd(0x07, {0xA5});
  }

private:
  Bus &bus_;
  Step step_ = Step::IDLE;
  static constexpr uint32_t SETTLE_MS = 5;   // BUSY takes a moment to assert after a command
  // CDI (0x50) byte 1: BDZ(7) BDV(5:4) N2OCP(3) DDX(1:0). The border -- the ring of glass
  // outside the 800x480 active area -- must be DRIVEN on every refresh: Waveshare's partial demo
  // floats it (BDZ=1, 0xA9), and on a panel whose rails are kept up between updates a floating
  // border drifts into a grey outline round the picture (seen on a reTerminal E1001, 2026-09-23).
  // Both refreshes now use the same data polarity (DDX=01) and the same border value (BDV=01).
  static constexpr uint8_t CDI_FULL = 0x11;      // BDZ=0 BDV=01 N2OCP=0 DDX=01
  static constexpr uint8_t CDI_PARTIAL = 0x19;   // BDZ=0 BDV=01 N2OCP=1 DDX=01
  bool full_ = true, clean_ = false, count_ = false, faulted_ = false, invert_ = false;
  bool stay_ = false, powered_ = false;
  int xb0_ = 0, xb1_ = ROW;
  Timing timing_{0, 0, 0, 0};
  const uint8_t *frame_ = nullptr, *prev_ = nullptr;
  int y0_ = 0, y1_ = H;
  uint32_t notBefore_ = 0, stepStart_ = 0, lastStatus_ = 0, started_ = 0, lastMs_ = 0, partials_ = 0;
  uint8_t line_[ROW * 10];

  void cmd(uint8_t c) { bus_.command(c); }
  bool waitIdle(uint32_t ms) {             // begin() only: the one place a block is acceptable
    for (uint32_t t = 0; bus_.busy(); t += 5) {
      if (t >= ms) return false;
      bus_.delayMs(5);
    }
    return true;
  }
  void cmd(uint8_t c, std::initializer_list<uint8_t> d) {
    bus_.command(c);
    uint8_t tmp[12];
    size_t n = 0;
    for (uint8_t b : d) if (n < sizeof tmp) tmp[n++] = b;
    bus_.data(tmp, n);
  }
  void finish(uint32_t now) {
    if (count_) {
      partials_ = full_ ? 0 : partials_ + (clean_ ? 0 : 1);
      lastMs_ = now - started_;
    }
    step_ = Step::IDLE;
  }
  void enter(Step s, uint32_t settleMs) {
    step_ = s;
    stepStart_ = bus_.millis();
    notBefore_ = stepStart_ + settleMs;
  }
  void powerOn() {
    started_ = bus_.millis();
    if (powered_) {                        // rails still up from the last update
      enter(Step::POWER_WAIT, 0);
      return;
    }
    cmd(0x04);                             // power on; BUSY until the rails are up
    enter(Step::POWER_WAIT, SETTLE_MS);
  }
  // The window's bytes, row after row, batched into as few bus writes as the buffer allows.
  void sendRows(uint8_t reg, const uint8_t *src, bool invert) {
    cmd(reg);
    const int n = xb1_ - xb0_;
    size_t fill = 0;
    for (int y = y0_; y < y1_; y++) {
      const uint8_t *row = src + (size_t)y * ROW + xb0_;
      if (fill + (size_t)n > sizeof line_) { bus_.data(line_, fill); fill = 0; }
      if (invert) for (int i = 0; i < n; i++) line_[fill + i] = (uint8_t)~row[i];
      else memcpy(line_ + fill, row, (size_t)n);
      fill += (size_t)n;
    }
    if (fill) bus_.data(line_, fill);
  }
  void sendAndRefresh() {
    if (full_) {
      sendRows(0x13, frame_, invert_);     // same data polarity as a partial (DDX=01 in both)
    } else {
      cmd(0x91);                           // partial in
      const uint16_t xa = (uint16_t)(xb0_ * 8), xe = (uint16_t)(xb1_ * 8 - 1);
      const uint16_t ya = (uint16_t)y0_, yb = (uint16_t)(y1_ - 1);
      cmd(0x90, {(uint8_t)(xa >> 8), (uint8_t)(xa & 0xFF), (uint8_t)(xe >> 8), (uint8_t)(xe & 0xFF),
                 (uint8_t)(ya >> 8), (uint8_t)(ya & 0xFF), (uint8_t)(yb >> 8), (uint8_t)(yb & 0xFF), 0x01});
      sendRows(0x10, prev_, invert_);      // what is on the glass (old)
      sendRows(0x13, frame_, invert_);     // what should be (new)
    }
    cmd(0x12);                             // display refresh
  }
};
