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

  // Queue a full refresh of `frame` (W*H/8 bytes). Returns false if a refresh is running.
  bool startFull(const uint8_t *frame) {
    if (step_ != Step::IDLE) return false;
    full_ = true; clean_ = false; frame_ = frame; prev_ = nullptr; y0_ = 0; y1_ = H;
    cmd(0xE0, {0x00});                     // cascade off: the real temperature picks the LUT
    cmd(0x92);                             // leave partial mode, if we were in it
    cmd(0x50, {0x10, 0x07});               // VCOM/data interval: normal polarity (1 = white)
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
  bool startPartial(const uint8_t *frame, const uint8_t *prev, int y0, int y1, bool clean = false) {
    if (step_ != Step::IDLE || !prev) return false;
    if (y0 < 0) y0 = 0;
    if (y1 > H) y1 = H;
    if (y0 >= y1) return false;
    full_ = false; clean_ = clean; frame_ = frame; prev_ = prev; y0_ = y0; y1_ = y1;
    cmd(0x50, {0xA9, 0x07});               // partial polarity (1 = ink), new->old copy
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
      case Step::POWER_WAIT:
        sendAndRefresh();
        enter(Step::REFRESH_WAIT, 100);
        break;
      case Step::REFRESH_WAIT:
        if (!full_) cmd(0x92);             // out of partial mode before powering down
        cmd(0x02);                         // power off (keeps registers and RAM)
        enter(Step::OFF_WAIT, 0);
        break;
      case Step::OFF_WAIT:
        partials_ = full_ ? 0 : partials_ + (clean_ ? 0 : 1);
        lastMs_ = now - started_;
        step_ = Step::IDLE;
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
  bool full_ = true, clean_ = false, faulted_ = false, invert_ = false;
  const uint8_t *frame_ = nullptr, *prev_ = nullptr;
  int y0_ = 0, y1_ = H;
  uint32_t notBefore_ = 0, stepStart_ = 0, lastStatus_ = 0, started_ = 0, lastMs_ = 0, partials_ = 0;
  uint8_t line_[ROW];

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
  void enter(Step s, uint32_t settleMs) {
    step_ = s;
    stepStart_ = bus_.millis();
    notBefore_ = stepStart_ + settleMs;
  }
  void powerOn() {
    started_ = bus_.millis();
    cmd(0x04);                             // power on; BUSY until the rails are up
    enter(Step::POWER_WAIT, 100);
  }
  void sendRows(uint8_t reg, const uint8_t *src, bool invert) {
    cmd(reg);
    for (int y = y0_; y < y1_; y++) {
      const uint8_t *row = src + (size_t)y * ROW;
      if (invert) {
        for (int i = 0; i < ROW; i++) line_[i] = (uint8_t)~row[i];
        bus_.data(line_, ROW);
      } else {
        bus_.data(row, ROW);
      }
    }
  }
  void sendAndRefresh() {
    if (full_) {
      sendRows(0x13, frame_, !invert_);    // normal polarity: 1 = white
    } else {
      cmd(0x91);                           // partial in
      const uint16_t x1 = W - 1, ya = (uint16_t)y0_, yb = (uint16_t)(y1_ - 1);
      cmd(0x90, {0x00, 0x00, (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xFF), (uint8_t)(ya >> 8),
                 (uint8_t)(ya & 0xFF), (uint8_t)(yb >> 8), (uint8_t)(yb & 0xFF), 0x01});
      sendRows(0x10, prev_, invert_);      // what is on the glass (old)
      sendRows(0x13, frame_, invert_);     // what should be (new)
    }
    cmd(0x12);                             // display refresh
  }
};
