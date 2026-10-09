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
//  PackMono default). Both refreshes send the bits with one data polarity (CDI DDX=01);
//  a glass that comes out negative (the reTerminal E1001 does) sets invert(true).
//
//  REFRESH KINDS
//   - full:    the panel's own full waveform (the flashing one) — clears ghosting.
//              Register sequence as ESPHome's `7.50inv2`, which is the one measured to
//              draw on the E1001's glass.
//   - partial: the fast "only changed pixels" waveform on a window: rows [y0, y1),
//              byte columns [xb0, xb1). Sequence as Waveshare's EPD_7IN5_V2 partial demo
//              (0xE0 02, 0xE5 6E, 0x91, 0x90 window) except that the border is driven, not
//              floated (see CDI_*), with one addition: the PREVIOUS frame's
//              window is written to DTM1 (0x10) every time, so a partial straight after
//              a full refresh (which leaves DTM1 stale) still compares against what is
//              really on the glass. Needs the caller's previous frame.
//   - clean:   the full waveform inside a partial window: only that window flashes.
//   - neutral: (bench) a register-LUT partial with zero field on unchanged pixels and the border,
//              a DC-balanced drive for each transition and one constant VCOM (startNeutral).
//  Partial refreshes accumulate ghosting. The policy is the caller's: EInkGhost.h keeps
//  per-row wear and says PARTIAL / CLEAN / FULL; the driver counts partialsSinceFull().
//
//  MEMORY: the driver holds pointers to the caller's frames (48 000 bytes each) and a
//  small line buffer for batching SPI writes. On ESP32 with frames in PSRAM, make sure
//  the Bus stages data through internal DMA-capable RAM: a DMA transfer straight out of
//  PSRAM that the SPI driver does not support reaches the controller as nothing.
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

  enum class Step : uint8_t { IDLE, POWER_WAIT, REFRESH_WAIT, OFF_WAIT, IDLE_POWER_WAIT, IDLE_OFF_WAIT };
  // Where the last refresh spent its time, ms: rails up, SPI, waveform, rails down.
  struct Timing { uint16_t power, send, refresh, off; };

  explicit UC8179(Bus &bus) : bus_(bus) {}

  // Which way round this glass is. Panels and bring-up code disagree about which bit is ink;
  // measured on the reTerminal E1001 (camera, 2026-09-23): frames sent as they are come out
  // white-on-black, so that board sets invert(true). Applies to full and partial alike.
  void invert(bool on) { invert_ = on; }

  // The forced temperature for partial refreshes (0xE5 with cascade 0xE0 = 02). It selects which of
  // the panel's OWN factory (OTP) waveforms a partial uses: 0x6E is the vendor's fast partial (the
  // default); 0x5A its fast full; others are undocumented bins. No custom LUT is ever loaded here.
  void partialTemperature(uint8_t t) { partialTemp_ = t; }
  uint8_t partialTemperature() const { return partialTemp_; }

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
    powered_ = false;                      // the reset took the rails down
    regLut_ = false;                       // and every register is the vendor's again
    neutral_ = false;
  }

  // S4 exit: back to the vendor's registers after a fast (register-LUT) refresh. With the rails
  // still up (stayPowered), a bare reset drops them WITHOUT the controller's power-off sequence,
  // and every such cut leaves a little DC on the whole glass at the fast VCOM (measured: fast
  // frames between factory partials greyed untouched glass by +34 levels in 30 min). So power off
  // properly first (0x02, BUSY), then reset. offBeforeReset(false) restores the old bare reset,
  // for the bench's A/B only.
  void offBeforeReset(bool on) { offBeforeReset_ = on; }
  void leaveFast() {
    if (powered_ && offBeforeReset_) {
      cmd(0x02);
      waitIdle(1000);
      powered_ = false;
    }
    begin();
  }

  bool isIdle() const { return step_ == Step::IDLE; }
  Step step() const { return step_; }
  bool faulted() const { return faulted_; }
  // BENCH ONLY: enable factory drive without writing pixels or starting a waveform.
  bool powerUpWithoutRefresh() {
    if (!isIdle() || faulted_ || regLut_ || bus_.busy()) return false;
    if (powered_) return true;
    railCycles_++;
    cmd(0x04);
    enter(Step::IDLE_POWER_WAIT, SETTLE_MS);
    return true;
  }
  bool controllerBusy() { return bus_.busy(); }  // physical BUSY pin, independent of software step
  uint32_t partialsSinceFull() const { return partials_; }
  uint32_t lastRefreshMs() const { return lastMs_; }   // duration of the last completed refresh
  const Timing &lastTiming() const { return timing_; }

  // BENCH: synchronize OLD SRAM to the completed physical frame, without a refresh.
  // Established writeImageToPrevious sequence: partial in, full RAM window, DTM1, partial out.
  // Refuse registered LUTs instead of resetting them; this operation changes no power/voltage.
  bool syncOld(const uint8_t *glass) {
    if (step_ != Step::IDLE || faulted_ || regLut_ || !glass || bus_.busy()) return false;
    const int saveY0 = y0_, saveY1 = y1_, saveX0 = xb0_, saveX1 = xb1_;
    y0_ = 0; y1_ = H; xb0_ = 0; xb1_ = ROW;
    cmd(0x91);
    cmd(0x90, {0x00, 0x00, 0x03, 0x1F, 0x00, 0x00, 0x01, 0xDF,
               (uint8_t)(scanOutside_ ? 0x01 : 0x00)});
    sendRows(0x10, glass, invert_);
    cmd(0x92);  // no 0x11 Data Stop or 0x12 refresh
    y0_ = saveY0; y1_ = saveY1; xb0_ = saveX0; xb1_ = saveX1;
    return true;
  }

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
    if (regLut_) leaveFast();              // S4: an OTP refresh starts from the vendor's registers
    full_ = true; clean_ = false; count_ = true; frame_ = frame; prev_ = nullptr; y0_ = 0; y1_ = H;
    xb0_ = 0; xb1_ = ROW;
    cmd(0xE0, {0x00});                     // cascade off: the real temperature picks the LUT
    cmd(0x92);                             // leave partial mode, if we were in it
    cmd(0x50, {CDI_FULL, 0x07});           // VCOM/data interval; border driven (see CDI_*)
    if (factoryVcom_ >= 0) cmd(0x82, {(uint8_t) factoryVcom_});   // bench only (factoryVcom)
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
    if (regLut_) leaveFast();              // S4: an OTP refresh starts from the vendor's registers
    full_ = false; clean_ = clean; count_ = true; frame_ = frame; prev_ = prev; y0_ = y0; y1_ = y1;
    xb0_ = xb0; xb1_ = xb1;
    cmd(0x50, {CDI_PARTIAL, 0x07});        // data polarity 1 = ink, new->old copy, border driven
    if (factoryVcom_ >= 0) cmd(0x82, {(uint8_t) factoryVcom_});   // bench only (factoryVcom)
    if (clean) {
      cmd(0xE0, {0x00});                   // cascade off: the temperature-selected full LUT
    } else {
      cmd(0xE0, {0x02});                   // cascade on: a forced temperature selects one of the
      cmd(0xE5, {partialTemp_});           //   panel's own OTP waveforms (0x6E: the fast partial)
    }
    powerOn();
    return true;
  }

  // FAST: a 1-bit waveform from register LUTs on a window (research E2+, owner-approved
  // 2026-09-23). Verified against the UC8179c datasheet (rev C0.6): PSR 0x00 bit 5 = REG (LUT from
  // register), bit 4 = KW mode; LUTC 0x20, LUTKW 0x22, LUTWK 0x23, LUTKK 0x24 are 60 bytes (10 groups
  // of 6: level byte, 4 frame counts, repeat), LUTWW 0x21 and LUTBD 0x25 42 bytes (7 groups); KW
  // mode uses 7 groups. Source levels 00 GND, 01 VDH, 10 VDL, 11 VDHR (never used). VDCS 0x82 0x26
  // = -2.00 V. With DDX=01, {NEW,OLD} = 01 -> LUTWK, 10 -> LUTKW, 00 -> LUTKK, 11 -> LUTWW; CDI
  // BDV=11 sends the border to LUTBD.
  // Safety (research S1-S4): unchanged pixels (WW, KK) and the border (BD) are never driven; KW and
  // WK are mirror images (one phase of `frames` at VDL and VDH); VCOM_DC is set explicitly; and the
  // next OTP refresh of any kind begins with a hardware reset + begin() (vendor registers).
  static constexpr uint8_t FAST_VCOM_DC = 0x26;           // -2.00 V (default; see fastVcom)
  static constexpr uint8_t FAST_VCOM_MIN = 0x0A;          // -0.60 V: the least negative allowed
  static constexpr uint8_t FAST_CDI = 0x39;               // BDZ=0 BDV=11 (LUTBD) N2OCP=1 DDX=01
  static constexpr uint8_t LVL_TO_WHITE = 0x80;           // phase 0 = 10b: VDL (K->W)
  static constexpr uint8_t LVL_TO_BLACK = 0x40;           // phase 0 = 01b: VDH (W->K)
  static_assert((LVL_TO_WHITE & 0xC0) != 0xC0 && (LVL_TO_BLACK & 0xC0) != 0xC0, "never VDHR (11b)");
  static_assert((LVL_TO_WHITE >> 6) + (LVL_TO_BLACK >> 6) == 3 && LVL_TO_WHITE != LVL_TO_BLACK,
                "KW and WK must be mirror images: one VDL, one VDH");
  static constexpr uint8_t FAST_MAX_FRAMES = 60;

  // A table: group 0 = one phase of `frames` at `level`, repeated once; every other group zero.
  static void fastLut(uint8_t *out, size_t len, uint8_t level, uint8_t frames) {
    memset(out, 0, len);
    if (!frames) return;
    out[0] = level; out[1] = frames; out[5] = 1;
  }

  bool startFast(const uint8_t *frame, const uint8_t *prev, int y0, int y1, int xb0, int xb1, uint8_t frames) {
    if (step_ != Step::IDLE || !prev || frames == 0 || frames > FAST_MAX_FRAMES) return false;
    if (y0 < 0) y0 = 0;
    if (y1 > H) y1 = H;
    if (xb0 < 0) xb0 = 0;
    if (xb1 > ROW) xb1 = ROW;
    if (y0 >= y1 || xb0 >= xb1) return false;
    full_ = false; clean_ = false; count_ = true; frame_ = frame; prev_ = prev; y0_ = y0; y1_ = y1;
    xb0_ = xb0; xb1_ = xb1;
    uint8_t t60[60], t42[42];
    cmd(0x00, {0x3F});                     // PSR: LUT from register, KW mode (rest as vendor 0x1F)
    cmd(0x82, {fastVcom_});                // S3: VCOM_DC explicit
    cmd(0x50, {FAST_CDI, 0x07});           // border -> LUTBD (all zero: not driven)
    cmd(0xE0, {0x00});                     // no forced temperature
    fastLut(t60, 60, 0x00, fastVcomTable_ ? 0 : frames); cmdData(0x20, t60, 60);   // VCOM (see fastVcom)
    fastLut(t42, 42, 0x00, 0);      cmdData(0x21, t42, 42);    // WW: never driven (S1)
    fastLut(t60, 60, LVL_TO_WHITE, frames); cmdData(0x22, t60, 60);   // KW: VDL
    fastLut(t60, 60, LVL_TO_BLACK, frames); cmdData(0x23, t60, 60);   // WK: VDH (mirror, S2)
    fastLut(t60, 60, 0x00, 0);      cmdData(0x24, t60, 60);    // KK: never driven (S1)
    fastLut(t42, 42, 0x00, 0);      cmdData(0x25, t42, 42);    // BD: never driven (S1)
    regLut_ = true;
    neutral_ = false;
    powerOn();
    return true;
  }
  // NULL refresh (research 2026-09-23 F3/X3): a register-LUT refresh that drives no pixel -- every
  // source LUT zero (GND), the VCOM LUT at VCOM_DC (level 00) for `frames` -- over the window of
  // `glass` (old = new = what is on the glass). The panel scans every row (PT_SCAN=1), so every
  // pixel's storage is rewritten at GND against a DRIVEN common electrode: a reference reset
  // between the factory and the fast path. S1/S2 hold trivially (nothing is driven).
  bool startNull(const uint8_t *glass, int y0, int y1, int xb0, int xb1, uint8_t frames) {
    if (step_ != Step::IDLE || !glass || frames == 0 || frames > FAST_MAX_FRAMES) return false;
    if (y0 < 0) y0 = 0;
    if (y1 > H) y1 = H;
    if (xb0 < 0) xb0 = 0;
    if (xb1 > ROW) xb1 = ROW;
    if (y0 >= y1 || xb0 >= xb1) return false;
    full_ = false; clean_ = false; count_ = true; frame_ = glass; prev_ = glass; y0_ = y0; y1_ = y1;
    xb0_ = xb0; xb1_ = xb1;
    uint8_t t60[60], t42[42];
    cmd(0x00, {0x3F});
    cmd(0x82, {fastVcom_});
    cmd(0x50, {FAST_CDI, 0x07});
    cmd(0xE0, {0x00});
    fastLut(t60, 60, 0x00, frames); cmdData(0x20, t60, 60);    // VCOM_DC, driven, `frames`
    fastLut(t42, 42, 0x00, 0);      cmdData(0x21, t42, 42);
    fastLut(t60, 60, 0x00, 0);      cmdData(0x22, t60, 60);
    fastLut(t60, 60, 0x00, 0);      cmdData(0x23, t60, 60);
    fastLut(t60, 60, 0x00, 0);      cmdData(0x24, t60, 60);
    fastLut(t42, 42, 0x00, 0);      cmdData(0x25, t42, 42);
    regLut_ = true;
    neutral_ = false;
    powerOn();
    return true;
  }
  // NEUTRAL partial (2026-10-01, the global-haze work; BENCH-SELECTED, never a default). A register
  // LUT partial for EVERY routine update, so no partial ever runs the undocumented OTP partial bin
  // (forced temperature 0x6E), whose untouched-pixel drive, VCOM phases, XON all-gate-on states,
  // VCOM_DC and rail voltages come from the OTP TR and cannot be seen or changed (UC8179c C0.6
  // "LUT format in OTP" p.50: each TR carries its own frame rate, VG/VDH/VDL, XON, VCOM_DC).
  //   N1  Unchanged pixels and the border are held at zero field: WW = KK = 00 (GND) and LUTBD = 00
  //       (VCOM) for every frame of the refresh -- same timing as the drive, so nothing ends early.
  //   N2  Every transition is DC-balanced on its own: K->W = VDH x n, then VDL x n; W->K = VDL x n,
  //       then VDH x n. The first phase pushes the pixel further into the state it is already in
  //       (no visible change), the second drives it across; the volt-frames sum to zero for each
  //       transition, not only over a K->W->K cycle as the FAST LUT's single phase does.
  //   N3  VCOM is one constant level for the whole drive (LUTC 00 = VCOM_DC, never 01/10 AC, never
  //       11 floating), at VDCS neutralVcom() -- by default 0x0A (-0.60 V), the value production
  //       already runs on the FAST path. Never more negative than that (the measured drift on
  //       untouched glass grew with |VCOM|: research 2026-09-23 sweep).
  //   N4  Bounded: n in [NEUTRAL_MIN_FRAMES, NEUTRAL_MAX_FRAMES] per phase, rails as begin() set
  //       them (0x01 untouched: VDH/VDL +-15 V), no VDHR (11b) anywhere, no XON (0x2A never written).
  //   N5  Gate scan follows partialScanOutside(): 0 confines the scan to the window rows.
  //   N6  The next OTP refresh of any kind begins with leaveFast() (POF, reset, vendor registers),
  //       exactly like FAST (S4). A BUSY timeout is a fault like any other.
  // lutBalanced() checks N1-N4's arithmetic on the bytes actually sent; startNeutral refuses
  // (sends nothing) if a table ever fails it.
  static constexpr uint8_t NEUTRAL_MIN_FRAMES = 4, NEUTRAL_MAX_FRAMES = 30;
  // Two groups of one phase each: group 0 at `first`, group 1 at `second`, both `frames` long,
  // repeated once; groups 2.. zero (the end condition: "all Number of Frames = 0").
  static void neutralLut(uint8_t *out, size_t len, uint8_t first, uint8_t second, uint8_t frames) {
    memset(out, 0, len);
    out[0] = first;  out[1] = frames; out[5] = 1;
    out[6] = second; out[7] = frames; out[11] = 1;
  }
  // Net source drive of a source LUT, in frames: +1 per VDH frame, -1 per VDL frame, x repeat.
  // Returns INT32_MIN when any phase that has frames selects 11b (VDHR / floating).
  static int32_t lutNetFrames(const uint8_t *t, size_t len) {
    int32_t net = 0;
    for (size_t g = 0; g + 6 <= len; g += 6)
      for (int ph = 0; ph < 4; ph++) {
        const uint8_t lvl = (uint8_t) ((t[g] >> (6 - 2 * ph)) & 3), fr = t[g + 1 + ph];
        if (!fr || !t[g + 5]) continue;
        if (lvl == 3) return INT32_MIN;
        net += (lvl == 1 ? 1 : lvl == 2 ? -1 : 0) * (int32_t) fr * (int32_t) t[g + 5];
      }
    return net;
  }
  // True when every phase that runs selects level 00 (GND for a source LUT, VCOM_DC / VCOM for
  // LUTC / LUTBD): a table that applies no field at all.
  static bool lutAllZeroLevel(const uint8_t *t, size_t len) {
    for (size_t g = 0; g + 6 <= len; g += 6)
      for (int ph = 0; ph < 4; ph++)
        if (t[g + 1 + ph] && t[g + 5] && ((t[g] >> (6 - 2 * ph)) & 3)) return false;
    return true;
  }
  // Fills the six tables startNeutral sends (VCOM 60, WW 42, KW 60, WK 60, KK 60, BD 42) and checks
  // them; false = refuse. Public so the host test sees exactly the bytes that go out.
  static bool neutralTables(uint8_t frames, uint8_t *c60, uint8_t *ww42, uint8_t *kw60, uint8_t *wk60,
                            uint8_t *kk60, uint8_t *bd42) {
    if (frames < NEUTRAL_MIN_FRAMES || frames > NEUTRAL_MAX_FRAMES) return false;
    neutralLut(c60, 60, 0x00, 0x00, frames);                       // N3: VCOM_DC, both groups
    neutralLut(ww42, 42, 0x00, 0x00, frames);                      // N1: W->W at GND
    neutralLut(kw60, 60, LVL_TO_BLACK, LVL_TO_WHITE, frames);      // N2: K->W: VDH n, VDL n
    neutralLut(wk60, 60, LVL_TO_WHITE, LVL_TO_BLACK, frames);      // N2: W->K: VDL n, VDH n
    neutralLut(kk60, 60, 0x00, 0x00, frames);                      // N1: K->K at GND
    neutralLut(bd42, 42, 0x00, 0x00, frames);                      // N1: border at VCOM
    return lutNetFrames(kw60, 60) == 0 && lutNetFrames(wk60, 60) == 0 &&
           lutAllZeroLevel(c60, 60) && lutAllZeroLevel(ww42, 42) && lutAllZeroLevel(kk60, 60) &&
           lutAllZeroLevel(bd42, 42);
  }
  bool startNeutral(const uint8_t *frame, const uint8_t *prev, int y0, int y1, int xb0, int xb1, uint8_t frames) {
    uint8_t c60[60], ww42[42], kw60[60], wk60[60], kk60[60], bd42[42];
    if (!neutralTables(frames, c60, ww42, kw60, wk60, kk60, bd42)) return false;
    if (!startRegister(frame, prev, y0, y1, xb0, xb1, c60, ww42, kw60, wk60, kk60, bd42, neutralVcom_, FAST_CDI, 0))
      return false;
    neutralCount_++;
    return true;
  }
private:
  // The shared register-partial sequence of NEUTRAL and PAIR (tables already checked by them).
  bool startRegister(const uint8_t *frame, const uint8_t *prev, int y0, int y1, int xb0, int xb1,
                     const uint8_t *c60, const uint8_t *ww42, const uint8_t *kw60, const uint8_t *wk60,
                     const uint8_t *kk60, const uint8_t *bd42, uint8_t vcom, uint8_t cdi, uint8_t evs) {
    if (step_ != Step::IDLE || !prev || !frame) return false;
    if (y0 < 0) y0 = 0;
    if (y1 > H) y1 = H;
    if (xb0 < 0) xb0 = 0;
    if (xb1 > ROW) xb1 = ROW;
    if (y0 >= y1 || xb0 >= xb1) return false;
    full_ = false; clean_ = false; count_ = true; frame_ = frame; prev_ = prev; y0_ = y0; y1_ = y1;
    xb0_ = xb0; xb1_ = xb1;
    cmd(0x00, {0x3F});                     // PSR: LUT from register, KW mode (as FAST)
    cmd(0x82, {vcom});                     // N3: VCOM_DC explicit (NEUTRAL/PAIR: never beyond 0x0A)
    cmd(0x50, {cdi, 0x07});                // NEUTRAL/PAIR: FAST_CDI, border -> LUTBD (zero), N2OCP=1
    if (evs) cmd(0x52, {evs});             // COPY only: the OTP TR's own end levels
    cmd(0xE0, {0x00});                     // no forced temperature
    cmdData(0x20, c60, 60);
    cmdData(0x21, ww42, 42);
    cmdData(0x22, kw60, 60);
    cmdData(0x23, wk60, 60);
    cmdData(0x24, kk60, 60);
    cmdData(0x25, bd42, 42);
    regLut_ = true;
    neutral_ = true;
    pair_ = false;
    copy_ = false;
    powerOn();
    return true;
  }
public:
  // PAIR partial (2026-10-01, after the 0.3.2 glass result; BENCH-SELECTED, never a default).
  // NEUTRAL's per-transition balance cancelled its own drive: changed pixels never reached full
  // black/white. PAIR keeps NEUTRAL's zero-field rules for everything that does not change (N1),
  // its constant VCOM (N3), bounds (N4), scan (N5) and exit fence (N6), but drives each transition
  // in ONE direction, like a normal partial:
  //   P1  K->W = VDL x n (LVL_TO_WHITE), W->K = VDH x n (LVL_TO_BLACK) -- the polarities measured on
  //       this glass (E2, 2026-09-23); optionally preceded by a symmetric pre-shake of s frames at
  //       the opposite level (K->W: VDH s then VDL n; W->K: VDL s then VDH n), s < n, s <= 10.
  //   P2  DC balance is over the PAIR: net(K->W) + net(W->K) == 0, net(K->W) < 0 < net(W->K).
  //       A pixel's transitions strictly alternate (it can only go K->W when it is black), so its
  //       running net stays within one transition, (n - s) frames at 15 V -- as long as every
  //       change of that pixel is a PAIR transition (the firmware keeps full refreshes from
  //       carrying picture changes while PAIR is on, see nostalgia_panel 0.3.3).
  // pairTables() refuses any table set whose pair sum is not 0 or whose sides do not drive the
  // right way; startPair sends nothing then.
  static constexpr uint8_t PAIR_MAX_SHAKE = 10;
  // group 0 at `first` for f1 frames then group 1 at `second` for f2 frames; f1 = 0 puts `second`
  // in group 0 (a zero group would be the end condition).
  static void twoPhaseLut(uint8_t *out, size_t len, uint8_t first, uint8_t f1, uint8_t second, uint8_t f2) {
    memset(out, 0, len);
    size_t g = 0;
    if (f1) { out[0] = first; out[1] = f1; out[5] = 1; g = 6; }
    out[g] = second; out[g + 1] = f2; out[g + 5] = 1;
  }
  static bool pairBalanced(const uint8_t *kw60, const uint8_t *wk60) {
    const int32_t a = lutNetFrames(kw60, 60), b = lutNetFrames(wk60, 60);
    return a != INT32_MIN && b != INT32_MIN && a < 0 && b > 0 && a + b == 0;
  }
  static bool pairTables(uint8_t frames, uint8_t shake, uint8_t *c60, uint8_t *ww42, uint8_t *kw60, uint8_t *wk60,
                         uint8_t *kk60, uint8_t *bd42) {
    if (frames < NEUTRAL_MIN_FRAMES || frames > NEUTRAL_MAX_FRAMES || shake > PAIR_MAX_SHAKE || shake >= frames)
      return false;
    twoPhaseLut(c60, 60, 0x00, shake, 0x00, frames);                        // N3: VCOM_DC throughout
    twoPhaseLut(ww42, 42, 0x00, shake, 0x00, frames);                       // N1: W->W at GND
    twoPhaseLut(kw60, 60, LVL_TO_BLACK, shake, LVL_TO_WHITE, frames);       // P1: K->W
    twoPhaseLut(wk60, 60, LVL_TO_WHITE, shake, LVL_TO_BLACK, frames);       // P1: W->K
    twoPhaseLut(kk60, 60, 0x00, shake, 0x00, frames);                       // N1: K->K at GND
    twoPhaseLut(bd42, 42, 0x00, shake, 0x00, frames);                       // N1: border at VCOM
    return pairBalanced(kw60, wk60) && lutAllZeroLevel(c60, 60) && lutAllZeroLevel(ww42, 42) &&
           lutAllZeroLevel(kk60, 60) && lutAllZeroLevel(bd42, 42);
  }
  bool startPair(const uint8_t *frame, const uint8_t *prev, int y0, int y1, int xb0, int xb1, uint8_t frames,
                 uint8_t shake = 0) {
    uint8_t c60[60], ww42[42], kw60[60], wk60[60], kk60[60], bd42[42];
    if (!pairTables(frames, shake, c60, ww42, kw60, wk60, kk60, bd42)) return false;
    if (!startRegister(frame, prev, y0, y1, xb0, xb1, c60, ww42, kw60, wk60, kk60, bd42, neutralVcom_, FAST_CDI, 0))
      return false;
    pair_ = true;
    pairCount_++;
    return true;
  }
  bool pairActive() const { return regLut_ && neutral_ && pair_; }

  // OTP COPY (2026-10-01, panel fw 0.3.5; BENCH-SELECTED, never a default). Loads a KW-mode table set
  // read out of the controller's own OTP (bench dump, 7 groups of 6 bytes each: LUTC, LUTWW, LUTKW,
  // LUTWK, LUTKK -- the OTP's byte format is the register format) into the register LUTs, so the
  // factory drive runs byte for byte, with only the caller's VDCS (and, if the caller passes a
  // modified table, its own change) different. It refuses (sends nothing) unless:
  //   C1  no phase that runs selects 11b (VDHR / floating) in any table;
  //   C2  LUTC is VCOM_DC only (level 00): no AC VCOM, no floating (the caller's table; floatVcom below);
  //   C3  every table has LUTC's exact frame/repeat bytes (the tables run in lockstep);
  //   C4  the drive is at most COPY_MAX_FRAMES frames long;
  //   C5  VDCS in [COPY_VCOM_MIN, COPY_VCOM_MAX] (-0.10 .. -2.00 V; 0x00 = controller floor, owner-approved bench 2026-10-02);
  //   C6  CDI is the factory partial's 0x19 (border -> LUTWK, N2OCP, DDX=01) or FAST_CDI 0x39 (border
  //       -> a zero LUTBD); EVS is 0 (not written), 0x02 (default) or 0x03 (border floats at the end).
  // Rails (0x01/0x06), frame rate (0x30) and XON (0x2A) are never written: the vendor values after
  // begin() are the OTP TR's (VDH/VDL 15 V, VG 20 V, 50 Hz, no XON) -- the caller checks its dump.
  static constexpr uint8_t COPY_VCOM_MIN = 0x00, COPY_VCOM_MAX = FAST_VCOM_DC, COPY_MAX_FRAMES = 120;
  static int copyFrames(const uint8_t *t42) {           // frames until the end condition
    int n = 0;
    for (int g = 0; g < 7; g++) {
      const uint8_t *s = t42 + g * 6;
      if (!s[5] || !(s[1] | s[2] | s[3] | s[4])) break;
      n += (s[1] + s[2] + s[3] + s[4]) * s[5];
    }
    return n;
  }
  static bool copyTablesOk(const uint8_t *c42, const uint8_t *ww42, const uint8_t *kw42, const uint8_t *wk42,
                           const uint8_t *kk42) {
    const uint8_t *src[4] = {ww42, kw42, wk42, kk42};
    if (!c42 || !lutAllZeroLevel(c42, 42)) return false;                         // C2
    for (const uint8_t *t : src) {
      if (!t || lutNetFrames(t, 42) == INT32_MIN) return false;                  // C1
      for (int g = 0; g < 7; g++)
        for (int i = 1; i <= 5; i++) if (t[g * 6 + i] != c42[g * 6 + i]) return false;   // C3
    }
    const int n = copyFrames(c42);
    return n > 0 && n <= COPY_MAX_FRAMES;                                         // C4
  }
  // FLOATING VCOM (2026-10-09, panel fw 0.3.11 experiment; owner-approved). The caller's LUTC must
  // still pass C2 (VCOM_DC only); `floatVcom` then sends the same table with every running group's
  // level byte set to 11b on all four phases (datasheet p.42/p.44: "the driver will float VCOM"), so
  // the common electrode floats for the whole drive instead of sitting at VDCS. The frame/repeat
  // bytes are untouched, so C3 lockstep still holds. Never a mix: either every running group of LUTC
  // floats, or none does (floatLutc / lutAllFloatLevel). Research S8 (2026-09-23) kept 11b out of
  // production on a drift model (a floating refresh inherits the previous path's rest state), not a
  // hazard; this is the controlled test of that, behind the panel's own persisted switch.
  static void floatLutc(const uint8_t *c42, uint8_t *out42) {
    memcpy(out42, c42, 42);
    for (int g = 0; g < 7; g++) {
      uint8_t *s = out42 + g * 6;
      if (!s[5] || !(s[1] | s[2] | s[3] | s[4])) break;     // the end condition, as copyFrames
      s[0] = 0xFF;
    }
  }
  // True when every running group of a 42-byte LUTC selects 11b on every phase that has frames.
  static bool lutAllFloatLevel(const uint8_t *t42) {
    bool any = false;
    for (int g = 0; g < 7; g++) {
      const uint8_t *s = t42 + g * 6;
      if (!s[5] || !(s[1] | s[2] | s[3] | s[4])) break;
      for (int ph = 0; ph < 4; ph++)
        if (s[1 + ph] && ((s[0] >> (6 - 2 * ph)) & 3) != 3) return false;
      any = true;
    }
    return any;
  }
  bool startOtpCopy(const uint8_t *frame, const uint8_t *prev, int y0, int y1, int xb0, int xb1,
                    const uint8_t *c42, const uint8_t *ww42, const uint8_t *kw42, const uint8_t *wk42,
                    const uint8_t *kk42, uint8_t vcom, uint8_t cdi = 0x19, uint8_t evs = 0,
                    bool floatVcom = false) {
    if (!copyTablesOk(c42, ww42, kw42, wk42, kk42)) return false;
    if (vcom < COPY_VCOM_MIN || vcom > COPY_VCOM_MAX) return false;               // C5
    if ((cdi != 0x19 && cdi != FAST_CDI) || (evs != 0 && evs != 0x02 && evs != 0x03)) return false;   // C6
    uint8_t c60[60] = {0}, kw60[60] = {0}, wk60[60] = {0}, kk60[60] = {0}, bd42[42] = {0};
    if (floatVcom) {
      floatLutc(c42, c60);
      if (!lutAllFloatLevel(c60)) return false;
    } else {
      memcpy(c60, c42, 42);
    }
    memcpy(kw60, kw42, 42); memcpy(wk60, wk42, 42); memcpy(kk60, kk42, 42);
    if (!startRegister(frame, prev, y0, y1, xb0, xb1, c60, ww42, kw60, wk60, kk60, bd42, vcom, cdi, evs))
      return false;
    copy_ = true;
    copyCount_++;
    if (floatVcom) copyFloatCount_++;
    return true;
  }
  bool copyActive() const { return regLut_ && neutral_ && copy_; }
  uint32_t copyCount() const { return copyCount_; }
  uint32_t copyFloatCount() const { return copyFloatCount_; }
  uint32_t pairCount() const { return pairCount_; }

  // VDCS for the neutral path only: clamped to 0x00..FAST_VCOM_MIN (-0.10 .. -0.60 V), so it can
  // never be more negative than production's FAST value. Default FAST_VCOM_MIN. Takes effect at the
  // next startNeutral (no command is sent here).
  void neutralVcom(uint8_t code) { neutralVcom_ = code > FAST_VCOM_MIN ? FAST_VCOM_MIN : code; }
  uint8_t neutralVcomCode() const { return neutralVcom_; }
  bool neutralActive() const { return regLut_ && neutral_; }
  uint32_t neutralCount() const { return neutralCount_; }

  // Rail power-ups since begin() (research S9: rail cycles are counted per session).
  uint32_t railCycles() const { return railCycles_; }
  bool fastActive() const { return regLut_; }

  // BENCH: factory partial gate coverage (UC8179c C0.6, R90H PT_SCAN).
  // Default 1 scans gates inside AND outside the window; 0 only inside.
  // Register-LUT FAST/NULL paths always retain their validated scan=1; NEUTRAL follows this.
  void partialScanOutside(bool on) { scanOutside_ = on; }
  bool partialScanOutside() const { return scanOutside_; }

  // The fast path's VCOM (research S3). VDCS code (0x82): V = -0.10 - 0.05 * code; limited to
  // FAST_VCOM_MIN..0x40 (-0.60 .. -3.30 V) for the long-form sweep of 2026-09-23. `table`: 0 = the VCOM LUT holds VCOM_DC for the phase (default),
  // 1 = an all-zero VCOM LUT (no VCOM group at all). Floating (11b) is never used.
  void fastVcom(uint8_t code, uint8_t table = 0) {
    fastVcom_ = code < FAST_VCOM_MIN ? FAST_VCOM_MIN : code > 0x40 ? 0x40 : code;
    fastVcomTable_ = table ? 1 : 0;
  }
  uint8_t fastVcomCode() const { return fastVcom_; }
  // BENCH: write VDCS explicitly (same clamp as fastVcom) on the factory (OTP) path too; -1 = never
  // (the vendor's value after reset, the default). For the mixed-refresh drift investigation.
  void factoryVcom(int code) {
    factoryVcom_ = code < 0 ? -1 : code < FAST_VCOM_MIN ? FAST_VCOM_MIN : code > 0x40 ? 0x40 : code;
  }
  uint8_t fastVcomTable() const { return fastVcomTable_; }

  // Advance the running refresh by at most one step. Cheap when idle or still busy.
  void poll() {
    if (step_ == Step::IDLE) return;
    const uint32_t now = bus_.millis();
    if ((int32_t)(now - notBefore_) < 0) return;
    if (bus_.busy()) {
      if (now - stepStart_ > BUSY_TIMEOUT_MS) {
        faulted_ = true;
        if (step_ == Step::IDLE_POWER_WAIT) {
          cmd(0x02);  // failed bench-only PON: attempt proper drive-off, never a waveform
          enter(Step::IDLE_OFF_WAIT, 0);
        } else step_ = Step::IDLE;
        return;
      }
      if (now - lastStatus_ >= 10) { cmd(0x71); lastStatus_ = now; }  // V2 wants status polls
      return;
    }
    switch (step_) {
      case Step::IDLE_OFF_WAIT:
        powered_ = false;
        step_ = Step::IDLE;
        break;
      case Step::IDLE_POWER_WAIT:
        powered_ = true;
        step_ = Step::IDLE;
        break;
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
  uint8_t partialTemp_ = 0x6E;
  bool offBeforeReset_ = true;  // leaveFast(): power off before the S4 reset
  bool scanOutside_ = true;  // factory PT_SCAN only; default unchanged
  bool regLut_ = false;       // register LUTs are loaded: the next OTP refresh resets first (S4)
  bool neutral_ = false;      // ... and they are the NEUTRAL or PAIR set (scan follows scanOutside_)
  bool pair_ = false;         // ... the PAIR set
  bool copy_ = false;         // ... an OTP COPY set
  uint32_t copyCount_ = 0, copyFloatCount_ = 0;
  uint32_t pairCount_ = 0;
  uint8_t neutralVcom_ = FAST_VCOM_MIN;
  uint32_t neutralCount_ = 0;
  uint8_t fastVcom_ = FAST_VCOM_DC, fastVcomTable_ = 0;
  int factoryVcom_ = -1;
  uint32_t railCycles_ = 0;
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
  void cmdData(uint8_t c, const uint8_t *d, size_t n) {
    bus_.command(c);
    if (n) bus_.data(d, n);
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
    railCycles_++;
    cmd(0x04);                             // power on; BUSY until the rails are up
    enter(Step::POWER_WAIT, SETTLE_MS);
  }
  // PTL PT_SCAN (UC8179c C0.6 R90H): FAST/NULL keep their validated 1; factory and NEUTRAL
  // partials follow partialScanOutside().
  uint8_t scanByte() const { return (uint8_t) ((regLut_ && !neutral_) || scanOutside_ ? 0x01 : 0x00); }
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
                 (uint8_t)(ya >> 8), (uint8_t)(ya & 0xFF), (uint8_t)(yb >> 8), (uint8_t)(yb & 0xFF), scanByte()});
      sendRows(0x10, prev_, invert_);      // what is on the glass (old)
      sendRows(0x13, frame_, invert_);     // what should be (new)
    }
    cmd(0x12);                             // display refresh
  }
};
