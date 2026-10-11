# NanoGFX API Reference

Complete reference for every public function in the library. One include gets the core:

```cpp
#include <NanoGFX.h>    // PackCanvas + PackRLE + PackFlush + PackMono + EInkGhost
#include <RM690B0.h>    // (optional, ESP32) QSPI AMOLED driver
#include <ST7789.h>     // (optional, ESP32) 4-wire SPI TFT driver
#include <UC8179.h>     // (optional, any MCU) 800x480 e-paper driver, bring your own bus
#include <PackInk.h>    // (optional, any MCU) textured 1-bit drawing for e-paper art
```

Contents: [Builds](#builds-arduino-esp-idf-esphome-host) · [PackCanvas](#packcanvas) ·
[PackRLE](#packrle) · [PackRLE images](#packrle-images) · [PackFlush](#packflush) ·
[RM690B0](#rm690b0) · [ST7789](#st7789) · [PackMono](#packmono) · [UC8179](#uc8179) ·
[EInkGhost](#einkghost) · [PackInk](#packink) · [Tools](#tools) · [Formats & contracts](#formats--contracts)

---

## Builds (Arduino, ESP-IDF, ESPHome, host)

NanoGFX is header-only and builds two ways from the same sources:

- **With Adafruit_GFX** (Arduino, when `<Adafruit_GFX.h>` is on the include path):
  `PackCanvas` derives from `GFXcanvas8`, exactly as in earlier releases.
- **Standalone** (ESP-IDF, ESPHome, a desktop compiler; or any build that defines
  `NANOGFX_STANDALONE`): `PackCanvas` derives from **`NanoGFX_Canvas8`** in `NanoGFXBase.h`, a
  subset of Adafruit_GFX 1.12 with the same members, virtuals and pixel output (pixels, lines,
  rects, round rects, circles, triangles, 1-bit bitmaps, the classic 5×7 font, **GFXfont**
  proportional fonts, `getTextBounds`, `print`/`printf`). Not included: `Print`/`String`,
  RGB/grayscale bitmaps, `invertDisplay`.

The selection is automatic (`__has_include`). `NGFX_GFX` and `NGFX_CANVAS8` name whichever
base is in use, for code that calls the base class explicitly.

**ESP-IDF component.** The repo root is a component (`CMakeLists.txt`, `idf_component.yml`).
Add it with a path dependency — `nanogfx: { path: ../NanoGFX }` in your `idf_component.yml`,
or `add_idf_component(path=...)` from an ESPHome external component.

**Host.** `tests/host_standalone.cpp` builds with any C++17 compiler:
`c++ -std=c++17 -Wall -Wextra -I src tests/host_standalone.cpp -o ngfx_host && ./ngfx_host`.

---

## PackCanvas

`class PackCanvas : public NGFX_CANVAS8` (`GFXcanvas8` on Arduino, `NanoGFX_Canvas8` standalone)

One canvas class, three storage formats selected **per instance**. Because it derives from
`Adafruit_GFX`, **every stock GFX call works**: `drawLine`, `drawRect`, `fillRect`,
`drawCircle`, `fillCircle`, `drawTriangle`, `fillTriangle`, `drawRoundRect`, `fillRoundRect`,
`drawChar`, `print`/`println`, `setCursor`, `setTextColor`, `setFont`, `getTextBounds`,
`setRotation`, … NanoGFX overrides the hot paths underneath so those calls hit packed
fast paths automatically.

### Construction & format

| Member | Description |
|---|---|
| `PackCanvas(uint16_t w, uint16_t h, bool allocate_buffer = true)` | Create a canvas. With `allocate_buffer=false`, supply memory later via `useBuffer()`. |
| `bool packed4 = false` | `false` = 8-bit palette canvas (1 byte/px, `GFXcanvas8` semantics + `setRotation` fast paths). `true` = 4-bit packed (2 px/byte). **Set before the first drawing call.** |
| `bool dualMode = false` | Packed only. Each line lives as a **run-list** until scattered pixels demote it to flat ([details](#dual-mode)). Enables the near-free `encodeFrame()`. |
| `void useBuffer(uint8_t *b)` | Point the canvas at caller-owned memory (e.g. ESP32 PSRAM/internal via `heap_caps_malloc`). Size it with `bufBytes()`. Construct with `allocate_buffer=false`. |
| `static size_t bufBytes(uint16_t w, uint16_t h, bool p4)` | Bytes needed for a `w`×`h` canvas. Packed canvases are **per-line slots** of `lineSlotBytes()` each (even-rounded), *not* globally packed nibbles. |
| `size_t lineSlotBytes() const` | Slot bytes per packed line. Row `y` starts at `rawBuffer() + y * lineSlotBytes()`. |

Colors everywhere are **palette indices 0–14** (only the low 4 bits are used on packed
canvases). Index 15 (`0xF`) is reserved as the codec escape — never draw with it.

### Drawing (overridden fast paths)

All standard `Adafruit_GFX` drawing works; these are the members NanoGFX overrides or adds.
Unless noted, calls use **logical coordinates** (they respect `setRotation` on 8-bit
canvases and the rotation matrix when set; packed canvases are native/rotation-0 by contract).

| Function | Behavior |
|---|---|
| `void drawPixel(int16_t x, int16_t y, uint16_t color)` | Single pixel. Routes through the rotation matrix when one is active. On a dual-mode line, the first scattered pixel *explodes* that line to flat (by design — see [splice policy](#dual-mode)). |
| `void writePixel(int16_t x, int16_t y, uint16_t color)` | Same as `drawPixel` but skips the virtual dispatch and re-checks in the common identity case — this is what glyph rendering hits. |
| `void fastDrawPixel(int16_t x, int16_t y, uint16_t color)` | Clipped, **unrotated** pixel write (bypasses matrix and `setRotation`). |
| `void putNib(int16_t x, int16_t y, uint8_t c)` | 4-bit "fast pixel": clipped, unrotated, mode-dispatched. On 8-bit instances it performs the historical global-nibble RMW. |
| `void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)` | Horizontal span. Flat lines: odd-lead nibble / whole-byte `memset` / even-tail. Run-form lines: **one splice, no pixels touched**. |
| `void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)` | Vertical span. Run-form lines take a 1-px splice per row (verticals through clean bands keep those rows compressed). |
| `void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)` | **Row-major** (one `drawFastHLine` per row): a memset or single splice per row instead of Adafruit's per-column loop. Degenerate/negative spans delegate to the base for bit-exact legacy semantics. |
| `void fillScreen(uint16_t color)` | 8-bit: one `memset`. Packed: whole-buffer `memset`. Dual-mode: **one run per line, O(H)** — no pixel writes at all (plus the sticky-flat frame policy, see below). |
| `void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)` | Span-batched Bresenham producing the **identical pixel set** to Adafruit's, without a virtual call per pixel: shallow segments emit one horizontal span per row, steep segments emit strided columns. Engages on identity canvases (packed, or 8-bit at rotation 0); `fastLine = false` forces the stock chain (fuzz oracle). |
| `uint8_t getPixel(int16_t x, int16_t y) const` | Format-aware read. Packed run-form lines are read by **walking the run-list — reading never explodes a line**. 8-bit honours `setRotation` exactly like `GFXcanvas8`. Out of bounds → 0. |

### Sprites, images, scrolling

These operate in **native coordinates** (rotation matrix and `setRotation` deliberately
ignored — the same convention as sprite pushes in TFT_eSPI/LovyanGFX).

| Function | Behavior |
|---|---|
| `void blit(const PackCanvas &src, int16_t dx, int16_t dy, int16_t transparent = -1)` | Stamp all of `src` at `(dx, dy)`. `transparent` = palette index to skip (0–15), or -1 for opaque. |
| `void blitRect(const PackCanvas &src, int16_t sx, int16_t sy, int16_t w, int16_t h, int16_t dx, int16_t dy, int16_t transparent = -1)` | Blit a sub-rect of `src`. Fully clipped on both ends. Fast paths: opaque + even `sx`/`dx` → one `memcpy` per row; run-form source rows → span emission (no per-pixel work). Mixed formats (8-bit↔packed) take an exact per-pixel path. |
| `void drawIndexedBitmap(int16_t x, int16_t y, const uint8_t *data, int16_t w, int16_t h, int16_t transparent = -1)` | Draw a nibble-packed image: 2 px/byte, **high nibble first**, row stride `(w+1)/2` bytes. `data` may live in flash (read via `pgm_read_byte`). Image data must use indices 0–14 only. |
| `void vScroll(int16_t y0, int16_t h, int16_t dy, uint8_t fill)` | Scroll rows `[y0, y0+h)` by `dy` (`>0` down, `<0` up); vacated rows become `fill`. On packed canvases this **moves line records** (three `memmove`s — slots + metadata): no per-pixel work, and run-form rows stay run-form. Native row coordinates. |

### Text

| Function | Behavior |
|---|---|
| `size_t write(uint8_t ch)` | (What `print` calls.) Classic-font glyphs on packed identity canvases render through the **dual-parity row blitter**: a parity-aligned nibble mask per glyph row, whole-byte mask-RMW stores — ~4–5× fewer ops per glyph, any x alignment, sizes 1–4, byte-identical to stock Adafruit output. Everything else (custom `GFXfont`s, opaque background, rotation) falls back to the stock chain automatically. |
| `bool fastText = true` | Set `false` to force the stock Adafruit text path (the fuzz suite uses this as the oracle). |
| `uint8_t textScale = 1` | Multiplies every `setTextSize()` — one knob to scale all text for a larger panel. |
| `void setTextSize(uint8_t s)` | Applies `s * textScale`. |

### Rotation

| Function | Behavior |
|---|---|
| `void setRotationMatrix(float angle = 0, float x = 0, float y = 0)` | Rotate subsequent draws by `angle` (radians) about `(x, y)` — for compass cards / rotated instruments. Call with no arguments to return to identity. Works on every format. The per-pixel transform is a precomputed **integer 16.16 affine** (four 32-bit multiplies — 2-3× faster than the float form it replaced; rounds to nearest; inputs bounded to ±8191). `drawFastH/VLine` under a matrix decompose to rotated pixels. |
| `bool identity` | True when no matrix rotation is active (read-only in practice). |
| `setRotation(r)` *(inherited)* | Quarter-turn logical rotation — supported on **8-bit** instances (fast paths included). Packed instances are native-orientation by contract; rotate at the panel instead (MADCTL). |

### Frame finalize & raw access

| Function | Behavior |
|---|---|
| `void encodeFrame(uint8_t *comp, uint16_t *lineLen, size_t stride, int y0, int y1, uint32_t *lineHash = nullptr)` | Encode lines `[y0, y1)` as PackRLE streams into region-relative fixed slots (`comp + (y-y0)*stride`, length in `lineLen[y-y0]`). Run-form lines emit via `prle_encode_runs` — **no pixel scan; the drawing already did the compression**. Every length ≤ `PRLE_STRIDE(W)`, guaranteed. Pass `lineHash` to also get a per-line FNV-1a of the stream (near-free — bytes are still in cache) for `PackFlush::flushHashed`. |
| `void flatten(int y0, int y1)` | Force lines `[y0, y1)` to flat so their slot bytes are plain packed nibbles — **required before poking/reading rows via `rawBuffer()`**. |
| `uint8_t *rawBuffer()` | The buffer, no flattening — only for code that flattened its rows first. |
| `uint8_t *getBuffer()` | Whole-canvas raw access: flattens everything first (then byte-identical to a plain packed canvas). |
| `const uint8_t *lineBytes(int y) const` | Pointer to row `y`'s slot bytes (valid as flat nibbles only while the line is flat). |
| `size_t snapshotBytes()` / `void snapshot(uint8_t *dst)` / `void restore(const uint8_t *src)` | Capture/restore the FULL canvas state — pixels and the dual-mode per-line metadata — with row-aware copies (run-form rows copy only their entries). For pre-rendered static **templates**: render chrome once, `snapshot()`, then `restore()` each frame instead of redrawing. `stickyFlat` is deliberately untouched (only `fillScreen` reads it, which a template replaces). |
| `int lineRunList(int y, const uint16_t **e) const` | While row `y` is run-form: run count, with `*e` pointed at the entries (`(x_start << 4) \| color`; a run ends at the next entry's x, the last at `W`). Returns -1 when flat (read `lineBytes()` instead). For compositors that consume runs directly. |
| `bool lineIsRuns(int y) const` / `int lineRuns(int y) const` | Telemetry: is row `y` run-form / how many runs. |

### Dual-mode

With `dualMode = true`, each line is **RUNS** (a sorted run-list in the line's slot) until
content demotes it to **FLAT** (plain nibbles):

- **Span ops splice** — `drawFastHLine`/`drawFastVLine`/`fillRect` rebuild the small list
  (cap `RUN_CAP` = 16 entries, same-color neighbours merged on both edges). No pixels touched.
- **Scattered pixels explode** — the first `drawPixel`/glyph pixel on a RUNS line renders it
  flat once; the line then runs at full flat speed. (Per-pixel splicing measured 10–20×
  slower on Xtensa; this policy is why dual-mode is *never worse* than the flat canvas.)
- **Sticky-flat** — a line that exploded starts the *next* frame flat (content is stable
  frame-to-frame), decaying every 128 frames so rows whose content moved away re-probe RUNS.
- `fillScreen` resets clean lines to one run and honours sticky-flat.

You never manage any of this; it's described so the performance model is predictable:
**structured rows (fills, bands, gauges) stay compressed; busy rows (text, maps) go flat.**

---

## PackRLE

The 15-color + escape-nibble codec (`PackRLE.h`, all `static inline` free functions).
Stream format: nibbles `0x0–0xE` are literal pixels; `0xF` escapes to
`[color][n2][n1][n0]` — a 12-bit run. **Worst case = flat packed size, proven**:
`PRLE_STRIDE(w) = (w+1)/2` bytes is always enough.

| Function | Description |
|---|---|
| `PRLE_STRIDE(w)` | Macro: fixed per-line slot size (= flat packed bytes). Encodes never exceed it. |
| `size_t prle_encode_flat(const uint8_t *flat, int w, uint8_t *out)` | Encode one line from flat packed nibbles (2 px/byte). Word-batched run scan; literal blocks copy verbatim. Returns stream bytes. |
| `size_t prle_encode_idx8(const uint8_t *idx, int w, uint8_t *out)` | Encode from 1-byte-per-pixel palette indices. |
| `size_t prle_encode_runs(const uint16_t *runs, int n, int w, uint8_t *out)` | Encode straight from a run-list (`(x<<4)\|color` entries) — what makes dual-mode encodes near-free. |
| `bool prle_decode_flat(const uint8_t *enc, size_t nb, int w, uint8_t *out)` | Decode to flat packed nibbles. Returns false on malformed input (defensive; never produced by these encoders). |
| `bool prle_decode_lut8(const uint8_t *enc, size_t nb, int w, uint8_t *out, const uint8_t lut[16])` | Decode to 1 byte/px through a palette LUT — runs become `memset`s. |
| `bool prle_decode_lut8p(const uint8_t *enc, size_t nb, int w, uint8_t *out, const uint8_t lut[16], const uint16_t lutPair[256])` | Same, plus a 256-entry **pair LUT** so literal bytes convert **2 px per lookup**. The fast path for 8-bpp panels (build `lutPair[b] = lut[b>>4] \| (lut[b&15] << 8)`). |
| `bool prle_decode_lut32(const uint8_t *enc, size_t nb, int w, uint32_t *out, const uint32_t lut[16])` | Decode to 32-bit colors (RGB888/canvas previews). |
| `bool prle_decode_p3(const uint8_t *enc, size_t nb, int w, uint8_t *out, const uint32_t p3Lut[256])` | Decode to **RGB444 wire triplets** (2 px → exactly 3 bytes; COLMOD `0x53` on ST7789-class panels): 25% less wire time, lossless for a 15-color palette. `out` needs one spare byte past the payload (size bounce buffers +4). |

Compile with `-DPRLE_TEST_MAIN` to build the codec's standalone self-test.

> **Xtensa/ESP32 note:** `PackRLE.h` pins `-O2` codegen via `#pragma GCC optimize` — Arduino's
> default `-Os` refuses to flatten the nibble writer and costs ~2× encode time.

---

## PackFlush

Dirty-band frame flushing (`PackFlush.h`). Keeps an "on-glass" copy of each row's
**encoded** stream — comparing compressed bytes, typically 10–50× smaller than pixels —
and pushes only coalesced bands of changed rows. For any panel with persistent GRAM.

| Function | Description |
|---|---|
| `void begin(uint8_t *glass, uint16_t *glassLen, int height, size_t slot)` | Attach caller-owned glass buffers: `height*slot` bytes + `height` lengths (put them in PSRAM on ESP32). `slot` must match the stride passed to `encodeFrame`. |
| `int flush(const uint8_t *comp, const uint16_t *lineLen, int gap, PushBand pushBand)` | Flush one frame. `pushBand(y0, y1)` (any callable) must write rows `[y0, y1]` to the panel — set the window, decode, send. `gap` = clean rows absorbed into a band rather than paying a window switch (4–16 typical). Returns rows pushed. First flush after `begin()`/`invalidate()` pushes the full frame and adopts it. |
| `void beginHashed(uint32_t *glassHash, uint16_t *glassLen, int height)` / `int flushHashed(const uint16_t *lineLen, const uint32_t *lineHash, int gap, PushBand)` | **Hashed mode**: the on-glass state is each row's (length, FNV-1a) pair from `encodeFrame(..., lineHash)` — no glass byte copy, no memcmp; dirty testing is two integer compares per row. A changed row is wrongly skipped only on a same-length hash collision (~2⁻³² per changed row) and self-heals on its next change. |
| `void invalidate()` | Force the next flush to push everything. Call when identical encoded bytes would no longer decode to identical pixels: **decode-palette changes, panel re-init/GRAM loss.** |

The glass copy updates only for rows actually pushed, so bookkeeping stays exact across
partial flushes. Worst case (everything changed) = exactly one full-frame band.

---

## RM690B0

Minimal, self-contained ESP32 QSPI driver for RM690B0-class AMOLEDs (`RM690B0.h`, ESP32
only). Encodes the proven bring-up: vendor init, RGB332 8-bpp mode, manual CS held across
RAM bursts, **polling transfers only** (sustained queued/ISR QSPI DMA never completes on
this panel — don't "upgrade" it).

| Function | Description |
|---|---|
| `bool begin(spi_host_device_t host, const Pins &p, uint32_t clockHz, uint32_t maxTransferBytes, uint8_t madctl, uint16_t offX, uint16_t offY, uint8_t brightness)` | Bus + panel init. `Pins{d0,d1,d2,d3,sck,cs,rst,pmicEn}` (`pmicEn` < 0 = none). GPIO-matrix routing caps the effective clock near 40 MHz; requesting 80 is clean. |
| `void writeReg(uint8_t reg, const uint8_t *params, uint32_t len)` | Single-line register write (cmd `0x02`, register in the 24-bit address field). |
| `void setAddrWindow(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye)` | CASET/RASET/RAMWR with the module's panel offsets applied. |
| `void ramBegin()` / `void ramEnd()` | Assert/release CS around a pixel-RAM burst (CS must stay low across all chunks of one window's data). |
| `void ramWrite(const uint8_t *buf, uint32_t bytes, bool first)` | Blocking chunk write. `first=true` on the first chunk after `setAddrWindow` (it carries the QIO RAMWR command; continuation chunks are headerless). |
| `void ramWriteStart(const uint8_t *buf, uint32_t bytes, bool first)` | **Split-polling** start: begins the transfer and returns while the SPI DMA shifts it out — decode the *next* chunk into another buffer meanwhile. One transfer in flight; don't touch `buf` until… |
| `void ramWriteEnd()` | …this, which spins only for whatever wire time remains. |
| `void setBrightness(uint8_t b)` | Backlight/OLED brightness (register `0x51`). |
| `void displayOn()` / `void displayOff()` | DCS `0x29` / `0x28`. |
| `void invert(bool on)` | DCS `0x21` / `0x20`. |
| `void sleep()` / `void wake()` | Display-off + sleep-in / sleep-out + display-on (with the required delays). After `wake()` the GRAM content is unspecified — call `PackFlush::invalidate()`. |
| `spi_device_handle_t handle()` | The underlying ESP-IDF SPI device, for advanced use. |

The canonical pipeline (this is NanoPFD's production loop):

```cpp
canvas.encodeFrame(comp, lens, SLOT, 0, H);
flush.flush(comp, lens, 8, [&](int y0, int y1) {
  amo.setAddrWindow(0, y0, W - 1, y1);
  amo.ramBegin();
  bool inflight = false; int bb = 0;
  for (int yb = y0; yb <= y1; yb += LPC) {
    int n = min(LPC, y1 + 1 - yb);
    uint8_t *dst = bounce + bb * LPC * W;
    for (int j = 0; j < n; j++)                       // decode chunk n+1 …
      prle_decode_lut8p(comp + (yb+j)*SLOT, lens[yb+j], W, dst + j*W, lut, lutPair);
    if (inflight) amo.ramWriteEnd();                  // … while chunk n was on the wire
    amo.ramWriteStart(dst, n * W, yb == y0);
    inflight = true; bb ^= 1;
  }
  if (inflight) amo.ramWriteEnd();
  amo.ramEnd();
});
```

---

## ST7789

Minimal ESP32 4-wire-SPI driver for ST7789-class TFTs (`ST7789.h`, ESP32 only) —
the same shape as RM690B0, so the flush pipeline code is identical across panels.
Manual CS + manual DC; polling transfers with the split-polling overlap.

| Function | Description |
|---|---|
| `bool begin(spi_host_device_t host, const Pins &p, uint32_t clockHz, uint32_t maxTransferBytes, bool ips = true)` | Bus + panel init. `Pins{dc,cs,sck,mosi,rst,bl}` (`rst`/`bl` < 0 = none). IPS modules (the common case) need inversion on — that's the `ips` default. One panel per SPI host; two panels on separate hosts flush fully in parallel. |
| `void setRotation(uint8_t r, uint16_t offX, uint16_t offY)` | MADCTL quarter-turn (rot0 `0x00`, rot1 `0x60`, rot2 `0xC0`, rot3 `0xA0`) plus this rotation's **explicit** window origin — modules smaller than the 240×320 GRAM sit at rotation-dependent origins (a 240×280: `(0,20)` at rot 0/2, `(20,0)` at rot 1/3), so the caller passes the known-good pair. |
| `void setAddrWindow(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye)` | CASET/RASET with the rotation's offsets applied. |
| `void ramBegin()` / `void ramEnd()` | Issue RAMWR and hold CS/DC for the burst / release. |
| `void ramWrite(const uint8_t *buf, uint32_t bytes)` | Blocking pixel write, RGB565 in **wire order** (big-endian — bake the byte swap into your `prle_decode_lut32` pair LUT so decoded rows are wire-ready). |
| `void ramWriteStart(...)` / `void ramWriteEnd()` | Split-polling overlap, exactly as in RM690B0: start the transfer, decode the next chunk into the other bounce half, then end. |
| `void writeCmd(uint8_t cmd, const uint8_t *data = nullptr, uint32_t len = 0)` | Raw DCS command. |
| `displayOn/Off`, `invert(bool)`, `sleep()`, `wake()`, `backlight(bool)` | Standard controls. After `wake()` call `PackFlush::invalidate()`. |

RGB565 pair LUT for SPI panels (wire order) vs RGB parallel framebuffers (native):

```cpp
// SPI (ST7789):  word = swap16(pal[hi]) | swap16(pal[lo]) << 16   -> bytes are wire-ready
// RGB fb (LCD_CAM): word =        pal[hi] |        pal[lo] << 16   -> aligned fb stores
for (int b = 0; b < 256; b++) pair[b] = f(pal[b >> 4]) | (uint32_t)f(pal[b & 15]) << 16;
```

---

## PackRLE images

A whole picture as one blob — how a host sends an image (an album cover, an icon) to a
device. The blob is the rows' PackRLE streams back to back, each prefixed with its byte length
as a little-endian `uint16`. Width and height travel beside it. Worst case
`h * (2 + PRLE_STRIDE(w))` bytes. `tools/packrle.py` writes the same format from Python.

| Function | Description |
|---|---|
| `size_t prle_image_encode_idx8(const uint8_t *idx, int w, int h, uint8_t *out)` | Encode `w×h` palette indices (one byte each, 0–14). `out` must hold `h * (2 + PRLE_STRIDE(w))` bytes. Returns bytes written. |
| `bool prle_image_decode_flat(const uint8_t *enc, size_t nb, int w, int h, uint8_t *flat)` | Decode into packed rows of stride `PRLE_STRIDE(w)` — the layout `PackCanvas::drawIndexedBitmap` draws. False on a truncated or malformed blob (rows decoded before the fault are kept). |

```cpp
static uint8_t flat[96 * PRLE_STRIDE(96)];
if (prle_image_decode_flat(blob, blobLen, 96, 96, flat))
  canvas.drawIndexedBitmap(x, y, flat, 96, 96);
```

---

## PackMono

`PackMono.h`: a packed 4-bit canvas out to a **1-bit panel** (e-paper, mono LCD/OLED).
Each palette index gets an **ink level** 0–16 (0 = paper, 16 = solid ink); levels in between
become a 4×4 ordered (Bayer) dither on export. Greys stay flat, compressible palette colours on
the canvas and only turn into dots on the way out. Default: index 1 = ink, all others paper.

| Function | Description |
|---|---|
| `void setLevel(uint8_t index, uint8_t inkLevel)` / `uint8_t getLevel(uint8_t index)` | Ink level of a palette index (clamped to 16). Rebuilds the 2 KB lookup table. |
| `static size_t rowBytes(int w)` | Output bytes per row, `(w + 7) / 8`. |
| `void exportRows(PackCanvas &c, uint8_t *out, int y0, int y1, bool inkBit = true, bool mirrorX = false)` | Export rows `[y0, y1)` to `out + y * rowBytes(W)`: MSB-first, x = 0 in bit 7. `inkBit` = the bit value for ink; `mirrorX` flips each row. Canvas must be `packed4` with an even width; rows are flattened first. Cost: one table lookup per 2 pixels (an 800×480 frame is ~190k lookups, a few ms on an ESP32-S3). |
| `static bool diffRows(const uint8_t *a, const uint8_t *b, int w, int h, int *y0, int *y1)` | The band of rows where two exported frames differ; false when identical. Comparing the 1-bit output means a redraw that lands on the same dots costs nothing. |

**Memory:** 2 KB of lookup table + 16 bytes. The output frame is the caller's
(`rowBytes(W) * H`, 48 000 bytes at 800×480).

---

## UC8179

`UC8179.h`: `template <class Bus> class UC8179`. Non-blocking driver for **800×480 UC8179
e-paper**: Seeed reTerminal E1001, Waveshare 7.5" V2, GoodDisplay GDEY075T7. Portable (no
ESP-IDF or Arduino dependency): you supply the bus.

```cpp
struct Bus {
  void command(uint8_t c);                  // DC low, one byte
  void data(const uint8_t *p, size_t n);    // DC high, n bytes
  bool busy();                              // true while the panel is busy (mind the polarity)
  void reset(bool high);                    // drive RST
  void delayMs(uint32_t ms);                // only used by begin()
  uint32_t millis();
};
```

**Why non-blocking.** An e-paper refresh is up to 4 s of BUSY. Every wait here is a state:
`start*()` queues a refresh, `poll()` advances it by at most one step and returns at once,
`isIdle()` says when the glass is done. Only `begin()` (~300 ms) and the SPI pushes block.

**Frames** are 1 bit per pixel, MSB-first, 100 bytes a row, **bit 1 = ink** (what `PackMono`
exports by default). The driver keeps pointers to your frames; it does not copy them.

| Function | Description |
|---|---|
| `explicit UC8179(Bus &bus)` | |
| `void invert(bool on)` | The glass shows frames negative without it (the reTerminal E1001 does): set once, before `begin()`. Applies to every refresh kind. |
| `void begin()` | Hard reset + register setup (power, booster, panel setting, 800×480 resolution). Blocking, ~300 ms; each step waits for BUSY. Call again after `faulted()`. |
| `bool startFull(const uint8_t *frame)` | Queue a full refresh (the flashing waveform the panel picks from its temperature sensor; clears all ghosting). ~4 s. False if a refresh is running. |
| `bool startPartial(const uint8_t *frame, const uint8_t *prev, int y0, int y1, bool clean = false, int xb0 = 0, int xb1 = 100)` | Queue a refresh of rows `[y0, y1)` × byte columns `[xb0, xb1)` (8 px each; the controller's window is byte-aligned). `prev` = what is on the glass now (also a full frame): its window is re-sent as the "old" image every time, so a partial right after a full refresh compares against the truth. `clean = false`: the fast waveform, no flash. `clean = true`: the full waveform **inside the window only** — that window flashes and its ghosting is cleared. False if busy or the window is empty. |
| `void poll()` | Advance the running refresh. Cheap; call from your loop. |
| `bool isIdle()` / `Step step()` | Done? / which wait it is in (`IDLE`, `POWER_WAIT`, `REFRESH_WAIT`, `OFF_WAIT`). |
| `bool faulted()` | BUSY stayed up past `BUSY_TIMEOUT_MS` (12 s). Call `begin()` again. |
| `void stayPowered(bool on)` / `bool powered()` / `bool powerDown()` | Keep the charge pumps up between refreshes during a run of frequent updates (a ticking position bar): saves the ~100–200 ms of rails up/down on each. Turn it off and `powerDown()` when the run ends. |
| `void sleep()` | Deep sleep (lowest power). Needs `begin()` to wake. Only when idle. |
| `uint32_t partialsSinceFull()` / `uint32_t lastRefreshMs()` / `const Timing &lastTiming()` | Counters for your policy and telemetry. `Timing{power, send, refresh, off}` is where the last refresh spent its time, in ms. |

**Measured** on a reTerminal E1001 (ESP32-S3, SPI 10 MHz): full refresh 4.05 s; full-width
partial with rails up 1.1–1.2 s; a 104×17 px partial window 0.88 s (SPI 1 ms, waveform 865 ms).

**Fast register-LUT waveform** (`startFast`, `fastVcom`, `partialTemperature`). Verified against
the UC8179c datasheet (rev C0.6) before use: PSR bit 5 = REG, bit 4 = KW; LUTC/KW/WK/KK 60 bytes,
LUTWW/LUTBD 42 (7 groups used in KW mode); levels 00 GND, 01 VDH, 10 VDL, 11 VDHR; VDCS `0x82`
V = −0.10 − 0.05 × code; PLL `0x30` default 50 Hz (untouched). With DDX = 01 the tables are
{NEW,OLD} 01 → WK, 10 → KW, 00 → KK, 11 → WW, and CDI BDV = 11 routes the border to LUTBD.

| Function | Description |
|---|---|
| `bool startFast(frame, prev, y0, y1, xb0, xb1, uint8_t frames)` | A window refreshed with one phase of `frames` at VDL (K→W) / VDH (W→K); WW, KK and the border are all-zero tables (unchanged pixels never driven). `frames` 1–60. |
| `void fastVcom(uint8_t code, uint8_t table = 0)` | The fast path's VCOM_DC (clamped 0x12–0x40) and VCOM table (0: VCOM_DC for the phase; 1: all-zero). |
| `bool fastActive()` | Register LUTs are loaded: the next factory refresh of any kind starts with a hardware reset + `begin()` (vendor registers). |
| `void partialTemperature(uint8_t t)` | The forced temperature (`0xE5`) that picks which factory OTP waveform a partial uses (default 0x6E). |

**Measured** on a reTerminal E1001 (2026-09-23, 23.5 °C, camera vs untouched glass):
- *VCOM matters.* At −2.00 V every fast refresh greyed all glass **outside** the window (−30 levels
  over 20 refreshes; VCOM is one electrode for the whole panel). A sweep found −1.00 V (code 0x12,
  table 0) drifts ≈ 0 over 20 refreshes; that is the default.
- *Frame count* (10 frames = N_min at ≥ 90 % of factory contrast): 16 fr 0.62 s/transition (100 %),
  13 fr 0.58 s (96 %), **10 fr 0.51 s (93 %)**, 8 fr 0.46 s (88 %). Factory partials: 0.89 s.
- *On a 240×128 art window:* 344 ms a frame (waveform 320 ms), 1 frame/s for 5 minutes, untouched
  glass +1.5.
- *Haze in the window* after 20 / 40 / 80 / 160 full toggles: +3.1 / +2.8 / +3.3 / +4.0 (factory
  partial: ≈0 / +3 / +6.5 / +10). A full refresh restores it.
- The factory OTP bins (forcing `0xE5`): none faster than 0x6E's 864 ms waveform.

**Border.** Both refresh kinds drive the border (CDI `0x11` full / `0x19` partial). Waveshare's
partial sequence floats it (`0xA9`); with the rails kept up between updates a floating border
drifts into a grey outline round the picture.

**Memory:** the driver object is ~1 KB (a 1000-byte line buffer that batches SPI writes). Two
caller frames of 48 000 bytes (next + on-glass). **ESP32 with PSRAM:** SPI DMA cannot read every
PSRAM layout; keep the driver object (and so its line buffer) in internal RAM, or have your
`Bus::data()` copy through an internal DMA-capable buffer. A DMA read the SPI driver cannot do
reaches the controller as nothing, and the panel refreshes whatever its RAM held before.

---

## EInkGhost

`EInkGhost.h`: which refresh a change deserves, **cell by cell**. Knows nothing about the
controller: 1-bit frames in, a `Plan` out.

- `PARTIAL` — fast, no flash, leaves a little ghost behind.
- `CLEAN` — the full waveform inside a window (`UC8179::startPartial(..., clean = true, ...)`).
- `FULL` — the whole screen.

**Wear is counted per 8×8 cell** (one byte column × 8 rows), in **full toggles**: every pixel a
partial refresh flips adds 1/64 to its cell. A progress bar that moves one pixel a tick wears its
edge cell by 1/64 a tick; an art frame wears the cells its dirty rectangle changed by how much of
them it flipped. A change whose changed cells would pass the budget is done as a `CLEAN` window over those
cells (snapped to whole cells) — never a full refresh. (v1 counted per row, and charged a one-pixel
tick like a redraw of the whole row.) Early cleaning of an event on a third-worn cell is opt-in
(`setEarlyClean`), off by default.

| Function | Description |
|---|---|
| `EInkGhost(int height, int rowBytes, uint16_t budget = 160)` | For an 800×480 panel: `EInkGhost g(480, 100, 80)`. Budget = **full toggles** a cell takes before it is cleaned (a partial that flips k of a cell's 64 pixels costs k/64). |
| `Plan plan(const uint8_t *glass, const uint8_t *frame, bool forceFull = false, bool ambient = false)` | What to do to get `frame` onto glass that shows `glass`. `Plan{kind, y0, y1, xb0, xb1}` is the window: rows `[y0, y1)`, byte columns `[xb0, xb1)`. `NONE` when nothing changed. |
| `void done(const Plan &p)` | The glass finished `p`. Call once per completed refresh, after the `plan()` that produced it. `FULL` forgets all wear; `CLEAN` forgets the cells wholly inside its window; `PARTIAL` wears the cells it changed. |
| `int headroom(y0, y1, xb0, xb1)` / `int lastHeadroom()` | Full toggles a rectangle takes before its most worn cell reaches the budget / how many more partials *like the last one* its most worn changed cell takes. For a caller that **paces a ticking region** so its budget lasts until the next scheduled `FULL`. |
| `uint16_t maxWear()` / `uint16_t wear(int y)` | The most worn cell overall / in row `y`'s cell row. |
| `bool idleClean(Plan *out, float share = 0.25)` | The worn region (cells at ≥ `share` of the budget) a quiet screen could clean. |
| `void setBudget(uint16_t)` / `void setTemperature(float c)` / `uint16_t budget()` | Fast waveforms ghost more in the cold: below 18 °C the budget is ⅔, below 10 °C ½. NaN = unknown. |
| `void setEarlyClean(bool)` | Clean an event's cells when they are already a third worn (a flash while it redraws anyway). Off by default. |

**Ghosting, measured** on a reTerminal E1001 (UC8179 fast partial waveform, ~23 °C, 2026-09-23).
Blocks of 208×80 px were flipped between two checkerboards (every pixel flips every time: one full
toggle per refresh), then drawn white, and a camera compared them with an untouched white block:

| Full toggles | Grey vs. untouched white (0–255 camera levels) |
|---|---|
| 2 / 10 / 20 | ≈ 0 (clean) |
| 40 | +3 |
| 80 | +6 to +7 (faint) |
| 160 | +10 (a visible grey block) |
| 360 / 480 | +11 / +13 |

No pattern ghost appeared at any count: the fast waveform leaves a flat grey, which a full refresh
removes. Two cheaper cleans were measured and rejected: a *scrub* (the region's inverse, then the
true image, both fast) left it about 29 levels darker; a *clean window* (the full waveform inside
the window, 4.0 s) made its window whiter than before but greyed the REST of the glass by about 20.
So on this controller the least visible policy is: count wear in full toggles, keep the busiest
region under ~80 between whole-screen refreshes (pace its updates if needed: `lastHeadroom()`), and
let the one full refresh an hour be the only clean.

```cpp
UC8179<Bus> epd(bus);  EInkGhost ghost(480, 100);  EInkGhost::Plan running{EInkGhost::NONE};
// loop():
epd.poll();
if (!epd.isIdle()) return;
if (running.kind != EInkGhost::NONE) { ghost.done(running); memcpy(glass, frame, 48000); running.kind = EInkGhost::NONE; }
if (redrawn) {
  mono.exportRows(canvas, frame, 0, 480);
  EInkGhost::Plan p = ghost.plan(glass, frame, false, /*ambient*/ onlyTheClockMoved);
  if (p.kind == EInkGhost::NONE && quietForMinutes) ghost.idleClean(&p);
  bool ok = p.kind == EInkGhost::FULL ? epd.startFull(frame)
          : p.kind != EInkGhost::NONE && epd.startPartial(frame, glass, p.y0, p.y1,
                                                          p.kind == EInkGhost::CLEAN, p.xb0, p.xb1);
  if (ok) running = p;
}
```

A whole-screen `FULL` now and then is still the caller's choice (pass `forceFull`), e.g. at
start-up and at the first change after an hour.

**Memory:** 6 bytes per cell (36 000 bytes for 800×480), from `malloc`.

---

## PackInk

`PackInk.h`, namespace `packink`: textured **1-bit drawing** for e-paper pictures. Header-only,
C++11, no heap (the caller owns every buffer), no Arduino. Independent of `PackCanvas`: it draws
into its own 1-bit bitmaps and `blit()`s the result onto any canvas.

A shape is drawn in two steps: **rasterise** it into an `InkCoverage` (which pixels it touches),
then **`paint()`** that coverage into the picture with a `Pen` (texture, level, mode, seed).

**Determinism is the contract.** Integer arithmetic only, floor division (`fdiv`) and positive
modulo (`pmod`) written out, one 32-bit hash (`mix`, lowbias32), one sine table (`sin1024`). A
host-side reference of the same rules draws identical pixels; `tests/ink_host.cpp` checks pinned
numbers from it.

**Bitmaps**

| Type / function | Description |
|---|---|
| `struct InkBits { int16_t w, h, stride; uint8_t *bits; }` | 1-bit bitmap, MSB-first rows of `(w+7)/8` bytes, **1 = ink** (the `PackMono` / `UC8179` order). `InkBits(w, h, buf)` wraps a caller buffer of `InkBits::bytesFor(w, h)` bytes. `clear()`, `get(x, y)`, `put(x, y, ink)`, `flip(x, y)`, `span(y, x0, x1)`, `orWith(other)`. |
| `struct InkCoverage : InkBits { uint32_t stamps; }` | The pixels one shape covers, plus how many pen stamps it took (a render budget can count both). `reset()` before each shape. |

**Shapes** (all add to an `InkCoverage`; coordinates are integers, anything off the bitmap is clipped)

| Function | Description |
|---|---|
| `fillRect(c, x, y, w, h)` / `outlineRect(c, x, y, w, h, s)` | Rectangle, filled or with a border `s` px thick. |
| `fillEllipse(c, cx, cy, rx, ry)` / `outlineEllipse(c, cx, cy, rx, ry, s)` | Ellipse; the outline is a ring (outer minus inner spans), so it never has gaps. |
| `fillPoly(c, pts, n)` | Even-odd polygon from `n` points (`pts` = x0, y0, x1, y1, ...). |
| `line(c, x0, y0, x1, y1, s)` / `stamp(c, x, y, s)` | Bresenham line stamped with a round pen of size `s` (1 = one pixel, 2 = 2×2, larger = a disc). |
| `PolyPen pen(c, s); pen.to(x, y) ...; pen.end() / pen.close()` | A polyline with the same pen. Feed it from the point generators below. |
| `quadPoints`, `cubicPoints`, `arcPoints`, `wavePoints`, `spiralPoints` | Emit points of a quadratic / cubic Bézier, an arc (degrees), a sine wave, or a spiral to a callable `out(x, y)`; segment count from `curveSegments(length)` (4–32). |
| `text(c, x, y, size, s, len, turn, align)` | The classic 5×7 font at integer scale `size`, rotated by `turn` (0/90/180/270) about the top-left corner, aligned `LEFT` / `CENTER` / `RIGHT`. |

**Painting**

| Type / function | Description |
|---|---|
| `struct Pen { texture, level, param, mode, seed }` | `level` 0 (paper) .. 16 (solid ink); `param` = texture scale 2..64; `seed` keys the random textures. |
| `enum Texture` | `FLAT` (4×4 Bayer, `PackMono`'s own dither), `NOISE` (white noise), `GATED` (noise only inside smooth random patches), `CLOUD` (bilinear value noise), `HATCH`, `LINES`, `VLINES`, `CROSS`, `DOTS` (halftone). |
| `enum Mode` | `COVER` (ink and paper: occludes what is under it), `GLAZE` (adds ink only), `ERASE` (makes paper), `INVERT` (flips). |
| `bool inked(pen, xl, yl)` | The per-pixel texture rule, in layer-local coordinates. |
| `paint(dst, cov, pen, mx, my, cx0, cy0, cx1, cy1, mask, maskOutside)` | Apply a coverage to `dst`. Only pixels inside the clip rectangle `[cx0, cx1) × [cy0, cy1)`, and inside `mask` (or outside it, with `maskOutside`), are touched. Textures read `(x - mx, y - my)`, so a moved layer carries its texture along. |
| `blit(canvas, x, y, bits, inkColour, paperColour)` | Draw an `InkBits` onto any canvas with `drawFastHLine` (e.g. a `PackCanvas`), run by run. |

```cpp
static uint8_t artBuf[240 / 8 * 128], covBuf[240 / 8 * 128];
packink::InkBits art(240, 128, artBuf);
packink::InkCoverage cov(240, 128, covBuf);
packink::Pen pen;
art.clear();
cov.reset(); packink::fillRect(cov, 0, 0, 240, 128);
pen.texture = packink::CLOUD; pen.param = 24; pen.level = 7;
packink::paint(art, cov, pen, 0, 0);                 // a clouded sky
cov.reset(); packink::fillEllipse(cov, 180, 40, 22, 22);
pen.mode = packink::ERASE;
packink::paint(art, cov, pen, 0, 0);                 // a moon cut out of it
packink::blit(canvas, 80, 86, art, 1, 0);            // onto a PackCanvas: index 1 = ink
```

**Memory:** two bitmaps of `(w+7)/8 × h` bytes (3 840 each at 240×128) for the picture and the
coverage, plus one per mask. Nothing else. Complete sketch: `examples/InkArt_Basics`.

---

## Tools

| Tool | What it does |
|---|---|
| `tools/packrle.py` | The PackRLE codec in pure Python (standard library only): `encode_row`, `decode_row`, `encode_image`, `decode_image`. For hosts that send pictures to a device. `tests/host_standalone.cpp` can check a blob from it byte for byte (`ngfx_host image.bin W H`). |
| `tools/ttf2gfxfont.py` | A TrueType/OpenType font as a GFXfont header, needing only Pillow (no FreeType build). Glyphs are rendered without anti-aliasing, for 1-bit panels. `python3 tools/ttf2gfxfont.py Font.ttf 24 --last 0xFF --name Serif24 > Serif24.h`; `--preview out.png` to check it first. |

---

## Formats & contracts

**Palette.** 15 colors (indices 0–14). Nibble `0xF` is the RLE escape: never a color, never
in image data. Palettes themselves live at the *edges* (your decode LUTs / composite), so a
palette edit recolors frames without redrawing — but remember `PackFlush::invalidate()`.

**Packed layout.** Even x = **high** nibble; a byte is `(px[even] << 4) | px[odd]`. Packed
canvases are stored as **per-line slots** of `lineSlotBytes()` bytes (`PRLE_STRIDE(w)`
rounded even so run-lists stay 16-bit aligned). Size buffers with `bufBytes()`; address rows
via `lineBytes()` / `lineSlotBytes()` — never `(y*w + x) >> 1` global math.

**Raw-access contract.** Before reading or poking a packed row's bytes: `flatten(y0, y1)`
(or use `getBuffer()`, which flattens everything). `getPixel`, `blitRect`, and
`encodeFrame` handle run-form lines themselves — no flattening needed around them.

**Rotation contract.** Packed canvases render native-orientation; compose quarter-turn
mounting at the panel (MADCTL) — quarter-turns add, so canvas-rot0 + panel-rotN reproduces
any canvas-rotN + panel-rot0 image. The float rotation *matrix* works on every format.

**Threading.** One writer per canvas. `encodeFrame` output may be consumed on another core
once drawing is done (that's NanoPFD's two-core pipeline). `PackFlush` is single-owner.

**Verification.** The consumer project's fuzz oracle
([NanoPFD `tools/gfxbench`](https://github.com/MoonFingerRF/NanoPFD/tree/main/tools/gfxbench))
proves the packed/dual paths byte-identical to stock Adafruit rendering across millions of
random ops — including text at every parity, `blitRect`, `drawIndexedBitmap`, `vScroll`,
and `getPixel` — under ASan/UBSan. Run it after modifying the canvas or codec.
