# NanoGFX API Reference

Complete reference for every public function in the library. One include gets the core:

```cpp
#include <NanoGFX.h>    // PackCanvas + PackRLE + PackFlush
#include <RM690B0.h>    // (optional, ESP32) QSPI AMOLED driver
```

Contents: [PackCanvas](#packcanvas) · [PackRLE](#packrle) · [PackFlush](#packflush) ·
[RM690B0](#rm690b0) · [ST7789](#st7789) · [Formats & contracts](#formats--contracts)

---

## PackCanvas

`class PackCanvas : public GFXcanvas8`

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
