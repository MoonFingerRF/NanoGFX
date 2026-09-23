# NanoGFX

**15-color packed-nibble graphics for microcontrollers** — an Adafruit_GFX-compatible
canvas that stores **2 pixels per byte**, fuses most of an RLE encode into the drawing
itself, renders text with a dual-parity blitter, and pushes frames with **split-polling
QSPI writes** that send **only the rows that changed**.

It also drives **1-bit e-paper**: the same canvas exports to an 800×480 UC8179 panel
through an ordered dither, a per-region ghosting policy picks partial / clean-window / full
refreshes, and the driver never blocks your loop while the glass updates.

Builds **with or without Arduino**: on Arduino it sits on Adafruit_GFX; on ESP-IDF, ESPHome
or a desktop compiler it uses its own Adafruit-compatible base (`NanoGFXBase.h`).

Battle-tested in [NanoPFD](https://github.com/MoonFingerRF/NanoPFD), an ESP32-S3 glass
cockpit: a 450×600 AMOLED runs **46–48 fps** with a WiFi access point and two Remote ID
receivers (BLE + WiFi) live on the same chip.

## Why 15 colors?

A 4-bit canvas has 16 nibble values. NanoGFX spends **one of them (`0xF`) as an RLE
escape code**, leaving 15 palette colors. That one sacrifice buys a codec whose worst
case is *exactly* the packed framebuffer size (never larger), and whose common case —
instrument panels, gauges, maps, UI — is 10–50× smaller. Compression this cheap changes
what a small MCU can drive:

| | classic 8-bit canvas | NanoGFX packed | NanoGFX dual-mode |
|---|---|---|---|
| 450×600 canvas RAM | 270 KB | **135 KB** | 135 KB + ~9 KB run-lists |
| draw a 100 px H-line | 100 byte writes | **50 byte writes** | **1 run-list entry** |
| RLE-encode a drawn line | full pixel scan | word-batched scan | **near-free** (already runs) |
| fill / clear | memset W×H | memset W×H/2 | **one run per line** |

## Supported displays

| Display | Driver | Notes |
|---|---|---|
| RM690B0 QSPI AMOLED (LilyGO T4-S3, 450×600) | `RM690B0.h` (ESP32) | split-polling QSPI, dirty rows only |
| ST7789 SPI TFTs (240×240/280/320) | `ST7789.h` (ESP32) | same pipeline, RGB444 wire option |
| RGB parallel panels (ST7701S etc., LCD_CAM) | none needed | decode dirty rows into the scanned framebuffer |
| **UC8179 7.5" e-paper, 800×480** (Seeed reTerminal E1001, Waveshare 7.5" V2, GoodDisplay GDEY075T7) | `UC8179.h` (any MCU, your bus) | non-blocking full / partial / clean-window refresh |
| Any other 1-bit panel (SSD16xx e-paper, mono OLED/LCD) | your driver | `PackMono` exports MSB-first 1-bit rows |

## The pieces

- **`PackCanvas`** — one class, three formats chosen *per instance*:
  - `packed4 = false` — classic 1 byte/px palette canvas (GFXcanvas8 semantics), with
    rotation fast paths.
  - `packed4 = true` — 4-bit packed, 2 px/byte. Whole-byte fills, pair-wise blits.
  - `dualMode = true` — each line lives as a **run-list** until the drawing pattern says
    otherwise: H/V line spans splice into runs; scattered pixels *explode* the line to
    flat form, and a sticky-flat policy stops thrash (a line that exploded starts the
    next frame flat). Fills and encodes on run-form lines skip pixel work entirely.
  - **Dual-parity text**: the classic 5×7 font renders a *row at a time* — a
    parity-aligned nibble mask built on the fly, blitted with whole-byte
    read-modify-writes — ~4–5× fewer operations per glyph than per-pixel drawing, at any
    x alignment and sizes 1–4. Byte-identical output to stock Adafruit_GFX (fuzz-proven).
  - **Sprites & more**: canvas-to-canvas `blit`/`blitRect` with a transparent index
    (memcpy rows when aligned, span emission from run-form rows), `drawIndexedBitmap`
    for flash-resident 2 px/byte images, `vScroll` that scrolls by **moving line
    records** (three memmoves, rows stay compressed), and a format-aware `getPixel`
    that walks run-lists without disturbing them.
- **`PackRLE`** — the codec. Encode from packed nibbles, 8-bit indices, or run-lists;
  decode to nibbles, 8-bit palette (with a 256-entry pair LUT: 2 px per lookup), or
  32-bit colors. Worst case = flat packed size, guaranteed.
- **`PackFlush`** — dirty-band frame flushing. Keeps an "on-glass" copy of each row's
  *encoded* stream (comparing compressed bytes, not pixels), memcmps incoming rows,
  coalesces changes into bands, and hands each band to your push callback. Panels with
  persistent GRAM only ever receive what changed — a static frame costs ~0 wire time.
- **`RM690B0`** (ESP32) — QSPI AMOLED driver. `ramWriteStart()`/`ramWriteEnd()` **split a
  polling transfer**: the SPI DMA shifts chunk *n* out while your CPU decodes chunk
  *n+1* — the transfer time hides under compute without the interrupt-driven queued
  path (which some panels, including this one, cannot sustain).
- **`PackMono`** — a packed canvas out to a **1-bit panel**. Each palette index gets an ink
  level 0–16; greys become a 4×4 ordered dither on export, so a grey fill stays one flat,
  compressible colour on the canvas. `diffRows()` finds the rows two exported frames differ in.
- **`UC8179`** — **non-blocking e-paper driver** for 800×480 UC8179 panels. `startFull()` /
  `startPartial()` queue a refresh and `poll()` advances it one step at a time, so a 4 s
  refresh never stalls buttons or networking. Partial windows are narrowed to the changed
  byte columns; a *clean* window runs the full waveform inside the window only; the rails can
  stay up for a run of 1 Hz updates. Bring your own bus (Arduino SPI, ESP-IDF, ESPHome, a
  simulator).
- **`EInkGhost`** — **e-paper ghosting policy, per region.** Tracks wear per row (a ticking
  clock adds a little, a real change adds more) and answers PARTIAL, CLEAN window, or FULL, so
  a busy corner gets cleaned on its own instead of flashing the whole screen.
- **PackRLE images + `tools/packrle.py`** — a whole picture as one blob (rows with length
  prefixes), encoded on a host in pure Python and decoded on the device.
- **`PackInk`** — textured 1-bit drawing for e-paper art: shapes, curves and text stamped as
  coverage, then painted with a texture (Bayer, noise, clouds, hatching, halftone dots) and a mode
  (cover, glaze, erase, invert) through clips and masks. Deterministic, so a host renderer can
  match it pixel for pixel. See [PackInk](#packink--textured-1-bit-drawing-e-paper-art) below.
- **`tools/ttf2gfxfont.py`** — any TrueType font as a GFXfont header, no FreeType build needed.
- **`ST7789`** (ESP32) — the same driver shape for classic 4-wire-SPI TFTs
  (240×240/280/320 modules), with the same split-polling overlap and explicit
  per-rotation window origins. RGB *parallel* panels (LCD_CAM framebuffers) need no
  driver at all: PackFlush + `prle_decode_lut32` decode dirty rows straight into the
  scanned framebuffer — that's how NanoPFD runs all three of its panel types on one
  pipeline.

Measured on NanoPFD's ESP32-S3 @ 240 MHz, 450×600 @ 8 bpp wire format, radios live:

| stage | naive | NanoGFX |
|---|---|---|
| full-frame push (decode + QSPI) | 13.3 ms serialized | **7.7 ms** (split-polling overlap) |
| quiescent frame push | 13.3 ms | **2–5 ms** (0–32 of 600 rows sent) |
| whole-panel fps (PFD + moving map + radios) | 12 | **46–48** |

## Quick start

```cpp
#include <NanoGFX.h>

PackCanvas canvas(240, 240, /*alloc*/ true);

void setup() {
  canvas.packed4  = true;   // 2 px/byte — set BEFORE first use
  canvas.dualMode = true;   // lines live as run-lists until scattered pixels arrive
  canvas.fillScreen(0);     // one run per line, no memset

  canvas.drawFastHLine(20, 100, 200, 3);   // splices ONE run into line 100
  canvas.fillRect(60, 60, 120, 40, 5);     // row-major: one span splice per line
  canvas.setCursor(30, 30);
  canvas.setTextColor(7);
  canvas.print("NanoGFX");                 // dual-parity blitter, whole-byte writes
}
```

Encode + flush only what changed (any panel with persistent GRAM):

```cpp
uint8_t  comp[240 * PRLE_STRIDE(240)];     // per-line slots; worst case guaranteed
uint16_t lens[240];
uint8_t  glass[240 * PRLE_STRIDE(240)];    // PackFlush's on-glass copy
uint16_t glassLens[240];
PackFlush flush;

void setup2() {
  flush.begin(glass, glassLens, 240, PRLE_STRIDE(240));
}

void frame() {
  draw(canvas);                            // your drawing
  canvas.encodeFrame(comp, lens, PRLE_STRIDE(240), 0, 240);   // run-form lines: near-free
  flush.flush(comp, lens, /*gap*/ 8, [&](int y0, int y1) {
    panelSetWindow(0, y0, 239, y1);        // your panel's window command
    pushRows(comp, lens, y0, y1);          // decode + write rows y0..y1
  });
}
```

E-paper (800×480 UC8179), without blocking the loop:

```cpp
#include <NanoGFX.h>
#include <UC8179.h>

struct Bus { /* command(), data(), busy(), reset(), delayMs(), millis() over your SPI */ };
Bus bus;
UC8179<Bus> epd(bus);
PackCanvas canvas(800, 480, false);   // 192 KB packed: put it in PSRAM
PackMono mono;                        // palette index -> ink level -> dither
EInkGhost ghost(480, 100);            // per-row wear -> PARTIAL / CLEAN / FULL
uint8_t *frame, *glass;               // 48 KB each: next frame, what the glass shows

void loop() {
  epd.poll();                                       // returns at once
  if (!epd.isIdle() || !somethingChanged()) return;
  draw(canvas);
  mono.exportRows(canvas, frame, 0, 480);
  EInkGhost::Plan p = ghost.plan(glass, frame);
  // start p with epd.startFull / epd.startPartial(frame, glass, p.y0, p.y1, clean, p.xb0, p.xb1);
  // when isIdle() again: ghost.done(p) and copy frame -> glass
}
```

`examples/UC8179_EInk` is the complete version.

See `examples/` for complete sketches, including the full ESP32-S3 AMOLED pipeline —
and **[docs/API.md](docs/API.md) for the complete function reference** (every PackCanvas /
PackRLE / PackFlush / RM690B0 / ST7789 / PackMono / UC8179 / EInkGhost call, the standalone
and ESP-IDF builds, plus the format contracts).

## Examples

- **`PackedCanvas_Basics`** — 2 px/byte in practice: draw a gauge face, compare RAM and
  fill/blit cost against an 8-bit canvas.
- **`DualMode_Runs`** — watch lines stay in run form: draw, then `encodeFrame()` and
  print each line's encoded size; scattered pixels demote exactly the lines they touch.
- **`FastText_DualParity`** — the dual-parity text blitter vs the stock per-pixel path,
  timed side by side at sizes 1–4 and both parities.
- **`Sprites_Blit`** — canvas sprites: transparent blits, sub-rect blits, flash-resident
  packed images, and format-aware pixel reads.
- **`ScrollingPlot`** — a live strip-chart on `vScroll`: scrolling moves line *records*
  (microseconds for a full region) and the rows stay compressed.
- **`RM690B0_DirtyPush`** (ESP32-S3) — the complete NanoPFD pipeline on a LilyGO T4-S3:
  dual-mode canvas → `encodeFrame` → `PackFlush` dirty bands → split-polling QSPI push,
  with per-frame stats printed (rows pushed, decode µs, wire µs).
- **`UC8179_EInk`** (ESP32-S3 + PSRAM) — a 7.5" e-paper panel (pins for the Seeed reTerminal
  E1001): a grey panel dithered by `PackMono`, a 1 Hz progress bar as a tiny partial window,
  `EInkGhost` choosing clean windows as rows wear, refresh timings on the serial log.

## Building

- **Arduino:** install Adafruit GFX Library, then this library; open an example.
- **ESP-IDF / ESPHome:** the repo is an ESP-IDF component (`nanogfx: { path: ../NanoGFX }`).
  Adafruit_GFX is not needed; `NanoGFXBase.h` is used automatically.
- **Host:** `c++ -std=c++17 -I src tests/host_standalone.cpp -o ngfx_host && ./ngfx_host`
  (canvas, text, PackMono, PackRLE images, the UC8179 state machine on a simulated bus).

## Rules of the road

- 15 colors max (`NUM_COLORS ≤ 15`); nibble `0xF` is the codec's escape and never a color.
- Set `packed4` / `dualMode` before the first drawing call on an instance.
- Reads (composite, blit, `getBuffer()`) on a dual-mode canvas go through `flatten()` /
  `rawBuffer()` — see the "raw-access contract" in `PackCanvas.h`.
- `PackFlush::invalidate()` whenever identical encoded bytes would stop meaning identical
  pixels: decode-palette edits, panel re-init, brightness-independent GRAM loss.
- E-paper: call `EInkGhost::done()` once per completed refresh and copy the frame to your
  on-glass buffer then; the next partial needs the true on-glass image. On ESP32 keep the
  `UC8179` object in internal RAM (its line buffer is what SPI DMA reads).
- The RM690B0 driver is polling by design; the split calls are the overlap mechanism.
  One transfer in flight at a time; don't touch the buffer until `ramWriteEnd()`.

## PackInk — textured 1-bit drawing (e-paper art)

`#include <PackInk.h>` (header-only, no heap, C++11, no Arduino needed). A second drawing layer
for 1-bit panels, where an intensity has to become dots *in a chosen way*:

- **`InkBits` / `InkCoverage`** — 1-bit bitmaps in PackMono/UC8179 order (MSB-first, 1 = ink);
  a coverage is the set of pixels one shape touches plus its pen-stamp count.
- **Shapes** — `fillRect`/`outlineRect`, `fillEllipse`/`outlineEllipse` (rings by span
  subtraction), even-odd `fillPoly`, Bresenham `line` stamped with a round pen (1, 2x2, disc),
  `PolyPen` polylines fed by `quadPoints`/`cubicPoints`/`arcPoints`/`wavePoints`/`spiralPoints`,
  and the classic 5x7 font at any integer scale in four rotations (`text`).
- **Textures** (`Pen.texture`, level 0..16, scale `param`): `FLAT` (4x4 Bayer — PackMono's own
  dither), `NOISE` (white noise), `GATED` (noise only inside smooth random patches), `CLOUD`
  (bilinear value noise), `HATCH`, `LINES`, `VLINES`, `CROSS`, `DOTS` (halftone).
- **Modes** — `COVER` (ink and paper: occludes), `GLAZE` (ink only), `ERASE`, `INVERT`; `paint()`
  takes a clip rectangle, an optional mask bitmap (inside/outside) and a texture offset so a moving
  layer carries its texture with it.
- **`blit()`** — an `InkBits` onto any canvas with `drawFastHLine` (PackCanvas included), in runs.

**Determinism is the contract**: integer arithmetic, floor division (`fdiv`) and positive modulo
(`pmod`) spelled out, one 32-bit hash (`lowbias32`), one copied sine table — so a Python reference
of the same rules draws identical pixels (`tests/ink_host.cpp` checks pinned numbers from it).
First user: nightly generative art on a 7.5" e-paper panel (reTerminal E1001).
Example: `examples/InkArt_Basics`.

## Provenance

Extracted from [NanoPFD](https://github.com/MoonFingerRF/NanoPFD)'s renderer after the
optimizations proved out on hardware (three different panels: dual SPI ST7789s, an RGB
ST7701S, and the QSPI RM690B0 AMOLED). The canvas ships with a fuzz oracle in that
project (`tools/gfxbench`) proving byte-identical behavior against stock Adafruit_GFX
across formats, rotations, and the text blitter — 2.2M checks per run.

MIT license. `NanoGFXBase.h` carries Adafruit_GFX code under its BSD license (`LICENSE-Adafruit-GFX`).
