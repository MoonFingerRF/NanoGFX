# NanoGFX

**15-color packed-nibble graphics for microcontrollers** — an Adafruit_GFX-compatible
canvas that stores **2 pixels per byte**, fuses most of an RLE encode into the drawing
itself, renders text with a dual-parity blitter, and pushes frames with **split-polling
QSPI writes** that send **only the rows that changed**.

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

See `examples/` for complete sketches, including the full ESP32-S3 AMOLED pipeline.

## Examples

- **`PackedCanvas_Basics`** — 2 px/byte in practice: draw a gauge face, compare RAM and
  fill/blit cost against an 8-bit canvas.
- **`DualMode_Runs`** — watch lines stay in run form: draw, then `encodeFrame()` and
  print each line's encoded size; scattered pixels demote exactly the lines they touch.
- **`FastText_DualParity`** — the dual-parity text blitter vs the stock per-pixel path,
  timed side by side at sizes 1–4 and both parities.
- **`RM690B0_DirtyPush`** (ESP32-S3) — the complete NanoPFD pipeline on a LilyGO T4-S3:
  dual-mode canvas → `encodeFrame` → `PackFlush` dirty bands → split-polling QSPI push,
  with per-frame stats printed (rows pushed, decode µs, wire µs).

## Rules of the road

- 15 colors max (`NUM_COLORS ≤ 15`); nibble `0xF` is the codec's escape and never a color.
- Set `packed4` / `dualMode` before the first drawing call on an instance.
- Reads (composite, blit, `getBuffer()`) on a dual-mode canvas go through `flatten()` /
  `rawBuffer()` — see the "raw-access contract" in `PackCanvas.h`.
- `PackFlush::invalidate()` whenever identical encoded bytes would stop meaning identical
  pixels: decode-palette edits, panel re-init, brightness-independent GRAM loss.
- The RM690B0 driver is polling by design; the split calls are the overlap mechanism.
  One transfer in flight at a time; don't touch the buffer until `ramWriteEnd()`.

## Provenance

Extracted from [NanoPFD](https://github.com/MoonFingerRF/NanoPFD)'s renderer after the
optimizations proved out on hardware (three different panels: dual SPI ST7789s, an RGB
ST7701S, and the QSPI RM690B0 AMOLED). The canvas ships with a fuzz oracle in that
project (`tools/gfxbench`) proving byte-identical behavior against stock Adafruit_GFX
across formats, rotations, and the text blitter — 2.2M checks per run.

MIT license.
