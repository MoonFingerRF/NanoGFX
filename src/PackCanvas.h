// ============================================================================
//  PackCanvas — a reusable palette canvas for Adafruit_GFX, in three formats
//  selected PER INSTANCE:
//
//    8-bit (packed4=false)          : classic 1 byte/px GFXcanvas8 semantics.
//    4-bit packed (packed4=true)    : 2 px/byte nibbles — half the RAM/traffic.
//    dual-mode (+ dualMode=true)    : per-line RUNS/FLAT with self-tuning
//                                     explode + sticky-flat (see below) — most
//                                     of an RLE encode is fused into drawing.
//
//  Pairs with PackRLE.h (the 15-color + escape-nibble RLE codec): encodeFrame()
//  emits each line's PackRLE stream, near-free for lines still in run form.
//
//  Dependencies: Adafruit_GFX only. Rotation matrix support (setRotationMatrix)
//  rotates draws about an arbitrary point (compass cards etc.). textScale
//  multiplies setTextSize for resolution-independent layouts.
//
//  Part of NanoGFX (https://github.com/MoonFingerRF/NanoGFX);
//  reusable in any Adafruit_GFX project. See PackRLE.h for the stream format.
// ============================================================================
#pragma once
#include <Adafruit_GFX.h>
#include "PackRLE.h"
#include "PackFont5x7.h"

// ============================================================================
//  PackCanvas: a palette canvas that is EITHER 8-bit (1 px/byte)
//  or 4-bit PACKED (2 px/byte), selected per-instance by `packed4`; and, for packed
//  canvases, optionally DUAL-MODE per line (`dualMode`):
//
//    RUNS — the line's slot holds a sorted run-list of uint16_t (x_start<<4)|color
//           entries (a run ends at the next entry's x; the last ends at W).
//           fillScreen = 1 run/line, O(H) — no pixel writes at all. Span fills
//           SPLICE into the list (O(runs), same-color merge both edges); single
//           pixels are 1-px splices. Most of the PackRLE *encode* is thereby fused
//           into drawing: encodeFrame() emits RUNS lines via prle_encode_runs with
//           no pixel scan.
//    FLAT — plain packed4 nibbles; every op is the original packed4 fast path.
//           A splice that would pass RUN_CAP EXPLODES the line to FLAT (render +
//           memcpy, once) — scattered-heavy lines (moving map, text rows) demote
//           themselves and never pay splice costs again that frame, so dual-mode
//           is never worse than the flat canvas.
//
//  dualMode == false (default): every line stays FLAT forever — bit-identical
//  behavior to the original packed4 canvas. Only fillScreen() consults the flag;
//  the pixel paths just see all-FLAT side-state.
//
//  RAW-ACCESS CONTRACT (attitude sampler + any composite/blit that reads bytes):
//  call flatten(y0, y1) before poking/reading rows via rawBuffer(); getBuffer()
//  flattens the whole canvas and is then byte-identical to a flat packed4 canvas.
//
//  4-bit nibble convention (WIDTH even -> rows byte-aligned): even x -> HIGH
//  nibble; a byte is (px[even]<<4)|px[odd]. The compass uses the rotation MATRIX
//  (setRotationMatrix), handled in drawPixel for all formats.
//  Fuzz coverage: tools/gfxbench/run_tests.sh (codec roundtrips + 10k-frame canvas
//  equivalence incl. splice/merge/explode paths, byte-identical vs flat).
// ============================================================================
class PackCanvas : public GFXcanvas8 {
public:
  bool packed4  = false;       // false = 8-bit (default), true = 4-bit packed
  bool dualMode = false;       // packed4 only: enable per-line RUNS/FLAT (see above)
  // SPLICE POLICY (device-measured; the naive splice-everything policy collapsed to
  // 9 fps): only LINE SPANS (H/V line segments) splice into RUNS lines — their lists
  // stay tiny, so a splice is a ~16-entry rebuild. SCATTERED pixels (drawPixel /
  // writePixel / putNib: text glyphs, map Bresenham) EXPLODE the line on first touch
  // and then run at full flat speed. Pure-span rows (fills, frames, blank bands) are
  // what dual-mode is for — they never see a pixel and encode for free.
  static constexpr int RUN_CAP    = 16;    // splice list cap: small = cheap rebuilds,
                                           // busy rows demote on their first pixel anyway
  static constexpr int NANO_MAX_W = 1024;  // stack render temp = PRLE_STRIDE(1024) = 512 B

  PackCanvas(uint16_t w, uint16_t h, bool allocate_buffer = true) : GFXcanvas8(w, h, allocate_buffer) {
    setRotationMatrix();
    // Slot bytes per line, rounded EVEN: run-lists overlay the slot as uint16_t
    // entries, so an odd PRLE_STRIDE (any w with w/2 odd, e.g. 450 -> 225) would
    // put every odd row's list at a misaligned address — UB, and a real penalty
    // on cores that trap or fix up unaligned halfword stores.
    lineStride = (uint16_t)((PRLE_STRIDE(w) + 1) & ~1);
    effCap = (int)(lineStride / 2);        // never more entries than the slot holds
    if (effCap > RUN_CAP) effCap = RUN_CAP;
    if (w > NANO_MAX_W) effCap = 0;        // degenerate: permanent FLAT (still correct)
    lineMode   = new uint8_t[h];
    runCount   = new uint16_t[h];
    stickyFlat = new uint8_t[h];
    memset(lineMode, M_FLAT, h);
    memset(runCount, 0, (size_t)h * sizeof(uint16_t));
    memset(stickyFlat, 0, h);
  }
  ~PackCanvas() { delete[] lineMode; delete[] runCount; delete[] stickyFlat; }

  float rotA, rotB, offX, offY;
  bool  identity;
  void setRotationMatrix(float angle = 0, float x = 0, float y = 0) {
    rotA = cosf(angle); rotB = sinf(angle); offX = x; offY = y;
    identity = (angle == 0.0f && x == 0.0f && y == 0.0f);
  }

  // 4-bit "fast drawPixel": clipped, unrotated; mode-dispatched in dual-mode.
  inline void putNib(int16_t x, int16_t y, uint8_t c) {
    if ((uint16_t)x >= (uint16_t)WIDTH || (uint16_t)y >= (uint16_t)HEIGHT) return;
    if (packed4) { pix4(x, y, c); return; }
    int32_t i = (int32_t)y * WIDTH + x;    // 8-bit instances: original nibble RMW semantics
    uint8_t *p = buffer + (i >> 1);
    if (i & 1) *p = (*p & 0xF0) | (c & 0x0F);
    else       *p = (*p & 0x0F) | ((c & 0x0F) << 4);
  }

  // Logical -> native coords for the 8-bit paths (setRotation support: BOARD_A runs
  // its two panels at rotations 2 and 1). Packed canvases are native/rot-0 by contract.
  inline void rot8(int16_t &x, int16_t &y) {
    switch (rotation) {
      case 1: { int16_t t = x; x = WIDTH - 1 - y; y = t; } break;
      case 2: x = WIDTH - 1 - x; y = HEIGHT - 1 - y; break;
      case 3: { int16_t t = x; x = y; y = HEIGHT - 1 - t; } break;
    }
  }
  void drawPixel(int16_t x, int16_t y, uint16_t color) {
    if (!identity) {                                          // compass rotation matrix
      float x_ = x - offX, y_ = y - offY;
      int16_t tx = (int16_t)(rotA * x_ - rotB * y_ + offX);
      int16_t ty = (int16_t)(rotA * y_ + rotB * x_ + offY);
      if (packed4) {                                          // packed = native coords
        if ((uint16_t)tx >= (uint16_t)WIDTH || (uint16_t)ty >= (uint16_t)HEIGHT) return;
        pix4(tx, ty, (uint8_t)color);
      } else {
        GFXcanvas8::drawPixel(tx, ty, color);                 // rotation-aware base
      }
      return;
    }
    if (packed4) {
      if ((uint16_t)x >= (uint16_t)WIDTH || (uint16_t)y >= (uint16_t)HEIGHT) return;
      pix4(x, y, (uint8_t)color);
      return;
    }
    if ((uint16_t)x >= (uint16_t)_width || (uint16_t)y >= (uint16_t)_height) return;
    rot8(x, y);
    buffer[(int32_t)y * WIDTH + x] = (uint8_t)color;
  }
  void fastDrawPixel(int16_t x, int16_t y, uint16_t color) {
    if (packed4) putNib(x, y, (uint8_t)color);
    else GFXcanvas8::drawPixel(x, y, color);
  }
  // Text/glyphs are drawn a pixel at a time via writePixel(). Override it so the common
  // identity case skips the virtual drawPixel() dispatch and the rotation re-check on
  // every glyph pixel. Falls back to drawPixel() for the rotated ND labels.
  void writePixel(int16_t x, int16_t y, uint16_t color) {
    if (identity) {
      if (packed4) {
        if ((uint16_t)x >= (uint16_t)WIDTH || (uint16_t)y >= (uint16_t)HEIGHT) return;
        pix4(x, y, (uint8_t)color);
        return;
      }
      if ((uint16_t)x >= (uint16_t)_width || (uint16_t)y >= (uint16_t)_height) return;
      rot8(x, y);
      buffer[(int32_t)y * WIDTH + x] = (uint8_t)color;
      return;
    }
    drawPixel(x, y, color);
  }

  void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
    if (!identity) { for (int16_t i = 0; i < w; i++) drawPixel(x + i, y, color); return; }
    if (!packed4 && rotation != 0) { GFXcanvas8::drawFastHLine(x, y, w, color); return; }
    if (w <= 0 || y < 0 || y >= HEIGHT) return;
    if (x < 0) { w += x; x = 0; }
    if (x + w > WIDTH) w = WIDTH - x;
    if (w <= 0) return;
    if (!packed4) { memset(buffer + (int32_t)y * WIDTH + x, (uint8_t)color, w); return; }
    uint8_t c = color & 0x0F;
    if (lineMode[y] == M_RUNS) { splice(y, x, x + w, c); return; }   // O(runs), no pixels touched
    uint8_t *row = slot(y);                // FLAT: odd lead / byte memset / even tail
    if (x & 1) { uint8_t *p = row + (x >> 1); *p = (*p & 0xF0) | c; x++; w--; }
    int bytes = w >> 1;
    if (bytes) { memset(row + (x >> 1), (uint8_t)((c << 4) | c), bytes); x += bytes * 2; w -= bytes * 2; }
    if (w == 1) { uint8_t *p = row + (x >> 1); *p = (*p & 0x0F) | (c << 4); }
  }

  void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
    if (!identity) { for (int16_t i = 0; i < h; i++) drawPixel(x, y + i, color); return; }
    if (!packed4 && rotation != 0) { GFXcanvas8::drawFastVLine(x, y, h, color); return; }
    if (h <= 0 || x < 0 || x >= WIDTH) return;
    if (y < 0) { h += y; y = 0; }
    if (y + h > HEIGHT) h = HEIGHT - y;
    if (h <= 0) return;
    if (!packed4) {
      uint8_t *pp = buffer + (int32_t)y * WIDTH + x, c = (uint8_t)color;
      for (int16_t i = 0; i < h; i++) { *pp = c; pp += WIDTH; }
      return;
    }
    // Per-line: a V line is a SPAN op, so RUNS lines take a 1-px splice (cheap at
    // RUN_CAP-sized lists) instead of exploding — frame verticals through blank
    // bands keep those rows in run form. FLAT lines take the strided nibble RMW.
    uint8_t c = color & 0x0F;
    for (int16_t i = 0; i < h; i++) {
      int yy = y + i;
      if (lineMode[yy] == M_RUNS) { splice(yy, x, x + 1, c); continue; }
      uint8_t *p = slot(yy) + (x >> 1);
      if (x & 1) *p = (*p & 0xF0) | c;
      else       *p = (*p & 0x0F) | (uint8_t)(c << 4);
    }
  }

  // Row-major fillRect: one drawFastHLine per row instead of Adafruit's per-COLUMN
  // VLine loop — a memset per row on FLAT lines and a single splice per row on RUNS
  // lines (Adafruit's column order would cost w separate ops per line). This is also
  // what keeps big mask rects span-shaped for dual-mode.
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    // Fast path only for the clean case; every quirk case (negative spans, active
    // matrix) delegates to the base column-major chain so legacy sign-normalization
    // semantics stay bit-exact with the historical canvases.
    if (!identity || w <= 0 || h <= 0) { Adafruit_GFX::fillRect(x, y, w, h, color); return; }
    for (int16_t j = 0; j < h; j++) drawFastHLine(x, y + j, w, color);
  }

  void fillScreen(uint16_t color) {
    if (!packed4) { memset(buffer, (uint8_t)color, (int32_t)WIDTH * HEIGHT); return; }
    uint8_t c = color & 0x0F;
    if (dualMode && effCap >= 1) {
      // STICKY-FLAT: a line that exploded last frame almost certainly explodes again
      // (content is stable frame-to-frame: tape/text/map rows stay busy, sky/mask rows
      // stay clean), so it starts this frame FLAT (one memset row) instead of paying a
      // runs-reset + render + 225B explode-copy every frame. Clean lines get the O(1)
      // run reset and encode for free. Sticky decays every STICKY_DECAY frames so rows
      // whose content moved away (tape numbers scrolled, map rotated) re-probe RUNS.
      uint8_t fill2 = (uint8_t)((c << 4) | c);
      if (++frameNo >= STICKY_DECAY) { frameNo = 0; memset(stickyFlat, 0, HEIGHT); }
      for (int y = 0; y < HEIGHT; y++) {
        if (stickyFlat[y]) { memset(slot(y), fill2, lineStride); lineMode[y] = M_FLAT; }
        else               { lineMode[y] = M_RUNS; runCount[y] = 1; runsOf(y)[0] = (uint16_t)c; }
      }
      return;
    }
    memset(buffer, (uint8_t)((c << 4) | c), ((int32_t)WIDTH * HEIGHT + 1) >> 1);
  }

  void useBuffer(uint8_t *b) {
    buffer = b;
    if (lineMode) memset(lineMode, M_FLAT, HEIGHT);   // raw contents unknown -> trust as flat
  }

  // ---- dual-parity classic-font text blitter ---------------------------------
  // Classic-font glyphs normally render one PIXEL at a time (a nibble read-modify-
  // write each, or an s x s fillRect per lit bit at size > 1). This override renders
  // a glyph ROW at a time: a parity-aligned nibble MASK for the row is built on the
  // fly (~20 ops) and whole bytes are blitted with mask-RMW — dst = (dst & ~m) |
  // (color & m) — which is ~4-5x fewer operations per glyph and needs no pre-baked
  // tables. Handles any x parity (that is the "dual font" trick: the mask is built
  // at the glyph's actual nibble offset), any size 1..4, edge clipping, and the
  // dual-mode contract (the glyph's rows are flattened once, up front).
  // Fast path conditions: classic font, packed4, identity (no rotation matrix),
  // transparent background (textcolor == textbgcolor), square size. Everything else
  // falls back to the stock Adafruit chain. `fastText=false` forces the fallback —
  // the fuzz suite uses it to prove the two paths render byte-identically.
  bool fastText = true;
  size_t write(uint8_t ch) {
    if (!fastText || gfxFont || !packed4 || !identity ||
        textcolor != textbgcolor || textsize_x != textsize_y ||
        textsize_x < 1 || textsize_x > 4)
      return Adafruit_GFX::write(ch);
    if (ch == '\n') { cursor_x = 0; cursor_y += textsize_y * 8; return 1; }
    if (ch == '\r') return 1;
    uint8_t s = textsize_x;
    if (wrap && (cursor_x + 6 * s) > _width) { cursor_x = 0; cursor_y += 8 * s; }
    unsigned char c = (unsigned char)ch;
    if (!_cp437 && c >= 176) c++;              // the classic-font remap Adafruit applies
    drawCharFast(cursor_x, cursor_y, c, (uint8_t)(textcolor & 0x0F), s);
    cursor_x += 6 * s;
    return 1;
  }
  uint8_t textScale = 1;
  void setTextSize(uint8_t s) { GFXcanvas8::setTextSize((uint8_t)(s * textScale)); }

  // Buffer bytes for a w*h canvas in the given format. Packed canvases are laid
  // out as per-line slots of lineSlotBytes() (PRLE_STRIDE rounded even), NOT as
  // globally packed nibbles — size external buffers with THIS, and address rows
  // via lineBytes()/lineSlotBytes(), never (y*w + x) >> 1.
  static size_t bufBytes(uint16_t w, uint16_t h, bool p4) {
    if (!p4) return (size_t)w * h;
    return (size_t)h * ((PRLE_STRIDE(w) + 1) & ~1);
  }
  // Slot bytes per packed line (row y starts at rawBuffer() + y * lineSlotBytes()).
  size_t lineSlotBytes() const { return lineStride; }

  // ---- raw-access + finalize API (see contract above) -----------------------
  // Force lines [y0, y1) to FLAT so their slot bytes are plain packed nibbles.
  void flatten(int y0, int y1) {
    if (!packed4) return;
    if (y0 < 0) y0 = 0;
    if (y1 > HEIGHT) y1 = HEIGHT;
    for (int y = y0; y < y1; y++)
      if (lineMode[y] == M_RUNS) explodeTo(y, runsOf(y), runCount[y]);
  }
  // Raw buffer WITHOUT flattening — only for code that flatten()ed its rows first.
  uint8_t *rawBuffer() { return buffer; }
  // Whole-canvas raw access: flattens everything (byte-identical to flat packed4).
  uint8_t *getBuffer() {
    if (packed4) flatten(0, HEIGHT);
    return buffer;
  }
  // Frame finalize: encode lines [y0, y1) into REGION-RELATIVE fixed slots
  // comp + (y-y0)*stride with lengths in lineLen[y-y0]. RUNS lines cost O(runs)
  // (the draw already did the compression); FLAT lines take the word-at-a-time
  // scan. Every length <= PRLE_STRIDE(W) by the codec's proven bound.
  void encodeFrame(uint8_t *comp, uint16_t *lineLen, size_t stride, int y0, int y1) {
    if (!packed4) return;
    if (y0 < 0) y0 = 0;
    if (y1 > HEIGHT) y1 = HEIGHT;
    for (int y = y0; y < y1; y++) {
      uint8_t *dst = comp + (size_t)(y - y0) * stride;
      lineLen[y - y0] = (uint16_t)((lineMode[y] == M_RUNS)
                          ? prle_encode_runs(runsOf(y), runCount[y], WIDTH, dst)
                          : prle_encode_flat(slot(y), WIDTH, dst));
    }
  }
  bool lineIsRuns(int y) const { return packed4 && lineMode[y] == M_RUNS; }   // telemetry
  int  lineRuns(int y)   const { return (packed4 && lineMode[y] == M_RUNS) ? runCount[y] : 0; }

  // ---- pixel read (format-aware, non-mutating) -------------------------------
  // Palette index at (x, y). Packed RUNS lines are read by walking the run list —
  // reading never explodes a line. The 8-bit path honours setRotation() exactly
  // like GFXcanvas8::getPixel. Out of bounds -> 0.
  uint8_t getPixel(int16_t x, int16_t y) const {
    if (!packed4) {
      if ((uint16_t)x >= (uint16_t)_width || (uint16_t)y >= (uint16_t)_height) return 0;
      switch (rotation) {
        case 1: { int16_t t = x; x = WIDTH - 1 - y; y = t; } break;
        case 2: x = WIDTH - 1 - x; y = HEIGHT - 1 - y; break;
        case 3: { int16_t t = x; x = y; y = HEIGHT - 1 - t; } break;
      }
      return buffer[(int32_t)y * WIDTH + x];
    }
    if ((uint16_t)x >= (uint16_t)WIDTH || (uint16_t)y >= (uint16_t)HEIGHT) return 0;
    if (lineMode[y] == M_RUNS) {              // walk the run list: last run starting <= x
      const uint16_t *e = (const uint16_t *)(buffer + (size_t)y * lineStride);
      int n = runCount[y];
      uint8_t c = (uint8_t)(e[0] & 0x0F);
      for (int k = 1; k < n && (int)(e[k] >> 4) <= x; k++) c = (uint8_t)(e[k] & 0x0F);
      return c;
    }
    uint8_t b = buffer[(size_t)y * lineStride + (x >> 1)];
    return (x & 1) ? (uint8_t)(b & 0x0F) : (uint8_t)(b >> 4);
  }

  // ---- canvas-to-canvas blit (sprites) ---------------------------------------
  // Copy the rect [sx, sy, w, h] of `src` onto this canvas at (dx, dy), optionally
  // skipping one palette index as transparent (pass 0-15; -1 = opaque copy).
  // NATIVE coordinates on both canvases: the rotation matrix and setRotation are
  // deliberately ignored (same convention as sprite pushes in other libraries).
  // Fast path: both canvases packed, opaque, and byte-aligned (even sx and dx) ->
  // one memcpy per row. Everything else takes an exact per-nibble loop.
  void blitRect(const PackCanvas &src, int16_t sx, int16_t sy, int16_t w, int16_t h,
                int16_t dx, int16_t dy, int16_t transparent = -1) {
    // clip against both canvases
    if (sx < 0) { w += sx; dx -= sx; sx = 0; }
    if (sy < 0) { h += sy; dy -= sy; sy = 0; }
    if (dx < 0) { w += dx; sx -= dx; dx = 0; }
    if (dy < 0) { h += dy; sy -= dy; dy = 0; }
    if (sx + w > src.WIDTH)  w = src.WIDTH  - sx;
    if (sy + h > src.HEIGHT) h = src.HEIGHT - sy;
    if (dx + w > WIDTH)  w = WIDTH  - dx;
    if (dy + h > HEIGHT) h = HEIGHT - dy;
    if (w <= 0 || h <= 0) return;
    if (packed4 && src.packed4) {
      flatten(dy, dy + h);                        // dual-mode targets go flat once
      bool aligned = transparent < 0 && !(sx & 1) && !(dx & 1);
      for (int16_t j = 0; j < h; j++) {
        int syy = sy + j, dyy = dy + j;
        uint8_t *drow = buffer + (size_t)dyy * lineStride;
        if (src.lineMode[syy] == M_RUNS) {        // run-form source row: emit spans
          const uint16_t *e = (const uint16_t *)(src.buffer + (size_t)syy * src.lineStride);
          int n = src.runCount[syy];
          for (int k = 0; k < n; k++) {
            int rx = e[k] >> 4;
            uint8_t c = (uint8_t)(e[k] & 0x0F);
            int re = (k + 1 < n) ? (e[k + 1] >> 4) : src.WIDTH;
            if (re <= sx || rx >= sx + w) continue;
            if (rx < sx) rx = sx;
            if (re > sx + w) re = sx + w;
            if ((int16_t)c == transparent) continue;
            nibSpan(drow, dx + (rx - sx), re - rx, c);   // native coords, flat target row
          }
          continue;
        }
        const uint8_t *srow = src.buffer + (size_t)syy * src.lineStride;
        if (aligned) {                            // even-to-even: whole bytes
          int bytes = w >> 1;
          if (bytes) memcpy(drow + (dx >> 1), srow + (sx >> 1), (size_t)bytes);
          if (w & 1) {                            // odd trailing pixel
            uint8_t c = (uint8_t)(srow[(sx + w - 1) >> 1] >> 4);
            uint8_t *p = drow + ((dx + w - 1) >> 1);
            *p = (uint8_t)((*p & 0x0F) | (c << 4));
          }
          continue;
        }
        for (int16_t i = 0; i < w; i++) {         // exact general path
          uint8_t b = srow[(sx + i) >> 1];
          uint8_t c = ((sx + i) & 1) ? (uint8_t)(b & 0x0F) : (uint8_t)(b >> 4);
          if ((int16_t)c == transparent) continue;
          uint8_t *p = drow + ((dx + i) >> 1);
          if ((dx + i) & 1) *p = (uint8_t)((*p & 0xF0) | c);
          else              *p = (uint8_t)((*p & 0x0F) | (c << 4));
        }
      }
      return;
    }
    // mixed / 8-bit formats: exact per-pixel copy in native coords
    for (int16_t j = 0; j < h; j++)
      for (int16_t i = 0; i < w; i++) {
        uint8_t c = src.packed4 ? src.getPixel(sx + i, sy + j)
                                : src.buffer[(int32_t)(sy + j) * src.WIDTH + (sx + i)];
        if ((int16_t)c == transparent) continue;
        if (packed4) pix4(dx + i, dy + j, c);
        else buffer[(int32_t)(dy + j) * WIDTH + (dx + i)] = c;
      }
  }
  // Whole-canvas convenience: blit all of `src` at (dx, dy).
  void blit(const PackCanvas &src, int16_t dx, int16_t dy, int16_t transparent = -1) {
    blitRect(src, 0, 0, src.WIDTH, src.HEIGHT, dx, dy, transparent);
  }

  // ---- packed 4-bit image draw ------------------------------------------------
  // Draw a nibble-packed image (2 px/byte, row stride = (w+1)/2 bytes, high nibble
  // first — the natural export of any indexed image) at (x, y), optionally with one
  // transparent palette index. `data` may live in flash (read via pgm_read_byte).
  // NATIVE coordinates; dual-mode target rows are flattened once.
  void drawIndexedBitmap(int16_t x, int16_t y, const uint8_t *data, int16_t w, int16_t h,
                         int16_t transparent = -1) {
    if (!packed4) {                              // 8-bit canvas: unpack per pixel
      for (int16_t j = 0; j < h; j++)
        for (int16_t i = 0; i < w; i++) {
          uint8_t b = pgm_read_byte(data + (size_t)j * ((w + 1) >> 1) + (i >> 1));
          uint8_t c = (i & 1) ? (uint8_t)(b & 0x0F) : (uint8_t)(b >> 4);
          if ((int16_t)c != transparent) fastDrawPixel(x + i, y + j, c);
        }
      return;
    }
    int16_t i0 = x < 0 ? -x : 0, j0 = y < 0 ? -y : 0;
    int16_t i1 = (x + w > WIDTH)  ? WIDTH  - x : w;
    int16_t j1 = (y + h > HEIGHT) ? HEIGHT - y : h;
    if (i0 >= i1 || j0 >= j1) return;
    flatten(y + j0, y + j1);
    const size_t stride = (size_t)((w + 1) >> 1);
    for (int16_t j = j0; j < j1; j++) {
      const uint8_t *srow = data + (size_t)j * stride;
      uint8_t *drow = buffer + (size_t)(y + j) * lineStride;
      for (int16_t i = i0; i < i1; i++) {
        uint8_t b = pgm_read_byte(srow + (i >> 1));
        uint8_t c = (i & 1) ? (uint8_t)(b & 0x0F) : (uint8_t)(b >> 4);
        if ((int16_t)c == transparent) continue;
        uint8_t *p = drow + ((x + i) >> 1);
        if ((x + i) & 1) *p = (uint8_t)((*p & 0xF0) | c);
        else             *p = (uint8_t)((*p & 0x0F) | (c << 4));
      }
    }
  }

  // ---- vertical scroll ---------------------------------------------------------
  // Scroll rows [y0, y0+h) by dy pixels (dy > 0 = down, dy < 0 = up); vacated rows
  // are filled with `fill`. On a packed canvas each row is a self-contained record
  // (slot + mode + run count), so the scroll MOVES LINE RECORDS — memmove of the
  // slots plus their metadata, no per-pixel work, and run-form rows stay run-form.
  // NATIVE row coordinates (unaffected by rotation / the matrix).
  void vScroll(int16_t y0, int16_t h, int16_t dy, uint8_t fill) {
    if (y0 < 0) { h += y0; y0 = 0; }
    if (y0 + h > HEIGHT) h = HEIGHT - y0;
    if (h <= 0 || dy == 0) return;
    int16_t ady = dy < 0 ? -dy : dy;
    if (ady > h) ady = h;                       // whole region vacated (native fill below)
    int16_t keep = h - ady;
    int16_t srcY = (dy > 0) ? y0 : y0 + ady;    // content source rows
    int16_t dstY = (dy > 0) ? y0 + ady : y0;
    if (!packed4) {
      if (keep) memmove(buffer + (int32_t)dstY * WIDTH, buffer + (int32_t)srcY * WIDTH,
                        (size_t)keep * WIDTH);
      memset(buffer + (int32_t)(dy > 0 ? y0 : y0 + keep) * WIDTH, fill, (size_t)ady * WIDTH);
      return;
    }
    if (keep) {
      memmove(buffer + (size_t)dstY * lineStride, buffer + (size_t)srcY * lineStride,
              (size_t)keep * lineStride);
      memmove(lineMode + dstY,   lineMode + srcY,   (size_t)keep);
      memmove(stickyFlat + dstY, stickyFlat + srcY, (size_t)keep);
      memmove(runCount + dstY,   runCount + srcY,   (size_t)keep * sizeof(uint16_t));
    }
    int16_t vac = dy > 0 ? y0 : y0 + keep;      // vacated rows: one run each (or memset)
    for (int16_t yv = vac; yv < vac + ady; yv++) {
      if (dualMode && effCap >= 1) {
        lineMode[yv] = M_RUNS; runCount[yv] = 1;
        ((uint16_t *)(buffer + (size_t)yv * lineStride))[0] = (uint16_t)(fill & 0x0F);
      } else {
        memset(buffer + (size_t)yv * lineStride, (uint8_t)((fill << 4) | (fill & 0x0F)), lineStride);
        lineMode[yv] = M_FLAT;
      }
    }
  }
  // Read-only line access for compositors/encoders that want to consume run-lists
  // DIRECTLY (e.g. an RGB565 composite that fills each run once instead of doing a
  // per-byte LUT walk). Returns the run count and points *e at the entries while the
  // line is in RUNS form; -1 means FLAT — read lineBytes() instead. Only valid while
  // no draw call is mutating the canvas (composite/encode run after the frame's draw).
  int lineRunList(int y, const uint16_t **e) const {
    if (!packed4 || lineMode[y] != M_RUNS) return -1;
    *e = (const uint16_t *)(buffer + (size_t)y * lineStride);
    return runCount[y];
  }
  const uint8_t *lineBytes(int y) const { return buffer + (size_t)y * lineStride; }

private:
  enum { M_RUNS = 0, M_FLAT = 1 };

  // One classic-font glyph, row-at-a-time mask blit (see write() above). Callers
  // guarantee: packed4, identity, transparent bg, 1 <= s <= 4.
  void drawCharFast(int16_t x, int16_t y, unsigned char c, uint8_t col, uint8_t s) {
    if (x >= WIDTH || y >= HEIGHT || x + 5 * s <= 0 || y + 8 * s <= 0) return;
    flatten(y < 0 ? 0 : y, y + 8 * s);         // dual-mode: glyph rows go flat once
    uint8_t colbits[5];
    for (int i = 0; i < 5; i++) colbits[i] = pgm_read_byte(&PACKFONT_5X7[(size_t)c * 5 + i]);
    const uint8_t cc = (uint8_t)((col << 4) | col);
    const int x0 = x & ~1;                     // mask is built from this byte boundary
    const int off = x - x0;                    // 0 or 1 leading pad nibble
    const int nn = off + 5 * s;                // mask nibbles
    const int nb = (nn + 1) >> 1;              // mask bytes (<= 11 at s=4)
    for (int j = 0; j < 8; j++) {              // glyph rows (bit j of each column byte)
      uint8_t m[11];
      memset(m, 0, (size_t)nb);
      bool any = false;
      for (int i = 0; i < 5; i++) {
        if (!((colbits[i] >> j) & 1)) continue;
        any = true;
        int k = off + i * s;                   // s consecutive lit nibbles
        for (int t = 0; t < s; t++, k++) m[k >> 1] |= (k & 1) ? 0x0F : 0xF0;
      }
      if (!any) continue;
      for (int r = 0; r < s; r++) {            // the row repeats s times at size s
        int yy = y + j * s + r;
        if ((uint16_t)yy >= (uint16_t)HEIGHT) continue;
        uint8_t *row = slot(yy) + (x0 >> 1);
        for (int b = 0; b < nb; b++) {
          uint8_t mb = m[b];
          if (!mb) continue;
          int px = x0 + 2 * b;                 // even by construction
          if (px + 1 < 0 || px >= WIDTH) continue;
          if (px + 1 >= WIDTH) mb &= 0xF0;     // odd pixel past the right edge
          if (px < 0)          mb &= 0x0F;     // even pixel past the left edge
          if (!mb) continue;
          row[b] = (uint8_t)((row[b] & ~mb) | (cc & mb));
        }
      }
    }
  }
  static constexpr int STICKY_DECAY = 128;  // frames between sticky-flat re-probes
  uint8_t  *lineMode   = nullptr;          // per-line mode
  uint16_t *runCount   = nullptr;          // valid while mode == M_RUNS
  uint8_t  *stickyFlat = nullptr;          // exploded last frame -> start FLAT next frame
  uint16_t  lineStride = 0;                // PRLE_STRIDE(WIDTH) — slot bytes per line
  int       effCap  = 0;                   // min(RUN_CAP, slot capacity in entries)
  int       frameNo = 0;                   // sticky decay counter (bumped in fillScreen)

  inline uint8_t  *slot(int y)   { return buffer + (size_t)y * lineStride; }
  inline uint16_t *runsOf(int y) { return (uint16_t *)slot(y); }   // even stride: 2-aligned

  // Fill len nibbles of color c at nibble offset x in a FLAT row: odd lead nibble,
  // whole-byte memset, even tail nibble (the same shape as drawFastHLine's flat path).
  static inline void nibSpan(uint8_t *row, int x, int len, uint8_t c) {
    if (len <= 0) return;
    if (x & 1) { uint8_t *p = row + (x >> 1); *p = (uint8_t)((*p & 0xF0) | c); x++; len--; }
    int bytes = len >> 1;
    if (bytes) { memset(row + (x >> 1), (uint8_t)((c << 4) | c), (size_t)bytes); x += bytes * 2; len -= bytes * 2; }
    if (len == 1) { uint8_t *p = row + (x >> 1); *p = (uint8_t)((*p & 0x0F) | (c << 4)); }
  }

  // Single in-bounds packed4 pixel write. With dualMode off every line is FLAT, so
  // this is the original nibble RMW plus one predictable branch. A pixel landing on
  // a RUNS line EXPLODES it first (scattered content = flat is the right home; a
  // per-pixel splice measured 10-20x slower on device).
  inline void pix4(int16_t x, int16_t y, uint8_t c) {
    if (lineMode[y] != M_FLAT) explodeTo(y, runsOf(y), runCount[y]);
    uint8_t *p = slot(y) + (x >> 1);
    if (x & 1) *p = (*p & 0xF0) | (c & 0x0F);
    else       *p = (*p & 0x0F) | ((c & 0x0F) << 4);
  }

  // Render a run-list to flat packed nibbles (dst = lineStride bytes). Runs are
  // contiguous from x=0; an even single tail stores the whole byte (pad = 0).
  void renderRunsTo(const uint16_t *e, int n, uint8_t *dst) {
    for (int k = 0; k < n; k++) {
      int x = e[k] >> 4;
      uint8_t c = (uint8_t)(e[k] & 0x0F);
      int xe = (k + 1 < n) ? (e[k + 1] >> 4) : WIDTH;
      int len = xe - x;
      if (len && (x & 1)) { uint8_t *p = dst + (x >> 1); *p = (uint8_t)((*p & 0xF0) | c); x++; len--; }
      if (len >= 2) { memset(dst + (x >> 1), (uint8_t)((c << 4) | c), (size_t)(len >> 1)); x += len & ~1; len &= 1; }
      if (len) dst[x >> 1] = (uint8_t)(c << 4);
    }
  }
  // Explode: render list e (the slot itself or a splice temp) to flat, go FLAT, and
  // remember the line as busy so the next fillScreen starts it flat (see sticky-flat).
  void explodeTo(int y, const uint16_t *e, int n) {
    uint8_t tmp[PRLE_STRIDE(NANO_MAX_W)];  // stack temp: rendering never reads its own writes
    renderRunsTo(e, n, tmp);
    memcpy(slot(y), tmp, lineStride);
    lineMode[y] = M_FLAT;
    stickyFlat[y] = 1;
  }

  // Splice [x0, x1) of color c into a RUNS line (callers clip). Rebuild into a stack
  // temp (grows by at most +2 entries), merging same-color neighbours on both edges;
  // commit, or explode to FLAT if the count would pass the cap.
  void splice(int y, int x0, int x1, uint8_t c) {
    uint16_t *e = runsOf(y);
    int n = runCount[y];
    uint16_t tmp[RUN_CAP + 2];
    int m = 0, k = 0;
    while (k < n) {                                        // runs ending at/before x0
      int xe = (k + 1 < n) ? (e[k + 1] >> 4) : WIDTH;
      if (xe > x0) break;
      tmp[m++] = e[k++];
    }
    if (k < n && (e[k] >> 4) < x0) tmp[m++] = e[k];        // left part of the split run
    if (!(m > 0 && (tmp[m - 1] & 0x0F) == c))              // the new run (merge left)
      tmp[m++] = (uint16_t)((x0 << 4) | c);
    int j = k;                                             // run containing x1 resumes after
    while (j < n) {
      int xe = (j + 1 < n) ? (e[j + 1] >> 4) : WIDTH;
      if (xe > x1) break;
      j++;
    }
    if (j < n) {
      uint8_t rc = (uint8_t)(e[j] & 0x0F);
      if (rc != c) tmp[m++] = (uint16_t)((x1 << 4) | rc);  // rc == c: right merge
    }
    for (int t = j + 1; t < n; t++) tmp[m++] = e[t];       // untouched tail
    if (m > effCap) { explodeTo(y, tmp, m); return; }      // RUN_CAP explode (one-way)
    memcpy(e, tmp, (size_t)m * sizeof(uint16_t));
    runCount[y] = (uint16_t)m;
  }
};
