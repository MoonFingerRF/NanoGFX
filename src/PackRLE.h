// ============================================================================
//  PackRLE.h — per-line "16th color" escape RLE for the 4-bit packed canvas.
//
//  A line of W pixels is a NIBBLE stream, MSB-first within each byte (same as the
//  packed4 canvas: even pixel index -> HIGH nibble):
//    nibble 0x0..0xE : literal — one pixel of that palette color.
//    nibble 0xF      : escape — the next 4 nibbles are [c][n2][n1][n0]:
//                      a run of N = (n2<<8)|(n1<<4)|n0 pixels of color c (c <= 0xE).
//                      c == 0xF after an escape is RESERVED; decoders reject it.
//                      N is 12 bits (1..4095) — max line is 640 px, no bias needed.
//    A trailing odd nibble pads with 0; decoders stop after W pixels.
//
//  Encoder rule (this is what makes the size bound PROVABLE): a run token costs 5
//  nibbles, so runs are only emitted at length >= PRLE_MIN_RUN = 6; anything shorter
//  is emitted as literals. Every emitted pixel therefore costs <= 1 nibble
//  (runs: 5 nibbles for >= 6 px; literals: exactly 1 nibble/px), so
//      encoded nibbles <= W  =>  encoded bytes <= ceil(W/2) == PRLE_STRIDE(W)
//  for ALL inputs — the worst case is exactly flat packed4 size, zero overhead.
//  Fixed per-line slot stride: PRLE_STRIDE(W). (Compare RLE332_MAX_BYTES(450)=455;
//  this is 225 — the comp buffers halve too.)
//
//  Conventions mirror RLE332.h: header-only C, no allocation, per-line independent
//  encode (parallel-core friendly), word-at-a-time ALIGNED scan on the flat encoder
//  (byte-step to 4-alignment first — Xtensa traps unaligned 32-bit loads).
// ============================================================================
#pragma once
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <stdbool.h>

// Arduino builds the sketch at -Os, which refuses to flatten the nibble-writer inline
// chain (a call per nibble on literal content — measured ~2x slower than RLE332's
// byte loops on the S3). Force speed codegen for these hot loops only.
#ifdef __XTENSA__
#pragma GCC push_options
#pragma GCC optimize("O2")
#endif

#define PRLE_STRIDE(w)   (((w) + 1) >> 1)   // fixed per-line slot bytes == flat packed4
#define PRLE_MIN_RUN     6                  // token = 5 nibbles -> only profitable at >= 6
#define PRLE_MAX_RUN     4095               // 12-bit run length field

// Aligned-load word type. Reading a uint8_t buffer through a uint32_t* is technically
// an aliasing violation; may_alias makes it defined on gcc/clang (incl. xtensa-esp32s3
// gcc, which RLE332.h already relies on for the same pattern). Loads are 4-aligned only.
typedef uint32_t __attribute__((may_alias)) prle_u32a;

// ---- nibble writer (encoders) ----------------------------------------------
// REGISTER-BUFFERED: the pending high nibble lives in a register and every output byte
// is stored exactly ONCE (no read-modify-write) — measured ~2x on literal-heavy content
// (the noisy map lines), which is what made the first per-nibble-RMW version slower
// than RLE332's memcpy'd literals on device. The final pad nibble of an odd-length
// stream is 0 by construction (the flush stores pend with low nibble 0). Nibble counts
// are identical to a naive writer, so the size-bound argument is unchanged.
typedef struct { uint8_t *out; size_t nn; uint8_t pend; } prle__w;
static inline void prle__wn(prle__w *W, uint8_t v) {
  if (W->nn & 1) W->out[W->nn >> 1] = (uint8_t)(W->pend | (v & 0x0F));
  else           W->pend = (uint8_t)((v & 0x0F) << 4);
  W->nn++;
}
static inline size_t prle__wend(prle__w *W) {     // flush the dangling high nibble
  if (W->nn & 1) W->out[W->nn >> 1] = W->pend;
  return (W->nn + 1) >> 1;
}
static inline void prle__wtok(prle__w *W, uint8_t c, int n) {  // escape token
  prle__wn(W, 0xF);
  prle__wn(W, c);
  prle__wn(W, (uint8_t)((n >> 8) & 0xF));
  prle__wn(W, (uint8_t)((n >> 4) & 0xF));
  prle__wn(W, (uint8_t)(n & 0xF));
}
// Emit one maximal same-color stretch under the MIN_RUN rule (the ONLY emission path
// in all encoders — this is where the <= 1 nibble/px invariant lives). len may exceed
// PRLE_MAX_RUN (not on real panels; kept correct anyway): split, each chunk still
// costs <= chunk nibbles.
static inline void prle__emit(prle__w *W, uint8_t c, int len) {
  while (len > 0) {
    int seg = (len > PRLE_MAX_RUN) ? PRLE_MAX_RUN : len;
    if (seg >= PRLE_MIN_RUN) prle__wtok(W, c, seg);           // 5 nibbles <= seg
    else for (int t = 0; t < seg; t++) prle__wn(W, c);        // seg nibbles == seg
    len -= seg;
  }
}

// Pixel read from a flat packed4 line (even x -> high nibble).
static inline uint8_t prle__px(const uint8_t *flat, int i) {
  uint8_t b = flat[i >> 1];
  return (uint8_t)((i & 1) ? (b & 0x0F) : (b >> 4));
}

// Emit pixels [i, i+n) of a PACKED source as literals, block-wise. Literal nibbles are
// the source nibbles VERBATIM (valid canvas content never holds 0xF), so once source
// and stream are byte-aligned this is a memcpy — or a 1-nibble-shifted copy when their
// parities differ (each token is 5 nibbles, so the stream flips parity per run). This
// is what keeps noisy/dotted lines at RLE332's memcpy-literal speed. n literal nibbles
// for n pixels — the <= 1 nibble/px bound is untouched.
static inline void prle__wlitblk(prle__w *W, const uint8_t *flat, int i, int n) {
  if ((i & 1) && n) { prle__wn(W, prle__px(flat, i)); i++; n--; }   // align the SOURCE
  int nb = n >> 1;
  if (nb) {
    if (!(W->nn & 1)) {                                   // parities match: straight copy
      memcpy(W->out + (W->nn >> 1), flat + (i >> 1), (size_t)nb);
    } else {                                              // stream is odd: shifted copy
      const uint8_t *s = flat + (i >> 1);
      uint8_t *o = W->out + (W->nn >> 1);
      uint8_t pend = W->pend;
      for (int t = 0; t < nb; t++) { uint8_t b = s[t]; *o++ = (uint8_t)(pend | (b >> 4)); pend = (uint8_t)(b << 4); }
      W->pend = pend;
    }
    W->nn += (size_t)nb * 2; i += nb * 2; n &= 1;
  }
  if (n) prle__wn(W, prle__px(flat, i));                  // final odd pixel
}

// Emit pixels [i, i+n) of an 8-BIT source as literals: after realigning the stream,
// pack two source bytes per output byte in a tight loop (no per-nibble branches).
static inline void prle__wlitblk8(prle__w *W, const uint8_t *idx, int i, int n) {
  if ((W->nn & 1) && n) { prle__wn(W, (uint8_t)(idx[i] & 0x0F)); i++; n--; }   // align the STREAM
  int nb = n >> 1;
  uint8_t *o = W->out + (W->nn >> 1);
  for (int t = 0; t < nb; t++)
    o[t] = (uint8_t)(((idx[i + 2 * t] & 0x0F) << 4) | (idx[i + 2 * t + 1] & 0x0F));
  W->nn += (size_t)nb * 2;
  if (n & 1) prle__wn(W, (uint8_t)(idx[i + 2 * nb] & 0x0F));
}

// Maximal same-color run at pixel i of a packed line, with a cheap early-out for the
// noise/dot case (runs of 1-2 skip the alignment/word machinery entirely).
static inline int prle__runflat(const uint8_t *flat, int i, int w) {
  uint8_t c = prle__px(flat, i);
  int maxrun = w - i;
  if (maxrun < 2 || prle__px(flat, i + 1) != c) return 1;
  int run = 2;
  if (run < maxrun && ((i + run) & 1) && prle__px(flat, i + run) == c) run++;
  if (run < maxrun && !((i + run) & 1)) {                 // byte boundary and still matching
    uint8_t cc = (uint8_t)((c << 4) | c);
    int b = (i + run) >> 1;
    while (run + 2 <= maxrun && ((uintptr_t)(flat + b) & 3u) && flat[b] == cc) { run += 2; b++; }
    if (run + 2 <= maxrun && flat[b] == cc) {
      uint32_t bc = (uint32_t)cc * 0x01010101u;
      while (run + 8 <= maxrun && *(const prle_u32a *)(flat + b) == bc) { run += 8; b += 4; }
      while (run + 2 <= maxrun && flat[b] == cc) { run += 2; b++; }
    }
    if (run < maxrun && prle__px(flat, i + run) == c) run++;   // half-matching tail byte
  }
  return run;
}

// ---- encode: FLAT packed4 line -> escape stream. Returns encoded bytes. ----
// Run scan is word-at-a-time on the packed bytes: byte == (c<<4)|c is a 2-px solid.
// Nibble-step to a byte boundary, byte-step to 4-alignment, then compare aligned
// 32-bit words against a broadcast (rle332_encode_indexed's proven pattern), then
// byte/nibble tails. Device note: on the S3 the aligned loads matter (trap) AND the
// word compares are where flat sky/ground lines get skipped in ~W/8 iterations.
static inline size_t prle_encode_flat(const uint8_t *flat, int w, uint8_t *out) {
  prle__w W = { out, 0, 0 };
  int i = 0;
  while (i < w) {
    // Find the next run >= MIN_RUN at/after pixel i by scanning BYTES for a pair of
    // adjacent equal SOLID bytes — every run >= 6 must contain one (worst alignment:
    // lo-nibble head + 2 solid bytes + hi-nibble tail). This keeps the busy-line scan
    // at ~1 compare/2px with zero nibble extraction; everything before the run is one
    // literal block (memcpy-class, see prle__wlitblk). The emitted stream is identical
    // to the naive per-stretch encoder's (fuzz-asserted against prle_encode_idx8).
    int runStart = -1, runEnd = 0;
    int b = (i + 1) >> 1, bmax = w >> 1;
    while (b + 1 < bmax) {
      uint8_t v = flat[b];
      if (v == flat[b + 1] && (v >> 4) == (v & 0x0F)) {
        uint8_t c = (uint8_t)(v & 0x0F);
        // extend LEFT byte-wise (bounded by the first full byte at/after i) + head nibble
        int bl = b, blMin = (i + 1) >> 1;
        while (bl > blMin && flat[bl - 1] == v) bl--;
        int rs = 2 * bl;
        if (rs > i && prle__px(flat, rs - 1) == c) rs--;
        // extend RIGHT byte-wise, word-batched (the long direction), + tail nibble
        int br = b + 2;
        while (br < bmax && ((uintptr_t)(flat + br) & 3u) && flat[br] == v) br++;
        if (br + 4 <= bmax && flat[br] == v) {
          uint32_t bc = (uint32_t)v * 0x01010101u;
          while (br + 4 <= bmax && *(const prle_u32a *)(flat + br) == bc) br += 4;
        }
        while (br < bmax && flat[br] == v) br++;
        int re = 2 * br;
        if (re < w && prle__px(flat, re) == c) re++;
        if (re - rs >= PRLE_MIN_RUN) { runStart = rs; runEnd = re; break; }
        b = (re + 1) >> 1;                 // 4-5 px run: stays literal, scan past it
        continue;
      }
      b++;
    }
    if (runStart < 0) {                    // no run to end of line: all literal
      prle__wlitblk(&W, flat, i, w - i);
      break;
    }
    if (runStart > i) prle__wlitblk(&W, flat, i, runStart - i);
    prle__emit(&W, prle__px(flat, runStart), runEnd - runStart);
    i = runEnd;
  }
  return prle__wend(&W);
}

// ---- encode: 8-bit index line -> escape stream (indices 0..14). Returns bytes. ----
// For canvases still stored 1 byte/px (transitional / rollback path): run scan is
// word-at-a-time on the index bytes (rle332_encode_indexed's proven pattern); emission
// goes through the same prle__emit, so the stream format and the <= PRLE_STRIDE(w)
// size bound are identical to the packed encoders. Indices are masked to 4 bits.
static inline int prle__run8(const uint8_t *idx, int i, int w) {   // run finder, 8-bit source
  uint8_t c = idx[i];
  if (i + 1 >= w || idx[i + 1] != c) return 1;                     // noise early-out
  const uint8_t *p = idx + i + 2, *end = idx + w;
  while (p < end && ((uintptr_t)p & 3u) && *p == c) p++;           // byte-step to 4-align
  if (p + 4 <= end && *p == c) {
    uint32_t bc = (uint32_t)c * 0x01010101u;
    while (p + 4 <= end && *(const prle_u32a *)p == bc) p += 4;    // aligned word compares
  }
  while (p < end && *p == c) p++;                                  // byte tail
  return (int)(p - (idx + i));
}

static inline size_t prle_encode_idx8(const uint8_t *idx, int w, uint8_t *out) {
  prle__w W = { out, 0, 0 };
  int i = 0;
  while (i < w) {
    int run = prle__run8(idx, i, w);
    if (run >= PRLE_MIN_RUN) { prle__emit(&W, (uint8_t)(idx[i] & 0x0F), run); i += run; continue; }
    int j = i + run;                          // literal region: block-pack it (see wlitblk8)
    while (j < w) {
      int r2 = prle__run8(idx, j, w);
      if (r2 >= PRLE_MIN_RUN) break;
      j += r2;
    }
    prle__wlitblk8(&W, idx, i, j - i);
    i = j;
  }
  return prle__wend(&W);
}

// ---- encode: run-list -> escape stream. Returns encoded bytes. -------------
// runs[k] = (x_start << 4) | color, sorted, runs[0] starts at x=0, run ends at the
// next entry's x (last ends at w). Near-free: O(runs) with no pixel scan at all —
// this is the whole point of keeping canvas lines in run-list form until finalize.
// Precondition (canvas-enforced): colors <= 0xE, x strictly increasing, coverage [0,w).
static inline size_t prle_encode_runs(const uint16_t *runs, int nRuns, int w, uint8_t *out) {
  prle__w W = { out, 0, 0 };
  for (int k = 0; k < nRuns; k++) {
    int x  = runs[k] >> 4;
    int xe = (k + 1 < nRuns) ? (runs[k + 1] >> 4) : w;
    prle__emit(&W, (uint8_t)(runs[k] & 0x0F), xe - x);
  }
  return prle__wend(&W);
}

// ---- decoders ---------------------------------------------------------------
// All three parse identically and return false on malformed input: escape with
// c == 0xF (reserved), run length N == 0, a run overrunning W, or the stream
// exhausting before W pixels (truncation). Bytes past the W-th pixel are ignored.
static inline uint8_t prle__rn(const uint8_t *enc, size_t k) {   // read nibble k
  uint8_t b = enc[k >> 1];
  return (uint8_t)((k & 1) ? (b & 0x0F) : (b >> 4));
}

// -> flat packed4 (roundtrip / canvas reload). Every output byte is fully written
// (even-pixel writes store the whole byte), so the odd-W pad nibble decodes as 0.
static inline bool prle_decode_flat(const uint8_t *enc, size_t nb, int w, uint8_t *flat) {
  size_t k = 0, kmax = nb * 2;
  int px = 0;
  while (px < w) {
    if (k >= kmax) return false;                        // truncated
    uint8_t v = prle__rn(enc, k++);
    if (v != 0xF) {                                     // literal pixel
      if (px & 1) { uint8_t *p = flat + (px >> 1); *p = (uint8_t)((*p & 0xF0) | v); }
      else        flat[px >> 1] = (uint8_t)(v << 4);
      px++;
      continue;
    }
    if (k + 4 > kmax) return false;                     // truncated escape
    uint8_t c = prle__rn(enc, k++);
    if (c == 0xF) return false;                         // reserved
    int n = ((int)prle__rn(enc, k) << 8) | ((int)prle__rn(enc, k + 1) << 4) | prle__rn(enc, k + 2);
    k += 3;
    if (n == 0 || px + n > w) return false;             // bad length / overrun
    // nibble fill: odd lead (RMW), byte memset middle, high-nibble tail
    if (px & 1) { uint8_t *p = flat + (px >> 1); *p = (uint8_t)((*p & 0xF0) | c); px++; n--; }
    if (n >= 2) { memset(flat + (px >> 1), (uint8_t)((c << 4) | c), (size_t)(n >> 1)); px += n & ~1; n &= 1; }
    if (n) { flat[px >> 1] = (uint8_t)(c << 4); px++; }
  }
  return true;
}

// -> 8-bit through a 16-entry LUT (BOARD_D bounce-decode replacement: the AMOLED
// wants RGB332 bytes; runs become a single memset(lut[c]) instead of N lookups).
static inline bool prle_decode_lut8(const uint8_t *enc, size_t nb, int w, uint8_t *out,
                                    const uint8_t lut[16]) {
  size_t k = 0, kmax = nb * 2;
  int px = 0;
  while (px < w) {
    if (k >= kmax) return false;
    uint8_t v = prle__rn(enc, k++);
    if (v != 0xF) { out[px++] = lut[v]; continue; }
    if (k + 4 > kmax) return false;
    uint8_t c = prle__rn(enc, k++);
    if (c == 0xF) return false;
    int n = ((int)prle__rn(enc, k) << 8) | ((int)prle__rn(enc, k + 1) << 4) | prle__rn(enc, k + 2);
    k += 3;
    if (n == 0 || px + n > w) return false;
    memset(out + px, lut[c], (size_t)n);
    px += n;
  }
  return true;
}

// -> 8-bit, FAST variant for the hot push path: while the stream cursor is byte-aligned,
// literal nibbles are consumed TWO AT A TIME through a 256-entry pair table
// (lutPair[b] = lut[b>>4] | lut[b&0xF]<<8 — build it next to lut); escapes, odd tails,
// and odd-parity cursors fall back to the nibble path. Output and rejection behavior
// are identical to prle_decode_lut8 (equivalence-fuzzed in the self-test). This is what
// keeps literal-heavy lines (attitude, busy map rows) at memcpy-class speed on device.
static inline bool prle_decode_lut8p(const uint8_t *enc, size_t nb, int w, uint8_t *out,
                                     const uint8_t lut[16], const uint16_t lutPair[256]) {
  size_t k = 0, kmax = nb * 2;
  int px = 0;
  while (px < w) {
    if (!(k & 1)) {                                     // byte-aligned: batch literal pairs
      while (px + 2 <= w && k + 2 <= kmax) {
        uint8_t b = enc[k >> 1];
        if ((b >> 4) == 0xF || (b & 0x0F) == 0xF) break;
        uint16_t pr = lutPair[b];
        out[px] = (uint8_t)pr; out[px + 1] = (uint8_t)(pr >> 8);
        px += 2; k += 2;
      }
      if (px >= w) break;
    }
    if (k >= kmax) return false;
    uint8_t v = prle__rn(enc, k++);
    if (v != 0xF) { out[px++] = lut[v]; continue; }
    if (k + 4 > kmax) return false;
    // token parse with whole-byte reads (k parity is known): [c][n2][n1][n0]
    uint8_t c;
    int n;
    if (!(k & 1)) {                     // c starts a fresh byte: [c n2][n1 n0]
      uint8_t b0 = enc[k >> 1], b1 = enc[(k >> 1) + 1];
      c = (uint8_t)(b0 >> 4);
      n = ((int)(b0 & 0x0F) << 8) | b1;
    } else {                            // c is a low nibble: [. c][n2 n1][n0 .]
      uint8_t b0 = enc[k >> 1], b1 = enc[(k >> 1) + 1], b2 = enc[(k >> 1) + 2];
      c = (uint8_t)(b0 & 0x0F);
      n = ((int)b1 << 4) | (b2 >> 4);
    }
    k += 4;
    if (c == 0xF) return false;
    if (n == 0 || px + n > w) return false;
    memset(out + px, lut[c], (size_t)n);
    px += n;
  }
  return true;
}

// -> RGB565 pairs through comboLut[256] (BOARD_C composite replacement): one packed
// byte (2 px) maps to one 32-bit store, exactly like the gComboLUT byte loop, but a
// run's interior becomes a 32-bit fill of comboLut[(c<<4)|c]. Odd-x run boundaries
// (a byte mixing two colors) are composed in `hi` before the store. Output is
// ceil(w/2) uint32 entries; the odd-W pad nibble contributes 0 (matches a flat
// buffer whose pad is 0). out is uint32_t* so its stores are naturally aligned.
static inline bool prle_decode_lut32(const uint8_t *enc, size_t nb, int w, uint32_t *out,
                                     const uint32_t comboLut[256]) {
  size_t k = 0, kmax = nb * 2;
  int px = 0;
  uint8_t hi = 0;                                       // pending high nibble (px even)
  while (px < w) {
    if (k >= kmax) return false;
    uint8_t v = prle__rn(enc, k++);
    int n = 1;
    if (v == 0xF) {
      if (k + 4 > kmax) return false;
      v = prle__rn(enc, k++);
      if (v == 0xF) return false;
      n = ((int)prle__rn(enc, k) << 8) | ((int)prle__rn(enc, k + 1) << 4) | prle__rn(enc, k + 2);
      k += 3;
      if (n == 0 || px + n > w) return false;
    }
    if (px & 1) {                                       // complete the pending pair
      out[px >> 1] = comboLut[hi | v];
      px++; n--;
    }
    if (n >= 2) {                                       // whole pairs: one lookup, word fill
      uint32_t val = comboLut[(v << 4) | v];
      uint32_t *o = out + (px >> 1);
      for (int t = n >> 1; t; t--) *o++ = val;
      px += n & ~1; n &= 1;
    }
    if (n) { hi = (uint8_t)(v << 4); px++; }            // dangling high nibble
  }
  if (w & 1) out[w >> 1] = comboLut[hi];                // pad low nibble = 0
  return true;
}

#ifdef __XTENSA__
#pragma GCC pop_options
#endif

// ============================================================================
//  Host self-test (mirrors RLE332.h's):
//  g++ -DPRLE_TEST_MAIN -x c++ PackRLE.h -o /tmp/prletest && /tmp/prletest
//  The heavy fuzz (1e5+ iterations, canvas equivalence) lives in test_packrle.cpp.
// ============================================================================
#ifdef PRLE_TEST_MAIN
#include <stdio.h>
#include <stdlib.h>
static int prle__t1(const uint8_t *pxl, int w, const char *name) {
  uint8_t flat[512] = {0}, enc[512], dec[512];
  for (int i = 0; i < w; i++) flat[i >> 1] |= (uint8_t)((i & 1) ? (pxl[i] & 0xF) : (pxl[i] << 4));
  size_t el = prle_encode_flat(flat, w, enc);
  memset(dec, 0xAA, sizeof dec);
  int ok = prle_decode_flat(enc, el, w, dec)
        && memcmp(flat, dec, (size_t)PRLE_STRIDE(w)) == 0
        && el <= (size_t)PRLE_STRIDE(w);
  printf("  %-20s w=%3d enc=%3d bound=%3d %s\n", name, w, (int)el, PRLE_STRIDE(w),
         ok ? "OK" : "*** FAIL ***");
  return ok;
}
int main(void) {
  uint8_t px[1024];
  int fails = 0, w = 480;
  for (int i = 0; i < w; i++) px[i] = 8;                       fails += !prle__t1(px, w, "solid");
  for (int i = 0; i < w; i++) px[i] = (uint8_t)((i & 1) ? 7 : 0); fails += !prle__t1(px, w, "alternating(worst)");
  for (int i = 0; i < w; i++) px[i] = (uint8_t)((i / 40) % 15);  fails += !prle__t1(px, w, "bands");
  for (int i = 0; i < w; i++) px[i] = (uint8_t)(rand() % 15);    fails += !prle__t1(px, w, "noise");
  for (int i = 0; i < 7; i++) px[i] = (uint8_t)(i % 15);         fails += !prle__t1(px, 7, "odd-tiny");
  uint8_t bad1[] = {0xFF, 0x00, 0x10};                            // escape then c=0xF
  uint8_t bad2[] = {0xF2, 0x00, 0x00};                            // N == 0
  uint8_t dump[64];
  fails += prle_decode_flat(bad1, sizeof bad1, 8, dump);          // must return false
  fails += prle_decode_flat(bad2, sizeof bad2, 8, dump);
  // prle_decode_lut8p (pair-table batching) must match prle_decode_lut8 exactly on
  // random content (mixed runs/noise -> tokens land at both cursor parities), and agree
  // on rejection for truncated streams.
  {
    uint8_t lut[16]; uint16_t lp[256];
    for (int i = 0; i < 16; i++) lut[i] = (uint8_t)(i * 17);
    for (int b = 0; b < 256; b++) lp[b] = (uint16_t)(lut[b >> 4] | (lut[b & 0x0F] << 8));
    int bad = 0;
    for (int it = 0; it < 20000 && !bad; it++) {
      int ww = 1 + rand() % 640;
      static uint8_t flat[512], enc[512], d1[768], d2[768];
      memset(flat, 0, sizeof flat);
      for (int i = 0; i < ww;) {                         // random run structure
        uint8_t c = (uint8_t)(rand() % 15);
        int len = 1 + ((rand() % 4 == 0) ? rand() % 60 : rand() % 3);
        if (i + len > ww) len = ww - i;
        for (int t = 0; t < len; t++, i++) flat[i >> 1] |= (uint8_t)((i & 1) ? c : c << 4);
      }
      size_t el = prle_encode_flat(flat, ww, enc);
      memset(d1, 0x5A, sizeof d1); memset(d2, 0xA5, sizeof d2);
      bool r1 = prle_decode_lut8 (enc, el, ww, d1, lut);
      bool r2 = prle_decode_lut8p(enc, el, ww, d2, lut, lp);
      if (r1 != r2 || !r1 || memcmp(d1, d2, (size_t)ww) != 0) bad = 1;
      if (el > 1) {                                      // truncation must reject in both
        size_t cut = (size_t)(rand() % (int)el);
        r1 = prle_decode_lut8 (enc, cut, ww, d1, lut);
        r2 = prle_decode_lut8p(enc, cut, ww, d2, lut, lp);
        if (r1 != r2) bad = 1;
      }
    }
    printf("  lut8p==lut8 fuzz     %s\n", bad ? "*** FAIL ***" : "OK");
    fails += bad;
  }
  // prle_encode_idx8 must emit the byte-identical stream encode_flat emits for the
  // same pixels (mixed runs + noise, all alignments via the odd start offset).
  for (int off = 0; off < 4; off++) {
    static uint8_t i8buf[512], flat[256], e1[256], e2[256];
    uint8_t *i8 = i8buf + off;
    for (int i = 0; i < 480; i++) i8[i] = (uint8_t)((i < 200) ? (i / 37) % 15 : rand() % 15);
    memset(flat, 0, sizeof flat);
    for (int i = 0; i < 480; i++) flat[i >> 1] |= (uint8_t)((i & 1) ? i8[i] : i8[i] << 4);
    size_t l1 = prle_encode_flat(flat, 480, e1), l2 = prle_encode_idx8(i8, 480, e2);
    int ok = (l1 == l2) && memcmp(e1, e2, l1) == 0 && l2 <= (size_t)PRLE_STRIDE(480);
    printf("  idx8==flat (off=%d)   %s\n", off, ok ? "OK" : "*** FAIL ***");
    fails += !ok;
  }
  printf("%s\n", fails ? "FAILED" : "ALL PASS");
  return fails ? 1 : 0;
}
#endif
