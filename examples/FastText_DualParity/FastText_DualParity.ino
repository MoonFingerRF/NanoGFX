// NanoGFX: FastText_DualParity
//
// On a 2-px/byte canvas a glyph column lands on either the HIGH or LOW nibble
// depending on its x position — the "parity" problem that usually forces text
// back to per-pixel drawing. PackCanvas's blitter instead renders each glyph
// ROW at a time: it builds a parity-aligned nibble MASK on the fly (~20 ops)
// and blits whole bytes with read-modify-writes. ~4-5x fewer operations per
// glyph, at ANY x alignment, sizes 1-4, byte-identical output to stock
// Adafruit_GFX (proven by NanoPFD's fuzz oracle over chars 32-255, both
// parities, clipped edges).
//
// This sketch times the same text drawn with the blitter on and off.
#include <NanoGFX.h>

#define W 240
#define H 120

PackCanvas cv(W, H);

static uint32_t bench(bool fast) {
  cv.fastText = fast;
  cv.fillScreen(0);
  uint32_t t = micros();
  for (int rep = 0; rep < 20; rep++)
    for (int size = 1; size <= 3; size++) {
      cv.setTextSize(size);
      cv.setTextColor(7);
      cv.setCursor(rep & 1, 10 + size * 26);   // alternate x parity on purpose
      cv.print("NanoGFX 0123456789");
    }
  return micros() - t;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  cv.packed4 = true;

  uint32_t slow = bench(false);   // stock Adafruit per-pixel path
  uint32_t fast = bench(true);    // dual-parity row blitter (the default)
  Serial.println("NanoGFX dual-parity text blitter");
  Serial.printf("stock per-pixel : %lu us\n", (unsigned long)slow);
  Serial.printf("dual-parity     : %lu us   (%.1fx)\n",
                (unsigned long)fast, (float)slow / (float)fast);
  // The blitter engages for the classic 5x7 font with textcolor != background
  // (transparent text), sizes 1-4, unrotated canvas. Anything else falls back
  // to the stock path automatically — output is always identical.
}

void loop() {}
