// NanoGFX: DualMode_Runs
//
// Dual-mode is the "RLE fused into drawing" trick: each canvas line lives as a
// tiny RUN-LIST until the drawing pattern says otherwise. H/V line spans and
// fills SPLICE runs (no pixels touched); scattered drawPixel work EXPLODES just
// the lines it lands on, which then run at full flat speed.
//
// encodeFrame() emits each line's PackRLE stream — for a line still in run
// form that costs almost nothing, because the drawing already did the
// compression. This sketch draws structured content, then prints how big each
// line's encoded stream is and which lines stayed in run form.
#include <NanoGFX.h>

#define W 240
#define H 120

PackCanvas cv(W, H);
uint8_t  comp[H * PRLE_STRIDE(W)];
uint16_t lens[H];

void setup() {
  Serial.begin(115200);
  delay(1500);

  cv.packed4  = true;
  cv.dualMode = true;                      // lines live as run-lists

  cv.fillScreen(0);                        // one run per line
  cv.fillRect(10, 10, 220, 30, 4);         // row-major: one span splice per line
  cv.drawFastHLine(0, 60, W, 2);           // one run spliced into line 60
  for (int x = 20; x < 220; x += 10)       // scattered pixels: these EXPLODE
    cv.drawPixel(x, 90, 6);                //   line 90 only — everything else stays runs

  uint32_t t = micros();
  cv.encodeFrame(comp, lens, PRLE_STRIDE(W), 0, H);
  uint32_t enc = micros() - t;

  size_t total = 0;
  for (int y = 0; y < H; y++) total += lens[y];
  Serial.println("NanoGFX dual-mode encode");
  Serial.printf("frame: %dx%d = %u packed bytes -> %u encoded bytes in %lu us\n",
                W, H, (unsigned)PackCanvas::bufBytes(W, H, true), (unsigned)total,
                (unsigned long)enc);
  Serial.println("per-line encoded sizes (note line 90, the exploded one):");
  for (int y = 0; y < H; y += 6) {
    Serial.printf("  y=%3d:", y);
    for (int k = y; k < y + 6 && k < H; k++) Serial.printf(" %4u", lens[k]);
    Serial.println();
  }
  // Typical output: blank/spliced lines encode to 2-8 bytes; the scattered
  // line 90 costs the word-batched flat scan and a few dozen bytes. The
  // worst case for ANY line is PRLE_STRIDE(W) bytes — guaranteed by the codec.
}

void loop() {}
