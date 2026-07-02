// NanoGFX: RM690B0_DirtyPush (ESP32-S3)
//
// The complete NanoPFD pipeline on a LilyGO T4-S3 (2.41" RM690B0 QSPI AMOLED,
// 450x600 used portrait):
//
//   dual-mode PackCanvas  ->  encodeFrame (RLE mostly fused into drawing)
//     ->  PackFlush (send ONLY the rows that changed)
//       ->  split-polling QSPI push (decode chunk n+1 while chunk n is on the wire)
//
// A clock face redraws its moving needle each frame; everything else on the
// panel is static, so PackFlush sends a handful of rows per frame. Per-second
// stats print rows pushed, decode time, and total push time — watch the push
// collapse when only the needle moves.
//
// This is the architecture that runs NanoPFD's 450x600 glass cockpit at
// 46-48 fps WITH a WiFi AP + BLE running on the same chip.
#include <NanoGFX.h>
#include <RM690B0.h>
#include "esp_heap_caps.h"

#define W 450
#define H 600
#define SLOT PRLE_STRIDE(W)     // fixed per-line slot: encode is NEVER larger
#define LPC  36                 // lines per push chunk (36*450 = 16.2 KB bounce)

// LilyGO T4-S3 wiring: D0=14 D1=10 D2=16 D3=12 SCK=15 CS=11 RST=13 PMIC-EN=9
static const RM690B0::Pins PINS = { 14, 10, 16, 12, 15, 11, 13, 9 };
RM690B0    amo;
PackCanvas cv(W, H, false);     // buffer supplied below (PSRAM-sized board: use internal if you can)
PackFlush  flushr;

uint8_t  *comp, *glass, *bounce;
uint16_t lens[H], glassLens[H];
uint8_t  lut332[16];            // palette index -> RGB332 (the panel runs 8 bpp)
uint16_t lutPair[256];          // 2 px per lookup

static const uint16_t PAL565[15] = {   // 15 colors max — nibble 0xF is the RLE escape
  0x0000, 0xFFFF, 0xF800, 0x07E0, 0x001F, 0xFFE0, 0x07FF, 0xC618,
  0x8410, 0xFD20, 0x8000, 0x0400, 0x0010, 0xF81F, 0x39E7 };

static void buildPalette() {
  for (int i = 0; i < 15; i++) {
    uint16_t c = PAL565[i];
    lut332[i] = (uint8_t)(((c >> 13) & 7) << 5 | ((c >> 8) & 7) << 2 | ((c >> 3) & 3));
  }
  lut332[15] = 0;
  for (int b = 0; b < 256; b++)
    lutPair[b] = (uint16_t)(lut332[b >> 4] | (lut332[b & 0x0F] << 8));
}

// Decode + write rows [y0, y1]: while chunk n rides the QSPI DMA, the CPU
// decodes chunk n+1 into the OTHER bounce half (split polling).
static uint32_t gDecUs, gRows;
static void pushBand(int y0, int y1) {
  amo.setAddrWindow(0, y0, W - 1, y1);
  amo.ramBegin();
  bool inflight = false;
  int bb = 0;
  for (int yb = y0; yb <= y1; yb += LPC) {
    int n = y1 + 1 - yb; if (n > LPC) n = LPC;
    uint8_t *dst = bounce + (size_t)bb * LPC * W;
    uint32_t t = micros();
    for (int j = 0; j < n; j++)
      prle_decode_lut8p(comp + (size_t)(yb + j) * SLOT, lens[yb + j], W,
                        dst + (size_t)j * W, lut332, lutPair);
    gDecUs += micros() - t;
    if (inflight) amo.ramWriteEnd();
    amo.ramWriteStart(dst, (uint32_t)n * W, yb == y0);
    inflight = true;
    bb ^= 1;
  }
  if (inflight) amo.ramWriteEnd();
  amo.ramEnd();
  gRows += y1 + 1 - y0;
}

static void drawFace(float ang) {
  // static face (these rows won't be re-sent once on glass)…
  static bool once = false;
  if (!once) {
    once = true;
    cv.fillScreen(0);
    cv.drawCircle(W / 2, 300, 200, 7);
    for (int a = 0; a < 360; a += 30) {
      float r = a * DEG_TO_RAD;
      cv.drawLine(W / 2 + 180 * cosf(r), 300 - 180 * sinf(r),
                  W / 2 + 196 * cosf(r), 300 - 196 * sinf(r), 7);
    }
    cv.setTextSize(3); cv.setTextColor(5);
    cv.setCursor(W / 2 - 70, 540); cv.print("NanoGFX");
  }
  // …moving needle (only the rows it sweeps become dirty)
  static int px1 = -1, py1 = -1;
  if (px1 >= 0) cv.drawLine(W / 2, 300, px1, py1, 0);   // erase old
  int x1 = W / 2 + (int)(170 * cosf(ang)), y1 = 300 - (int)(170 * sinf(ang));
  cv.drawLine(W / 2, 300, x1, y1, 2);
  px1 = x1; py1 = y1;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  cv.packed4 = true;
  cv.dualMode = true;
  cv.useBuffer((uint8_t *)heap_caps_malloc(PackCanvas::bufBytes(W, H, true),
                                           MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  comp   = (uint8_t *)heap_caps_malloc((size_t)H * SLOT, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  glass  = (uint8_t *)heap_caps_malloc((size_t)H * SLOT, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  bounce = (uint8_t *)heap_caps_malloc((size_t)2 * LPC * W, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  buildPalette();
  flushr.begin(glass, glassLens, H, SLOT);
  amo.begin(SPI3_HOST, PINS, 80000000, LPC * W * 16 + 8,
            /*madctl*/ 0x00, /*offX*/ 16, /*offY*/ 0, /*brightness*/ 200);
}

void loop() {
  static float ang = 0;
  static uint32_t frames = 0, rowsSum = 0, decSum = 0, pushSum = 0, lastPrint = 0;
  ang += 0.03f;
  drawFace(ang);

  cv.encodeFrame(comp, lens, SLOT, 0, H);
  gDecUs = 0; gRows = 0;
  uint32_t t = micros();
  flushr.flush(comp, lens, /*gap*/ 8, pushBand);
  pushSum += micros() - t;
  rowsSum += gRows; decSum += gDecUs; frames++;

  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    Serial.printf("fps=%lu  rows/frame=%lu/600  dec=%luus  push=%luus\n",
                  (unsigned long)frames, (unsigned long)(rowsSum / frames),
                  (unsigned long)(decSum / frames), (unsigned long)(pushSum / frames));
    frames = rowsSum = decSum = pushSum = 0;
  }
}
