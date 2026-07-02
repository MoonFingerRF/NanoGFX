// NanoGFX: PackedCanvas_Basics
//
// The core idea in one sketch: a 15-color canvas that stores 2 PIXELS PER BYTE.
// Draws a small gauge face two ways — into a classic 8-bit palette canvas and
// into a 4-bit packed one — and prints the RAM and time cost of each.
//
// Works on any board with enough RAM for the two canvases (~86 KB total here);
// no display required (this example is about the canvas itself).
#include <NanoGFX.h>

#define W 240
#define H 240

PackCanvas c8(W, H);     // classic 1 byte/px
PackCanvas c4(W, H);     // 2 px/byte

// A little gauge face: bezel rings, tick marks, needle, label.
static void drawGauge(PackCanvas &c) {
  c.fillScreen(0);
  c.drawCircle(W / 2, H / 2, 110, 7);
  c.drawCircle(W / 2, H / 2, 104, 8);
  for (int a = -60; a <= 240; a += 30) {              // major ticks
    float r = a * DEG_TO_RAD;
    int x0 = W / 2 + (int)(88 * cosf(r)), y0 = H / 2 - (int)(88 * sinf(r));
    int x1 = W / 2 + (int)(100 * cosf(r)), y1 = H / 2 - (int)(100 * sinf(r));
    c.drawLine(x0, y0, x1, y1, 7);
  }
  c.fillTriangle(W / 2 - 4, H / 2, W / 2 + 4, H / 2, W / 2 + 60, H / 2 - 55, 2);  // needle
  c.fillCircle(W / 2, H / 2, 8, 7);
  c.setCursor(W / 2 - 20, H - 40);
  c.setTextColor(5);
  c.setTextSize(2);
  c.print("RPM");
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  c4.packed4 = true;                    // BEFORE the first drawing call

  Serial.println("NanoGFX PackedCanvas basics");
  Serial.printf("8-bit canvas buffer: %u bytes\n", (unsigned)PackCanvas::bufBytes(W, H, false));
  Serial.printf("4-bit canvas buffer: %u bytes  (2 px/byte)\n", (unsigned)PackCanvas::bufBytes(W, H, true));

  uint32_t t = micros(); drawGauge(c8); uint32_t t8 = micros() - t;
  t = micros();          drawGauge(c4); uint32_t t4 = micros() - t;
  Serial.printf("gauge draw: 8-bit %lu us, 4-bit %lu us\n", (unsigned long)t8, (unsigned long)t4);

  // Fills touch HALF the bytes on the packed canvas:
  t = micros(); c8.fillScreen(3); t8 = micros() - t;
  t = micros(); c4.fillScreen(3); t4 = micros() - t;
  Serial.printf("fillScreen: 8-bit %lu us, 4-bit %lu us\n", (unsigned long)t8, (unsigned long)t4);

  // To feed a display, expand nibbles through your palette. One byte = TWO pixels,
  // so a 256-entry pair LUT converts 2 px per lookup — see RM690B0_DirtyPush for
  // the full zero-copy pipeline (encode + dirty bands + QSPI).
}

void loop() {}
