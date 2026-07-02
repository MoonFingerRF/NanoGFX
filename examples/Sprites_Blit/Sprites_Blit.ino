// NanoGFX: Sprites_Blit
//
// Canvas-to-canvas blits — NanoGFX's sprite mechanism. Any PackCanvas can be a
// sprite: draw it once, then stamp it onto another canvas with blit()/blitRect(),
// optionally treating one palette index as TRANSPARENT.
//
// Because both canvases are 2 px/byte, an opaque blit at even x alignment is a
// straight memcpy per row; run-form source rows blit as SPANS (no per-pixel work
// at all). The general case (odd alignment / transparency) is an exact nibble
// loop. drawIndexedBitmap() does the same for flash-resident images.
#include <NanoGFX.h>

PackCanvas screen(240, 160);
PackCanvas ship(24, 16);        // a small sprite, drawn once

// A 12x8 flash-resident packed image (2 px/byte, high nibble first, stride 6).
// Index 0 is used as the transparent key below. 15 colors max: nibble 0xF is
// the codec's escape and must never appear in image data.
static const uint8_t STAR[6 * 8] PROGMEM = {
  0x00,0x05,0x50,0x00,0x00,0x00,  0x00,0x57,0x75,0x00,0x00,0x00,
  0x05,0x77,0x77,0x50,0x00,0x00,  0x57,0x77,0x77,0x75,0x00,0x00,
  0x05,0x77,0x77,0x50,0x00,0x00,  0x00,0x57,0x75,0x00,0x00,0x00,
  0x00,0x05,0x50,0x00,0x00,0x00,  0x00,0x00,0x00,0x00,0x00,0x00,
};

void setup() {
  Serial.begin(115200);
  delay(1500);
  screen.packed4 = true; screen.dualMode = true;
  ship.packed4 = true;

  // Draw the sprite once: color 0 will be its transparent key.
  ship.fillScreen(0);
  ship.fillTriangle(2, 14, 12, 1, 22, 14, 6);
  ship.fillRect(10, 8, 4, 6, 2);

  screen.fillScreen(1);                       // sky
  screen.fillRect(0, 130, 240, 30, 11);       // ground

  uint32_t t = micros();
  for (int i = 0; i < 8; i++)
    screen.blit(ship, (int16_t)(10 + i * 28), (int16_t)(30 + (i & 1) * 20), /*transparent*/ 0);
  uint32_t tBlit = micros() - t;

  t = micros();
  screen.blitRect(ship, 6, 4, 12, 8, 100, 90, 0);   // a sub-rect of the sprite
  uint32_t tRect = micros() - t;

  screen.drawIndexedBitmap(180, 95, STAR, 12, 8, /*transparent*/ 0);

  Serial.println("NanoGFX sprites");
  Serial.printf("8 transparent 24x16 blits: %lu us; sub-rect blit: %lu us\n",
                (unsigned long)tBlit, (unsigned long)tRect);
  Serial.printf("spot checks: sky=%d ship=%d ground=%d\n",
                screen.getPixel(0, 0), screen.getPixel(22, 43), screen.getPixel(5, 140));
  // getPixel() is format-aware: it walks run-form lines without disturbing them,
  // so reading never costs you the compression the drawing built up.
}

void loop() {}
