// NanoGFX: ScrollingPlot
//
// A live strip-chart (the classic "scrolling telemetry plot") built on
// vScroll(). On a packed canvas every row is a self-contained record — slot
// bytes + line mode + run count — so a vertical scroll MOVES LINE RECORDS
// with three memmoves: no per-pixel work, and rows that were in run form are
// STILL in run form afterwards. Scrolling a 240x160 region costs microseconds
// and doesn't disturb the compression that encodeFrame()/PackFlush rely on.
//
// (For a horizontally-scrolling chart, draw into a canvas rotated 90 degrees
// in your layout — the fast axis is rows by design.)
#include <NanoGFX.h>

#define W 240
#define H 160

PackCanvas plot(W, H);

void setup() {
  Serial.begin(115200);
  delay(1500);
  plot.packed4 = true; plot.dualMode = true;
  plot.fillScreen(0);
}

void loop() {
  static float phase = 0;
  phase += 0.15f;

  uint32_t t = micros();
  plot.vScroll(0, H, -1, /*fill*/ 0);        // scroll UP one row; bottom row vacated
  uint32_t tScroll = micros() - t;

  // Draw the newest sample row at the bottom: a dot per channel + a grid tick.
  int v1 = (int)(W / 2 + sinf(phase) * 90);
  int v2 = (int)(W / 2 + cosf(phase * 0.7f) * 60);
  if (((int)(phase / 0.15f)) % 25 == 0)      // horizontal grid line every 25 samples
    plot.drawFastHLine(0, H - 1, W, 8);      //   -> ONE run: stays compressed
  plot.drawPixel((int16_t)v1, H - 1, 6);
  plot.drawPixel((int16_t)v2, H - 1, 2);

  static uint32_t frames = 0, scrollSum = 0, lastPrint = 0;
  scrollSum += tScroll; frames++;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    int runRows = 0;
    for (int y = 0; y < H; y++) runRows += plot.lineIsRuns(y);
    Serial.printf("vScroll avg %lu us; %d/%d rows still run-form after %lu scrolls\n",
                  (unsigned long)(scrollSum / frames), runRows, H, (unsigned long)frames);
    frames = 0; scrollSum = 0;
  }
  delay(16);
}
