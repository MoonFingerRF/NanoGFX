// InkArt_Basics — PackInk.h: textured 1-bit drawing, composited onto a PackCanvas.
//
// A moon behind cloud texture, hatched hills and a word, on a 240x128 ink bitmap, then blitted
// into a packed canvas (index 1 = ink, 0 = paper) ready for PackMono -> e-paper. Every rule is
// integer-exact, so the same calls on a laptop give the same pixels.
#include <NanoGFX.h>
#include <PackInk.h>

using namespace packink;

static const int W = 240, H = 128;
static uint8_t artBuf[W / 8 * H], covBuf[W / 8 * H];
PackCanvas canvas(400, 300, true);

void drawArt() {
  InkBits art(W, H, artBuf);
  InkCoverage cov(W, H, covBuf);
  art.clear();
  Pen pen;

  cov.reset(); fillRect(cov, 0, 0, W, H);                       // sky: value-noise cloud
  pen.texture = CLOUD; pen.param = 24; pen.level = 7; pen.seed = 4;
  paint(art, cov, pen, 0, 0);

  cov.reset(); fillEllipse(cov, 180, 40, 22, 22);               // the moon cuts through it
  pen.mode = ERASE;
  paint(art, cov, pen, 0, 0);

  cov.reset();                                                  // hills: hatched, covering
  const int32_t hills[] = {0, 128, 0, 96, 60, 80, 130, 100, 200, 84, 240, 94, 240, 128};
  fillPoly(cov, hills, 7);
  pen.mode = COVER; pen.texture = HATCH; pen.param = 5; pen.level = 10;
  paint(art, cov, pen, 0, 0);

  cov.reset(); text(cov, 8, 8, 2, "night", 5, 0, LEFT);         // a word, flipped over the sky
  pen.mode = INVERT; pen.texture = FLAT; pen.level = 16;
  paint(art, cov, pen, 0, 0);

  blit(canvas, 80, 86, art, 1, 0);
}

void setup() {
  canvas.packed4 = true;
  canvas.fillScreen(0);
  drawArt();
}

void loop() {}
