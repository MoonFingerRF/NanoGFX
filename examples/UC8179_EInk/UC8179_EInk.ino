// NanoGFX: UC8179_EInk (ESP32-S3 with PSRAM)
//
// A 7.5" 800x480 UC8179 e-paper panel (Seeed reTerminal E1001, Waveshare 7.5" V2,
// GoodDisplay GDEY075T7) driven the NanoGFX way:
//
//   PackCanvas (4-bit, 15 palette indices)  ->  PackMono (greys become a 4x4 dither)
//     ->  EInkGhost (PARTIAL / CLEAN window / FULL, from per-cell wear)
//       ->  UC8179 (non-blocking: loop() never waits for the glass)
//
// A progress bar ticks once a second (an "ambient" change: a tiny partial window, no
// flash) and a counter changes every 10 s (an "event"). Watch the serial log: the bar
// costs one small partial per second, worn rows get a CLEAN window of their own, and the
// whole screen is only flashed at start-up.
//
// Pins are the reTerminal E1001's (Seeed's cookbook). Change them for your board.
// BUSY on the E1001 is active LOW; on a bare Waveshare HAT it is active high.
#include <SPI.h>
#include <NanoGFX.h>
#include <UC8179.h>

static const int PIN_SCK = 7, PIN_MOSI = 9, PIN_CS = 10, PIN_DC = 11, PIN_RST = 12, PIN_BUSY = 13;
static const bool BUSY_ACTIVE_LOW = true;
static const bool GLASS_INVERTED = true;       // the E1001 comes out negative without it

struct Bus {
  void command(uint8_t c) {
    digitalWrite(PIN_DC, LOW);
    digitalWrite(PIN_CS, LOW);
    SPI.transfer(c);
    digitalWrite(PIN_CS, HIGH);
  }
  void data(const uint8_t *p, size_t n) {
    digitalWrite(PIN_DC, HIGH);
    digitalWrite(PIN_CS, LOW);
    SPI.writeBytes(p, n);                       // from the driver's own (internal) line buffer
    digitalWrite(PIN_CS, HIGH);
  }
  bool busy() { return digitalRead(PIN_BUSY) == (BUSY_ACTIVE_LOW ? LOW : HIGH); }
  void reset(bool high) { digitalWrite(PIN_RST, high ? HIGH : LOW); }
  void delayMs(uint32_t ms) { delay(ms); }
  uint32_t millis() { return ::millis(); }
};

static const int W = 800, H = 480, ROW = W / 8;
enum : uint8_t { PAPER = 0, INK = 1, GREY = 2, LIGHT = 3 };

Bus bus;
UC8179<Bus> epd(bus);
PackCanvas canvas(W, H, false);                // buffer supplied from PSRAM below
PackMono mono;
EInkGhost ghost(H, ROW, 80);                 // 80 full toggles a cell (measured on a UC8179)
uint8_t *frame, *glass;                        // 1-bit: what should be / what is on the glass
EInkGhost::Plan running{EInkGhost::NONE, 0, 0, 0, 0};
bool firstFrame = true;

static void draw(uint32_t seconds) {
  canvas.fillScreen(PAPER);
  canvas.setTextColor(INK);
  canvas.setTextSize(3);
  canvas.setCursor(24, 24);
  canvas.print("NanoGFX on e-paper");
  canvas.drawFastHLine(0, 70, W, INK);

  canvas.fillRect(24, 110, 360, 200, LIGHT);   // a grey panel: flat on the canvas, dots on glass
  canvas.setTextSize(8);
  canvas.setCursor(48, 160);
  canvas.printf("%lu", (unsigned long)(seconds / 10));   // changes every 10 s: an event

  const int bx = 24, by = 400, bw = 752, bh = 16;       // ticks every second: ambient
  canvas.fillRect(bx, by, bw, bh, GREY);
  canvas.fillRect(bx, by, (int)(bw * (seconds % 60) / 59), bh, INK);
}

static void startPlan(const EInkGhost::Plan &p) {
  bool ok = p.kind == EInkGhost::FULL ? epd.startFull(frame)
          : epd.startPartial(frame, glass, p.y0, p.y1, p.kind == EInkGhost::CLEAN, p.xb0, p.xb1);
  if (ok) running = p;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_CS, OUTPUT);  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_DC, OUTPUT);
  pinMode(PIN_RST, OUTPUT);
  pinMode(PIN_BUSY, INPUT_PULLUP);
  SPI.begin(PIN_SCK, -1, PIN_MOSI, -1);
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));

  canvas.packed4 = true;                       // 800x480 at 2 px/byte = 192 KB
  canvas.useBuffer((uint8_t *)ps_malloc(PackCanvas::bufBytes(W, H, true)));
  frame = (uint8_t *)ps_calloc(ROW * H, 1);    // 48 KB each
  glass = (uint8_t *)ps_calloc(ROW * H, 1);
  mono.setLevel(GREY, 6);                      // ink levels 0..16 -> Bayer dither density
  mono.setLevel(LIGHT, 2);

  epd.invert(GLASS_INVERTED);
  epd.begin();
}

void loop() {
  epd.poll();                                  // advances the refresh; returns at once
  if (!epd.isIdle()) return;
  if (running.kind != EInkGhost::NONE) {       // the glass finished the last plan
    ghost.done(running);
    memcpy(glass, frame, ROW * H);
    const UC8179<Bus>::Timing &t = epd.lastTiming();
    Serial.printf("%s rows %d-%d bytes %d-%d: %u ms (rails %u, spi %u, waveform %u)\n",
                  running.kind == EInkGhost::FULL ? "FULL" : running.kind == EInkGhost::CLEAN ? "CLEAN" : "partial",
                  running.y0, running.y1, running.xb0, running.xb1, (unsigned)epd.lastRefreshMs(),
                  t.power, t.send, t.refresh);
    running.kind = EInkGhost::NONE;
  }

  static uint32_t last = UINT32_MAX;
  const uint32_t seconds = millis() / 1000;
  if (seconds == last) return;
  last = seconds;

  draw(seconds);
  mono.exportRows(canvas, frame, 0, H);        // 1 = ink, MSB-first, 100 bytes a row
  const bool ambient = seconds % 10 != 0;      // only the bar moved
  epd.stayPowered(true);                       // a run of 1 Hz updates: keep the rails up
  EInkGhost::Plan p = ghost.plan(glass, frame, firstFrame, ambient);
  firstFrame = false;
  if (p.kind == EInkGhost::NONE && !ghost.idleClean(&p)) return;
  startPlan(p);
}
