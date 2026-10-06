// Host test for ST7305Pack.h: the fast packers against the per-pixel reference, every orientation,
// random and structured canvases; the change window and gather. Build and run:
//   c++ -std=c++17 -O2 -Wall -Wextra -Isrc tests/st7305_host.cpp -o /tmp/st7305_host && /tmp/st7305_host
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "ST7305Pack.h"

using namespace st7305;

static uint32_t rng = 12345;
static uint8_t rnd() {
  rng = rng * 1664525u + 1013904223u;
  return (uint8_t) (rng >> 24);
}

static int failures = 0;
#define CHECK(c, ...)                                   \
  do {                                                  \
    if (!(c)) {                                         \
      failures++;                                       \
      printf("FAIL %s:%d: ", __FILE__, __LINE__);       \
      printf(__VA_ARGS__);                              \
      printf("\n");                                     \
    }                                                   \
  } while (0)

int main() {
  const char *names[] = {"portrait", "landscape", "portrait_180", "landscape_180"};
  for (int oi = 0; oi < 4; oi++) {
    const Orientation o = (Orientation) oi;
    const int W = canvasWidth(o), H = canvasHeight(o), S = canvasStride(o);
    std::vector<uint8_t> rows((size_t) S * H), fast(FRAME_BYTES), ref(FRAME_BYTES);
    for (int pass = 0; pass < 6; pass++) {
      for (auto &b : rows) b = pass == 0 ? 0 : pass == 1 ? 0xFF : rnd();
      if (pass == 2) {   // single pixels at the corners and edges
        for (auto &b : rows) b = 0;
        const int pts[][2] = {{0, 0}, {W - 1, 0}, {0, H - 1}, {W - 1, H - 1}, {W / 2, H / 3}, {1, 1}, {W - 2, H - 3}};
        for (auto &p : pts) rows[p[1] * S + (p[0] >> 3)] |= (uint8_t) (0x80 >> (p[0] & 7));
      }
      // bits beyond W in a row's last byte are padding: the reference ignores them, so must the packer
      pack(rows.data(), S, o, fast.data());
      packReference(rows.data(), S, o, ref.data());
      int diff = 0;
      for (int i = 0; i < FRAME_BYTES; i++) diff += fast[i] != ref[i];
      CHECK(diff == 0, "%s pass %d: %d bytes differ", names[oi], pass, diff);
    }
    // timing (host, indicative only)
    for (auto &b : rows) b = rnd();
    const auto t0 = std::chrono::steady_clock::now();
    const int N = 2000;
    for (int i = 0; i < N; i++) pack(rows.data(), S, o, fast.data());
    const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / N;
    printf("%-14s %dx%d stride %d: pack %.1f us/frame (host)\n", names[oi], W, H, S, us);
  }

  // change window + gather
  std::vector<uint8_t> a(FRAME_BYTES), b(FRAME_BYTES), out(FRAME_BYTES);
  for (auto &x : a) x = rnd();
  b = a;
  CHECK(rectOfChange(a.data(), b.data()).empty(), "equal frames give a window");
  b[10 * PAGE_BYTES + 7] ^= 1;      // page 10, unit 2
  b[42 * PAGE_BYTES + 70] ^= 0x80;  // page 42, unit 23
  Window w = rectOfChange(a.data(), b.data());
  CHECK(w.p0 == 10 && w.p1 == 42 && w.u0 == 2 && w.u1 == 23, "window %d..%d x %d..%d", w.p0, w.p1, w.u0, w.u1);
  const int n = gather(b.data(), w, out.data());
  CHECK(n == (42 - 10 + 1) * (23 - 2 + 1) * 3, "gather bytes %d", n);
  CHECK(out[0] == b[10 * PAGE_BYTES + 6], "gather first byte");
  CHECK(out[n - 1] == b[42 * PAGE_BYTES + 23 * 3 + 2], "gather last byte");
  CHECK(Window::full().bytes() == FRAME_BYTES, "full window bytes");

  printf(failures ? "FAILED (%d)\n" : "all ST7305 pack tests passed\n", failures);
  return failures ? 1 : 0;
}
