// ============================================================================
//  EInkGhost.h — which kind of e-paper refresh a change deserves, cell by cell.
//
//  E-paper has three refreshes and they trade speed against cleanliness:
//    PARTIAL  fast waveform, only changed pixels move, no flash — but every one leaves a
//             little ghost of what was there, and ghosts add up.
//    CLEAN    the full waveform inside a window: that region flashes, nothing else does,
//             and the region's ghosts are gone.
//    FULL     the full waveform everywhere (whole-screen flash).
//
//  WEAR IS COUNTED PER CELL (8 x 8 pixels: one byte column of 8 rows), in FULL TOGGLES: every
//  pixel a partial refresh flips adds 1/64 to its cell, so a partial that flips the whole cell
//  adds 1, one that flips 16 of its pixels adds 1/4. That is what the fast waveform's greying
//  follows (measured on a UC8179, see the NanoGFX docs). A progress bar that advances one pixel a
//  tick wears its edge cell by 1/64 a tick; an art frame wears the cells it changed by how much
//  of them it changed. (v1 kept one counter per row and charged any change as a full redraw.)
//
//  The policy, all of it the caller's to schedule:
//  * a change whose changed cells would pass the budget is done as a CLEAN window covering those
//    cells (and the rest of the change), never a full refresh;
//  * optional early clean (setEarlyClean, off by default): an EVENT change landing on cells
//    already a third worn is cleaned while it is redrawn anyway;
//  * headroom(rect): how many more partials a region takes before it must be cleaned — for a
//    caller that stretches a ticking region's cadence so the budget lasts until its next FULL;
//  * maxWear()/idleClean(): the worst cell, and the worn region of a quiet screen;
//  * the budget shrinks in the cold, where fast waveforms ghost more.
//
//  It knows nothing about the controller: 1-bit frames in, a Plan out. Pair it with
//  UC8179::startPartial(..., clean, xb0, xb1) / startFull and call done() when the glass is.
//  Memory: 6 bytes per cell (an 800x480 panel: 6 000 cells, 36 KB), from malloc.
// ============================================================================
#pragma once
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

class EInkGhost {
public:
  enum Kind : uint8_t { NONE, PARTIAL, CLEAN, FULL };
  struct Plan { Kind kind; int y0, y1, xb0, xb1; };
  static constexpr int CELL_ROWS = 8;         // a cell: one byte column (8 px) x 8 rows
  static constexpr uint16_t EVENT_WEAR = 1;   // kept for v1 callers: every change counts 1 per cell

  // `budget`: full toggles a cell takes before it is cleaned.
  EInkGhost(int height, int rowBytes, uint16_t budget = 160)
      : h_(height), rb_(rowBytes), ch_((height + CELL_ROWS - 1) / CELL_ROWS), base_(budget), budget_(budget) {
    const size_t n = (size_t) ch_ * (size_t) rb_;
    wear_ = (uint32_t *) calloc(n, sizeof(uint32_t));
    flips_ = (uint16_t *) calloc(n, sizeof(uint16_t));
  }
  ~EInkGhost() { free(wear_); free(flips_); }

  // Room temperature in C: below 18 C the budget is two thirds, below 10 C half. NaN: unknown.
  void setTemperature(float c) {
    temp_ = c;
    applyBudget();
  }
  void setBudget(uint16_t b) {
    base_ = b < 2 ? 2 : b;
    applyBudget();
  }
  void setEarlyClean(bool on) { early_ = on; }
  uint16_t budget() const { return budget_; }

  // The most worn cell in row y (v1's per-row view), in full toggles.
  uint16_t wear(int y) const {
    if (y < 0 || y >= h_) return 0;
    uint32_t m = 0;
    const uint32_t *r = wear_ + (size_t) (y / CELL_ROWS) * rb_;
    for (int b = 0; b < rb_; b++) if (r[b] > m) m = r[b];
    return (uint16_t) (m / PX);
  }
  uint16_t maxWear() const {
    uint32_t m = 0;
    for (size_t i = 0, n = (size_t) ch_ * rb_; i < n; i++) if (wear_[i] > m) m = wear_[i];
    return (uint16_t) (m / PX);
  }
  // Partials the rect [y0,y1) x bytes [xb0,xb1) takes before its most worn cell reaches the budget.
  int headroom(int y0, int y1, int xb0, int xb1) const {
    if (y1 <= y0 || xb1 <= xb0) return budget_;
    uint32_t m = 0;
    for (int cy = clampRow(y0) / CELL_ROWS; cy <= clampRow(y1 - 1) / CELL_ROWS && cy < ch_; cy++)
      for (int b = xb0 < 0 ? 0 : xb0; b < xb1 && b < rb_; b++)
        if (wear_[(size_t) cy * rb_ + b] > m) m = wear_[(size_t) cy * rb_ + b];
    const uint32_t cap = (uint32_t) budget_ * PX;
    return m >= cap ? 0 : (int) ((cap - m) / PX);
  }

  // What to do to get `frame` onto glass that shows `glass`. `ambient` = nothing but a
  // self-ticking value moved (no new data, no button); it only matters for early clean.
  Plan plan(const uint8_t *glass, const uint8_t *frame, bool forceFull = false, bool ambient = false) {
    memset(flips_, 0, (size_t) ch_ * rb_ * sizeof(uint16_t));
    int first = -1, last = -1, xa = rb_, xb = 0;
    int oy0 = h_, oy1 = -1, ox0 = rb_, ox1 = -1;          // cells that would pass the budget
    bool worn = false;
    for (int y = 0; y < h_; y++) {
      const uint8_t *g = glass + (size_t) y * rb_, *f = frame + (size_t) y * rb_;
      int a = 0, b = rb_ - 1;
      while (a < rb_ && g[a] == f[a]) a++;
      if (a == rb_) continue;
      while (b > a && g[b] == f[b]) b--;
      if (first < 0) first = y;
      last = y;
      if (a < xa) xa = a;
      if (b + 1 > xb) xb = b + 1;
      uint16_t *fl = flips_ + (size_t) (y / CELL_ROWS) * rb_;
      for (int x = a; x <= b; x++) fl[x] = (uint16_t) (fl[x] + __builtin_popcount((unsigned) (g[x] ^ f[x])));
    }
    const uint32_t cap = (uint32_t) budget_ * PX;
    for (int cy = 0; cy < ch_; cy++)
      for (int x = 0; x < rb_; x++) {
        const size_t i = (size_t) cy * rb_ + x;
        if (!flips_[i]) continue;
        if (wear_[i] + flips_[i] > cap) {
          if (cy * CELL_ROWS < oy0) oy0 = cy * CELL_ROWS;
          if (cy * CELL_ROWS + CELL_ROWS > oy1) oy1 = cy * CELL_ROWS + CELL_ROWS;
          if (x < ox0) ox0 = x;
          if (x + 1 > ox1) ox1 = x + 1;
        }
        if (early_ && !ambient && wear_[i] >= cap / 3) worn = true;
      }
    if (forceFull) return {FULL, 0, h_, 0, rb_};
    if (first < 0) return {NONE, 0, 0, 0, 0};
    Plan p{PARTIAL, first, last + 1, xa, xb};
    if (oy1 > oy0 || worn) {
      p.kind = CLEAN;
      if (oy1 > oy0) {                     // the clean covers the change and the worn cells
        if (oy0 < p.y0) p.y0 = oy0;
        if (oy1 > p.y1) p.y1 = oy1;
        if (ox0 < p.xb0) p.xb0 = ox0;
        if (ox1 > p.xb1) p.xb1 = ox1;
      }
      // whole cells, so the clean can forgive every cell it touched
      p.y0 = p.y0 / CELL_ROWS * CELL_ROWS;
      p.y1 = (p.y1 + CELL_ROWS - 1) / CELL_ROWS * CELL_ROWS;
      if (p.y1 > h_) p.y1 = h_;
    }
    return p;
  }

  // The worn region a quiet screen could clean now (cells at >= `share` of the budget).
  bool idleClean(Plan *out, float share = 0.25f) const {
    int y0 = h_, y1 = -1, x0 = rb_, x1 = -1;
    const uint32_t floor = (uint32_t) (budget_ * share * PX);
    for (int cy = 0; cy < ch_; cy++)
      for (int b = 0; b < rb_; b++) {
        const uint32_t w = wear_[(size_t) cy * rb_ + b];
        if (w == 0 || w < floor) continue;
        if (cy * CELL_ROWS < y0) y0 = cy * CELL_ROWS;
        if (cy * CELL_ROWS + CELL_ROWS > y1) y1 = cy * CELL_ROWS + CELL_ROWS;
        if (b < x0) x0 = b;
        if (b + 1 > x1) x1 = b + 1;
      }
    if (y1 <= y0) return false;
    *out = {CLEAN, y0, y1 > h_ ? h_ : y1, x0, x1};
    return true;
  }

  // The glass finished `p` (after the plan() that produced it: the changed cells are that plan's).
  void done(const Plan &p) {
    const size_t n = (size_t) ch_ * rb_;
    if (p.kind == FULL) {
      memset(wear_, 0, n * sizeof(uint32_t));
      return;
    }
    if (p.kind == CLEAN) {       // a clean forgives the cells wholly inside its window
      for (int cy = 0; cy < ch_; cy++) {
        const int r0 = cy * CELL_ROWS, r1 = r0 + CELL_ROWS > h_ ? h_ : r0 + CELL_ROWS;
        if (r0 < p.y0 || r1 > p.y1) continue;
        for (int b = p.xb0 < 0 ? 0 : p.xb0; b < p.xb1 && b < rb_; b++) wear_[(size_t) cy * rb_ + b] = 0;
      }
      return;
    }
    if (p.kind == PARTIAL) {
      uint32_t m = 0, step = 0;
      for (size_t i = 0; i < n; i++)
        if (flips_[i]) {
          wear_[i] += flips_[i];
          if (wear_[i] > m) { m = wear_[i]; step = flips_[i]; }
        }
      last_max_ = m;
      last_step_ = step;
    }
  }
  // How many more partials like the last one its most worn changed cell takes before the budget
  // (the region that just ticked): budget left / what the last partial cost that cell.
  int lastHeadroom() const {
    const uint32_t cap = (uint32_t) budget_ * PX;
    if (last_max_ >= cap) return 0;
    return last_step_ ? (int) ((cap - last_max_) / last_step_) : (int) ((cap - last_max_) / PX);
  }

private:
  int h_, rb_, ch_;
  uint16_t base_, budget_;
  float temp_ = 0.0f / 0.0f;
  bool early_ = false;
  static constexpr uint32_t PX = 64;   // pixel flips in one full toggle of a cell
  uint32_t *wear_;            // pixel flips per cell since its last clean
  uint16_t *flips_;           // this plan's pixel flips per cell
  uint32_t last_max_ = 0, last_step_ = 0;   // the last PARTIAL's most worn cell, and its cost there
  int clampRow(int y) const { return y < 0 ? 0 : y >= h_ ? h_ - 1 : y; }
  void applyBudget() {
    const float c = temp_;
    budget_ = c != c ? base_ : c < 10 ? base_ / 2 : c < 18 ? (uint16_t) (base_ * 2 / 3) : base_;
    if (budget_ < 2) budget_ = 2;
  }
};
