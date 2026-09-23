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
//  WEAR IS COUNTED PER CELL (8 x 8 pixels: one byte column of 8 rows), not per row: a cell
//  gains 1 for every partial refresh that changed any pixel in it, and nothing otherwise. A
//  progress bar that advances one pixel a tick wears only the cell its edge is in; an art frame
//  that redraws a block wears exactly that block's cells; a clock digit wears its digit. So a
//  region is cleaned for what really happened to it, and only it (v1 kept one counter per row,
//  which charged a one-pixel tick like a full redraw of the row).
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
//  Memory: 3 bytes per cell (an 800x480 panel: 6 000 cells, 18 KB), from malloc.
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

  // `budget`: partial refreshes a cell takes before it is cleaned.
  EInkGhost(int height, int rowBytes, uint16_t budget = 160)
      : h_(height), rb_(rowBytes), ch_((height + CELL_ROWS - 1) / CELL_ROWS), base_(budget), budget_(budget) {
    const size_t n = (size_t) ch_ * (size_t) rb_;
    wear_ = (uint16_t *) calloc(n, sizeof(uint16_t));
    changed_ = (uint8_t *) calloc(n, 1);
  }
  ~EInkGhost() { free(wear_); free(changed_); }

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

  // The most worn cell in row y (v1's per-row view).
  uint16_t wear(int y) const {
    if (y < 0 || y >= h_) return 0;
    uint16_t m = 0;
    const uint16_t *r = wear_ + (size_t) (y / CELL_ROWS) * rb_;
    for (int b = 0; b < rb_; b++) if (r[b] > m) m = r[b];
    return m;
  }
  uint16_t maxWear() const {
    uint16_t m = 0;
    for (size_t i = 0, n = (size_t) ch_ * rb_; i < n; i++) if (wear_[i] > m) m = wear_[i];
    return m;
  }
  // Partials the rect [y0,y1) x bytes [xb0,xb1) takes before its most worn cell reaches the budget.
  int headroom(int y0, int y1, int xb0, int xb1) const {
    if (y1 <= y0 || xb1 <= xb0) return budget_;
    uint16_t m = 0;
    for (int cy = clampRow(y0) / CELL_ROWS; cy <= clampRow(y1 - 1) / CELL_ROWS && cy < ch_; cy++)
      for (int b = xb0 < 0 ? 0 : xb0; b < xb1 && b < rb_; b++)
        if (wear_[(size_t) cy * rb_ + b] > m) m = wear_[(size_t) cy * rb_ + b];
    return m >= budget_ ? 0 : budget_ - m;
  }

  // What to do to get `frame` onto glass that shows `glass`. `ambient` = nothing but a
  // self-ticking value moved (no new data, no button); it only matters for early clean.
  Plan plan(const uint8_t *glass, const uint8_t *frame, bool forceFull = false, bool ambient = false) {
    memset(changed_, 0, (size_t) ch_ * rb_);
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
      const int cy = y / CELL_ROWS;
      uint8_t *chg = changed_ + (size_t) cy * rb_;
      const uint16_t *w = wear_ + (size_t) cy * rb_;
      for (int x = a; x <= b; x++) {
        if (g[x] == f[x] || chg[x]) continue;
        chg[x] = 1;
        if (w[x] + 1 > budget_) {
          if (cy * CELL_ROWS < oy0) oy0 = cy * CELL_ROWS;
          if (cy * CELL_ROWS + CELL_ROWS > oy1) oy1 = cy * CELL_ROWS + CELL_ROWS;
          if (x < ox0) ox0 = x;
          if (x + 1 > ox1) ox1 = x + 1;
        }
        if (early_ && !ambient && w[x] >= budget_ / 3) worn = true;
      }
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
    const uint16_t floor = (uint16_t) (budget_ * share);
    for (int cy = 0; cy < ch_; cy++)
      for (int b = 0; b < rb_; b++) {
        const uint16_t w = wear_[(size_t) cy * rb_ + b];
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
      memset(wear_, 0, n * sizeof(uint16_t));
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
      uint16_t m = 0;
      for (size_t i = 0; i < n; i++)
        if (changed_[i]) {
          if (wear_[i] < 60000) wear_[i]++;
          if (wear_[i] > m) m = wear_[i];
        }
      last_max_ = m;
    }
  }
  // Partials left in the cells the last PARTIAL changed (the region that just ticked).
  int lastHeadroom() const { return last_max_ >= budget_ ? 0 : budget_ - last_max_; }

private:
  int h_, rb_, ch_;
  uint16_t base_, budget_;
  float temp_ = 0.0f / 0.0f;
  bool early_ = false;
  uint16_t *wear_;
  uint8_t *changed_;          // this plan's changed cells
  uint16_t last_max_ = 0;     // the most worn cell of the last PARTIAL
  int clampRow(int y) const { return y < 0 ? 0 : y >= h_ ? h_ - 1 : y; }
  void applyBudget() {
    const float c = temp_;
    budget_ = c != c ? base_ : c < 10 ? base_ / 2 : c < 18 ? (uint16_t) (base_ * 2 / 3) : base_;
    if (budget_ < 2) budget_ = 2;
  }
};
