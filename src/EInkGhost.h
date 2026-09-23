// ============================================================================
//  EInkGhost.h — which kind of e-paper refresh a change deserves, region by region.
//
//  E-paper has three refreshes and they trade speed against cleanliness:
//    PARTIAL  fast waveform, only changed pixels move, no flash — but every one leaves a
//             little ghost of what was there, and ghosts add up.
//    CLEAN    the full waveform inside a window: that region flashes, nothing else does,
//             and the region's ghosts are gone.
//    FULL     the full waveform everywhere (whole-screen flash).
//  A single "full every N updates" counter gets this wrong both ways: a clock that ticks
//  in one corner flashes the whole screen for it, while a busy region ghosts long before
//  the global count is reached. EInkGhost keeps WEAR per row, with the byte-column extent
//  of what changed there, so it can answer with a window that covers exactly the change:
//
//  * every partial adds wear to the rows that actually changed — AMBIENT updates (a track
//    position ticking once a second) add 1, EVENT updates (new values, a button) add
//    EVENT_WEAR, because a ticking bar moves a few pixels while an event redraws text;
//  * a change whose rows would pass the budget is done as a CLEAN window instead;
//  * an EVENT change landing on rows already a third worn is cleaned while it is being
//    redrawn anyway (a track change, a pause) — the flash is then where the eye expects
//    something to change;
//  * a quiet screen has its worn regions cleaned while nobody is looking (idleClean);
//  * the budget shrinks in the cold, where fast waveforms ghost more.
//
//  It knows nothing about the controller: 1-bit frames in, a Plan out. Pair it with
//  UC8179::startPartial(..., clean, xb0, xb1) / startFull and call done() when the
//  glass is.
// ============================================================================
#pragma once
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

class EInkGhost {
public:
  enum Kind : uint8_t { NONE, PARTIAL, CLEAN, FULL };
  struct Plan { Kind kind; int y0, y1, xb0, xb1; };
  static constexpr uint16_t EVENT_WEAR = 8;

  // `budget` is in wear units: EVENT_WEAR per event update, 1 per ambient tick.
  EInkGhost(int height, int rowBytes, uint16_t budget = 160)
      : h_(height), rb_(rowBytes), base_(budget), budget_(budget) {
    wear_ = (uint16_t *)calloc((size_t)height, sizeof(uint16_t));
    cx0_ = (uint8_t *)malloc((size_t)height);
    cx1_ = (uint8_t *)calloc((size_t)height, 1);
    wx0_ = (uint8_t *)malloc((size_t)height);
    wx1_ = (uint8_t *)calloc((size_t)height, 1);
    memset(cx0_, 0xFF, (size_t)height);
    memset(wx0_, 0xFF, (size_t)height);
  }
  ~EInkGhost() { free(wear_); free(cx0_); free(cx1_); free(wx0_); free(wx1_); }

  // Room temperature in C: below 18 C the budget is two thirds, below 10 C half.
  void setTemperature(float c) {
    if (c != c) { budget_ = base_; return; }   // NaN: unknown
    budget_ = c < 10 ? base_ / 2 : c < 18 ? (uint16_t)(base_ * 2 / 3) : base_;
    if (budget_ < 2 * EVENT_WEAR) budget_ = 2 * EVENT_WEAR;
  }
  uint16_t budget() const { return budget_; }
  uint16_t wear(int y) const { return (y >= 0 && y < h_) ? wear_[y] : 0; }

  // What to do to get `frame` onto glass that shows `glass`. `ambient` = nothing but a
  // self-ticking value moved (no new data, no button).
  Plan plan(const uint8_t *glass, const uint8_t *frame, bool forceFull = false, bool ambient = false) {
    ambient_ = ambient;
    int first = -1, last = -1, xa = rb_, xb = 0;
    bool over = false, worn = false;
    const uint16_t w = ambient ? 1 : EVENT_WEAR;
    for (int y = 0; y < h_; y++) {
      const uint8_t *g = glass + (size_t)y * rb_, *f = frame + (size_t)y * rb_;
      int a = 0, b = rb_ - 1;
      while (a < rb_ && g[a] == f[a]) a++;
      if (a == rb_) { cx0_[y] = 0xFF; cx1_[y] = 0; continue; }
      while (b > a && g[b] == f[b]) b--;
      cx0_[y] = (uint8_t)a;
      cx1_[y] = (uint8_t)(b + 1);
      if (first < 0) first = y;
      last = y;
      if (a < xa) xa = a;
      if (b + 1 > xb) xb = b + 1;
      if (wear_[y] + w > budget_) over = true;
      if (!ambient && wear_[y] >= budget_ / 3) worn = true;
    }
    if (forceFull) return {FULL, 0, h_, 0, rb_};
    if (first < 0) return {NONE, 0, 0, 0, 0};
    Plan p{PARTIAL, first, last + 1, xa, xb};
    if (over || worn) {
      p.kind = CLEAN;                          // and cover the worn extent in those rows too
      for (int y = p.y0; y < p.y1; y++)
        if (wear_[y] && wx0_[y] != 0xFF) {
          if (wx0_[y] < p.xb0) p.xb0 = wx0_[y];
          if (wx1_[y] > p.xb1) p.xb1 = wx1_[y];
        }
    }
    return p;
  }

  // The worn region a quiet screen should clean now (rows at >= a quarter of the budget).
  bool idleClean(Plan *out) const {
    int first = -1, last = -1, xa = rb_, xb = 0;
    for (int y = 0; y < h_; y++) {
      if (wear_[y] == 0 || wear_[y] < budget_ / 4) continue;
      if (first < 0) first = y;
      last = y;
      if (wx0_[y] < xa) xa = wx0_[y];
      if (wx1_[y] > xb) xb = wx1_[y];
    }
    if (first < 0 || xa >= xb) return false;
    *out = {CLEAN, first, last + 1, xa, xb};
    return true;
  }

  // The glass finished `p` (after plan(): the changed-row marks are that plan's).
  void done(const Plan &p) {
    if (p.kind == FULL) {
      memset(wear_, 0, (size_t)h_ * sizeof(uint16_t));
      memset(wx0_, 0xFF, (size_t)h_);
      memset(wx1_, 0, (size_t)h_);
      return;
    }
    const uint16_t w = ambient_ ? 1 : EVENT_WEAR;
    for (int y = p.y0; y < p.y1 && y < h_; y++) {
      if (p.kind == CLEAN) {
        // A clean forgives only the wear it covered: a row worn outside the window keeps it.
        if (wx0_[y] >= p.xb0 && wx1_[y] <= p.xb1) { wear_[y] = 0; wx0_[y] = 0xFF; wx1_[y] = 0; }
      } else if (p.kind == PARTIAL && cx1_[y]) {
        wear_[y] = (uint16_t)(wear_[y] + w > 60000 ? 60000 : wear_[y] + w);
        if (cx0_[y] < wx0_[y]) wx0_[y] = cx0_[y];
        if (cx1_[y] > wx1_[y]) wx1_[y] = cx1_[y];
      }
    }
  }

private:
  int h_, rb_;
  uint16_t base_, budget_;
  bool ambient_ = false;
  uint16_t *wear_;
  uint8_t *cx0_, *cx1_, *wx0_, *wx1_;   // this plan's changed extent; accumulated worn extent
};
