// ============================================================================
//  EInkGhost.h — which kind of e-paper refresh a change deserves, row by row.
//
//  E-paper has three refreshes and they trade speed against cleanliness:
//    PARTIAL  fast waveform, only changed pixels move, no flash (~1 s) — but every one
//             leaves a little ghost of what was there, and ghosts add up.
//    CLEAN    the full waveform inside a row window: that band flashes, the rest of the
//             screen does not, and the band's ghosts are gone.
//    FULL     the full waveform everywhere (~4 s, whole-screen flash).
//  A single "full every N updates" counter gets this wrong both ways: a clock that ticks
//  in one row flashes the whole screen for it, while a busy band ghosts long before the
//  global count is reached. EInkGhost counts PER ROW: each partial adds one to the rows
//  that actually changed; a band whose rows run over the budget gets a CLEAN window
//  instead of another partial; a quiet screen gets its worn bands cleaned while nobody is
//  looking (idleClean). The budget shrinks in the cold, where partial waveforms ghost more.
//
//  It knows nothing about the controller: frames in (1-bit rows), a Plan out. Pair it
//  with UC8179::startPartial(..., clean) / startFull, and call done() when the glass is.
// ============================================================================
#pragma once
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

class EInkGhost {
public:
  enum Kind : uint8_t { NONE, PARTIAL, CLEAN, FULL };
  struct Plan { Kind kind; int y0, y1; };

  EInkGhost(int height, int rowBytes, uint8_t budget = 16)
      : h_(height), rb_(rowBytes), base_(budget), budget_(budget) {
    wear_ = (uint8_t *)calloc((size_t)height, 1);
    changed_ = (uint8_t *)calloc((size_t)height, 1);
  }
  ~EInkGhost() { free(wear_); free(changed_); }

  // Partial refreshes a row may take before it is cleaned. Room temperature in C: below
  // 18 C the budget is two thirds, below 10 C half (fast waveforms are tuned for ~25 C).
  void setTemperature(float c) {
    if (c != c) { budget_ = base_; return; }   // NaN: unknown
    budget_ = c < 10 ? base_ / 2 : c < 18 ? (uint8_t)(base_ * 2 / 3) : base_;
    if (budget_ < 2) budget_ = 2;
  }
  uint8_t budget() const { return budget_; }
  uint8_t wear(int y) const { return (y >= 0 && y < h_) ? wear_[y] : 0; }

  // What to do to get `frame` onto glass that shows `glass`.
  Plan plan(const uint8_t *glass, const uint8_t *frame, bool forceFull = false) {
    int first = -1, last = -1;
    bool worn = false;
    for (int y = 0; y < h_; y++) {
      changed_[y] = memcmp(glass + (size_t)y * rb_, frame + (size_t)y * rb_, (size_t)rb_) != 0;
      if (!changed_[y]) continue;
      if (first < 0) first = y;
      last = y;
      if (wear_[y] + 1 > budget_) worn = true;
    }
    if (forceFull) return {FULL, 0, h_};
    if (first < 0) return {NONE, 0, 0};
    return {worn ? CLEAN : PARTIAL, first, last + 1};
  }

  // The worn band a quiet screen should clean now (rows at >= half the budget), if any.
  bool idleClean(Plan *out) const {
    int first = -1, last = -1;
    for (int y = 0; y < h_; y++)
      if (wear_[y] >= (budget_ + 1) / 2) { if (first < 0) first = y; last = y; }
    if (first < 0) return false;
    *out = {CLEAN, first, last + 1};
    return true;
  }

  // The glass finished `p` (after plan(): the changed-row marks are that plan's).
  void done(const Plan &p) {
    if (p.kind == FULL) { memset(wear_, 0, (size_t)h_); return; }
    for (int y = p.y0; y < p.y1 && y < h_; y++) {
      if (p.kind == CLEAN) wear_[y] = 0;
      else if (p.kind == PARTIAL && changed_[y] && wear_[y] < 255) wear_[y]++;
    }
  }

private:
  int h_, rb_;
  uint8_t base_, budget_;
  uint8_t *wear_, *changed_;
};
