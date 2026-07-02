// ============================================================================
//  NanoGFX — 15-color packed-nibble graphics for microcontrollers.
//
//  One include gets the platform-agnostic core:
//    PackCanvas  — Adafruit_GFX palette canvas: 8-bit, 4-bit packed (2 px/byte),
//                  or dual-mode (per-line run-lists that fuse most of an RLE
//                  encode into the drawing itself) — chosen per instance.
//    PackRLE     — the 15-color + escape-nibble RLE codec (encode from flat
//                  nibbles, 8-bit indices, or run-lists; decode to nibbles,
//                  8-bit LUT, or 32-bit LUT).
//    PackFlush   — dirty-band frame flushing: keep an on-glass copy of each
//                  row's encoded stream and push only the rows that changed.
//
//  Panel drivers (include the ones you use; ESP32-only):
//    RM690B0.h   — QSPI AMOLED driver with split-polling writes
//                  (ramWriteStart/ramWriteEnd let the CPU prepare chunk n+1
//                  while chunk n rides the SPI DMA).
//
//  Battle-tested in NanoPFD (an ESP32-S3 glass cockpit): 450x600 AMOLED at
//  46-48 fps with WiFi AP + two BLE/WiFi receivers live on the same chip.
//  MIT license.
// ============================================================================
#pragma once
#include "PackRLE.h"
#include "PackCanvas.h"
#include "PackFlush.h"
