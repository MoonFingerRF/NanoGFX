// ============================================================================
//  RM690B0.h — minimal, self-contained ESP32 QSPI driver for RM690B0-class
//  AMOLED panels (e.g. LilyGO T4-S3 2.41", 450x600 usable).
//
//  Proven working settings that this driver encodes (from the NanoPFD bring-up):
//   - register writes: single-line cmd 0x02 with the register in the 24-bit
//     address field (reg << 8); QIO pixel stream via cmd 0x32 / addr 0x002C00.
//   - manual CS (the panel needs CS held across the whole RAMWR burst; letting
//     the SPI driver toggle it per transaction breaks the stream).
//   - POLLING transfers only: sustained QUEUED QSPI DMA on this panel never
//     signals completion (post_cb never fires; get_trans_result deadlocks), so
//     don't "upgrade" this to async — it was tried exhaustively. For CPU overlap
//     use ramWriteStart()/ramWriteEnd(): they SPLIT a polling transfer (the DMA
//     shifts autonomously between the two calls) while staying on the working
//     polling path.
//   - 8 bpp RGB332 (0x3A = 0x02) is supported by the controller and halves the
//     wire time vs RGB565; pass your MADCTL/offsets for the module's mounting.
//   - GPIO-matrix routed pins cap the effective clock near 40 MHz; requesting
//     80 MHz is clean and yields ~40 effective.
//
//  Dependencies: esp-idf spi_master + Arduino GPIO. No display library needed.
//  Part of NanoGFX (https://github.com/MoonFingerRF/NanoGFX); MIT license.
// ============================================================================
#pragma once
#include <Arduino.h>
#include "driver/spi_master.h"

class RM690B0 {
public:
  struct Pins { int8_t d0, d1, d2, d3, sck, cs, rst, pmicEn; };   // pmicEn < 0 = none

  // Initialize bus + panel. maxTransferBytes must cover your largest single
  // ramWrite chunk. brightness: 0-255 (register 0x51).
  bool begin(spi_host_device_t host, const Pins &p, uint32_t clockHz,
             uint32_t maxTransferBytes, uint8_t madctl,
             uint16_t offX, uint16_t offY, uint8_t brightness) {
    pins = p; xOff = offX; yOff = offY;
    pinMode(p.cs, OUTPUT); digitalWrite(p.cs, HIGH);
    if (p.pmicEn >= 0) { pinMode(p.pmicEn, OUTPUT); digitalWrite(p.pmicEn, HIGH); }
    pinMode(p.rst, OUTPUT);
    digitalWrite(p.rst, HIGH); delay(20);
    digitalWrite(p.rst, LOW);  delay(20);
    digitalWrite(p.rst, HIGH); delay(120);

    spi_bus_config_t bus; memset(&bus, 0, sizeof(bus));
    bus.data0_io_num = p.d0; bus.data1_io_num = p.d1;
    bus.data2_io_num = p.d2; bus.data3_io_num = p.d3;
    bus.data4_io_num = bus.data5_io_num = bus.data6_io_num = bus.data7_io_num = -1;
    bus.sclk_io_num = p.sck;
    bus.max_transfer_sz = (int)maxTransferBytes;
    bus.flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS;
    if (spi_bus_initialize(host, &bus, SPI_DMA_CH_AUTO) != ESP_OK) return false;

    spi_device_interface_config_t dev; memset(&dev, 0, sizeof(dev));
    dev.command_bits = 8; dev.address_bits = 24; dev.mode = 0;
    dev.clock_speed_hz = (int)clockHz; dev.spics_io_num = -1;   // manual CS (see header)
    dev.flags = SPI_DEVICE_HALFDUPLEX; dev.queue_size = 1;
    if (spi_bus_add_device(host, &dev, &spi) != ESP_OK) return false;

    // Panel init: vendor page 0x20 tweaks, RGB332 pixel format, TE on, sleep-out,
    // display-on, then brightness. len bit 0x80 = 120 ms delay, 0x20 = 10 ms.
    static const struct { uint8_t reg, param, len; } seq[] = {
      {0xFE,0x20,0x01},{0x26,0x0A,0x01},{0x24,0x80,0x01},{0x5A,0x51,0x01},{0x5B,0x2E,0x01},
      {0xFE,0x00,0x01},{0x3A,0x02,0x01},{0xC2,0x00,0x21},{0x35,0x00,0x01},{0x51,0x00,0x01},
      {0x11,0x00,0x80},{0x29,0x00,0x20},{0x51,0xFF,0x01},
    };
    for (uint32_t i = 0; i < sizeof(seq) / sizeof(seq[0]); i++) {
      writeReg(seq[i].reg, &seq[i].param, seq[i].len & 0x1F);
      if (seq[i].len & 0x80) delay(120);
      if (seq[i].len & 0x20) delay(10);
    }
    writeReg(0x36, &madctl, 1);
    setBrightness(brightness);
    return true;
  }

  // Single-line register write (cmd 0x02, reg in the address field).
  void writeReg(uint8_t reg, const uint8_t *params, uint32_t len) {
    spi_transaction_t t; memset(&t, 0, sizeof(t));
    t.flags = SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR;
    t.cmd  = 0x02;
    t.addr = (uint32_t)reg << 8;
    if (len) { t.tx_buffer = params; t.length = 8 * len; }
    cs(true); spi_device_polling_transmit(spi, &t); cs(false);
  }

  void setBrightness(uint8_t b) { writeReg(0x51, &b, 1); }

  // Standard MIPI-DCS panel controls.
  void displayOn()  { writeReg(0x29, NULL, 0); }
  void displayOff() { writeReg(0x28, NULL, 0); }
  void invert(bool on) { writeReg(on ? 0x21 : 0x20, NULL, 0); }
  void sleep() { writeReg(0x28, NULL, 0); writeReg(0x10, NULL, 0); delay(5); }    // display off + sleep-in
  void wake()  { writeReg(0x11, NULL, 0); delay(120); writeReg(0x29, NULL, 0); }  // sleep-out + display on

  // CASET/RASET/RAMWR with the module's panel offsets applied.
  void setAddrWindow(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye) {
    xs += xOff; xe += xOff; ys += yOff; ye += yOff;
    uint8_t c[4] = { (uint8_t)(xs >> 8), (uint8_t)xs, (uint8_t)(xe >> 8), (uint8_t)xe };
    uint8_t r[4] = { (uint8_t)(ys >> 8), (uint8_t)ys, (uint8_t)(ye >> 8), (uint8_t)ye };
    writeReg(0x2A, c, 4);
    writeReg(0x2B, r, 4);
    writeReg(0x2C, NULL, 0);
  }

  // Pixel-RAM streaming into the current window: hold CS across the whole burst,
  // first chunk carries the QIO RAMWR command, continuation chunks are headerless.
  void ramBegin() { cs(true); }
  void ramWrite(const uint8_t *buf, uint32_t bytes, bool first) {
    ramWriteStart(buf, bytes, first);
    ramWriteEnd();
  }
  // SPLIT-POLLING variant: start the transfer, return immediately (the SPI DMA
  // shifts it out autonomously), do CPU work, then ramWriteEnd() spins only for
  // whatever remains. This is how the caller overlaps preparing chunk n+1 under
  // chunk n's wire time — WITHOUT the queued/ISR path, which this panel breaks
  // (see the header). Rules: exactly one transfer in flight; `buf` and this
  // object must stay untouched until ramWriteEnd(); same-device polling only.
  void ramWriteStart(const uint8_t *buf, uint32_t bytes, bool first) {
    memset(&inflight, 0, sizeof(inflight));
    if (first) {
      inflight.base.flags = SPI_TRANS_MODE_QIO;
      inflight.base.cmd = 0x32; inflight.base.addr = 0x002C00;
    } else {
      inflight.base.flags = SPI_TRANS_MODE_QIO | SPI_TRANS_VARIABLE_CMD |
                            SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;
      inflight.command_bits = 0; inflight.address_bits = 0; inflight.dummy_bits = 0;
    }
    inflight.base.tx_buffer = buf; inflight.base.length = bytes * 8;
    spi_device_polling_start(spi, (spi_transaction_t *)&inflight, portMAX_DELAY);
  }
  void ramWriteEnd() { spi_device_polling_end(spi, portMAX_DELAY); }
  void ramEnd() { cs(false); }

  spi_device_handle_t handle() { return spi; }

private:
  spi_device_handle_t spi = nullptr;
  spi_transaction_ext_t inflight {};   // must outlive polling_start..polling_end
  Pins pins {};
  uint16_t xOff = 0, yOff = 0;
  inline void cs(bool low) { digitalWrite(pins.cs, low ? LOW : HIGH); }
};
