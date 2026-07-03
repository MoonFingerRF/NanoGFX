// ============================================================================
//  ST7789.h — minimal, self-contained ESP32 4-wire-SPI driver for ST7789-class
//  TFTs (240x240 / 240x280 / 240x320 modules), part of NanoGFX.
//
//  Built for the NanoGFX pipeline: setAddrWindow + RGB565 RAM bursts, with the
//  same SPLIT-POLLING overlap as the RM690B0 driver — ramWriteStart() returns
//  while the SPI DMA shifts the chunk out, so the CPU decodes the next chunk
//  meanwhile, and ramWriteEnd() spins only for the remainder. Combined with
//  PackFlush, a panel receives only the rows that changed, decoded straight
//  from PackRLE streams (prle_decode_lut32 with a byte-swapped RGB565 pair LUT).
//
//  Conventions:
//   - Manual CS + manual DC (no per-transaction callbacks): commands are tiny
//     polling writes with DC low; pixel bursts hold CS with DC high.
//   - setRotation(r, offX, offY): MADCTL quarter-turns with EXPLICIT window
//     offsets — panels smaller than the 240x320 GRAM sit at module-specific
//     origins that differ per rotation (e.g. a 240x280: (0,20) at rot 0/2),
//     so the caller passes the known-good pair instead of trusting a formula.
//   - IPS modules need inversion ON (`ips=true` in begin, the common case).
//
//  Dependencies: esp-idf spi_master + Arduino GPIO. MIT license.
// ============================================================================
#pragma once
#include <Arduino.h>
#include "driver/spi_master.h"

class ST7789 {
public:
  struct Pins { int8_t dc, cs, sck, mosi, rst, bl; };   // rst/bl < 0 = none

  // colmod: 0x55 = RGB565 (2 B/px, default), 0x53 = RGB444 (3 B per 2 px — 25%
  // less wire time; ideal for palette content, pair with prle_decode_p3).
  bool begin(spi_host_device_t host, const Pins &p, uint32_t clockHz,
             uint32_t maxTransferBytes, bool ips = true, uint8_t colmod = 0x55) {
    pins = p;
    pinMode(p.dc, OUTPUT); digitalWrite(p.dc, HIGH);
    pinMode(p.cs, OUTPUT); digitalWrite(p.cs, HIGH);
    if (p.bl >= 0) { pinMode(p.bl, OUTPUT); digitalWrite(p.bl, HIGH); }
    if (p.rst >= 0) {
      pinMode(p.rst, OUTPUT);
      digitalWrite(p.rst, HIGH); delay(10);
      digitalWrite(p.rst, LOW);  delay(10);
      digitalWrite(p.rst, HIGH); delay(120);
    }
    spi_bus_config_t bus; memset(&bus, 0, sizeof(bus));
    bus.mosi_io_num = p.mosi; bus.miso_io_num = -1; bus.sclk_io_num = p.sck;
    bus.quadwp_io_num = -1; bus.quadhd_io_num = -1;
    bus.max_transfer_sz = (int)maxTransferBytes;
    if (spi_bus_initialize(host, &bus, SPI_DMA_CH_AUTO) != ESP_OK) return false;
    spi_device_interface_config_t dev; memset(&dev, 0, sizeof(dev));
    dev.clock_speed_hz = (int)clockHz; dev.mode = 0;
    dev.spics_io_num = -1;                                // manual CS
    dev.flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY;
    dev.queue_size = 1;
    if (spi_bus_add_device(host, &dev, &spi) != ESP_OK) return false;

    writeCmd(0x01); delay(150);                           // SWRESET
    writeCmd(0x11); delay(120);                           // SLPOUT
    writeCmd(0x3A, &colmod, 1);                           // pixel format (see above)
    writeCmd(ips ? 0x21 : 0x20);                          // IPS modules: inversion ON
    writeCmd(0x13);                                       // NORON
    writeCmd(0x29); delay(10);                            // DISPON
    setRotation(0, 0, 0);
    return true;
  }

  // Quarter-turn rotation + this rotation's window origin in GRAM. MADCTL:
  // rot0 0x00, rot1 MX|MV, rot2 MX|MY, rot3 MY|MV (RGB order).
  void setRotation(uint8_t r, uint16_t offX, uint16_t offY) {
    static const uint8_t MAD[4] = { 0x00, 0x60, 0xC0, 0xA0 };
    uint8_t m = MAD[r & 3];
    writeCmd(0x36, &m, 1);
    xOff = offX; yOff = offY;
  }

  void writeCmd(uint8_t cmd, const uint8_t *data = nullptr, uint32_t len = 0) {
    cs(true);
    digitalWrite(pins.dc, LOW);
    xfer(&cmd, 1);
    if (len) { digitalWrite(pins.dc, HIGH); xfer(data, len); }
    cs(false);
  }

  void setAddrWindow(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye) {
    xs += xOff; xe += xOff; ys += yOff; ye += yOff;
    uint8_t c[4] = { (uint8_t)(xs >> 8), (uint8_t)xs, (uint8_t)(xe >> 8), (uint8_t)xe };
    uint8_t r[4] = { (uint8_t)(ys >> 8), (uint8_t)ys, (uint8_t)(ye >> 8), (uint8_t)ye };
    writeCmd(0x2A, c, 4);
    writeCmd(0x2B, r, 4);
  }

  // Pixel-RAM streaming: ramBegin issues RAMWR and leaves CS low + DC high;
  // stream RGB565 in wire order (byte-swapped — bake the swap into your pair
  // LUT so the decode emits wire-ready bytes) in one or more chunks; ramEnd
  // releases CS.
  void ramBegin() {
    cs(true);
    digitalWrite(pins.dc, LOW);
    uint8_t cmd = 0x2C;
    xfer(&cmd, 1);
    digitalWrite(pins.dc, HIGH);
  }
  void ramWrite(const uint8_t *buf, uint32_t bytes) { ramWriteStart(buf, bytes); ramWriteEnd(); }
  // Split polling (see RM690B0.h for the pattern): one transfer in flight;
  // don't touch `buf` or this object until ramWriteEnd().
  void ramWriteStart(const uint8_t *buf, uint32_t bytes) {
    memset(&inflight, 0, sizeof(inflight));
    inflight.tx_buffer = buf;
    inflight.length = bytes * 8;
    spi_device_polling_start(spi, &inflight, portMAX_DELAY);
  }
  void ramWriteEnd() { spi_device_polling_end(spi, portMAX_DELAY); }
  void ramEnd() { cs(false); }

  void displayOn()  { writeCmd(0x29); }
  void displayOff() { writeCmd(0x28); }
  void invert(bool on) { writeCmd(on ? 0x21 : 0x20); }
  void sleep() { writeCmd(0x28); writeCmd(0x10); delay(5); }
  void wake()  { writeCmd(0x11); delay(120); writeCmd(0x29); }
  void backlight(bool on) { if (pins.bl >= 0) digitalWrite(pins.bl, on ? HIGH : LOW); }

  spi_device_handle_t handle() { return spi; }

private:
  spi_device_handle_t spi = nullptr;
  spi_transaction_t inflight {};        // must outlive polling_start..polling_end
  Pins pins {};
  uint16_t xOff = 0, yOff = 0;
  inline void cs(bool low) { digitalWrite(pins.cs, low ? LOW : HIGH); }
  inline void xfer(const uint8_t *d, uint32_t n) {
    spi_transaction_t t; memset(&t, 0, sizeof(t));
    t.tx_buffer = d; t.length = n * 8;
    spi_device_polling_transmit(spi, &t);
  }
};
