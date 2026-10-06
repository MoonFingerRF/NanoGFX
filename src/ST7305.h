// ============================================================================
//  ST7305.h — ESP-IDF driver for the Sitronix ST7305 reflective mono LCD as fitted
//  to the Waveshare ESP32-S3-RLCD-4.2 (300 x 400, 1 bit per pixel, no backlight),
//  part of NanoGFX. The frame layout and the packers are ST7305Pack.h.
//
//  What the controller can do (ST7305 datasheet V0.2, verified on the board by the
//  Nostalgia RLCD firmware's self-test):
//   - 1 bit per pixel, nothing else ("bi-level 1-bit per pixel source drive"): any
//     grey is dithering, spatial or temporal.
//   - High Power Mode scans the glass at 16 / 32 / 25.5 / 51 Hz (OSCSET D8h selects
//     the 32 or 51 Hz family, FRCTRL B2h bit 4 the full or half rate); Low Power Mode
//     at 0.25 .. 8 Hz. Waveshare's demos run 25.5 Hz (LVGL) and 16 Hz (factory).
//     The glass therefore shows at most 51 distinct frames a second.
//   - Write clock: tSCYC(write) >= 30 ns, i.e. 33 MHz by the book; reads 150 ns.
//     The board routes SCK/MOSI through the GPIO matrix (GPIO11/12 are swapped
//     against SPI2's IO_MUX pins), which caps the S3 at 40 MHz anyway.
//   - Partial windows: CASET in 12-column units, RASET in line pairs (Window).
//   - TE (GPIO6 on this board): a V-blank pulse per scanned frame, 35h 00h.
//   - RAMRD over the single bidirectional SDA line (3-wire half duplex, one dummy
//     bit after the command): a written frame can be read back and compared, which
//     is how the self-test finds the fastest write clock that is still exact.
//
//  Conventions (as ST7789.h / RM690B0.h): manual CS and DC on plain GPIOs; commands
//  are small polling writes; pixel data is ONE queued DMA transaction started by
//  writeStart() and finished by writeWait(), so the caller packs or draws the next
//  frame while the bytes shift out. Pixel buffers passed to writeStart() must be
//  DMA-capable (internal RAM) and stay untouched until writeWait().
//
//  Dependencies: ESP-IDF driver/spi_master, driver/gpio, FreeRTOS. MIT license.
// ============================================================================
#pragma once
#if defined(ESP_PLATFORM)
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_heap_caps.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <string.h>

#include <initializer_list>

#include "ST7305Pack.h"

namespace st7305 {

struct Pins {
  int dc, cs, sck, mosi, rst, te;   // rst/te < 0 = not wired
};

// The analogue and timing settings that differ between Waveshare's own builds (datasheet 8.2).
struct Tuning {
  uint8_t vshp = 0x69, vslp = 0x19, vshn = 0x4B, vsln = 0x19;   // C1h C2h C4h C5h source voltages
  bool osc51 = true;     // D8h OSCSW 000: the 51 Hz family (false: 32 Hz family)
  bool full_rate = true; // B2h HFRA: 51 / 32 Hz (false: 25.5 / 16 Hz)
  uint8_t lpm_rate = 2;  // B2h LFRA, Low Power Mode: 0.25 0.5 1 2 4 8 Hz for 0..5
  static Tuning waveshareLvgl() { Tuning t; t.full_rate = false; return t; }        // 25.5 Hz
  static Tuning waveshareFactory() {                                                // 16 Hz, lower drive
    Tuning t; t.vshp = 0x41; t.vshn = 0x41; t.osc51 = false; t.full_rate = false; t.lpm_rate = 5; return t;
  }
  float hpmHz() const { return osc51 ? (full_rate ? 51.0f : 25.5f) : (full_rate ? 32.0f : 16.0f); }
};

class Panel {
 public:
  bool begin(spi_host_device_t host, const Pins &p, uint32_t write_hz, uint32_t read_hz = 4000000) {
    pins_ = p;
    host_ = host;
    read_hz_ = read_hz;
    gpio_config_t out = {};
    out.mode = GPIO_MODE_OUTPUT;
    out.pin_bit_mask = (1ULL << p.dc) | (1ULL << p.cs) | (p.rst >= 0 ? (1ULL << p.rst) : 0);
    if (gpio_config(&out) != ESP_OK) return false;
    gpio_set_level((gpio_num_t) p.cs, 1);
    gpio_set_level((gpio_num_t) p.dc, 1);
    spi_bus_config_t bus = {};
    bus.mosi_io_num = p.mosi;
    bus.miso_io_num = -1;
    bus.sclk_io_num = p.sck;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = FRAME_BYTES + 64;
    if (spi_bus_initialize(host, &bus, SPI_DMA_CH_AUTO) != ESP_OK) return false;
    if (!addWrite_(write_hz)) return false;
    spi_device_interface_config_t rd = {};
    rd.clock_speed_hz = (int) read_hz;
    rd.mode = 0;
    rd.spics_io_num = -1;
    rd.flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_3WIRE;
    rd.queue_size = 1;
    if (spi_bus_add_device(host, &rd, &rd_) != ESP_OK) return false;
    rxbuf_ = (uint8_t *) heap_caps_malloc(RX_CHUNK + 4, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    return rxbuf_ != nullptr;
  }

  // Change the write clock (the self-test's sweep). Returns the clock the bus really runs.
  uint32_t setWriteClock(uint32_t hz) {
    writeWait();
    if (wr_) spi_bus_remove_device(wr_);
    wr_ = nullptr;
    addWrite_(hz);
    return writeClock();
  }
  uint32_t writeClock() const { return actual_hz_; }

  // Hardware reset + Waveshare's init (their display_bsp.cpp, with the tuning above), display on, HPM.
  void init(const Tuning &t) {
    tuning_ = t;
    if (pins_.rst >= 0) {
      gpio_set_level((gpio_num_t) pins_.rst, 1);
      vTaskDelay(pdMS_TO_TICKS(20));
      gpio_set_level((gpio_num_t) pins_.rst, 0);
      vTaskDelay(pdMS_TO_TICKS(20));
      gpio_set_level((gpio_num_t) pins_.rst, 1);
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    cmd(0xD6, {0x17, 0x02});                         // NVM load control
    cmd(0xD1, {0x01});                               // booster on
    cmd(0xC0, {0x11, 0x04});                         // gate voltage
    cmd(0xC1, {t.vshp, t.vshp, t.vshp, t.vshp});     // source voltages
    cmd(0xC2, {t.vslp, t.vslp, t.vslp, t.vslp});
    cmd(0xC4, {t.vshn, t.vshn, t.vshn, t.vshn});
    cmd(0xC5, {t.vsln, t.vsln, t.vsln, t.vsln});
    cmd(0xD8, {(uint8_t) (t.osc51 ? 0x80 : 0xA6), 0xE9});   // OSCSET
    cmd(0xB2, {frctrl_(t)});                         // FRCTRL
    cmd(0xB3, {0xE5, 0xF6, 0x05, 0x46, 0x77, 0x77, 0x77, 0x77, 0x76, 0x45});   // gate EQ, HPM
    cmd(0xB4, {0x05, 0x46, 0x77, 0x77, 0x77, 0x77, 0x76, 0x45});               // gate EQ, LPM
    cmd(0x62, {0x32, 0x03, 0x1F});                   // gate timing
    cmd(0xB7, {0x13});                               // source EQ
    cmd(0xB0, {0x64});                               // duty: 200 line pairs
    cmd(0x11);                                       // sleep out
    vTaskDelay(pdMS_TO_TICKS(200));
    cmd(0xC9, {0x00});                               // source voltage select
    cmd(0x36, {0x48});                               // MADCTL
    cmd(0x3A, {0x11});                               // data format: 1 bit
    cmd(0xB9, {0x20});                               // gamma mode: mono
    cmd(0xB8, {0x29});                               // panel setting
    cmd(0x21);                                       // inversion on: a 0 bit is black (ST7305Pack writes ink as 0)
    cmd(0x35, {0x00});                               // TE on, V-blank only
    cmd(0xD0, {0xFF});                               // auto power down off
    cmd(0x38);                                       // High Power Mode
    cmd(0x29);                                       // display on
  }
  const Tuning &tuning() const { return tuning_; }
  // Frame rate in HPM, live (FRCTRL alone, no re-init): the 51 / 25.5 Hz switch of the 51 Hz family.
  void setFullRate(bool full) {
    tuning_.full_rate = full;
    cmd(0xB2, {frctrl_(tuning_)});
  }
  void highPower(bool on) { cmd(on ? 0x38 : 0x39); }   // LPM keeps the image at 0.25..8 Hz scans
  void sleep(bool on) {
    cmd(on ? 0x10 : 0x11);
    vTaskDelay(pdMS_TO_TICKS(on ? 5 : 120));
  }
  void displayOn(bool on) { cmd(on ? 0x29 : 0x28); }

  void cmd(uint8_t c, std::initializer_list<uint8_t> data) {
    uint8_t d[16];
    size_t n = 0;
    for (uint8_t v : data) d[n++] = v;
    cmd(c, d, n);
  }
  void cmd(uint8_t c, const uint8_t *data = nullptr, size_t n = 0) {
    writeWait();
    gpio_set_level((gpio_num_t) pins_.cs, 0);
    gpio_set_level((gpio_num_t) pins_.dc, 0);
    poll_(&c, 1);
    if (n) {
      gpio_set_level((gpio_num_t) pins_.dc, 1);
      poll_(data, n);
    }
    gpio_set_level((gpio_num_t) pins_.cs, 1);
  }

  // Select a window and start shifting `n` bytes (the window's, page by page) out by DMA.
  void writeStart(const Window &w, const uint8_t *bytes, size_t n) {
    writeWait();
    setWindow_(w);
    const uint8_t ramwr = 0x2C;
    gpio_set_level((gpio_num_t) pins_.cs, 0);
    gpio_set_level((gpio_num_t) pins_.dc, 0);
    poll_(&ramwr, 1);
    gpio_set_level((gpio_num_t) pins_.dc, 1);
    memset(&trans_, 0, sizeof trans_);
    trans_.length = n * 8;
    trans_.tx_buffer = bytes;
    if (spi_device_queue_trans(wr_, &trans_, portMAX_DELAY) == ESP_OK) {
      pending_ = true;
    } else {
      gpio_set_level((gpio_num_t) pins_.cs, 1);
    }
  }
  bool writeDone() {
    if (!pending_) return true;
    spi_transaction_t *r = nullptr;
    if (spi_device_get_trans_result(wr_, &r, 0) != ESP_OK) return false;
    finish_();
    return true;
  }
  void writeWait() {
    if (!pending_) return;
    spi_transaction_t *r = nullptr;
    spi_device_get_trans_result(wr_, &r, portMAX_DELAY);
    finish_();
  }
  // Blocking full-frame write (bytes in internal RAM).
  void writeFrame(const uint8_t *frame) {
    writeStart(Window::full(), frame, FRAME_BYTES);
    writeWait();
  }

  // Read a window back (RAMRD: command, DC high, a dummy bit, data MSB first) at the read clock. The
  // RLCD-4.2's reply arrives one whole byte after the dummy bit (measured 2026-10-05: wrote 3C 5E 81,
  // read 00 3C 5E), so `readSkip` leading bytes are dropped; the self-test finds the value.
  void setReadSkip(int bytes) { read_skip_ = bytes < 0 ? 0 : bytes > 4 ? 4 : bytes; }
  int readSkip() const { return read_skip_; }
  bool readRam(const Window &w, uint8_t *out, size_t n) {
    writeWait();
    setWindow_(w);
    const uint8_t ramrd = 0x2E;
    gpio_set_level((gpio_num_t) pins_.cs, 0);
    gpio_set_level((gpio_num_t) pins_.dc, 0);
    poll_(&ramrd, 1);
    gpio_set_level((gpio_num_t) pins_.dc, 1);
    bool ok = true, first = true;
    size_t skip = (size_t) read_skip_, done = 0;
    while (ok && done < n) {
      const size_t want = n - done + skip;
      const size_t k = want < RX_CHUNK ? want : RX_CHUNK;
      spi_transaction_ext_t t = {};
      t.base.flags = first ? SPI_TRANS_VARIABLE_DUMMY : 0;
      t.dummy_bits = first ? 1 : 0;
      t.base.rxlength = k * 8;
      t.base.length = 0;
      t.base.rx_buffer = rxbuf_;
      ok = spi_device_polling_transmit(rd_, &t.base) == ESP_OK;
      if (ok) {
        const size_t drop = skip < k ? skip : k;
        memcpy(out + done, rxbuf_ + drop, k - drop);
        done += k - drop;
        skip -= drop;
      }
      first = false;
    }
    gpio_set_level((gpio_num_t) pins_.cs, 1);
    return ok;
  }
  // A register read (RDDID 04h: 3 bytes after a dummy bit; RDID1-3 DAh..DCh: 1 byte, no dummy).
  bool readReg(uint8_t c, uint8_t *out, size_t n, int dummy_bits) {
    writeWait();
    gpio_set_level((gpio_num_t) pins_.cs, 0);
    gpio_set_level((gpio_num_t) pins_.dc, 0);
    poll_(&c, 1);
    spi_transaction_ext_t t = {};
    t.base.flags = dummy_bits ? SPI_TRANS_VARIABLE_DUMMY : 0;
    t.dummy_bits = (uint8_t) dummy_bits;
    t.base.rxlength = n * 8;
    t.base.rx_buffer = rxbuf_;
    const bool ok = spi_device_polling_transmit(rd_, &t.base) == ESP_OK;
    gpio_set_level((gpio_num_t) pins_.cs, 1);
    if (ok) memcpy(out, rxbuf_, n);
    return ok;
  }

  // ---- TE: an edge counter + a semaphore per V-blank (attach once)
  bool teAttach() {
    if (pins_.te < 0) return false;
    if (te_sem_) return true;
    te_sem_ = xSemaphoreCreateBinary();
    gpio_config_t in = {};
    in.mode = GPIO_MODE_INPUT;
    in.pin_bit_mask = 1ULL << pins_.te;
    in.intr_type = GPIO_INTR_POSEDGE;
    if (gpio_config(&in) != ESP_OK) return false;
    const esp_err_t e = gpio_install_isr_service(0);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return false;
    return gpio_isr_handler_add((gpio_num_t) pins_.te, &Panel::teIsr_, this) == ESP_OK;
  }
  // Block until the next V-blank (true) or the timeout (false: no TE, e.g. display off).
  bool waitTe(uint32_t timeout_ms) {
    if (!te_sem_) return false;
    xSemaphoreTake(te_sem_, 0);            // drop a stale edge: wait for the NEXT one
    return xSemaphoreTake(te_sem_, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
  }
  uint32_t teCount() const { return te_count_; }
  int64_t teLastUs() const { return te_last_us_; }

 private:
  static constexpr size_t RX_CHUNK = 64;
  static void IRAM_ATTR teIsr_(void *arg) {
    Panel *self = (Panel *) arg;
    self->te_count_++;
    self->te_last_us_ = esp_timer_get_time();
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(self->te_sem_, &woken);
    if (woken) portYIELD_FROM_ISR();
  }
  static uint8_t frctrl_(const Tuning &t) {
    return (uint8_t) ((t.full_rate ? 0x10 : 0x00) | (t.lpm_rate > 5 ? 5 : t.lpm_rate));
  }
  bool addWrite_(uint32_t hz) {
    spi_device_interface_config_t dev = {};
    dev.clock_speed_hz = (int) hz;
    dev.mode = 0;
    dev.spics_io_num = -1;                  // manual CS
    dev.flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY;
    dev.queue_size = 2;
    if (spi_bus_add_device(host_, &dev, &wr_) != ESP_OK) return false;
    int khz = 0;
    spi_device_get_actual_freq(wr_, &khz);
    actual_hz_ = (uint32_t) khz * 1000u;
    return true;
  }
  void poll_(const uint8_t *d, size_t n) {
    spi_transaction_t t = {};
    if (n <= 4) {
      t.flags = SPI_TRANS_USE_TXDATA;
      memcpy(t.tx_data, d, n);
    } else {
      t.tx_buffer = d;
    }
    t.length = n * 8;
    spi_device_polling_transmit(wr_, &t);
  }
  void setWindow_(const Window &w) {
    cmd(0x2A, {(uint8_t) (XS0 + w.u0), (uint8_t) (XS0 + w.u1)});
    cmd(0x2B, {(uint8_t) (YS0 + w.p0), (uint8_t) (YS0 + w.p1)});
  }
  void finish_() {
    pending_ = false;
    gpio_set_level((gpio_num_t) pins_.cs, 1);
  }

  Pins pins_{};
  spi_host_device_t host_ = SPI2_HOST;
  spi_device_handle_t wr_ = nullptr, rd_ = nullptr;
  uint32_t actual_hz_ = 0, read_hz_ = 0;
  spi_transaction_t trans_{};
  bool pending_ = false;
  uint8_t *rxbuf_ = nullptr;
  int read_skip_ = 1;
  Tuning tuning_{};
  SemaphoreHandle_t te_sem_ = nullptr;
  volatile uint32_t te_count_ = 0;
  volatile int64_t te_last_us_ = 0;
};

}  // namespace st7305
#endif  // ESP_PLATFORM
