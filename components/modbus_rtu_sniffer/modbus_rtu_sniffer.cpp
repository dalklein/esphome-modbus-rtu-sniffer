#include "modbus_rtu_sniffer.h"
#ifdef USE_ESP32
#include "esphome/core/log.h"
#include <driver/gpio.h>

namespace esphome {
namespace modbus_rtu_sniffer {

static const char *const TAG = "modbus_rtu_sniffer";
static const size_t RX_RING = 1024;   // must exceed the 128-byte hw FIFO
static const int QUEUE_DEPTH = 32;
static const size_t MAX_FRAME = 256;

static uint16_t crc16_modbus(const uint8_t *d, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++)
      crc = (crc & 1) ? (uint16_t) ((crc >> 1) ^ 0xA001) : (uint16_t) (crc >> 1);
  }
  return crc;
}

void ModbusRtuSniffer::setup() {
  // No hardware ownership at all. The uart component installs the driver, arms
  // UART_INTR_RXFIFO_TOUT from its own `rx_timeout:` (symbol times), and -- when
  // `event_queue_size:` is set -- keeps the event queue that records where each
  // frame ended. All we do is ask for the result.
  this->ok_ = this->parent_->supports_frame_reads();
  if (!this->ok_) {
    ESP_LOGE(TAG,
             "this uart cannot supply frames. On esp-idf set 'event_queue_size: 20' on the "
             "uart, and 'rx_timeout: 3' or more for a Modbus RTU inter-frame gap.");
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "bound sensors: %u", (unsigned) this->sensors_.size());
  ESP_LOGI(TAG, "listening via uart read_frame(), rx_timeout=%u symbols",
           (unsigned) this->parent_->get_rx_timeout());
}

void ModbusRtuSniffer::loop() {
  if (!this->ok_)
    return;
  // Each successful read_frame() is ONE hardware-delimited chunk. Overrun handling and
  // flushing now live in the uart component, which is the point: this component carries
  // no platform code.
  while (this->parent_->read_frame(this->frame_buf_)) {
    if (!this->frame_buf_.empty())
      this->split_and_handle_(this->frame_buf_.data(), this->frame_buf_.size());
  }
  const uint32_t now = millis();
  if (now - this->last_report_ > 10000) {
    this->last_report_ = now;
    ESP_LOGI(TAG,
             "stats: frames=%" PRIu32 " crc_ok=%" PRIu32 " crc_bad=%" PRIu32 " | req=%" PRIu32
             " rsp=%" PRIu32 " orphan=%" PRIu32 " mismatch=%" PRIu32 " resync=%" PRIu32 " | ev1=%" PRIu32 " evN=%" PRIu32 " | overrun=%" PRIu32
             " line_err=%" PRIu32,
             this->frames_, this->crc_ok_, this->crc_bad_, this->req_, this->rsp_, this->orphan_,
             this->mismatched_, this->resync_, this->ev_single_, this->ev_merged_, this->overruns_, this->brk_);
    // Confirms uart_set_always_rx_timeout() actually armed: with it armed EVERY burst ends on a
    // line-idle boundary, so timeout_events == frames. Materially fewer means boundaries are
    // still being missed. The IDF call returns void, so this is the only way to know.
    const auto fs = this->parent_->get_frame_stats();
    ESP_LOGI(TAG, "uart events: data=%" PRIu32 " timeout=%" PRIu32 " | overrun=%" PRIu32
                  " desync=%" PRIu32,
             fs.data_events, fs.timeout_events, fs.overruns, fs.desyncs);
    const uint32_t min_us = (uint32_t) (35ULL * 1000000ULL / this->parent_->get_baud_rate());
    for (auto &kv : this->turn_) {
      auto &t = kv.second;
      if (t.n < 5) continue;
      ESP_LOGI(TAG, "turnaround addr=%02x n=%" PRIu32 " min=%" PRIu32 "us avg=%" PRIu32
                    "us max=%" PRIu32 "us | under %" PRIu32 "us(3.5char): %" PRIu32 " = %.0f%%",
               kv.first, t.n, t.min_us, t.sum_us / t.n, t.max_us, min_us, t.under_min,
               100.0f * t.under_min / t.n);
    }
  }
}

// A UART_DATA event is NOT guaranteed to be exactly one frame.
//
// The hardware idle-detect marks the boundary, but ev.size reports however many bytes had
// ACCUMULATED in the ring buffer by the time we serviced the queue. A slow loop therefore
// coalesces frames again. Measured 2026-09-09 on the live bus: crc_bad went 4.5% -> 56% purely
// by making the loop slower (adding MQTT logging), and every bad frame was a valid request
// concatenated with its valid response.
//
// So walk the buffer and cut frames by their DECLARED length, confirming each with CRC:
//   fc3/4 request  -> 8 bytes
//   fc3/4 response -> 5 + bytecount
// If neither validates at this offset, advance one byte and resynchronise. That is cheap
// (a CRC over <=8 bytes) and self-correcting, and it makes the parser independent of how
// many frames happen to arrive in one event.
void ModbusRtuSniffer::split_and_handle_(const uint8_t *b, size_t n) {
  size_t i = 0;
  uint32_t found = 0;
  while (i + 4 <= n) {
    const uint8_t fn = b[i + 1];
    bool matched = false;

    if (fn == 0x03 || fn == 0x04) {
      // try REQUEST (fixed 8 bytes)
      if (i + 8 <= n) {
        uint16_t calc = crc16_modbus(b + i, 6);
        uint16_t seen = (uint16_t) b[i + 6] | ((uint16_t) b[i + 7] << 8);
        if (calc == seen) {
          this->frames_++;
          found++;
          this->handle_frame_(b + i, 8);
          i += 8;
          matched = true;
        }
      }
      // try RESPONSE (5 + bytecount)
      if (!matched) {
        const size_t rlen = 5u + b[i + 2];
        if (b[i + 2] > 0 && (b[i + 2] & 1) == 0 && i + rlen <= n) {
          uint16_t calc = crc16_modbus(b + i, rlen - 2);
          uint16_t seen = (uint16_t) b[i + rlen - 2] | ((uint16_t) b[i + rlen - 1] << 8);
          if (calc == seen) {
            this->frames_++;
            found++;
            this->handle_frame_(b + i, rlen);
            i += rlen;
            matched = true;
          }
        }
      }
    }
    if (!matched) {
      // No valid frame starts here. Resynchronise one byte at a time.
      this->resync_++;
      i++;
    }
  }
  if (found == 1) this->ev_single_++;
  else if (found > 1) {
    this->ev_merged_++;
  }
}

// Direction is decided by LENGTH, not by an alternation state.
//
// Because hardware framing gives us whole frames with exact lengths, fc3/fc4 direction is
// deterministic:
//   request  = addr fn start_hi start_lo cnt_hi cnt_lo crc crc   -> ALWAYS 8 bytes
//   response = addr fn bytecount data... crc crc                 -> 5 + 2n bytes, ALWAYS ODD
// A response can never be 8 bytes: that needs bytecount==3, impossible when registers are
// 2 bytes wide. So the two can never be confused.
//
// This is why the per-address `pending_` map matters: on this bus address 0x0E is polled
// constantly and NEVER answers, so request/response do not alternate globally. Keying the
// outstanding request by slave address is what keeps 0x0E's unanswered polls from being
// paired with that address's replies.
void ModbusRtuSniffer::handle_frame_(const uint8_t *f, size_t len) {
  if (len < 4)
    return;
  uint16_t calc = crc16_modbus(f, len - 2);
  uint16_t seen = (uint16_t) f[len - 2] | ((uint16_t) f[len - 1] << 8);
  if (calc != seen) {
    this->crc_bad_++;   // should now be ~0: the splitter only emits CRC-valid frames
    // DIAGNOSTIC: what do bad frames look like?
    //   short + valid-looking header  -> frame was SPLIT (gap too tight)
    //   long / two headers visible    -> frames MERGED (gap too loose)
    //   right length, garbled bytes   -> electrical
    return;
  }
  this->crc_ok_++;

  const uint8_t addr = f[0];
  const uint8_t fn = f[1];
  if (fn != 0x03 && fn != 0x04)
    return;  // only holding/input register reads carry register data

  if (len == 8) {
    // REQUEST
    Pending p;
    p.start = ((uint16_t) f[2] << 8) | f[3];
    p.count = ((uint16_t) f[4] << 8) | f[5];
    p.valid = true;
    this->pending_[addr] = p;
    this->req_++;
    this->last_req_addr_ = addr;
    this->last_req_end_us_ = micros();
    this->req_open_ = true;
    ESP_LOGV(TAG, "req  addr=%02x start=%u count=%u", addr, p.start, p.count);
    return;
  }

  if ((len & 1) == 1 && len >= 7) {
    // RESPONSE
    this->rsp_++;
    if (this->req_open_ && this->last_req_addr_ == addr) {
      // now() is just after the response's LAST byte; subtract its own wire time to get
      // the moment its FIRST byte started. 10 bits per byte on 8N1.
      const uint32_t wire_us = (uint32_t) ((uint64_t) len * 10ULL * 1000000ULL / this->parent_->get_baud_rate());
      const uint32_t now_us = micros();
      const uint32_t start_us = now_us - wire_us;
      if (start_us > this->last_req_end_us_) {
        const uint32_t gap = start_us - this->last_req_end_us_;
        if (gap < 200000) {   // ignore anything absurd
          auto &t = this->turn_[addr];
          if (t.n == 0) { t.min_us = gap; t.max_us = gap; }
          t.n++;
          t.sum_us += gap;
          if (gap < t.min_us) t.min_us = gap;
          if (gap > t.max_us) t.max_us = gap;
          // Modbus RTU 3.5 chars = 35 bit-times
          const uint32_t min_us = (uint32_t) (35ULL * 1000000ULL / this->parent_->get_baud_rate());
          if (gap < min_us) t.under_min++;
        }
      }
      this->req_open_ = false;
    }
    const uint8_t byte_count = f[2];
    if (byte_count + 5u != len) {
      this->mismatched_++;
      ESP_LOGW(TAG, "rsp addr=%02x bytecount=%u but frame=%u - ignoring", addr, byte_count,
               (unsigned) len);
      return;
    }
    auto it = this->pending_.find(addr);
    if (it == this->pending_.end() || !it->second.valid) {
      // Reply with no request seen. Happens after a reset or a lost request; never guess.
      this->orphan_++;
      return;
    }
    const uint16_t count = byte_count / 2;
    if (count != it->second.count) {
      this->mismatched_++;
      ESP_LOGW(TAG, "rsp addr=%02x has %u regs, request asked %u - ignoring", addr, count,
               it->second.count);
      it->second.valid = false;
      return;
    }
    const uint16_t start = it->second.start;
    for (uint16_t i = 0; i < count; i++) {
      uint16_t raw = ((uint16_t) f[3 + 2 * i] << 8) | f[4 + 2 * i];
      this->publish_(addr, start + i, raw);
    }
    // Consume it: one request yields exactly one response.
    it->second.valid = false;
  }
}

void ModbusRtuSniffer::publish_(uint8_t address, uint16_t reg, uint16_t raw) {
  for (auto &e : this->sensors_) {
    if (e.address == address && e.reg == reg && e.sensor != nullptr) {
      // S_WORD: the same 16 bits reinterpreted. reg 203 (battery power) is the one that
      // matters most -- discharge is negative and would otherwise read as ~65000.
      const float v = e.is_signed ? (float) (int16_t) raw : (float) raw;
      ESP_LOGV(TAG, "publish addr=%02x reg=%u raw=%u -> %.0f", address, reg, raw, v);
      e.sensor->publish_state(v);
    }
  }
}

void ModbusRtuSniffer::dump_config() {
  ESP_LOGCONFIG(TAG, "Modbus sniffer (uart read_frame API)");
  ESP_LOGCONFIG(TAG, "  Baud: %" PRIu32 ", rx_timeout: %u symbols (~%.1f ms)",
                this->parent_->get_baud_rate(), (unsigned) this->parent_->get_rx_timeout(),
                this->parent_->get_rx_timeout() * 10000.0f / this->parent_->get_baud_rate());
  ESP_LOGCONFIG(TAG, "  Sensors: %u", (unsigned) this->sensors_.size());
}

}  // namespace modbus_rtu_sniffer
}  // namespace esphome
#endif
