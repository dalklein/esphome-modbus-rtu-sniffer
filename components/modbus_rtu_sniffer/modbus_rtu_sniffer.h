#pragma once
#include "esphome/core/component.h"
#ifdef USE_ESP32
#include <map>
#include <vector>
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace modbus_rtu_sniffer {

// Receive-only Modbus RTU sniffer that takes frame boundaries from HARDWARE.
//
// Why this exists: Modbus RTU marks message boundaries with silence on the line, and
// read_array() hands back bytes from a ring buffer that has ALREADY discarded that
// boundary. A component is then forced to re-derive framing from millis() in the
// cooperative loop, which holds only while the loop stays faster than the inter-message
// gap -- and fails silently, with no CRC error, when it does not.
//
// This component does NOT own the UART. It is an ordinary uart::UARTDevice and gets
// idle-delimited chunks from UARTComponent::read_frame(). The boundary comes from silicon
// (UART_INTR_RXFIFO_TOUT, armed by the uart component's own `rx_timeout`), so a slow or busy
// loop only makes this component LATE, never wrong.
//
// A chunk may hold MORE than one Modbus message -- a request and its response usually arrive
// together, since the turnaround is shorter than the idle threshold. split_and_handle_() walks
// the chunk by declared length and CRC, which is safe precisely because the chunk's edges are
// guaranteed to fall between messages.
class ModbusRtuSniffer : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::BUS; }


 public:
  // Register a sensor bound to one (slave address, register) pair.
  void add_sensor(uint8_t address, uint16_t reg, bool is_signed, sensor::Sensor *s) {
    this->sensors_.push_back(SensorEntry{address, reg, is_signed, s});
  }

 protected:
  struct SensorEntry {
    uint8_t address;
    uint16_t reg;
    bool is_signed;   // S_WORD -> reinterpret the 16 bits as int16_t
    sensor::Sensor *sensor;
  };
  // A request seen on the wire, awaiting its response. Keyed by slave address, because
  // request/response do NOT strictly alternate on this bus: 0x0E is polled constantly and
  // never answers, so a global "last request" would be wrong most of the time.
  struct Pending {
    uint16_t start;
    uint16_t count;
    bool valid;
  };
  void split_and_handle_(const uint8_t *b, size_t n);
  void handle_frame_(const uint8_t *f, size_t len);
  void publish_(uint8_t address, uint16_t reg, uint16_t raw);

  std::vector<SensorEntry> sensors_;
  std::map<uint8_t, Pending> pending_;
  uint32_t req_{0}, rsp_{0}, orphan_{0}, mismatched_{0}, resync_{0};
  // Frames extracted per UART event. With gap_symbols below the slave's turnaround, a request
  // and its response arrive as ONE event (merged). Above it, they arrive separately. This is a
  // pure FRAMING observation - no absolute timestamps - which is the one thing this sniffer is
  // reliable at, unlike its turnaround metric.
  uint32_t ev_single_{0}, ev_merged_{0};
  // Turnaround measurement: microseconds between the END of a request and the START of its
  // response, per slave address. Tests whether a fake meter/battery answers FASTER than the
  // Modbus RTU 3.5-char minimum, which would not give the master's RS485 transceiver time to
  // switch from transmit to receive -- a plausible cause of the inverter's sporadic
  // "check CT clamps" complaints.
  struct Turn { uint32_t n, under_min, sum_us, min_us, max_us; };
  std::map<uint8_t, Turn> turn_;
  uint8_t last_req_addr_{0};
  uint32_t last_req_end_us_{0};
  bool req_open_{false};

  std::vector<uint8_t> frame_buf_;
  bool ok_{false};

  // stats
  uint32_t frames_{0}, crc_ok_{0}, crc_bad_{0}, overruns_{0}, brk_{0};
  uint32_t last_report_{0};
};

}  // namespace modbus_rtu_sniffer
}  // namespace esphome
#endif
