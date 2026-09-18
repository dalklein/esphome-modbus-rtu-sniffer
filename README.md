# esphome-modbus-rtu-sniffer

A **passive** Modbus RTU sniffer for ESPHome. It listens to a bus that two other devices are
talking on, decodes the traffic, and publishes registers as ESPHome sensors. It never transmits.

Works with **stock ESPHome** — no core patch, no forked component, no extra task. It runs in the
ordinary `loop()` like any other component.

Typical use: an inverter polls a battery or meter over RS485, the protocol is undocumented or the
vendor's app is the only reader, and you want the values in Home Assistant without interfering.

## The one setting that matters

> **`rx_full_threshold` must be larger than your longest *burst*** — everything that arrives
> between two idle gaps, which is usually a request **and its reply together**, not one frame.

Modbus RTU has no length prefix and no terminator — message boundaries are silence on the line. On
the ESP32 that silence is detected in hardware (`UART_INTR_RXFIFO_TOUT`, armed by `rx_timeout`).
But there is a second interrupt, `RXFIFO_FULL`, which fires once `rx_full_threshold` bytes have
arrived — and if that trips mid-frame, the frame is delivered in pieces.

This matters more than it first appears. If `rx_timeout` is longer than the request-to-response
turnaround — 10 symbols (~10.4 ms) against 5.9 ms on the bus measured here — then no idle gap
separates a request from its reply, and both land in the FIFO as one burst. Size against
`request + longest response`, not against the longest single frame. On the bus here that is
8 + 51 = 59 bytes, so 120 leaves margin.

ESPHome's computed default is about ten milliseconds of bytes: **8 bytes at 9600 baud**. A Modbus
read request is exactly 8 bytes. Measured on a live bus at that default, **1792 of 1793 reads
returned two or more frames merged together**. Raising the threshold above the longest burst:

```
rx_full_threshold=8    ->  ev1=1     evN=1792     (every read merged)
rx_full_threshold=120  ->  ev1=2989  evN=1        (one frame per read)
```

With the threshold raised, `RXFIFO_FULL` never fires, so bytes reach the ring buffer only at the
idle timeout — in whole-frame units. That holds **regardless of how busy the loop is**: measured on
a board also running a Modbus master and a second protocol on another UART, with 25% of seconds
containing a loop iteration of 144–161 ms against a 28.3 ms inter-frame gap, across ~14,000 frames
the parser never once saw a partial frame.

*(Credit for this: an ESPHome maintainer pointed it out in the project Discord. It is the reason
this component needs no core changes.)*

ESPHome caps `rx_full_threshold` at 120 and the ESP32 FIFO is 128 bytes, while Modbus RTU permits
frames up to 256. If your bus produces bursts longer than 120 bytes — a long reply, or a request
plus a long reply with no idle gap between them — this approach cannot cover them. Shortening
`rx_timeout` below the turnaround splits request from reply and halves the burst, at the cost of
less margin against inter-byte gaps.

## Configuration

```yaml
external_components:
  - source: github://dalklein/esphome-modbus-rtu-sniffer
    components: [modbus_rtu_sniffer]

uart:
  - id: bus
    rx_pin: GPIO17          # RX ONLY -- no tx_pin, so this cannot transmit
    baud_rate: 9600
    rx_buffer_size: 1024
    rx_timeout: 10          # idle symbols marking a message end
    rx_full_threshold: 120  # MUST exceed your longest frame -- see above

modbus_rtu_sniffer:
  id: sniff
  uart_id: bus

sensor:
  - platform: modbus_rtu_sniffer
    modbus_rtu_sniffer_id: sniff
    address: 0x0F           # the slave whose replies you want
    register: 206
    value_type: U_WORD      # or S_WORD
    name: "battery soc"
```

### Choosing `rx_timeout`

Longer than the slowest inter-byte gap the devices actually emit, shorter than the inter-frame
idle. Too short splits messages; too long merges more of them into one read, which is harmless —
they are split again by length and CRC. A climbing `resync` count means it is too short.

## How it works

Bytes are read in `loop()` and messages are split out by declared length and CRC. Because the reads
end on hardware idle boundaries, no message straddles the edge, and an incomplete tail is carried to
the next iteration rather than discarded.

There is no hidden cleverness: no extra task, no busy-wait, no timing heuristic. The hardware
does the delimiting and the parser does the rest.

For completeness — an earlier version of this component took the boundary from an explicit
[`read_frame()` API proposed for ESPHome](https://github.com/esphome/esphome/compare/dev...dalklein:esphome:uart-read-frame).
Measured side by side on the same bus, the two were indistinguishable once `rx_full_threshold` was
set correctly, so that path was dropped and the component now needs no core changes at all.

## Diagnostics

Logged every 10 s at INFO:

| field | meaning |
|---|---|
| `crc_bad` | frames that failed CRC — should be ~0 on a healthy bus |
| `resync` | parser could not find a frame start; a rising count means `rx_timeout` is wrong |
| `pollcarry` | reads that ended mid-frame, carried to the next loop. Should stay 0 with the threshold set correctly |
| `pollflush` | buffer dropped as unparseable — **the failure signal** |
| `ev1` / `evN` | reads containing exactly one message vs more than one |

## Alternatives

[modbus-spy-esphome](https://github.com/pdjong/modbus-spy-esphome) is the established option and
decodes considerably more of the protocol.

| | modbus_spy | this |
|---|---|---|
| framing | inter-byte **timing** | declared **length + CRC** |
| runs in | dedicated FreeRTOS task, pinned to a core, busy-waiting at 50-100 us | ESPHome `loop()` |
| function codes | 01, 02, 03, 04, 05, 06, 0F, 10 | **03, 04 only** |
| outputs | sensor + binary_sensor | sensor |

Different trade-off rather than a straight improvement: framing by content removes the need for the
task and the busy-wait, but if you need writes or coils decoded, use `modbus_spy`.

Related: [feature-requests#2835 "Add sniff option to modbus component"](https://github.com/esphome/feature-requests/issues/2835).
The tidier long-term answer is `modbus` itself gaining a passive role, rather than a separate
component.

## Limitations

* **ESP32 only.** The framing guarantee depends on `rx_full_threshold`, which is ESP32-only in
  ESPHome. The code itself is platform-generic but is untested elsewhere.
* Decodes function codes **3 and 4**. Writes and coils are not decoded.
* Responses carry values but no register numbers, so each response is paired with the preceding
  request. A missed request means that response is skipped, never misattributed.
* Frames longer than 120 bytes are not covered — see above.
* No write support, by design. This never transmits.

## Credits

Built by [@dalklein](https://github.com/dalklein) with [Claude Code](https://claude.com/claude-code).

Framing behaviour was verified against an independent Raspberry Pi with an FTDI adapter on the same
bus: decoded register values matched the sniffer's published values exactly, across 2.46 million
captured frames.

## Licence

[MIT](LICENSE).
