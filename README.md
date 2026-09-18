# esphome-modbus-rtu-sniffer

A **passive** Modbus RTU sniffer for ESPHome. It listens to a bus that two other devices are
talking on, decodes the traffic, and publishes registers as ESPHome sensors. It never transmits.

Typical use: an inverter polls a battery or meter over RS485, the protocol is undocumented or the
vendor's app is the only reader, and you want the values in Home Assistant without interfering.

## ⚠️ Requires an ESPHome `uart` change that is not merged yet

This component is built on `uart::UARTComponent::read_frame()`, an additive `uart` API that is
**proposed but not yet submitted** to ESPHome. It will not compile against stock ESPHome. Until it
lands you need the patched `uart` alongside this component — see [INSTALL.md](INSTALL.md).

> The PR is not open yet. In the meantime the change itself can be reviewed here:
> [**esphome/dev … dalklein:uart-read-frame**](https://github.com/esphome/esphome/compare/dev...dalklein:esphome:uart-read-frame)
> — 5 files, +185/−3, all of it additive and off by default.

Background on why a sniffer needs this at all:
[feature-requests#2835 "Add sniff option to modbus component"](https://github.com/esphome/feature-requests/issues/2835),
open since 2024.

## Why not just read bytes and split them?

Modbus RTU has no length prefix and no terminator — message boundaries are **silence on the line**.
`available()` / `read_array()` hand you bytes with that boundary already discarded, so a sniffer has
to re-derive it from `loop()` timing. That works only while the loop is reliably faster than the
inter-message gap. On a device doing anything else it is not: measured on the author's board (a
sniffer plus a Modbus master plus another protocol on a second UART), 25% of one-second windows
contained a loop iteration longer than the bus's 28.3 ms minimum inter-frame idle.

`read_frame()` returns the bytes between two line-idle gaps, so the chunk boundary is guaranteed to
fall **between** messages. This component then splits the chunk by declared length and CRC — which
is safe precisely because no message can straddle the edge.

Note a chunk may contain **more than one** message: a request and its response usually arrive
together, since the turnaround is shorter than the idle threshold. That is expected and handled.

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
    event_queue_size: 20    # REQUIRED -- enables read_frame()
    rx_full_threshold: 120  # optional; above the longest message = one event per message

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

Must be **longer than the slowest inter-byte gap** the devices actually emit, and **shorter than
the inter-frame idle**. Too short splits messages; too long merges more of them into one chunk,
which is harmless. If you see `resync` climbing, it is too short.

## Diagnostics

Logged every 10 s at INFO:

```
stats: frames=2697245 crc_ok=2697245 crc_bad=0 | req=... rsp=... orphan=0 mismatch=0 resync=0 | ev1=... evN=...
uart events: data=1920300 timeout=1920300 | overrun=0 desync=0
```

| field | meaning |
|---|---|
| `crc_bad` | frames that failed CRC — should be ~0 on a healthy bus |
| `resync` | parser could not find a frame start; a few is fine, a rising count means `rx_timeout` is wrong |
| `ev1` / `evN` | reads containing exactly one message vs more than one |
| `data` vs `timeout` | **must be equal** — every read ended on a hardware idle boundary |
| `desync` / `overrun` | event stream and ring buffer disagreed, or RX overflowed |

`data == timeout` is the health check worth watching. A growing gap means reads are being triggered
by the FIFO threshold rather than line idle, so framing is no longer guaranteed.

## Limitations

* **ESP32 / ESP-IDF only** — `read_frame()` has no implementation on other platforms.
* Decodes function codes **3 and 4** (read holding / input registers). Writes are not decoded.
* Responses carry values but no register numbers, so the component pairs each response with the
  preceding request. A missed request means that response is skipped rather than misattributed.
* No write support, by design. This never transmits.

## Alternatives

[modbus-spy-esphome](https://github.com/pdjong/modbus-spy-esphome) works against stock ESPHome
today. It gets frame boundaries by running a dedicated FreeRTOS task pinned to a core, busy-waiting
at 50-100 us. If you cannot apply the `uart` patch, use that.

## Credits

Built by [@dalklein](https://github.com/dalklein) with [Claude Code](https://claude.com/claude-code).

The framing behaviour was verified against an independent Raspberry Pi with an FTDI adapter on the
same bus, so the counters above are cross-checked against something other than the firmware
reporting on itself.

## Licence

[MIT](LICENSE).
