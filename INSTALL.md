# Installing before the `uart` change is merged

This component needs `read_frame()`, which is not in any ESPHome release yet. Two options.

## Option 1 — patched `uart` as a second local component (recommended)

Take the four patched files from the PR branch and drop them in beside this component:

```bash
git clone --branch uart-read-frame --depth 1 https://github.com/dalklein/esphome.git /tmp/eh
mkdir -p uartpatch
cp -r /tmp/eh/esphome/components/uart uartpatch/
```

```yaml
external_components:
  - source: github://dalklein/esphome-modbus-rtu-sniffer
    components: [modbus_rtu_sniffer]
  - source: { type: local, path: uartpatch }
    components: [uart]
```

The local `uart` overrides the built-in one. **Pin it to the upstream commit it was built from**
and re-fetch the whole directory when updating — editing individual files inside an
`external_components` override accumulates drift that no build error will surface.

## Option 2 — build ESPHome from the branch

```bash
git clone --branch uart-read-frame https://github.com/dalklein/esphome.git
cd esphome && pip install -e .
```

Then no `uart` entry in `external_components` is needed.

## Verifying it took

At INFO level the component logs `uart events: data=N timeout=N`. **Those two must be equal.** If
the component instead logs that the UART has no event queue, `event_queue_size` is missing from
your `uart:` block.
