# UV-K5 APRS — browser tools

Browser-based companion tools for the **TA1JS APRS edition** of the Quansheng
UV-K5/K6/5R firmware. No install — everything runs in the browser over Web
Serial (desktop Chrome/Edge) or WebUSB (Android Chrome + OTG).

**Live:** https://uvk5.canata.dev

A single-page app (bilingual TR/EN) with:

- **Install** — guided in-browser flasher: back up, wipe, flash the firmware, and write your callsign/position, all over USB.
- **Web Beacon** — live-GPS APRS beacon with SmartBeaconing.
- **Codes** — turn a location/callsign/message into keypad codes.
- **Control** — live screen mirror + virtual keypad + TX controls (needs TA1JS v1.1+ firmware).
- **Channels** — read/edit/write memory channels, import from an EEPROM backup.
- **Settings** — friendly editor for every stored setting; restore from backup.
- **APRS** — live packet monitor + a chat interface for APRS messaging (stored in your browser).

The prebuilt firmware image (`ta1js.bin`) is served alongside the app for the
in-browser flasher.

## Firmware

Source: **https://github.com/bcanata/uv-k5-firmware-ta1js**

A fork of F4HWN ← egzumer ← OneOfEleven/fagci ← DualTachyon's open
re-implementation. See that repo for the firmware licence and lineage.

## Licence

Web tools: [MIT](LICENSE). Firmware: Apache-2.0 (in the firmware repo).
