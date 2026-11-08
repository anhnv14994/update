# HihiFi Optical S/PDIF Input

Optical input for ESP32-S3 using a TOSLINK receiver module whose DATA pin provides
3.3 V logic-level S/PDIF.

## Current bring-up scope

- ESP-IDF 5.2.x finite RMT RX transactions with DMA.
- Dedicated decoder task; RMT ISR only publishes the completed DMA symbol count.
- Timing acquisition and Biphase-Mark decode adapted from netham45/esp32-spdif.
- PCM output: interleaved stereo signed 16-bit.
- Detected rates: 44.1 kHz and 48 kHz only.
- Optical Direct currently bypasses PEQ, Analog Character, and Stereo Spatial.
- I2S clock follows the detected nominal rate. The current phase is a bring-up
  implementation; long-duration clock-drift compensation and hardware soak
  testing remain required before calling it production-stable.

## Configuration

Open `idf.py menuconfig` → HihiFi Optical Input:

- Enable Optical S/PDIF input.
- Set DATA GPIO to the GPIO physically wired to the receiver module.
- The default GPIO4 is provisional and must be checked against the actual board.

The receiver module supply must match its datasheet. Its DATA output must not
exceed the ESP32-S3 GPIO's 3.3 V input limit.

## Source priority

- Valid Optical lock: Optical takes priority and the RAOP/AirPlay receiver is
  deinitialized.
- Optical lock lost: release I2S, restore 44.1 kHz baseline, and allow AirPlay
  to start again when Wi-Fi is connected.
- Only one source writes to I2S at a time.

## Source format

Configure the TV/transport optical output to **PCM stereo** at 44.1 or 48 kHz.
This decoder does not decode Dolby Digital, DTS, or other compressed bitstreams.
24-bit subframe audio is currently reduced to 16-bit PCM.

## Build

Use the project's existing ESP-IDF 5.2.7 environment:

```powershell
idf.py set-target esp32s3
idf.py reconfigure
idf.py build
```

Do not flash until the DATA GPIO and receiver-module logic voltage have been
verified on the actual board.

## Third-party license

The pulse-timing and Biphase-Mark decoder is adapted from
[netham45/esp32-spdif](https://github.com/netham45/esp32-spdif), licensed under
AGPL-3.0. See [COPYING](COPYING) and [NOTICE.md](NOTICE.md). Review the license
obligations before distributing firmware containing this component.
