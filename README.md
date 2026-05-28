# Arduino Mega KWP1281 ECU Emulator

[![CI](https://github.com/RXTX4816/OBDisplay-Emu/actions/workflows/ci.yml/badge.svg)](https://github.com/RXTX4816/OBDisplay-Emu/actions/workflows/ci.yml)
[![RAM](https://img.shields.io/badge/RAM-1880_%2F_8192_B_(22.9%25)-blue)](https://github.com/RXTX4816/OBDisplay-Emu/actions/workflows/ci.yml)
[![Flash](https://img.shields.io/badge/Flash-23904_%2F_253952_B_(9.4%25)-blue)](https://github.com/RXTX4816/OBDisplay-Emu/actions/workflows/ci.yml)
[![Platform](https://img.shields.io/badge/platform-ATmega2560-orange)](https://docs.platformio.org/en/latest/boards/atmelavr/megaatmega2560.html)

This code is for the Arduino Mega with a 480x320 Non-Touch Color Display Shield (Optional)

![](assets/mainscreen.png)

Used to test [OBDisplay-Uno](https://github.com/RXTX4816/OBDisplay-Uno) or [OBDisplay-Mega](https://github.com/RXTX4816/OBDisplay-Mega) when not having a real ECU around.

## Installation

Pre-built firmware is available on the [Releases](https://github.com/RXTX4816/OBDisplay-Emu/releases) page — no toolchain required.

### Download and flash

1. Download `OBDisplay-Emu-<version>.hex` from the latest release.
2. Flash with `avrdude` (included with the Arduino IDE or install separately):
   ```bash
   avrdude -c arduino -p atmega2560 -P /dev/ttyUSB0 -b 115200 \
     -U flash:w:OBDisplay-Uno-<version>.hex:i
   ```
   Replace `/dev/ttyUSB0` with your port (`COM3` on Windows, `/dev/cu.usbmodem*` on macOS).
3. Or upload directly via PlatformIO if you have the repo cloned:
   ```bash
   pio run --target upload
   ```

The release also includes `OBDisplay-Emu-<version>.elf` — this is for developers only (symbol-level debugging with `avr-gdb`, flash analysis with `avr-nm`). You cannot flash it directly.


## Features

Uses Hardwareserial and implements ECHO (Every byte_in is mirrored by the KLine interface).
Full-duplex=true // If Hardwareserial client
Full duplex=false // If Softwareserial

Mimicks a KWP1281 ECU on address 0x17 with baud 10400.

Displays the last 20 received message_types on the LCD

Features:
- 5baud init
- sync bytes and device data
- acknowledge and group reading (only group 1, 2 and 3 implemented)
- automatic reconnect
- LCD display shows connection status

## Compatibility knobs

Some KWP1281 clients are picky about physical-layer behavior. These compile-time knobs live in [src/server.h](src/server.h):

- `KWP_EMU_ECHO_RX_BYTES` (default `0`): optionally echo received bytes back on TX.
- `KWP_EMU_INTERBYTE_DELAY_MS` (default `5`): pacing between bytes.
- `KWP_EMU_CONSUME_OPTIONAL_LAST_COMPLEMENT` (default `1`): tolerate clients that also send a complement after the final byte.
