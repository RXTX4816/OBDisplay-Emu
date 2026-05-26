# Logic analyzer capture checklist (KWP1281 / K-line)

This page is a practical checklist to capture a **real** KWP1281 session so the emulator can be made byte- and timing-accurate.

The safest and most useful approach is to probe the **TTL UART side** (RX/TX/GND) of your interface (e.g., inside a modified KKL cable / transceiver board), not the 12V K-line directly.

## Safety (read first)

- Do **not** connect a typical cheap logic analyzer input directly to the vehicle K-line (OBD pin 7). K-line is ~12V and bidirectional.
- Probe only:
  - the transceiver/adapter **TTL UART** pins (usually 5V or 3.3V)
  - and **GND**
- Confirm your analyzer’s input tolerance:
  - Many cheap analyzers are **3.3V-only**.
  - If your TTL lines are 5V, use a level shifter or a resistor divider.

## What to capture

Capture both directions at once:
- **Tester → ECU** (TX)
- **ECU → Tester** (RX)

If your interface is already modified so RX/TX go directly to an Arduino, probe on those TTL wires:
- Analyzer CH1 → TX (tester output)
- Analyzer CH2 → RX (tester input)
- Analyzer GND → GND

## PulseView (sigrok) setup

1. Start PulseView.
2. Select your logic analyzer device.
3. Set sample rate:
   - For 10400 baud, **2–4 MHz** is typically plenty and keeps files smaller.
4. Add decoders:
   - Add an **Async Serial** decoder on the TX channel.
   - Add an **Async Serial** decoder on the RX channel.
5. Serial parameters:
   - Baud: **10400**
   - Data bits: **8**
   - Parity: **None**
   - Stop bits: **1**

## Capturing the 5-baud init

5 baud means **200 ms per bit**, so it will look *very slow* in the trace.

- Start capture *before* initiating a session.
- You may not be able to “decode” 5-baud automatically with the same decoder settings.
- That’s OK: you mainly need the waveform to measure:
  - bit widths (should be ~200 ms)
  - the address byte pattern
  - the time gap before switching to 10400

Tip: you can keep the async serial decoders set to 10400 for the main session, and simply zoom out to visually inspect the 5-baud region.

## Recording procedure

1. Start PulseView capture.
2. Start your tester (VCDS / Arduino tester / other).
3. Perform a short, repeatable script (for address 0x17):
   - connect
   - read ID / measuring blocks
   - read DTCs
   - disconnect
4. Stop capture.
5. Save the session file (PulseView `.sr`).

## Exporting data for emulator work

For emulator implementation, you want a “golden transcript” you can diff.

- Export decoded bytes for TX and RX (CSV) with timestamps.
- If you can’t export both directions into one CSV, export them separately and keep them together.

What matters most:
- exact byte order
- any per-byte complement/handshake behavior
- block framing (`LEN`, `CNT`, `TYPE`, payload, `0x03`)
- inter-byte gaps and ECU response delays

## Common pitfalls

- Wrong ground reference (noisy/garbled decode) → connect analyzer GND to the same GND as the interface.
- Wrong voltage (3.3V analyzer on 5V signal) → level shift.
- TX/RX swapped → decode looks like nonsense; swap channels.
- Capturing on the 12V K-line directly → likely to damage equipment.
