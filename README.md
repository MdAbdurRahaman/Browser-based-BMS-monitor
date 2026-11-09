# Browser-based BMS Monitor

A single-file web app that reads live battery data from a smart BMS over a USB serial (COM) port, using the browser's [Web Serial API](https://developer.mozilla.org/en-US/docs/Web/API/Web_Serial_API). It has no backend, no build step and no dependencies to install.

## Features

- **Pack overview:** state of charge, voltage, current, power, remaining and full capacity (Ah), cycle count, temperatures, and estimated time to empty or full
- **Cell view:** per-cell voltages with min, max, average and delta, balancing indicators, and an automatic LiFePO₄ or Li-ion scale
- **Status:** charge and discharge MOSFET state and decoded protection faults
- **Trend chart:** live voltage and current history
- **Serial log:** every TX/RX frame in hex, plus a box for sending raw hex commands
- **CSV export:** saves the recorded history (time, voltage, current, SOC, temperatures, every cell)
- Light and dark themes, works on narrow screens

## Supported BMS protocols

| Protocol | Typical boards | Default baud | Commands used |
|---|---|---|---|
| JBD / Xiaoxiang | JBD, Overkill Solar, many "smart BMS" boards | 9600 | `0x03` basic info, `0x04` cell voltages |
| Daly UART | Daly smart BMS (UART/USB) | 9600 | `0x90`–`0x95`, `0x98` |

Other protocols (JK, ANT, Seplos, Pace RS485) are not decoded yet, but you can still talk to them with the raw hex console.

## Requirements

- **Chrome, Edge or Opera on desktop** (Windows, macOS, Linux). Firefox and Safari don't support Web Serial.
- A USB-to-UART adapter (CH340, CP2102, FT232) or the BMS's own USB cable, with its driver installed.
- The page must be opened from `file://`, `localhost` or `https://`. Web Serial is blocked on plain `http://`.

## Usage

1. Download `index.html`, or clone the repo:
   ```bash
   git clone https://github.com/MdAbdurRahaman/Browser-based-BMS-monitor.git
   ```
2. Open `index.html` in Chrome or Edge.
3. Choose the protocol and baud rate (9600 for most boards).
4. Click **Connect COM port** and pick your port from the browser's list, for example `USB-SERIAL CH340 (COM19)`.
5. The dashboard refreshes at the interval you choose (1–10 s).

Before you connect, the page shows sample values marked **Demo data**.

### Hosting on GitHub Pages

Turn on Pages for this repo (Settings → Pages → deploy from `main`, root folder). The app is then served over HTTPS at
`https://mdabdurrahaman.github.io/Browser-based-BMS-monitor/`, and Web Serial works from there.

## Wiring notes

- Connect adapter **TX → BMS RX**, **RX → BMS TX**, and **GND → GND**. Don't connect the adapter's VCC unless your BMS documentation says to.
- Check that your adapter's logic level (3.3 V or 5 V) matches what the BMS UART expects.

## Troubleshooting

| Symptom | Things to check |
|---|---|
| "Could not open the port" | Another program has the port open. Close the vendor app, Arduino serial monitor, PuTTY and similar tools. |
| TX frames in the log, no RX | Swap TX and RX. Confirm the baud rate. Wake the BMS by connecting a charger or load, since many boards sleep. |
| RX bytes appear but nothing decodes | The protocol selection is probably wrong. Try the other one, or copy the raw reply and open an issue. |
| No COM port in the list | The USB-serial driver is missing (for example CH340 or CP210x on Windows). |

## Protocol reference

**JBD:** request `DD A5 <cmd> 00 <chk_hi> <chk_lo> 77`, where the checksum is `0x10000 − sum(cmd, len, data)`.
Basic info: `DD A5 03 00 FF FD 77`. Cell voltages: `DD A5 04 00 FF FC 77`.

**Daly:** 13-byte frames `A5 40 <cmd> 08 <8 data bytes> <chk>`, where the checksum is the low byte of the sum of the first 12 bytes.
Example: `A5 40 90 08 00 00 00 00 00 00 00 00 7D`.

## Project structure

```
index.html   the whole app (HTML, CSS and JavaScript in one file)
README.md    this file
```

## Disclaimer

This is a monitoring tool and it only sends read commands. It does not change BMS settings, unless you send a write command yourself through the raw hex console. Use at your own risk when working with high-energy battery packs.
