# WinDECT

**US DECT 6.0 wideband scanner and live voice decoder for Windows.**

Uses a [HackRF One](https://greatscottgadgets.com/hackrf/) to monitor all 10 US DECT 6.0 channels simultaneously, automatically lock onto active calls, and play decoded voice audio in real time.

![WinDECT scanning all 10 DECT channels](screenshot.JPG)

---

## Requirements

- Windows 10 or 11 (64-bit)
- [HackRF One](https://greatscottgadgets.com/hackrf/) with the WinUSB driver installed
  - Install via [Zadig](https://zadig.akeo.ie/): select **HackRF One**, choose **WinUSB**, click *Install Driver*

---

## Quick Start

1. Download the latest release zip from the [Releases](../../releases) page
2. Extract the folder anywhere — keep `windect.exe` and the three `.dll` files together
3. Plug in your HackRF One
4. Open a Command Prompt or Windows Terminal in the extracted folder
5. Run:

```
windect.exe
```

WinDECT will immediately start scanning all 10 US DECT 6.0 channels. When it detects a voice call it locks onto that channel, plays the decoded audio through your default speakers, and returns to scanning when the call ends.

---

## How It Works

| Mode | Behaviour |
|------|-----------|
| **Scanning** | HackRF tunes to 1929.312 MHz at 18.432 Msps. All 10 channels are decoded simultaneously. A live table shows packet counts and voice activity. |
| **Locked** | On first voice packet, HackRF retunes to the active channel at 4.608 Msps. G.721 ADPCM is decoded and played at 8 kHz. Both sides of the call (RFP + PP) are mixed into one audio stream. |
| **Call ended** | After 2 seconds of silence the scanner returns to wideband mode automatically. |

---

## Options

```
windect [options]

  -g <lna>   LNA gain  0-40 dB, steps of 8  (default: 32)
  -v <vga>   VGA gain  0-62 dB, steps of 2  (default: 20)
  -a         Enable HackRF RF amplifier (+14 dB)
  -q         Quiet: suppress live channel table, show events only
  -h         Show help
```

**Tip:** If voice sounds weak or the scanner misses calls, try increasing LNA gain (`-g 40`) or enabling the amp (`-a`). If the HackRF overloads near strong signals, lower the gains.

---

## Channels

| Ch | Frequency |
|----|-----------|
| 0  | 1921.536 MHz |
| 1  | 1923.264 MHz |
| 2  | 1924.992 MHz |
| 3  | 1926.720 MHz |
| 4  | 1928.448 MHz |
| 5  | 1930.176 MHz |
| 6  | 1931.904 MHz |
| 7  | 1933.632 MHz |
| 8  | 1935.360 MHz |
| 9  | 1937.088 MHz |

---

## Legal Notice

DECT 6.0 telephone calls are private communications. Laws regarding the interception of wireless communications vary by jurisdiction. This software is provided for educational and research purposes only. Use responsibly and in accordance with applicable laws.
