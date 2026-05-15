# Raspberry Pi Projects

**Author:** Masoud Bakhshi

A collection of hands-on Raspberry Pi projects aimed at students and engineers who want to get comfortable with GPIO, sensors, power electronics, and real hardware interfacing. Each project is self-contained with its own wiring guide and explanation of what's happening under the hood.

---

## Hardware used across these projects

| # | Module | What it's for |
|--:|--------|---------------|
| 1 | AZDelivery TXS0108E (×3) | 3.3 V ↔ 5 V logic level conversion |
| 2 | AZDelivery HW-140 | Adjustable buck-boost DC power supply |
| 3 | G4 Halogen Bulbs 12 V 20 W (×12) | Resistive load for switching and dimming experiments |
| 4 | AZDelivery ACS712 20A (×5) | DC/AC current measurement |
| 5 | DollaTek D4184 MOSFET Module (×5) | Low-side switching and PWM control |
| 6 | Waveshare ADS1263 ADC HAT | 32-bit precision analog input via SPI |
| 7 | GeeekPi GPIO Screw Terminal HAT | Safe screw-terminal access to all GPIO pins |

---

## Project list

| # | Folder | Topic | Level |
|--:|--------|-------|-------|
| 01 | [01_Blink_LED](01_Blink_LED/) | GPIO output — blink an LED | Beginner |

More projects are added over time as the series grows.

---

## Getting started

All scripts are written in Python 3 and tested on Raspberry Pi OS. Clone the repo to your Pi:

```bash
git clone https://github.com/masoudbakhshi/RaspberryPi.git
cd RaspberryPi
```

Each project folder has its own `README.md` with full wiring instructions and an explanation of how the code works. Start with `01_Blink_LED` if you're new to GPIO.
