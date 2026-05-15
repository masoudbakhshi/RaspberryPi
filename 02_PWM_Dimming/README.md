# 02 — PWM LED Dimming

**Author:** Masoud Bakhshi  
**Level:** Beginner–Intermediate  
**Topic:** PWM (Pulse Width Modulation), analog-like control, LED brightness

---

## What this does

Instead of simply switching an LED on or off, this project controls its brightness by rapidly toggling the pin at 1000 Hz and varying how long it stays on each cycle — this is called PWM (Pulse Width Modulation). The LED fades smoothly from off to full brightness and back in a continuous loop.

---

## How PWM works

A PWM signal switches between HIGH and LOW at a fixed frequency. The **duty cycle** is the percentage of each period the signal stays HIGH:

```
100% duty cycle → always ON  → full brightness
 50% duty cycle → half ON    → half brightness (appears)
  0% duty cycle → always OFF → LED off
```

At 1000 Hz the switching is 1000 times per second — far too fast for the eye to see as flicker. The eye only perceives the average brightness, which is proportional to the duty cycle. This is how dimmers, motor speed controllers, and audio amplifiers work at a fundamental level.

```
Duty 25%:  ██░░░░░░  ██░░░░░░  ██░░░░░░
Duty 50%:  ████░░░░  ████░░░░  ████░░░░
Duty 75%:  ██████░░  ██████░░  ██████░░
           ← one PWM period →
```

---

## Hardware needed

| Item | Purpose |
|------|---------|
| Raspberry Pi (any model with 40-pin header) | Main board |
| GeeekPi GPIO Screw Terminal HAT | Easy wiring access |
| Standard LED (any color) | Visual output |
| 330 Ω resistor | Current limiting |

Wiring is identical to `01_Blink_LED` — no changes needed if already wired.

---

## Wiring

```
RPi GPIO17 (pin 11)
        │
      [330 Ω]
        │
      LED (+) anode
      LED (−) cathode
        │
      GND (pin 6)
```

---

## How to run

```bash
python3 pwm_dimming.py
```

You will see a live duty cycle readout in the terminal while the LED fades in and out:

```
Duty cycle:  75%  ███████████████
```

Stop with **Ctrl+C**.

---

## Things to try

- Change `FREQUENCY` to 50 Hz — you will see visible flicker because it drops below the eye's persistence threshold (~60 Hz).
- Change `STEP_DELAY` to `0.005` for a faster fade or `0.05` for a slower one.
- Set a fixed duty cycle instead of a loop — replace the `while True` block with `set_duty(50)` and `time.sleep(10)` to hold 50% brightness for 10 seconds.

---

## Related tutorials in this repo

- [`01_Blink_LED`](../01_Blink_LED/) — GPIO digital output basics
- [`03_ACS712_Current_Sensor`](../03_ACS712_Current_Sensor/) — measure current through the LED with ACS712 + ADS1263
