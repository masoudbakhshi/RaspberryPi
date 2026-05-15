# 01 — Blink LED

**Author:** Masoud Bakhshi  
**Level:** Beginner  
**Topic:** GPIO output, digital signals, basic wiring

---

## What this does

This is the simplest possible GPIO project. The Raspberry Pi toggles a pin between HIGH (3.3 V) and LOW (0 V) in a loop, which makes an LED blink at a fixed rate. It's a good starting point for understanding how the Pi talks to external components and how to structure a clean GPIO script.

---

## Hardware needed

| Item | Purpose |
|------|---------|
| Raspberry Pi (any model with 40-pin header) | Main board |
| GeeekPi GPIO Screw Terminal HAT | Easy and safe wiring access to GPIO pins |
| Standard LED (any color) | Visual output |
| 330 Ω resistor (220–470 Ω works too) | Current limiting — protects the LED |
| Two short wires | Connections |

---

## Wiring

```
RPi GPIO17 (pin 11)
        │
      [330 Ω]
        │
      LED (+) anode   ← longer leg
      LED (−) cathode ← shorter leg
        │
      GND (pin 6)
```

On the GeeekPi HAT, locate the screw terminals labeled **GPIO17** and **GND** — they sit right next to each other which makes wiring straightforward.

> **Why a resistor?** The Pi's GPIO pins output 3.3 V and can safely source about 16 mA. A typical LED runs at 2 V / 10–20 mA. Without a resistor the current would be too high and could damage the pin or burn out the LED. The resistor drops the extra voltage and keeps current in a safe range.

---

## How to run

1. Make sure `RPi.GPIO` is installed — it's included by default on Raspberry Pi OS. If not:
   ```bash
   pip install RPi.GPIO
   ```

2. Copy `blink_led.py` to your Pi and run it:
   ```bash
   python3 blink_led.py
   ```

3. The LED starts blinking immediately. Stop it with **Ctrl+C**.

---

## What happens in the code

The script sets GPIO17 as an output pin, then enters an infinite loop. Each pass turns the pin HIGH (LED on), waits half a second, turns it LOW (LED off), and waits again. When you hit Ctrl+C, the `finally` block calls `GPIO.cleanup()` which resets the pin to its default input state — always do this at the end to leave the hardware in a clean state.

---

## Things to try

- Change `BLINK_ON` and `BLINK_OFF` at the top of the script to adjust the timing.
- Use a different GPIO pin — just update `LED_PIN` and rewire accordingly.
- Set `BLINK_ON = 0.02` and `BLINK_OFF = 0.02` to get 25 Hz flicker — the LED will look dimmer due to how human eyes perceive fast switching. This is the idea behind PWM dimming.

---

## Related tutorials in this repo

- `02_PWM_Dimming` — control LED brightness with hardware PWM *(coming soon)*
