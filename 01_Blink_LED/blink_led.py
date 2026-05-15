# =============================================================================
# Blink LED — Raspberry Pi GPIO Tutorial
# Author  : Masoud Bakhshi
# Date    : 2026-05-15
# Hardware: Raspberry Pi + GeeekPi GPIO Screw Terminal HAT + any standard LED
# =============================================================================
#
# Wiring:
#   GPIO17 (pin 11) → 330 Ω resistor → LED anode (+, long leg)
#   LED cathode (−, short leg) → GND (pin 6)
#
# Note: uses lgpio — the native GPIO library for Raspberry Pi OS kernel 6.x+.
#       RPi.GPIO is outdated and silently fails on newer kernels.
# =============================================================================

import lgpio
import time

LED_PIN   = 17    # BCM numbering — GPIO17
BLINK_ON  = 0.5  # seconds LED stays on
BLINK_OFF = 0.5  # seconds LED stays off

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, LED_PIN)

print("Blinking LED on GPIO17 — press Ctrl+C to stop")

try:
    while True:
        lgpio.gpio_write(h, LED_PIN, 1)
        time.sleep(BLINK_ON)
        lgpio.gpio_write(h, LED_PIN, 0)
        time.sleep(BLINK_OFF)

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    lgpio.gpio_write(h, LED_PIN, 0)
    lgpio.gpiochip_close(h)
    print("GPIO released.")
