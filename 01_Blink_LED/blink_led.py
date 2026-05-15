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
# The screw terminal on the GeeekPi HAT labeled GPIO17 connects directly
# to BCM pin 17. Use the GND terminal next to it to complete the circuit.
# =============================================================================

import RPi.GPIO as GPIO
import time

LED_PIN = 17        # BCM numbering — matches GPIO17 label on GeeekPi HAT
BLINK_ON  = 0.5    # seconds the LED stays on
BLINK_OFF = 0.5    # seconds the LED stays off

GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)

print("Blinking LED on GPIO17 — press Ctrl+C to stop")

try:
    while True:
        GPIO.output(LED_PIN, GPIO.HIGH)
        time.sleep(BLINK_ON)
        GPIO.output(LED_PIN, GPIO.LOW)
        time.sleep(BLINK_OFF)

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    GPIO.cleanup()   # always reset pin state on exit
    print("GPIO cleaned up.")
