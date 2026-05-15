# =============================================================================
# PWM LED Dimming — Raspberry Pi GPIO Tutorial
# Author  : Masoud Bakhshi
# Date    : 2026-05-15
# Hardware: Raspberry Pi + GeeekPi GPIO Screw Terminal HAT + LED + 330 Ω resistor
# =============================================================================
#
# Wiring (same as 01_Blink_LED):
#   GPIO17 (pin 11) → 330 Ω resistor → LED anode (+, long leg)
#   LED cathode (−, short leg) → GND (pin 6)
#
# PWM frequency : 1000 Hz  — fast enough to eliminate visible flicker
# Duty cycle    : 0 % → 100 % → 0 % in smooth steps (fade in / fade out)
#
# Note: uses lgpio — the correct GPIO library for Raspberry Pi OS kernel 6.x+
# =============================================================================

import lgpio
import time

LED_PIN   = 17      # BCM numbering — GPIO17
FREQUENCY = 1000    # Hz — PWM carrier frequency
STEP      = 1       # duty cycle step size (%)
STEP_DELAY = 0.02   # seconds between steps — controls fade speed

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, LED_PIN)

def set_duty(duty):
    lgpio.tx_pwm(h, LED_PIN, FREQUENCY, duty)

print("PWM dimming on GPIO17 — press Ctrl+C to stop")
print(f"Frequency: {FREQUENCY} Hz | Step: {STEP}% every {int(STEP_DELAY*1000)} ms\n")

try:
    while True:
        # fade in: 0% → 100%
        for duty in range(0, 101, STEP):
            set_duty(duty)
            print(f"\rDuty cycle: {duty:3d}%  {'█' * (duty // 5):<20}", end="", flush=True)
            time.sleep(STEP_DELAY)

        # fade out: 100% → 0%
        for duty in range(100, -1, -STEP):
            set_duty(duty)
            print(f"\rDuty cycle: {duty:3d}%  {'█' * (duty // 5):<20}", end="", flush=True)
            time.sleep(STEP_DELAY)

except KeyboardInterrupt:
    print("\n\nStopped by user.")

finally:
    set_duty(0)
    lgpio.tx_pwm(h, LED_PIN, 0, 0)
    lgpio.gpiochip_close(h)
    print("GPIO released.")
