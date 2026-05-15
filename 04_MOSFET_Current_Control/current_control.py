# =============================================================================
# Closed-Loop MOSFET PWM Current Controller: Raspberry Pi Tutorial
# Author  : Masoud Bakhshi
# Date    : 2026-05-15
# Hardware: Raspberry Pi + Waveshare ADS1263 HAT + ACS712 20A
#           + DollaTek D4184 MOSFET Module + G4 Halogen Lamps (12V)
# =============================================================================
#
# Wiring:
#   MOSFET control : GPIO23 (pin 16) -> D4184 IN
#   Lamp circuit   : 12V -> Lamps -> ACS712 IP+ -> ACS712 IP- -> D4184 OUT -> GND
#   ACS712 signal  : ACS712 OUT -> ADS1263 AIN0
#   AIN1 reference : Pi 3.3V (pin 1) -> ADS1263 AIN1
#   ADS1263 RST    : GPIO18 | ADS1263 DRDY: GPIO17 (do not use for other things)
#
# How it works:
#   A PI controller reads current from the ACS712 every ~50ms and adjusts
#   the MOSFET PWM duty cycle to keep the lamp current at the target setpoint.
#   Proportional term reacts to instantaneous error; integral term eliminates
#   steady-state offset that proportional alone cannot remove.
#
# PI tuning parameters (adjust if needed):
#   Kp: higher = faster response, but may oscillate
#   Ki: higher = faster elimination of steady-state error, but may wind up
# =============================================================================

import spidev
import lgpio
import time

# Hardware pins
MOSFET_PIN = 23   # D4184 MOSFET gate control (PWM output)
RST_PIN    = 18   # ADS1263 reset
DRDY_PIN   = 17   # ADS1263 data ready (input, active LOW)

# PWM settings
PWM_FREQ   = 200  # Hz, suitable for resistive lamp loads

# ADS1263 SPI
SPI_BUS    = 0
SPI_DEVICE = 0
SPI_SPEED  = 2000000  # 2 MHz

# ADS1263 commands
CMD_RESET  = 0x06
CMD_START1 = 0x08
CMD_STOP1  = 0x0A
CMD_RDATA1 = 0x12
CMD_WREG   = 0x40

# ADS1263 registers
REG_MODE2  = 0x04
REG_INPMUX = 0x05
REG_REFMUX = 0x0E

# ACS712 20A parameters
SENSITIVITY = 0.100  # V/A
VREF        = 2.5    # ADS1263 internal reference

# PI controller parameters
TARGET_A   =  1.0   # target current magnitude in amps
Kp         = 15.0   # proportional gain
Ki         =  5.0   # integral gain
INTEGRAL_CLAMP = 40.0  # anti-windup clamp (% duty)

# =============================================================================
# ADS1263 driver (same as project 03)
# =============================================================================

spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = SPI_SPEED
spi.mode = 0b01

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, RST_PIN, 1)
lgpio.gpio_claim_input(h, DRDY_PIN)
lgpio.gpio_claim_output(h, MOSFET_PIN, 0)


def _write_reg(reg, value):
    spi.xfer2([CMD_WREG | reg, 0x00, value])


def _wait_drdy(timeout=3.0):
    t0 = time.time()
    while lgpio.gpio_read(h, DRDY_PIN) == 1:
        if time.time() - t0 > timeout:
            raise TimeoutError("ADS1263 DRDY timeout. Check HAT seating and SPI.")
        time.sleep(0.0001)


def _read_adc1_raw():
    _wait_drdy()
    data = spi.xfer2([CMD_RDATA1, 0x00, 0x00, 0x00, 0x00, 0x00])
    raw = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5]
    if raw >= 0x80000000:
        raw -= 0x100000000
    return raw


AVERAGE_N = 4

def read_voltage():
    samples = [_read_adc1_raw() for _ in range(AVERAGE_N)]
    return (sum(samples) / AVERAGE_N / 0x7FFFFFFF) * VREF


def ads_init():
    lgpio.gpio_write(h, RST_PIN, 0)
    time.sleep(0.01)
    lgpio.gpio_write(h, RST_PIN, 1)
    time.sleep(0.5)
    spi.xfer2([CMD_RESET])
    time.sleep(0.5)
    _write_reg(REG_MODE2, 0x04)   # 20 SPS, gain 1x
    _write_reg(REG_INPMUX, 0x01)  # AIN0(+) vs AIN1(-)
    _write_reg(REG_REFMUX, 0x00)  # internal 2.5V reference
    spi.xfer2([CMD_START1])
    time.sleep(0.5)


def set_duty(duty):
    duty = max(0.0, min(100.0, duty))
    lgpio.tx_pwm(h, MOSFET_PIN, PWM_FREQ, duty)


def cleanup():
    set_duty(0)
    lgpio.tx_pwm(h, MOSFET_PIN, 0, 0)
    lgpio.gpiochip_close(h)
    spi.xfer2([CMD_STOP1])
    spi.close()


# =============================================================================
# PI current controller
# =============================================================================

def run_controller(zero_v, target_abs, polarity, initial_duty=0.0):
    """
    Closed-loop PI current controller.
    target_abs   : target current magnitude in amps (always positive)
    polarity     : +1 or -1, sign of measured current (from calibration check)
    initial_duty : pre-estimated duty cycle to avoid hunting at startup
    """
    integral  = 0.0
    duty      = initial_duty   # start near the expected operating point
    set_duty(duty)

    # Flush stale ADC readings from the calibration and polarity-check phase,
    # and let the lamps settle at the initial duty before the loop begins.
    time.sleep(0.5)
    for _ in range(3):
        _read_adc1_raw()  # discard

    t_prev    = time.time()

    print(f"\nTarget current: {polarity * target_abs:+.1f} A")
    print(f"Kp={Kp}  Ki={Ki}  PWM={PWM_FREQ} Hz")
    print(f"{'Time':>6}  {'Measured':>10}  {'Error':>8}  {'Duty':>6}  Bar")
    print("-" * 60)

    t_start = time.time()

    while True:
        v       = read_voltage()
        i_meas  = (v - zero_v) / SENSITIVITY   # signed current (A)
        i_abs   = abs(i_meas)
        error   = target_abs - i_abs           # positive = need more current

        t_now   = time.time()
        dt      = t_now - t_prev
        t_prev  = t_now

        integral += error * dt
        integral  = max(-INTEGRAL_CLAMP, min(INTEGRAL_CLAMP, integral))

        duty = Kp * error + Ki * integral
        duty = max(0.0, min(100.0, duty))

        set_duty(duty)

        elapsed = t_now - t_start
        bar_len = int(i_abs / target_abs * 20)
        bar     = '█' * min(bar_len, 20)
        target_marker = '|' if bar_len >= 20 else ''

        print(
            f"{elapsed:6.1f}s  "
            f"{i_meas:+8.3f} A  "
            f"{error:+6.3f} A  "
            f"{duty:5.1f}%  "
            f"{bar:<20}{target_marker}",
            flush=True
        )

        time.sleep(0.05)


# =============================================================================
# Main
# =============================================================================

print("=" * 60)
print("  MOSFET PWM Closed-Loop Current Controller")
print("  Masoud Bakhshi")
print("=" * 60)

ads_init()
print("ADS1263 initialised.\n")

# Calibrate with MOSFET off
print("Calibrating zero-current baseline (MOSFET OFF, no current flowing)...")
time.sleep(2)
cal_samples = [read_voltage() for _ in range(5)]
zero_v = sum(cal_samples) / len(cal_samples)
print(f"Zero-current offset: {zero_v * 1000:.2f} mV\n")

# Quick polarity check: pulse MOSFET briefly to detect current direction
print("Detecting current polarity...")
set_duty(30)
time.sleep(0.5)
v_check  = read_voltage()
i_check  = (v_check - zero_v) / SENSITIVITY
polarity = -1 if i_check < 0 else 1
set_duty(0)
print(f"Current polarity: {'negative (IP+/IP- reversed)' if polarity < 0 else 'positive'}")
print(f"Measured at 30% duty: {i_check:+.3f} A")

# Estimate starting duty cycle so the controller begins near the setpoint.
# This avoids the initial on/off hunting caused by starting from 0% duty.
if abs(i_check) > 0.05:
    initial_duty = 30.0 * TARGET_A / abs(i_check)
    initial_duty = max(0.0, min(80.0, initial_duty))
else:
    initial_duty = 0.0
print(f"Estimated initial duty: {initial_duty:.1f}%\n")

print(f"Starting closed-loop control. Target: {polarity * TARGET_A:+.1f} A")
print("Press Ctrl+C to stop.\n")

try:
    run_controller(zero_v, TARGET_A, polarity, initial_duty)

except KeyboardInterrupt:
    print("\n\nStopped by user.")

finally:
    cleanup()
    print("MOSFET off. GPIO and SPI released.")
