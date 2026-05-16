# =============================================================================
# Closed-Loop PI Current Controller: Raspberry Pi Tutorial
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
#   ADS1263 RST    : GPIO18 | ADS1263 DRDY: GPIO17
#
# MOSFET safety:
#   GPIO23 is configured LOW at Pi boot via /boot/firmware/config.txt
#   (add line: gpio=23=op,dl).
#   Signal handlers (SIGTERM, SIGINT, SIGHUP) guarantee the MOSFET turns off
#   on any exit: normal stop, crash, remote kill, or SSH disconnect.
#
# Controller: PI
#   ADC rate  : 2400 SPS, 5-sample average -> ~480 Hz control loop
#   Kp        : proportional gain (react to current error)
#   Ki        : integral gain (eliminate steady-state offset)
#   Reference : entered by the user at startup (0.1 to 18.0 A)
# =============================================================================

import spidev
import lgpio
import time
import signal
import sys

# Hardware pins
MOSFET_PIN = 23
RST_PIN    = 18
DRDY_PIN   = 17

PWM_FREQ   = 500

# ADS1263 SPI
SPI_BUS    = 0
SPI_DEVICE = 0
SPI_SPEED  = 2000000

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

# ACS712 20A
SENSITIVITY = 0.100  # V/A
VREF        = 2.5

# PI parameters
Kp              = 8.0
Ki              = 4.0
INTEGRAL_CLAMP  = 35.0   # anti-windup (% duty)
MAX_CURRENT_A   = 18.0   # ACS712 20A module, leaving 2A safety margin

# ADC averaging
AVERAGE_N = 5            # 5 samples at 2400 SPS = ~2.1ms per reading = ~480 Hz loop

# =============================================================================
# Hardware setup
# =============================================================================

spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = SPI_SPEED
spi.mode = 0b01

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, RST_PIN, 1)
lgpio.gpio_claim_input(h, DRDY_PIN)
lgpio.gpio_claim_output(h, MOSFET_PIN, 0)   # initialize LOW (MOSFET off)


def _write_reg(reg, value):
    spi.xfer2([CMD_WREG | reg, 0x00, value])


def _wait_drdy(timeout=3.0):
    t0 = time.time()
    while lgpio.gpio_read(h, DRDY_PIN) == 1:
        if time.time() - t0 > timeout:
            raise TimeoutError("ADS1263 DRDY timeout. Check HAT seating and SPI.")
        time.sleep(0.00005)


def _read_adc1_raw():
    _wait_drdy()
    data = spi.xfer2([CMD_RDATA1, 0x00, 0x00, 0x00, 0x00, 0x00])
    raw = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5]
    if raw >= 0x80000000:
        raw -= 0x100000000
    return raw


def read_current(zero_v):
    samples = [_read_adc1_raw() for _ in range(AVERAGE_N)]
    v = (sum(samples) / AVERAGE_N / 0x7FFFFFFF) * VREF
    return (v - zero_v) / SENSITIVITY


def set_duty(duty):
    duty = max(0.0, min(100.0, duty))
    lgpio.tx_pwm(h, MOSFET_PIN, PWM_FREQ, duty)


def mosfet_off():
    lgpio.tx_pwm(h, MOSFET_PIN, 0, 0)
    lgpio.gpio_write(h, MOSFET_PIN, 0)


def ads_init():
    lgpio.gpio_write(h, RST_PIN, 0)
    time.sleep(0.01)
    lgpio.gpio_write(h, RST_PIN, 1)
    time.sleep(0.5)
    spi.xfer2([CMD_RESET])
    time.sleep(0.5)
    _write_reg(REG_MODE2, 0x0A)   # 2400 SPS, gain 1x
    _write_reg(REG_INPMUX, 0x01)  # AIN0(+) vs AIN1(-)
    _write_reg(REG_REFMUX, 0x00)  # internal 2.5V reference
    spi.xfer2([CMD_START1])
    time.sleep(0.5)


# =============================================================================
# Safe shutdown on any exit signal
# =============================================================================

_shutdown_done = False

def safe_shutdown(signum=None, frame=None):
    global _shutdown_done
    if _shutdown_done:
        return
    _shutdown_done = True
    try:
        mosfet_off()
        lgpio.gpiochip_close(h)
        spi.xfer2([CMD_STOP1])
        spi.close()
    except Exception:
        pass
    print("\nMOSFET off. Shutdown complete.")
    sys.exit(0)

signal.signal(signal.SIGINT,  safe_shutdown)
signal.signal(signal.SIGTERM, safe_shutdown)
signal.signal(signal.SIGHUP,  safe_shutdown)


# =============================================================================
# PID controller
# =============================================================================

def run_pid(zero_v, target_abs, initial_duty):
    integral = 0.0
    duty     = initial_duty

    set_duty(duty)
    time.sleep(0.3)
    for _ in range(4):      # flush stale ADC samples
        _read_adc1_raw()

    t_prev  = time.time()
    t_start = t_prev

    print(f"\nTarget: {target_abs:+.1f} A   Kp={Kp}  Ki={Ki}  PWM={PWM_FREQ} Hz  Loop~480 Hz")
    print(f"{'Time':>6}  {'Current':>9}  {'Error':>7}  {'Duty':>6}  Bar")
    print("-" * 62)

    while True:
        i_meas = read_current(zero_v)
        i_abs  = abs(i_meas)
        error  = target_abs - i_abs

        t_now  = time.time()
        dt     = max(t_now - t_prev, 0.001)
        t_prev = t_now

        integral += Ki * error * dt
        integral  = max(-INTEGRAL_CLAMP, min(INTEGRAL_CLAMP, integral))

        duty = Kp * error + integral
        duty = max(0.0, min(100.0, duty))
        set_duty(duty)

        elapsed = t_now - t_start
        bar     = '█' * min(int(i_abs / target_abs * 20), 20)
        marker  = '|' if i_abs >= target_abs * 0.98 else ''

        print(
            f"{elapsed:6.1f}s  "
            f"{i_meas:+7.3f} A  "
            f"{error:+5.3f} A  "
            f"{duty:5.1f}%  "
            f"{bar:<20}{marker}",
            flush=True
        )


# =============================================================================
# Main
# =============================================================================

print("=" * 62)
print("  MOSFET PI Closed-Loop Current Controller")
print("  Masoud Bakhshi")
print("=" * 62)

ads_init()
print("ADS1263 initialised at 2400 SPS.\n")

# Zero-current calibration
print("Calibrating (MOSFET OFF, no current)...")
time.sleep(1.5)
zero_v = sum([(sum([_read_adc1_raw() for _ in range(AVERAGE_N)]) / AVERAGE_N / 0x7FFFFFFF) * VREF
              for _ in range(5)]) / 5
print(f"Zero offset: {zero_v * 1000:.2f} mV\n")

# Polarity check
print("Detecting polarity...")
set_duty(25)
time.sleep(0.4)
i_check = read_current(zero_v)
mosfet_off()
polarity = -1 if i_check < 0 else 1
print(f"Measured at 25% duty: {i_check:+.3f} A  (polarity: {'negative' if polarity < 0 else 'positive'})")

while True:
    try:
        raw = input(f"Enter reference current in amps (0.1 to {MAX_CURRENT_A}): ").strip()
        TARGET_A = float(raw)
        if not (0.1 <= TARGET_A <= MAX_CURRENT_A):
            print(f"  Value out of range. Enter a number between 0.1 and {MAX_CURRENT_A}.")
            continue
        break
    except ValueError:
        print("  Invalid input. Enter a number, e.g. 1.5")

if abs(i_check) > 0.1:
    initial_duty = min(25.0 * TARGET_A / abs(i_check), 80.0)
else:
    initial_duty = 10.0
print(f"Initial duty estimate: {initial_duty:.1f}%\n")

print(f"Starting PI control. Target: {polarity * TARGET_A:+.2f} A")
print("Press Ctrl+C to stop.\n")

try:
    run_pid(zero_v, TARGET_A, initial_duty)
except Exception as e:
    print(f"\nError: {e}")
finally:
    safe_shutdown()
