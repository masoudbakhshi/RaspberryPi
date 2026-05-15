# =============================================================================
# ACS712 Current Sensor via ADS1263 ADC HAT: Raspberry Pi Tutorial
# Author  : Masoud Bakhshi
# Date    : 2026-05-15
# Hardware: Raspberry Pi + Waveshare ADS1263 HAT + ACS712 20A module
#           + GeeekPi Screw Terminal HAT + LED + 330 ohm resistor
# =============================================================================
#
# Wiring overview:
#   LED circuit  : GPIO27 -> 330 ohm -> LED(+) -> LED(-) -> ACS712 IP+ -> ACS712 IP- -> GND
#   ACS712 power : ACS712 VCC -> 5V (Pi pin 2) | ACS712 GND -> GND (Pi pin 6)
#   ACS712 signal: ACS712 OUT -> ADS1263 AIN0 screw terminal
#   2.5V reference: 10kΩ from 5V -> AIN1 screw terminal -> 10kΩ -> GND
#
# Why differential (AIN0 vs AIN1)?
#   The ACS712 outputs 2.5V at zero current, which equals the ADS1263 internal
#   2.5V reference. Measuring single-ended would saturate the ADC at zero current.
#   A voltage divider on AIN1 creates a 2.5V reference so the differential
#   reading is 0V at zero current and scales with actual current flow.
#
# ADS1263 GPIO pins used by the Waveshare HAT:
#   RST   -> GPIO18 (pin 12)
#   DRDY  -> GPIO17 (pin 11) -- do NOT use GPIO17 for anything else
#   SPI0  -> GPIO8/9/10/11 (CE0, MISO, MOSI, SCLK)
#
# ACS712 20A sensitivity: 100 mV/A
# =============================================================================

import spidev
import lgpio
import time

# Hardware pins
RST_PIN  = 18   # ADS1263 reset (output)
DRDY_PIN = 17   # ADS1263 data ready (input, active LOW)
LED_PIN  = 27   # LED control (GPIO27, because GPIO17 is taken by DRDY)

# ADS1263 SPI
SPI_BUS    = 0
SPI_DEVICE = 0        # CE0 = GPIO8
SPI_SPEED  = 2000000  # 2 MHz

# ADS1263 commands
CMD_RESET  = 0x06
CMD_START1 = 0x08
CMD_STOP1  = 0x0A
CMD_RDATA1 = 0x12
CMD_WREG   = 0x40

# ADS1263 registers
REG_MODE2  = 0x04   # PGA gain and data rate
REG_INPMUX = 0x05   # input channel multiplexer
REG_REFMUX = 0x0E   # reference source

# ACS712 20A parameters
SENSITIVITY = 0.100  # V/A (100 mV per amp)
VREF        = 2.5    # ADS1263 internal reference voltage

# =============================================================================
# ADS1263 driver
# =============================================================================

spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = SPI_SPEED
spi.mode = 0b01  # SPI mode 1: CPOL=0, CPHA=1

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, RST_PIN, 1)
lgpio.gpio_claim_input(h, DRDY_PIN)
lgpio.gpio_claim_output(h, LED_PIN, 0)


def _write_reg(reg, value):
    spi.xfer2([CMD_WREG | reg, 0x00, value])


def _wait_drdy(timeout=3.0):
    t0 = time.time()
    while lgpio.gpio_read(h, DRDY_PIN) == 1:
        if time.time() - t0 > timeout:
            raise TimeoutError(
                "ADS1263 DRDY timeout. Check that the HAT is seated and SPI is enabled."
            )
        time.sleep(0.0001)


def _read_adc1_raw():
    _wait_drdy()
    spi.writebytes([CMD_RDATA1])
    data = spi.readbytes(5)   # STATUS(1) + DATA(4)
    raw = (data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4]
    if raw >= 0x80000000:     # two's complement for negative values
        raw -= 0x100000000
    return raw


def ads_init():
    lgpio.gpio_write(h, RST_PIN, 0)
    time.sleep(0.01)
    lgpio.gpio_write(h, RST_PIN, 1)
    time.sleep(0.5)

    spi.xfer2([CMD_RESET])
    time.sleep(0.5)

    # 20 SPS, PGA gain = 1x
    _write_reg(REG_MODE2, 0x04)

    # Differential: AIN0(+) vs AIN1(-)
    _write_reg(REG_INPMUX, 0x01)

    # Internal 2.5V reference
    _write_reg(REG_REFMUX, 0x00)

    # Start continuous conversions
    spi.xfer2([CMD_START1])
    time.sleep(0.5)


def read_voltage():
    raw = _read_adc1_raw()
    return (raw / 0x7FFFFFFF) * VREF


def read_current_A(zero_voltage):
    v = read_voltage()
    return (v - zero_voltage) / SENSITIVITY


def cleanup():
    spi.xfer2([CMD_STOP1])
    lgpio.gpio_write(h, LED_PIN, 0)
    lgpio.gpiochip_close(h)
    spi.close()


# =============================================================================
# Main
# =============================================================================

print("=" * 50)
print("  ACS712 Current Sensor")
print("  Masoud Bakhshi")
print("=" * 50)

ads_init()
print("ADS1263 initialised.\n")

# Calibrate zero-current baseline
print("Calibrating zero-current baseline...")
print("Make sure NO current is flowing through the ACS712 (LED should be OFF).")
time.sleep(2)

samples = [read_voltage() for _ in range(20)]
zero_v = sum(samples) / len(samples)
print(f"Zero-current reference voltage: {zero_v * 1000:.2f} mV")
print(f"(Ideal is ~0 mV differential; any offset is from component tolerance)\n")

# Measure current
print("Turning LED ON, measuring current through the circuit.")
print("Press Ctrl+C to stop.\n")

lgpio.gpio_write(h, LED_PIN, 1)
time.sleep(0.2)

try:
    while True:
        current_A  = read_current_A(zero_v)
        current_mA = current_A * 1000
        bar = '█' * min(int(abs(current_mA) / 1), 30)
        print(f"\rCurrent: {current_mA:+7.2f} mA  {bar:<30}", end="", flush=True)
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n\nStopped by user.")

finally:
    cleanup()
    print("GPIO and SPI released.")
