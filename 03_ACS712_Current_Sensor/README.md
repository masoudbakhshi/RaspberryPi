# 03: ACS712 Current Sensor

**Author:** Masoud Bakhshi  
**Level:** Intermediate  
**Topic:** Analog current sensing, differential ADC measurement, SPI communication

---

## What this does

The ACS712 is a Hall-effect current sensor that outputs an analog voltage proportional to the current flowing through it. Since the Raspberry Pi has no analog inputs, this voltage is fed into the **Waveshare ADS1263 ADC HAT** (a 32-bit precision ADC) which reads it over SPI.

The script measures the current through an LED circuit in real time and displays it in the terminal.

---

## How the ACS712 works

The ACS712 passes the load current through a copper conductor. The magnetic field produced by that current is detected by a Hall-effect sensor, which converts it to an output voltage:

```
Output voltage = VCC/2 + (Current x Sensitivity)
               = 2.5V  + (I x 0.1 V/A)     <- for the 20A version
```

At zero current the output sits at exactly **2.5 V** (half of VCC). Positive current raises it, negative current lowers it.

> **Important: common GND required.** If the ACS712 module is powered from a separate supply, its GND must be connected to Pi GND. A floating GND causes the ADC to read the wrong reference voltage, producing current readings that are 3 to 4 times higher than the actual value. With all GNDs tied together, the datasheet sensitivity (100 mV/A for 20A) is correct with no further calibration needed.

---

## Why differential measurement?

At zero current, the ACS712 outputs 2.5 V, which equals the ADS1263 internal reference. Measuring single-ended (AIN vs GND) would put the signal at full scale even with no current flowing, leaving no headroom to measure positive current.

The solution is to build a **2.5 V reference** with a simple voltage divider (two equal resistors from 5V to GND) and connect it to AIN1. The ADS1263 then measures **AIN0 minus AIN1**, which is 0 V at zero current and scales by 100 mV per amp.

```
At 0 A  : AIN0 = 2.5V, AIN1 = 2.5V -> differential = 0V
At 15 mA: AIN0 = 2.5015V            -> differential = +1.5mV
```

---

## Hardware needed

| Item | Purpose |
|------|---------|
| Raspberry Pi (40-pin header) | Main board |
| Waveshare ADS1263 ADC HAT | 32-bit ADC that reads ACS712 analog output over SPI |
| AZDelivery ACS712 20A module | Current sensor, sensitivity verified at **0.100 V/A** (matches Allegro datasheet) |
| Standard LED + 330 ohm resistor | Test load whose current we measure |
| 2x 10 kohm resistors | Voltage divider that creates 2.5V reference for AIN1 |
| Wires | |

---

## Wiring

### ADS1263 HAT

Seat the HAT directly on the Raspberry Pi 40-pin GPIO header. It uses:

| Signal | BCM pin | Notes |
|--------|---------|-------|
| SCLK / MOSI / MISO / CE0 | 11 / 10 / 9 / 8 | Hardware SPI0 |
| RST | 18 | Reset |
| DRDY | **17** | Data ready. Do not use GPIO17 for anything else when this HAT is connected. |

### ACS712 module

```
Pi 5V (pin 2)  --> ACS712 VCC
Pi GND (pin 6) --> ACS712 GND

Current path (in series with LED circuit):
  GPIO27 (pin 13) -> 330 ohm -> LED(+) -> LED(-) -> ACS712 IP+ -> ACS712 IP- -> GND

ACS712 OUT  -->  ADS1263 AIN0 screw terminal
```

### Voltage divider (2.5V reference on AIN1)

```
Pi 5V -- 10 kohm --+-- ADS1263 AIN1 screw terminal
                   |
                10 kohm
                   |
                 GND
```

### Full wiring diagram

```
Pi 5V --------------------------------- ACS712 VCC
Pi GND -------------------------------- ACS712 GND

GPIO27 (pin 13)
    |
  [330 ohm]
    |
  LED (+)
  LED (-)
    |
  ACS712 IP+
  ACS712 IP-
    |
  GND

ACS712 OUT ---------------------------- ADS1263 AIN0

Pi 5V --- 10 kohm --- AIN1 --- 10 kohm --- GND
```

> **Note:** GPIO17 is used by the ADS1263 HAT as the DRDY (data ready) signal. The LED has moved from GPIO17 (used in tutorials 01 and 02) to **GPIO27** in this project.

---

## How to run

```bash
python3 acs712_current.py
```

The script will:
1. Initialise the ADS1263 over SPI
2. Run a zero-current calibration for 2 seconds (keep LED off during this)
3. Turn the LED on and display live current readings

Stop with **Ctrl+C**.

Expected output with a 330 ohm resistor and 3.3V GPIO output (~10 mA):
```
Current:  +10.34 mA  ##########
```

---

## How to read the result

```
Current = Differential voltage / ACS712 sensitivity
        = (AIN0 - AIN1) / 0.1 V/A
```

The 32-bit ADS1263 resolves changes of ~1.16 nV, which is far more resolution than needed to detect milliamp-level changes in LED current.

---

## Things to try

- Turn off the LED mid-run and watch the current drop to 0 mA. Modify the script to toggle the LED every 2 seconds to see it clearly.
- Change `SENSITIVITY` if using a different ACS712 variant: 5A uses 0.185 V/A, 20A uses 0.100 V/A, 30A uses 0.066 V/A.
- Increase the data rate in `REG_MODE2` from `0x04` (20 SPS) to `0x07` (100 SPS) for faster updates.

---

## Related tutorials in this repo

- [`01_Blink_LED`](../01_Blink_LED/) covers GPIO digital output basics
- [`02_PWM_Dimming`](../02_PWM_Dimming/) covers PWM LED brightness control
- [`04_MOSFET_Current_Control`](../04_MOSFET_Current_Control/) covers closed-loop PI current control with D4184 MOSFET and ACS712 feedback
