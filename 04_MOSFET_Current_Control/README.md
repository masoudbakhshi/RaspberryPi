# 04: MOSFET PWM Closed-Loop Current Controller

**Author:** Masoud Bakhshi  
**Level:** Intermediate to Advanced  
**Topic:** Closed-loop control, PI controller, MOSFET switching, current feedback

---

## What this does

This project uses a D4184 MOSFET module to switch a 12V halogen lamp load via PWM, while the ACS712 current sensor continuously measures the actual current. A PI controller compares the measured current to a 1A target and automatically adjusts the PWM duty cycle to keep the current stable at that setpoint.

This is the foundation of real-world power electronics control: variable-speed motor drives, battery chargers, and LED drivers all work on this same principle.

---

## How the PI controller works

```
            setpoint (1A)
                |
                v
          [+] error = setpoint - measured
           |
      +----|----+
      |         |
   Kp x e    Ki x integral(e)
      |         |
      +----|----+
           |
           v
       duty cycle (0-100%)
           |
           v
       D4184 MOSFET
           |
           v
       Lamp current
           |
    [ACS712 feedback]-----> measured current
```

- **Proportional term (Kp):** reacts immediately to error. Larger Kp = faster response, but too high causes oscillation.
- **Integral term (Ki):** accumulates error over time and eliminates the steady-state offset that proportional alone leaves behind.
- **Anti-windup clamp:** prevents the integral from growing unbounded when the output is saturated.

---

## Hardware needed

| Item | Purpose |
|------|---------|
| Raspberry Pi (40-pin header) | Main board |
| Waveshare ADS1263 ADC HAT | Reads ACS712 analog output over SPI |
| AZDelivery ACS712 20A module | Current sensor in the lamp circuit |
| DollaTek D4184 MOSFET module | Low-side switching of the 12V lamp load |
| G4 Halogen Bulbs 12V 20W | Load (4 lamps used in this setup) |
| 12V power supply (HW-140) | Powers the lamp circuit |

---

## Wiring

```
Raspberry Pi GPIO23 (pin 16) ----> D4184 IN (MOSFET gate control)

12V supply (+)
    |
  [Lamps]
    |
  ACS712 IP+
  ACS712 IP-
    |
  D4184 OUT (drain-source connection)
    |
  GND (common with Pi GND and 12V supply GND)

ACS712 OUT  ----> ADS1263 AIN0 screw terminal
Pi 3.3V     ----> ADS1263 AIN1 screw terminal  (reference)

ADS1263 HAT GPIO usage:
  DRDY -> GPIO17  |  RST -> GPIO18  |  SPI0 -> GPIO 8/9/10/11
```

> All GND connections must be tied together: Pi GND, 12V supply GND, and D4184 GND.

---

## How to run

```bash
python3 current_control.py
```

The script will:
1. Calibrate the zero-current baseline (keep lamps off for 1.5 seconds)
2. Pulse the MOSFET briefly to detect current polarity
3. Ask you to enter the reference current in amps
4. Start the PI control loop and hold the current at your setpoint

```
Enter reference current in amps (0.1 to 18.0): 1.5
```

Live output format:
```
  Time    Current    Error    Duty  Bar
--------------------------------------------------------------
   0.4s   +0.000 A  +1.500 A  30.0%
   5.2s   +0.872 A  +0.628 A  15.3%  ████████████
  10.0s   +1.498 A  +0.002 A  19.1%  ███████████████████ |
  12.0s   +1.501 A  -0.001 A  18.9%  ████████████████████|
```

Stop with **Ctrl+C**. The MOSFET turns off immediately on exit.

Valid range: **0.1 A to 18.0 A** (ACS712 20A sensor with 2A safety margin).

---

## Tuning the controller

The gains Kp and Ki at the top of the script can be adjusted:

| Symptom | Action |
|---------|--------|
| Current reaches setpoint slowly | Increase Kp |
| Current oscillates around setpoint | Decrease Kp |
| Steady-state error remains after settling | Increase Ki |
| Current overshoots repeatedly | Decrease Ki |

---

## Things to try

- Change `TARGET_A` to a different value (0.5, 2.0) to target different currents.
- Set `Kp = 0` to see pure integral control (slow but no overshoot).
- Set `Ki = 0` to see pure proportional control (fast but leaves steady-state error).
- Log the time, current, and duty cycle to a CSV file for plotting.

---

## Related tutorials in this repo

- [`01_Blink_LED`](../01_Blink_LED/) covers GPIO digital output basics
- [`02_PWM_Dimming`](../02_PWM_Dimming/) covers PWM LED brightness control
- [`03_ACS712_Current_Sensor`](../03_ACS712_Current_Sensor/) covers current measurement with ACS712 and ADS1263
