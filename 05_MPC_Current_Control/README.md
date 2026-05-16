# 05: Closed-Loop MPC Current Controller

**Author:** Masoud Bakhshi  
**Level:** Advanced  
**Topic:** Model Predictive Control, constrained optimization, online system identification

---

## What this does

This project controls the same lamp-and-MOSFET circuit from project 04 using a **Finite-Horizon Constrained MPC** instead of a PI controller. Before entering the control loop the script identifies the plant model from a live step test, precomputes the optimal gain matrix offline, and then runs a full constrained optimization at every sample (~480 Hz) in under a millisecond of Python time.

The result is a controller that:
- **Requires no manual gain tuning** — the model drives the response
- **Enforces hard constraints** on both the duty cycle and its rate of change
- **Has zero steady-state error by construction** through the velocity (incremental) formulation
- **Looks ahead 63 ms** into the predicted future at every step

---

## How MPC works

### Plant model

The current response is modelled as a first-order discrete-time system:

```
x[k+1] = A * x[k]  +  B * u[k]
y[k]   = x[k]

A = exp(-Ts / τ)          discrete pole
B = K · (1 − A)           discrete input gain
K = steady-state A/% duty  (identified online)
τ = time constant          (identified online)
```

### Velocity (incremental) formulation

Instead of optimising the duty cycle `u` directly, the MPC optimises the **increment** `Δu = u[k] − u[k−1]`. This transforms the state to an augmented vector `z = [x ; u_prev]`:

```
z[k+1] = A_aug · z[k]  +  B_aug · Δu[k]
y[k]   = C_aug · z[k]

A_aug = [[A, B],      B_aug = [[B],      C_aug = [1, 0]
          [0, 1]]               [1]]
```

The `u_prev` state acts as a discrete integrator. Any constant disturbance or model mismatch accumulates in `u_prev` until the error is driven to zero — offset-free tracking without explicit anti-windup logic.

### Prediction over the horizon

Stacking Np predicted outputs into a vector Y_pred:

```
Y_pred = F · z[k]  +  Phi · ΔU

F[i]      = C_aug · A_aug^(i+1)           free response   (Np × 2)
Phi[i,j]  = C_aug · A_aug^(i−j) · B_aug   forced response (Np × Nc)  for i ≥ j
```

### Cost function

At every sample the MPC minimises:

```
J = ‖Y_ref − Y_pred‖²_Q  +  ‖ΔU‖²_R

Q = Q_W · I   tracking weight  (larger → faster, tighter)
R = R_W · I   increment weight (larger → smoother, slower)
```

### Unconstrained solution and precomputed gain

Differentiating J with respect to ΔU and setting to zero gives:

```
ΔU* = (Phi^T Q Phi + R)^-1  Phi^T Q  ·  (Y_ref − F·z)
    = K_mpc · E

K_mpc  (Nc × Np) : computed once after system ID, never changes
E      (Np,)     : tracking error over the prediction horizon
```

The online computation per sample is **one matrix-vector multiply** — no iterative solver.

### Constraint projection

After computing ΔU* the first increment is projected onto the box constraints:

```
du[0]  = clip(ΔU*[0],  −DU_MAX,  +DU_MAX)   rate-of-change limit
u[k]   = clip(u[k−1] + du[0],  0,  100)      duty-cycle bounds
```

Only the first move is applied and the optimisation is repeated at the next sample (receding horizon principle).

---

## Startup sequence

```
1. ADC calibration    — measures zero-current voltage offset (MOSFET off, 1.5 s)
2. Polarity detection — 25% open-loop pulse to determine current direction
3. System ID          — 40% open-loop step, collects 200 samples, fits A and B
                        via least squares on  x[k+1] = A·x[k] + B·u
4. MPC precomputation — builds F, Phi, K_mpc from identified model
5. Setpoint entry     — user enters target current in amps
6. Control loop       — ~480 Hz, receding-horizon MPC, live terminal display
```

---

## Hardware needed

Identical to [04_MOSFET_Current_Control](../04_MOSFET_Current_Control/). No changes to wiring.

| Item | Purpose |
|------|---------|
| Raspberry Pi (40-pin header) | Main board |
| Waveshare ADS1263 ADC HAT | Reads ACS712 analog output over SPI |
| AZDelivery ACS712 20A module | Current sensor — sensitivity calibrated to **0.339 V/A** (clone chip; datasheet value is 0.100 V/A — always verify with an ammeter) |
| DollaTek D4184 MOSFET module | Low-side switching of the 12V lamp load |
| G4 Halogen Bulbs 12V 20W | Load |
| 12V power supply (HW-140) | Powers the lamp circuit |

---

## Wiring

Identical to project 04. See [04_MOSFET_Current_Control/README.md](../04_MOSFET_Current_Control/README.md) for the full wiring diagram.

```
Raspberry Pi GPIO23 (pin 16) ----> D4184 IN
12V supply (+) -> Lamps -> ACS712 IP+ -> ACS712 IP- -> D4184 OUT -> GND
ACS712 OUT  ----> ADS1263 AIN0
Pi 3.3V     ----> ADS1263 AIN1  (differential reference)
ADS1263 HAT: DRDY -> GPIO17  |  RST -> GPIO18  |  SPI0 -> GPIO 8/9/10/11
```

---

## Dependencies

In addition to the standard library, `numpy` is required:

```bash
pip3 install numpy
```

`numpy` is also available via apt if pip is not set up:

```bash
sudo apt install python3-numpy
```

---

## How to run

```bash
python3 mpc_current_control.py
```

The startup sequence prints the identified model parameters before asking for a setpoint:

```
ADS1263 initialised at 2400 SPS.

Calibrating zero-current baseline (MOSFET off)...
Zero offset: 1247.83 mV

Detecting current polarity...
Measured at 25% duty: +0.612 A  (polarity: positive)

Identifying plant model from step response (keep lamps connected)...
  Applying 40% duty step...
  Identified  A  = 0.14823  (discrete pole)
  Identified  B  = 0.02471  A/%
  DC gain     K  = 0.02900  A/% duty
  Time const  τ  = 3.76 ms  (sampling Ts = 2.11 ms)

Building MPC prediction matrices...
  Prediction horizon   Np = 30  (63.3 ms ahead)
  Control horizon      Nc = 10
  Tracking weight      Q  = 100.0
  Increment weight     R  = 0.5
  Rate limit        ΔDuty = ±10% per step
```

Live output during control:

```
Target: +1.500 A   Np=30  Nc=10  Q=100.0  R=0.5  ΔDuty_max=±10%
  Time    Current    Error     ΔDuty    Duty   Bar
────────────────────────────────────────────────────────────────────
   0.4s  +0.000 A  +1.500 A   +10.00%  30.0%
   1.2s  +0.901 A  +0.599 A    +8.50%  44.1%  ████████████
   5.0s  +1.496 A  +0.004 A    +0.18%  51.7%  ███████████████████ |
   8.3s  +1.500 A  +0.000 A    +0.01%  51.7%  ████████████████████|
```

Stop with **Ctrl+C**. The MOSFET turns off immediately.

---

## MPC parameters and what they do

| Parameter | Default | Effect |
|-----------|---------|--------|
| `NP` | 30 | Prediction horizon. Longer → smoother, more computation per matrix build. |
| `NC` | 10 | Control horizon. Must be ≤ Np. Fewer free moves → more regularised. |
| `Q_W` | 100 | Tracking weight. Increase to reduce steady-state error and speed up convergence. |
| `R_W` | 0.5 | Increment penalty. Increase to smooth the duty cycle at the cost of slower response. |
| `DU_MAX` | 10 | Max duty-cycle step per sample (%). Limits slew rate regardless of Q/R. |
| `SYSID_DUTY` | 40 | Open-loop duty during plant identification. Must produce a measurable, safe current. |

> **Note:** Changing `NP`, `NC`, `Q_W`, or `R_W` only requires restarting the script — `K_mpc` is recomputed automatically from the fresh system ID at each run.

---

## Things to try

- **Compare with the PI controller.** Run project 04 at the same setpoint and observe how the settling time and duty-cycle trajectory differ.
- **Slow the response down** by raising `R_W` to 5.0. The `ΔDuty` column will show the MPC deliberately limiting its own aggressiveness.
- **Speed it up** by raising `Q_W` to 500. The controller will hit `DU_MAX` on every step during the transient.
- **Stress-test the constraint.** Set `DU_MAX = 2` and watch the smooth, ramp-like duty trajectory even during a large step change.
- **Log data** by redirecting output to a file (`python3 mpc_current_control.py | tee log.csv`), then parse it in Python or a spreadsheet to plot current vs time and compare the step response against the predicted trajectory.
- **Change setpoints mid-run.** Stop with Ctrl+C, restart, enter a different current, and compare how quickly the MPC settles versus the PI controller.

---

## Related tutorials in this repo

- [`04_MOSFET_Current_Control`](../04_MOSFET_Current_Control/) — same hardware, PI controller — a good baseline to compare against
- [`03_ACS712_Current_Sensor`](../03_ACS712_Current_Sensor/) — how the ACS712 and ADS1263 measure current
- [`02_PWM_Dimming`](../02_PWM_Dimming/) — PWM basics
- [`01_Blink_LED`](../01_Blink_LED/) — GPIO digital output basics
