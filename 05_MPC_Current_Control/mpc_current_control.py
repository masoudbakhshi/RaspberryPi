# =============================================================================
# Closed-Loop MPC Current Controller: Raspberry Pi Tutorial
# Author  : Masoud Bakhshi
# Date    : 2026-05-16
# Hardware: Raspberry Pi + Waveshare ADS1263 HAT + ACS712 20A
#           + DollaTek D4184 MOSFET Module + G4 Halogen Lamps (12V)
# =============================================================================
#
# Wiring: (identical to 04_MOSFET_Current_Control)
#   MOSFET control : GPIO23 (pin 16)  ->  D4184 IN
#   Lamp circuit   : 12V -> Lamps -> ACS712 IP+ -> ACS712 IP- -> D4184 OUT -> GND
#   ACS712 signal  : ACS712 OUT  ->  ADS1263 AIN0
#   AIN1 reference : Pi 3.3V (pin 1)  ->  ADS1263 AIN1
#   ADS1263 RST    : GPIO18  |  ADS1263 DRDY: GPIO17
#
# MOSFET safety:
#   GPIO23 boot-configured LOW via /boot/firmware/config.txt (gpio=23=op,dl).
#   SIGINT / SIGTERM / SIGHUP handlers guarantee MOSFET off on any exit.
#
# Controller: Finite-Horizon Constrained MPC (Model Predictive Control)
#
#   Startup sequence:
#     1. Zero-current calibration  (ADC baseline, MOSFET off)
#     2. Polarity detection         (25 % open-loop pulse)
#     3. Online system identification  (step test -> first-order model)
#     4. MPC gain matrix precomputation  (F, Phi, K_mpc  -- done once)
#     5. Setpoint entry
#     6. Closed-loop MPC at ~480 Hz
#
#   Plant model -- discrete-time first-order, identified at startup:
#     x[k+1] = A * x[k]  +  B * u[k]       (x = current, u = duty %)
#
#   Augmented velocity-form state  z = [x ; u_prev]:
#     z[k+1] = A_aug * z[k]  +  B_aug * du[k]    (du = duty increment)
#     y[k]   = C_aug * z[k]
#
#   A_aug = [[A, B],      B_aug = [[B],      C_aug = [1, 0]
#             [0, 1]]               [1]]
#
#   The u_prev state acts as a discrete integrator: steady-state error is
#   zero by construction, with no explicit anti-windup logic needed.
#
#   Prediction over Np steps:
#     Y_pred = F * z[k]  +  Phi * DeltaU
#     F[i]      = C_aug * A_aug^(i+1)           (free response,  Np x 2)
#     Phi[i,j]  = C_aug * A_aug^(i-j) * B_aug   (forced response, Np x Nc)
#
#   Cost minimised at every step:
#     J = ||Y_ref - Y_pred||^2_Q  +  ||DeltaU||^2_R
#
#   Unconstrained solution (precomputed gain K_mpc, Nc x Np):
#     K_mpc   = (Phi^T Q Phi + R)^-1  Phi^T Q
#     DeltaU* = K_mpc * (Y_ref - F * z)
#
#   Constraint projection (exact for box constraints):
#     du[0] = clip(DeltaU*[0],  -DU_MAX, +DU_MAX)
#     u     = clip(u_prev + du[0],  U_MIN,  U_MAX)
#
#   Only the first control move is applied (receding horizon).
# =============================================================================

import spidev
import lgpio
import numpy as np
import time
import signal
import atexit
import sys

# ── Hardware pins ─────────────────────────────────────────────────────────────
MOSFET_PIN = 23
RST_PIN    = 18
DRDY_PIN   = 17

PWM_FREQ   = 500          # Hz

# ── ADS1263 SPI ───────────────────────────────────────────────────────────────
SPI_BUS    = 0
SPI_DEVICE = 0
SPI_SPEED  = 2_000_000

CMD_RESET  = 0x06
CMD_START1 = 0x08
CMD_STOP1  = 0x0A
CMD_RDATA1 = 0x12
CMD_WREG   = 0x40

REG_MODE2  = 0x04
REG_INPMUX = 0x05
REG_REFMUX = 0x0E

# ── ACS712 20A ────────────────────────────────────────────────────────────────
SENSITIVITY = 0.339       # V/A  (empirically calibrated: module op-amp gain ~3.4x over datasheet 100 mV/A)
VREF        = 2.5         # internal ADC reference (V)

# ── ADC averaging ─────────────────────────────────────────────────────────────
AVERAGE_N  = 5            # 5 samples @ 2400 SPS  ->  ~2.1 ms / reading  ->  ~480 Hz loop

# ── MPC tuning ───────────────────────────────────────────────────────────────
NP     = 30               # prediction horizon (steps)  -- looks ~63 ms ahead
NC     = 10               # control horizon (free control increments)
Q_W    = 100.0            # output tracking weight  (larger -> tighter tracking)
R_W    = 0.5              # control-increment weight (larger -> smoother, slower)
DU_MAX = 10.0             # max duty-cycle change per step (%)
U_MIN  = 0.0              # duty-cycle lower bound (%)
U_MAX  = 100.0            # duty-cycle upper bound (%)

# ── System-identification settings ───────────────────────────────────────────
SYSID_DUTY    = 40.0      # open-loop step duty cycle for plant ID (%)
SYSID_SAMPLES = 200       # ADC readings collected during step test
SYSID_TAU_MIN = 3e-3      # minimum allowed tau (s) -- prevents near-singular matrices

MAX_CURRENT_A = 18.0      # ACS712 20 A module, 2 A safety margin

# =============================================================================
# Safe shutdown  -- registered BEFORE hardware init so any crash turns MOSFET off
# =============================================================================

h   = None   # set before GPIO init so shutdown handler can check
spi = None

_shutdown_done = False


def safe_shutdown(signum=None, frame=None):
    global _shutdown_done
    if _shutdown_done:
        return
    _shutdown_done = True
    try:
        if h is not None:
            lgpio.tx_pwm(h, MOSFET_PIN, 0, 0)
            lgpio.gpio_write(h, MOSFET_PIN, 0)
            lgpio.gpiochip_close(h)
    except Exception:
        pass
    try:
        if spi is not None:
            spi.xfer2([CMD_STOP1])
            spi.close()
    except Exception:
        pass
    print("\nMOSFET off. Shutdown complete.")
    sys.exit(0)


signal.signal(signal.SIGINT,  safe_shutdown)
signal.signal(signal.SIGTERM, safe_shutdown)
signal.signal(signal.SIGHUP,  safe_shutdown)
atexit.register(safe_shutdown)   # catches normal interpreter exit too


# =============================================================================
# Hardware initialisation
# =============================================================================

spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = SPI_SPEED
spi.mode = 0b01

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, RST_PIN, 1)
lgpio.gpio_claim_input(h, DRDY_PIN)
lgpio.gpio_claim_output(h, MOSFET_PIN, 0)   # MOSFET off at startup


def _write_reg(reg, val):
    spi.xfer2([CMD_WREG | reg, 0x00, val])


def _wait_drdy(timeout=3.0):
    t0 = time.time()
    while lgpio.gpio_read(h, DRDY_PIN) == 1:
        if time.time() - t0 > timeout:
            raise TimeoutError("ADS1263 DRDY timeout. Check HAT seating and SPI.")
        time.sleep(0.00005)


def _read_adc1_raw():
    _wait_drdy()
    data = spi.xfer2([CMD_RDATA1, 0x00, 0x00, 0x00, 0x00, 0x00])
    raw  = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5]
    if raw >= 0x80000000:
        raw -= 0x100000000
    return raw


def read_current(zero_v):
    samples = [_read_adc1_raw() for _ in range(AVERAGE_N)]
    v = (sum(samples) / AVERAGE_N / 0x7FFFFFFF) * VREF
    return (v - zero_v) / SENSITIVITY


def set_duty(duty):
    lgpio.tx_pwm(h, MOSFET_PIN, PWM_FREQ, max(0.0, min(100.0, duty)))


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
    _write_reg(REG_MODE2,  0x0A)   # 2400 SPS, PGA gain 1x
    _write_reg(REG_INPMUX, 0x01)   # AIN0(+) vs AIN1(-)
    _write_reg(REG_REFMUX, 0x00)   # internal 2.5 V reference
    spi.xfer2([CMD_START1])
    time.sleep(0.5)



# =============================================================================
# Online system identification
# =============================================================================

def identify_plant(zero_v):
    """
    Apply an open-loop duty step, record current samples, then fit the
    discrete first-order model  x[k+1] = A*x[k] + B*u  using least squares.

    Because the lamp load is primarily resistive, the electrical time constant
    is much shorter than the sampling period.  The identified A captures the
    effective dynamics seen through the ADC averaging filter.

    Returns
    -------
    A_id   : discrete-time pole  (dimensionless, in (0, 1))
    B_id   : discrete-time input gain  (A per % duty)
    K_dc   : DC gain = B_id / (1 - A_id)  (A per % duty, steady-state)
    tau_s  : time constant  (s)
    Ts_eff : measured sampling period  (s)
    """
    print(f"  Applying {SYSID_DUTY:.0f}% duty step...")
    set_duty(SYSID_DUTY)
    time.sleep(0.05)   # let PWM settle before collecting data

    t_stamps = []
    samples  = []
    for _ in range(SYSID_SAMPLES):
        t_stamps.append(time.time())
        samples.append(read_current(zero_v))

    mosfet_off()

    t_stamps = np.array(t_stamps)
    samples  = np.array(samples)
    Ts_eff   = float(np.mean(np.diff(t_stamps)))

    # Least-squares fit:  x[k+1] = A * x[k]  +  B * u_sysid
    # Build regression matrix [x[k], u_sysid * ones]  ->  solve for [A, B]
    N    = len(samples)
    y_ls = samples[1:]                                              # (N-1,)
    X_ls = np.column_stack([samples[:-1],
                             np.full(N - 1, SYSID_DUTY)])          # (N-1, 2)
    theta, _, _, _ = np.linalg.lstsq(X_ls, y_ls, rcond=None)
    A_id, B_id = float(theta[0]), float(theta[1])

    # Clamp A to (0, 1) -- must be stable and causal
    A_id = float(np.clip(A_id, 0.0, 0.9999))

    # DC gain and time constant from identified discrete model
    K_dc  = B_id / max(1.0 - A_id, 1e-6)
    tau_s = float(-Ts_eff / np.log(max(A_id, 1e-9)))
    tau_s = max(tau_s, SYSID_TAU_MIN)

    if abs(K_dc) < 1e-5:
        raise ValueError(
            "System ID failed: DC gain is near zero. "
            "Check that the 12 V supply is on and the wiring is correct."
        )

    return A_id, B_id, K_dc, tau_s, Ts_eff


# =============================================================================
# MPC gain matrix precomputation (runs once after system ID)
# =============================================================================

def build_mpc(A_id, B_id, Ts_eff):
    """
    Construct all MPC matrices from the identified discrete model.

    Augmented velocity-form (z = [x ; u_prev]):
        A_aug = [[A, B],    B_aug = [[B],    C_aug = [1, 0]
                  [0, 1]]             [1]]

    Prediction:
        Y = F * z  +  Phi * DeltaU

    Precomputed gain (applied online every sample):
        K_mpc = (Phi^T Q Phi + R)^-1  Phi^T Q        (NC x NP)

    Returns
    -------
    A_aug, B_aug, C_aug : augmented system matrices
    F                   : free-response matrix   (NP x 2)
    Phi                 : forced-response matrix  (NP x NC)
    K_mpc               : precomputed MPC gain    (NC x NP)
    """
    A, B = A_id, B_id

    # Augmented system
    A_aug = np.array([[A, B],
                      [0.0, 1.0]])
    B_aug = np.array([B, 1.0])    # (2,) -- 1-D for clean dot products
    C_aug = np.array([1.0, 0.0])  # (2,)

    # Free-response matrix F  (NP x 2):  F[i] = C_aug @ A_aug^(i+1)
    F    = np.zeros((NP, 2))
    Apow = np.eye(2)
    for i in range(NP):
        Apow = Apow @ A_aug
        F[i] = C_aug @ Apow

    # Forced-response matrix Phi  (NP x NC):
    #   Phi[i, j] = C_aug @ A_aug^(i-j) @ B_aug   for i >= j, else 0
    # Precompute A_aug^k @ B_aug for k = 0, 1, ..., NP-1
    AB_pow = np.zeros((NP, 2))
    AB_pow[0] = B_aug                          # k = 0 : A^0 * B = B
    for k in range(1, NP):
        AB_pow[k] = A_aug @ AB_pow[k - 1]     # k : A^k * B

    Phi = np.zeros((NP, NC))
    for i in range(NP):
        for j in range(min(i + 1, NC)):
            Phi[i, j] = C_aug @ AB_pow[i - j]

    # Diagonal weight matrices
    Q_bar = Q_W * np.eye(NP)
    R_bar = R_W * np.eye(NC)

    # Precomputed MPC gain:  K_mpc = (Phi^T Q Phi + R)^-1  Phi^T Q
    PhiT_Q = Phi.T @ Q_bar
    K_mpc  = np.linalg.solve(PhiT_Q @ Phi + R_bar, PhiT_Q)

    return A_aug, B_aug, C_aug, F, Phi, K_mpc


# =============================================================================
# Single MPC control step  (called ~480 times per second)
# =============================================================================

def mpc_step(z, ref, F, K_mpc, u_prev):
    """
    Compute one MPC control move (receding horizon).

    Parameters
    ----------
    z      : augmented state [i_meas ; u_prev]  (2-vector)
    ref    : current setpoint  (A)
    F      : free-response matrix  (NP x 2)
    K_mpc  : precomputed gain  (NC x NP)
    u_prev : duty cycle applied at the previous step  (%)

    Returns
    -------
    u_next     : new duty cycle to apply  (%)
    du_applied : actual duty increment after constraint projection  (%)
    """
    Y_ref = np.full(NP, ref)                     # constant reference trajectory
    E     = Y_ref - F @ z                         # prediction-error vector  (NP,)
    DU    = K_mpc @ E                             # unconstrained increments  (NC,)

    # Project first increment onto constraints (exact for box constraints)
    du0    = float(np.clip(DU[0], -DU_MAX, DU_MAX))
    u_next = float(np.clip(u_prev + du0, U_MIN, U_MAX))

    return u_next, u_next - u_prev


# =============================================================================
# MPC closed-loop control loop
# =============================================================================

def run_mpc(zero_v, target_abs, initial_duty, F, K_mpc):
    u_prev = initial_duty
    set_duty(u_prev)
    time.sleep(0.3)
    for _ in range(4):
        _read_adc1_raw()   # flush stale ADC pipeline samples

    t_prev  = time.time()
    t_start = t_prev

    print(f"\nTarget: {target_abs:+.3f} A   "
          f"Np={NP}  Nc={NC}  Q={Q_W}  R={R_W}  ΔDuty_max=±{DU_MAX:.0f}%")
    print(f"{'Time':>6}  {'Current':>9}  {'Error':>8}  {'ΔDuty':>7}  {'Duty':>6}  Bar")
    print("─" * 68)

    while True:
        i_meas = read_current(zero_v)
        i_abs  = abs(i_meas)
        error  = target_abs - i_abs

        z = np.array([i_abs, u_prev])

        u_next, du_applied = mpc_step(z, target_abs, F, K_mpc, u_prev)
        set_duty(u_next)
        u_prev = u_next

        t_now   = time.time()
        elapsed = t_now - t_start
        t_prev  = t_now

        bar    = '█' * min(int(i_abs / max(target_abs, 0.01) * 20), 20)
        marker = '|' if i_abs >= target_abs * 0.98 else ''

        print(
            f"{elapsed:6.1f}s  "
            f"{i_meas:+7.3f} A  "
            f"{error:+6.3f} A  "
            f"{du_applied:+6.2f}%  "
            f"{u_next:5.1f}%  "
            f"{bar:<20}{marker}",
            flush=True
        )


# =============================================================================
# Main
# =============================================================================

print("=" * 68)
print("  MPC Closed-Loop Current Controller")
print("  Masoud Bakhshi")
print("=" * 68)

ads_init()
print("ADS1263 initialised at 2400 SPS.\n")

# ── Step 1: Zero-current calibration ─────────────────────────────────────────
print("Calibrating zero-current baseline (MOSFET off)...")
time.sleep(1.5)
zero_v = sum(
    [(sum([_read_adc1_raw() for _ in range(AVERAGE_N)]) / AVERAGE_N / 0x7FFFFFFF) * VREF
     for _ in range(5)]
) / 5
print(f"Zero offset: {zero_v * 1000:.2f} mV\n")

# ── Step 2: Polarity detection ────────────────────────────────────────────────
print("Detecting current polarity...")
set_duty(25)
time.sleep(0.4)
i_check  = read_current(zero_v)
mosfet_off()
polarity = -1 if i_check < 0 else 1
print(f"Measured at 25% duty: {i_check:+.3f} A  "
      f"(polarity: {'negative' if polarity < 0 else 'positive'})\n")

# ── Step 3: Online system identification ──────────────────────────────────────
print("Identifying plant model from step response (keep lamps connected)...")
try:
    A_id, B_id, K_dc, tau_s, Ts_eff = identify_plant(zero_v)
except ValueError as e:
    print(f"\nSystem identification error: {e}")
    safe_shutdown()

print(f"  Identified  A  = {A_id:.5f}  (discrete pole)")
print(f"  Identified  B  = {B_id:.5f}  A/%")
print(f"  DC gain     K  = {K_dc:.5f}  A/% duty")
print(f"  Time const  τ  = {tau_s * 1000:.2f} ms  "
      f"(sampling Ts = {Ts_eff * 1000:.2f} ms)\n")

# ── Step 4: Build MPC matrices (offline, done once) ───────────────────────────
print("Building MPC prediction matrices...")
A_aug, B_aug, C_aug, F, Phi, K_mpc = build_mpc(A_id, B_id, Ts_eff)
print(f"  Prediction horizon   Np = {NP}  ({NP * Ts_eff * 1000:.1f} ms ahead)")
print(f"  Control horizon      Nc = {NC}")
print(f"  Tracking weight      Q  = {Q_W}")
print(f"  Increment weight     R  = {R_W}")
print(f"  Rate limit        ΔDuty = ±{DU_MAX:.0f}% per step\n")

# ── Step 5: Setpoint entry ────────────────────────────────────────────────────
while True:
    try:
        raw = input(f"Enter reference current in amps (0.1 to {MAX_CURRENT_A}): ").strip()
        TARGET_A = float(raw)
        if not (0.1 <= TARGET_A <= MAX_CURRENT_A):
            print(f"  Out of range. Enter a value between 0.1 and {MAX_CURRENT_A}.")
            continue
        break
    except ValueError:
        print("  Invalid input. Enter a number, e.g. 1.5")

# Initial duty estimate from identified DC gain (capped at 70% for safety)
initial_duty = float(np.clip(TARGET_A / max(abs(K_dc), 1e-6), U_MIN, 70.0))
print(f"Initial duty estimate: {initial_duty:.1f}%  (from K_dc)\n")

print(f"Starting MPC control. Target: {polarity * TARGET_A:+.3f} A")
print("Press Ctrl+C to stop.\n")

try:
    run_mpc(zero_v, TARGET_A, initial_duty, F, K_mpc)
except Exception as e:
    print(f"\nError: {e}")
finally:
    safe_shutdown()
