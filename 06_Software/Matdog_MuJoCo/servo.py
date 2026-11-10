"""ST3215-C018 actuator model of the MuJoCo twin (initial representation).

Per servo, in joint space (1 tick = 2*pi/4096 rad), two layers:

1. Internal profile (the algorithm of scripts/offline_motion/servo_plant.inc):
   a setpoint follows the latest goal of a SYNC_WRITE frame with a trapezoid
   bounded by the frame's Goal Speed (reg 46, step/s) and ACC (reg 41,
   x100 step/s^2, min with the register-85 ceiling; 0 = no ramp). The braking
   rule is servo_plant's: brake onto the goal at `brake`, the frame's ACC unless
   a lower braking capability is imposed (`brake_cap`, step/s^2). New goals take
   effect after `latency_s`.
2. Physical drive: P loop on the setpoint, duty = k (setpoint - q) in ticks
   (zero inside the dead band) + c * setpoint speed / omega0, clamped at
   TorqueLimit (RAM 48); DC motor torque
   tau_full(V) * duty - (tau_full / omega0) * qdot. In MuJoCo the back-EMF term
   is the joint damping (implicit), the rotor inertia the joint armature
   (tau_m = J omega0 / tau_full) and the gear friction a Coulomb frictionloss
   (fraction of tau_full). Torque OFF: no drive, no back-EMF (H-bridge open).

PARAMS lists every number with its status: MEASURED, FIRMWARE, VENDOR,
IDENTIFIED (fitted on measured bench data, unloaded) or HYPOTHESIS.
"""
import dataclasses
import math

import numpy as np

TICK = 2 * math.pi / 4096

# name: (nominal, status, source)
PARAMS = {
    "k_duty_per_tick": (8.4e-3, "MEASURED", "PresentLoad / position error at rest under load: RCA 28/10 8.4 permille/tick (LOWER 11/31/41), rc.11 STAND150 HOLD 8.4-8.7"),
    "torque_limit": (1.0, "FIRMWARE + HW LOG", "RAM 48 is not written by SimpleMotion (only register 85): power-up value = EEPROM MaxTorque 1000, read 1000 in rc.11 STAND150 and on 10/11 (1fbee7b). The handoff's 500 is the calibration layer's writeReviewedRamTorqueLimit: run as a sensitivity case. Bench: 500 clamps the duty, no-load speed 1550-1800 step/s"),
    "no_load_speed_step_s": (2829.0, "MEASURED", "bench QC V6.1, 11.1 V, no load (vendor 45 rpm at 12 V scales to 2839 step/s)"),
    "stall_torque_nm_12v": (30 * 0.0980665, "VENDOR", "Feetech ST-3215-C018 A/0: 30 kg.cm stall at 12 V, 2.7 A; R = 4.44 ohm agrees with the rc.11 HOLD current (D^2 V / I = 4.49 ohm)"),
    "tau_m_s": (0.025, "IDENTIFIED (unloaded)", "identify_bench.py on 624 bench moves (ACC 25-50, speed 0/600): best on all captures c 0.70, tau_m 25 ms, f 0.04 (RMSE median 1.42 ticks; P only 4.0); on the 20 normalized captures alone (the robot's EEPROM profile) c 0.60, tau_m 15 ms, f 0.03 (mean 1.21 ticks) = variant bench_alt, same twin results within a few %; 35-50 ms worse. J = tau_m * tau_full / omega0"),
    "friction_frac": (0.04, "IDENTIFIED (unloaded)", "same fit: 0.03-0.04 of the stall torque; static breakaway up to ~0.09 seen at rest, load dependence NOT MEASURED"),
    "ff_frac": (0.70, "IDENTIFIED (unloaded, <= 600 step/s)", "duty share proportional to the profile speed (sp_speed / omega0): the measured cruise lag needs it (600 step/s: 12.7-14.5 ticks at 208 permille, P only would give 25; 300 step/s: 1.3 ticks). Internal speed loop (regs 0x25, 0x27) NOT identified; walk speeds 1000-1950 step/s NOT measured"),
    "deadband_ticks": (1.0, "FIRMWARE", "EEPROM CWDead = CCWDead = 1 (bench static MAE 2-2.75 ticks)"),
    "latency_s": (0.0015, "HYPOTHESIS", "servo_plant draws 0-3 ms"),
    "brake_cap_step_s2": (math.inf, "HYPOTHESIS", "inf = the frame's ACC; bench unloaded arrival braking at ACC 50: p10 8 397, median 9 231 (MEASURED); loaded: NOT MEASURED"),
    "acc_ceiling": (220, "FIRMWARE", "register 85 written by the candidate (kAccCeilingTarget)"),
    "volt": (11.1, "MEASURED", "bus voltage of the bench and of the HW sessions (11.1-11.4 V); 9.6 V is the low-battery case of servo_plant"),
}


@dataclasses.dataclass
class Params:
    k_duty_per_tick: float = PARAMS["k_duty_per_tick"][0]
    torque_limit: float = PARAMS["torque_limit"][0]
    no_load_speed_step_s: float = PARAMS["no_load_speed_step_s"][0]
    stall_torque_nm_12v: float = PARAMS["stall_torque_nm_12v"][0]
    tau_m_s: float = PARAMS["tau_m_s"][0]
    friction_frac: float = PARAMS["friction_frac"][0]
    ff_frac: float = PARAMS["ff_frac"][0]
    deadband_ticks: float = PARAMS["deadband_ticks"][0]
    latency_s: float = PARAMS["latency_s"][0]
    brake_cap_step_s2: float = PARAMS["brake_cap_step_s2"][0]
    acc_ceiling: int = PARAMS["acc_ceiling"][0]
    volt: float = PARAMS["volt"][0]

    @property
    def tau_full(self):  # N*m at 100 % duty, motor stalled
        return self.stall_torque_nm_12v * self.volt / 12.0

    @property
    def omega0(self):  # rad/s at 100 % duty, no load
        return self.no_load_speed_step_s * self.volt / 11.1 * TICK

    @property
    def damping(self):  # back-EMF, N*m*s/rad, voltage independent
        return self.tau_full / self.omega0

    @property
    def armature(self):  # kg*m^2 at the output
        return self.tau_m_s * self.damping

    @property
    def friction(self):  # N*m
        return self.friction_frac * self.tau_full

    @property
    def kp(self):  # N*m/rad of the P loop before the clamp
        return self.k_duty_per_tick / TICK * self.tau_full


class Servos:
    """n servos: profile state and P-loop duty. Positions in radians."""

    def __init__(self, params, q0, unit=None):
        self.p = params
        n = len(q0)
        self.unit = np.ones(n) if unit is None else np.asarray(unit, float)  # per-unit capability factor
        self.sp = np.array(q0, float)          # profile setpoint
        self.v = np.zeros(n)
        self.goal = self.sp.copy()
        self.vmax = np.full(n, np.inf)
        self.amax = np.full(n, np.inf)
        self.bmax = np.full(n, np.inf)
        self.pending = None                    # (apply time, goal, vmax, amax, bmax)
        self.torque_on = np.ones(n, bool)

    def command(self, now, goal, speed_step_s, acc):
        """A SYNC_WRITE frame: 12 goals (rad) with one drive {speed, ACC}."""
        acc = min(int(acc), self.p.acc_ceiling)
        vmax = speed_step_s * TICK if speed_step_s > 0 else np.inf
        amax = acc * 100 * TICK if acc > 0 else np.inf
        bmax = min(amax, self.p.brake_cap_step_s2 * TICK)
        self.pending = (now + self.p.latency_s, np.array(goal, float), vmax, amax, bmax)

    def hold(self, q):
        """Torque ON at the current position (the firmware enables at the measured pose)."""
        self.sp[:] = q
        self.goal[:] = q
        self.v[:] = 0

    def step(self, now, dt):
        if self.pending is not None and now >= self.pending[0]:
            _, self.goal[:], vmax, amax, bmax = self.pending
            self.vmax[:], self.amax[:], self.bmax[:] = vmax, amax, bmax
            self.pending = None
        d = self.goal - self.sp
        direction = np.where(d > 0, 1.0, -1.0)
        toward = self.v * direction > 0
        stop_dist = self.v * self.v / (2 * self.bmax)
        braking = toward & (np.abs(d) <= stop_dist)
        accel = np.where(self.v * direction < 0, self.bmax, self.amax)
        v = np.where(braking, self.v - direction * self.bmax * dt,
                     np.clip(self.v + direction * accel * dt, -self.vmax, self.vmax))
        v = np.where(np.isinf(self.amax) & ~braking, direction * np.minimum(self.vmax, np.abs(d) / dt), v)
        stepped = v * dt
        arrive = (np.abs(stepped) >= np.abs(d)) & (v * direction > 0)
        arrive |= d == 0
        self.sp = np.where(arrive, self.goal, self.sp + stepped)
        self.v = np.where(arrive, 0.0, v)

    def duty(self, q):
        e = (self.sp - q) / TICK
        d = np.where(np.abs(e) <= self.p.deadband_ticks, 0.0, self.p.k_duty_per_tick * e)
        d = d + self.p.ff_frac * self.v / self.p.omega0
        d = np.clip(d, -self.p.torque_limit, self.p.torque_limit)
        return np.where(self.torque_on, d, 0.0)

    def torque(self, q):
        """Drive torque (N*m) without the back-EMF term (MuJoCo joint damping)."""
        return self.duty(q) * self.p.tau_full * self.unit


def apply_to_model(model, params, dofs, unit=None, torque_on=None):
    """Write the motor's armature, back-EMF damping and friction on the 12 joint dofs.
    `unit` scales a servo's torque constant (drive and back-EMF), not its inertia
    or its gear friction."""
    unit = np.ones(len(dofs)) if unit is None else np.asarray(unit, float)
    on = np.ones(len(dofs), bool) if torque_on is None else np.asarray(torque_on, bool)
    model.dof_armature[dofs] = params.armature
    model.dof_damping[dofs] = np.where(on, params.damping * unit, 0.0)
    model.dof_frictionloss[dofs] = params.friction
