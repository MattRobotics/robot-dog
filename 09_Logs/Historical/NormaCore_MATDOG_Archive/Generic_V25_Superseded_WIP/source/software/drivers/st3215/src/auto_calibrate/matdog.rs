//! MATDOG-specific, data-driven, RAM-only ST3215 mechanical end-stop calibrator.
//!
//! The full LF path is one explicitly armed, persistent state machine. The
//! active probing joint is the only joint whose target advances during contact
//! search. Every other canonical motor has a state-specific role and is
//! monitored until the mandatory verified global torque-OFF cleanup.

use crate::protocol::{self, RamRegister};
use crate::st3215_proto::{CommandResult, InferenceState, TxEnvelope};
use crate::state::{CalibrationStatus, ST3215BusCommunicator};
use bytes::Bytes;
use log::{error, info};
use prost::Message;
use std::collections::BTreeSet;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use tokio::sync::watch;
use tokio::time::{Duration, Instant};

type DynError = Box<dyn std::error::Error + Send + Sync>;

pub const MATDOG_MOTOR_IDS: [u8; 12] = [11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43];
pub(crate) const MATDOG_ARM_ENV: &str = "MATDOG_NATIVE_CALIBRATOR_ARM";

const HOME_TICK: u16 = 2048;
const TICKS_PER_REVOLUTION: i32 = 4096;
const GUARD_OVERSHOOT_TICKS: u16 = 64;
const BASELINE_TRAVEL_TICKS: u16 = 64;
// V38: all six LF contacts have completed supervised hardware passes.  The
// original motion envelope was deliberately slow for first contact discovery;
// use a still bounded but materially faster production-calibration envelope.
// The hard-current/status abort, model corridors and mechanical guards remain
// unchanged.  TorqueLimit 500 is still only half of the ST3215 command range.
const TORQUE_LIMIT: u16 = 500;
const GOAL_SPEED: u16 = 160;
const ACCELERATION: u8 = 8;
const COARSE_STEP_TICKS: u16 = 64;
const FINE_STEP_TICKS: u16 = 8;
const BACKOFF_TICKS: u16 = 96;
const STATIC_TOLERANCE_TICKS: u16 = 10;
// V36 hardware evidence on LF HIP MAX showed a normal 13-tick directional
// settle at target=1968, present=1981 before the model contact corridor.
// Permit one bounded coarse-step continuation outside the corridor only.
// The strict 10-tick detector gate remains active inside every corridor.
const OUTSIDE_CORRIDOR_SETTLE_TOLERANCE_TICKS: u16 = 16;
// Keep the outer approach decision consistent with the detector. The V24 M13
// fine pass showed a healthy 13-tick directional settle; errors beyond this
// global 16-tick floor still enter plateau confirmation or fail closed.
const PROBE_TRACKING_ERROR_FLOOR_TICKS: u16 = OUTSIDE_CORRIDOR_SETTLE_TOLERANCE_TICKS;
// The active probe can settle a few ticks farther from digital home under
// geometry-prerequisite load and gearbox backlash. Keep this tolerance
// separate so prerequisite drift and contact tracking remain at 10 ticks.
const PROBE_HOME_TOLERANCE_TICKS: u16 = 16;
// During reverse recovery the torque-off probe can be passively displaced by
// the upper-link motion. Keep this bounded separately, then actively re-home
// and verify the probe before final global torque OFF.
const PROBE_PASSIVE_RESTORE_DRIFT_TICKS: u16 = 32;
// A prerequisite observed at the digital-home endpoint may settle just beyond
// the 10-tick static gate before torque is enabled. Widen only that startup
// endpoint; the prerequisite target endpoint and all live hold checks remain
// at STATIC_TOLERANCE_TICKS.
const STARTUP_PREREQUISITE_HOME_SETTLE_TICKS: u16 = 16;
const STARTUP_HOME_RECOVERY_LIMIT_TICKS: u16 = 64;
const REPEATABILITY_TOLERANCE_TICKS: u16 = 16;
const BASELINE_MIN_SAMPLES: usize = 6;
const MINIMUM_CONTACT_TRAVEL_TICKS: u16 = 24;
const TARGET_STARTUP_SAMPLES: u8 = 4;
const CONTACT_SETTLE_WINDOW: Duration = Duration::from_millis(900);
const HARD_CURRENT_ABORT_RAW: u16 = 200;
const MAX_TEMPERATURE_LIMIT_ADDRESS: usize = 0x0D;
const EXPECTED_TEMPERATURE_LIMIT_C: u8 = 70;
const COMMAND_TIMEOUT: Duration = Duration::from_secs(5);
const TELEMETRY_TIMEOUT: Duration = Duration::from_secs(2);
const MAX_TELEMETRY_AGE: Duration = Duration::from_secs(3);
const MOTION_TIMEOUT: Duration = Duration::from_secs(12);
// Long MAX returns and +90-degree prerequisites can exceed the original fixed
// 12-second budget at the original GOAL_SPEED=80. Size the deadline from
// the commanded distance using a conservative half-speed floor for the V38
// GOAL_SPEED=160 envelope, retaining 12 seconds
// as the minimum for short movements and telemetry/settling overhead.
const MIN_EXPECTED_MOTION_TICKS_PER_SECOND: u64 = 80;
const MOTION_SETTLE_MARGIN: Duration = Duration::from_secs(5);

const LF_ALLOWED: [u8; 4] = [11, 12, 13, 42];
const RF_ALLOWED: [u8; 4] = [21, 22, 23, 32];
const RH_ALLOWED: [u8; 3] = [31, 32, 33];
const LH_ALLOWED: [u8; 3] = [41, 42, 43];

const HIP_MIN_DELTA: i16 = -512;
const HIP_MAX_DELTA: i16 = 512;
const UPPER_MIN_DELTA: i16 = -597;
const UPPER_MAX_DELTA: i16 = 1394;
const LOWER_MIN_DELTA: i16 = -1047;
const LOWER_MAX_DELTA: i16 = 427;
const UPPER_30_DELTA: i16 = 341;
const UPPER_90_DELTA: i16 = 1024;
const UPPER_85_DELTA: i16 = 967;
const LOWER_FOLDED_DELTA: i16 = -990;
const CONTACT_ACCEPTANCE_INNER_TICKS: u16 = 64;
const LF_HIP_SEQUENCE_ARM_VALUE: &str = "LF_HIP_M13_MIN_MAX";
const LF_FULL_SEQUENCE_ARM_VALUE: &str = "LF_LEG_STATE_MACHINE";
const ADAPTIVE_FINE_SCOUT_TICKS: u16 = 32;
// Fine approaches use smaller increments and may settle slightly before the
// coarse scout. A lag greater than one fine step is treated as a bounded
// friction/chamfer plateau, not as the final mechanical endpoint.
const FINE_CONTACT_SCOUT_LAG_TOLERANCE_TICKS: u16 = FINE_STEP_TICKS;
const AFFINE_SCALE_MIN_PERMILLE: u16 = 850;
const AFFINE_SCALE_MAX_PERMILLE: u16 = 1150;
const KINEMATIC_PLATEAU_SAMPLES: usize = 3;
const KINEMATIC_PLATEAU_POSITION_SPAN_TICKS: u16 = 3;
const MODEL_ZERO_ENDPOINT_CONSISTENCY_TICKS: u16 = 24;
const LF_CONTACT_WITNESS_TOLERANCE_TICKS: u16 = 24;
const MODEL_ZERO_MAX_SHIFT_FROM_DIGITAL_HOME_TICKS: u16 = 96;
const LF_TRANSITION_SETTLED_SAMPLES: u8 = 4;
const LF_TRANSITION_SETTLE_WINDOW: Duration = Duration::from_millis(400);
const LF_HELD_MAX_SPEED_RAW: u16 = 4;
const NON_PARTICIPATING_MAX_DRIFT_TICKS: u16 = 16;
const HIP_HARDWARE_BLOCK_REASON: &str =
    "isolated HIP hardware profiles remain blocked; use the reviewed LF_HIP_M13_MIN_MAX sequence after LF UPPER/LOWER proof";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Leg {
    Lf,
    Rf,
    Rh,
    Lh,
}

impl Leg {
    const fn label(self) -> &'static str {
        match self {
            Self::Lf => "LF",
            Self::Rf => "RF",
            Self::Rh => "RH",
            Self::Lh => "LH",
        }
    }

    const fn allowed_motor_ids(self) -> &'static [u8] {
        match self {
            Self::Lf => &LF_ALLOWED,
            Self::Rf => &RF_ALLOWED,
            Self::Rh => &RH_ALLOWED,
            Self::Lh => &LH_ALLOWED,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum JointKind {
    Hip,
    Upper,
    Lower,
}

impl JointKind {
    const fn label(self) -> &'static str {
        match self {
            Self::Hip => "HIP",
            Self::Upper => "UPPER",
            Self::Lower => "LOWER",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ContactSide {
    Min,
    Max,
}

impl ContactSide {
    const fn label(self) -> &'static str {
        match self {
            Self::Min => "MIN",
            Self::Max => "MAX",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct JointSpec {
    leg: Leg,
    kind: JointKind,
    name: &'static str,
    motor_id: u8,
    direction: i8,
    min_delta: i16,
    max_delta: i16,
}

impl JointSpec {
    const fn limit_delta(self, side: ContactSide) -> i16 {
        match side {
            ContactSide::Min => self.min_delta,
            ContactSide::Max => self.max_delta,
        }
    }

    fn tick_for_delta(self, q_delta: i16) -> Result<u16, String> {
        let tick = i32::from(HOME_TICK) + i32::from(self.direction) * i32::from(q_delta);
        u16::try_from(tick)
            .ok()
            .filter(|value| *value <= protocol::MAX_ANGLE_STEP)
            .ok_or_else(|| {
                format!(
                    "{} target is outside unsigned ST3215 range: {tick}",
                    self.name
                )
            })
    }
}

const JOINT_SPECS: [JointSpec; 12] = [
    JointSpec {
        leg: Leg::Lf,
        kind: JointKind::Hip,
        name: "lf_hip_joint",
        motor_id: 13,
        direction: -1,
        min_delta: HIP_MIN_DELTA,
        max_delta: HIP_MAX_DELTA,
    },
    JointSpec {
        leg: Leg::Lf,
        kind: JointKind::Upper,
        name: "lf_upper_leg_joint",
        motor_id: 12,
        direction: 1,
        min_delta: UPPER_MIN_DELTA,
        max_delta: UPPER_MAX_DELTA,
    },
    JointSpec {
        leg: Leg::Lf,
        kind: JointKind::Lower,
        name: "lf_lower_leg_joint",
        motor_id: 11,
        direction: -1,
        min_delta: LOWER_MIN_DELTA,
        max_delta: LOWER_MAX_DELTA,
    },
    JointSpec {
        leg: Leg::Rf,
        kind: JointKind::Hip,
        name: "rf_hip_joint",
        motor_id: 23,
        direction: -1,
        min_delta: HIP_MIN_DELTA,
        max_delta: HIP_MAX_DELTA,
    },
    JointSpec {
        leg: Leg::Rf,
        kind: JointKind::Upper,
        name: "rf_upper_leg_joint",
        motor_id: 22,
        direction: -1,
        min_delta: UPPER_MIN_DELTA,
        max_delta: UPPER_MAX_DELTA,
    },
    JointSpec {
        leg: Leg::Rf,
        kind: JointKind::Lower,
        name: "rf_lower_leg_joint",
        motor_id: 21,
        direction: 1,
        min_delta: LOWER_MIN_DELTA,
        max_delta: LOWER_MAX_DELTA,
    },
    JointSpec {
        leg: Leg::Rh,
        kind: JointKind::Hip,
        name: "rh_hip_joint",
        motor_id: 33,
        direction: 1,
        min_delta: HIP_MIN_DELTA,
        max_delta: HIP_MAX_DELTA,
    },
    JointSpec {
        leg: Leg::Rh,
        kind: JointKind::Upper,
        name: "rh_upper_leg_joint",
        motor_id: 32,
        direction: -1,
        min_delta: UPPER_MIN_DELTA,
        max_delta: UPPER_MAX_DELTA,
    },
    JointSpec {
        leg: Leg::Rh,
        kind: JointKind::Lower,
        name: "rh_lower_leg_joint",
        motor_id: 31,
        direction: 1,
        min_delta: LOWER_MIN_DELTA,
        max_delta: LOWER_MAX_DELTA,
    },
    JointSpec {
        leg: Leg::Lh,
        kind: JointKind::Hip,
        name: "lh_hip_joint",
        motor_id: 43,
        direction: 1,
        min_delta: HIP_MIN_DELTA,
        max_delta: HIP_MAX_DELTA,
    },
    JointSpec {
        leg: Leg::Lh,
        kind: JointKind::Upper,
        name: "lh_upper_leg_joint",
        motor_id: 42,
        direction: 1,
        min_delta: UPPER_MIN_DELTA,
        max_delta: UPPER_MAX_DELTA,
    },
    JointSpec {
        leg: Leg::Lh,
        kind: JointKind::Lower,
        name: "lh_lower_leg_joint",
        motor_id: 41,
        direction: -1,
        min_delta: LOWER_MIN_DELTA,
        max_delta: LOWER_MAX_DELTA,
    },
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct StaticTarget {
    motor_id: u8,
    target_tick: u16,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ContactProfile {
    pub(crate) arm_value: String,
    pub(crate) label: String,
    pub(crate) leg: Leg,
    pub(crate) joint: JointKind,
    pub(crate) side: ContactSide,
    pub(crate) joint_name: &'static str,
    pub(crate) motor_id: u8,
    pub(crate) probe_sign: i8,
    pub(crate) urdf_limit_tick: u16,
    pub(crate) guard_tick: u16,
    pub(crate) baseline_target_tick: u16,
    pub(crate) allowed_motor_ids: &'static [u8],
    prerequisites: Vec<StaticTarget>,
}

fn spec_for(leg: Leg, kind: JointKind) -> &'static JointSpec {
    JOINT_SPECS
        .iter()
        .find(|spec| spec.leg == leg && spec.kind == kind)
        .expect("complete MATDOG joint table")
}

fn static_target(leg: Leg, kind: JointKind, q_delta: i16) -> Result<StaticTarget, String> {
    let spec = spec_for(leg, kind);
    Ok(StaticTarget {
        motor_id: spec.motor_id,
        target_tick: spec.tick_for_delta(q_delta)?,
    })
}

fn hip_upper_clearance_delta(leg: Leg, side: ContactSide) -> i16 {
    match (leg, side) {
        (Leg::Lf, ContactSide::Min) | (Leg::Rf, ContactSide::Max) | (Leg::Rh, _) | (Leg::Lh, _) => {
            UPPER_90_DELTA
        }
        (Leg::Lf, ContactSide::Max) | (Leg::Rf, ContactSide::Min) => UPPER_85_DELTA,
    }
}

fn prerequisites_for(
    leg: Leg,
    kind: JointKind,
    side: ContactSide,
) -> Result<Vec<StaticTarget>, String> {
    let mut targets = Vec::new();
    match leg {
        Leg::Lf => targets.push(static_target(Leg::Lh, JointKind::Upper, UPPER_30_DELTA)?),
        Leg::Rf => targets.push(static_target(Leg::Rh, JointKind::Upper, UPPER_30_DELTA)?),
        Leg::Rh | Leg::Lh => {}
    }

    match kind {
        JointKind::Upper => {
            targets.push(static_target(leg, JointKind::Hip, 0)?);
            targets.push(static_target(leg, JointKind::Lower, 0)?);
        }
        JointKind::Lower => {
            targets.push(static_target(leg, JointKind::Hip, 0)?);
            targets.push(static_target(leg, JointKind::Upper, UPPER_90_DELTA)?);
        }
        JointKind::Hip => {
            targets.push(static_target(
                leg,
                JointKind::Upper,
                hip_upper_clearance_delta(leg, side),
            )?);
            targets.push(static_target(leg, JointKind::Lower, LOWER_FOLDED_DELTA)?);
        }
    }
    Ok(targets)
}

#[cfg(test)]
fn prerequisite_restore_order(prerequisites: &[StaticTarget], probing_motor_id: u8) -> Vec<u8> {
    prerequisites
        .iter()
        .filter(|target| target.motor_id != probing_motor_id)
        .rev()
        .map(|target| target.motor_id)
        .collect()
}

fn build_profile(leg: Leg, joint: JointKind, side: ContactSide) -> Result<ContactProfile, String> {
    let spec = *spec_for(leg, joint);
    let urdf_limit_tick = spec.tick_for_delta(spec.limit_delta(side))?;
    let q_sign = match side {
        ContactSide::Min => -1,
        ContactSide::Max => 1,
    };
    let probe_sign = spec.direction * q_sign;
    let guard =
        i32::from(urdf_limit_tick) + i32::from(probe_sign) * i32::from(GUARD_OVERSHOOT_TICKS);
    let baseline = i32::from(HOME_TICK) + i32::from(probe_sign) * i32::from(BASELINE_TRAVEL_TICKS);
    let guard_tick = u16::try_from(guard)
        .ok()
        .filter(|value| *value <= protocol::MAX_ANGLE_STEP)
        .ok_or_else(|| {
            format!(
                "{} {} guard leaves unsigned range: {guard}",
                spec.name,
                side.label()
            )
        })?;
    let baseline_target_tick = u16::try_from(baseline)
        .ok()
        .filter(|value| *value <= protocol::MAX_ANGLE_STEP)
        .ok_or_else(|| format!("{} baseline leaves unsigned range: {baseline}", spec.name))?;

    let arm_value = format!(
        "{}_{}_M{}_{}",
        leg.label(),
        joint.label(),
        spec.motor_id,
        side.label()
    );
    Ok(ContactProfile {
        label: arm_value.clone(),
        arm_value,
        leg,
        joint,
        side,
        joint_name: spec.name,
        motor_id: spec.motor_id,
        probe_sign,
        urdf_limit_tick,
        guard_tick,
        baseline_target_tick,
        allowed_motor_ids: leg.allowed_motor_ids(),
        prerequisites: prerequisites_for(leg, joint, side)?,
    })
}

fn lf_hip_sequence_profile(side: ContactSide) -> Result<ContactProfile, String> {
    let mut profile = build_profile(Leg::Lf, JointKind::Hip, side)?;
    profile.arm_value = LF_HIP_SEQUENCE_ARM_VALUE.to_string();
    profile.label = LF_HIP_SEQUENCE_ARM_VALUE.to_string();
    profile.prerequisites = vec![
        static_target(Leg::Lh, JointKind::Upper, UPPER_30_DELTA)?,
        static_target(Leg::Lf, JointKind::Upper, UPPER_90_DELTA)?,
        static_target(Leg::Lf, JointKind::Lower, LOWER_FOLDED_DELTA)?,
    ];
    Ok(profile)
}

fn is_lf_hip_sequence(profile: &ContactProfile) -> bool {
    profile.arm_value == LF_HIP_SEQUENCE_ARM_VALUE
        && profile.leg == Leg::Lf
        && profile.joint == JointKind::Hip
        && profile.motor_id == 13
}

fn lf_full_sequence_profile() -> Result<ContactProfile, String> {
    let mut profile = build_profile(Leg::Lf, JointKind::Upper, ContactSide::Min)?;
    profile.arm_value = LF_FULL_SEQUENCE_ARM_VALUE.to_string();
    profile.label = LF_FULL_SEQUENCE_ARM_VALUE.to_string();
    profile.allowed_motor_ids = &LF_ALLOWED;
    profile.prerequisites = vec![static_target(Leg::Lh, JointKind::Upper, UPPER_30_DELTA)?];
    Ok(profile)
}

fn is_lf_full_sequence(profile: &ContactProfile) -> bool {
    profile.arm_value == LF_FULL_SEQUENCE_ARM_VALUE
        && profile.leg == Leg::Lf
        && profile.allowed_motor_ids == &LF_ALLOWED
}

pub(crate) fn all_profiles() -> Result<Vec<ContactProfile>, String> {
    let mut profiles = Vec::with_capacity(24);
    for leg in [Leg::Lf, Leg::Rf, Leg::Rh, Leg::Lh] {
        for joint in [JointKind::Upper, JointKind::Lower, JointKind::Hip] {
            for side in [ContactSide::Min, ContactSide::Max] {
                profiles.push(build_profile(leg, joint, side)?);
            }
        }
    }
    Ok(profiles)
}

pub(crate) fn profile_for_arm_value(value: &str) -> Result<ContactProfile, String> {
    if value == LF_FULL_SEQUENCE_ARM_VALUE {
        return lf_full_sequence_profile();
    }
    if value == LF_HIP_SEQUENCE_ARM_VALUE {
        return lf_hip_sequence_profile(ContactSide::Min);
    }
    all_profiles()?
        .into_iter()
        .find(|profile| profile.arm_value == value)
        .ok_or_else(|| {
            let mut supported = all_profiles()
                .unwrap_or_default()
                .into_iter()
                .map(|profile| profile.arm_value)
                .collect::<Vec<_>>();
            supported.push(LF_HIP_SEQUENCE_ARM_VALUE.to_string());
            supported.push(LF_FULL_SEQUENCE_ARM_VALUE.to_string());
            format!(
                "unsupported {MATDOG_ARM_ENV}={value:?}; expected one of: {}",
                supported.join(", ")
            )
        })
}

fn hardware_profile_allowed(profile: &ContactProfile) -> Result<(), String> {
    if profile.joint == JointKind::Hip
        && !is_lf_hip_sequence(profile)
        && !is_lf_full_sequence(profile)
    {
        return Err(format!("{}: {}", profile.label, HIP_HARDWARE_BLOCK_REASON));
    }
    Ok(())
}

pub(crate) fn active_profile() -> Result<ContactProfile, String> {
    let value = std::env::var(MATDOG_ARM_ENV)
        .map_err(|_| format!("MATDOG calibrator is not armed: set {MATDOG_ARM_ENV} explicitly"))?;
    let profile = profile_for_arm_value(&value)?;
    hardware_profile_allowed(&profile)?;
    Ok(profile)
}

pub(crate) fn armed_ram_write_allowed(motor_id: u8, address: u32, value: &[u8]) -> bool {
    let Ok(profile) = active_profile() else {
        return false;
    };
    ram_write_allowed_for_profile(&profile, motor_id, address, value)
}

pub(crate) fn ram_write_allowed_for_profile(
    profile: &ContactProfile,
    motor_id: u8,
    address: u32,
    value: &[u8],
) -> bool {
    // Non-profile MATDOG joints are admitted only for the bounded startup-home
    // sequence. GoalPosition remains constrained by armed_goal_target_allowed().
    if !MATDOG_MOTOR_IDS.contains(&motor_id) {
        return false;
    }

    let register = [
        RamRegister::TorqueEnable,
        RamRegister::Acc,
        RamRegister::GoalPosition,
        RamRegister::GoalSpeed,
        RamRegister::TorqueLimit,
    ]
    .into_iter()
    .find(|register| {
        register.address() as u32 == address && register.size() as usize == value.len()
    });

    let profile_participant = !is_lf_full_sequence(profile) || LF_ALLOWED.contains(&motor_id);
    let startup_home_recovery_motor = MATDOG_MOTOR_IDS.contains(&motor_id);
    match register {
        // Exact global torque OFF must always remain reachable. Torque ON and
        // the exact low-energy RAM settings are also available to every
        // canonical joint for the same bounded startup-home normalization.
        // GoalPosition remains constrained by armed_goal_target_allowed().
        Some(RamRegister::TorqueEnable) => {
            value == [0] || (value == [1] && (profile_participant || startup_home_recovery_motor))
        }
        Some(RamRegister::Acc) => {
            (profile_participant || startup_home_recovery_motor) && value == [ACCELERATION]
        }
        Some(RamRegister::GoalSpeed) => {
            (profile_participant || startup_home_recovery_motor)
                && value == GOAL_SPEED.to_le_bytes()
        }
        Some(RamRegister::TorqueLimit) => {
            (profile_participant || startup_home_recovery_motor)
                && value == TORQUE_LIMIT.to_le_bytes()
        }
        Some(RamRegister::GoalPosition) => {
            let target = u16::from_le_bytes([value[0], value[1]]);
            armed_goal_target_allowed(profile, motor_id, target)
        }
        _ => false,
    }
}

fn full_sequence_joint_goal_allowed(leg: Leg, joint: JointKind, motor_id: u8, target: u16) -> bool {
    let spec = spec_for(leg, joint);
    if motor_id != spec.motor_id {
        return false;
    }
    let Ok(minimum) = build_profile(leg, joint, ContactSide::Min) else {
        return false;
    };
    let Ok(maximum) = build_profile(leg, joint, ContactSide::Max) else {
        return false;
    };
    let low = minimum
        .guard_tick
        .min(maximum.guard_tick)
        .min(HOME_TICK.saturating_sub(MODEL_ZERO_MAX_SHIFT_FROM_DIGITAL_HOME_TICKS));
    let high = minimum.guard_tick.max(maximum.guard_tick).max(
        HOME_TICK
            .saturating_add(MODEL_ZERO_MAX_SHIFT_FROM_DIGITAL_HOME_TICKS)
            .min(protocol::MAX_ANGLE_STEP),
    );
    (low..=high).contains(&target)
}

fn lf_full_sequence_goal_allowed(motor_id: u8, target: u16) -> bool {
    if full_sequence_joint_goal_allowed(Leg::Lf, JointKind::Hip, motor_id, target)
        || full_sequence_joint_goal_allowed(Leg::Lf, JointKind::Upper, motor_id, target)
        || full_sequence_joint_goal_allowed(Leg::Lf, JointKind::Lower, motor_id, target)
    {
        return true;
    }

    if motor_id == spec_for(Leg::Lh, JointKind::Upper).motor_id {
        let Ok(parking) = static_target(Leg::Lh, JointKind::Upper, UPPER_30_DELTA) else {
            return false;
        };
        // q=0 normalization accepts a settled readback within the static
        // tolerance. prepare_motor() then primes the torque-OFF servo at that
        // observed tick before enabling torque. Admit only that same bounded
        // q=0 priming band; the parking target and mechanical corridor remain
        // unchanged.
        let low = HOME_TICK
            .min(parking.target_tick)
            .saturating_sub(STATIC_TOLERANCE_TICKS);
        let high = HOME_TICK
            .max(parking.target_tick)
            .min(protocol::MAX_ANGLE_STEP);
        return (low..=high).contains(&target);
    }

    false
}

fn lf_full_joint_corridor(motor_id: u8) -> Option<TickCorridor> {
    let joint = match motor_id {
        11 => Some(JointKind::Lower),
        12 => Some(JointKind::Upper),
        13 => Some(JointKind::Hip),
        _ => None,
    }?;
    let minimum = build_profile(Leg::Lf, joint, ContactSide::Min).ok()?;
    let maximum = build_profile(Leg::Lf, joint, ContactSide::Max).ok()?;
    Some(TickCorridor {
        low: minimum.guard_tick.min(maximum.guard_tick),
        high: minimum
            .guard_tick
            .max(maximum.guard_tick)
            .min(protocol::MAX_ANGLE_STEP),
    })
}

fn lf_parking_corridor() -> Result<TickCorridor, String> {
    let parking = static_target(Leg::Lh, JointKind::Upper, UPPER_30_DELTA)?;
    Ok(TickCorridor {
        low: HOME_TICK
            .min(parking.target_tick)
            .saturating_sub(PROBE_PASSIVE_RESTORE_DRIFT_TICKS),
        high: HOME_TICK
            .max(parking.target_tick)
            .saturating_add(PROBE_PASSIVE_RESTORE_DRIFT_TICKS)
            .min(protocol::MAX_ANGLE_STEP),
    })
}

fn lf_passive_corridor(state: LfSessionState, motor_id: u8) -> Result<TickCorridor, String> {
    if motor_id == 42 {
        return lf_parking_corridor();
    }
    if !matches!(motor_id, 11 | 12 | 13) {
        return Err(format!("M{motor_id} is not an LF passive participant"));
    }
    if matches!(
        state,
        LfSessionState::Preflight
            | LfSessionState::InitialRecovery
            | LfSessionState::Cleanup
            | LfSessionState::TorqueOff
    ) {
        return lf_full_joint_corridor(motor_id)
            .ok_or_else(|| format!("no full LF corridor for M{motor_id}"));
    }
    Ok(TickCorridor {
        low: HOME_TICK.saturating_sub(PROBE_PASSIVE_RESTORE_DRIFT_TICKS),
        high: HOME_TICK
            .saturating_add(PROBE_PASSIVE_RESTORE_DRIFT_TICKS)
            .min(protocol::MAX_ANGLE_STEP),
    })
}

fn lf_participant_corridor(motor_id: u8) -> Result<TickCorridor, String> {
    if motor_id == 42 {
        lf_parking_corridor()
    } else {
        lf_full_joint_corridor(motor_id)
            .ok_or_else(|| format!("no commanded LF corridor for M{motor_id}"))
    }
}

fn armed_goal_target_allowed(profile: &ContactProfile, motor_id: u8, target: u16) -> bool {
    // Before strict profile roles exist, every canonical MATDOG joint may
    // receive exactly one generic geometric target: digital HOME/q=0. Initial
    // telemetry is an observation, never an admission corridor and never an
    // externally commandable target range.
    if MATDOG_MOTOR_IDS.contains(&motor_id) && target == HOME_TICK {
        return true;
    }
    if is_lf_full_sequence(profile) {
        return lf_full_sequence_goal_allowed(motor_id, target);
    }
    if is_lf_hip_sequence(profile) && motor_id == profile.motor_id {
        let Ok(minimum) = lf_hip_sequence_profile(ContactSide::Min) else {
            return false;
        };
        let Ok(maximum) = lf_hip_sequence_profile(ContactSide::Max) else {
            return false;
        };
        let low = minimum
            .guard_tick
            .min(maximum.guard_tick)
            .saturating_sub(STATIC_TOLERANCE_TICKS);
        let high = minimum
            .guard_tick
            .max(maximum.guard_tick)
            .saturating_add(STATIC_TOLERANCE_TICKS)
            .min(protocol::MAX_ANGLE_STEP);
        return (low..=high).contains(&target);
    }

    if motor_id == profile.motor_id {
        let low = profile
            .guard_tick
            .min(HOME_TICK)
            .saturating_sub(STATIC_TOLERANCE_TICKS);
        let high = profile
            .guard_tick
            .max(HOME_TICK)
            .saturating_add(STATIC_TOLERANCE_TICKS);
        return (low..=high.min(protocol::MAX_ANGLE_STEP)).contains(&target);
    }

    let Some(prerequisite) = profile
        .prerequisites
        .iter()
        .find(|prerequisite| prerequisite.motor_id == motor_id)
    else {
        return false;
    };
    let low = prerequisite
        .target_tick
        .min(HOME_TICK)
        .saturating_sub(STATIC_TOLERANCE_TICKS);
    let high = prerequisite
        .target_tick
        .max(HOME_TICK)
        .saturating_add(STATIC_TOLERANCE_TICKS)
        .min(protocol::MAX_ANGLE_STEP);
    (low..=high).contains(&target)
}

pub fn is_exact_matdog_motor_set(found: &[u8]) -> bool {
    if found.len() != MATDOG_MOTOR_IDS.len() {
        return false;
    }
    let found: BTreeSet<u8> = found.iter().copied().collect();
    found == MATDOG_MOTOR_IDS.into_iter().collect()
}

fn contact_acceptance_bounds(profile: &ContactProfile) -> (u16, u16) {
    let inner = i32::from(profile.urdf_limit_tick)
        - i32::from(profile.probe_sign) * i32::from(CONTACT_ACCEPTANCE_INNER_TICKS);
    let inner = u16::try_from(inner).unwrap_or(if profile.probe_sign > 0 {
        0
    } else {
        protocol::MAX_ANGLE_STEP
    });
    (
        inner.min(profile.guard_tick),
        inner.max(profile.guard_tick).min(protocol::MAX_ANGLE_STEP),
    )
}

fn adaptive_contact_acceptance_bounds(
    profile: &ContactProfile,
    coarse_scout_tick: Option<u16>,
) -> (u16, u16) {
    let (mut low, mut high) = contact_acceptance_bounds(profile);
    if let Some(scout) = coarse_scout_tick {
        // The coarse pass is allowed to discover an earlier real stop on the
        // HOME-facing side of the model corridor. Never extend beyond the
        // mechanical guard; extend only away from it by a bounded amount.
        if profile.probe_sign > 0 {
            low = low.min(scout.saturating_sub(ADAPTIVE_FINE_SCOUT_TICKS));
        } else {
            high = high.max(
                scout
                    .saturating_add(ADAPTIVE_FINE_SCOUT_TICKS)
                    .min(protocol::MAX_ANGLE_STEP),
            );
        }
    }
    (low, high)
}

fn probe_tracking_error_limit(step_ticks: u16) -> u16 {
    step_ticks
        .saturating_add(4)
        .max(PROBE_TRACKING_ERROR_FLOOR_TICKS)
}

fn fine_contact_scout_lag_ticks(
    candidate_tick: u16,
    coarse_scout_tick: u16,
    probe_sign: i8,
) -> u16 {
    let signed_lag =
        i32::from(signed_tick_delta(coarse_scout_tick, candidate_tick)) * i32::from(probe_sign);
    signed_lag.max(0).min(i32::from(u16::MAX)) as u16
}

fn fine_contact_reproduces_coarse_depth(
    candidate_tick: u16,
    coarse_scout_tick: u16,
    probe_sign: i8,
) -> bool {
    fine_contact_scout_lag_ticks(candidate_tick, coarse_scout_tick, probe_sign)
        <= FINE_CONTACT_SCOUT_LAG_TOLERANCE_TICKS
}

#[cfg(test)]
fn position_inside_adaptive_contact_acceptance(
    profile: &ContactProfile,
    coarse_scout_tick: u16,
    position: u16,
) -> bool {
    let (low, high) = adaptive_contact_acceptance_bounds(profile, Some(coarse_scout_tick));
    (low..=high).contains(&position)
}

#[cfg(test)]
fn position_inside_contact_acceptance(profile: &ContactProfile, position: u16) -> bool {
    let (low, high) = contact_acceptance_bounds(profile);
    (low..=high).contains(&position)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ContactState {
    FreeMotion,
    ContactSuspected,
    ContactConfirmed,
    EarlyStall,
    HardAbort,
}

#[derive(Debug, Default)]
struct StableTargetGate {
    consecutive_samples: u8,
    first_qualifying_sample: Option<Instant>,
}

impl StableTargetGate {
    fn observe_at(
        &mut self,
        observation: MotorObservation,
        target_tick: u16,
        tolerance_ticks: u16,
        now: Instant,
    ) -> bool {
        let qualifies = circular_distance(observation.position, target_tick) <= tolerance_ticks
            && speed_magnitude(observation.velocity) <= LF_HELD_MAX_SPEED_RAW;
        if !qualifies {
            self.consecutive_samples = 0;
            self.first_qualifying_sample = None;
            return false;
        }
        self.consecutive_samples = self.consecutive_samples.saturating_add(1);
        let first = *self.first_qualifying_sample.get_or_insert(now);
        self.consecutive_samples >= LF_TRANSITION_SETTLED_SAMPLES
            && now.duration_since(first) >= LF_TRANSITION_SETTLE_WINDOW
    }
}

fn lf_initial_recovery_needed(observation: MotorObservation) -> bool {
    circular_distance(observation.position, HOME_TICK) > PROBE_HOME_TOLERANCE_TICKS
        || speed_magnitude(observation.velocity) > LF_HELD_MAX_SPEED_RAW
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct MotorObservation {
    monotonic_stamp_ns: u64,
    position: u16,
    velocity: u16,
    current: u16,
    temperature: u8,
    temperature_limit: u8,
    goal_position: u16,
    torque_limit: u16,
    torque_enabled: bool,
    status: u8,
    has_driver_error: bool,
}

fn validate_matdog_temperature(motor_id: u8, observation: MotorObservation) -> Result<(), String> {
    if observation.temperature_limit != EXPECTED_TEMPERATURE_LIMIT_C {
        return Err(format!(
            "M{motor_id} configured temperature limit changed: {}°C != {}°C",
            observation.temperature_limit, EXPECTED_TEMPERATURE_LIMIT_C
        ));
    }
    if observation.temperature > observation.temperature_limit {
        return Err(format!(
            "M{motor_id} thermal abort: {}°C > configured {}°C",
            observation.temperature, observation.temperature_limit
        ));
    }
    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LfSessionState {
    Preflight,
    InitialRecovery,
    Parking,
    UpperMin,
    UpperMax,
    UpperHorizontal,
    LowerMin,
    LowerMax,
    LowerFolded,
    HipMin,
    HipMax,
    Diagnostics,
    ReturnHip,
    ReturnLowerHeld,
    ReturnUpper,
    RestoreParking,
    Cleanup,
    TorqueOff,
}

impl LfSessionState {
    const fn label(self) -> &'static str {
        match self {
            Self::Preflight => "preflight",
            Self::InitialRecovery => "initial-recovery",
            Self::Parking => "parking",
            Self::UpperMin => "upper-min",
            Self::UpperMax => "upper-max",
            Self::UpperHorizontal => "upper-horizontal",
            Self::LowerMin => "lower-min",
            Self::LowerMax => "lower-max",
            Self::LowerFolded => "lower-folded",
            Self::HipMin => "hip-min",
            Self::HipMax => "hip-max",
            Self::Diagnostics => "diagnostics",
            Self::ReturnHip => "return-hip",
            Self::ReturnLowerHeld => "return-lower-held",
            Self::ReturnUpper => "return-upper",
            Self::RestoreParking => "restore-parking",
            Self::Cleanup => "cleanup",
            Self::TorqueOff => "torque-off",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LfActiveKind {
    Commanded,
    ContactProbe,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct LfActiveMotor {
    motor_id: u8,
    target_tick: u16,
    kind: LfActiveKind,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct TickCorridor {
    low: u16,
    high: u16,
}

impl TickCorridor {
    const fn contains(self, tick: u16) -> bool {
        tick >= self.low && tick <= self.high
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LfMotorRole {
    ActivelyCommanded { target_tick: u16 },
    ActivelyHeld { target_tick: u16 },
    PassiveTorqueOffSafe { corridor: TickCorridor },
    NonParticipatingTorqueOff { entry_tick: u16 },
    ContactProbe { target_tick: u16 },
}

/// The ONE MATDOG leg session engine (G2 contract section 6, section 20).
///
/// Phase 2A has a single engine type on purpose: there is no per-leg
/// state-machine family and no second physical executor. The `mode` field is
/// the migration boundary — it records which reviewed LF session mode
/// (contract section 6.2) this session expands, so the remaining LF legacy
/// modes can later migrate into this engine instead of duplicating it. G3-A
/// only records the mode: no transition, no role derivation, no corridor and
/// no GoalPosition write reads it.
#[derive(Debug, Clone)]
struct LegSessionStateMachine {
    // Read by the structural foundation only until the legacy modes migrate.
    #[allow(dead_code)]
    mode: LfSessionMode,
    state: LfSessionState,
    active: Option<LfActiveMotor>,
    held_targets: Vec<StaticTarget>,
    entry_positions: Vec<(u8, u16)>,
    contacts: [Option<DualContactResult>; 3],
    fixed_scale: [Option<ModelZeroEstimate>; 3],
    affine: [Option<AffineJointCalibration>; 3],
    trace: Vec<LfSessionState>,
}

impl LegSessionStateMachine {
    fn new(mode: LfSessionMode, entry_positions: Vec<(u8, u16)>) -> Result<Self, String> {
        let ids = entry_positions
            .iter()
            .map(|(motor_id, _)| *motor_id)
            .collect::<Vec<_>>();
        if !is_exact_matdog_motor_set(&ids) {
            return Err(
                "LF state machine requires one entry position for every canonical ID".into(),
            );
        }
        Ok(Self {
            mode,
            state: LfSessionState::Preflight,
            active: None,
            held_targets: Vec::new(),
            entry_positions,
            contacts: [None; 3],
            fixed_scale: [None; 3],
            affine: [None; 3],
            trace: vec![LfSessionState::Preflight],
        })
    }

    #[allow(dead_code)]
    fn mode(&self) -> LfSessionMode {
        self.mode
    }

    fn transition(&mut self, next: LfSessionState) -> Result<(), String> {
        let allowed = self.state == next
            || matches!(next, LfSessionState::Cleanup)
            || matches!(
                (self.state, next),
                (LfSessionState::Preflight, LfSessionState::InitialRecovery)
                    | (LfSessionState::InitialRecovery, LfSessionState::Parking)
                    | (LfSessionState::Parking, LfSessionState::UpperMin)
                    | (LfSessionState::UpperMin, LfSessionState::UpperMax)
                    | (LfSessionState::UpperMax, LfSessionState::UpperHorizontal)
                    | (LfSessionState::UpperHorizontal, LfSessionState::LowerMin)
                    | (LfSessionState::LowerMin, LfSessionState::LowerMax)
                    | (LfSessionState::LowerMax, LfSessionState::LowerFolded)
                    | (LfSessionState::LowerFolded, LfSessionState::HipMin)
                    | (LfSessionState::HipMin, LfSessionState::HipMax)
                    | (LfSessionState::HipMax, LfSessionState::Diagnostics)
                    | (LfSessionState::Diagnostics, LfSessionState::ReturnHip)
                    | (LfSessionState::ReturnHip, LfSessionState::ReturnLowerHeld)
                    | (LfSessionState::ReturnLowerHeld, LfSessionState::ReturnUpper)
                    | (LfSessionState::ReturnUpper, LfSessionState::RestoreParking)
                    | (LfSessionState::RestoreParking, LfSessionState::Cleanup)
                    | (LfSessionState::Cleanup, LfSessionState::TorqueOff)
            );
        if !allowed {
            return Err(format!(
                "invalid LF state transition: {} -> {}",
                self.state.label(),
                next.label()
            ));
        }
        if self.state != next && !matches!(next, LfSessionState::Cleanup) {
            self.validate_transition_entry(next)?;
        }
        if self.state != next {
            self.state = next;
            if next != LfSessionState::Diagnostics {
                self.active = None;
            }
            self.trace.push(next);
        }
        Ok(())
    }

    fn validate_transition_entry(&self, next: LfSessionState) -> Result<(), String> {
        let required_holds: &[u8] = match next {
            LfSessionState::Preflight
            | LfSessionState::InitialRecovery
            | LfSessionState::Parking
            | LfSessionState::Cleanup
            | LfSessionState::TorqueOff => &[],
            LfSessionState::UpperMin
            | LfSessionState::UpperMax
            | LfSessionState::UpperHorizontal => &[42],
            LfSessionState::LowerMin | LfSessionState::LowerMax | LfSessionState::LowerFolded => {
                &[12, 42]
            }
            LfSessionState::HipMin
            | LfSessionState::HipMax
            | LfSessionState::Diagnostics
            | LfSessionState::ReturnHip => &[11, 12, 42],
            LfSessionState::ReturnLowerHeld
            | LfSessionState::ReturnUpper
            | LfSessionState::RestoreParking => &[11, 12, 13, 42],
        };
        let observed_holds = self
            .held_targets
            .iter()
            .map(|target| target.motor_id)
            .collect::<BTreeSet<_>>();
        let expected_holds = required_holds.iter().copied().collect::<BTreeSet<_>>();
        if observed_holds != expected_holds {
            return Err(format!(
                "LF transition {} -> {} has wrong held set: expected={expected_holds:?}, observed={observed_holds:?}",
                self.state.label(),
                next.label(),
            ));
        }

        let required_previous_active = match (self.state, next) {
            (LfSessionState::UpperMin, LfSessionState::UpperMax)
            | (LfSessionState::UpperMax, LfSessionState::UpperHorizontal) => Some(12),
            (LfSessionState::LowerMin, LfSessionState::LowerMax)
            | (LfSessionState::LowerMax, LfSessionState::LowerFolded) => Some(11),
            (LfSessionState::HipMin, LfSessionState::HipMax)
            | (LfSessionState::HipMax, LfSessionState::Diagnostics)
            | (LfSessionState::Diagnostics, LfSessionState::ReturnHip) => Some(13),
            _ => None,
        };
        if let Some(required_motor) = required_previous_active {
            if self.active.map(|active| active.motor_id) != Some(required_motor) {
                return Err(format!(
                    "LF transition {} -> {} requires active M{required_motor}",
                    self.state.label(),
                    next.label(),
                ));
            }
        }
        Ok(())
    }

    fn active_motor_allowed(&self, motor_id: u8) -> bool {
        match self.state {
            LfSessionState::InitialRecovery => matches!(motor_id, 11 | 12 | 13),
            LfSessionState::Parking | LfSessionState::RestoreParking => motor_id == 42,
            LfSessionState::UpperMin
            | LfSessionState::UpperMax
            | LfSessionState::UpperHorizontal => motor_id == 12,
            LfSessionState::LowerMin | LfSessionState::LowerMax | LfSessionState::LowerFolded => {
                motor_id == 11
            }
            LfSessionState::HipMin
            | LfSessionState::HipMax
            | LfSessionState::Diagnostics
            | LfSessionState::ReturnHip => motor_id == 13,
            LfSessionState::ReturnLowerHeld => motor_id == 11,
            LfSessionState::ReturnUpper => motor_id == 12,
            LfSessionState::Preflight | LfSessionState::Cleanup | LfSessionState::TorqueOff => {
                false
            }
        }
    }

    fn set_active(
        &mut self,
        motor_id: u8,
        target_tick: u16,
        kind: LfActiveKind,
    ) -> Result<(), String> {
        if !self.active_motor_allowed(motor_id) {
            return Err(format!(
                "M{motor_id} cannot be active in LF state {}",
                self.state.label()
            ));
        }
        let corridor = lf_participant_corridor(motor_id)?;
        if !corridor.contains(target_tick) {
            return Err(format!(
                "M{motor_id} target {target_tick} outside LF state corridor {}..={}",
                corridor.low, corridor.high
            ));
        }
        self.active = Some(LfActiveMotor {
            motor_id,
            target_tick,
            kind,
        });
        Ok(())
    }

    fn update_active_target(&mut self, motor_id: u8, target_tick: u16) -> Result<(), String> {
        let corridor = lf_participant_corridor(motor_id)?;
        if !corridor.contains(target_tick) {
            return Err(format!(
                "M{motor_id} target {target_tick} outside LF commanded corridor {}..={}",
                corridor.low, corridor.high
            ));
        }
        let active = self
            .active
            .as_mut()
            .filter(|active| active.motor_id == motor_id)
            .ok_or_else(|| {
                format!(
                    "M{motor_id} GoalPosition update has no matching active LF role in {}",
                    self.state.label()
                )
            })?;
        active.target_tick = target_tick;
        Ok(())
    }

    fn clear_active(&mut self, motor_id: u8) {
        if self.active.map(|active| active.motor_id) == Some(motor_id) {
            self.active = None;
        }
    }

    fn hold(&mut self, target: StaticTarget) -> Result<(), String> {
        let allowed = matches!(
            (self.state, target.motor_id),
            (LfSessionState::Parking, 42)
                | (LfSessionState::UpperHorizontal, 12)
                | (LfSessionState::LowerFolded, 11)
                | (LfSessionState::ReturnHip, 13)
                | (LfSessionState::ReturnLowerHeld, 11)
                | (LfSessionState::ReturnUpper, 12)
        );
        if !allowed {
            return Err(format!(
                "M{} cannot be promoted to held in LF state {}",
                target.motor_id,
                self.state.label()
            ));
        }
        let active = self.active.ok_or_else(|| {
            format!(
                "M{} cannot be held without a completed active move",
                target.motor_id
            )
        })?;
        if active.motor_id != target.motor_id || active.target_tick != target.target_tick {
            return Err(format!(
                "M{} held target {} does not match active M{} target {}",
                target.motor_id, target.target_tick, active.motor_id, active.target_tick
            ));
        }
        let corridor = lf_participant_corridor(target.motor_id)?;
        if !corridor.contains(target.target_tick) {
            return Err(format!(
                "M{} held target {} outside {}..={}",
                target.motor_id, target.target_tick, corridor.low, corridor.high
            ));
        }
        self.release(target.motor_id);
        self.held_targets.push(target);
        self.clear_active(target.motor_id);
        Ok(())
    }

    fn release(&mut self, motor_id: u8) {
        self.held_targets
            .retain(|target| target.motor_id != motor_id);
    }

    fn role_for(&self, motor_id: u8) -> Result<LfMotorRole, String> {
        if let Some(active) = self.active.filter(|active| active.motor_id == motor_id) {
            return Ok(match active.kind {
                LfActiveKind::Commanded => LfMotorRole::ActivelyCommanded {
                    target_tick: active.target_tick,
                },
                LfActiveKind::ContactProbe => LfMotorRole::ContactProbe {
                    target_tick: active.target_tick,
                },
            });
        }
        if let Some(target) = self
            .held_targets
            .iter()
            .find(|target| target.motor_id == motor_id)
        {
            return Ok(LfMotorRole::ActivelyHeld {
                target_tick: target.target_tick,
            });
        }
        if matches!(motor_id, 11 | 12 | 13 | 42) {
            return Ok(LfMotorRole::PassiveTorqueOffSafe {
                corridor: lf_passive_corridor(self.state, motor_id)?,
            });
        }
        let entry_tick = self
            .entry_positions
            .iter()
            .find_map(|(entry_id, tick)| (*entry_id == motor_id).then_some(*tick))
            .ok_or_else(|| format!("M{motor_id} has no LF session-entry observation"))?;
        Ok(LfMotorRole::NonParticipatingTorqueOff { entry_tick })
    }

    fn record_contacts(&mut self, joint: JointKind, contacts: DualContactResult) {
        self.contacts[lf_joint_index(joint)] = Some(contacts);
    }

    fn record_diagnostics(
        &mut self,
        joint: JointKind,
        fixed_scale: ModelZeroEstimate,
        affine: AffineJointCalibration,
    ) {
        let index = lf_joint_index(joint);
        self.fixed_scale[index] = Some(fixed_scale);
        self.affine[index] = Some(affine);
    }

    fn has_complete_evidence(&self) -> bool {
        self.contacts.iter().all(Option::is_some)
            && self.fixed_scale.iter().all(Option::is_some)
            && self.affine.iter().all(Option::is_some)
    }

    fn trace_summary(&self) -> String {
        self.trace
            .iter()
            .map(|state| state.label())
            .collect::<Vec<_>>()
            .join(" -> ")
    }

    fn complete_verified_cleanup(&mut self) -> Result<(), String> {
        self.held_targets.clear();
        self.active = None;
        self.transition(LfSessionState::TorqueOff)
    }
}

const fn lf_joint_index(joint: JointKind) -> usize {
    match joint {
        JointKind::Hip => 0,
        JointKind::Upper => 1,
        JointKind::Lower => 2,
    }
}

const fn lf_contact_state(joint: JointKind, side: ContactSide) -> LfSessionState {
    match (joint, side) {
        (JointKind::Upper, ContactSide::Min) => LfSessionState::UpperMin,
        (JointKind::Upper, ContactSide::Max) => LfSessionState::UpperMax,
        (JointKind::Lower, ContactSide::Min) => LfSessionState::LowerMin,
        (JointKind::Lower, ContactSide::Max) => LfSessionState::LowerMax,
        (JointKind::Hip, ContactSide::Min) => LfSessionState::HipMin,
        (JointKind::Hip, ContactSide::Max) => LfSessionState::HipMax,
    }
}

fn validate_lf_role_observation(
    motor_id: u8,
    observation: MotorObservation,
    role: LfMotorRole,
    now_ns: u64,
) -> Result<(), String> {
    let max_age_ns = u64::try_from(MAX_TELEMETRY_AGE.as_nanos()).unwrap_or(u64::MAX);
    let age_ns = now_ns.saturating_sub(observation.monotonic_stamp_ns);
    if observation.monotonic_stamp_ns == 0 || age_ns > max_age_ns {
        return Err(format!(
            "M{motor_id} telemetry stale in LF role {role:?}: age_ns={age_ns}"
        ));
    }
    if observation.has_driver_error || observation.status != 0 {
        return Err(format!(
            "M{motor_id} unhealthy in LF role {role:?}: status=0x{:02X}, driver_error={}",
            observation.status, observation.has_driver_error
        ));
    }
    if observation.current >= HARD_CURRENT_ABORT_RAW {
        return Err(format!(
            "M{motor_id} hard current in LF role {role:?}: {} >= {}",
            observation.current, HARD_CURRENT_ABORT_RAW
        ));
    }
    validate_matdog_temperature(motor_id, observation)
        .map_err(|message| format!("{message} in LF role {role:?}"))?;

    match role {
        LfMotorRole::ActivelyCommanded { target_tick }
        | LfMotorRole::ContactProbe { target_tick } => {
            validate_lf_active_readback(motor_id, observation, target_tick)
        }
        LfMotorRole::ActivelyHeld { target_tick } => {
            validate_lf_active_readback(motor_id, observation, target_tick)?;
            let error = circular_distance(observation.position, target_tick);
            if error > STATIC_TOLERANCE_TICKS {
                return Err(format!(
                    "actively-held M{motor_id} drifted: target={target_tick}, present={}, error={error}",
                    observation.position
                ));
            }
            Ok(())
        }
        LfMotorRole::PassiveTorqueOffSafe { corridor } => {
            if observation.torque_enabled {
                return Err(format!("passive-safe M{motor_id} unexpectedly torque ON"));
            }
            if !corridor.contains(observation.position) {
                return Err(format!(
                    "passive-safe M{motor_id} left state corridor: present={}, allowed={}..={}",
                    observation.position, corridor.low, corridor.high
                ));
            }
            Ok(())
        }
        LfMotorRole::NonParticipatingTorqueOff { entry_tick } => {
            if observation.torque_enabled {
                return Err(format!(
                    "non-participating M{motor_id} unexpectedly torque ON"
                ));
            }
            let drift = circular_distance(observation.position, entry_tick);
            if drift > NON_PARTICIPATING_MAX_DRIFT_TICKS {
                return Err(format!(
                    "non-participating M{motor_id} moved unexpectedly: entry={entry_tick}, present={}, drift={drift}",
                    observation.position
                ));
            }
            Ok(())
        }
    }
}

fn validate_lf_active_readback(
    motor_id: u8,
    observation: MotorObservation,
    target_tick: u16,
) -> Result<(), String> {
    if !observation.torque_enabled {
        return Err(format!("active M{motor_id} torque unexpectedly OFF"));
    }
    if observation.torque_limit != TORQUE_LIMIT {
        return Err(format!(
            "active M{motor_id} torque limit changed: expected={TORQUE_LIMIT}, observed={}",
            observation.torque_limit
        ));
    }
    if observation.goal_position != target_tick {
        return Err(format!(
            "active M{motor_id} goal changed: expected={target_tick}, observed={}",
            observation.goal_position
        ));
    }
    let corridor = lf_participant_corridor(motor_id)?;
    if !corridor.contains(observation.position) || !corridor.contains(target_tick) {
        return Err(format!(
            "active M{motor_id} left commanded corridor: present={}, target={target_tick}, allowed={}..={}",
            observation.position, corridor.low, corridor.high
        ));
    }
    Ok(())
}

fn validate_lf_session_snapshot(
    state: &InferenceState,
    bus_serial: &str,
    session: &LegSessionStateMachine,
    ignored_motor: u8,
    now_ns: u64,
) -> Result<(), String> {
    let found = motor_ids_for_bus(state, bus_serial).map_err(|error| error.to_string())?;
    if !is_exact_matdog_motor_set(&found) {
        return Err(format!(
            "LF runtime ID set changed: expected={:?}, found={found:?}",
            MATDOG_MOTOR_IDS
        ));
    }
    for motor_id in MATDOG_MOTOR_IDS {
        if motor_id == ignored_motor {
            continue;
        }
        let observation = observation_from_state(state, bus_serial, motor_id)
            .map_err(|error| error.to_string())?;
        let role = session.role_for(motor_id)?;
        validate_lf_role_observation(motor_id, observation, role, now_ns)?;
    }
    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StartupRole {
    Probe,
    Prerequisite { target_tick: u16 },
    HomeOnly,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct StartupEntryPlan {
    home_recovery_motors: Vec<u8>,
    home_ready_motors: BTreeSet<u8>,
}

fn startup_role_for_profile(profile: &ContactProfile, motor_id: u8) -> StartupRole {
    if motor_id == profile.motor_id {
        return StartupRole::Probe;
    }
    if let Some(target) = profile
        .prerequisites
        .iter()
        .find(|target| target.motor_id == motor_id)
    {
        return StartupRole::Prerequisite {
            target_tick: target.target_tick,
        };
    }
    StartupRole::HomeOnly
}

fn startup_prerequisite_bounds(target_tick: u16) -> (u16, u16) {
    if target_tick >= HOME_TICK {
        (
            HOME_TICK.saturating_sub(STARTUP_PREREQUISITE_HOME_SETTLE_TICKS),
            target_tick
                .saturating_add(STATIC_TOLERANCE_TICKS)
                .min(protocol::MAX_ANGLE_STEP),
        )
    } else {
        (
            target_tick.saturating_sub(STATIC_TOLERANCE_TICKS),
            HOME_TICK
                .saturating_add(STARTUP_PREREQUISITE_HOME_SETTLE_TICKS)
                .min(protocol::MAX_ANGLE_STEP),
        )
    }
}

// Preserve the strict model/guard endpoint while allowing the active probe
// to settle within the already validated digital-home tolerance under
// geometry-prerequisite load and gearbox backlash.
fn startup_probe_bounds(profile: &ContactProfile) -> (u16, u16) {
    if profile.probe_sign < 0 {
        (
            profile.guard_tick.saturating_sub(STATIC_TOLERANCE_TICKS),
            HOME_TICK
                .saturating_add(PROBE_HOME_TOLERANCE_TICKS)
                .min(protocol::MAX_ANGLE_STEP),
        )
    } else {
        (
            HOME_TICK.saturating_sub(PROBE_HOME_TOLERANCE_TICKS),
            profile
                .guard_tick
                .saturating_add(STATIC_TOLERANCE_TICKS)
                .min(protocol::MAX_ANGLE_STEP),
        )
    }
}

fn home_hold_tolerance(
    profile: &ContactProfile,
    motor_id: u8,
    probe_home_handoff_active: bool,
) -> u16 {
    if probe_home_handoff_active && motor_id == profile.motor_id {
        PROBE_PASSIVE_RESTORE_DRIFT_TICKS
    } else {
        STATIC_TOLERANCE_TICKS
    }
}

fn startup_envelope(profile: &ContactProfile, motor_id: u8) -> (u16, u16) {
    if is_lf_hip_sequence(profile) && motor_id == profile.motor_id {
        let minimum = lf_hip_sequence_profile(ContactSide::Min)
            .expect("validated LF HIP MIN sequence profile");
        let maximum = lf_hip_sequence_profile(ContactSide::Max)
            .expect("validated LF HIP MAX sequence profile");
        let low = minimum
            .guard_tick
            .min(maximum.guard_tick)
            .saturating_sub(STATIC_TOLERANCE_TICKS);
        let high = minimum
            .guard_tick
            .max(maximum.guard_tick)
            .saturating_add(STATIC_TOLERANCE_TICKS)
            .min(protocol::MAX_ANGLE_STEP);
        return (low, high);
    }

    match startup_role_for_profile(profile, motor_id) {
        StartupRole::Probe => startup_probe_bounds(profile),
        StartupRole::Prerequisite { target_tick } if target_tick != HOME_TICK => {
            startup_prerequisite_bounds(target_tick)
        }
        StartupRole::Prerequisite { .. } | StartupRole::HomeOnly => (
            HOME_TICK.saturating_sub(STARTUP_HOME_RECOVERY_LIMIT_TICKS),
            HOME_TICK
                .saturating_add(STARTUP_HOME_RECOVERY_LIMIT_TICKS)
                .min(protocol::MAX_ANGLE_STEP),
        ),
    }
}

fn startup_position_allowed(profile: &ContactProfile, motor_id: u8, position: u16) -> bool {
    let (low, high) = startup_envelope(profile, motor_id);
    (low..=high).contains(&position)
}

fn startup_home_initial_position_valid(position: u16) -> bool {
    position <= protocol::MAX_ANGLE_STEP
}

fn startup_role_label(role: StartupRole) -> String {
    match role {
        StartupRole::Probe => "probe".to_string(),
        StartupRole::Prerequisite { target_tick } => {
            format!("prerequisite(target={target_tick})")
        }
        StartupRole::HomeOnly => "home-only".to_string(),
    }
}

fn validate_profile_entry_hold(
    profile: &ContactProfile,
    motor_id: u8,
    ignored_motor: u8,
    home_ready_motors: &BTreeSet<u8>,
    established_prerequisites: &BTreeSet<u8>,
    observation: MotorObservation,
) -> Result<(), String> {
    if motor_id == ignored_motor {
        return Ok(());
    }
    if observation.has_driver_error || observation.status != 0 {
        return Err(format!(
            "profile-entry M{motor_id} unhealthy: status=0x{:02X}, driver_error={}",
            observation.status, observation.has_driver_error
        ));
    }
    if observation.current >= HARD_CURRENT_ABORT_RAW {
        return Err(format!(
            "profile-entry M{motor_id} hard current abort: {} >= {}",
            observation.current, HARD_CURRENT_ABORT_RAW
        ));
    }

    if established_prerequisites.contains(&motor_id) {
        let target = profile
            .prerequisites
            .iter()
            .find(|target| target.motor_id == motor_id)
            .ok_or_else(|| format!("profile-entry established M{motor_id} has no target"))?;
        if !observation.torque_enabled {
            return Err(format!(
                "profile-entry prerequisite M{motor_id} unexpectedly torque-disabled"
            ));
        }
        if observation.torque_limit != TORQUE_LIMIT {
            return Err(format!(
                "profile-entry prerequisite M{motor_id} torque-limit changed: expected={}, observed={}",
                TORQUE_LIMIT, observation.torque_limit
            ));
        }
        if observation.goal_position != target.target_tick {
            return Err(format!(
                "profile-entry prerequisite M{motor_id} goal changed: expected={}, observed={}",
                target.target_tick, observation.goal_position
            ));
        }
        if circular_distance(observation.position, target.target_tick) > STATIC_TOLERANCE_TICKS {
            return Err(format!(
                "profile-entry prerequisite M{motor_id} drifted: target={}, present={}, tolerance={}",
                target.target_tick, observation.position, STATIC_TOLERANCE_TICKS
            ));
        }
        return Ok(());
    }

    if observation.torque_enabled {
        return Err(format!(
            "profile-entry pending M{motor_id} unexpectedly torque-enabled"
        ));
    }

    if home_ready_motors.contains(&motor_id) {
        let distance = circular_distance(observation.position, HOME_TICK);
        if distance > STATIC_TOLERANCE_TICKS {
            return Err(format!(
                "profile-entry recovered M{motor_id} left home: present={}, distance={}, tolerance={}",
                observation.position, distance, STATIC_TOLERANCE_TICKS
            ));
        }
        return Ok(());
    }

    if !startup_position_allowed(profile, motor_id, observation.position) {
        let (low, high) = startup_envelope(profile, motor_id);
        return Err(format!(
            "profile-entry pending M{motor_id} left restart envelope: role={}, present={}, allowed={}..={}",
            startup_role_label(startup_role_for_profile(profile, motor_id)),
            observation.position,
            low,
            high
        ));
    }
    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct BaselineStats {
    median_current: u16,
    mad_current: u16,
}

impl BaselineStats {
    fn from_samples(samples: &[u16]) -> Result<Self, &'static str> {
        if samples.is_empty() {
            return Err("empty current baseline");
        }
        let median_current = median(samples);
        let deviations: Vec<u16> = samples
            .iter()
            .map(|value| value.abs_diff(median_current))
            .collect();
        Ok(Self {
            median_current,
            mad_current: median(&deviations),
        })
    }

    fn contact_threshold(self) -> u16 {
        self.median_current
            .saturating_add(self.mad_current.saturating_mul(4).max(5))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct HybridContactConfig {
    max_progress_ticks: u16,
    max_velocity_raw: u16,
    target_reached_tolerance_ticks: u16,
    min_travel_ticks: u16,
    persistence_samples: u8,
    hard_current_abort_raw: u16,
}

impl Default for HybridContactConfig {
    fn default() -> Self {
        Self {
            max_progress_ticks: 2,
            max_velocity_raw: 10,
            target_reached_tolerance_ticks: STATIC_TOLERANCE_TICKS,
            min_travel_ticks: MINIMUM_CONTACT_TRAVEL_TICKS,
            persistence_samples: 3,
            hard_current_abort_raw: HARD_CURRENT_ABORT_RAW,
        }
    }
}

#[derive(Debug)]
struct HybridContactDetector {
    start_position: u16,
    previous_position: u16,
    baseline: BaselineStats,
    config: HybridContactConfig,
    probe_sign: i8,
    confirming_samples: u8,
    active_target: Option<u16>,
    target_samples_seen: u8,
    acceptance_low: u16,
    acceptance_high: u16,
}

impl HybridContactDetector {
    #[cfg(test)]
    fn new(start_position: u16, baseline: BaselineStats, probe_sign: i8) -> Self {
        Self::with_acceptance(
            start_position,
            baseline,
            probe_sign,
            0,
            protocol::MAX_ANGLE_STEP,
        )
    }

    #[cfg(test)]
    fn new_for_profile(
        start_position: u16,
        baseline: BaselineStats,
        profile: &ContactProfile,
    ) -> Self {
        Self::new_for_profile_with_scout(start_position, baseline, profile, None)
    }

    fn new_for_profile_with_scout(
        start_position: u16,
        baseline: BaselineStats,
        profile: &ContactProfile,
        coarse_scout_tick: Option<u16>,
    ) -> Self {
        let (acceptance_low, acceptance_high) =
            adaptive_contact_acceptance_bounds(profile, coarse_scout_tick);
        Self::with_acceptance(
            start_position,
            baseline,
            profile.probe_sign,
            acceptance_low,
            acceptance_high,
        )
    }

    fn with_acceptance(
        start_position: u16,
        baseline: BaselineStats,
        probe_sign: i8,
        acceptance_low: u16,
        acceptance_high: u16,
    ) -> Self {
        Self {
            start_position,
            previous_position: start_position,
            baseline,
            config: HybridContactConfig::default(),
            probe_sign,
            confirming_samples: 0,
            active_target: None,
            target_samples_seen: 0,
            acceptance_low,
            acceptance_high,
        }
    }

    fn observe(&mut self, observation: MotorObservation, commanded_target: u16) -> ContactState {
        if observation.has_driver_error
            || observation.status != 0
            || !observation.torque_enabled
            || observation.torque_limit != TORQUE_LIMIT
            || observation.goal_position != commanded_target
            || observation.current >= self.config.hard_current_abort_raw
        {
            return ContactState::HardAbort;
        }

        if self.active_target != Some(commanded_target) {
            self.active_target = Some(commanded_target);
            self.target_samples_seen = 0;
            self.previous_position = observation.position;
            self.confirming_samples = 0;
            return ContactState::FreeMotion;
        }
        self.target_samples_seen = self.target_samples_seen.saturating_add(1);

        let travel =
            directional_progress(observation.position, self.start_position, self.probe_sign);
        let progress = directional_progress(
            observation.position,
            self.previous_position,
            self.probe_sign,
        );
        self.previous_position = observation.position;
        let low_velocity = speed_magnitude(observation.velocity) <= self.config.max_velocity_raw;
        let low_progress = progress <= self.config.max_progress_ticks;
        let enough_travel = travel >= self.config.min_travel_ticks;
        let goal_error = circular_distance(observation.position, commanded_target);
        let inside_acceptance =
            (self.acceptance_low..=self.acceptance_high).contains(&observation.position);
        let target_settle_tolerance = if inside_acceptance {
            self.config.target_reached_tolerance_ticks
        } else {
            OUTSIDE_CORRIDOR_SETTLE_TOLERANCE_TICKS
        };
        let target_ahead = i32::from(signed_tick_delta(commanded_target, observation.position))
            * i32::from(self.probe_sign)
            > 0;
        let _current_supports_contact = observation.current >= self.baseline.contact_threshold();

        if goal_error <= target_settle_tolerance {
            self.confirming_samples = 0;
            return ContactState::FreeMotion;
        }
        if self.target_samples_seen <= TARGET_STARTUP_SAMPLES {
            self.confirming_samples = 0;
            return ContactState::FreeMotion;
        }

        if enough_travel && low_progress && low_velocity && target_ahead {
            self.confirming_samples = self.confirming_samples.saturating_add(1);
            if self.confirming_samples >= self.config.persistence_samples {
                if inside_acceptance {
                    ContactState::ContactConfirmed
                } else {
                    ContactState::EarlyStall
                }
            } else {
                ContactState::ContactSuspected
            }
        } else {
            self.confirming_samples = 0;
            ContactState::FreeMotion
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ContactResult {
    coarse_scout_tick: u16,
    first_tick: u16,
    second_tick: u16,
    spread_ticks: u16,
    baseline: BaselineStats,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct DualContactResult {
    minimum: ContactResult,
    maximum: ContactResult,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ModelZeroEstimate {
    joint_name: &'static str,
    motor_id: u8,
    minimum_contact_tick: u16,
    maximum_contact_tick: u16,
    zero_from_minimum_tick: u16,
    zero_from_maximum_tick: u16,
    endpoint_disagreement_ticks: u16,
    estimated_zero_tick: u16,
    shift_from_digital_home_ticks: u16,
    accepted: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct AffineJointCalibration {
    joint_name: &'static str,
    motor_id: u8,
    minimum_contact_tick: u16,
    maximum_contact_tick: u16,
    expected_span_ticks: u16,
    measured_span_ticks: u16,
    scale_permille: u16,
    estimated_zero_tick: u16,
    shift_from_digital_home_ticks: u16,
    accepted: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct JointCalibrationEvidence {
    spec: JointSpec,
    contacts: DualContactResult,
    fixed_scale: ModelZeroEstimate,
    affine: AffineJointCalibration,
    contact_witness_accepted: bool,
    accepted: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct LfCalibrationOutcome {
    joints: [JointCalibrationEvidence; 3],
}

fn lf_reference_contact_ticks(joint: JointKind) -> (u16, u16) {
    match joint {
        JointKind::Hip => (2535, 1617),
        JointKind::Upper => (1443, 3442),
        JointKind::Lower => (3093, 1666),
    }
}

fn lf_contact_witness_deviations(joint: JointKind, contacts: DualContactResult) -> (u16, u16) {
    let (minimum, maximum) = lf_reference_contact_ticks(joint);
    (
        circular_distance(contact_result_tick(contacts.minimum), minimum),
        circular_distance(contact_result_tick(contacts.maximum), maximum),
    )
}

fn lf_contact_witness_accepted(joint: JointKind, contacts: DualContactResult) -> bool {
    let (minimum, maximum) = lf_contact_witness_deviations(joint, contacts);
    minimum <= LF_CONTACT_WITNESS_TOLERANCE_TICKS && maximum <= LF_CONTACT_WITNESS_TOLERANCE_TICKS
}

fn derive_affine_joint_calibration(
    spec: JointSpec,
    contacts: DualContactResult,
) -> AffineJointCalibration {
    let minimum_contact_tick = contact_result_tick(contacts.minimum);
    let maximum_contact_tick = contact_result_tick(contacts.maximum);
    let expected_span_ticks = spec.max_delta.abs_diff(spec.min_delta);
    let measured_span_ticks =
        directional_progress(maximum_contact_tick, minimum_contact_tick, spec.direction);
    let scale_permille = if expected_span_ticks == 0 {
        0
    } else {
        ((u32::from(measured_span_ticks) * 1000 + u32::from(expected_span_ticks) / 2)
            / u32::from(expected_span_ticks)) as u16
    };
    let zero_numerator = i32::from(-spec.min_delta) * i32::from(measured_span_ticks);
    let zero_denominator = i32::from(spec.max_delta - spec.min_delta);
    let zero_distance = if zero_denominator == 0 {
        0
    } else {
        (zero_numerator + zero_denominator / 2).div_euclid(zero_denominator)
    };
    let estimated_zero =
        i32::from(minimum_contact_tick) + i32::from(spec.direction) * zero_distance;
    let estimated_zero_tick = estimated_zero.clamp(0, i32::from(protocol::MAX_ANGLE_STEP)) as u16;
    let shift_from_digital_home_ticks = circular_distance(estimated_zero_tick, HOME_TICK);
    let accepted = (AFFINE_SCALE_MIN_PERMILLE..=AFFINE_SCALE_MAX_PERMILLE)
        .contains(&scale_permille)
        && shift_from_digital_home_ticks <= MODEL_ZERO_MAX_SHIFT_FROM_DIGITAL_HOME_TICKS;

    AffineJointCalibration {
        joint_name: spec.name,
        motor_id: spec.motor_id,
        minimum_contact_tick,
        maximum_contact_tick,
        expected_span_ticks,
        measured_span_ticks,
        scale_permille,
        estimated_zero_tick,
        shift_from_digital_home_ticks,
        accepted,
    }
}

fn ticks_to_degrees(ticks: i32) -> f64 {
    f64::from(ticks) * 360.0 / f64::from(TICKS_PER_REVOLUTION)
}

fn fixed_q0_correction_degrees(estimate: ModelZeroEstimate, spec: JointSpec) -> f64 {
    f64::from(spec.direction)
        * ticks_to_degrees(i32::from(signed_tick_delta(
            estimate.estimated_zero_tick,
            HOME_TICK,
        )))
}

fn affine_q0_correction_degrees(calibration: AffineJointCalibration, spec: JointSpec) -> f64 {
    let scale = f64::from(calibration.scale_permille) / 1000.0;
    f64::from(spec.direction)
        * ticks_to_degrees(i32::from(signed_tick_delta(
            calibration.estimated_zero_tick,
            HOME_TICK,
        )))
        / scale
}

fn urdf_span_degrees(spec: JointSpec) -> f64 {
    ticks_to_degrees(i32::from(spec.max_delta - spec.min_delta))
}

fn measured_span_degrees(calibration: AffineJointCalibration) -> f64 {
    ticks_to_degrees(i32::from(calibration.measured_span_ticks))
}

fn affine_ticks_per_degree(calibration: AffineJointCalibration, spec: JointSpec) -> f64 {
    f64::from(calibration.measured_span_ticks) / urdf_span_degrees(spec)
}

fn affine_endpoint_residual_degrees(
    calibration: AffineJointCalibration,
    spec: JointSpec,
    side: ContactSide,
) -> f64 {
    let q_delta = f64::from(spec.limit_delta(side));
    let scaled_delta = q_delta * f64::from(calibration.scale_permille) / 1000.0;
    let predicted =
        f64::from(calibration.estimated_zero_tick) + f64::from(spec.direction) * scaled_delta;
    let measured = f64::from(match side {
        ContactSide::Min => calibration.minimum_contact_tick,
        ContactSide::Max => calibration.maximum_contact_tick,
    });
    (measured - predicted) * 360.0 / f64::from(TICKS_PER_REVOLUTION)
}

fn derive_joint_evidence(spec: JointSpec, contacts: DualContactResult) -> JointCalibrationEvidence {
    let fixed_scale = derive_model_zero(spec, contacts);
    let affine = derive_affine_joint_calibration(spec, contacts);
    let contact_witness_accepted = lf_contact_witness_accepted(spec.kind, contacts);
    JointCalibrationEvidence {
        spec,
        contacts,
        fixed_scale,
        affine,
        contact_witness_accepted,
        // Affine span/q0 is authoritative; fixed-scale disagreement remains a
        // diagnostic. The uniform hardware witness rejects temporary obstacles.
        accepted: affine.accepted && contact_witness_accepted,
    }
}

fn lf_machine_profile_record(evidence: JointCalibrationEvidence) -> String {
    let spec = evidence.spec;
    let fixed = evidence.fixed_scale;
    let affine = evidence.affine;
    let minimum =
        build_profile(Leg::Lf, spec.kind, ContactSide::Min).expect("validated LF MIN profile");
    let maximum =
        build_profile(Leg::Lf, spec.kind, ContactSide::Max).expect("validated LF MAX profile");
    format!(
        "MATDOG_LF_PROFILE_V1|joint={}|joint_name={}|motor_id={}|direction={}|urdf_min_delta={}|urdf_max_delta={}|urdf_min_tick={}|urdf_max_tick={}|coarse_min={}|coarse_max={}|fine_min_1={}|fine_min_2={}|fine_max_1={}|fine_max_2={}|repeatability_min={}|repeatability_max={}|contact_min={}|contact_max={}|q0_fixed={}|q0_affine={}|endpoint_disagreement={}|q0_shift={}|scale_permille={}|safe_min_tick={}|safe_max_tick={}|accepted={}",
        spec.kind.label(),
        spec.name,
        spec.motor_id,
        spec.direction,
        spec.min_delta,
        spec.max_delta,
        minimum.urdf_limit_tick,
        maximum.urdf_limit_tick,
        evidence.contacts.minimum.coarse_scout_tick,
        evidence.contacts.maximum.coarse_scout_tick,
        evidence.contacts.minimum.first_tick,
        evidence.contacts.minimum.second_tick,
        evidence.contacts.maximum.first_tick,
        evidence.contacts.maximum.second_tick,
        evidence.contacts.minimum.spread_ticks,
        evidence.contacts.maximum.spread_ticks,
        fixed.minimum_contact_tick,
        fixed.maximum_contact_tick,
        fixed.estimated_zero_tick,
        affine.estimated_zero_tick,
        fixed.endpoint_disagreement_ticks,
        fixed.shift_from_digital_home_ticks,
        affine.scale_permille,
        minimum.urdf_limit_tick.min(maximum.urdf_limit_tick),
        minimum.urdf_limit_tick.max(maximum.urdf_limit_tick),
        evidence.accepted,
    )
}

fn joint_degree_evidence(evidence: JointCalibrationEvidence) -> String {
    let spec = evidence.spec;
    let fixed = evidence.fixed_scale;
    let affine = evidence.affine;
    let urdf_min_degrees = ticks_to_degrees(i32::from(spec.min_delta));
    let urdf_max_degrees = ticks_to_degrees(i32::from(spec.max_delta));
    let disagreement_degrees = ticks_to_degrees(i32::from(fixed.endpoint_disagreement_ticks));
    let scale_percent = (f64::from(affine.scale_permille) - 1000.0) / 10.0;
    let measured_degrees = measured_span_degrees(affine);
    let urdf_degrees = urdf_span_degrees(spec);
    format!(
        "LF {} {} M{} | URDF MIN/MAX {:+.2}°/{:+.2}° | coarse scout MIN/MAX {}/{} tick (discarded) | fine MIN {}/{} tick, fine MAX {}/{} tick | fixed q0 MIN={} MAX={} midpoint={} correction={:+.2}° disagreement={:.2}° (limit {:.2}°) | affine {} M{} q0={} correction={:+.2}° scale={:.4} tick/° ({:+.1}% nominal) span_ticks={}/{} q0_shift={} measured_range={:.2}° URDF_range={:.2}° range_error={:+.2}° residual_MIN={:+.2}° residual_MAX={:+.2}° | Q0_DIAGNOSTIC: {} ({})",
        spec.kind.label(),
        fixed.joint_name,
        fixed.motor_id,
        urdf_min_degrees,
        urdf_max_degrees,
        evidence.contacts.minimum.coarse_scout_tick,
        evidence.contacts.maximum.coarse_scout_tick,
        evidence.contacts.minimum.first_tick,
        evidence.contacts.minimum.second_tick,
        evidence.contacts.maximum.first_tick,
        evidence.contacts.maximum.second_tick,
        fixed.zero_from_minimum_tick,
        fixed.zero_from_maximum_tick,
        fixed.estimated_zero_tick,
        fixed_q0_correction_degrees(fixed, spec),
        disagreement_degrees,
        ticks_to_degrees(i32::from(MODEL_ZERO_ENDPOINT_CONSISTENCY_TICKS)),
        affine.joint_name,
        affine.motor_id,
        affine.estimated_zero_tick,
        affine_q0_correction_degrees(affine, spec),
        affine_ticks_per_degree(affine, spec),
        scale_percent,
        affine.measured_span_ticks,
        affine.expected_span_ticks,
        affine.shift_from_digital_home_ticks,
        measured_degrees,
        urdf_degrees,
        measured_degrees - urdf_degrees,
        affine_endpoint_residual_degrees(affine, spec, ContactSide::Min),
        affine_endpoint_residual_degrees(affine, spec, ContactSide::Max),
        if evidence.accepted { "ACCEPT" } else { "REJECT" },
        if !evidence.contact_witness_accepted {
            "contact differs from the supervised LF hardware witness"
        } else if !affine.accepted {
            "affine span/q0 gate is outside its global reference band"
        } else if !fixed.accepted {
            "affine gate accepted; fixed-scale disagreement retained as diagnostic"
        } else {
            "fine contacts agree with hardware witness, fixed scale and affine checks"
        },
    )
}

fn circular_midpoint_tick(first: u16, second: u16) -> u16 {
    let delta = i32::from(signed_tick_delta(second, first));
    // Divide the doubled unwrapped sum, not delta alone. This makes the
    // integer midpoint symmetric for odd one-tick pairs such as 3443/3442.
    let midpoint = (i32::from(first) * 2 + delta).div_euclid(2);
    midpoint.rem_euclid(TICKS_PER_REVOLUTION) as u16
}

fn contact_result_tick(result: ContactResult) -> u16 {
    circular_midpoint_tick(result.first_tick, result.second_tick)
}

fn zero_candidate_from_contact(spec: JointSpec, side: ContactSide, contact_tick: u16) -> u16 {
    let q_delta = spec.limit_delta(side);
    let candidate = i32::from(contact_tick) - i32::from(spec.direction) * i32::from(q_delta);
    candidate.rem_euclid(TICKS_PER_REVOLUTION) as u16
}

fn derive_model_zero(spec: JointSpec, contacts: DualContactResult) -> ModelZeroEstimate {
    let minimum_contact_tick = contact_result_tick(contacts.minimum);
    let maximum_contact_tick = contact_result_tick(contacts.maximum);
    let zero_from_minimum_tick =
        zero_candidate_from_contact(spec, ContactSide::Min, minimum_contact_tick);
    let zero_from_maximum_tick =
        zero_candidate_from_contact(spec, ContactSide::Max, maximum_contact_tick);
    let endpoint_disagreement_ticks =
        circular_distance(zero_from_minimum_tick, zero_from_maximum_tick);
    let estimated_zero_tick =
        circular_midpoint_tick(zero_from_minimum_tick, zero_from_maximum_tick);
    let shift_from_digital_home_ticks = circular_distance(estimated_zero_tick, HOME_TICK);
    let accepted = endpoint_disagreement_ticks <= MODEL_ZERO_ENDPOINT_CONSISTENCY_TICKS
        && shift_from_digital_home_ticks <= MODEL_ZERO_MAX_SHIFT_FROM_DIGITAL_HOME_TICKS;
    ModelZeroEstimate {
        joint_name: spec.name,
        motor_id: spec.motor_id,
        minimum_contact_tick,
        maximum_contact_tick,
        zero_from_minimum_tick,
        zero_from_maximum_tick,
        endpoint_disagreement_ticks,
        estimated_zero_tick,
        shift_from_digital_home_ticks,
        accepted,
    }
}

fn is_allowed_matdog_ram_register(register: RamRegister) -> bool {
    matches!(
        register,
        RamRegister::TorqueEnable
            | RamRegister::Acc
            | RamRegister::GoalPosition
            | RamRegister::GoalSpeed
            | RamRegister::TorqueLimit
    )
}

fn validate_ram_write(register: RamRegister, value: &[u8]) -> Result<(), DynError> {
    if !is_allowed_matdog_ram_register(register) {
        return Err(format!("MATDOG RAM write is not allowlisted: {}", register.name()).into());
    }
    if value.len() != register.size() as usize {
        return Err(format!(
            "MATDOG RAM write size mismatch for {}: expected={}, actual={}",
            register.name(),
            register.size(),
            value.len()
        )
        .into());
    }
    Ok(())
}

fn global_torque_off_writes() -> Vec<(u8, Vec<u8>)> {
    MATDOG_MOTOR_IDS
        .iter()
        .map(|&motor_id| (motor_id, vec![0]))
        .collect()
}

pub async fn auto_calibrate(
    target_bus_serial: String,
    found_motors: Vec<u8>,
    comm: Arc<ST3215BusCommunicator>,
) -> Result<Arc<AtomicBool>, Box<dyn std::error::Error>> {
    if !is_exact_matdog_motor_set(&found_motors) {
        return Err(format!(
            "MATDOG requires exact IDs {:?}; found {:?}",
            MATDOG_MOTOR_IDS, found_motors
        )
        .into());
    }
    let profile =
        active_profile().map_err(|message| -> Box<dyn std::error::Error> { message.into() })?;

    let (inference_tx, inference_rx) = watch::channel(InferenceState::default());
    let inference_queue_id = comm.normfs.resolve("st3215/inference");
    let normfs = comm.normfs.clone();
    tokio::spawn(async move {
        let _ = normfs.subscribe(
            &inference_queue_id,
            Box::new(move |entries: &[(normfs::UintN, bytes::Bytes)]| {
                for (_, data) in entries {
                    if let Ok(state) = InferenceState::decode(data.as_ref()) {
                        if inference_tx.send(state).is_err() {
                            return false;
                        }
                    }
                }
                true
            }),
        );
    });

    let stop_requested = Arc::new(AtomicBool::new(false));
    let stop_flag = stop_requested.clone();
    let serial_for_task = target_bus_serial.clone();
    let serial_for_cleanup = target_bus_serial.clone();
    let comm_for_cleanup = comm.clone();

    tokio::spawn(async move {
        let result = if is_lf_full_sequence(&profile) {
            run_lf_full_calibration(
                profile,
                serial_for_task,
                found_motors,
                comm,
                inference_rx,
                stop_requested,
            )
            .await
        } else if is_lf_hip_sequence(&profile) {
            run_lf_hip_min_max(
                profile,
                serial_for_task,
                found_motors,
                comm,
                inference_rx,
                stop_requested,
            )
            .await
        } else {
            run_profile(
                profile,
                serial_for_task,
                found_motors,
                comm,
                inference_rx,
                stop_requested,
            )
            .await
        };
        if let Err(err) = result {
            error!("MATDOG native profile failed: {err}");
        }
        comm_for_cleanup.clear_calibration_stop(&serial_for_cleanup);
    });

    Ok(stop_flag)
}

async fn run_profile(
    profile: ContactProfile,
    target_bus_serial: String,
    found_motors: Vec<u8>,
    comm: Arc<ST3215BusCommunicator>,
    inference_rx: watch::Receiver<InferenceState>,
    stop_requested: Arc<AtomicBool>,
) -> Result<(), DynError> {
    if !is_exact_matdog_motor_set(&found_motors) {
        return Err("MATDOG exact motor set changed before profile start".into());
    }

    let mut calibrator = MatdogRamOnlyCalibrator::new(
        profile,
        target_bus_serial.clone(),
        comm.clone(),
        inference_rx,
        stop_requested,
    );
    execute_single_contact_session(&mut calibrator).await
}

/// The single-contact session, from its step-zero preflight to its terminal
/// event. Extracted so the offline trace regression drives exactly this code
/// rather than a reconstruction of it.
async fn execute_single_contact_session(
    calibrator: &mut MatdogRamOnlyCalibrator,
) -> Result<(), DynError> {
    // Structural, authority-free progress envelope. A recognized but rejected
    // legacy profile therefore still publishes its historical preflight, still
    // enters the production executor, and still reaches the unconditional
    // verified global torque-OFF below. The reviewed LF grammar total is
    // validated where success is declared, in mark_done.
    calibrator.total_steps = single_contact_progress_envelope(&calibrator.profile);
    calibrator.publish_progress(
        0,
        "MATDOG native profile preflight",
        CalibrationStatus::InProgress,
        None,
    );

    let result = calibrator.run().await.map_err(|err| err.to_string());
    let cleanup = calibrator
        .global_torque_off_verified()
        .await
        .map_err(|err| err.to_string());
    match (result, cleanup) {
        (Ok(contact), Ok(())) => {
            info!(
                "MATDOG {} complete: first={}, second={}, spread={}, baseline_median={}, baseline_mad={}",
                calibrator.profile.label,
                contact.first_tick,
                contact.second_tick,
                contact.spread_ticks,
                contact.baseline.median_current,
                contact.baseline.mad_current
            );
            match calibrator.mark_done() {
                Ok(()) => Ok(()),
                Err(incomplete) => {
                    let message = incomplete.to_string();
                    calibrator.mark_failed(&message);
                    Err(message.into())
                }
            }
        }
        (Err(run_err), Ok(())) => {
            calibrator.mark_failed(&run_err);
            Err(run_err.into())
        }
        (Ok(_), Err(cleanup_err)) => {
            let message =
                format!("MATDOG profile completed but torque-OFF cleanup failed: {cleanup_err}");
            calibrator.mark_failed(&message);
            Err(message.into())
        }
        (Err(run_err), Err(cleanup_err)) => {
            let message = format!("{run_err}; torque-OFF cleanup also failed: {cleanup_err}");
            calibrator.mark_failed(&message);
            Err(message.into())
        }
    }
}

async fn run_lf_hip_min_max(
    minimum_profile: ContactProfile,
    target_bus_serial: String,
    found_motors: Vec<u8>,
    comm: Arc<ST3215BusCommunicator>,
    inference_rx: watch::Receiver<InferenceState>,
    stop_requested: Arc<AtomicBool>,
) -> Result<(), DynError> {
    if !is_exact_matdog_motor_set(&found_motors) {
        return Err("MATDOG exact motor set changed before LF HIP sequence start".into());
    }
    if !is_lf_hip_sequence(&minimum_profile) || minimum_profile.side != ContactSide::Min {
        return Err("LF HIP sequence did not start from its reviewed MIN profile".into());
    }
    let maximum_profile = lf_hip_sequence_profile(ContactSide::Max)
        .map_err(|message| -> DynError { message.into() })?;

    let mut calibrator = MatdogRamOnlyCalibrator::new(
        minimum_profile,
        target_bus_serial,
        comm,
        inference_rx,
        stop_requested,
    );
    execute_hip_pair_session(&mut calibrator, maximum_profile).await
}

/// The hip-pair session, from its step-zero preflight to its terminal event.
async fn execute_hip_pair_session(
    calibrator: &mut MatdogRamOnlyCalibrator,
    maximum_profile: ContactProfile,
) -> Result<(), DynError> {
    calibrator.total_steps = calibrator.derive_expected_progress_total()?;
    calibrator.publish_progress(
        0,
        "LF HIP MIN+MAX shared-geometry preflight",
        CalibrationStatus::InProgress,
        None,
    );

    let result = calibrator
        .run_lf_hip_min_max(maximum_profile)
        .await
        .map_err(|err| err.to_string());
    let cleanup = calibrator
        .global_torque_off_verified()
        .await
        .map_err(|err| err.to_string());

    match (result, cleanup) {
        (Ok(contacts), Ok(())) => {
            info!(
                "MATDOG {} complete: MIN first={}, second={}, spread={}, baseline_median={}, baseline_mad={}; MAX first={}, second={}, spread={}, baseline_median={}, baseline_mad={}",
                LF_HIP_SEQUENCE_ARM_VALUE,
                contacts.minimum.first_tick,
                contacts.minimum.second_tick,
                contacts.minimum.spread_ticks,
                contacts.minimum.baseline.median_current,
                contacts.minimum.baseline.mad_current,
                contacts.maximum.first_tick,
                contacts.maximum.second_tick,
                contacts.maximum.spread_ticks,
                contacts.maximum.baseline.median_current,
                contacts.maximum.baseline.mad_current
            );
            match calibrator.mark_done() {
                Ok(()) => Ok(()),
                Err(incomplete) => {
                    let message = incomplete.to_string();
                    calibrator.mark_failed(&message);
                    Err(message.into())
                }
            }
        }
        (Err(run_err), Ok(())) => {
            calibrator.mark_failed(&run_err);
            Err(run_err.into())
        }
        (Ok(_), Err(cleanup_err)) => {
            let message = format!(
                "MATDOG LF HIP sequence completed but torque-OFF cleanup failed: {cleanup_err}"
            );
            calibrator.mark_failed(&message);
            Err(message.into())
        }
        (Err(run_err), Err(cleanup_err)) => {
            let message = format!("{run_err}; torque-OFF cleanup also failed: {cleanup_err}");
            calibrator.mark_failed(&message);
            Err(message.into())
        }
    }
}

async fn run_lf_full_calibration(
    sentinel: ContactProfile,
    target_bus_serial: String,
    found_motors: Vec<u8>,
    comm: Arc<ST3215BusCommunicator>,
    inference_rx: watch::Receiver<InferenceState>,
    stop_requested: Arc<AtomicBool>,
) -> Result<(), DynError> {
    if !is_exact_matdog_motor_set(&found_motors) {
        return Err("MATDOG exact motor set changed before LF native calibration".into());
    }
    if !is_lf_full_sequence(&sentinel) {
        return Err("LF native calibration did not receive its exact arm sentinel".into());
    }

    let mut calibrator = MatdogRamOnlyCalibrator::new(
        sentinel,
        target_bus_serial,
        comm,
        inference_rx,
        stop_requested,
    );
    execute_full_leg_session(&mut calibrator).await
}

/// The full-leg session, from its step-zero preflight to its terminal event.
async fn execute_full_leg_session(
    calibrator: &mut MatdogRamOnlyCalibrator,
) -> Result<(), DynError> {
    calibrator.total_steps = calibrator.derive_expected_progress_total()?;
    calibrator.publish_progress(
        0,
        "single-session LF native calibration preflight",
        CalibrationStatus::InProgress,
        None,
    );

    let run_result = calibrator
        .run_lf_state_machine()
        .await
        .map_err(|err| err.to_string());
    match run_result {
        Ok(outcome) => {
            let cleanup = calibrator
                .global_torque_off_verified()
                .await
                .map_err(|err| err.to_string());
            match cleanup {
                Ok(()) => {
                    for evidence in outcome.joints {
                        info!("MATDOG LF EVIDENCE: {}", joint_degree_evidence(evidence));
                        info!("{}", lf_machine_profile_record(evidence));
                    }
                    info!(
                        "MATDOG {} measurement complete: M13_q0_fixed={}, M12_q0_fixed={}, M11_q0_fixed={}, M13_q0_affine={}, M12_q0_affine={}, M11_q0_affine={}, status=LF_STAGED, movement_RAM_only=true, EEPROM_written=false",
                        LF_FULL_SEQUENCE_ARM_VALUE,
                        outcome.joints[0].fixed_scale.estimated_zero_tick,
                        outcome.joints[1].fixed_scale.estimated_zero_tick,
                        outcome.joints[2].fixed_scale.estimated_zero_tick,
                        outcome.joints[0].affine.estimated_zero_tick,
                        outcome.joints[1].affine.estimated_zero_tick,
                        outcome.joints[2].affine.estimated_zero_tick,
                    );
                    match calibrator.mark_done() {
                        Ok(()) => Ok(()),
                        Err(incomplete) => {
                            let message = incomplete.to_string();
                            calibrator.mark_failed(&message);
                            Err(message.into())
                        }
                    }
                }
                Err(cleanup_err) => {
                    let message = format!(
                        "LF native calibration completed but final torque-OFF failed: {cleanup_err}"
                    );
                    calibrator.mark_failed(&message);
                    Err(message.into())
                }
            }
        }
        Err(run_err) => {
            let cleanup = calibrator
                .global_torque_off_verified()
                .await
                .map_err(|err| err.to_string());
            let message = match cleanup {
                Ok(()) => run_err,
                Err(cleanup_err) => {
                    format!("{run_err}; immediate global torque-OFF also failed: {cleanup_err}")
                }
            };
            calibrator.mark_failed(&message);
            Err(message.into())
        }
    }
}

/// Which historical write envelope the unique raw GoalPosition constructor must
/// use. The two transports keep their different motor-admission gates (CG-7).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum GoalWriteRoute {
    StartupHome,
    ArmedProfile,
}

/// Coarse or fine contact-search pass. The step size and the scout policy are
/// DERIVED from this plus the validated session mode; neither is ever a caller
/// choice (G2 section 8.4).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ProbePass {
    Coarse,
    Fine,
}

// ===========================================================================
// P2A-G3-C — GRAMMAR-DERIVED PROGRESS TOTAL
//
// G2 PRG-2 / PRG-5 / PRG-6: `expected_total` is the COUNT of progress-emitting
// operations in the expanded grammar of the selected mode. It is not spec data,
// not a stored constant, and a standalone number is not sufficient evidence.
//
// The expansion below is pure and structural. It carries no tick, no pose, no
// motor and no command: it exists only to express the reviewed structure of
// section 6.1 / 6.2 so the total falls out of its length. The historical motion
// executors keep their own choreography and are not driven from it.
// ===========================================================================

/// One reviewed entry operation of the mode-specific entry sequence (§6.1).
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EntryProgress {
    ExactSetVerify,
    GlobalTorqueOff,
    /// Full session only: all-12 q=0 normalization, then session creation.
    FullNormalization,
    SessionCreation,
    /// Legacy modes only: restart-safe profile entry, home-only recovery and
    /// prerequisite establishment.
    ProfileEntryInspection,
    HomeOnlyRecovery,
    PrerequisiteEstablishment,
}

/// One phase a contact side emits. The three reviewed modes emit different
/// phase lists; each list is enumerated, never counted by a literal.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ContactSidePhase {
    MovingBaseline,
    CoarseScoutingPass,
    CoarseBackoff,
    FineMetrologyPassOne,
    FineMetrologyBackoff,
    FineMetrologyPassTwo,
    FineToFineRepeatability,
    CoarseApproach,
    BackoffAndRecovery,
    FineRepeatApproach,
    Repeatability,
}

/// The seven doubled contact-side phases of the full session (§14.5).
const LF_FULL_CONTACT_SIDE_PHASES: [ContactSidePhase; 7] = [
    ContactSidePhase::MovingBaseline,
    ContactSidePhase::CoarseScoutingPass,
    ContactSidePhase::CoarseBackoff,
    ContactSidePhase::FineMetrologyPassOne,
    ContactSidePhase::FineMetrologyBackoff,
    ContactSidePhase::FineMetrologyPassTwo,
    ContactSidePhase::FineToFineRepeatability,
];

/// The five phases a hip-pair contact side emits (§14.3): one coarse and one
/// unscouted fine pass, never two scouted fine passes.
const LF_HIP_PAIR_CONTACT_SIDE_PHASES: [ContactSidePhase; 5] = [
    ContactSidePhase::MovingBaseline,
    ContactSidePhase::CoarseApproach,
    ContactSidePhase::BackoffAndRecovery,
    ContactSidePhase::FineRepeatApproach,
    ContactSidePhase::Repeatability,
];

/// The seven phases a single-contact session emits (§14.4): one coarse scout
/// and two scouted fine passes.
const LF_SINGLE_CONTACT_SIDE_PHASES: [ContactSidePhase; 7] = [
    ContactSidePhase::MovingBaseline,
    ContactSidePhase::CoarseScoutingPass,
    ContactSidePhase::CoarseBackoff,
    ContactSidePhase::FineMetrologyPassOne,
    ContactSidePhase::FineMetrologyBackoff,
    ContactSidePhase::FineMetrologyPassTwo,
    ContactSidePhase::FineToFineRepeatability,
];

/// One progress-emitting structural component of the reviewed grammar.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ProgressStep {
    Entry(EntryProgress),
    /// Whole-session parking hold (Full only).
    Parking,
    /// A joint block's single prepare operation.
    JointPrepare(JointKind),
    /// One phase of one contact side of one joint block.
    ContactSidePhase {
        joint: JointKind,
        side: ContactSide,
        phase: ContactSidePhase,
    },
    /// Hip-pair only: the probe returns home between MIN and MAX.
    BetweenSideReturnHome,
    /// Full only: static-hold transition of a completed joint.
    StaticHold(JointKind),
    /// Full only: endpoint and affine q0 diagnostics.
    Diagnostics,
    /// Full only: staged affine q0 return of one joint.
    ReturnStaged(JointKind),
    /// Full only: the whole-session parking hold is released.
    RestoreParking,
    /// Legacy modes: the probe returns home.
    ProbeReturnHome,
    /// Legacy modes: LIFO prerequisite restore.
    RestorePrerequisites,
    /// Terminal verified global torque OFF.
    FinalTorqueOff,
}

/// Full-session joint blocks in the reviewed calibration order, and whether the
/// block is followed by a static-hold transition (§6.2).
const LF_FULL_JOINT_BLOCKS: [(JointKind, bool); 3] = [
    (JointKind::Upper, true),
    (JointKind::Lower, true),
    (JointKind::Hip, false),
];

/// Full-session staged-return order (§14.2 emissions 54-56).
const LF_FULL_STAGED_RETURN_ORDER: [JointKind; 3] =
    [JointKind::Hip, JointKind::Lower, JointKind::Upper];

/// Expand the reviewed grammar of a mode into its progress-emitting operations
/// (G2 §6.1, §6.2, §14). Pure and engine-owned.
fn lf_progress_plan(mode: LfSessionMode) -> Vec<ProgressStep> {
    let mut plan = vec![
        ProgressStep::Entry(EntryProgress::ExactSetVerify),
        ProgressStep::Entry(EntryProgress::GlobalTorqueOff),
    ];
    match mode {
        LfSessionMode::LfFullLegSession => {
            plan.push(ProgressStep::Entry(EntryProgress::FullNormalization));
            plan.push(ProgressStep::Entry(EntryProgress::SessionCreation));
            plan.push(ProgressStep::Parking);
            for (joint, static_hold) in LF_FULL_JOINT_BLOCKS {
                plan.push(ProgressStep::JointPrepare(joint));
                for side in [ContactSide::Min, ContactSide::Max] {
                    for phase in LF_FULL_CONTACT_SIDE_PHASES {
                        plan.push(ProgressStep::ContactSidePhase { joint, side, phase });
                    }
                }
                if static_hold {
                    plan.push(ProgressStep::StaticHold(joint));
                }
            }
            plan.push(ProgressStep::Diagnostics);
            for joint in LF_FULL_STAGED_RETURN_ORDER {
                plan.push(ProgressStep::ReturnStaged(joint));
            }
            plan.push(ProgressStep::RestoreParking);
        }
        LfSessionMode::LfHipPairLegacy => {
            plan.push(ProgressStep::Entry(EntryProgress::ProfileEntryInspection));
            plan.push(ProgressStep::Entry(EntryProgress::HomeOnlyRecovery));
            plan.push(ProgressStep::Entry(
                EntryProgress::PrerequisiteEstablishment,
            ));
            plan.push(ProgressStep::JointPrepare(JointKind::Hip));
            for side in [ContactSide::Min, ContactSide::Max] {
                for phase in LF_HIP_PAIR_CONTACT_SIDE_PHASES {
                    plan.push(ProgressStep::ContactSidePhase {
                        joint: JointKind::Hip,
                        side,
                        phase,
                    });
                }
                if side == ContactSide::Min {
                    plan.push(ProgressStep::BetweenSideReturnHome);
                }
            }
            plan.push(ProgressStep::ProbeReturnHome);
            plan.push(ProgressStep::RestorePrerequisites);
        }
        LfSessionMode::LfSingleContactLegacy { joint, side } => {
            plan.extend(single_contact_progress_shape(joint.joint_kind(), side));
            return plan;
        }
    }
    plan.push(ProgressStep::FinalTorqueOff);
    plan
}

/// The single-contact progress SHAPE of section 14.4, from its restart-safe
/// entry to its terminal torque-off.
///
/// PROGRESS METADATA ONLY. It names which joint and side the reviewed phases
/// belong to and carries no motor id, no tick, no pose, no GoalPosition, no
/// `EngineContext` and no motion authority. It is the single structural source
/// for both the LF grammar expansion above and the legacy wrapper's historical
/// preflight envelope, so neither needs a numeric total.
fn single_contact_progress_shape(joint: JointKind, side: ContactSide) -> Vec<ProgressStep> {
    let mut shape = vec![
        ProgressStep::Entry(EntryProgress::ProfileEntryInspection),
        ProgressStep::Entry(EntryProgress::HomeOnlyRecovery),
        ProgressStep::Entry(EntryProgress::PrerequisiteEstablishment),
        ProgressStep::JointPrepare(joint),
    ];
    for phase in LF_SINGLE_CONTACT_SIDE_PHASES {
        shape.push(ProgressStep::ContactSidePhase { joint, side, phase });
    }
    shape.push(ProgressStep::ProbeReturnHome);
    shape.push(ProgressStep::RestorePrerequisites);
    shape.push(ProgressStep::FinalTorqueOff);
    shape
}

/// The historical single-contact progress envelope: the length of that shape
/// for the profile's own joint and side, plus the two reviewed entry operations
/// every mode shares. It requires NO LF authority, so a recognized but rejected
/// legacy profile keeps its historical preflight and its unconditional
/// verified-cleanup path exactly as it had before G3-C.
fn single_contact_progress_envelope(profile: &ContactProfile) -> u32 {
    (2 + single_contact_progress_shape(profile.joint, profile.side).len()) as u32
}

/// PRG-2 / PRG-5: the expected total is the LENGTH of that expansion.
fn lf_expected_progress_total(mode: LfSessionMode) -> u32 {
    lf_progress_plan(mode).len() as u32
}

/// The engine authority context for an armed profile (G2 sections 4.4, 5.2, 8.0).
///
/// It exists only behind the sealed LF brand, and the brand exists only for the
/// six reviewed LF runtime arm values. RF/RH/LH profiles and the isolated LF hip
/// profiles therefore receive NO authority context at all, so none of the twelve
/// motion operations can emit for them (ARM-1, ARM-3r).
fn engine_authority_for_arm_value(arm_value: &str) -> Option<EngineContext> {
    let mode = lf_runtime_session_mode(arm_value).ok()?;
    let armed = validate_lf_v25(&canonical_lf_v25_raw_spec(arm_value), mode).ok()?;
    Some(EngineContext::enter(&armed))
}

/// The scout policy, as a pure rule of the validated mode and the current pass
/// (G2 section 6.2, W10 versus W11). A coarse pass never scouts. A fine pass
/// scouts in the full session and in the single-contact legacy modes, and never
/// scouts in the hip-pair legacy mode, where a scouted fine pass would add a
/// friction/chamfer continuation the immutable sequence does not have.
const fn derived_scout_policy_for(
    mode: LfSessionMode,
    pass: ProbePass,
    recorded_coarse_scout: Option<u16>,
) -> Option<u16> {
    match (pass, mode) {
        (ProbePass::Coarse, _) | (ProbePass::Fine, LfSessionMode::LfHipPairLegacy) => None,
        (ProbePass::Fine, _) => recorded_coarse_scout,
    }
}

/// Whether a full-session contact side opens the joint pair or follows a side
/// that already recorded an accepted contact. An identifier, never a tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ContactSideEntry {
    FirstOfPair,
    AfterRecordedContact,
}

/// The reviewed operation-level node whose motion the engine is executing.
///
/// Identifiers only: no tick, no pose and no evidence is stored here, so every
/// operation re-derives its own value at emit time. This is engine state, not a
/// permission that could be created now and redeemed later (AUTH-5).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum GoalNode {
    /// W5 — whole-session LF parking hold.
    Parking,
    /// W7/W8 — moving-current baseline for the active probe.
    MovingBaseline,
    /// W9/W10/W11 — contact-search advance.
    ProbeAdvance { pass: ProbePass },
    /// W14 — bounded backoff from the contact just recorded in this node.
    Backoff,
    /// W15/W16 — static-hold transition of a completed joint.
    StaticHold { joint: JointKind },
    /// W17/W18/W19 — staged affine q0 return.
    ReturnStaged { joint: JointKind },
    /// W20 — parking restore to digital home.
    RestoreParking,
    /// W21 — active probe return to digital home.
    ProbeReturnHome,
    /// W22 — post-restore probe settle at digital home.
    PostRestoreSettle,
    /// W23 — prerequisite restore to digital home, engine-owned LIFO.
    RestorePrerequisite { motor_id: u8 },
}

impl GoalNode {
    /// The settle tolerance this node has always used. `return_home` therefore
    /// keeps its node-specific tolerances: 16 for the probe return W21, 10 for
    /// the post-restore settle W22 (G2 section 8.6).
    const fn settle_tolerance(self) -> u16 {
        match self {
            Self::Backoff => STATIC_TOLERANCE_TICKS.saturating_add(2),
            Self::ProbeReturnHome => PROBE_HOME_TOLERANCE_TICKS,
            _ => STATIC_TOLERANCE_TICKS,
        }
    }
}

/// What an engine operation actually derived and emitted. Returned so the
/// surrounding motion loop observes the same values the operation committed to,
/// instead of re-deriving or being handed a free choice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct EmittedGoal {
    motor_id: u8,
    target: u16,
}

/// A completed commanded move: the goal its owning operation derived, plus the
/// observation the settle loop accepted.
#[derive(Debug, Clone, Copy)]
struct CommandedMove {
    emitted: EmittedGoal,
    observation: MotorObservation,
}

/// What a moving-baseline step derived and emitted.
///
/// `relative_entry` is the SINGLE entry observation the relative W7 rule
/// consumed: the same observation that was safety-checked and that produced the
/// target. It is reported back so the caller's sample bookkeeping continues from
/// exactly that observation and no second telemetry read is inserted between the
/// two uses. The absolute W8 rule derives no target from telemetry, reads none,
/// and reports `None`.
#[derive(Debug, Clone, Copy)]
struct BaselineStep {
    emitted: EmittedGoal,
    relative_entry: Option<MotorObservation>,
}

/// A fresh observation offered to `prime_at_present`. The engine REBINDS it
/// before use: exact expected motor, freshness gate and full safety gate, and
/// the value written is that observation's own position (DER-1).
#[derive(Debug, Clone, Copy)]
struct FreshObservation {
    motor_id: u8,
    observation: MotorObservation,
}

/// The detector verdict produced in the current approach iteration. The value
/// written by `stop_pressure_at_observation` is this verdict's own observation
/// position (DER-2).
#[derive(Debug, Clone, Copy)]
struct ContactVerdict {
    motor_id: u8,
    observation: MotorObservation,
}

struct MatdogRamOnlyCalibrator {
    profile: ContactProfile,
    target_bus_serial: String,
    comm: Arc<ST3215BusCommunicator>,
    inference_rx: watch::Receiver<InferenceState>,
    stop_requested: Arc<AtomicBool>,
    command_nonce: u64,
    command_counter: u64,
    current_step: u32,
    total_steps: u32,
    held_targets: Vec<StaticTarget>,
    probe_home_handoff_active: bool,
    lf_session: Option<LegSessionStateMachine>,
    /// Engine-owned authority context (G2 section 8.0), created from the sealed
    /// `ArmableLfSessionSpec`. It is `None` whenever the armed value is not one
    /// of the six reviewed LF runtime tokens, which makes every one of the
    /// twelve motion operations fail closed for RF/RH/LH and for the isolated
    /// LF hip profiles (ARM-1, ARM-3r). It is engine state, never a
    /// transferable permission (CTX-1).
    context: Option<EngineContext>,
    /// The reviewed operation-level node whose motion is currently executing.
    /// Identifiers only — no tick, pose or evidence is stored here.
    goal_node: Option<GoalNode>,
    /// The motor primed by W2, so W3 can prove it reasserts the same motor.
    legacy_primed_motor: Option<u8>,
    /// The last GoalPosition commanded by `probe_advance_step` in this contact
    /// search, so the next step derives `advance_tick(previous_target, ..)`.
    probe_target: Option<u16>,
    /// The contact tick the approach that just completed recorded in this node,
    /// so `backoff_step` derives its own value (section 8.6).
    node_contact_tick: Option<u16>,
    /// The accepted contact tick recorded earlier in THIS session, so
    /// `stop_pressure_at_recorded_contact` derives its own value.
    recorded_contact_tick: Option<u16>,
    /// The coarse scout depth recorded in this contact side, so the fine-pass
    /// scout policy is derived rather than chosen.
    recorded_coarse_scout: Option<u16>,
    /// G3-C: the ordered progress events this engine actually published.
    #[cfg(test)]
    emitted_progress: std::sync::Mutex<Vec<(String, u32, CalibrationStatus)>>,
}

impl MatdogRamOnlyCalibrator {
    fn new(
        profile: ContactProfile,
        target_bus_serial: String,
        comm: Arc<ST3215BusCommunicator>,
        inference_rx: watch::Receiver<InferenceState>,
        stop_requested: Arc<AtomicBool>,
    ) -> Self {
        let context = engine_authority_for_arm_value(&profile.arm_value);
        Self {
            profile,
            target_bus_serial,
            comm,
            inference_rx,
            stop_requested,
            command_nonce: systime::get_monotonic_stamp_ns(),
            command_counter: 0,
            current_step: 0,
            total_steps: 0,
            held_targets: Vec::new(),
            probe_home_handoff_active: false,
            lf_session: None,
            context,
            goal_node: None,
            legacy_primed_motor: None,
            probe_target: None,
            node_contact_tick: None,
            recorded_contact_tick: None,
            recorded_coarse_scout: None,
            #[cfg(test)]
            emitted_progress: std::sync::Mutex::new(Vec::new()),
        }
    }

    async fn run(&mut self) -> Result<ContactResult, DynError> {
        self.next_phase("Verify exact MATDOG ID set")?;
        self.wait_for_exact_motor_set().await?;

        self.next_phase("Verified global torque OFF")?;
        self.global_torque_off_verified().await?;

        self.next_phase("Inspect restart-safe profile entry")?;
        let mut entry_plan = self.inspect_profile_entry().await?;

        self.next_phase("Recover home-only joints to digital home")?;
        self.recover_home_only_joints(&mut entry_plan).await?;

        self.next_phase("Establish geometry prerequisites from restart-safe state")?;
        self.establish_prerequisites_restart_safe(&entry_plan)
            .await?;
        self.begin_engine_session(GrammarNode::Prerequisites)?;

        self.next_phase("Prime and return probing joint home")?;
        self.prepare_motor(self.profile.motor_id).await?;
        self.move_motor_to(GoalNode::ProbeReturnHome).await?;

        self.next_phase("Acquire moving-current baseline")?;
        let baseline = self.acquire_moving_current_baseline().await?;

        self.next_phase("Coarse scouting approach — measurement discarded")?;
        let coarse_scout_tick = self
            .approach_with_scout(ProbePass::Coarse, baseline)
            .await?;

        self.next_phase("Backoff after coarse scout")?;
        self.backoff_and_verify(baseline).await?;

        self.next_phase("First fine metrology approach")?;
        let first_tick = self.approach_with_scout(ProbePass::Fine, baseline).await?;

        self.next_phase("Backoff between identical fine approaches")?;
        self.backoff_and_verify(baseline).await?;

        self.next_phase("Second fine metrology approach")?;
        let second_tick = self.approach_with_scout(ProbePass::Fine, baseline).await?;
        self.recorded_contact_tick = Some(second_tick);

        self.next_phase("Verify fine-to-fine repeatability")?;
        let spread_ticks = repeatability_spread(first_tick, second_tick)?;

        self.next_phase("Return probing joint home")?;
        self.stop_pressure_at_recorded_contact().await?;
        self.move_motor_to(GoalNode::ProbeReturnHome).await?;
        self.set_motor_torque_verified(self.profile.motor_id, false)
            .await?;
        self.probe_home_handoff_active = true;

        self.next_phase("Restore prerequisite joints one at a time")?;
        self.restore_prerequisites().await?;
        self.probe_home_handoff_active = false;

        // Restoring the upper link can passively pull the torque-off lower
        // probe a few ticks away from digital home. Re-prime only that probe,
        // settle it tightly at home, then release and verify the off-state.
        self.prepare_motor(self.profile.motor_id).await?;
        self.move_motor_to(GoalNode::PostRestoreSettle).await?;
        self.set_motor_torque_verified(self.profile.motor_id, false)
            .await?;
        let probe_at_rest = self.latest_observation(self.profile.motor_id)?;
        self.ensure_observation_fresh(self.profile.motor_id, probe_at_rest)?;
        if circular_distance(probe_at_rest.position, HOME_TICK) > PROBE_HOME_TOLERANCE_TICKS {
            return Err(format!(
                "M{} post-restore home settle failed: present={}, expected={}, tolerance={}",
                self.profile.motor_id,
                probe_at_rest.position,
                HOME_TICK,
                PROBE_HOME_TOLERANCE_TICKS
            )
            .into());
        }

        self.next_phase("Final verified global torque OFF")?;

        Ok(ContactResult {
            coarse_scout_tick,
            first_tick,
            second_tick,
            spread_ticks,
            baseline,
        })
    }

    async fn run_lf_hip_min_max(
        &mut self,
        maximum_profile: ContactProfile,
    ) -> Result<DualContactResult, DynError> {
        if !is_lf_hip_sequence(&self.profile)
            || self.profile.side != ContactSide::Min
            || !is_lf_hip_sequence(&maximum_profile)
            || maximum_profile.side != ContactSide::Max
            || self.profile.prerequisites != maximum_profile.prerequisites
        {
            return Err("invalid LF HIP MIN+MAX sequence profile pair".into());
        }

        self.next_phase("Verify exact MATDOG ID set")?;
        self.wait_for_exact_motor_set().await?;

        self.next_phase("Verified global torque OFF")?;
        self.global_torque_off_verified().await?;

        self.next_phase("Inspect restart-safe LF HIP sequence entry")?;
        let mut entry_plan = self.inspect_profile_entry().await?;

        self.next_phase("Recover home-only joints to digital home")?;
        self.recover_home_only_joints(&mut entry_plan).await?;

        self.next_phase("Set M12 horizontal and M11 parallel")?;
        self.establish_prerequisites_restart_safe(&entry_plan)
            .await?;
        self.begin_engine_session(GrammarNode::Prerequisites)?;

        self.next_phase("Prime LF HIP M13 at digital home")?;
        self.prepare_motor(self.profile.motor_id).await?;
        self.move_motor_to(GoalNode::ProbeReturnHome).await?;

        self.next_phase("LF HIP MIN moving-current baseline")?;
        let minimum_baseline = self.acquire_moving_current_baseline().await?;

        self.next_phase("LF HIP MIN coarse approach")?;
        let minimum_first = self
            .approach_with_scout(ProbePass::Coarse, minimum_baseline)
            .await?;

        self.next_phase("LF HIP MIN backoff and recovery")?;
        self.backoff_and_verify(minimum_baseline).await?;

        self.next_phase("LF HIP MIN fine repeat approach")?;
        let minimum_second = self
            .approach_with_scout(ProbePass::Fine, minimum_baseline)
            .await?;
        self.recorded_contact_tick = Some(minimum_second);

        self.next_phase("LF HIP MIN repeatability")?;
        let minimum_spread = repeatability_spread(minimum_first, minimum_second)?;

        self.next_phase("Return M13 home between MIN and MAX")?;
        self.stop_pressure_at_recorded_contact().await?;
        self.move_motor_to(GoalNode::ProbeReturnHome).await?;

        let minimum = ContactResult {
            coarse_scout_tick: minimum_first,
            first_tick: minimum_first,
            second_tick: minimum_second,
            spread_ticks: minimum_spread,
            baseline: minimum_baseline,
        };

        self.profile = maximum_profile;
        self.verify_profile_holds().await?;

        self.next_phase("LF HIP MAX moving-current baseline")?;
        let maximum_baseline = self.acquire_moving_current_baseline().await?;

        self.next_phase("LF HIP MAX coarse approach")?;
        let maximum_first = self
            .approach_with_scout(ProbePass::Coarse, maximum_baseline)
            .await?;

        self.next_phase("LF HIP MAX backoff and recovery")?;
        self.backoff_and_verify(maximum_baseline).await?;

        self.next_phase("LF HIP MAX fine repeat approach")?;
        let maximum_second = self
            .approach_with_scout(ProbePass::Fine, maximum_baseline)
            .await?;
        self.recorded_contact_tick = Some(maximum_second);

        self.next_phase("LF HIP MAX repeatability")?;
        let maximum_spread = repeatability_spread(maximum_first, maximum_second)?;

        self.next_phase("Return LF HIP M13 home")?;
        self.stop_pressure_at_recorded_contact().await?;
        self.move_motor_to(GoalNode::ProbeReturnHome).await?;
        self.set_motor_torque_verified(self.profile.motor_id, false)
            .await?;
        self.probe_home_handoff_active = true;

        self.next_phase("Restore M11, M12 and M42 to home")?;
        self.restore_prerequisites().await?;
        self.probe_home_handoff_active = false;

        self.prepare_motor(self.profile.motor_id).await?;
        self.move_motor_to(GoalNode::PostRestoreSettle).await?;
        self.set_motor_torque_verified(self.profile.motor_id, false)
            .await?;
        let hip_at_rest = self.latest_observation(self.profile.motor_id)?;
        self.ensure_observation_fresh(self.profile.motor_id, hip_at_rest)?;
        if circular_distance(hip_at_rest.position, HOME_TICK) > PROBE_HOME_TOLERANCE_TICKS {
            return Err(format!(
                "M{} LF HIP final home settle failed: present={}, expected={}, tolerance={}",
                self.profile.motor_id, hip_at_rest.position, HOME_TICK, PROBE_HOME_TOLERANCE_TICKS
            )
            .into());
        }

        self.next_phase("Final verified global torque OFF")?;

        Ok(DualContactResult {
            minimum,
            maximum: ContactResult {
                coarse_scout_tick: maximum_first,
                first_tick: maximum_first,
                second_tick: maximum_second,
                spread_ticks: maximum_spread,
                baseline: maximum_baseline,
            },
        })
    }

    async fn run_lf_state_machine(&mut self) -> Result<LfCalibrationOutcome, DynError> {
        self.next_phase("Verify exact MATDOG ID set once")?;
        self.wait_for_exact_motor_set().await?;

        self.next_phase("Verified global torque OFF once at session entry")?;
        self.global_torque_off_verified().await?;

        self.next_phase("Normalize every displaced MATDOG joint to q=0 with one uniform rule")?;
        self.normalize_all_matdog_joints_to_q0().await?;

        self.next_phase("Create LF state machine from verified q=0 session entry")?;
        self.inspect_lf_native_session_entry()?;
        self.begin_engine_session(GrammarNode::InitialRecovery)?;
        self.transition_lf_state(LfSessionState::InitialRecovery)?;
        self.verify_lf_session_others_except(0)?;

        self.transition_lf_state(LfSessionState::Parking)?;
        self.next_phase("Park LH upper M42 once for the complete LF session")?;
        let rear_parking = static_target(Leg::Lh, JointKind::Upper, UPPER_30_DELTA)
            .map_err(|message| -> DynError { message.into() })?;
        self.enter_goal_node(GoalNode::Parking)?;
        self.prepare_motor(rear_parking.motor_id).await?;
        // W5 — the whole-session parking move.
        self.move_lf_session_motor_to().await?;
        self.upsert_held_target(rear_parking)?;

        self.transition_lf_state(LfSessionState::UpperMin)?;
        self.next_phase("Prepare LF UPPER M12 once")?;
        if !self.latest_observation(12)?.torque_enabled {
            self.prepare_motor(12).await?;
        }
        let upper_contacts = self
            .measure_lf_joint_pair_efficient(
                build_profile(Leg::Lf, JointKind::Upper, ContactSide::Min)
                    .map_err(|message| -> DynError { message.into() })?,
                build_profile(Leg::Lf, JointKind::Upper, ContactSide::Max)
                    .map_err(|message| -> DynError { message.into() })?,
            )
            .await?;
        self.record_lf_contacts(JointKind::Upper, upper_contacts)?;

        self.transition_lf_state(LfSessionState::UpperHorizontal)?;
        self.next_phase("Transition M12 directly from MAX contact to horizontal hold")?;
        self.profile = build_profile(Leg::Lf, JointKind::Lower, ContactSide::Min)
            .map_err(|message| -> DynError { message.into() })?;
        // W15 — static-hold transition of the completed upper joint.
        let upper_hold = self
            .move_motor_to(GoalNode::StaticHold {
                joint: JointKind::Upper,
            })
            .await?
            .emitted;
        self.upsert_held_target(StaticTarget {
            motor_id: upper_hold.motor_id,
            target_tick: upper_hold.target,
        })?;

        self.transition_lf_state(LfSessionState::LowerMin)?;
        self.next_phase("Prepare LF LOWER M11 once")?;
        self.prepare_motor(11).await?;
        let lower_contacts = self
            .measure_lf_joint_pair_efficient(
                build_profile(Leg::Lf, JointKind::Lower, ContactSide::Min)
                    .map_err(|message| -> DynError { message.into() })?,
                build_profile(Leg::Lf, JointKind::Lower, ContactSide::Max)
                    .map_err(|message| -> DynError { message.into() })?,
            )
            .await?;
        self.record_lf_contacts(JointKind::Lower, lower_contacts)?;

        self.transition_lf_state(LfSessionState::LowerFolded)?;
        self.next_phase("Transition M11 directly from MAX contact to HIP parallel hold")?;
        // W16 — static-hold transition of the completed lower joint.
        let folded = self
            .move_motor_to(GoalNode::StaticHold {
                joint: JointKind::Lower,
            })
            .await?
            .emitted;
        self.upsert_held_target(StaticTarget {
            motor_id: folded.motor_id,
            target_tick: folded.target,
        })?;

        self.transition_lf_state(LfSessionState::HipMin)?;
        self.next_phase("Prepare LF HIP M13 once")?;
        self.prepare_motor(13).await?;
        let hip_contacts = self
            .measure_lf_joint_pair_efficient(
                lf_hip_sequence_profile(ContactSide::Min)
                    .map_err(|message| -> DynError { message.into() })?,
                lf_hip_sequence_profile(ContactSide::Max)
                    .map_err(|message| -> DynError { message.into() })?,
            )
            .await?;
        self.record_lf_contacts(JointKind::Hip, hip_contacts)?;

        self.transition_lf_state(LfSessionState::Diagnostics)?;
        self.next_phase("Derive endpoint and affine q0 diagnostics from all fine contacts")?;
        let hip = derive_joint_evidence(*spec_for(Leg::Lf, JointKind::Hip), hip_contacts);
        let upper = derive_joint_evidence(*spec_for(Leg::Lf, JointKind::Upper), upper_contacts);
        let lower = derive_joint_evidence(*spec_for(Leg::Lf, JointKind::Lower), lower_contacts);
        let outcome = LfCalibrationOutcome {
            joints: [hip, upper, lower],
        };
        for evidence in outcome.joints {
            self.record_lf_diagnostics(evidence.spec.kind, evidence.fixed_scale, evidence.affine)?;
            info!("MATDOG LF EVIDENCE: {}", joint_degree_evidence(evidence));
        }
        let session = self
            .lf_session
            .as_ref()
            .ok_or("LF diagnostics lost their persistent session")?;
        if !session.has_complete_evidence() {
            return Err("LF diagnostics incomplete after six accepted fine pairs".into());
        }
        info!("MATDOG LF TRACE: {}", session.trace_summary());
        for evidence in outcome.joints {
            let (minimum, maximum) =
                lf_contact_witness_deviations(evidence.spec.kind, evidence.contacts);
            info!(
                "MATDOG LF CONTACT WITNESS: {} M{} min_deviation={} max_deviation={} tolerance={} accepted={}",
                evidence.spec.kind.label(),
                evidence.spec.motor_id,
                minimum,
                maximum,
                LF_CONTACT_WITNESS_TOLERANCE_TICKS,
                evidence.contact_witness_accepted,
            );
            if !evidence.fixed_scale.accepted {
                info!(
                    "MATDOG LF FIXED-SCALE DIAGNOSTIC WARNING: {} M{} endpoint_disagreement={} limit={}; affine_q0={} scale_permille={}",
                    evidence.spec.kind.label(),
                    evidence.spec.motor_id,
                    evidence.fixed_scale.endpoint_disagreement_ticks,
                    MODEL_ZERO_ENDPOINT_CONSISTENCY_TICKS,
                    evidence.affine.estimated_zero_tick,
                    evidence.affine.scale_permille,
                );
            }
        }
        let diagnostic_rejections = outcome
            .joints
            .iter()
            .filter(|evidence| !evidence.accepted)
            .map(|evidence| {
                format!(
                    "{} M{} endpoint_disagreement={}tick/{:.2}deg affine_scale={}permille",
                    evidence.spec.kind.label(),
                    evidence.spec.motor_id,
                    evidence.fixed_scale.endpoint_disagreement_ticks,
                    ticks_to_degrees(i32::from(evidence.fixed_scale.endpoint_disagreement_ticks)),
                    evidence.affine.scale_permille,
                )
            })
            .collect::<Vec<_>>();
        if !diagnostic_rejections.is_empty() {
            return Err(format!(
                "MATDOG LF affine+witness freeze gate rejected before EEPROM staging: {}; contact witness tolerance={} ticks; affine scale reference={}..={} permille; fixed-scale endpoint consistency limit={} ticks is diagnostic only",
                diagnostic_rejections.join("; "),
                LF_CONTACT_WITNESS_TOLERANCE_TICKS,
                AFFINE_SCALE_MIN_PERMILLE,
                AFFINE_SCALE_MAX_PERMILLE,
                MODEL_ZERO_ENDPOINT_CONSISTENCY_TICKS,
            )
            .into());
        }
        info!(
            "MATDOG LF URDF FREEZE GATE: PASS; all three joints accepted by uniform contact-witness and affine span/q0 gates; fixed-scale disagreement retained as diagnostic"
        );

        self.transition_lf_state(LfSessionState::ReturnHip)?;
        self.next_phase("Move LF HIP M13 from MAX contact to URDF-derived staged q=0")?;
        self.profile =
            lf_full_sequence_profile().map_err(|message| -> DynError { message.into() })?;
        let hip_staged_q0 = outcome.joints[0].affine.estimated_zero_tick;
        // W17 — the staged affine q0 for the hip. The operation derives the
        // value from THIS session's own accepted evidence; the session evidence
        // was recorded from this same outcome, so the two must agree.
        let hip_staged = self
            .move_motor_to(GoalNode::ReturnStaged {
                joint: JointKind::Hip,
            })
            .await?
            .emitted;
        self.assert_staged_matches_outcome(JointKind::Hip, hip_staged, hip_staged_q0)?;
        self.upsert_held_target(StaticTarget {
            motor_id: hip_staged.motor_id,
            target_tick: hip_staged.target,
        })?;

        self.transition_lf_state(LfSessionState::ReturnLowerHeld)?;
        self.next_phase("Move LF LOWER M11 to URDF-derived staged q=0 and hold")?;
        self.remove_held_target(11);
        let lower_staged_q0 = outcome.joints[2].affine.estimated_zero_tick;
        // W18 — staged affine q0 for the lower joint.
        let lower_staged = self
            .move_motor_to(GoalNode::ReturnStaged {
                joint: JointKind::Lower,
            })
            .await?
            .emitted;
        self.assert_staged_matches_outcome(JointKind::Lower, lower_staged, lower_staged_q0)?;
        self.upsert_held_target(StaticTarget {
            motor_id: lower_staged.motor_id,
            target_tick: lower_staged.target,
        })?;

        self.transition_lf_state(LfSessionState::ReturnUpper)?;
        self.next_phase("Move LF UPPER M12 to URDF-derived staged q=0 while M11 holds")?;
        self.remove_held_target(12);
        let upper_staged_q0 = outcome.joints[1].affine.estimated_zero_tick;
        // W19 — staged affine q0 for the upper joint.
        let upper_staged = self
            .move_motor_to(GoalNode::ReturnStaged {
                joint: JointKind::Upper,
            })
            .await?
            .emitted;
        self.assert_staged_matches_outcome(JointKind::Upper, upper_staged, upper_staged_q0)?;
        self.upsert_held_target(StaticTarget {
            motor_id: upper_staged.motor_id,
            target_tick: upper_staged.target,
        })?;

        self.transition_lf_state(LfSessionState::RestoreParking)?;
        self.next_phase("Restore LH upper M42 once at end of LF calibration")?;
        self.remove_held_target(42);
        // W20 — parking restore to digital home.
        self.move_motor_to(GoalNode::RestoreParking).await?;

        self.next_phase("Final verified global torque OFF")?;
        Ok(outcome)
    }

    fn inspect_lf_native_session_entry(&mut self) -> Result<(), DynError> {
        let mut entry_positions = Vec::with_capacity(MATDOG_MOTOR_IDS.len());
        for motor_id in MATDOG_MOTOR_IDS {
            let observation = self.latest_observation(motor_id)?;
            self.ensure_observation_fresh(motor_id, observation)?;
            entry_positions.push((motor_id, observation.position));
        }
        self.lf_session = Some(
            LegSessionStateMachine::new(LfSessionMode::LfFullLegSession, entry_positions)
                .map_err(|message| -> DynError { message.into() })?,
        );
        self.verify_lf_session_others_except(0)
    }

    fn transition_lf_state(&mut self, next: LfSessionState) -> Result<(), DynError> {
        let session = self
            .lf_session
            .as_mut()
            .ok_or("LF state transition requested before session creation")?;
        session
            .transition(next)
            .map_err(|message| -> DynError { message.into() })?;
        info!("MATDOG LF STATE: {}", next.label());
        Ok(())
    }

    fn set_lf_active(
        &mut self,
        motor_id: u8,
        target_tick: u16,
        kind: LfActiveKind,
    ) -> Result<(), DynError> {
        let session = self
            .lf_session
            .as_mut()
            .ok_or("LF active role requested before session creation")?;
        session
            .set_active(motor_id, target_tick, kind)
            .map_err(|message| -> DynError { message.into() })
    }

    fn record_lf_contacts(
        &mut self,
        joint: JointKind,
        contacts: DualContactResult,
    ) -> Result<(), DynError> {
        let session = self
            .lf_session
            .as_mut()
            .ok_or("LF contacts produced without a state machine")?;
        session.record_contacts(joint, contacts);
        Ok(())
    }

    fn record_lf_diagnostics(
        &mut self,
        joint: JointKind,
        fixed_scale: ModelZeroEstimate,
        affine: AffineJointCalibration,
    ) -> Result<(), DynError> {
        let session = self
            .lf_session
            .as_mut()
            .ok_or("LF diagnostics produced without a state machine")?;
        session.record_diagnostics(joint, fixed_scale, affine);
        Ok(())
    }

    /// W5 — the whole-session parking move. The motor and the LF historical
    /// pose are derived by `prerequisite_or_parking_move`; this function only
    /// owns the settle loop.
    async fn move_lf_session_motor_to(&mut self) -> Result<MotorObservation, DynError> {
        let EmittedGoal { motor_id, target } = self.prerequisite_or_parking_move().await?;
        let tolerance = GoalNode::Parking.settle_tolerance();
        let start = self.latest_observation(motor_id)?;
        let mut last_stamp = start.monotonic_stamp_ns;
        let mut stable_target = StableTargetGate::default();
        let deadline =
            Instant::now() + motion_timeout_for_distance(circular_distance(start.position, target));
        while Instant::now() < deadline {
            self.check_stop()?;
            let observation = self
                .wait_for_motor_observation_after(motor_id, last_stamp, TELEMETRY_TIMEOUT)
                .await?;
            last_stamp = observation.monotonic_stamp_ns;
            self.ensure_observation_safe(motor_id, observation, true, Some(target))?;
            self.verify_lf_session_others_except(motor_id)?;
            if stable_target.observe_at(observation, target, tolerance, Instant::now()) {
                return Ok(observation);
            }
        }
        let last = self.latest_observation(motor_id)?;
        Err(format!(
            "M{motor_id} LF session transition timeout: target={target}, present={}",
            last.position
        )
        .into())
    }

    fn verify_lf_session_others_except(&self, ignored_motor: u8) -> Result<(), DynError> {
        let session = self
            .lf_session
            .as_ref()
            .ok_or("LF role verification requested before session creation")?;
        validate_lf_session_snapshot(
            &self.current_state(),
            &self.target_bus_serial,
            session,
            ignored_motor,
            systime::get_monotonic_stamp_ns(),
        )
        .map_err(|message| -> DynError { message.into() })
    }

    fn upsert_held_target(&mut self, target: StaticTarget) -> Result<(), DynError> {
        if let Some(session) = &mut self.lf_session {
            session
                .hold(target)
                .map_err(|message| -> DynError { message.into() })?;
        }
        self.held_targets
            .retain(|held| held.motor_id != target.motor_id);
        self.held_targets.push(target);
        Ok(())
    }

    fn remove_held_target(&mut self, motor_id: u8) {
        self.held_targets
            .retain(|target| target.motor_id != motor_id);
        if let Some(session) = &mut self.lf_session {
            session.release(motor_id);
        }
    }

    async fn measure_lf_joint_pair_efficient(
        &mut self,
        minimum_profile: ContactProfile,
        maximum_profile: ContactProfile,
    ) -> Result<DualContactResult, DynError> {
        if minimum_profile.motor_id != maximum_profile.motor_id
            || minimum_profile.side != ContactSide::Min
            || maximum_profile.side != ContactSide::Max
        {
            return Err("invalid efficient MIN/MAX profile pair".into());
        }
        self.remove_held_target(minimum_profile.motor_id);
        self.transition_lf_state(lf_contact_state(minimum_profile.joint, ContactSide::Min))?;
        self.profile = minimum_profile;
        self.set_lf_active(
            self.profile.motor_id,
            self.latest_observation(self.profile.motor_id)?
                .goal_position,
            LfActiveKind::ContactProbe,
        )?;
        let minimum = self
            .measure_lf_contact_side_efficient(ContactSideEntry::FirstOfPair)
            .await?;

        // W13 — pressure release at the contact this session just recorded.
        self.stop_pressure_at_recorded_contact().await?;
        self.transition_lf_state(lf_contact_state(maximum_profile.joint, ContactSide::Max))?;
        self.profile = maximum_profile;
        self.set_lf_active(
            self.profile.motor_id,
            self.latest_observation(self.profile.motor_id)?
                .goal_position,
            LfActiveKind::ContactProbe,
        )?;
        let maximum = self
            .measure_lf_contact_side_efficient(ContactSideEntry::AfterRecordedContact)
            .await?;
        Ok(DualContactResult { minimum, maximum })
    }

    async fn measure_lf_contact_side_efficient(
        &mut self,
        entry: ContactSideEntry,
    ) -> Result<ContactResult, DynError> {
        self.next_phase(&format!(
            "{} moving baseline from current pose",
            self.profile.label
        ))?;
        if entry == ContactSideEntry::AfterRecordedContact {
            // W13 — release pressure at the previous side's recorded contact.
            self.stop_pressure_at_recorded_contact().await?;
        }
        let baseline = self.acquire_moving_current_baseline_forward().await?;

        self.next_phase(&format!("{} coarse scouting pass", self.profile.label))?;
        let coarse_scout_tick = self
            .approach_with_scout(ProbePass::Coarse, baseline)
            .await?;

        self.next_phase(&format!("{} coarse backoff", self.profile.label))?;
        self.backoff_and_verify(baseline).await?;

        self.next_phase(&format!("{} fine metrology pass 1", self.profile.label))?;
        let first_tick = self.approach_with_scout(ProbePass::Fine, baseline).await?;

        self.next_phase(&format!("{} fine metrology backoff", self.profile.label))?;
        self.backoff_and_verify(baseline).await?;

        self.next_phase(&format!("{} fine metrology pass 2", self.profile.label))?;
        let second_tick = self.approach_with_scout(ProbePass::Fine, baseline).await?;
        self.recorded_contact_tick = Some(second_tick);

        self.next_phase(&format!(
            "{} fine-to-fine repeatability",
            self.profile.label
        ))?;
        let spread_ticks = repeatability_spread(first_tick, second_tick)?;
        info!(
            "MATDOG {} persistent contact side complete: scout={}, fine1={}, fine2={}, spread={}, baseline_median={}, baseline_mad={}",
            self.profile.label,
            coarse_scout_tick,
            first_tick,
            second_tick,
            spread_ticks,
            baseline.median_current,
            baseline.mad_current,
        );
        Ok(ContactResult {
            coarse_scout_tick,
            first_tick,
            second_tick,
            spread_ticks,
            baseline,
        })
    }

    async fn acquire_moving_current_baseline_forward(&mut self) -> Result<BaselineStats, DynError> {
        let motor_id = self.profile.motor_id;
        // W7 — the full session's RELATIVE moving baseline. The operation
        // acquires the ONE entry observation, checks it, derives the target from
        // it and emits. That same observation then seeds this loop's sample
        // bookkeeping, so a single snapshot serves both uses exactly as the
        // immutable source did.
        self.enter_goal_node(GoalNode::MovingBaseline)?;
        let step = self.moving_baseline_step().await?;
        let initial = step.relative_entry.ok_or(
            "the full session's relative moving baseline must report its entry observation",
        )?;
        let target = step.emitted.target;
        let mut samples = Vec::new();
        let mut last_stamp = initial.monotonic_stamp_ns;
        let mut previous_position = initial.position;
        let deadline = Instant::now() + MOTION_TIMEOUT;
        while Instant::now() < deadline {
            self.check_stop()?;
            let observation = self
                .wait_for_motor_observation_after(motor_id, last_stamp, TELEMETRY_TIMEOUT)
                .await?;
            last_stamp = observation.monotonic_stamp_ns;
            self.ensure_observation_safe(motor_id, observation, true, Some(target))?;
            self.verify_profile_holds().await?;
            if circular_distance(observation.position, previous_position) > 0
                || speed_magnitude(observation.velocity) > 0
            {
                samples.push(observation.current);
            }
            previous_position = observation.position;
            if circular_distance(observation.position, target) <= STATIC_TOLERANCE_TICKS
                && samples.len() >= BASELINE_MIN_SAMPLES
            {
                break;
            }
        }
        if samples.len() < BASELINE_MIN_SAMPLES {
            return Err(format!(
                "{} insufficient moving baseline samples: {} < {}",
                self.profile.label,
                samples.len(),
                BASELINE_MIN_SAMPLES,
            )
            .into());
        }
        BaselineStats::from_samples(&samples).map_err(|message| -> DynError { message.into() })
    }

    async fn inspect_profile_entry(&mut self) -> Result<StartupEntryPlan, DynError> {
        let mut outliers = Vec::new();
        let mut home_recovery_motors = Vec::new();
        let mut home_ready_motors = BTreeSet::new();

        // All-or-nothing inventory. No torque is enabled until every motor has
        // fresh, healthy telemetry in the envelope allowed by the armed profile.
        for motor_id in MATDOG_MOTOR_IDS {
            let observation = self.latest_observation(motor_id)?;
            self.ensure_observation_fresh(motor_id, observation)?;
            if observation.torque_enabled {
                outliers.push(format!(
                    "M{motor_id}:torque-enabled,role={}",
                    startup_role_label(startup_role_for_profile(&self.profile, motor_id))
                ));
                continue;
            }
            if observation.has_driver_error || observation.status != 0 {
                outliers.push(format!(
                    "M{motor_id}:unhealthy,status=0x{:02X},driver_error={}",
                    observation.status, observation.has_driver_error
                ));
                continue;
            }
            if observation.current >= HARD_CURRENT_ABORT_RAW {
                outliers.push(format!(
                    "M{motor_id}:current={},limit={}",
                    observation.current, HARD_CURRENT_ABORT_RAW
                ));
                continue;
            }
            if !startup_position_allowed(&self.profile, motor_id, observation.position) {
                let (low, high) = startup_envelope(&self.profile, motor_id);
                outliers.push(format!(
                    "M{motor_id}:role={},present={},allowed={}..={}",
                    startup_role_label(startup_role_for_profile(&self.profile, motor_id)),
                    observation.position,
                    low,
                    high
                ));
                continue;
            }

            if matches!(
                startup_role_for_profile(&self.profile, motor_id),
                StartupRole::HomeOnly
            ) {
                let distance = circular_distance(observation.position, HOME_TICK);
                if distance <= STATIC_TOLERANCE_TICKS {
                    home_ready_motors.insert(motor_id);
                } else {
                    home_recovery_motors.push(motor_id);
                }
            }
        }

        if !outliers.is_empty() {
            return Err(format!(
                "restart-safe profile entry refused before motion; outliers=[{}]",
                outliers.join(", ")
            )
            .into());
        }

        for target in &self.profile.prerequisites {
            let observation = self.latest_observation(target.motor_id)?;
            info!(
                "MATDOG {} restart prerequisite inventory: M{} present={} target={}",
                self.profile.label, target.motor_id, observation.position, target.target_tick
            );
        }
        let probe = self.latest_observation(self.profile.motor_id)?;
        info!(
            "MATDOG {} restart probe inventory: M{} present={} home={} guard={}",
            self.profile.label,
            self.profile.motor_id,
            probe.position,
            HOME_TICK,
            self.profile.guard_tick
        );

        Ok(StartupEntryPlan {
            home_recovery_motors,
            home_ready_motors,
        })
    }

    async fn recover_home_only_joints(
        &mut self,
        plan: &mut StartupEntryPlan,
    ) -> Result<(), DynError> {
        let established_prerequisites = BTreeSet::new();
        for motor_id in plan.home_recovery_motors.clone() {
            self.verify_profile_entry_holds_except(
                0,
                &plan.home_ready_motors,
                &established_prerequisites,
            )
            .await?;

            let before = self.latest_observation(motor_id)?;
            info!(
                "MATDOG {} home-only recovery: M{} present={} target={} distance={}",
                self.profile.label,
                motor_id,
                before.position,
                HOME_TICK,
                circular_distance(before.position, HOME_TICK)
            );
            // W2 — torque-OFF HOME prime, then the torque enable, then W3 — the
            // torque-ON HOME reassertion. Two physically distinct writes (F1.3).
            self.advance_entry_step(EntryStep::LegacyProfileEntry {
                motor: motor_id,
                stage: LegacyEntryStage::Prime,
            })?;
            self.prepare_startup_home_recovery_motor(motor_id).await?;
            self.advance_entry_step(EntryStep::LegacyProfileEntry {
                motor: motor_id,
                stage: LegacyEntryStage::Reassert,
            })?;
            self.move_profile_entry_motor_to_target(
                &plan.home_ready_motors,
                &established_prerequisites,
            )
            .await?;
            self.set_startup_home_torque_verified(motor_id, false)
                .await?;
            plan.home_ready_motors.insert(motor_id);
        }
        self.verify_profile_entry_holds_except(
            0,
            &plan.home_ready_motors,
            &established_prerequisites,
        )
        .await
    }

    async fn establish_prerequisites_restart_safe(
        &mut self,
        plan: &StartupEntryPlan,
    ) -> Result<(), DynError> {
        let mut established_prerequisites = BTreeSet::new();
        for target in self.profile.prerequisites.clone() {
            self.verify_profile_entry_holds_except(
                0,
                &plan.home_ready_motors,
                &established_prerequisites,
            )
            .await?;

            let before = self.latest_observation(target.motor_id)?;
            info!(
                "MATDOG {} establish prerequisite: M{} present={} target={}",
                self.profile.label, target.motor_id, before.position, target.target_tick
            );
            // The entry context is advanced BEFORE the write-bearing operations
            // of this prerequisite: W4 (prepare_motor) and then W6. Without a
            // reviewed LF authority this fails here, so neither can emit.
            self.advance_entry_step(EntryStep::LegacyProfileEntry {
                motor: target.motor_id,
                stage: LegacyEntryStage::Prerequisite,
            })?;
            self.prepare_motor(target.motor_id).await?;
            // W6 — restart-safe prerequisite establishment through the armed
            // policy writer.
            self.move_profile_entry_motor_to_target(
                &plan.home_ready_motors,
                &established_prerequisites,
            )
            .await?;
            if self
                .held_targets
                .iter()
                .any(|held| held.motor_id == target.motor_id)
            {
                return Err(format!(
                    "duplicate restart-safe prerequisite target for M{}",
                    target.motor_id
                )
                .into());
            }
            self.held_targets.push(target);
            established_prerequisites.insert(target.motor_id);
            self.verify_profile_entry_holds_except(
                0,
                &plan.home_ready_motors,
                &established_prerequisites,
            )
            .await?;
        }
        Ok(())
    }

    /// Profile-entry motion. Which write this is — the W3 torque-ON HOME
    /// reassertion or the W6 prerequisite establishment — is derived from the
    /// current entry step, never from a caller flag, and the motor and target
    /// come from the owning engine operation.
    async fn move_profile_entry_motor_to_target(
        &mut self,
        home_ready_motors: &BTreeSet<u8>,
        established_prerequisites: &BTreeSet<u8>,
    ) -> Result<MotorObservation, DynError> {
        let EmittedGoal { motor_id, target } = match self.current_entry_step()? {
            EntryStep::LegacyProfileEntry {
                motor,
                stage: LegacyEntryStage::Reassert,
            } => {
                self.home_reassert_torque_on().await?;
                EmittedGoal {
                    motor_id: motor,
                    target: HOME_TICK,
                }
            }
            EntryStep::LegacyProfileEntry {
                stage: LegacyEntryStage::Prerequisite,
                ..
            } => self.prerequisite_or_parking_move().await?,
            step => {
                return Err(format!(
                    "profile-entry motion is not reachable from entry step {step:?}"
                )
                .into())
            }
        };
        let mut last_stamp = self.latest_observation(motor_id)?.monotonic_stamp_ns;
        let start_position = self.latest_observation(motor_id)?.position;
        let distance_ticks = circular_distance(start_position, target);
        let motion_timeout = motion_timeout_for_distance(distance_ticks);
        let deadline = Instant::now() + motion_timeout;
        info!(
            "MATDOG {} move plan: M{} start={} target={} distance={} timeout_ms={}",
            self.profile.label,
            motor_id,
            start_position,
            target,
            distance_ticks,
            motion_timeout.as_millis()
        );

        while Instant::now() < deadline {
            self.check_stop()?;
            let observation = self
                .wait_for_motor_observation_after(motor_id, last_stamp, TELEMETRY_TIMEOUT)
                .await?;
            last_stamp = observation.monotonic_stamp_ns;
            self.ensure_observation_safe(motor_id, observation, true, Some(target))?;
            self.verify_profile_entry_holds_except(
                motor_id,
                home_ready_motors,
                established_prerequisites,
            )
            .await?;
            if circular_distance(observation.position, target) <= STATIC_TOLERANCE_TICKS {
                return Ok(observation);
            }
        }

        let last = self.latest_observation(motor_id)?;
        Err(format!(
            "M{motor_id} profile-entry timeout: target={target}, present={}, error={}",
            last.position,
            circular_distance(last.position, target)
        )
        .into())
    }

    async fn verify_profile_entry_holds_except(
        &self,
        ignored_motor: u8,
        home_ready_motors: &BTreeSet<u8>,
        established_prerequisites: &BTreeSet<u8>,
    ) -> Result<(), DynError> {
        for motor_id in MATDOG_MOTOR_IDS {
            if motor_id == ignored_motor {
                continue;
            }
            let observation = self.latest_observation(motor_id)?;
            self.ensure_observation_fresh(motor_id, observation)?;
            validate_profile_entry_hold(
                &self.profile,
                motor_id,
                ignored_motor,
                home_ready_motors,
                established_prerequisites,
                observation,
            )
            .map_err(|message| -> DynError { message.into() })?;
        }
        Ok(())
    }

    async fn restore_prerequisites(&mut self) -> Result<(), DynError> {
        while let Some(target) = self.held_targets.pop() {
            // W23 — engine-owned LIFO restore to digital home.
            self.move_motor_to(GoalNode::RestorePrerequisite {
                motor_id: target.motor_id,
            })
            .await?;
            self.set_motor_torque_verified(target.motor_id, false)
                .await?;
        }
        Ok(())
    }

    fn verify_uniform_startup_home_snapshot(
        &self,
        ignored_motor: u8,
        home_ready_motors: &BTreeSet<u8>,
    ) -> Result<(), DynError> {
        for motor_id in MATDOG_MOTOR_IDS {
            if motor_id == ignored_motor {
                continue;
            }
            let observation = self.latest_observation(motor_id)?;
            self.ensure_observation_safe(motor_id, observation, false, None)?;
            if observation.torque_enabled {
                return Err(
                    format!("M{motor_id} startup-home normalization expected torque OFF").into(),
                );
            }
            let distance = circular_distance(observation.position, HOME_TICK);
            if home_ready_motors.contains(&motor_id) {
                if distance > STATIC_TOLERANCE_TICKS {
                    return Err(format!(
                        "M{motor_id} left q=0 after startup recovery: present={}, distance={}, tolerance={}",
                        observation.position, distance, STATIC_TOLERANCE_TICKS
                    )
                    .into());
                }
            } else if !startup_home_initial_position_valid(observation.position) {
                return Err(format!(
                    "M{motor_id} reported an invalid unsigned encoder position before q=0 normalization: present={}, valid=0..={}",
                    observation.position,
                    protocol::MAX_ANGLE_STEP
                )
                .into());
            }
        }
        Ok(())
    }

    async fn move_startup_home_motor_to_q0(
        &mut self,
        motor_id: u8,
        home_ready_motors: &BTreeSet<u8>,
    ) -> Result<MotorObservation, DynError> {
        // prepare_startup_home_recovery_motor() has already written HOME while
        // torque was OFF and then enabled this one servo with the low-energy
        // RAM envelope. From here only fresh telemetry drives the decision.
        let start = self.latest_observation(motor_id)?;
        let mut last_stamp = start.monotonic_stamp_ns;
        let mut stable_target = StableTargetGate::default();
        let distance = circular_distance(start.position, HOME_TICK);
        let deadline = Instant::now() + motion_timeout_for_distance(distance);

        info!(
            "MATDOG {} uniform startup-home recovery: M{} start={} target={} distance={}",
            self.profile.label, motor_id, start.position, HOME_TICK, distance
        );

        while Instant::now() < deadline {
            self.check_stop()?;
            let observation = self
                .wait_for_motor_observation_after(motor_id, last_stamp, TELEMETRY_TIMEOUT)
                .await?;
            last_stamp = observation.monotonic_stamp_ns;
            self.ensure_observation_safe(motor_id, observation, true, Some(HOME_TICK))?;
            self.verify_uniform_startup_home_snapshot(motor_id, home_ready_motors)?;
            if stable_target.observe_at(
                observation,
                HOME_TICK,
                STATIC_TOLERANCE_TICKS,
                Instant::now(),
            ) {
                return Ok(observation);
            }
        }

        let last = self.latest_observation(motor_id)?;
        Err(format!(
            "M{motor_id} startup-home recovery timeout: target={}, present={}, error={}",
            HOME_TICK,
            last.position,
            circular_distance(last.position, HOME_TICK)
        )
        .into())
    }

    async fn normalize_all_matdog_joints_to_q0(&mut self) -> Result<(), DynError> {
        let mut home_ready_motors = BTreeSet::new();

        // One uniform gate for every canonical joint. No per-joint operational
        // corridor is evaluated before q=0 normalization.
        self.verify_uniform_startup_home_snapshot(0, &home_ready_motors)?;

        for motor_id in MATDOG_MOTOR_IDS {
            let before = self.latest_observation(motor_id)?;
            let distance = circular_distance(before.position, HOME_TICK);
            let recovery_needed =
                distance > STATIC_TOLERANCE_TICKS || lf_initial_recovery_needed(before);
            if !recovery_needed {
                home_ready_motors.insert(motor_id);
                info!(
                    "MATDOG {} startup-home already ready: M{} present={} distance={}",
                    self.profile.label, motor_id, before.position, distance
                );
                continue;
            }

            self.verify_uniform_startup_home_snapshot(motor_id, &home_ready_motors)?;
            // W1 — the full session's all-12 q=0 prime. The settle stage that
            // follows dwells on this goal and issues no write of its own.
            self.advance_entry_step(EntryStep::FullNormalization {
                motor: motor_id,
                stage: NormalizationStage::Prime,
            })?;
            self.prepare_startup_home_recovery_motor(motor_id).await?;
            self.advance_entry_step(EntryStep::FullNormalization {
                motor: motor_id,
                stage: NormalizationStage::Settle,
            })?;
            self.move_startup_home_motor_to_q0(motor_id, &home_ready_motors)
                .await?;
            self.set_startup_home_torque_verified(motor_id, false)
                .await?;
            home_ready_motors.insert(motor_id);
            self.verify_uniform_startup_home_snapshot(0, &home_ready_motors)?;
        }

        if home_ready_motors.len() != MATDOG_MOTOR_IDS.len() {
            return Err(format!(
                "startup-home normalization incomplete: ready={:?}",
                home_ready_motors
            )
            .into());
        }
        self.verify_uniform_startup_home_snapshot(0, &home_ready_motors)
    }

    async fn prepare_startup_home_recovery_motor(&mut self, motor_id: u8) -> Result<(), DynError> {
        if !MATDOG_MOTOR_IDS.contains(&motor_id) {
            return Err(format!("M{motor_id} is outside the exact MATDOG motor set").into());
        }
        let initial = self.latest_observation(motor_id)?;
        self.ensure_observation_safe(motor_id, initial, false, None)?;
        if initial.torque_enabled {
            return Err(format!(
                "M{motor_id} startup-home recovery requires torque OFF before priming"
            )
            .into());
        }
        if !startup_home_initial_position_valid(initial.position) {
            return Err(format!(
                "M{motor_id} reported an invalid unsigned encoder position before q=0 normalization: present={}, valid=0..={}",
                initial.position,
                protocol::MAX_ANGLE_STEP
            )
            .into());
        }

        info!(
            "MATDOG {} q=0 normalization prepare: M{} present={} home={} distance={}",
            self.profile.label,
            motor_id,
            initial.position,
            HOME_TICK,
            circular_distance(initial.position, HOME_TICK)
        );

        // Match the useful SO101/ElRobot startup principle: do not reject a
        // healthy motor because of its initial angle. MATDOG remains RAM-only,
        // so set the exact low-energy envelope, write HOME while torque is OFF,
        // then enable only this servo and verify its telemetry-driven return.
        self.write_startup_home_ram_verified(
            motor_id,
            RamRegister::TorqueLimit,
            TORQUE_LIMIT.to_le_bytes().to_vec(),
        )
        .await?;
        self.write_startup_home_ram_verified(motor_id, RamRegister::Acc, vec![ACCELERATION])
            .await?;
        self.write_startup_home_ram_verified(
            motor_id,
            RamRegister::GoalSpeed,
            GOAL_SPEED.to_le_bytes().to_vec(),
        )
        .await?;
        // W1 / W2 — the torque-OFF q=0 prime, owned by the engine operation.
        self.home_normalization_prime().await?;
        self.set_startup_home_torque_verified(motor_id, true).await
    }

    /// POLICY WRITER A — the startup-home goal policy writer (G2 section 8.3).
    ///
    /// Successor of the historical `set_startup_home_goal_verified`. Reachable
    /// only from the reviewed startup operations covering W1, W2 and W3, and it
    /// keeps the historical startup gating: any canonical MATDOG motor plus the
    /// startup-local RAM-write policy, NOT the armed-profile allowlist (CG-7).
    async fn startup_home_goal_policy_write(
        &mut self,
        motor_id: u8,
        target: u16,
    ) -> Result<(), DynError> {
        if target > protocol::MAX_ANGLE_STEP {
            return Err(format!("unsigned startup GoalPosition out of range: {target}").into());
        }
        self.construct_goal_position_write(motor_id, target, GoalWriteRoute::StartupHome)
            .await
    }

    async fn set_startup_home_torque_verified(
        &mut self,
        motor_id: u8,
        enabled: bool,
    ) -> Result<(), DynError> {
        self.write_startup_home_ram_verified(
            motor_id,
            RamRegister::TorqueEnable,
            vec![u8::from(enabled)],
        )
        .await?;
        let observation = self.latest_observation(motor_id)?;
        if observation.torque_enabled != enabled {
            return Err(format!(
                "M{motor_id} startup torque readback mismatch: expected={enabled}, observed={}",
                observation.torque_enabled
            )
            .into());
        }
        Ok(())
    }

    async fn write_startup_home_ram_verified(
        &mut self,
        motor_id: u8,
        register: RamRegister,
        value: Vec<u8>,
    ) -> Result<(), DynError> {
        validate_ram_write(register, &value)?;
        if !MATDOG_MOTOR_IDS.contains(&motor_id) {
            return Err(format!("M{motor_id} is outside the exact MATDOG motor set").into());
        }
        if !ram_write_allowed_for_profile(
            &self.profile,
            motor_id,
            register.address() as u32,
            &value,
        ) {
            return Err(format!(
                "M{motor_id} startup RAM write rejected locally: register={}, value={:?}",
                register.name(),
                value
            )
            .into());
        }

        let initial_stamp = self.latest_observation(motor_id)?.monotonic_stamp_ns;
        let command_id = self.next_command_id();
        let envelope = TxEnvelope {
            monotonic_stamp_ns: systime::get_monotonic_stamp_ns(),
            local_stamp_ns: systime::get_local_stamp_ns(),
            app_start_id: systime::get_app_start_id(),
            target_bus_serial: self.target_bus_serial.clone(),
            command_id: command_id.clone(),
            write: Some(crate::st3215_proto::St3215WriteCommand {
                motor_id: motor_id as u32,
                address: register.address() as u32,
                value: value.clone().into(),
            }),
            ..Default::default()
        };
        self.comm.send_tx(&envelope)?;
        self.wait_for_command_result(&command_id).await?;
        self.wait_for_register_value(motor_id, register, &value, initial_stamp)
            .await
    }

    async fn prepare_motor(&mut self, motor_id: u8) -> Result<(), DynError> {
        if !self.profile.allowed_motor_ids.contains(&motor_id) {
            return Err(format!("M{motor_id} is outside armed profile motor allowlist").into());
        }
        // W4 — prime at the motor's own present observation, before torque.
        let initial = self.latest_observation(motor_id)?;
        self.prime_at_present(&FreshObservation {
            motor_id,
            observation: initial,
        })
        .await?;
        self.write_motor_ram_verified(
            motor_id,
            RamRegister::TorqueLimit,
            TORQUE_LIMIT.to_le_bytes().to_vec(),
        )
        .await?;
        self.write_motor_ram_verified(motor_id, RamRegister::Acc, vec![ACCELERATION])
            .await?;
        self.write_motor_ram_verified(
            motor_id,
            RamRegister::GoalSpeed,
            GOAL_SPEED.to_le_bytes().to_vec(),
        )
        .await?;
        self.set_motor_torque_verified(motor_id, true).await
    }

    async fn acquire_moving_current_baseline(&mut self) -> Result<BaselineStats, DynError> {
        let motor_id = self.profile.motor_id;
        let initial = self.latest_observation(motor_id)?;
        let mut samples = Vec::new();
        let mut last_stamp = initial.monotonic_stamp_ns;
        let mut previous_position = initial.position;
        // W8 — the two legacy modes' ABSOLUTE moving baseline.
        self.enter_goal_node(GoalNode::MovingBaseline)?;
        self.moving_baseline_step().await?;
        let deadline = Instant::now() + MOTION_TIMEOUT;

        while Instant::now() < deadline {
            self.check_stop()?;
            let observation = self
                .wait_for_motor_observation_after(motor_id, last_stamp, TELEMETRY_TIMEOUT)
                .await?;
            last_stamp = observation.monotonic_stamp_ns;
            self.ensure_observation_safe(
                motor_id,
                observation,
                true,
                Some(self.profile.baseline_target_tick),
            )?;
            self.verify_profile_holds().await?;
            if circular_distance(observation.position, previous_position) > 0
                || speed_magnitude(observation.velocity) > 0
            {
                samples.push(observation.current);
            }
            previous_position = observation.position;
            if circular_distance(observation.position, self.profile.baseline_target_tick)
                <= STATIC_TOLERANCE_TICKS
                && samples.len() >= BASELINE_MIN_SAMPLES
            {
                break;
            }
        }
        if samples.len() < BASELINE_MIN_SAMPLES {
            return Err(format!(
                "insufficient moving baseline samples: {} < {}",
                samples.len(),
                BASELINE_MIN_SAMPLES
            )
            .into());
        }
        let baseline = BaselineStats::from_samples(&samples)
            .map_err(|message| -> DynError { message.into() })?;
        // W21 — probe return home after the absolute baseline.
        self.move_motor_to(GoalNode::ProbeReturnHome).await?;
        Ok(baseline)
    }

    /// Contact search over one pass. The step size and the scout policy are
    /// DERIVED from `pass` plus the validated session mode by
    /// `probe_advance_step`; they are never caller choices (W9/W10/W11).
    async fn approach_with_scout(
        &mut self,
        pass: ProbePass,
        baseline: BaselineStats,
    ) -> Result<u16, DynError> {
        let step_ticks = self.derived_probe_step(pass);
        let coarse_scout_tick = self.derived_scout_policy(pass)?;
        let motor_id = self.profile.motor_id;
        let start = self.latest_observation(motor_id)?;
        self.ensure_observation_safe(motor_id, start, true, None)?;
        let mut detector = HybridContactDetector::new_for_profile_with_scout(
            start.position,
            baseline,
            &self.profile,
            coarse_scout_tick,
        );
        self.probe_target = Some(start.position);
        self.enter_goal_node(GoalNode::ProbeAdvance { pass })?;
        let mut target;
        let mut last_stamp = start.monotonic_stamp_ns;

        'approach_steps: loop {
            self.check_stop()?;
            let emitted = self.probe_advance_step().await?;
            target = emitted.target;
            let settle_deadline = Instant::now() + CONTACT_SETTLE_WINDOW;
            let mut last_observation = None;

            while Instant::now() < settle_deadline {
                let observation = self
                    .wait_for_motor_observation_after(motor_id, last_stamp, TELEMETRY_TIMEOUT)
                    .await?;
                last_stamp = observation.monotonic_stamp_ns;
                last_observation = Some(observation);
                self.ensure_observation_safe(motor_id, observation, true, Some(target))?;
                self.verify_profile_holds().await?;
                if circular_distance(observation.position, target) <= STATIC_TOLERANCE_TICKS {
                    break;
                }
                match detector.observe(observation, target) {
                    ContactState::FreeMotion | ContactState::ContactSuspected => {}
                    ContactState::ContactConfirmed => {
                        if let Some(scout) = coarse_scout_tick {
                            let scout_lag = fine_contact_scout_lag_ticks(
                                observation.position,
                                scout,
                                self.profile.probe_sign,
                            );
                            if !fine_contact_reproduces_coarse_depth(
                                observation.position,
                                scout,
                                self.profile.probe_sign,
                            ) {
                                info!(
                                    "MATDOG {} friction plateau bypass: target={}, present={}, coarse_scout={}, scout_lag={}, allowed_lag={}, current={}, threshold={}, velocity={}",
                                    self.profile.label,
                                    target,
                                    observation.position,
                                    scout,
                                    scout_lag,
                                    FINE_CONTACT_SCOUT_LAG_TOLERANCE_TICKS,
                                    observation.current,
                                    baseline.contact_threshold(),
                                    speed_magnitude(observation.velocity),
                                );
                                continue 'approach_steps;
                            }
                        }
                        info!(
                            "MATDOG {} contact: step={}, target={}, present={}, error={}, current={}, threshold={}, velocity={}, scout={:?}",
                            self.profile.label,
                            step_ticks,
                            target,
                            observation.position,
                            circular_distance(observation.position, target),
                            observation.current,
                            baseline.contact_threshold(),
                            speed_magnitude(observation.velocity),
                            coarse_scout_tick,
                        );
                        // FINAL ACCEPTED CONTACT -> stop advancing immediately.
                        self.stop_pressure_at_observation(&ContactVerdict {
                            motor_id,
                            observation,
                        })
                        .await?;
                        self.record_node_contact(pass, observation.position);
                        return Ok(observation.position);
                    }
                    ContactState::EarlyStall => {
                        self.stop_pressure_at_observation(&ContactVerdict {
                            motor_id,
                            observation,
                        })
                        .await?;
                        let (acceptance_low, acceptance_high) =
                            adaptive_contact_acceptance_bounds(&self.profile, coarse_scout_tick);
                        return Err(format!(
                            "{} early stall outside adaptive contact corridor: target={}, present={}, acceptance={}..={}, URDF={}, guard={}, current={}, threshold={}, velocity={}",
                            self.profile.label,
                            target,
                            observation.position,
                            acceptance_low,
                            acceptance_high,
                            self.profile.urdf_limit_tick,
                            self.profile.guard_tick,
                            observation.current,
                            baseline.contact_threshold(),
                            speed_magnitude(observation.velocity)
                        )
                        .into());
                    }
                    ContactState::HardAbort => {
                        return self.abort_with_global_torque_off(format!(
                            "{} hard abort: tick={}, goal={}, current={}, torque_enabled={}, torque_limit={}, status=0x{:02X}, driver_error={}",
                            self.profile.label,
                            observation.position,
                            observation.goal_position,
                            observation.current,
                            observation.torque_enabled,
                            observation.torque_limit,
                            observation.status,
                            observation.has_driver_error
                        )).await;
                    }
                }
            }

            let observation =
                last_observation.ok_or("contact settle window produced no telemetry")?;
            self.ensure_observation_safe(motor_id, observation, true, Some(target))?;
            let goal_error = circular_distance(observation.position, target);
            let nominal_tracking_limit = step_ticks.saturating_add(4);
            let tracking_error_limit = probe_tracking_error_limit(step_ticks);
            if goal_error > nominal_tracking_limit && goal_error <= tracking_error_limit {
                info!(
                    "MATDOG {} bounded tracking-lag continuation: step={}, target={}, present={}, error={}, nominal_limit={}, bounded_limit={}, current={}, velocity={}",
                    self.profile.label,
                    step_ticks,
                    target,
                    observation.position,
                    goal_error,
                    nominal_tracking_limit,
                    tracking_error_limit,
                    observation.current,
                    speed_magnitude(observation.velocity),
                );
            }
            if goal_error > tracking_error_limit {
                if let Some(scout) = coarse_scout_tick {
                    if let Some(contact) = self
                        .confirm_kinematic_plateau(target, last_stamp, scout)
                        .await?
                    {
                        let scout_lag = fine_contact_scout_lag_ticks(
                            contact.position,
                            scout,
                            self.profile.probe_sign,
                        );
                        if !fine_contact_reproduces_coarse_depth(
                            contact.position,
                            scout,
                            self.profile.probe_sign,
                        ) {
                            info!(
                                "MATDOG {} adaptive friction plateau bypass: target={}, present={}, coarse_scout={}, scout_lag={}, allowed_lag={}, current={}",
                                self.profile.label,
                                target,
                                contact.position,
                                scout,
                                scout_lag,
                                FINE_CONTACT_SCOUT_LAG_TOLERANCE_TICKS,
                                contact.current,
                            );
                            continue 'approach_steps;
                        }
                        info!(
                            "MATDOG {} adaptive kinematic contact: step={}, target={}, present={}, error={}, current={}, scout={}",
                            self.profile.label,
                            step_ticks,
                            target,
                            contact.position,
                            circular_distance(contact.position, target),
                            contact.current,
                            scout,
                        );
                        self.stop_pressure_at_observation(&ContactVerdict {
                            motor_id,
                            observation: contact,
                        })
                        .await?;
                        self.record_node_contact(pass, contact.position);
                        return Ok(contact.position);
                    }
                }
                self.stop_pressure_at_observation(&ContactVerdict {
                    motor_id,
                    observation,
                })
                .await?;
                return Err(format!(
                    "{} tracking failed without confirmed contact: target={}, present={}, current={}",
                    self.profile.label, target, observation.position, observation.current
                )
                .into());
            }
        }
    }

    async fn confirm_kinematic_plateau(
        &mut self,
        target: u16,
        mut last_stamp: u64,
        coarse_scout_tick: u16,
    ) -> Result<Option<MotorObservation>, DynError> {
        let motor_id = self.profile.motor_id;
        let (low, high) =
            adaptive_contact_acceptance_bounds(&self.profile, Some(coarse_scout_tick));
        let mut observations = Vec::with_capacity(KINEMATIC_PLATEAU_SAMPLES);
        for _ in 0..KINEMATIC_PLATEAU_SAMPLES {
            let observation = self
                .wait_for_motor_observation_after(motor_id, last_stamp, TELEMETRY_TIMEOUT)
                .await?;
            last_stamp = observation.monotonic_stamp_ns;
            self.ensure_observation_safe(motor_id, observation, true, Some(target))?;
            self.verify_profile_holds().await?;
            let target_ahead = i32::from(signed_tick_delta(target, observation.position))
                * i32::from(self.profile.probe_sign)
                > 0;
            if !(low..=high).contains(&observation.position)
                || !target_ahead
                || speed_magnitude(observation.velocity)
                    > HybridContactConfig::default().max_velocity_raw
                || circular_distance(observation.position, coarse_scout_tick)
                    > ADAPTIVE_FINE_SCOUT_TICKS
            {
                return Ok(None);
            }
            observations.push(observation);
        }
        let min_position = observations
            .iter()
            .map(|observation| observation.position)
            .min()
            .unwrap_or(0);
        let max_position = observations
            .iter()
            .map(|observation| observation.position)
            .max()
            .unwrap_or(0);
        if circular_distance(min_position, max_position) > KINEMATIC_PLATEAU_POSITION_SPAN_TICKS {
            return Ok(None);
        }
        Ok(observations.last().copied())
    }

    /// W14 — bounded backoff. The contact tick comes from the contact this node
    /// just recorded, never from a caller argument.
    async fn backoff_and_verify(&mut self, baseline: BaselineStats) -> Result<(), DynError> {
        let recovered = self.move_motor_to(GoalNode::Backoff).await?.observation;
        if recovered.current > baseline.contact_threshold() {
            return Err(format!(
                "{} current did not recover after backoff: {} > {}",
                self.profile.label,
                recovered.current,
                baseline.contact_threshold()
            )
            .into());
        }
        Ok(())
    }

    async fn abort_with_global_torque_off<T>(&mut self, message: String) -> Result<T, DynError> {
        match self.global_torque_off_verified().await {
            Ok(()) => Err(message.into()),
            Err(cleanup_err) => Err(format!(
                "{message}; immediate verified global torque-OFF also failed: {cleanup_err}"
            )
            .into()),
        }
    }

    /// The staged q0 an operation derived from this session's own evidence must
    /// equal the affine estimate this session recorded for that joint. It
    /// cannot differ by construction; if it ever did, the session's evidence and
    /// the emitted goal would have diverged and the run must stop.
    fn assert_staged_matches_outcome(
        &self,
        joint: JointKind,
        staged: EmittedGoal,
        recorded_affine_zero_tick: u16,
    ) -> Result<(), DynError> {
        if staged.target != recorded_affine_zero_tick
            || staged.motor_id != spec_for(Leg::Lf, joint).motor_id
        {
            return Err(format!(
                "{joint:?} staged q0 diverged from this session's recorded affine evidence: emitted M{} target={}, recorded={recorded_affine_zero_tick}",
                staged.motor_id, staged.target
            )
            .into());
        }
        Ok(())
    }

    /// Record the contact this contact-search node just accepted, and the
    /// coarse scout depth when the pass that produced it was the coarse one.
    /// Both feed later derivations; neither is caller-supplied.
    fn record_node_contact(&mut self, pass: ProbePass, tick: u16) {
        self.node_contact_tick = Some(tick);
        if pass == ProbePass::Coarse {
            self.recorded_coarse_scout = Some(tick);
        }
    }

    /// A commanded move: what the owning engine operation derived and emitted,
    /// plus the observation the settle loop accepted.
    async fn move_motor_to(&mut self, node: GoalNode) -> Result<CommandedMove, DynError> {
        self.enter_goal_node(node)?;
        let tolerance = node.settle_tolerance();
        let emitted = match node {
            GoalNode::Backoff => self.backoff_step().await?,
            GoalNode::StaticHold { .. } => self.static_hold_transition().await?,
            GoalNode::ReturnStaged { .. } => self.staged_affine_q0().await?,
            GoalNode::RestoreParking
            | GoalNode::ProbeReturnHome
            | GoalNode::PostRestoreSettle
            | GoalNode::RestorePrerequisite { .. } => self.return_home().await?,
            node => {
                return Err(format!("{node:?} does not own a commanded move").into());
            }
        };
        let EmittedGoal { motor_id, target } = emitted;
        let persistent_lf_session = self.lf_session.is_some();
        let mut last_stamp = self.latest_observation(motor_id)?.monotonic_stamp_ns;
        let start_position = self.latest_observation(motor_id)?.position;
        let distance_ticks = circular_distance(start_position, target);
        let motion_timeout = motion_timeout_for_distance(distance_ticks);
        let deadline = Instant::now() + motion_timeout;
        let mut stable_target = StableTargetGate::default();
        info!(
            "MATDOG {} move plan: M{} start={} target={} distance={} timeout_ms={}",
            self.profile.label,
            motor_id,
            start_position,
            target,
            distance_ticks,
            motion_timeout.as_millis()
        );
        while Instant::now() < deadline {
            self.check_stop()?;
            let observation = self
                .wait_for_motor_observation_after(motor_id, last_stamp, TELEMETRY_TIMEOUT)
                .await?;
            last_stamp = observation.monotonic_stamp_ns;
            self.ensure_observation_safe(motor_id, observation, true, Some(target))?;
            self.verify_static_holds_except(motor_id).await?;
            let inside_target = circular_distance(observation.position, target) <= tolerance;
            let low_velocity = speed_magnitude(observation.velocity) <= LF_HELD_MAX_SPEED_RAW;
            if inside_target && (!persistent_lf_session || low_velocity) {
                if !persistent_lf_session {
                    return Ok(CommandedMove {
                        emitted,
                        observation,
                    });
                }
                if stable_target.observe_at(observation, target, tolerance, Instant::now()) {
                    return Ok(CommandedMove {
                        emitted,
                        observation,
                    });
                }
            } else {
                stable_target = StableTargetGate::default();
            }
        }
        let last = self.latest_observation(motor_id)?;
        Err(format!(
            "M{motor_id} target timeout: target={target}, present={}, error={}",
            last.position,
            circular_distance(last.position, target)
        )
        .into())
    }

    async fn verify_profile_holds(&self) -> Result<(), DynError> {
        self.verify_static_holds_except(self.profile.motor_id).await
    }

    async fn verify_static_holds_except(&self, ignored_motor: u8) -> Result<(), DynError> {
        if self.lf_session.is_some() {
            return self.verify_lf_session_others_except(ignored_motor);
        }
        for motor_id in MATDOG_MOTOR_IDS {
            if motor_id == ignored_motor {
                continue;
            }

            let observation = self.latest_observation(motor_id)?;
            self.ensure_observation_fresh(motor_id, observation)?;

            if let Some(target) = self
                .held_targets
                .iter()
                .find(|target| target.motor_id == motor_id)
            {
                self.ensure_observation_safe(
                    motor_id,
                    observation,
                    true,
                    Some(target.target_tick),
                )?;
                if circular_distance(observation.position, target.target_tick)
                    > STATIC_TOLERANCE_TICKS
                {
                    return Err(format!(
                        "static prerequisite M{motor_id} drifted: target={}, present={}, tolerance={}",
                        target.target_tick,
                        observation.position,
                        STATIC_TOLERANCE_TICKS
                    )
                    .into());
                }
            } else {
                if observation.torque_enabled {
                    return Err(
                        format!("non-active M{motor_id} unexpectedly torque-enabled").into(),
                    );
                }
                if observation.has_driver_error || observation.status != 0 {
                    return Err(format!("non-active M{motor_id} became unhealthy").into());
                }
                let tolerance =
                    home_hold_tolerance(&self.profile, motor_id, self.probe_home_handoff_active);
                if circular_distance(observation.position, HOME_TICK) > tolerance {
                    return Err(format!(
                        "non-active M{motor_id} left home: present={}, expected={}, tolerance={}",
                        observation.position, HOME_TICK, tolerance
                    )
                    .into());
                }
            }
        }
        Ok(())
    }

    /// POLICY WRITER B — the normal armed-goal policy writer (G2 section 8.3).
    ///
    /// Successor of the historical `set_motor_goal_verified`. Reachable only
    /// from the reviewed engine operations covering W4..W23, and it keeps the
    /// historical armed-profile allowlist gating (CG-7). The two policy writers
    /// are deliberately NOT collapsed: they admit different motor sets.
    async fn armed_goal_policy_write(&mut self, motor_id: u8, target: u16) -> Result<(), DynError> {
        if target > protocol::MAX_ANGLE_STEP {
            return Err(format!("unsigned GoalPosition out of range: {target}").into());
        }
        if let Some(session) = &mut self.lf_session {
            session
                .update_active_target(motor_id, target)
                .map_err(|message| -> DynError { message.into() })?;
        }
        self.construct_goal_position_write(motor_id, target, GoalWriteRoute::ArmedProfile)
            .await
    }

    /// THE unique raw GoalPosition constructor / emitter (G2 section 8.3, CG-1).
    ///
    /// This is the only production location in the MATDOG namespace that builds
    /// a `RamRegister::GoalPosition` write envelope. Its direct production
    /// callers are exactly the two policy writers above; the twelve engine
    /// operations of section 8.2 reach it indirectly, two levels up. Terminal
    /// verified global torque OFF, hard abort and operator stop stay outside
    /// this graph and continue to write `TorqueEnable` only (CG-8).
    async fn construct_goal_position_write(
        &mut self,
        motor_id: u8,
        target: u16,
        route: GoalWriteRoute,
    ) -> Result<(), DynError> {
        self.emit_goal_write_envelope(
            motor_id,
            RamRegister::GoalPosition,
            target.to_le_bytes().to_vec(),
            route,
        )
        .await
    }

    /// Transport selection BENEATH the construction boundary. The two historical
    /// RAM writers keep their different motor-admission gates and are never
    /// merged: the startup transport admits any canonical MATDOG motor, the
    /// armed transport admits only `profile.allowed_motor_ids` (CG-7).
    async fn emit_goal_write_envelope(
        &mut self,
        motor_id: u8,
        register: RamRegister,
        value: Vec<u8>,
        route: GoalWriteRoute,
    ) -> Result<(), DynError> {
        match route {
            GoalWriteRoute::StartupHome => {
                self.write_startup_home_ram_verified(motor_id, register, value)
                    .await
            }
            GoalWriteRoute::ArmedProfile => {
                self.write_motor_ram_verified(motor_id, register, value)
                    .await
            }
        }
    }

    async fn set_motor_torque_verified(
        &mut self,
        motor_id: u8,
        enabled: bool,
    ) -> Result<(), DynError> {
        self.write_motor_ram_verified(motor_id, RamRegister::TorqueEnable, vec![u8::from(enabled)])
            .await?;
        let observation = self.latest_observation(motor_id)?;
        if observation.torque_enabled != enabled {
            return Err(format!(
                "M{motor_id} torque readback mismatch: expected={enabled}, observed={}",
                observation.torque_enabled
            )
            .into());
        }
        if !enabled {
            if let Some(session) = &mut self.lf_session {
                session.clear_active(motor_id);
            }
        }
        Ok(())
    }

    async fn global_torque_off_verified(&mut self) -> Result<(), DynError> {
        if let Some(session) = &mut self.lf_session {
            session
                .transition(LfSessionState::Cleanup)
                .map_err(|message| -> DynError { message.into() })?;
        }
        let writes = global_torque_off_writes();
        self.sync_write_ram_verified(RamRegister::TorqueEnable, &writes)
            .await?;
        for motor_id in MATDOG_MOTOR_IDS {
            let observation =
                observation_from_state(&self.current_state(), &self.target_bus_serial, motor_id)?;
            if observation.torque_enabled {
                return Err(format!("M{motor_id} remained torque-enabled after global OFF").into());
            }
        }
        self.held_targets.clear();
        if let Some(session) = &mut self.lf_session {
            session
                .complete_verified_cleanup()
                .map_err(|message| -> DynError { message.into() })?;
        }
        Ok(())
    }

    async fn write_motor_ram_verified(
        &mut self,
        motor_id: u8,
        register: RamRegister,
        value: Vec<u8>,
    ) -> Result<(), DynError> {
        validate_ram_write(register, &value)?;
        if !self.profile.allowed_motor_ids.contains(&motor_id) {
            return Err(format!("M{motor_id} is outside armed profile motor allowlist").into());
        }
        let initial_stamp = self.latest_observation(motor_id)?.monotonic_stamp_ns;
        let command_id = self.next_command_id();
        let envelope = TxEnvelope {
            monotonic_stamp_ns: systime::get_monotonic_stamp_ns(),
            local_stamp_ns: systime::get_local_stamp_ns(),
            app_start_id: systime::get_app_start_id(),
            target_bus_serial: self.target_bus_serial.clone(),
            command_id: command_id.clone(),
            write: Some(crate::st3215_proto::St3215WriteCommand {
                motor_id: motor_id as u32,
                address: register.address() as u32,
                value: value.clone().into(),
            }),
            ..Default::default()
        };
        self.comm.send_tx(&envelope)?;
        self.wait_for_command_result(&command_id).await?;
        self.wait_for_register_value(motor_id, register, &value, initial_stamp)
            .await
    }

    async fn sync_write_ram_verified(
        &mut self,
        register: RamRegister,
        writes: &[(u8, Vec<u8>)],
    ) -> Result<(), DynError> {
        if writes.is_empty() {
            return Err("MATDOG sync-write cannot be empty".into());
        }
        let mut unique_motor_ids = BTreeSet::new();
        for (motor_id, value) in writes {
            validate_ram_write(register, value)?;
            if !MATDOG_MOTOR_IDS.contains(motor_id) || !unique_motor_ids.insert(*motor_id) {
                return Err(format!("invalid MATDOG sync-write motor ID: {motor_id}").into());
            }
        }
        let initial_stamps: Vec<(u8, u64)> = writes
            .iter()
            .map(|(motor_id, _)| {
                observation_from_state(&self.current_state(), &self.target_bus_serial, *motor_id)
                    .map(|obs| (*motor_id, obs.monotonic_stamp_ns))
            })
            .collect::<Result<_, _>>()?;
        let command_id = self.next_command_id();
        let envelope = TxEnvelope {
            monotonic_stamp_ns: systime::get_monotonic_stamp_ns(),
            local_stamp_ns: systime::get_local_stamp_ns(),
            app_start_id: systime::get_app_start_id(),
            target_bus_serial: self.target_bus_serial.clone(),
            command_id: command_id.clone(),
            sync_write: Some(crate::st3215_proto::St3215SyncWriteCommand {
                address: register.address() as u32,
                motors: writes
                    .iter()
                    .map(|(motor_id, value)| {
                        crate::st3215_proto::st3215_sync_write_command::MotorWrite {
                            motor_id: *motor_id as u32,
                            value: value.clone().into(),
                        }
                    })
                    .collect(),
            }),
            ..Default::default()
        };
        self.comm.send_tx(&envelope)?;
        self.wait_for_command_result(&command_id).await?;
        for ((motor_id, value), (_, initial_stamp)) in writes.iter().zip(initial_stamps) {
            self.wait_for_register_value(*motor_id, register, value, initial_stamp)
                .await?;
        }
        Ok(())
    }

    async fn wait_for_register_value(
        &mut self,
        motor_id: u8,
        register: RamRegister,
        expected: &[u8],
        initial_stamp: u64,
    ) -> Result<(), DynError> {
        let deadline = Instant::now() + COMMAND_TIMEOUT;
        let mut last_stamp = initial_stamp;
        while Instant::now() < deadline {
            let observation = self
                .wait_for_motor_observation_after(motor_id, last_stamp, TELEMETRY_TIMEOUT)
                .await?;
            last_stamp = observation.monotonic_stamp_ns;
            let state = self.current_state();
            let motor = find_motor(&state, &self.target_bus_serial, motor_id)?;
            if motor_ram_register_matches(motor, register, expected) {
                return Ok(());
            }
        }
        Err(format!(
            "M{motor_id} RAM readback timeout for {} at 0x{:02X}",
            register.name(),
            register.address()
        )
        .into())
    }

    async fn wait_for_command_result(&mut self, command_id: &Bytes) -> Result<(), DynError> {
        let deadline = Instant::now() + COMMAND_TIMEOUT;
        loop {
            let state = self.current_state();
            if let Some(result) = command_result_for(&state, &self.target_bus_serial, command_id) {
                match CommandResult::try_from(result) {
                    Ok(CommandResult::CrSuccess) => return Ok(()),
                    Ok(CommandResult::CrRejected) => return Err("ST3215 command rejected".into()),
                    Ok(CommandResult::CrFailed) => return Err("ST3215 command failed".into()),
                    Ok(CommandResult::CrProcessing) => {}
                    Err(_) => return Err(format!("invalid ST3215 command result: {result}").into()),
                }
            }
            tokio::select! {
                changed = self.inference_rx.changed() => {
                    if changed.is_err() { return Err("ST3215 inference channel closed".into()); }
                }
                _ = tokio::time::sleep_until(deadline) => {
                    return Err("ST3215 command result timeout".into());
                }
            }
        }
    }

    async fn wait_for_exact_motor_set(&mut self) -> Result<(), DynError> {
        let deadline = Instant::now() + COMMAND_TIMEOUT;
        loop {
            let state = self.current_state();
            if let Ok(found) = motor_ids_for_bus(&state, &self.target_bus_serial) {
                if is_exact_matdog_motor_set(&found) {
                    return Ok(());
                }
                if found.len() >= MATDOG_MOTOR_IDS.len() {
                    return Err(format!(
                        "MATDOG inference ID mismatch: expected {:?}, found {:?}",
                        MATDOG_MOTOR_IDS, found
                    )
                    .into());
                }
            }
            tokio::select! {
                changed = self.inference_rx.changed() => {
                    if changed.is_err() { return Err("ST3215 inference channel closed".into()); }
                }
                _ = tokio::time::sleep_until(deadline) => {
                    return Err("MATDOG exact ID set timeout".into());
                }
            }
        }
    }

    fn latest_observation(&self, motor_id: u8) -> Result<MotorObservation, DynError> {
        observation_from_state(&self.current_state(), &self.target_bus_serial, motor_id)
    }

    async fn wait_for_motor_observation_after(
        &mut self,
        motor_id: u8,
        minimum_stamp: u64,
        timeout: Duration,
    ) -> Result<MotorObservation, DynError> {
        let deadline = Instant::now() + timeout;
        loop {
            if let Ok(observation) =
                observation_from_state(&self.current_state(), &self.target_bus_serial, motor_id)
            {
                if observation.monotonic_stamp_ns > minimum_stamp {
                    return Ok(observation);
                }
            }
            tokio::select! {
                changed = self.inference_rx.changed() => {
                    if changed.is_err() { return Err("ST3215 inference channel closed".into()); }
                }
                _ = tokio::time::sleep_until(deadline) => {
                    return Err(format!("M{motor_id} fresh telemetry timeout").into());
                }
            }
        }
    }

    fn current_state(&self) -> InferenceState {
        self.inference_rx.borrow().clone()
    }

    fn ensure_observation_fresh(
        &self,
        motor_id: u8,
        observation: MotorObservation,
    ) -> Result<(), DynError> {
        let now = systime::get_monotonic_stamp_ns();
        let max_age_ns = u64::try_from(MAX_TELEMETRY_AGE.as_nanos()).unwrap_or(u64::MAX);
        let age_ns = now.saturating_sub(observation.monotonic_stamp_ns);
        if observation.monotonic_stamp_ns == 0 || age_ns > max_age_ns {
            return Err(format!(
                "M{motor_id} telemetry stale: age_ns={age_ns}, max_age_ns={max_age_ns}"
            )
            .into());
        }
        Ok(())
    }

    fn ensure_observation_safe(
        &self,
        motor_id: u8,
        observation: MotorObservation,
        require_torque: bool,
        expected_goal: Option<u16>,
    ) -> Result<(), DynError> {
        self.ensure_observation_fresh(motor_id, observation)?;
        if observation.has_driver_error {
            return Err(format!("M{motor_id} driver error present").into());
        }
        if observation.status != 0 {
            return Err(format!("M{motor_id} servo status is 0x{:02X}", observation.status).into());
        }
        if require_torque && !observation.torque_enabled {
            return Err(format!("M{motor_id} torque unexpectedly disabled").into());
        }
        if require_torque && observation.torque_limit != TORQUE_LIMIT {
            return Err(format!(
                "M{motor_id} torque-limit changed: expected={}, observed={}",
                TORQUE_LIMIT, observation.torque_limit
            )
            .into());
        }
        if let Some(expected_goal) = expected_goal {
            if observation.goal_position != expected_goal {
                return Err(format!(
                    "M{motor_id} goal changed: expected={expected_goal}, observed={}",
                    observation.goal_position
                )
                .into());
            }
        }
        if observation.current >= HARD_CURRENT_ABORT_RAW {
            return Err(format!(
                "M{motor_id} hard current abort: {} >= {}",
                observation.current, HARD_CURRENT_ABORT_RAW
            )
            .into());
        }
        self.ensure_temperature_safe(motor_id, observation)?;
        if require_torque {
            if let Some(session) = &self.lf_session {
                let role = session
                    .role_for(motor_id)
                    .map_err(|message| -> DynError { message.into() })?;
                match role {
                    LfMotorRole::ActivelyCommanded { target_tick }
                    | LfMotorRole::ContactProbe { target_tick }
                    | LfMotorRole::ActivelyHeld { target_tick } => {
                        validate_lf_active_readback(motor_id, observation, target_tick)
                            .map_err(|message| -> DynError { message.into() })?;
                    }
                    LfMotorRole::PassiveTorqueOffSafe { .. }
                    | LfMotorRole::NonParticipatingTorqueOff { .. } => {
                        return Err(format!(
                            "M{motor_id} requires torque without an active/held LF role"
                        )
                        .into());
                    }
                }
            }
        }
        Ok(())
    }

    fn ensure_temperature_safe(
        &self,
        motor_id: u8,
        observation: MotorObservation,
    ) -> Result<(), DynError> {
        validate_matdog_temperature(motor_id, observation)
            .map_err(|message| -> DynError { message.into() })
    }

    fn check_stop(&self) -> Result<(), DynError> {
        if self.stop_requested.load(Ordering::Relaxed) {
            Err("MATDOG calibration stopped by operator".into())
        } else {
            Ok(())
        }
    }

    fn next_phase(&mut self, phase: &str) -> Result<(), DynError> {
        self.check_stop()?;
        self.current_step += 1;
        self.publish_progress(
            self.current_step,
            phase,
            CalibrationStatus::InProgress,
            None,
        );
        Ok(())
    }

    fn publish_progress(
        &self,
        current: u32,
        phase: &str,
        status: CalibrationStatus,
        error: Option<&str>,
    ) {
        let rendered = format!("{}: {phase}", self.profile.label);
        // G3-C capture point: the ACTUAL rendered event, recorded at the
        // production publication boundary so a regression compares emissions,
        // never a reconstruction. Test builds only; it publishes nothing.
        #[cfg(test)]
        self.emitted_progress
            .lock()
            .expect("progress capture lock")
            .push((rendered.clone(), current, status));
        self.comm.update_calibration_progress(
            &self.target_bus_serial,
            current,
            self.total_steps,
            &rendered,
            status,
            error,
        );
    }

    /// PRG-3: DONE requires `executed_progress == expected_total`. If any
    /// reviewed operation was skipped the terminal event is REFUSED and no
    /// `Done` is published; the caller takes its existing failure route.
    fn mark_done(&self) -> Result<(), DynError> {
        // PRG-5 / PRG-6: a session may be declared successful only when the
        // total it published equals the reviewed grammar expansion of its own
        // validated LF mode. A rejected non-LF profile never reaches here.
        let derived = self.derive_expected_progress_total()?;
        if derived != self.total_steps {
            return Err(format!(
                "MATDOG progress envelope {} does not match the reviewed grammar total {derived}; terminal Done refused",
                self.total_steps
            )
            .into());
        }
        if self.current_step != self.total_steps {
            return Err(format!(
                "MATDOG progress incomplete: executed={}, expected={}; terminal Done refused",
                self.current_step, self.total_steps
            )
            .into());
        }
        self.publish_progress(self.total_steps, "completed", CalibrationStatus::Done, None);
        Ok(())
    }

    /// PRG-2 / PRG-5: derive the expected total by expanding the reviewed
    /// grammar of this engine's validated mode and counting its
    /// progress-emitting operations.
    fn derive_expected_progress_total(&self) -> Result<u32, DynError> {
        Ok(lf_expected_progress_total(self.session_mode()?))
    }

    fn mark_failed(&self, message: &str) {
        self.publish_progress(
            self.current_step,
            "failed",
            CalibrationStatus::Failed,
            Some(message),
        );
    }

    fn next_command_id(&mut self) -> Bytes {
        self.command_counter += 1;
        make_command_id(
            systime::get_app_start_id(),
            self.command_nonce,
            self.command_counter,
        )
    }
}

// ===========================================================================
// P2A-G3-B — THE TWELVE INTENT-SPECIFIC ENGINE MOTION OPERATIONS
//
// G2 contract section 8.2, with the per-operation preconditions of section 8.6.
//
// Each operation, in ONE call and in this order: derives its own value and
// evidence from the current EngineContext / node / session, checks mode,
// context step or grammar node, active motor, observation freshness and guard,
// and only then emits through exactly one of the two policy writers (AUTH-4).
// There is no authorize-now / emit-later path and no motion-authority value of
// any kind (AUTH-5, N-2).
//
// Arguments are IDENTIFIERS and EVIDENCE only. No operation accepts a raw tick,
// a raw motor id, a step size, a scout flag, a pose value or an evidence record
// as a caller-selected argument (DER-3).
// ===========================================================================
impl MatdogRamOnlyCalibrator {
    /// The reviewed LF session mode this engine was armed for.
    fn session_mode(&self) -> Result<LfSessionMode, DynError> {
        Ok(self.authority_context()?.mode())
    }

    fn authority_context(&self) -> Result<&EngineContext, DynError> {
        self.context.as_ref().ok_or_else(|| -> DynError {
            format!(
                "{}: MATDOG motion requires one of the six reviewed LF runtime session authorities",
                self.profile.label
            )
            .into()
        })
    }

    fn current_entry_step(&self) -> Result<EntryStep, DynError> {
        self.authority_context()?
            .entry_step()
            .ok_or_else(|| -> DynError {
                "startup-home writes are unreachable once the session context exists".into()
            })
    }

    fn current_goal_node(&self) -> Result<GoalNode, DynError> {
        self.goal_node
            .ok_or_else(|| -> DynError { "no reviewed motion node is currently entered".into() })
    }

    /// Resolve the LIVE session authority for a session-phase motion operation
    /// (CTX-4, AUTH-4, AUTH-6).
    ///
    /// This is the immediate context check every session-phase operation performs
    /// before it may reach a policy writer. It fails closed when the engine holds
    /// no LF authority at all — RF, RH, LH and the isolated LF hip profiles never
    /// obtain one — and when the engine is still in the pre-session entry phase.
    /// It never consults engine bookkeeping in place of the context.
    fn session_motion_authority(&self) -> Result<(LfSessionMode, GoalNode), DynError> {
        let mode = self.session_mode()?;
        match self.authority_context()? {
            EngineContext::Session(_) => {}
            EngineContext::Entry(_) => {
                return Err(
                    "session motion is unreachable from the pre-session entry context".into(),
                )
            }
        }
        Ok((mode, self.current_goal_node()?))
    }

    /// Resolve the LIVE session authority for the two pressure-release
    /// operations and return the active probe motor (CTX-4, SAFE-2).
    ///
    /// W12 and W13 are SESSION-phase operations: the context must be a
    /// `Session`, and an `Entry` context can never authorize them. No grammar
    /// node predicate is imposed on purpose — every reviewed pressure release,
    /// including the ones on the failure path (early stall, tracking failure),
    /// must stay issuable wherever the contact search fails, so narrowing this
    /// to a node would weaken SAFE-2 without adding any authority.
    fn session_probe_authority(&self) -> Result<u8, DynError> {
        self.session_mode()?;
        match self.authority_context()? {
            EngineContext::Session(_) => {}
            EngineContext::Entry(_) => {
                return Err(
                    "pressure release is unreachable from the pre-session entry context".into(),
                )
            }
        }
        let motor_id = self.profile.motor_id;
        if !self.derived_participants()?.contains(&motor_id) {
            return Err(
                format!("M{motor_id} is not a derived participant of this LF session").into(),
            );
        }
        // Where a persistent LF session exists, the probe must still be its
        // active motor. This restates, one step earlier, the binding the
        // session already enforces when the goal is written.
        if let Some(session) = &self.lf_session {
            match session.active {
                Some(active) if active.motor_id == motor_id => {}
                _ => {
                    return Err(
                        format!("M{motor_id} is not the active LF motor of this session").into(),
                    )
                }
            }
        }
        Ok(motor_id)
    }

    /// Advance the pre-session entry context. CTX-3 is enforced inside: the
    /// full-session normalization steps are unreachable in the legacy modes and
    /// the legacy profile-entry steps are unreachable in the full session.
    fn advance_entry_step(&mut self, step: EntryStep) -> Result<(), DynError> {
        let context = self.context.as_mut().ok_or_else(|| -> DynError {
            format!(
                "{}: MATDOG entry requires one of the six reviewed LF runtime session authorities",
                self.profile.label
            )
            .into()
        })?;
        context
            .advance_entry(step)
            .map_err(|message| -> DynError { message.into() })
    }

    /// Promote the pre-session Entry context to the Session context at the
    /// mode's first session-owned grammar node (G2 section 8.0).
    fn begin_engine_session(&mut self, node: GrammarNode) -> Result<(), DynError> {
        let context = self.context.take().ok_or_else(|| -> DynError {
            format!(
                "{}: MATDOG session creation requires a reviewed LF runtime session authority",
                self.profile.label
            )
            .into()
        })?;
        self.context = Some(
            context
                .begin_session(node)
                .map_err(|message| -> DynError { message.into() })?,
        );
        Ok(())
    }

    /// Enter a reviewed motion node. The grammar node is kept in step with the
    /// session context, so a node the selected mode does not own is refused
    /// before any motion (GRM-1, section 6.2).
    fn enter_goal_node(&mut self, node: GoalNode) -> Result<(), DynError> {
        let grammar = self.grammar_node_for(node);
        let context = self.context.as_mut().ok_or_else(|| -> DynError {
            format!(
                "{}: MATDOG motion requires a reviewed LF runtime session authority",
                self.profile.label
            )
            .into()
        })?;
        context
            .enter_node(grammar)
            .map_err(|message| -> DynError { message.into() })?;
        self.goal_node = Some(node);
        Ok(())
    }

    fn grammar_node_for(&self, node: GoalNode) -> GrammarNode {
        match node {
            GoalNode::Parking => GrammarNode::Parking,
            GoalNode::MovingBaseline
            | GoalNode::ProbeAdvance { .. }
            | GoalNode::Backoff
            | GoalNode::ProbeReturnHome => GrammarNode::ContactSearch {
                joint: self.profile.joint,
                side: self.profile.side,
            },
            GoalNode::StaticHold { joint } => GrammarNode::StaticHold { joint },
            GoalNode::ReturnStaged { joint } => GrammarNode::ReturnStaged { joint },
            GoalNode::RestoreParking => GrammarNode::RestoreParking,
            GoalNode::PostRestoreSettle | GoalNode::RestorePrerequisite { .. } => {
                GrammarNode::RestorePrerequisites
            }
        }
    }

    /// Session participants derived from the SELECTED SESSION MODE alone
    /// (G2 section 6.4).
    fn derived_participants(&self) -> Result<Vec<u8>, DynError> {
        lf_session_participants(self.session_mode()?)
            .map_err(|message| -> DynError { message.into() })
    }

    /// Rebind a fresh observation before it may influence a write (DER-1).
    fn rebind_fresh_observation(
        &self,
        motor_id: u8,
        offered: &FreshObservation,
        require_torque: bool,
    ) -> Result<MotorObservation, DynError> {
        if offered.motor_id != motor_id {
            return Err(format!(
                "fresh observation is bound to M{}, not to the derived motor M{motor_id}",
                offered.motor_id
            )
            .into());
        }
        self.ensure_observation_safe(motor_id, offered.observation, require_torque, None)?;
        Ok(offered.observation)
    }

    // --- W1, W2 -----------------------------------------------------------
    /// Startup q=0 prime while torque is OFF. Full-mode all-12 normalization
    /// (W1) and legacy home-only recovery (W2) share this operation and this
    /// exact literal `HOME_TICK` value; they differ only in the entry step that
    /// makes them reachable (CTX-3).
    async fn home_normalization_prime(&mut self) -> Result<(), DynError> {
        let motor_id = match self.current_entry_step()? {
            EntryStep::FullNormalization {
                motor,
                stage: NormalizationStage::Prime,
            } => motor,
            EntryStep::LegacyProfileEntry {
                motor,
                stage: LegacyEntryStage::Prime,
            } => {
                self.legacy_primed_motor = Some(motor);
                motor
            }
            step => {
                return Err(format!(
                    "W1/W2 startup q=0 prime is not reachable from entry step {step:?}"
                )
                .into())
            }
        };
        if !MATDOG_MOTOR_IDS.contains(&motor_id) {
            return Err(format!("M{motor_id} is outside the exact MATDOG motor set").into());
        }
        let observation = self.latest_observation(motor_id)?;
        self.ensure_observation_fresh(motor_id, observation)?;
        if observation.torque_enabled {
            return Err(format!(
                "M{motor_id} startup q=0 prime requires torque OFF before the write"
            )
            .into());
        }
        if !startup_home_initial_position_valid(observation.position) {
            return Err(format!(
                "M{motor_id} reported an invalid unsigned encoder position before q=0 normalization: present={}, valid=0..={}",
                observation.position,
                protocol::MAX_ANGLE_STEP
            )
            .into());
        }
        self.startup_home_goal_policy_write(motor_id, HOME_TICK)
            .await
    }

    // --- W3 ---------------------------------------------------------------
    /// Legacy home-only q=0 REASSERTION while torque is ON. This is a second,
    /// physically distinct write issued after the torque enable that follows
    /// W2; it is never merged with the prime (F1.3).
    async fn home_reassert_torque_on(&mut self) -> Result<(), DynError> {
        let motor_id = match self.current_entry_step()? {
            EntryStep::LegacyProfileEntry {
                motor,
                stage: LegacyEntryStage::Reassert,
            } => motor,
            step => {
                return Err(format!(
                    "W3 HOME reassertion is not reachable from entry step {step:?}"
                )
                .into())
            }
        };
        if self.legacy_primed_motor != Some(motor_id) {
            return Err(format!(
                "M{motor_id} HOME reassertion does not match the motor primed by W2"
            )
            .into());
        }
        let observation = self.latest_observation(motor_id)?;
        self.ensure_observation_fresh(motor_id, observation)?;
        if !observation.torque_enabled {
            return Err(format!(
                "M{motor_id} HOME reassertion requires torque ON after the preceding prime"
            )
            .into());
        }
        self.startup_home_goal_policy_write(motor_id, HOME_TICK)
            .await
    }

    // --- W4 ---------------------------------------------------------------
    /// Prime a motor at its own present observation before torque is enabled.
    async fn prime_at_present(
        &mut self,
        observation: &FreshObservation,
    ) -> Result<EmittedGoal, DynError> {
        let motor_id = observation.motor_id;
        // CTX-4 / AUTH-4 / AUTH-6: resolve and check the LIVE authority context
        // FIRST. Without one — RF, RH, LH or an isolated LF hip profile — no
        // path below this point exists, so no GoalPosition can be written.
        match self.authority_context()? {
            // W4 in the pre-session entry phase belongs to the legacy
            // prerequisite step and to that step's own motor.
            EngineContext::Entry(_) => match self.current_entry_step()? {
                EntryStep::LegacyProfileEntry {
                    motor,
                    stage: LegacyEntryStage::Prerequisite,
                } if motor == motor_id => {}
                step => {
                    return Err(format!(
                        "W4 prime of M{motor_id} is not reachable from entry step {step:?}"
                    )
                    .into())
                }
            },
            EngineContext::Session(_) => {}
        }
        if !self.derived_participants()?.contains(&motor_id) {
            return Err(
                format!("M{motor_id} is not a derived participant of this LF session").into(),
            );
        }
        if !self.profile.allowed_motor_ids.contains(&motor_id) {
            return Err(format!("M{motor_id} is outside armed profile motor allowlist").into());
        }
        let rebound = self.rebind_fresh_observation(motor_id, observation, false)?;
        let target = rebound.position;
        if self.lf_session.is_some() {
            self.set_lf_active(motor_id, target, LfActiveKind::Commanded)?;
        }
        self.armed_goal_policy_write(motor_id, target).await?;
        Ok(EmittedGoal { motor_id, target })
    }

    // --- W5, W6 -----------------------------------------------------------
    /// Whole-session parking move (W5, Parking node) and restart-safe
    /// prerequisite establishment (W6, legacy entry step). Both take no pose
    /// argument: the LF historical pose is derived from the node or from the
    /// armed profile's own prerequisite list (BRAND-3, section 11.5).
    async fn prerequisite_or_parking_move(&mut self) -> Result<EmittedGoal, DynError> {
        let emitted = match self.authority_context()? {
            EngineContext::Entry(_) => {
                let motor_id = match self.current_entry_step()? {
                    EntryStep::LegacyProfileEntry {
                        motor,
                        stage: LegacyEntryStage::Prerequisite,
                    } => motor,
                    step => {
                        let message =
                            format!("W6 prerequisite establishment is not reachable from {step:?}");
                        return Err(message.into());
                    }
                };
                let target = self
                    .profile
                    .prerequisites
                    .iter()
                    .find(|target| target.motor_id == motor_id)
                    .copied()
                    .ok_or_else(|| {
                        format!("M{motor_id} is not a prerequisite of the armed profile")
                    })?;
                EmittedGoal {
                    motor_id,
                    target: target.target_tick,
                }
            }
            EngineContext::Session(_) => {
                if self.current_goal_node()? != GoalNode::Parking {
                    return Err("W5 parking move is only reachable from the Parking node".into());
                }
                let parking = static_target(Leg::Lh, JointKind::Upper, UPPER_30_DELTA)
                    .map_err(|message| -> DynError { message.into() })?;
                EmittedGoal {
                    motor_id: parking.motor_id,
                    target: parking.target_tick,
                }
            }
        };
        let participants = self.derived_participants()?;
        if !participants.contains(&emitted.motor_id) {
            return Err(format!(
                "M{} is not a derived participant of this LF session",
                emitted.motor_id
            )
            .into());
        }
        if self.lf_session.is_some() {
            self.set_lf_active(emitted.motor_id, emitted.target, LfActiveKind::Commanded)?;
        }
        self.armed_goal_policy_write(emitted.motor_id, emitted.target)
            .await?;
        Ok(emitted)
    }

    // --- W7, W8 -----------------------------------------------------------
    /// Moving-current baseline step. The baseline RULE is derived from the
    /// validated mode: the full session moves RELATIVE to the present pose
    /// (W7), the two legacy modes move to their profile's ABSOLUTE baseline
    /// tick (W8). Neither is a caller choice (section 8.4).
    async fn moving_baseline_step(&mut self) -> Result<BaselineStep, DynError> {
        let (mode, node) = self.session_motion_authority()?;
        if node != GoalNode::MovingBaseline {
            return Err("W7/W8 moving baseline is only reachable from its own node".into());
        }
        let motor_id = self.profile.motor_id;
        let (target, relative_entry) = match mode {
            LfSessionMode::LfFullLegSession => {
                // W7 acquires exactly ONE entry observation. It is checked, it
                // derives the relative target, and it is reported back so the
                // caller's sample bookkeeping continues from that same
                // observation, exactly as the immutable source did.
                let initial = self.latest_observation(motor_id)?;
                self.ensure_observation_safe(motor_id, initial, true, None)?;
                let relative = advance_tick(
                    initial.position,
                    self.profile.probe_sign,
                    BASELINE_TRAVEL_TICKS,
                )?;
                if passed_guard(relative, self.profile.guard_tick, self.profile.probe_sign) {
                    return Err(
                        format!("{} baseline move would pass guard", self.profile.label).into(),
                    );
                }
                (relative, Some(initial))
            }
            LfSessionMode::LfHipPairLegacy | LfSessionMode::LfSingleContactLegacy { .. } => {
                // W8 is absolute: it derives no target from telemetry and
                // therefore reads none. Its caller keeps its own single
                // bookkeeping observation, unchanged.
                (self.profile.baseline_target_tick, None)
            }
        };
        self.armed_goal_policy_write(motor_id, target).await?;
        Ok(BaselineStep {
            emitted: EmittedGoal { motor_id, target },
            relative_entry,
        })
    }

    // --- W9, W10, W11 -----------------------------------------------------
    /// Contact-search advance. The step size comes from the current pass and
    /// the scout policy from the validated mode; the value is always
    /// `advance_tick(previous_target, probe_sign, step)` and the mechanical
    /// guard is checked immediately before the write (C-1, section 8.6).
    async fn probe_advance_step(&mut self) -> Result<EmittedGoal, DynError> {
        let (_mode, node) = self.session_motion_authority()?;
        let pass = match node {
            GoalNode::ProbeAdvance { pass } => pass,
            node => {
                return Err(
                    format!("W9/W10/W11 probe advance is not reachable from {node:?}").into(),
                )
            }
        };
        let step_ticks = self.derived_probe_step(pass);
        let motor_id = self.profile.motor_id;
        let previous_target = self
            .probe_target
            .ok_or("contact search advanced before its start target was established")?;
        let next_target = advance_tick(previous_target, self.profile.probe_sign, step_ticks)?;
        if passed_guard(
            next_target,
            self.profile.guard_tick,
            self.profile.probe_sign,
        ) {
            return Err(format!(
                "{} travel guard reached without contact: next={}, URDF={}, guard={}",
                self.profile.label,
                next_target,
                self.profile.urdf_limit_tick,
                self.profile.guard_tick
            )
            .into());
        }
        self.armed_goal_policy_write(motor_id, next_target).await?;
        self.probe_target = Some(next_target);
        Ok(EmittedGoal {
            motor_id,
            target: next_target,
        })
    }

    /// Derived, never chosen: coarse is always `COARSE_STEP_TICKS`, fine is
    /// always `FINE_STEP_TICKS`.
    const fn derived_probe_step(&self, pass: ProbePass) -> u16 {
        match pass {
            ProbePass::Coarse => COARSE_STEP_TICKS,
            ProbePass::Fine => FINE_STEP_TICKS,
        }
    }

    /// Derived, never chosen: a coarse pass never scouts; a fine pass scouts in
    /// the full session and in the single-contact legacy modes, and does NOT
    /// scout in the hip-pair legacy mode (section 6.2, W10 versus W11).
    fn derived_scout_policy(&self, pass: ProbePass) -> Result<Option<u16>, DynError> {
        Ok(derived_scout_policy_for(
            self.session_mode()?,
            pass,
            self.recorded_coarse_scout,
        ))
    }

    // --- W12 --------------------------------------------------------------
    /// Immediate pressure release at the position of the observation that
    /// produced the current detector verdict. It must remain issuable on the
    /// failure path; if it cannot be issued, the caller falls through to the
    /// existing verified global torque-OFF (SAFE-2).
    async fn stop_pressure_at_observation(
        &mut self,
        verdict: &ContactVerdict,
    ) -> Result<(), DynError> {
        // CTX-4: W12 is a SESSION-phase pressure release. The live context is
        // resolved and its variant checked before the write; an Entry context
        // never authorizes it.
        let motor_id = self.session_probe_authority()?;
        if verdict.motor_id != motor_id {
            return Err(format!(
                "detector verdict is bound to M{}, not to the active probe M{motor_id}",
                verdict.motor_id
            )
            .into());
        }
        self.armed_goal_policy_write(motor_id, verdict.observation.position)
            .await
    }

    // --- W13 --------------------------------------------------------------
    /// Immediate pressure release at a contact tick recorded earlier in THIS
    /// session. The value is never a fresh observation and never a caller
    /// argument.
    async fn stop_pressure_at_recorded_contact(&mut self) -> Result<(), DynError> {
        // CTX-4: W13 is likewise SESSION-phase; Entry never authorizes it.
        let motor_id = self.session_probe_authority()?;
        let recorded = self
            .recorded_contact_tick
            .ok_or("no accepted contact has been recorded in this session")?;
        self.armed_goal_policy_write(motor_id, recorded).await
    }

    // --- W14 --------------------------------------------------------------
    /// Bounded backoff derived from the contact tick just recorded in this
    /// node. It is refused if it would cross digital home.
    async fn backoff_step(&mut self) -> Result<EmittedGoal, DynError> {
        let (_mode, node) = self.session_motion_authority()?;
        if node != GoalNode::Backoff {
            return Err("W14 backoff is only reachable from its own node".into());
        }
        let motor_id = self.profile.motor_id;
        let contact_tick = self
            .node_contact_tick
            .ok_or("backoff requested before this node recorded a contact")?;
        let target = advance_tick(contact_tick, -self.profile.probe_sign, BACKOFF_TICKS)?;
        if crossed_home(target, self.profile.probe_sign) {
            return Err(format!("{} backoff crosses home: {target}", self.profile.label).into());
        }
        if self.lf_session.is_some() {
            self.set_lf_active(motor_id, target, LfActiveKind::Commanded)?;
        }
        self.armed_goal_policy_write(motor_id, target).await?;
        Ok(EmittedGoal { motor_id, target })
    }

    // --- W15, W16 ---------------------------------------------------------
    /// Static-hold transition of a completed joint to its LF historical pose.
    /// The pose is derived from the node's joint, never supplied.
    async fn static_hold_transition(&mut self) -> Result<EmittedGoal, DynError> {
        let (mode, node) = self.session_motion_authority()?;
        let joint = match node {
            GoalNode::StaticHold { joint } => joint,
            node => {
                return Err(format!(
                    "W15/W16 static-hold transition is not reachable from {node:?}"
                )
                .into())
            }
        };
        if mode != LfSessionMode::LfFullLegSession {
            return Err("static-hold transitions exist only in the full LF session".into());
        }
        let delta = match joint {
            JointKind::Upper => UPPER_90_DELTA,
            JointKind::Lower => LOWER_FOLDED_DELTA,
            JointKind::Hip => {
                return Err("the LF hip has no static-hold transition".into());
            }
        };
        let pose = static_target(Leg::Lf, joint, delta)
            .map_err(|message| -> DynError { message.into() })?;
        if self.lf_session.is_some() {
            self.set_lf_active(pose.motor_id, pose.target_tick, LfActiveKind::Commanded)?;
        }
        self.armed_goal_policy_write(pose.motor_id, pose.target_tick)
            .await?;
        Ok(EmittedGoal {
            motor_id: pose.motor_id,
            target: pose.target_tick,
        })
    }

    // --- W17, W18, W19 ----------------------------------------------------
    /// Staged affine q0 return. The value is THIS session's accepted affine
    /// estimate for THAT joint; the fixed-scale diagnostic can never satisfy it
    /// and the staged shift may not exceed the model-zero cap (section 8.6).
    async fn staged_affine_q0(&mut self) -> Result<EmittedGoal, DynError> {
        let (mode, node) = self.session_motion_authority()?;
        let joint = match node {
            GoalNode::ReturnStaged { joint } => joint,
            node => {
                return Err(format!("W17/W18/W19 staged q0 is not reachable from {node:?}").into())
            }
        };
        if mode != LfSessionMode::LfFullLegSession {
            return Err("staged affine q0 returns exist only in the full LF session".into());
        }
        let index = lf_joint_index(joint);
        let session = self
            .lf_session
            .as_ref()
            .ok_or("staged affine q0 requested without a live LF session")?;
        let contacts = session.contacts[index]
            .ok_or_else(|| format!("{:?} has no recorded contacts in this session", joint))?;
        let affine = session.affine[index].ok_or_else(|| {
            format!(
                "{:?} has no accepted affine evidence in this session",
                joint
            )
        })?;
        if !affine.accepted || !lf_contact_witness_accepted(joint, contacts) {
            return Err(format!(
                "{:?} staged q0 refused: affine.accepted={}, contact witness accepted={}",
                joint,
                affine.accepted,
                lf_contact_witness_accepted(joint, contacts)
            )
            .into());
        }
        let target = affine.estimated_zero_tick;
        if circular_distance(target, HOME_TICK) > MODEL_ZERO_MAX_SHIFT_FROM_DIGITAL_HOME_TICKS {
            return Err(format!(
                "{:?} staged q0 shift exceeds the digital-home cap: target={target}, cap={}",
                joint, MODEL_ZERO_MAX_SHIFT_FROM_DIGITAL_HOME_TICKS
            )
            .into());
        }
        let motor_id = spec_for(Leg::Lf, joint).motor_id;
        if self.lf_session.is_some() {
            self.set_lf_active(motor_id, target, LfActiveKind::Commanded)?;
        }
        self.armed_goal_policy_write(motor_id, target).await?;
        Ok(EmittedGoal { motor_id, target })
    }

    // --- W20, W21, W22, W23 -----------------------------------------------
    /// Return to digital home. The motor is the current node's selected restore
    /// or probe motor, the value is always exactly `HOME_TICK`, and the settle
    /// tolerance is the node's own (16 for W21, 10 for W22).
    async fn return_home(&mut self) -> Result<EmittedGoal, DynError> {
        let (mode, node) = self.session_motion_authority()?;
        let motor_id = match node {
            GoalNode::RestoreParking => {
                if mode != LfSessionMode::LfFullLegSession {
                    return Err("parking restore exists only in the full LF session".into());
                }
                spec_for(Leg::Lh, JointKind::Upper).motor_id
            }
            GoalNode::ProbeReturnHome | GoalNode::PostRestoreSettle => self.profile.motor_id,
            GoalNode::RestorePrerequisite { motor_id } => motor_id,
            node => {
                return Err(format!("W20..W23 return home is not reachable from {node:?}").into())
            }
        };
        let participants = self.derived_participants()?;
        if !participants.contains(&motor_id) {
            return Err(
                format!("M{motor_id} is not a derived participant of this LF session").into(),
            );
        }
        if self.lf_session.is_some() {
            self.set_lf_active(motor_id, HOME_TICK, LfActiveKind::Commanded)?;
        }
        self.armed_goal_policy_write(motor_id, HOME_TICK).await?;
        Ok(EmittedGoal {
            motor_id,
            target: HOME_TICK,
        })
    }
}

fn find_motor<'a>(
    state: &'a InferenceState,
    bus_serial: &str,
    motor_id: u8,
) -> Result<&'a crate::st3215_proto::inference_state::MotorState, DynError> {
    let bus = state
        .buses
        .iter()
        .find(|bus| bus.bus.as_ref().map(|bus| bus.serial_number.as_str()) == Some(bus_serial))
        .ok_or_else(|| format!("ST3215 bus not found: {bus_serial}"))?;
    bus.motors
        .iter()
        .find(|motor| motor.id == motor_id as u32)
        .ok_or_else(|| format!("M{motor_id} not found on bus {bus_serial}").into())
}

fn motor_ids_for_bus(state: &InferenceState, bus_serial: &str) -> Result<Vec<u8>, DynError> {
    let bus = state
        .buses
        .iter()
        .find(|bus| bus.bus.as_ref().map(|bus| bus.serial_number.as_str()) == Some(bus_serial))
        .ok_or_else(|| format!("ST3215 bus not found: {bus_serial}"))?;
    bus.motors
        .iter()
        .map(|motor| {
            u8::try_from(motor.id)
                .map_err(|_| format!("invalid ST3215 motor ID in inference: {}", motor.id).into())
        })
        .collect()
}

fn motor_ram_register_matches(
    motor: &crate::st3215_proto::inference_state::MotorState,
    register: RamRegister,
    expected: &[u8],
) -> bool {
    let address = register.address() as usize;
    motor.state.len() >= address + expected.len()
        && &motor.state[address..address + expected.len()] == expected
}

fn observation_from_state(
    state: &InferenceState,
    bus_serial: &str,
    motor_id: u8,
) -> Result<MotorObservation, DynError> {
    let motor = find_motor(state, bus_serial, motor_id)?;
    let bytes = motor.state.as_ref();
    let torque_limit_addr = RamRegister::TorqueLimit.address() as usize;
    let temperature_limit_addr = MAX_TEMPERATURE_LIMIT_ADDRESS;
    let temperature_addr = RamRegister::PresentTemperature.address() as usize;
    let status_addr = RamRegister::Status.address() as usize;
    if bytes.len() < RamRegister::PresentCurrent.address() as usize + 2
        || bytes.len() < torque_limit_addr + 2
        || bytes.len() <= status_addr
    {
        return Err(format!("M{motor_id} state too short: {} bytes", bytes.len()).into());
    }
    Ok(MotorObservation {
        monotonic_stamp_ns: motor.monotonic_stamp_ns,
        position: protocol::get_motor_position(bytes),
        velocity: protocol::get_motor_velocity(bytes),
        current: protocol::get_motor_current(bytes),
        temperature: bytes[temperature_addr],
        temperature_limit: bytes[temperature_limit_addr],
        goal_position: protocol::get_motor_goal_position(bytes),
        torque_limit: u16::from_le_bytes([bytes[torque_limit_addr], bytes[torque_limit_addr + 1]]),
        torque_enabled: protocol::is_torque_enabled(bytes),
        status: bytes[status_addr],
        has_driver_error: motor.error.is_some(),
    })
}

fn command_result_for(state: &InferenceState, bus_serial: &str, command_id: &Bytes) -> Option<i32> {
    let bus = state
        .buses
        .iter()
        .find(|bus| bus.bus.as_ref().map(|bus| bus.serial_number.as_str()) == Some(bus_serial))?;
    bus.motors.iter().find_map(|motor| {
        let last = motor.last_command.as_ref()?;
        let command = last.command.as_ref()?;
        (&command.command_id == command_id).then_some(last.result)
    })
}

fn median(values: &[u16]) -> u16 {
    let mut sorted = values.to_vec();
    sorted.sort_unstable();
    sorted[sorted.len() / 2]
}

fn repeatability_spread(first_tick: u16, second_tick: u16) -> Result<u16, DynError> {
    let spread = circular_distance(first_tick, second_tick);
    if spread > REPEATABILITY_TOLERANCE_TICKS {
        Err(format!(
            "contact not repeatable: first={first_tick}, second={second_tick}, spread={spread}"
        )
        .into())
    } else {
        Ok(spread)
    }
}

fn make_command_id(app_start_id: u64, nonce: u64, counter: u64) -> Bytes {
    let mut bytes = Vec::with_capacity(24);
    bytes.extend_from_slice(&app_start_id.to_le_bytes());
    bytes.extend_from_slice(&nonce.to_le_bytes());
    bytes.extend_from_slice(&counter.to_le_bytes());
    Bytes::from(bytes)
}

fn signed_tick_delta(value: u16, reference: u16) -> i16 {
    ((value as i32 - reference as i32 + TICKS_PER_REVOLUTION / 2).rem_euclid(TICKS_PER_REVOLUTION)
        - TICKS_PER_REVOLUTION / 2) as i16
}

fn circular_distance(a: u16, b: u16) -> u16 {
    signed_tick_delta(a, b).unsigned_abs()
}

fn directional_progress(value: u16, reference: u16, sign: i8) -> u16 {
    (i32::from(signed_tick_delta(value, reference)) * i32::from(sign)).max(0) as u16
}

fn advance_tick(value: u16, sign: i8, amount: u16) -> Result<u16, DynError> {
    let next = i32::from(value) + i32::from(sign) * i32::from(amount);
    u16::try_from(next)
        .ok()
        .filter(|tick| *tick <= protocol::MAX_ANGLE_STEP)
        .ok_or_else(|| format!("unsigned GoalPosition out of range: {next}").into())
}

fn motion_timeout_for_distance(distance_ticks: u16) -> Duration {
    let travel_ms = u64::from(distance_ticks)
        .saturating_mul(1000)
        .saturating_add(MIN_EXPECTED_MOTION_TICKS_PER_SECOND - 1)
        / MIN_EXPECTED_MOTION_TICKS_PER_SECOND;
    Duration::from_millis(travel_ms)
        .saturating_add(MOTION_SETTLE_MARGIN)
        .max(MOTION_TIMEOUT)
}

fn passed_guard(value: u16, guard: u16, sign: i8) -> bool {
    if sign < 0 {
        value < guard
    } else {
        value > guard
    }
}

fn crossed_home(value: u16, probe_sign: i8) -> bool {
    if probe_sign < 0 {
        value > HOME_TICK
    } else {
        value < HOME_TICK
    }
}

fn speed_magnitude(raw: u16) -> u16 {
    raw & 0x7FFF
}

// ===========================================================================
// P2A-G3-A — STRUCTURAL FOUNDATION
//
// Contract: MATDOG_PHASE2A_G2_GENERIC_SPEC_CONTRACT_2026-08-13.md revision
// 2.2d, sha256 e436ed6975d120bc6f394ca88674803b5e5d54caf9ad6d4da07fc1d9b79798a3.
//
// This block introduces ONLY the structural boundaries the later generic
// extraction needs:
//
//   section 5.2   the exhaustive six-token LF runtime arm resolver
//   section 6.2   LfSessionMode — the three reviewed LF session modes
//   section 4.4   the sealed LF armable brand and validate_lf_v25
//   section 4.5   LF choreography derived from the sealed mode, never read
//                 from a raw spec
//   section 13    the LF V25 hardware oracle binding
//   section 8.0   the pre-session Entry context and the Session context
//   section 20    ONE LegSessionStateMachine as the single engine name
//
// It owns NO motion. The twenty-three historical GoalPosition write paths of
// contract section 7 keep their existing callers, values, torque state and
// ordering, and the twelve engine operations / two policy writers / one raw
// constructor graph of section 8.3 is deliberately NOT extracted here. Every
// item below is currently reachable only from the MATDOG test module, which
// is why the dead-code allowances are scoped to it: wiring the engine onto
// this foundation is the next reviewed slice, not this one.
// ===========================================================================

/// Contract section 6.2: the only two joints an LF single-contact session may
/// select. Isolated Hip is not representable here — it stays hardware-blocked
/// (ARM-3r).
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum UpperOrLower {
    Upper,
    Lower,
}

impl UpperOrLower {
    #[allow(dead_code)]
    const fn joint_kind(self) -> JointKind {
        match self {
            Self::Upper => JointKind::Upper,
            Self::Lower => JointKind::Lower,
        }
    }
}

/// Contract section 6.2: the three — and only three — reviewed LF session
/// modes. All three are expansions of the one engine grammar; none of them is
/// a separate state machine.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LfSessionMode {
    /// Immutable 58-operation full-leg calibration. Arm `LF_LEG_STATE_MACHINE`.
    LfFullLegSession,
    /// Immutable HIP MIN+MAX shared-geometry sequence. Arm `LF_HIP_M13_MIN_MAX`.
    LfHipPairLegacy,
    /// Immutable single-contact sessions: `LF_UPPER_M12_{MIN,MAX}` and
    /// `LF_LOWER_M11_{MIN,MAX}`.
    LfSingleContactLegacy {
        joint: UpperOrLower,
        side: ContactSide,
    },
}

/// Contract section 5.2: Phase 2A runtime arm resolution, exhaustive.
///
/// Exactly six historical tokens resolve to a runtime LF session. Everything
/// else fails closed, including the two isolated LF Hip profiles (ARM-3r) and
/// all eighteen RF/RH/LH single-contact tokens (ARM-1). Offline enumeration of
/// all twenty-four profiles through `all_profiles` / `profile_for_arm_value` is
/// unchanged and carries no motion capability (ARM-5).
#[allow(dead_code)]
fn lf_runtime_session_mode(arm_value: &str) -> Result<LfSessionMode, String> {
    match arm_value {
        "LF_LEG_STATE_MACHINE" => Ok(LfSessionMode::LfFullLegSession),
        "LF_HIP_M13_MIN_MAX" => Ok(LfSessionMode::LfHipPairLegacy),
        "LF_UPPER_M12_MIN" => Ok(LfSessionMode::LfSingleContactLegacy {
            joint: UpperOrLower::Upper,
            side: ContactSide::Min,
        }),
        "LF_UPPER_M12_MAX" => Ok(LfSessionMode::LfSingleContactLegacy {
            joint: UpperOrLower::Upper,
            side: ContactSide::Max,
        }),
        "LF_LOWER_M11_MIN" => Ok(LfSessionMode::LfSingleContactLegacy {
            joint: UpperOrLower::Lower,
            side: ContactSide::Min,
        }),
        "LF_LOWER_M11_MAX" => Ok(LfSessionMode::LfSingleContactLegacy {
            joint: UpperOrLower::Lower,
            side: ContactSide::Max,
        }),
        _ => Err(lf_runtime_arm_rejection(arm_value)),
    }
}

/// Classifies a value the runtime resolver refused, without widening the
/// resolver itself. Recognized historical profile data stays recognized; it
/// simply has no Phase 2A runtime session.
#[allow(dead_code)]
fn lf_runtime_arm_rejection(arm_value: &str) -> String {
    match profile_for_arm_value(arm_value) {
        Ok(profile) if profile.leg == Leg::Lf && profile.joint == JointKind::Hip => {
            format!("{arm_value}: {HIP_HARDWARE_BLOCK_REASON}")
        }
        Ok(_) => format!(
            "{arm_value}: recognized offline MATDOG profile data with no Phase 2A LF runtime session"
        ),
        Err(_) => format!("unsupported MATDOG LF runtime arm value: {arm_value:?}"),
    }
}

/// Contract section 6.4: the session-scoped hold targets a mode owns, derived
/// from the immutable profile builders. No new pose value is introduced.
#[allow(dead_code)]
fn lf_mode_hold_targets(mode: LfSessionMode) -> Result<Vec<StaticTarget>, String> {
    let profile = match mode {
        LfSessionMode::LfFullLegSession => lf_full_sequence_profile()?,
        LfSessionMode::LfHipPairLegacy => lf_hip_sequence_profile(ContactSide::Min)?,
        LfSessionMode::LfSingleContactLegacy { joint, side } => {
            build_profile(Leg::Lf, joint.joint_kind(), side)?
        }
    };
    Ok(profile.prerequisites)
}

/// Contract section 6.4: runtime participants are derived from the SELECTED
/// SESSION MODE, never declared by data, and never from an endpoint-local
/// geometry dependency (PART-1).
#[allow(dead_code)]
fn lf_session_participants(mode: LfSessionMode) -> Result<Vec<u8>, String> {
    let mut participants = vec![
        spec_for(Leg::Lf, JointKind::Hip).motor_id,
        spec_for(Leg::Lf, JointKind::Upper).motor_id,
        spec_for(Leg::Lf, JointKind::Lower).motor_id,
    ];
    for target in lf_mode_hold_targets(mode)? {
        if !participants.contains(&target.motor_id) {
            participants.push(target.motor_id);
        }
    }
    participants.sort_unstable();
    Ok(participants)
}

/// Contract section 13: the LF hardware oracle — opaque, `JointKind`-keyed and
/// non-spoofable.
///
/// The only oracle is the immutable release `release/matdog-lf-calibrator-v25`
/// (`f87dd1fbc7e8100d275c74f9af448642f3429680`). The tick witnesses and the
/// tolerance delegate to the immutable runtime values already used by the LF
/// contact witness gate; this module adds no new coordinate.
#[allow(dead_code)]
mod lf_v25_oracle {
    use super::{ContactSide, JointKind, LF_CONTACT_WITNESS_TOLERANCE_TICKS};

    /// LF physical hardware evidence artifact (contract section 3.1).
    const LF_PHYSICAL_EVIDENCE_SHA256: &str =
        "6eae3201a00b5299550028d5b4e1e73d67520deccf5a85e548f3b07b1777cab4";
    /// Reconciler source identity — a DIFFERENT identity (contract IMP-20).
    const LF_RECONCILER_SOURCE_SHA256: &str =
        "111da4045c983c4f3a5acd0061dbbe3c7b77fbda7e120693a927f5f0a40f5dfe";

    /// Inert recorded geometry-versus-hardware comparison (contract section 2).
    /// Evidence only: it grants nothing and never upgrades a target domain.
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub(super) struct HardwareEvidenceBinding {
        pub(super) joint: JointKind,
        pub(super) side: ContactSide,
        pub(super) geometry_degrees: f64,
        pub(super) hardware_degrees: f64,
        pub(super) delta_degrees: f64,
        pub(super) agrees: bool,
    }

    const LF_V25_RECONCILIATION: [HardwareEvidenceBinding; 6] = [
        HardwareEvidenceBinding {
            joint: JointKind::Hip,
            side: ContactSide::Min,
            geometry_degrees: -46.012,
            hardware_degrees: -42.803,
            delta_degrees: -3.209,
            agrees: false,
        },
        HardwareEvidenceBinding {
            joint: JointKind::Hip,
            side: ContactSide::Max,
            geometry_degrees: 45.223,
            hardware_degrees: 39.375,
            delta_degrees: 5.848,
            agrees: false,
        },
        HardwareEvidenceBinding {
            joint: JointKind::Upper,
            side: ContactSide::Min,
            geometry_degrees: -52.133,
            hardware_degrees: -53.525,
            delta_degrees: 1.393,
            agrees: true,
        },
        HardwareEvidenceBinding {
            joint: JointKind::Upper,
            side: ContactSide::Max,
            geometry_degrees: 121.875,
            hardware_degrees: 122.607,
            delta_degrees: -0.732,
            agrees: true,
        },
        HardwareEvidenceBinding {
            joint: JointKind::Lower,
            side: ContactSide::Min,
            geometry_degrees: -92.074,
            hardware_degrees: -91.846,
            delta_degrees: -0.229,
            agrees: true,
        },
        HardwareEvidenceBinding {
            joint: JointKind::Lower,
            side: ContactSide::Max,
            geometry_degrees: 38.180,
            hardware_degrees: 34.277,
            delta_degrees: 3.902,
            agrees: false,
        },
    ];

    /// Opaque. The private field is the seal: no code outside this module can
    /// construct one, so no non-LF path can name the oracle or its tick data
    /// (ORA-5).
    pub(super) struct LfV25Oracle {
        _sealed: (),
    }

    static LF_V25_ORACLE: LfV25Oracle = LfV25Oracle { _sealed: () };

    /// Module-private constructor; the only way to obtain the oracle.
    pub(super) fn lf_v25_oracle() -> &'static LfV25Oracle {
        &LF_V25_ORACLE
    }

    impl LfV25Oracle {
        /// Keyed, never positional (ORA-4).
        pub(super) fn reference_contact_ticks(&self, joint: JointKind) -> (u16, u16) {
            super::lf_reference_contact_ticks(joint)
        }

        pub(super) fn tolerance_ticks(&self) -> u16 {
            LF_CONTACT_WITNESS_TOLERANCE_TICKS
        }

        pub(super) fn reconciliation(
            &self,
            joint: JointKind,
            side: ContactSide,
        ) -> HardwareEvidenceBinding {
            *LF_V25_RECONCILIATION
                .iter()
                .find(|record| record.joint == joint && record.side == side)
                .expect("complete six-record LF V25 reconciliation")
        }

        pub(super) fn physical_evidence_sha256(&self) -> &'static str {
            LF_PHYSICAL_EVIDENCE_SHA256
        }

        pub(super) fn reconciler_source_sha256(&self) -> &'static str {
            LF_RECONCILER_SOURCE_SHA256
        }
    }
}

/// Contract sections 4, 4.3, 4.4 and 4.5: the inert raw spec, the exact LF
/// canonical identity validation, and the sealed LF armable brand.
#[allow(dead_code)]
mod lf_brand {
    use super::{
        lf_runtime_session_mode, lf_session_participants, lf_v25_oracle, spec_for, ContactSide,
        JointKind, JointSpec, Leg, LfSessionMode, LfV25Oracle,
    };

    /// One raw joint identity entry. Inert and unvalidated.
    #[derive(Debug, Clone, PartialEq, Eq)]
    pub(super) struct RawJointIdentity {
        pub(super) kind: JointKind,
        pub(super) name: String,
        pub(super) motor_id: u8,
        pub(super) direction: i8,
        pub(super) min_delta: i16,
        pub(super) max_delta: i16,
    }

    /// Raw parking dependency reference (LFID-8).
    #[derive(Debug, Clone, PartialEq, Eq)]
    pub(super) struct RawParkingReference {
        pub(super) leg: Leg,
        pub(super) kind: JointKind,
        pub(super) joint_name: String,
        pub(super) motor_id: u8,
    }

    /// Raw historical oracle identity (LFID-6).
    #[derive(Debug, Clone, PartialEq, Eq)]
    pub(super) struct RawOracleIdentity {
        pub(super) physical_evidence_sha256: String,
        pub(super) reconciler_source_sha256: String,
        pub(super) tolerance_ticks: u16,
        pub(super) reference_contact_ticks: [(JointKind, u16, u16); 3],
    }

    /// Contract section 4.5: motion-bearing choreography a raw spec may carry
    /// and that `validate_lf_v25` NEVER reads (BRAND-1, BRAND-2). Mutating any
    /// field here cannot change runtime motion, because nothing reads it.
    #[derive(Debug, Clone, Default, PartialEq, Eq)]
    pub(super) struct RawChoreographyHints {
        pub(super) joint_order: Vec<JointKind>,
        pub(super) side_order: Vec<[ContactSide; 2]>,
        pub(super) prerequisite_pose_deltas: Vec<i16>,
        pub(super) parking_held_whole_session: bool,
        pub(super) restore_order: Vec<u8>,
        pub(super) historical_pose_source: String,
    }

    /// Contract section 4: inert, leg-agnostic, unvalidated. Constructible
    /// anywhere. No engine consumer, no motion operation, no oracle accessor.
    #[derive(Debug, Clone, PartialEq, Eq)]
    pub(super) struct RawLegCalibrationSpec {
        pub(super) leg: Leg,
        pub(super) requested_arm_value: String,
        pub(super) joints: [RawJointIdentity; 3],
        pub(super) parking_reference: Option<RawParkingReference>,
        pub(super) oracle_identity: RawOracleIdentity,
        /// NEVER read by `validate_lf_v25`.
        pub(super) choreography: RawChoreographyHints,
    }

    /// One rule of contract section 4.3, so every violation is rejected and
    /// attributable individually.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub(super) enum LfIdentityError {
        /// LFID-1 `leg == Leg::Lf`.
        Leg,
        /// LFID-2 exact joint names, one entry per canonical joint.
        JointName,
        /// LFID-3 motor IDs exactly Hip 13, Upper 12, Lower 11.
        MotorId,
        /// LFID-4 directions exactly Hip -1, Upper +1, Lower -1.
        Direction,
        /// LFID-5 limits exactly -512/+512, -597/+1394, -1047/+427.
        Limits,
        /// LFID-6 historical LF V25 oracle identity.
        OracleIdentity,
        /// LFID-7 the requested session mode is a reviewed LF mode.
        SessionMode,
        /// LFID-8 the parking dependency is exactly M42 = LH upper.
        ParkingIdentity,
        /// LFID-9 derived participants are exactly {11,12,13} u {42}.
        Participants,
    }

    impl LfIdentityError {
        pub(super) const fn rule(self) -> &'static str {
            match self {
                Self::Leg => "LFID-1",
                Self::JointName => "LFID-2",
                Self::MotorId => "LFID-3",
                Self::Direction => "LFID-4",
                Self::Limits => "LFID-5",
                Self::OracleIdentity => "LFID-6",
                Self::SessionMode => "LFID-7",
                Self::ParkingIdentity => "LFID-8",
                Self::Participants => "LFID-9",
            }
        }
    }

    /// The sealed inner representation. Private TO THIS MODULE, so no code
    /// outside `lf_brand` — including the rest of the calibrator module and its
    /// test module — can construct or mutate a branded spec (SEAL-1).
    struct ArmableInner {
        mode: LfSessionMode,
        joints: [JointSpec; 3],
        participants: Vec<u8>,
    }

    /// LF ONLY. The single spec type the engine will accept.
    ///
    /// Deliberately NOT `Clone`, NOT `Copy`, NOT `Default`, NOT deserializable,
    /// with no public inner constructor and no escape hatch: the only safe
    /// producer is `validate_lf_v25` (SEAL-2).
    pub(super) struct ArmableLfSessionSpec {
        inner: ArmableInner,
    }

    /// The ONLY safe producer. Performs LFID-1..LFID-9 of contract section 4.3.
    ///
    /// Every motion-bearing field is DERIVED here from the sealed mode and the
    /// immutable canonical LF joint table; `raw.choreography` is never read
    /// (BRAND-1).
    pub(super) fn validate_lf_v25(
        raw: &RawLegCalibrationSpec,
        mode: LfSessionMode,
    ) -> Result<ArmableLfSessionSpec, LfIdentityError> {
        // LFID-1
        if raw.leg != Leg::Lf {
            return Err(LfIdentityError::Leg);
        }

        // LFID-7 — the requested arm value must resolve to exactly this mode.
        if lf_runtime_session_mode(&raw.requested_arm_value) != Ok(mode) {
            return Err(LfIdentityError::SessionMode);
        }

        // LFID-2..LFID-5 — every canonical joint exactly once, in canonical
        // order, compared field by field against the immutable joint table.
        let canonical_kinds = [JointKind::Hip, JointKind::Upper, JointKind::Lower];
        for (index, kind) in canonical_kinds.into_iter().enumerate() {
            let raw_joint = &raw.joints[index];
            if raw_joint.kind != kind {
                return Err(LfIdentityError::JointName);
            }
            let canonical = spec_for(Leg::Lf, kind);
            if raw_joint.name != canonical.name {
                return Err(LfIdentityError::JointName);
            }
            if raw_joint.motor_id != canonical.motor_id {
                return Err(LfIdentityError::MotorId);
            }
            if raw_joint.direction != canonical.direction {
                return Err(LfIdentityError::Direction);
            }
            if raw_joint.min_delta != canonical.min_delta
                || raw_joint.max_delta != canonical.max_delta
            {
                return Err(LfIdentityError::Limits);
            }
        }

        // LFID-6
        let oracle = lf_v25_oracle();
        if raw.oracle_identity.physical_evidence_sha256 != oracle.physical_evidence_sha256()
            || raw.oracle_identity.reconciler_source_sha256 != oracle.reconciler_source_sha256()
            || raw.oracle_identity.tolerance_ticks != oracle.tolerance_ticks()
        {
            return Err(LfIdentityError::OracleIdentity);
        }
        for (kind, minimum, maximum) in raw.oracle_identity.reference_contact_ticks {
            if (minimum, maximum) != oracle.reference_contact_ticks(kind) {
                return Err(LfIdentityError::OracleIdentity);
            }
        }
        for kind in canonical_kinds {
            if !raw
                .oracle_identity
                .reference_contact_ticks
                .iter()
                .any(|(recorded, _, _)| *recorded == kind)
            {
                return Err(LfIdentityError::OracleIdentity);
            }
        }

        // LFID-8 — the parking dependency, when present, is exactly M42 = LH
        // upper, and its motor is the canonical motor of the joint it names.
        if let Some(parking) = raw.parking_reference.as_ref() {
            let canonical = spec_for(Leg::Lh, JointKind::Upper);
            if parking.leg != Leg::Lh
                || parking.kind != JointKind::Upper
                || parking.joint_name != canonical.name
                || parking.motor_id != canonical.motor_id
            {
                return Err(LfIdentityError::ParkingIdentity);
            }
        }

        // LFID-9 — participants derived from the mode alone.
        let participants =
            lf_session_participants(mode).map_err(|_| LfIdentityError::Participants)?;
        let expected = {
            let mut expected = vec![
                spec_for(Leg::Lf, JointKind::Hip).motor_id,
                spec_for(Leg::Lf, JointKind::Upper).motor_id,
                spec_for(Leg::Lf, JointKind::Lower).motor_id,
                spec_for(Leg::Lh, JointKind::Upper).motor_id,
            ];
            expected.sort_unstable();
            expected
        };
        if participants != expected {
            return Err(LfIdentityError::Participants);
        }

        Ok(ArmableLfSessionSpec {
            inner: ArmableInner {
                mode,
                joints: [
                    *spec_for(Leg::Lf, JointKind::Hip),
                    *spec_for(Leg::Lf, JointKind::Upper),
                    *spec_for(Leg::Lf, JointKind::Lower),
                ],
                participants,
            },
        })
    }

    impl ArmableLfSessionSpec {
        pub(super) fn mode(&self) -> LfSessionMode {
            self.inner.mode
        }

        pub(super) fn joints(&self) -> &[JointSpec; 3] {
            &self.inner.joints
        }

        pub(super) fn participants(&self) -> &[u8] {
            &self.inner.participants
        }

        /// SEAL-3 / ORA-1: the oracle is reachable only through the brand.
        pub(super) fn oracle(&self) -> &'static LfV25Oracle {
            lf_v25_oracle()
        }
    }

    /// The compiled-in canonical LF V25 raw spec (contract section 3.3: G3
    /// reads no geometry artifact). It is still inert: it must pass
    /// `validate_lf_v25` like any other raw spec.
    pub(super) fn canonical_lf_v25_raw_spec(arm_value: &str) -> RawLegCalibrationSpec {
        let oracle = lf_v25_oracle();
        let raw_identity = |kind: JointKind| {
            let canonical = spec_for(Leg::Lf, kind);
            RawJointIdentity {
                kind,
                name: canonical.name.to_string(),
                motor_id: canonical.motor_id,
                direction: canonical.direction,
                min_delta: canonical.min_delta,
                max_delta: canonical.max_delta,
            }
        };
        let parking = spec_for(Leg::Lh, JointKind::Upper);
        RawLegCalibrationSpec {
            leg: Leg::Lf,
            requested_arm_value: arm_value.to_string(),
            joints: [
                raw_identity(JointKind::Hip),
                raw_identity(JointKind::Upper),
                raw_identity(JointKind::Lower),
            ],
            parking_reference: Some(RawParkingReference {
                leg: Leg::Lh,
                kind: JointKind::Upper,
                joint_name: parking.name.to_string(),
                motor_id: parking.motor_id,
            }),
            oracle_identity: RawOracleIdentity {
                physical_evidence_sha256: oracle.physical_evidence_sha256().to_string(),
                reconciler_source_sha256: oracle.reconciler_source_sha256().to_string(),
                tolerance_ticks: oracle.tolerance_ticks(),
                reference_contact_ticks: [JointKind::Hip, JointKind::Upper, JointKind::Lower].map(
                    |kind| {
                        let (minimum, maximum) = oracle.reference_contact_ticks(kind);
                        (kind, minimum, maximum)
                    },
                ),
            },
            choreography: RawChoreographyHints::default(),
        }
    }
}

/// Contract section 6.1: the fixed lifecycle grammar nodes of the one engine.
/// Structural only in G3-A — no node performs, authorizes or gates a write.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum GrammarNode {
    InitialRecovery,
    Parking,
    Prerequisites,
    ContactSearch { joint: JointKind, side: ContactSide },
    StaticHold { joint: JointKind },
    Diagnostics,
    ReturnStaged { joint: JointKind },
    RestoreParking,
    RestorePrerequisites,
    Cleanup,
    TorqueOff,
}

/// Contract section 8.0: which entry step is currently executing.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EntryStep {
    ExactSetVerify,
    GlobalTorqueOff,
    /// `LfFullLegSession` only (CTX-3).
    FullNormalization {
        motor: u8,
        stage: NormalizationStage,
    },
    /// `LfHipPairLegacy` / `LfSingleContactLegacy` only (CTX-3).
    LegacyProfileEntry {
        motor: u8,
        stage: LegacyEntryStage,
    },
}

/// Prime writes (W1); Settle does not write.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum NormalizationStage {
    Prime,
    Settle,
}

/// W2 (torque-OFF prime), W3 (torque-ON reassertion) and W6 (prerequisite).
/// The prime and the reassertion stay two distinct stages, exactly as the two
/// writes they describe stay distinct (contract F1.3).
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LegacyEntryStage {
    Prime,
    Reassert,
    Prerequisite,
}

/// Contract section 8.0: engine-owned context, never a transferable motion
/// token.
///
/// No capability object, nonce, epoch or session permission value exists here,
/// and nothing may be authorized now and emitted later: an `EngineContext`
/// records *where the engine is*, not *what it may do later* (CTX-1, N-2).
#[allow(dead_code)]
mod lf_engine_context {
    use super::{ArmableLfSessionSpec, EntryStep, GrammarNode, JointKind, LfSessionMode};

    /// Pre-session entry state. Fields are private to this module, so the only
    /// way to obtain one is the lifecycle below (CTX-2).
    pub(super) struct EntryContext {
        mode: LfSessionMode,
        step: EntryStep,
    }

    /// Session-owned state, after session creation.
    pub(super) struct SessionContext {
        mode: LfSessionMode,
        node: GrammarNode,
        active: Option<u8>,
    }

    pub(super) enum EngineContext {
        Entry(EntryContext),
        Session(SessionContext),
    }

    impl EngineContext {
        /// The ONLY constructor, and it requires the sealed brand (CTX-2,
        /// SEAL-3). A session always begins in the entry phase.
        pub(super) fn enter(spec: &ArmableLfSessionSpec) -> Self {
            Self::Entry(EntryContext {
                mode: spec.mode(),
                step: EntryStep::ExactSetVerify,
            })
        }

        pub(super) fn mode(&self) -> LfSessionMode {
            match self {
                Self::Entry(entry) => entry.mode,
                Self::Session(session) => session.mode,
            }
        }

        pub(super) fn entry_step(&self) -> Option<EntryStep> {
            match self {
                Self::Entry(entry) => Some(entry.step),
                Self::Session(_) => None,
            }
        }

        pub(super) fn grammar_node(&self) -> Option<GrammarNode> {
            match self {
                Self::Entry(_) => None,
                Self::Session(session) => Some(session.node),
            }
        }

        pub(super) fn active_motor(&self) -> Option<u8> {
            match self {
                Self::Entry(_) => None,
                Self::Session(session) => session.active,
            }
        }

        /// CTX-3: `FullNormalization` is reachable only in the full session and
        /// `LegacyProfileEntry` only in the two legacy modes. Entry steps are
        /// unreachable once the session exists.
        pub(super) fn advance_entry(&mut self, step: EntryStep) -> Result<(), String> {
            let Self::Entry(entry) = self else {
                return Err("entry steps are unreachable from a session context".into());
            };
            let allowed = match (step, entry.mode) {
                (EntryStep::ExactSetVerify | EntryStep::GlobalTorqueOff, _) => true,
                (EntryStep::FullNormalization { .. }, LfSessionMode::LfFullLegSession) => true,
                (
                    EntryStep::LegacyProfileEntry { .. },
                    LfSessionMode::LfHipPairLegacy | LfSessionMode::LfSingleContactLegacy { .. },
                ) => true,
                _ => false,
            };
            if !allowed {
                return Err(format!(
                    "entry step {step:?} is not reachable in {:?}",
                    entry.mode
                ));
            }
            entry.step = step;
            Ok(())
        }

        /// Entry -> Session, once. The session is created at a grammar node the
        /// selected mode actually owns.
        pub(super) fn begin_session(self, node: GrammarNode) -> Result<Self, String> {
            let Self::Entry(entry) = self else {
                return Err("the session context already exists".into());
            };
            if !grammar_node_admissible(entry.mode, node) {
                return Err(format!(
                    "grammar node {node:?} is not part of {:?}",
                    entry.mode
                ));
            }
            Ok(Self::Session(SessionContext {
                mode: entry.mode,
                node,
                active: None,
            }))
        }

        pub(super) fn enter_node(&mut self, node: GrammarNode) -> Result<(), String> {
            let Self::Session(session) = self else {
                return Err("grammar nodes are unreachable from an entry context".into());
            };
            if !grammar_node_admissible(session.mode, node) {
                return Err(format!(
                    "grammar node {node:?} is not part of {:?}",
                    session.mode
                ));
            }
            session.node = node;
            session.active = None;
            Ok(())
        }

        pub(super) fn set_active(&mut self, motor: Option<u8>) -> Result<(), String> {
            let Self::Session(session) = self else {
                return Err("no motor is active before the session exists".into());
            };
            session.active = motor;
            Ok(())
        }
    }

    /// Contract section 6.1/6.2: which grammar nodes each mode expands.
    /// `Cleanup` is reachable from every phase (GRM-5) in every mode.
    pub(super) fn grammar_node_admissible(mode: LfSessionMode, node: GrammarNode) -> bool {
        match node {
            GrammarNode::Cleanup | GrammarNode::TorqueOff => true,
            GrammarNode::InitialRecovery
            | GrammarNode::Parking
            | GrammarNode::Diagnostics
            | GrammarNode::RestoreParking
            | GrammarNode::ReturnStaged { .. }
            | GrammarNode::StaticHold { .. } => mode == LfSessionMode::LfFullLegSession,
            GrammarNode::Prerequisites | GrammarNode::RestorePrerequisites => {
                !matches!(mode, LfSessionMode::LfFullLegSession)
            }
            GrammarNode::ContactSearch { joint, side } => match mode {
                LfSessionMode::LfFullLegSession => true,
                LfSessionMode::LfHipPairLegacy => joint == JointKind::Hip,
                LfSessionMode::LfSingleContactLegacy {
                    joint: selected,
                    side: selected_side,
                } => joint == selected.joint_kind() && side == selected_side,
            },
        }
    }
}

// The structural foundation is named from the calibrator module so the sealed
// modules stay the single definition site. Until the engine is wired onto it,
// its only consumer is the MATDOG test module.
#[allow(unused_imports)]
use lf_brand::{
    canonical_lf_v25_raw_spec, validate_lf_v25, ArmableLfSessionSpec, LfIdentityError,
    RawChoreographyHints, RawJointIdentity, RawLegCalibrationSpec, RawOracleIdentity,
    RawParkingReference,
};
#[allow(unused_imports)]
use lf_engine_context::{grammar_node_admissible, EngineContext};
#[allow(unused_imports)]
use lf_v25_oracle::{lf_v25_oracle, HardwareEvidenceBinding, LfV25Oracle};

#[cfg(test)]
#[path = "matdog_test.rs"]
mod tests;
