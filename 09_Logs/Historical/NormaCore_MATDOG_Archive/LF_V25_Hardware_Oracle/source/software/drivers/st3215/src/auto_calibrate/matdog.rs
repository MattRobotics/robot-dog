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

#[derive(Debug, Clone)]
struct LfSessionStateMachine {
    state: LfSessionState,
    active: Option<LfActiveMotor>,
    held_targets: Vec<StaticTarget>,
    entry_positions: Vec<(u8, u16)>,
    contacts: [Option<DualContactResult>; 3],
    fixed_scale: [Option<ModelZeroEstimate>; 3],
    affine: [Option<AffineJointCalibration>; 3],
    trace: Vec<LfSessionState>,
}

impl LfSessionStateMachine {
    fn new(entry_positions: Vec<(u8, u16)>) -> Result<Self, String> {
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
    session: &LfSessionStateMachine,
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
    calibrator.total_steps = 16;
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
            calibrator.mark_done();
            Ok(())
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
    calibrator.total_steps = 20;
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
            calibrator.mark_done();
            Ok(())
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
    calibrator.total_steps = 58;
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
                    calibrator.mark_done();
                    Ok(())
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
    lf_session: Option<LfSessionStateMachine>,
}

impl MatdogRamOnlyCalibrator {
    fn new(
        profile: ContactProfile,
        target_bus_serial: String,
        comm: Arc<ST3215BusCommunicator>,
        inference_rx: watch::Receiver<InferenceState>,
        stop_requested: Arc<AtomicBool>,
    ) -> Self {
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

        self.next_phase("Prime and return probing joint home")?;
        self.prepare_motor(self.profile.motor_id).await?;
        self.move_motor_to(self.profile.motor_id, HOME_TICK, PROBE_HOME_TOLERANCE_TICKS)
            .await?;

        self.next_phase("Acquire moving-current baseline")?;
        let baseline = self.acquire_moving_current_baseline().await?;

        self.next_phase("Coarse scouting approach — measurement discarded")?;
        let coarse_scout_tick = self.approach(COARSE_STEP_TICKS, baseline).await?;

        self.next_phase("Backoff after coarse scout")?;
        self.backoff_and_verify(coarse_scout_tick, baseline).await?;

        self.next_phase("First fine metrology approach")?;
        let first_tick = self
            .approach_with_scout(FINE_STEP_TICKS, baseline, Some(coarse_scout_tick))
            .await?;

        self.next_phase("Backoff between identical fine approaches")?;
        self.backoff_and_verify(first_tick, baseline).await?;

        self.next_phase("Second fine metrology approach")?;
        let second_tick = self
            .approach_with_scout(FINE_STEP_TICKS, baseline, Some(coarse_scout_tick))
            .await?;

        self.next_phase("Verify fine-to-fine repeatability")?;
        let spread_ticks = repeatability_spread(first_tick, second_tick)?;

        self.next_phase("Return probing joint home")?;
        self.stop_pressure(self.profile.motor_id, second_tick)
            .await?;
        self.move_motor_to(self.profile.motor_id, HOME_TICK, PROBE_HOME_TOLERANCE_TICKS)
            .await?;
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
        self.move_motor_to(self.profile.motor_id, HOME_TICK, STATIC_TOLERANCE_TICKS)
            .await?;
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

        self.next_phase("Prime LF HIP M13 at digital home")?;
        self.prepare_motor(self.profile.motor_id).await?;
        self.move_motor_to(self.profile.motor_id, HOME_TICK, PROBE_HOME_TOLERANCE_TICKS)
            .await?;

        self.next_phase("LF HIP MIN moving-current baseline")?;
        let minimum_baseline = self.acquire_moving_current_baseline().await?;

        self.next_phase("LF HIP MIN coarse approach")?;
        let minimum_first = self.approach(COARSE_STEP_TICKS, minimum_baseline).await?;

        self.next_phase("LF HIP MIN backoff and recovery")?;
        self.backoff_and_verify(minimum_first, minimum_baseline)
            .await?;

        self.next_phase("LF HIP MIN fine repeat approach")?;
        let minimum_second = self.approach(FINE_STEP_TICKS, minimum_baseline).await?;

        self.next_phase("LF HIP MIN repeatability")?;
        let minimum_spread = repeatability_spread(minimum_first, minimum_second)?;

        self.next_phase("Return M13 home between MIN and MAX")?;
        self.stop_pressure(self.profile.motor_id, minimum_second)
            .await?;
        self.move_motor_to(self.profile.motor_id, HOME_TICK, PROBE_HOME_TOLERANCE_TICKS)
            .await?;

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
        let maximum_first = self.approach(COARSE_STEP_TICKS, maximum_baseline).await?;

        self.next_phase("LF HIP MAX backoff and recovery")?;
        self.backoff_and_verify(maximum_first, maximum_baseline)
            .await?;

        self.next_phase("LF HIP MAX fine repeat approach")?;
        let maximum_second = self.approach(FINE_STEP_TICKS, maximum_baseline).await?;

        self.next_phase("LF HIP MAX repeatability")?;
        let maximum_spread = repeatability_spread(maximum_first, maximum_second)?;

        self.next_phase("Return LF HIP M13 home")?;
        self.stop_pressure(self.profile.motor_id, maximum_second)
            .await?;
        self.move_motor_to(self.profile.motor_id, HOME_TICK, PROBE_HOME_TOLERANCE_TICKS)
            .await?;
        self.set_motor_torque_verified(self.profile.motor_id, false)
            .await?;
        self.probe_home_handoff_active = true;

        self.next_phase("Restore M11, M12 and M42 to home")?;
        self.restore_prerequisites().await?;
        self.probe_home_handoff_active = false;

        self.prepare_motor(self.profile.motor_id).await?;
        self.move_motor_to(self.profile.motor_id, HOME_TICK, STATIC_TOLERANCE_TICKS)
            .await?;
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
        self.transition_lf_state(LfSessionState::InitialRecovery)?;
        self.verify_lf_session_others_except(0)?;

        self.transition_lf_state(LfSessionState::Parking)?;
        self.next_phase("Park LH upper M42 once for the complete LF session")?;
        let rear_parking = static_target(Leg::Lh, JointKind::Upper, UPPER_30_DELTA)
            .map_err(|message| -> DynError { message.into() })?;
        self.prepare_motor(rear_parking.motor_id).await?;
        self.move_lf_session_motor_to(
            rear_parking.motor_id,
            rear_parking.target_tick,
            STATIC_TOLERANCE_TICKS,
        )
        .await?;
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
        self.move_motor_to(
            12,
            static_target(Leg::Lf, JointKind::Upper, UPPER_90_DELTA)
                .map_err(|message| -> DynError { message.into() })?
                .target_tick,
            STATIC_TOLERANCE_TICKS,
        )
        .await?;
        self.upsert_held_target(
            static_target(Leg::Lf, JointKind::Upper, UPPER_90_DELTA)
                .map_err(|message| -> DynError { message.into() })?,
        )?;

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
        let folded = static_target(Leg::Lf, JointKind::Lower, LOWER_FOLDED_DELTA)
            .map_err(|message| -> DynError { message.into() })?;
        self.move_motor_to(11, folded.target_tick, STATIC_TOLERANCE_TICKS)
            .await?;
        self.upsert_held_target(folded)?;

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
        self.move_motor_to(13, hip_staged_q0, STATIC_TOLERANCE_TICKS)
            .await?;
        self.upsert_held_target(StaticTarget {
            motor_id: 13,
            target_tick: hip_staged_q0,
        })?;

        self.transition_lf_state(LfSessionState::ReturnLowerHeld)?;
        self.next_phase("Move LF LOWER M11 to URDF-derived staged q=0 and hold")?;
        self.remove_held_target(11);
        let lower_staged_q0 = outcome.joints[2].affine.estimated_zero_tick;
        self.move_motor_to(11, lower_staged_q0, STATIC_TOLERANCE_TICKS)
            .await?;
        self.upsert_held_target(StaticTarget {
            motor_id: 11,
            target_tick: lower_staged_q0,
        })?;

        self.transition_lf_state(LfSessionState::ReturnUpper)?;
        self.next_phase("Move LF UPPER M12 to URDF-derived staged q=0 while M11 holds")?;
        self.remove_held_target(12);
        let upper_staged_q0 = outcome.joints[1].affine.estimated_zero_tick;
        self.move_motor_to(12, upper_staged_q0, STATIC_TOLERANCE_TICKS)
            .await?;
        self.upsert_held_target(StaticTarget {
            motor_id: 12,
            target_tick: upper_staged_q0,
        })?;

        self.transition_lf_state(LfSessionState::RestoreParking)?;
        self.next_phase("Restore LH upper M42 once at end of LF calibration")?;
        self.remove_held_target(42);
        self.move_motor_to(42, HOME_TICK, STATIC_TOLERANCE_TICKS)
            .await?;

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
            LfSessionStateMachine::new(entry_positions)
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

    async fn move_lf_session_motor_to(
        &mut self,
        motor_id: u8,
        target: u16,
        tolerance: u16,
    ) -> Result<MotorObservation, DynError> {
        self.set_lf_active(motor_id, target, LfActiveKind::Commanded)?;
        self.set_motor_goal_verified(motor_id, target).await?;
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
        let minimum = self.measure_lf_contact_side_efficient(None).await?;

        self.stop_pressure(self.profile.motor_id, minimum.second_tick)
            .await?;
        self.transition_lf_state(lf_contact_state(maximum_profile.joint, ContactSide::Max))?;
        self.profile = maximum_profile;
        self.set_lf_active(
            self.profile.motor_id,
            self.latest_observation(self.profile.motor_id)?
                .goal_position,
            LfActiveKind::ContactProbe,
        )?;
        let maximum = self
            .measure_lf_contact_side_efficient(Some(minimum.second_tick))
            .await?;
        Ok(DualContactResult { minimum, maximum })
    }

    async fn measure_lf_contact_side_efficient(
        &mut self,
        previous_contact_tick: Option<u16>,
    ) -> Result<ContactResult, DynError> {
        self.next_phase(&format!(
            "{} moving baseline from current pose",
            self.profile.label
        ))?;
        if let Some(previous) = previous_contact_tick {
            self.stop_pressure(self.profile.motor_id, previous).await?;
        }
        let baseline = self.acquire_moving_current_baseline_forward().await?;

        self.next_phase(&format!("{} coarse scouting pass", self.profile.label))?;
        let coarse_scout_tick = self
            .approach_with_scout(COARSE_STEP_TICKS, baseline, None)
            .await?;

        self.next_phase(&format!("{} coarse backoff", self.profile.label))?;
        self.backoff_and_verify(coarse_scout_tick, baseline).await?;

        self.next_phase(&format!("{} fine metrology pass 1", self.profile.label))?;
        let first_tick = self
            .approach_with_scout(FINE_STEP_TICKS, baseline, Some(coarse_scout_tick))
            .await?;

        self.next_phase(&format!("{} fine metrology backoff", self.profile.label))?;
        self.backoff_and_verify(first_tick, baseline).await?;

        self.next_phase(&format!("{} fine metrology pass 2", self.profile.label))?;
        let second_tick = self
            .approach_with_scout(FINE_STEP_TICKS, baseline, Some(coarse_scout_tick))
            .await?;

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
        let initial = self.latest_observation(motor_id)?;
        self.ensure_observation_safe(motor_id, initial, true, None)?;
        let target = advance_tick(
            initial.position,
            self.profile.probe_sign,
            BASELINE_TRAVEL_TICKS,
        )?;
        if passed_guard(target, self.profile.guard_tick, self.profile.probe_sign) {
            return Err(format!("{} baseline move would pass guard", self.profile.label).into());
        }
        let mut samples = Vec::new();
        let mut last_stamp = initial.monotonic_stamp_ns;
        let mut previous_position = initial.position;
        self.set_motor_goal_verified(motor_id, target).await?;
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
            self.prepare_startup_home_recovery_motor(motor_id).await?;
            self.move_profile_entry_motor_to_target(
                motor_id,
                HOME_TICK,
                &plan.home_ready_motors,
                &established_prerequisites,
                true,
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
            self.prepare_motor(target.motor_id).await?;
            self.move_profile_entry_motor_to_target(
                target.motor_id,
                target.target_tick,
                &plan.home_ready_motors,
                &established_prerequisites,
                false,
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

    async fn move_profile_entry_motor_to_target(
        &mut self,
        motor_id: u8,
        target: u16,
        home_ready_motors: &BTreeSet<u8>,
        established_prerequisites: &BTreeSet<u8>,
        startup_writer: bool,
    ) -> Result<MotorObservation, DynError> {
        if startup_writer {
            self.set_startup_home_goal_verified(motor_id, target)
                .await?;
        } else {
            self.set_motor_goal_verified(motor_id, target).await?;
        }
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
            self.move_motor_to(target.motor_id, HOME_TICK, STATIC_TOLERANCE_TICKS)
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
            self.prepare_startup_home_recovery_motor(motor_id).await?;
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
        self.set_startup_home_goal_verified(motor_id, HOME_TICK)
            .await?;
        self.set_startup_home_torque_verified(motor_id, true).await
    }

    async fn set_startup_home_goal_verified(
        &mut self,
        motor_id: u8,
        target: u16,
    ) -> Result<(), DynError> {
        if target > protocol::MAX_ANGLE_STEP {
            return Err(format!("unsigned startup GoalPosition out of range: {target}").into());
        }
        self.write_startup_home_ram_verified(
            motor_id,
            RamRegister::GoalPosition,
            target.to_le_bytes().to_vec(),
        )
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
        let initial = self.latest_observation(motor_id)?;
        self.ensure_observation_safe(motor_id, initial, false, None)?;
        if self.lf_session.is_some() {
            self.set_lf_active(motor_id, initial.position, LfActiveKind::Commanded)?;
        }
        self.set_motor_goal_verified(motor_id, initial.position)
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
        self.set_motor_goal_verified(motor_id, self.profile.baseline_target_tick)
            .await?;
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
        self.move_motor_to(motor_id, HOME_TICK, PROBE_HOME_TOLERANCE_TICKS)
            .await?;
        Ok(baseline)
    }

    async fn approach(
        &mut self,
        step_ticks: u16,
        baseline: BaselineStats,
    ) -> Result<u16, DynError> {
        self.approach_with_scout(step_ticks, baseline, None).await
    }

    async fn approach_with_scout(
        &mut self,
        step_ticks: u16,
        baseline: BaselineStats,
        coarse_scout_tick: Option<u16>,
    ) -> Result<u16, DynError> {
        let motor_id = self.profile.motor_id;
        let start = self.latest_observation(motor_id)?;
        self.ensure_observation_safe(motor_id, start, true, None)?;
        let mut detector = HybridContactDetector::new_for_profile_with_scout(
            start.position,
            baseline,
            &self.profile,
            coarse_scout_tick,
        );
        let mut target = start.position;
        let mut last_stamp = start.monotonic_stamp_ns;

        'approach_steps: loop {
            self.check_stop()?;
            let next_target = advance_tick(target, self.profile.probe_sign, step_ticks)?;
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
            self.set_motor_goal_verified(motor_id, next_target).await?;
            target = next_target;
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
                        self.stop_pressure(motor_id, observation.position).await?;
                        return Ok(observation.position);
                    }
                    ContactState::EarlyStall => {
                        self.stop_pressure(motor_id, observation.position).await?;
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
                        self.stop_pressure(motor_id, contact.position).await?;
                        return Ok(contact.position);
                    }
                }
                self.stop_pressure(motor_id, observation.position).await?;
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

    async fn backoff_and_verify(
        &mut self,
        contact_tick: u16,
        baseline: BaselineStats,
    ) -> Result<(), DynError> {
        let target = advance_tick(contact_tick, -self.profile.probe_sign, BACKOFF_TICKS)?;
        if crossed_home(target, self.profile.probe_sign) {
            return Err(format!("{} backoff crosses home: {target}", self.profile.label).into());
        }
        let recovered = self
            .move_motor_to(
                self.profile.motor_id,
                target,
                STATIC_TOLERANCE_TICKS.saturating_add(2),
            )
            .await?;
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

    async fn stop_pressure(&mut self, motor_id: u8, present_position: u16) -> Result<(), DynError> {
        self.set_motor_goal_verified(motor_id, present_position)
            .await
    }

    async fn move_motor_to(
        &mut self,
        motor_id: u8,
        target: u16,
        tolerance: u16,
    ) -> Result<MotorObservation, DynError> {
        let persistent_lf_session = self.lf_session.is_some();
        if persistent_lf_session {
            self.set_lf_active(motor_id, target, LfActiveKind::Commanded)?;
        }
        self.set_motor_goal_verified(motor_id, target).await?;
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
                    return Ok(observation);
                }
                if stable_target.observe_at(observation, target, tolerance, Instant::now()) {
                    return Ok(observation);
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

    async fn set_motor_goal_verified(&mut self, motor_id: u8, target: u16) -> Result<(), DynError> {
        if target > protocol::MAX_ANGLE_STEP {
            return Err(format!("unsigned GoalPosition out of range: {target}").into());
        }
        if let Some(session) = &mut self.lf_session {
            session
                .update_active_target(motor_id, target)
                .map_err(|message| -> DynError { message.into() })?;
        }
        self.write_motor_ram_verified(
            motor_id,
            RamRegister::GoalPosition,
            target.to_le_bytes().to_vec(),
        )
        .await
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
        self.comm.update_calibration_progress(
            &self.target_bus_serial,
            current,
            self.total_steps,
            &format!("{}: {phase}", self.profile.label),
            status,
            error,
        );
    }

    fn mark_done(&self) {
        self.publish_progress(self.total_steps, "completed", CalibrationStatus::Done, None);
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

#[cfg(test)]
#[path = "matdog_test.rs"]
mod tests;
