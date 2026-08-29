/*
 * MATDOG FULL LEG CALIBRATOR V1 — build-stage configuration
 *
 * ONE place decides what this firmware image is allowed to do. There is no
 * second switch to forget, and every value is fail-closed by default: an image
 * built with no flags at all is an H0, bootstrap-denied image.
 *
 * Producing a validation build is a single explicit compiler flag, e.g.
 *
 *     arduino-cli compile --build-property \
 *       "compiler.cpp.extra_flags=-DFLC_AUTHORIZED_STAGE=3 -DFLC_H3_BOOTSTRAP_APPROVED=1"
 *
 * See tools/build_stage.sh, which is the supported way to do this.
 *
 * RAISING THE STAGE IS NOT A BYPASS. The stage gate only widens which named
 * operation may be attempted. Identity, fresh census, measured direction,
 * geometry prerequisites and every hard servo guard are enforced independently
 * and are unaffected by this file. A test asserts that.
 */

#ifndef FLC_STAGE_CONFIG_H
#define FLC_STAGE_CONFIG_H

// --------------------------------------------------------------------------
// Progressive hardware validation stages
// --------------------------------------------------------------------------

#define FLC_STAGE_H0_ESP32_ONLY 0
#define FLC_STAGE_H1_CENSUS_READONLY 1
#define FLC_STAGE_H2_MANUAL_Q0 2
#define FLC_STAGE_H3_JOINT_CHARACTERIZE 3
#define FLC_STAGE_H4_JOINT_CALIBRATE 4
#define FLC_STAGE_H5_LEG 5
#define FLC_STAGE_H6_FOUR_LEGS 6
#define FLC_STAGE_H7_FREEZE 7

// Default: ESP32 only. Nothing on the servo bus may be driven.
#ifndef FLC_AUTHORIZED_STAGE
#define FLC_AUTHORIZED_STAGE FLC_STAGE_H0_ESP32_ONLY
#endif

#if FLC_AUTHORIZED_STAGE < 0 || FLC_AUTHORIZED_STAGE > 7
#error "FLC_AUTHORIZED_STAGE must be 0..7"
#endif

// --------------------------------------------------------------------------
// H3 BOOTSTRAP APPROVAL
//
// H3 exists to MEASURE the contact behaviour of the reassembled joints. But a
// joint cannot be characterized without moving it, and moving it requires some
// initial torque limit, speed and acceleration. That is a genuine bootstrap
// problem, and it is resolved honestly rather than by promoting LF V25 numbers:
//
//   - the bootstrap envelope is a SEPARATE object from the characterized
//     result, and can never be read as one (see FlcParameterOrigin);
//   - it is deliberately more conservative than any historical value;
//   - it is inert unless this flag is set at build time AND the operator
//     confirms it in the live session;
//   - it is reported by @STATUS so a session can never silently be running on
//     bootstrap numbers;
//   - it never becomes canonical and is never written to EEPROM;
//   - the independent hard guards (overcurrent, thermal, voltage, travel and
//     time budgets, wrap domain) apply to it unchanged.
// --------------------------------------------------------------------------

#ifndef FLC_H3_BOOTSTRAP_APPROVED
#define FLC_H3_BOOTSTRAP_APPROVED 0
#endif

// Build identity reported by @STATUS. The supported build script stamps the
// exact Git commit. A hand-written compile remains visibly UNSTAMPED rather
// than claiming provenance it does not have.
#define FLC_STRINGIFY_INNER(value) #value
#define FLC_STRINGIFY(value) FLC_STRINGIFY_INNER(value)
#ifndef FLC_BUILD_GIT_SHA_TOKEN
#define FLC_BUILD_GIT_SHA_TOKEN UNSTAMPED
#endif
#define FLC_BUILD_GIT_SHA FLC_STRINGIFY(FLC_BUILD_GIT_SHA_TOKEN)
#ifndef FLC_BUILD_WORKTREE_DIRTY
#define FLC_BUILD_WORKTREE_DIRTY 1
#endif

// Conservative first-motion envelope for an assembled, unloaded leg joint.
//
// PROVENANCE: NOT a measurement of this build, and NOT inherited from LF V25.
// Chosen strictly below every historical envelope so the first motion on the
// reassembled robot is the gentlest one anybody has run:
//   - torque limit 200 is below the Provisioner V6 bench centering value (300)
//     and well below the LF V25 calibration value (500);
//   - speed 60 is far below the LF V25 approach speed (160);
//   - acceleration 8 matches the slowest historical value rather than exceeding it.
// These exist to make a SUPERVISED first characterization possible. They are
// candidates for refutation by H3, not results.
#ifndef FLC_BOOTSTRAP_TORQUE_LIMIT
#define FLC_BOOTSTRAP_TORQUE_LIMIT 200
#endif
#ifndef FLC_BOOTSTRAP_GOAL_SPEED
#define FLC_BOOTSTRAP_GOAL_SPEED 60
#endif
#ifndef FLC_BOOTSTRAP_ACCELERATION
#define FLC_BOOTSTRAP_ACCELERATION 8
#endif
// Retreat distance used by H3 itself. H3 measures whether it was sufficient and
// reports a characterized value for H4.
#ifndef FLC_BOOTSTRAP_RETREAT_TICKS
#define FLC_BOOTSTRAP_RETREAT_TICKS 96
#endif

// Hard ceilings. A bootstrap or characterized envelope may never exceed these,
// whatever the build flags say. Enforced in flc_calibration_engine.h.
#define FLC_ABSOLUTE_MAX_TORQUE_LIMIT 500
#define FLC_ABSOLUTE_MAX_GOAL_SPEED 400
#define FLC_ABSOLUTE_MAX_ACCELERATION 50
// Sized from the real MATDOG joint excursions rather than picked round: the
// largest single traversal from q0 is upper_leg to its +122.5 deg geometric
// contact, about 1394 ticks. 1800 admits that plus margin while staying below
// the 2048 half-revolution limit, so a legal traversal can never be long enough
// to make the circular direction of travel ambiguous.
#define FLC_ABSOLUTE_MAX_TRAVEL_BUDGET_TICKS 1800
#define FLC_ABSOLUTE_MAX_TIME_BUDGET_MS 30000u

#endif  // FLC_STAGE_CONFIG_H
