use super::*;

fn motor_state(
    motor_id: u32,
    register_bytes: Vec<u8>,
) -> crate::st3215_proto::inference_state::MotorState {
    crate::st3215_proto::inference_state::MotorState {
        id: motor_id,
        state: register_bytes.into(),
        ..Default::default()
    }
}

fn inference_state(
    bus_serial: &str,
    motors: Vec<crate::st3215_proto::inference_state::MotorState>,
) -> InferenceState {
    InferenceState {
        buses: vec![crate::st3215_proto::inference_state::BusState {
            bus: Some(crate::st3215_proto::St3215Bus {
                serial_number: bus_serial.to_string(),
                ..Default::default()
            }),
            motors,
            ..Default::default()
        }],
        ..Default::default()
    }
}

fn set_register(bytes: &mut [u8], register: RamRegister, value: &[u8]) {
    let address = register.address() as usize;
    bytes[address..address + value.len()].copy_from_slice(value);
}

fn observation(position: u16, velocity: u16, current: u16, goal: u16) -> MotorObservation {
    MotorObservation {
        monotonic_stamp_ns: u64::from(position) + 1,
        position,
        velocity,
        current,
        temperature: 25,
        temperature_limit: 70,
        goal_position: goal,
        torque_limit: TORQUE_LIMIT,
        torque_enabled: true,
        status: 0,
        has_driver_error: false,
    }
}

#[test]
fn matdog_temperature_contract_requires_exact_70c_limit_and_rejects_over_limit() {
    let mut observed = observation(HOME_TICK, 0, 0, HOME_TICK);
    assert!(validate_matdog_temperature(12, observed).is_ok());

    observed.temperature_limit = 71;
    let error = validate_matdog_temperature(12, observed).unwrap_err();
    assert!(error.contains("configured temperature limit changed"));

    observed.temperature_limit = EXPECTED_TEMPERATURE_LIMIT_C;
    observed.temperature = EXPECTED_TEMPERATURE_LIMIT_C;
    assert!(validate_matdog_temperature(12, observed).is_ok());

    observed.temperature = EXPECTED_TEMPERATURE_LIMIT_C + 1;
    let error = validate_matdog_temperature(12, observed).unwrap_err();
    assert!(error.contains("thermal abort"));
}

#[test]
fn exact_matdog_id_set_is_required() {
    assert!(is_exact_matdog_motor_set(&MATDOG_MOTOR_IDS));
    let mut reversed = MATDOG_MOTOR_IDS;
    reversed.reverse();
    assert!(is_exact_matdog_motor_set(&reversed));
    assert!(!is_exact_matdog_motor_set(&MATDOG_MOTOR_IDS[..11]));
    let mut unexpected = MATDOG_MOTOR_IDS;
    unexpected[11] = 44;
    assert!(!is_exact_matdog_motor_set(&unexpected));
}

#[test]
fn profile_table_covers_exactly_24_unique_contacts() {
    let profiles = all_profiles().unwrap();
    assert_eq!(profiles.len(), 24);

    let tokens: BTreeSet<_> = profiles
        .iter()
        .map(|profile| profile.arm_value.as_str())
        .collect();
    assert_eq!(tokens.len(), 24);

    for leg in [Leg::Lf, Leg::Rf, Leg::Rh, Leg::Lh] {
        for joint in [JointKind::Upper, JointKind::Hip, JointKind::Lower] {
            for side in [ContactSide::Min, ContactSide::Max] {
                assert!(profiles.iter().any(|profile| {
                    profile.leg == leg && profile.joint == joint && profile.side == side
                }));
            }
        }
    }
}

#[test]
fn validated_m12_min_profile_preserves_hardware_pilot_numbers() {
    let profile = profile_for_arm_value("LF_UPPER_M12_MIN").unwrap();
    assert_eq!(profile.motor_id, 12);
    assert_eq!(profile.probe_sign, -1);
    assert_eq!(profile.urdf_limit_tick, 1451);
    assert_eq!(profile.guard_tick, 1387);
    assert_eq!(profile.baseline_target_tick, 1984);
    assert_eq!(profile.allowed_motor_ids, &LF_ALLOWED);

    let parked_lh_upper = static_target(Leg::Lh, JointKind::Upper, UPPER_30_DELTA).unwrap();
    assert!(profile.prerequisites.contains(&parked_lh_upper));
    assert_eq!(parked_lh_upper.motor_id, 42);
    assert_eq!(parked_lh_upper.target_tick, 2389);
}

#[test]
fn directions_transform_q_limits_into_leg_specific_unsigned_ticks() {
    assert_eq!(
        profile_for_arm_value("LF_HIP_M13_MIN")
            .unwrap()
            .urdf_limit_tick,
        2560
    );
    assert_eq!(
        profile_for_arm_value("LF_HIP_M13_MAX")
            .unwrap()
            .urdf_limit_tick,
        1536
    );
    assert_eq!(
        profile_for_arm_value("RF_UPPER_M22_MIN")
            .unwrap()
            .urdf_limit_tick,
        2645
    );
    assert_eq!(
        profile_for_arm_value("RF_UPPER_M22_MAX")
            .unwrap()
            .urdf_limit_tick,
        654
    );
    assert_eq!(
        profile_for_arm_value("RH_LOWER_M31_MIN")
            .unwrap()
            .urdf_limit_tick,
        1001
    );
    assert_eq!(
        profile_for_arm_value("LH_LOWER_M41_MAX")
            .unwrap()
            .urdf_limit_tick,
        1621
    );
}

#[test]
fn every_guard_extends_only_64_ticks_beyond_its_urdf_limit() {
    for profile in all_profiles().unwrap() {
        assert_eq!(
            i32::from(profile.guard_tick) - i32::from(profile.urdf_limit_tick),
            i32::from(profile.probe_sign) * i32::from(GUARD_OVERSHOOT_TICKS)
        );
        assert!(profile.guard_tick <= protocol::MAX_ANGLE_STEP);
        assert_eq!(
            i32::from(profile.baseline_target_tick) - i32::from(HOME_TICK),
            i32::from(profile.probe_sign) * i32::from(BASELINE_TRAVEL_TICKS)
        );
    }
}

#[test]
fn front_profiles_park_only_the_ipsilateral_rear_upper() {
    for profile in all_profiles().unwrap() {
        let has_lh_parking = profile
            .prerequisites
            .iter()
            .any(|target| target.motor_id == 42 && target.target_tick == 2389);
        let has_rh_parking = profile
            .prerequisites
            .iter()
            .any(|target| target.motor_id == 32 && target.target_tick == 1707);
        match profile.leg {
            Leg::Lf => {
                assert!(has_lh_parking);
                assert!(!has_rh_parking);
            }
            Leg::Rf => {
                assert!(has_rh_parking);
                assert!(!has_lh_parking);
            }
            Leg::Rh | Leg::Lh => {
                assert!(!has_lh_parking);
                assert!(!has_rh_parking);
            }
        }
    }
}

#[test]
fn ordered_profile_table_lists_upper_then_lower_then_hip() {
    let profiles = all_profiles().unwrap();
    let lf: Vec<_> = profiles
        .iter()
        .filter(|profile| profile.leg == Leg::Lf)
        .map(|profile| (profile.joint, profile.side))
        .collect();
    assert_eq!(
        lf,
        vec![
            (JointKind::Upper, ContactSide::Min),
            (JointKind::Upper, ContactSide::Max),
            (JointKind::Lower, ContactSide::Min),
            (JointKind::Lower, ContactSide::Max),
            (JointKind::Hip, ContactSide::Min),
            (JointKind::Hip, ContactSide::Max),
        ]
    );
}

#[test]
fn lf_lower_profiles_use_horizontal_upper_and_exact_unsigned_numbers() {
    let minimum = profile_for_arm_value("LF_LOWER_M11_MIN").unwrap();
    assert_eq!(minimum.motor_id, 11);
    assert_eq!(minimum.probe_sign, 1);
    assert_eq!(minimum.urdf_limit_tick, 3095);
    assert_eq!(minimum.guard_tick, 3159);
    assert_eq!(minimum.baseline_target_tick, 2112);
    assert_eq!(minimum.allowed_motor_ids, &LF_ALLOWED);
    assert!(minimum.prerequisites.contains(&StaticTarget {
        motor_id: 42,
        target_tick: 2389
    }));
    assert!(minimum.prerequisites.contains(&StaticTarget {
        motor_id: 13,
        target_tick: 2048
    }));
    assert!(minimum.prerequisites.contains(&StaticTarget {
        motor_id: 12,
        target_tick: 3072
    }));

    let maximum = profile_for_arm_value("LF_LOWER_M11_MAX").unwrap();
    assert_eq!(maximum.probe_sign, -1);
    assert_eq!(maximum.urdf_limit_tick, 1621);
    assert_eq!(maximum.guard_tick, 1557);
    assert_eq!(maximum.baseline_target_tick, 1984);
}

#[test]
fn hip_prerequisites_are_compact_and_side_specific() {
    let cases = [
        ("LF_HIP_M13_MIN", 12, 3072, 11, 3038),
        ("LF_HIP_M13_MAX", 12, 3015, 11, 3038),
        ("RF_HIP_M23_MIN", 22, 1081, 21, 1058),
        ("RF_HIP_M23_MAX", 22, 1024, 21, 1058),
        ("RH_HIP_M33_MIN", 32, 1024, 31, 1058),
        ("RH_HIP_M33_MAX", 32, 1024, 31, 1058),
        ("LH_HIP_M43_MIN", 42, 3072, 41, 3038),
        ("LH_HIP_M43_MAX", 42, 3072, 41, 3038),
    ];
    for (token, upper_id, upper_tick, lower_id, lower_tick) in cases {
        let profile = profile_for_arm_value(token).unwrap();
        assert!(profile.prerequisites.contains(&StaticTarget {
            motor_id: upper_id,
            target_tick: upper_tick,
        }));
        assert!(profile.prerequisites.contains(&StaticTarget {
            motor_id: lower_id,
            target_tick: lower_tick,
        }));
    }
}

#[test]
fn isolated_hip_hardware_profiles_are_blocked_but_lower_is_allowed() {
    let hip = profile_for_arm_value("LF_HIP_M13_MIN").unwrap();
    let error = hardware_profile_allowed(&hip).unwrap_err();
    assert!(error.contains(HIP_HARDWARE_BLOCK_REASON));

    let lower = profile_for_arm_value("LF_LOWER_M11_MIN").unwrap();
    assert!(hardware_profile_allowed(&lower).is_ok());
}

#[test]
fn contact_acceptance_corridors_match_model_inner_boundary_and_guard() {
    let m12_min = profile_for_arm_value("LF_UPPER_M12_MIN").unwrap();
    assert_eq!(contact_acceptance_bounds(&m12_min), (1387, 1515));
    assert!(position_inside_contact_acceptance(&m12_min, 1443));

    let m12_max = profile_for_arm_value("LF_UPPER_M12_MAX").unwrap();
    assert_eq!(contact_acceptance_bounds(&m12_max), (3378, 3506));
    assert!(position_inside_contact_acceptance(&m12_max, 3442));

    let m13_min = profile_for_arm_value("LF_HIP_M13_MIN").unwrap();
    assert_eq!(contact_acceptance_bounds(&m13_min), (2496, 2624));
    assert!(!position_inside_contact_acceptance(&m13_min, 2405));

    let m11_min = profile_for_arm_value("LF_LOWER_M11_MIN").unwrap();
    assert_eq!(contact_acceptance_bounds(&m11_min), (3031, 3159));
}

#[test]
fn armed_motor_allowlists_are_leg_scoped_and_include_front_parking_joint() {
    assert_eq!(
        build_profile(Leg::Lf, JointKind::Upper, ContactSide::Min)
            .unwrap()
            .allowed_motor_ids,
        &LF_ALLOWED
    );
    assert_eq!(
        build_profile(Leg::Rf, JointKind::Upper, ContactSide::Min)
            .unwrap()
            .allowed_motor_ids,
        &RF_ALLOWED
    );
    assert_eq!(
        build_profile(Leg::Rh, JointKind::Upper, ContactSide::Min)
            .unwrap()
            .allowed_motor_ids,
        &RH_ALLOWED
    );
    assert_eq!(
        build_profile(Leg::Lh, JointKind::Upper, ContactSide::Min)
            .unwrap()
            .allowed_motor_ids,
        &LH_ALLOWED
    );
}

#[test]
fn lf_upper_m12_max_profile_matches_reviewed_geometry() {
    let profile = profile_for_arm_value("LF_UPPER_M12_MAX").unwrap();
    assert_eq!(profile.motor_id, 12);
    assert_eq!(profile.probe_sign, 1);
    assert_eq!(profile.urdf_limit_tick, 3442);
    assert_eq!(profile.guard_tick, 3506);
    assert_eq!(profile.baseline_target_tick, 2112);
    assert_eq!(profile.allowed_motor_ids, &LF_ALLOWED);

    assert!(profile
        .prerequisites
        .contains(&static_target(Leg::Lh, JointKind::Upper, UPPER_30_DELTA).unwrap()));
    assert!(profile
        .prerequisites
        .contains(&static_target(Leg::Lf, JointKind::Hip, 0).unwrap()));
    assert!(profile
        .prerequisites
        .contains(&static_target(Leg::Lf, JointKind::Lower, 0).unwrap()));
}

#[test]
fn actively_held_static_role_uses_position_error_not_instantaneous_speed() {
    // These are all motors that can own an ActivelyHeld role in the LF state
    // machine. The other eight canonical motors are covered by the separate
    // NonParticipatingTorqueOff tests below.
    for motor_id in [11_u8, 12, 13, 42] {
        for velocity in [LF_HELD_MAX_SPEED_RAW + 1, 50, u16::MAX] {
            let observed = observation(HOME_TICK + 1, velocity, 0, HOME_TICK);
            let now_ns = observed.monotonic_stamp_ns + 1;
            let result = validate_lf_role_observation(
                motor_id,
                observed,
                LfMotorRole::ActivelyHeld {
                    target_tick: HOME_TICK,
                },
                now_ns,
            );
            assert!(
                result.is_ok(),
                "M{motor_id} velocity={velocity}: {result:?}"
            );
        }
    }
}

#[test]
fn actively_held_static_role_remains_fail_closed_on_real_state_errors() {
    for motor_id in [11_u8, 12, 13, 42] {
        let mut torque_off = observation(HOME_TICK, 0, 0, HOME_TICK);
        torque_off.torque_enabled = false;
        let torque_now_ns = torque_off.monotonic_stamp_ns + 1;
        assert!(validate_lf_role_observation(
            motor_id,
            torque_off,
            LfMotorRole::ActivelyHeld {
                target_tick: HOME_TICK,
            },
            torque_now_ns,
        )
        .unwrap_err()
        .contains("torque unexpectedly OFF"));

        let wrong_goal = observation(HOME_TICK, 0, 0, HOME_TICK + 1);
        let goal_now_ns = wrong_goal.monotonic_stamp_ns + 1;
        assert!(validate_lf_role_observation(
            motor_id,
            wrong_goal,
            LfMotorRole::ActivelyHeld {
                target_tick: HOME_TICK,
            },
            goal_now_ns,
        )
        .unwrap_err()
        .contains("goal changed"));

        let drifted = observation(HOME_TICK + STATIC_TOLERANCE_TICKS + 1, 0, 0, HOME_TICK);
        let drift_now_ns = drifted.monotonic_stamp_ns + 1;
        let error = validate_lf_role_observation(
            motor_id,
            drifted,
            LfMotorRole::ActivelyHeld {
                target_tick: HOME_TICK,
            },
            drift_now_ns,
        )
        .unwrap_err();
        assert!(error.contains("actively-held"), "M{motor_id}: {error}");
        assert!(error.contains("drifted"), "M{motor_id}: {error}");
    }
}

#[test]
fn fine_contact_scout_depth_gate_is_direction_independent_and_bounded() {
    for probe_sign in [-1_i8, 1_i8] {
        let scout = 2000_u16;
        let one_step_before = if probe_sign > 0 {
            scout - FINE_STEP_TICKS
        } else {
            scout + FINE_STEP_TICKS
        };
        let too_early = if probe_sign > 0 {
            scout - FINE_STEP_TICKS - 1
        } else {
            scout + FINE_STEP_TICKS + 1
        };
        let beyond_scout = if probe_sign > 0 { scout + 4 } else { scout - 4 };

        assert!(fine_contact_reproduces_coarse_depth(
            scout, scout, probe_sign
        ));
        assert!(fine_contact_reproduces_coarse_depth(
            one_step_before,
            scout,
            probe_sign
        ));
        assert!(!fine_contact_reproduces_coarse_depth(
            too_early, scout, probe_sign
        ));
        assert!(fine_contact_reproduces_coarse_depth(
            beyond_scout,
            scout,
            probe_sign
        ));
    }

    // Normal V23 fine/coarse offsets remain valid.
    assert!(fine_contact_reproduces_coarse_depth(1438, 1434, -1));
    assert!(fine_contact_reproduces_coarse_depth(3443, 3446, 1));
    assert!(fine_contact_reproduces_coarse_depth(3093, 3097, 1));

    // V23 M11 MAX: 1666 is 14 ticks before the deeper 1652 scout and must be
    // traversed as a friction/chamfer plateau rather than frozen as endpoint.
    assert!(!fine_contact_reproduces_coarse_depth(1666, 1652, -1));
}

#[test]
fn v24_m13_fine_tracking_uses_detector_consistent_global_floor() {
    assert_eq!(FINE_STEP_TICKS.saturating_add(4), 12);
    assert_eq!(probe_tracking_error_limit(FINE_STEP_TICKS), 16);
    assert_eq!(probe_tracking_error_limit(COARSE_STEP_TICKS), 68);
    let error = circular_distance(1674, 1687);
    assert_eq!(error, 13);
    assert!(error <= probe_tracking_error_limit(FINE_STEP_TICKS));
    assert!(17 > probe_tracking_error_limit(FINE_STEP_TICKS));
}

#[test]
fn lf_contact_witness_gate_is_uniform_and_rejects_v24_m12_cable_obstruction() {
    assert_eq!(LF_CONTACT_WITNESS_TOLERANCE_TICKS, 24);
    let contact = |a, b| ContactResult {
        coarse_scout_tick: a,
        first_tick: a,
        second_tick: b,
        spread_ticks: circular_distance(a, b),
        baseline: BaselineStats {
            median_current: 1,
            mad_current: 0,
        },
    };
    let upper = DualContactResult {
        minimum: contact(1438, 1440),
        maximum: contact(3398, 3397),
    };
    assert_eq!(
        lf_contact_witness_deviations(JointKind::Upper, upper),
        (4, 45)
    );
    assert!(!lf_contact_witness_accepted(JointKind::Upper, upper));
    let lower = DualContactResult {
        minimum: contact(3093, 3094),
        maximum: contact(1660, 1657),
    };
    assert_eq!(
        lf_contact_witness_deviations(JointKind::Lower, lower),
        (0, 8)
    );
    assert!(lf_contact_witness_accepted(JointKind::Lower, lower));
    let hip = DualContactResult {
        minimum: contact(2535, 2535),
        maximum: contact(1597, 1597),
    };
    assert_eq!(lf_contact_witness_deviations(JointKind::Hip, hip), (0, 20));
    assert!(lf_contact_witness_accepted(JointKind::Hip, hip));
}

#[test]
fn affine_gate_accepts_real_span_while_fixed_scale_stays_diagnostic() {
    let contact = |a, b| ContactResult {
        coarse_scout_tick: a,
        first_tick: a,
        second_tick: b,
        spread_ticks: circular_distance(a, b),
        baseline: BaselineStats {
            median_current: 1,
            mad_current: 0,
        },
    };
    for (joint, contacts) in [
        (
            JointKind::Lower,
            DualContactResult {
                minimum: contact(3093, 3094),
                maximum: contact(1660, 1657),
            },
        ),
        (
            JointKind::Hip,
            DualContactResult {
                minimum: contact(2535, 2535),
                maximum: contact(1597, 1597),
            },
        ),
    ] {
        let evidence = derive_joint_evidence(*spec_for(Leg::Lf, joint), contacts);
        assert!(!evidence.fixed_scale.accepted);
        assert!(evidence.affine.accepted);
        assert!(evidence.contact_witness_accepted);
        assert!(evidence.accepted);
    }
}

#[test]
fn nonparticipating_torque_off_uses_real_position_drift_not_instantaneous_speed() {
    for motor_id in MATDOG_MOTOR_IDS {
        for velocity in [LF_HELD_MAX_SPEED_RAW + 1, 50, u16::MAX] {
            let mut observed = off_observation(HOME_TICK + 1);
            observed.velocity = velocity;
            let now_ns = observed.monotonic_stamp_ns + 1;
            assert!(validate_lf_role_observation(
                motor_id,
                observed,
                LfMotorRole::NonParticipatingTorqueOff {
                    entry_tick: HOME_TICK,
                },
                now_ns,
            )
            .is_ok());
        }
    }
}

#[test]
fn nonparticipating_torque_off_remains_fail_closed_on_torque_or_real_drift() {
    for motor_id in MATDOG_MOTOR_IDS {
        let mut torque_on = off_observation(HOME_TICK);
        torque_on.torque_enabled = true;
        let torque_now_ns = torque_on.monotonic_stamp_ns + 1;
        let torque_error = validate_lf_role_observation(
            motor_id,
            torque_on,
            LfMotorRole::NonParticipatingTorqueOff {
                entry_tick: HOME_TICK,
            },
            torque_now_ns,
        )
        .unwrap_err();
        assert!(torque_error.contains("unexpectedly torque ON"));

        let drifted_tick = HOME_TICK + NON_PARTICIPATING_MAX_DRIFT_TICKS + 1;
        let mut drifted = off_observation(drifted_tick);
        drifted.velocity = 0;
        let drift_now_ns = drifted.monotonic_stamp_ns + 1;
        let drift_error = validate_lf_role_observation(
            motor_id,
            drifted,
            LfMotorRole::NonParticipatingTorqueOff {
                entry_tick: HOME_TICK,
            },
            drift_now_ns,
        )
        .unwrap_err();
        assert!(drift_error.contains("moved unexpectedly"));
        assert!(drift_error.contains("drift=17"));
    }
}

#[test]
fn startup_home_goal_gate_is_global_and_exact_home_only_for_non_profile_joints() {
    let profile = profile_for_arm_value(LF_FULL_SEQUENCE_ARM_VALUE).unwrap();
    let non_profile_joints = [21_u8, 22, 23, 31, 32, 33, 41, 43];

    // q=0 normalization is available to every canonical joint.
    for motor_id in MATDOG_MOTOR_IDS {
        assert!(armed_goal_target_allowed(&profile, motor_id, HOME_TICK));
    }

    // Initial telemetry may be anywhere in the valid unsigned encoder range,
    // but it never becomes an allowed command target. Non-profile joints may
    // receive exactly HOME and no ±window around HOME.
    for motor_id in non_profile_joints {
        for target in [
            0_u16,
            1,
            1984,
            2006,
            2069,
            2112,
            2136,
            protocol::MAX_ANGLE_STEP,
        ] {
            assert_ne!(target, HOME_TICK);
            assert!(!armed_goal_target_allowed(&profile, motor_id, target));
        }
    }
}

#[test]
fn startup_recovery_ram_gate_allows_only_exact_home_for_non_profile_joints() {
    let profile = profile_for_arm_value(LF_FULL_SEQUENCE_ARM_VALUE).unwrap();
    let non_profile_joints = [21_u8, 22, 23, 31, 32, 33, 41, 43];

    for motor_id in non_profile_joints {
        let allowed = |register: RamRegister, value: &[u8]| {
            ram_write_allowed_for_profile(&profile, motor_id, register.address() as u32, value)
        };

        // Uniform RAM-only preparation and cleanup.
        assert!(allowed(RamRegister::TorqueEnable, &[0]));
        assert!(allowed(RamRegister::TorqueEnable, &[1]));
        assert!(allowed(RamRegister::Acc, &[ACCELERATION]));
        assert!(allowed(RamRegister::GoalSpeed, &GOAL_SPEED.to_le_bytes()));
        assert!(allowed(
            RamRegister::TorqueLimit,
            &TORQUE_LIMIT.to_le_bytes()
        ));

        // The only generic position command is exact digital q=0.
        assert!(allowed(RamRegister::GoalPosition, &HOME_TICK.to_le_bytes()));

        for target in [
            0_u16,
            1,
            1984,
            2006,
            2069,
            2112,
            2136,
            2389,
            protocol::MAX_ANGLE_STEP,
        ] {
            assert_ne!(target, HOME_TICK);
            assert!(!allowed(RamRegister::GoalPosition, &target.to_le_bytes()));
        }

        // Alternate motion envelopes remain blocked.
        assert!(!allowed(RamRegister::Acc, &[ACCELERATION + 1]));
        assert!(!allowed(
            RamRegister::GoalSpeed,
            &(GOAL_SPEED + 1).to_le_bytes()
        ));
        assert!(!allowed(
            RamRegister::TorqueLimit,
            &(TORQUE_LIMIT + 1).to_le_bytes()
        ));
    }
}

#[test]
fn startup_v10_pose_classifies_m42_as_valid_prerequisite_residue() {
    let profile = profile_for_arm_value("LF_UPPER_M12_MAX").unwrap();
    assert_eq!(
        startup_role_for_profile(&profile, 42),
        StartupRole::Prerequisite { target_tick: 2389 }
    );
    assert!(startup_position_allowed(&profile, 42, 2386));
    assert!(!startup_position_allowed(&profile, 42, 2400));

    let mut m42 = observation(2386, 0, 0, 2389);
    m42.torque_enabled = false;
    let home_ready = BTreeSet::new();
    let established = BTreeSet::new();
    assert!(validate_profile_entry_hold(&profile, 42, 22, &home_ready, &established, m42).is_ok());

    m42.torque_enabled = true;
    assert!(validate_profile_entry_hold(&profile, 42, 22, &home_ready, &established, m42).is_err());

    let established = BTreeSet::from([42]);
    assert!(validate_profile_entry_hold(&profile, 42, 22, &home_ready, &established, m42).is_ok());
}

#[test]
fn startup_probe_and_prerequisite_corridors_are_restart_safe() {
    let profile = profile_for_arm_value("LF_UPPER_M12_MAX").unwrap();
    for position in [2040, HOME_TICK, 2112, 3000, 3442, 3506, 3516] {
        assert!(
            startup_position_allowed(&profile, 12, position),
            "M12 {position}"
        );
    }
    assert!(!startup_position_allowed(&profile, 12, 3517));

    for position in [2038, HOME_TICK, 2200, 2386, 2389, 2399] {
        assert!(
            startup_position_allowed(&profile, 42, position),
            "M42 {position}"
        );
    }
    assert!(!startup_position_allowed(&profile, 42, 2400));

    assert!(startup_position_allowed(&profile, 22, 2112));
    assert!(!startup_position_allowed(&profile, 22, 2113));
}

#[test]
fn startup_prerequisite_home_endpoint_accepts_observed_m42_2037_without_weakening_target() {
    let profile = profile_for_arm_value("LF_LOWER_M11_MIN").unwrap();
    let observed = 2037;
    let observed_error = circular_distance(observed, HOME_TICK);
    assert_eq!(observed_error, 11);
    assert_eq!(STATIC_TOLERANCE_TICKS, 10);
    assert_eq!(STARTUP_PREREQUISITE_HOME_SETTLE_TICKS, 16);
    assert!(observed_error > STATIC_TOLERANCE_TICKS);
    assert!(observed_error <= STARTUP_PREREQUISITE_HOME_SETTLE_TICKS);
    assert_eq!(startup_envelope(&profile, 42), (2032, 2399));
    assert!(startup_position_allowed(&profile, 42, observed));
    assert!(!startup_position_allowed(&profile, 42, 2400));
}

#[test]
fn startup_probe_home_endpoint_accepts_observed_m11_2059_without_weakening_guard_side() {
    let profile = profile_for_arm_value("LF_LOWER_M11_MAX").unwrap();
    assert_eq!(profile.probe_sign, -1);
    assert_eq!(startup_probe_bounds(&profile), (1547, 2064));
    assert_eq!(startup_envelope(&profile, 11), (1547, 2064));
    assert!(startup_position_allowed(&profile, 11, 2059));
    assert!(startup_position_allowed(&profile, 11, 2064));
    assert!(!startup_position_allowed(&profile, 11, 2065));
    assert!(startup_position_allowed(&profile, 11, 1547));
    assert!(!startup_position_allowed(&profile, 11, 1546));

    let mut probe = observation(2059, 0, 0, HOME_TICK);
    probe.torque_enabled = false;
    let home_ready = BTreeSet::new();
    let established = BTreeSet::new();
    assert!(
        validate_profile_entry_hold(&profile, 11, 0, &home_ready, &established, probe,).is_ok()
    );
}

#[test]
fn startup_wrong_profile_residue_is_rejected() {
    let profile = profile_for_arm_value("LF_UPPER_M12_MAX").unwrap();
    assert_eq!(
        startup_role_for_profile(&profile, 32),
        StartupRole::HomeOnly
    );
    assert!(!startup_position_allowed(&profile, 32, 1707));

    let rf = profile_for_arm_value("RF_UPPER_M22_MAX").unwrap();
    assert_eq!(
        startup_role_for_profile(&rf, 32),
        StartupRole::Prerequisite { target_tick: 1707 }
    );
    assert!(startup_position_allowed(&rf, 32, 1707));
}

#[test]
fn startup_envelopes_match_exhaustive_oracle_for_all_profiles_and_ticks() {
    for profile in all_profiles().unwrap() {
        for motor_id in MATDOG_MOTOR_IDS {
            let role = startup_role_for_profile(&profile, motor_id);
            let expected_bounds = match role {
                StartupRole::Probe => startup_probe_bounds(&profile),
                StartupRole::Prerequisite { target_tick } if target_tick != HOME_TICK => {
                    startup_prerequisite_bounds(target_tick)
                }
                StartupRole::Prerequisite { .. } | StartupRole::HomeOnly => (
                    HOME_TICK.saturating_sub(STARTUP_HOME_RECOVERY_LIMIT_TICKS),
                    HOME_TICK
                        .saturating_add(STARTUP_HOME_RECOVERY_LIMIT_TICKS)
                        .min(protocol::MAX_ANGLE_STEP),
                ),
            };
            assert_eq!(startup_envelope(&profile, motor_id), expected_bounds);
            for position in 0..=protocol::MAX_ANGLE_STEP {
                let expected = (expected_bounds.0..=expected_bounds.1).contains(&position);
                assert_eq!(
                    startup_position_allowed(&profile, motor_id, position),
                    expected,
                    "profile={} M{} position={}",
                    profile.label,
                    motor_id,
                    position
                );
            }
        }
    }
}

#[test]
fn profile_entry_order_is_restart_safe_and_probe_lifecycle_is_strict() {
    let source = include_str!("matdog.rs");
    let run_start = source.find("    async fn run(&mut self)").expect("run");
    let inspect_start = source[run_start..]
        .find("    async fn inspect_profile_entry(")
        .map(|offset| run_start + offset)
        .expect("inspect function");
    let run = &source[run_start..inspect_start];

    let inspect = run.find("Inspect restart-safe profile entry").unwrap();
    let recover = run
        .find("Recover home-only joints to digital home")
        .unwrap();
    let establish = run
        .find("Establish geometry prerequisites from restart-safe state")
        .unwrap();
    let home_probe = run.find("Prime and return probing joint home").unwrap();
    let baseline = run.find("Acquire moving-current baseline").unwrap();
    assert!(
        inspect < recover && recover < establish && establish < home_probe && home_probe < baseline
    );
    assert!(!run.contains("Verify all joints near digital home"));
    assert!(!run.contains("Apply geometry prerequisites one joint at a time"));

    let baseline_start = source
        .find("    async fn acquire_moving_current_baseline(")
        .expect("baseline function");
    let approach_start = source[baseline_start..]
        .find("    async fn approach_with_scout(")
        .map(|offset| baseline_start + offset)
        .expect("approach function");
    let backoff_start = source[approach_start..]
        .find("    async fn backoff_and_verify(")
        .map(|offset| approach_start + offset)
        .expect("backoff function");
    let baseline_body = &source[baseline_start..approach_start];
    let approach_body = &source[approach_start..backoff_start];
    assert!(baseline_body.contains("self.verify_profile_holds().await?;"));
    assert!(!baseline_body.contains("self.verify_static_holds().await?;"));
    assert!(approach_body.contains("self.verify_profile_holds().await?;"));
    assert!(!approach_body.contains("self.verify_static_holds().await?;"));

    let return_home = run.find("Return probing joint home").unwrap();
    let torque_off = run[return_home..]
        .find("self.set_motor_torque_verified(self.profile.motor_id, false)")
        .map(|offset| return_home + offset)
        .unwrap();
    let restore = run
        .find("Restore prerequisite joints one at a time")
        .unwrap();
    assert!(return_home < torque_off && torque_off < restore);
}

#[test]
fn motion_timeout_covers_observed_m12_max_return() {
    let distance = circular_distance(3327, HOME_TICK);
    assert_eq!(distance, 1279);
    assert_eq!(GOAL_SPEED, 160);
    assert_eq!(MIN_EXPECTED_MOTION_TICKS_PER_SECOND, 80);

    // V38 can command this return inside the original 12-second capacity,
    // while deadline sizing intentionally retains the slower hardware-derived
    // 80 tick/s floor plus the unchanged five-second settling margin.
    assert!(u64::from(distance) <= u64::from(GOAL_SPEED) * MOTION_TIMEOUT.as_secs());

    let timeout = motion_timeout_for_distance(distance);
    assert!(timeout > MOTION_TIMEOUT);
    assert_eq!(timeout, Duration::from_millis(20_988));
}

#[test]
fn motion_timeout_keeps_short_moves_fast_and_scales_for_every_profile() {
    assert_eq!(motion_timeout_for_distance(64), MOTION_TIMEOUT);

    for profile in all_profiles().unwrap() {
        let distance = circular_distance(profile.guard_tick, HOME_TICK);
        let ideal_ms_at_commanded_speed =
            (u64::from(distance) * 1000 + u64::from(GOAL_SPEED) - 1) / u64::from(GOAL_SPEED);
        assert!(
            motion_timeout_for_distance(distance)
                >= Duration::from_millis(ideal_ms_at_commanded_speed)
                    .saturating_add(MOTION_SETTLE_MARGIN)
        );
    }
}

#[test]
fn probe_home_tolerance_covers_observed_m13_settle_without_weakening_static_gate() {
    let observed_error = circular_distance(2059, HOME_TICK);
    assert_eq!(observed_error, 11);
    assert_eq!(STATIC_TOLERANCE_TICKS, 10);
    assert_eq!(PROBE_HOME_TOLERANCE_TICKS, 16);
    assert!(observed_error > STATIC_TOLERANCE_TICKS);
    assert!(observed_error <= PROBE_HOME_TOLERANCE_TICKS);
}

#[test]
fn probe_home_tolerance_is_scoped_to_startup_home_endpoint_and_active_probe_returns() {
    let profile = profile_for_arm_value("LF_LOWER_M11_MIN").unwrap();
    assert!(startup_probe_bounds(&profile).1 >= HOME_TICK + PROBE_HOME_TOLERANCE_TICKS);
    assert_eq!(STATIC_TOLERANCE_TICKS, 10);
    assert_eq!(PROBE_HOME_TOLERANCE_TICKS, 16);

    let passive = lf_passive_corridor(LfSessionState::UpperMin, 11).unwrap();
    assert!(passive.contains(2059));
    assert!(!passive.contains(HOME_TICK + PROBE_PASSIVE_RESTORE_DRIFT_TICKS + 1));
}

#[test]
fn probe_reverse_recovery_accepts_observed_2031_then_requires_final_rehome() {
    let profile = profile_for_arm_value("LF_LOWER_M11_MAX").unwrap();
    let observed_error = circular_distance(2031, HOME_TICK);
    assert_eq!(observed_error, 17);
    assert_eq!(PROBE_PASSIVE_RESTORE_DRIFT_TICKS, 32);
    assert_eq!(home_hold_tolerance(&profile, 11, true), 32);
    assert_eq!(home_hold_tolerance(&profile, 11, false), 10);
    assert_eq!(home_hold_tolerance(&profile, 12, true), 10);
    assert!(observed_error <= PROBE_PASSIVE_RESTORE_DRIFT_TICKS);
    assert!(circular_distance(2016, HOME_TICK) <= PROBE_PASSIVE_RESTORE_DRIFT_TICKS);
    assert!(circular_distance(2015, HOME_TICK) > PROBE_PASSIVE_RESTORE_DRIFT_TICKS);

    let source = include_str!("matdog.rs");
    let run_start = source
        .find("    async fn run(&mut self)")
        .expect("run function");
    let inspect_start = source[run_start..]
        .find("    async fn inspect_profile_entry(")
        .map(|offset| run_start + offset)
        .expect("inspect function");
    let run = &source[run_start..inspect_start];

    let restore = run.find("self.restore_prerequisites().await?;").unwrap();
    let rehome_prepare = run[restore..]
        .find("self.prepare_motor(self.profile.motor_id).await?;")
        .map(|offset| restore + offset)
        .unwrap();
    // The post-restore settle is the W22 node; its tolerance is still the
    // 10-tick static tolerance, distinct from the 16-tick probe return of W21.
    assert_eq!(
        GoalNode::PostRestoreSettle.settle_tolerance(),
        STATIC_TOLERANCE_TICKS
    );
    assert_eq!(
        GoalNode::ProbeReturnHome.settle_tolerance(),
        PROBE_HOME_TOLERANCE_TICKS
    );
    let rehome_move = run[rehome_prepare..]
        .find("GoalNode::PostRestoreSettle")
        .map(|offset| rehome_prepare + offset)
        .unwrap();
    let rehome_torque_off = run[rehome_move..]
        .find("self.set_motor_torque_verified(self.profile.motor_id, false)")
        .map(|offset| rehome_move + offset)
        .unwrap();
    let final_phase = run
        .find(r#"self.next_phase("Final verified global torque OFF")?;"#)
        .unwrap();

    assert!(
        restore < rehome_prepare
            && rehome_prepare < rehome_move
            && rehome_move < rehome_torque_off
            && rehome_torque_off < final_phase
    );
}

#[test]
fn robust_current_baseline_uses_median_and_mad() {
    let baseline = BaselineStats::from_samples(&[10, 11, 10, 12, 10, 90]).unwrap();
    assert_eq!(baseline.median_current, 11);
    assert_eq!(baseline.mad_current, 1);
    assert_eq!(baseline.contact_threshold(), 16);
}

#[test]
fn direction_generic_detector_confirms_stall_in_both_tick_directions() {
    let baseline = BaselineStats {
        median_current: 1,
        mad_current: 0,
    };

    let mut decreasing = HybridContactDetector::new(HOME_TICK, baseline, -1);
    assert_eq!(
        decreasing.observe(observation(1470, 0, 1, 1431), 1431),
        ContactState::FreeMotion
    );
    for _ in 0..TARGET_STARTUP_SAMPLES {
        assert_eq!(
            decreasing.observe(observation(1470, 0, 1, 1431), 1431),
            ContactState::FreeMotion
        );
    }
    assert_eq!(
        decreasing.observe(observation(1470, 0, 1, 1431), 1431),
        ContactState::ContactSuspected
    );
    assert_eq!(
        decreasing.observe(observation(1470, 0, 1, 1431), 1431),
        ContactState::ContactSuspected
    );
    assert_eq!(
        decreasing.observe(observation(1470, 0, 1, 1431), 1431),
        ContactState::ContactConfirmed
    );

    let mut increasing = HybridContactDetector::new(HOME_TICK, baseline, 1);
    assert_eq!(
        increasing.observe(observation(2620, 0, 1, 2660), 2660),
        ContactState::FreeMotion
    );
    for _ in 0..TARGET_STARTUP_SAMPLES {
        assert_eq!(
            increasing.observe(observation(2620, 0, 1, 2660), 2660),
            ContactState::FreeMotion
        );
    }
    assert_eq!(
        increasing.observe(observation(2620, 0, 1, 2660), 2660),
        ContactState::ContactSuspected
    );
    assert_eq!(
        increasing.observe(observation(2620, 0, 1, 2660), 2660),
        ContactState::ContactSuspected
    );
    assert_eq!(
        increasing.observe(observation(2620, 0, 1, 2660), 2660),
        ContactState::ContactConfirmed
    );
}

#[test]
fn lf_hip_max_v36_1981_settle_continues_once_but_real_stall_and_contact_remain_bounded() {
    let profile = profile_for_arm_value("LF_HIP_M13_MAX").unwrap();
    assert_eq!(contact_acceptance_bounds(&profile), (1472, 1600));
    assert_eq!(STATIC_TOLERANCE_TICKS, 10);
    assert_eq!(OUTSIDE_CORRIDOR_SETTLE_TOLERANCE_TICKS, 16);

    let baseline = BaselineStats {
        median_current: 2,
        mad_current: 0,
    };

    // Exact V36 hardware state: outside the MAX corridor, 13 ticks behind the
    // command and below the current threshold. This is bounded target settle,
    // not contact and not yet an early stall.
    let mut detector = HybridContactDetector::new_for_profile(HOME_TICK, baseline, &profile);
    let settle_target = 1968;
    let settle = observation(1981, 0, 2, settle_target);
    assert_eq!(circular_distance(settle.position, settle_target), 13);
    assert!(!position_inside_contact_acceptance(
        &profile,
        settle.position
    ));
    for _ in 0..(usize::from(TARGET_STARTUP_SAMPLES) + 8) {
        assert_eq!(
            detector.observe(settle, settle_target),
            ContactState::FreeMotion
        );
    }

    // The grace is bounded. If M13 does not follow the next 32-tick coarse
    // command, accumulated error becomes 45 ticks and the original early-stall
    // protection fires after the unchanged persistence window.
    let next_target = 1936;
    let stuck = observation(1981, 0, 2, next_target);
    assert_eq!(circular_distance(stuck.position, next_target), 45);
    assert_eq!(
        detector.observe(stuck, next_target),
        ContactState::FreeMotion
    );
    for _ in 0..TARGET_STARTUP_SAMPLES {
        assert_eq!(
            detector.observe(stuck, next_target),
            ContactState::FreeMotion
        );
    }
    assert_eq!(
        detector.observe(stuck, next_target),
        ContactState::ContactSuspected
    );
    assert_eq!(
        detector.observe(stuck, next_target),
        ContactState::ContactSuspected
    );
    assert_eq!(
        detector.observe(stuck, next_target),
        ContactState::EarlyStall
    );

    // Inside the reviewed MAX corridor, the strict 10-tick gate remains in
    // force: a 13-tick persistent kinematic stall is still confirmed contact.
    let mut contact_detector =
        HybridContactDetector::new_for_profile(HOME_TICK, baseline, &profile);
    let contact_target = 1517;
    let contact = observation(1530, 0, 2, contact_target);
    assert_eq!(circular_distance(contact.position, contact_target), 13);
    assert!(position_inside_contact_acceptance(
        &profile,
        contact.position
    ));
    assert_eq!(
        contact_detector.observe(contact, contact_target),
        ContactState::FreeMotion
    );
    for _ in 0..TARGET_STARTUP_SAMPLES {
        assert_eq!(
            contact_detector.observe(contact, contact_target),
            ContactState::FreeMotion
        );
    }
    assert_eq!(
        contact_detector.observe(contact, contact_target),
        ContactState::ContactSuspected
    );
    assert_eq!(
        contact_detector.observe(contact, contact_target),
        ContactState::ContactSuspected
    );
    assert_eq!(
        contact_detector.observe(contact, contact_target),
        ContactState::ContactConfirmed
    );
}

#[test]
fn v38_motion_envelope_is_faster_but_keeps_bounded_contact_guards() {
    assert_eq!(TORQUE_LIMIT, 500);
    assert_eq!(GOAL_SPEED, 160);
    assert_eq!(ACCELERATION, 8);
    assert_eq!(COARSE_STEP_TICKS, 64);
    assert_eq!(FINE_STEP_TICKS, 8);
    assert_eq!(CONTACT_SETTLE_WINDOW, Duration::from_millis(900));
    assert_eq!(HARD_CURRENT_ABORT_RAW, 200);
    for profile in all_profiles().unwrap() {
        assert_eq!(
            i32::from(profile.guard_tick) - i32::from(profile.urdf_limit_tick),
            i32::from(profile.probe_sign) * i32::from(GUARD_OVERSHOOT_TICKS)
        );
    }
}

#[test]
fn lf_full_sequence_is_one_explicit_hardware_arm_with_union_goal_gate() {
    let profile = profile_for_arm_value(LF_FULL_SEQUENCE_ARM_VALUE).unwrap();
    assert!(is_lf_full_sequence(&profile));
    assert!(hardware_profile_allowed(&profile).is_ok());
    assert_eq!(profile.allowed_motor_ids, &LF_ALLOWED);

    for token in [
        "LF_UPPER_M12_MIN",
        "LF_UPPER_M12_MAX",
        "LF_LOWER_M11_MIN",
        "LF_LOWER_M11_MAX",
    ] {
        let stage = profile_for_arm_value(token).unwrap();
        assert!(lf_full_sequence_goal_allowed(
            stage.motor_id,
            stage.guard_tick
        ));
        let beyond_guard = advance_tick(stage.guard_tick, stage.probe_sign, 1).unwrap();
        assert!(!lf_full_sequence_goal_allowed(stage.motor_id, beyond_guard));
        assert!(lf_full_sequence_goal_allowed(stage.motor_id, HOME_TICK));
    }
    for side in [ContactSide::Min, ContactSide::Max] {
        let hip = lf_hip_sequence_profile(side).unwrap();
        assert!(lf_full_sequence_goal_allowed(13, hip.guard_tick));
    }
    assert!(lf_full_sequence_goal_allowed(42, 2389));
    assert!(!lf_full_sequence_goal_allowed(42, 2390));
    // A torque-OFF M42 that settled one tick below q0 must be primeable
    // before the bounded +30-degree parking move.
    assert!(lf_full_sequence_goal_allowed(42, 2047));
    assert!(!lf_full_sequence_goal_allowed(
        42,
        HOME_TICK.saturating_sub(STATIC_TOLERANCE_TICKS + 1)
    ));
    assert!(!lf_full_sequence_goal_allowed(23, HOME_TICK));
    assert!(!lf_full_sequence_goal_allowed(23, HOME_TICK + 65));
    assert!(!lf_full_sequence_goal_allowed(99, HOME_TICK));
}

fn contact_result(first_tick: u16, second_tick: u16) -> ContactResult {
    ContactResult {
        coarse_scout_tick: first_tick,
        first_tick,
        second_tick,
        spread_ticks: circular_distance(first_tick, second_tick),
        baseline: BaselineStats {
            median_current: 1,
            mad_current: 0,
        },
    }
}

fn lf_entry_positions() -> Vec<(u8, u16)> {
    MATDOG_MOTOR_IDS
        .iter()
        .map(|motor_id| {
            (
                *motor_id,
                match *motor_id {
                    23 => 2140,
                    42 => 2385,
                    _ => HOME_TICK,
                },
            )
        })
        .collect()
}

fn off_observation(position: u16) -> MotorObservation {
    let mut value = observation(position, 0, 0, position);
    value.torque_enabled = false;
    value
}

fn on_observation(position: u16, goal: u16) -> MotorObservation {
    observation(position, 0, 1, goal)
}

fn supervised_lf_witness_contacts(joint: JointKind) -> DualContactResult {
    let contact = |first_tick: u16, second_tick: u16| ContactResult {
        coarse_scout_tick: first_tick,
        first_tick,
        second_tick,
        spread_ticks: circular_distance(first_tick, second_tick),
        baseline: BaselineStats {
            median_current: 1,
            mad_current: 0,
        },
    };
    match joint {
        JointKind::Upper => DualContactResult {
            minimum: contact(1442, 1444),
            maximum: contact(3441, 3443),
        },
        JointKind::Lower => DualContactResult {
            minimum: contact(3092, 3094),
            maximum: contact(1665, 1667),
        },
        JointKind::Hip => DualContactResult {
            minimum: contact(2534, 2536),
            maximum: contact(1616, 1618),
        },
    }
}

fn model_consistent_contacts(joint: JointKind) -> DualContactResult {
    let minimum = build_profile(Leg::Lf, joint, ContactSide::Min).unwrap();
    let maximum = build_profile(Leg::Lf, joint, ContactSide::Max).unwrap();
    DualContactResult {
        minimum: contact_result(minimum.urdf_limit_tick, minimum.urdf_limit_tick),
        maximum: contact_result(maximum.urdf_limit_tick, maximum.urdf_limit_tick),
    }
}

fn valid_lf_sessions_by_state() -> Vec<LegSessionStateMachine> {
    let mut sessions = Vec::new();
    let mut session =
        LegSessionStateMachine::new(LfSessionMode::LfFullLegSession, lf_entry_positions()).unwrap();
    sessions.push(session.clone());

    session.transition(LfSessionState::InitialRecovery).unwrap();
    sessions.push(session.clone());
    session.transition(LfSessionState::Parking).unwrap();
    session
        .set_active(42, 2389, LfActiveKind::Commanded)
        .unwrap();
    session
        .hold(StaticTarget {
            motor_id: 42,
            target_tick: 2389,
        })
        .unwrap();
    sessions.push(session.clone());

    session.transition(LfSessionState::UpperMin).unwrap();
    session
        .set_active(12, 1451, LfActiveKind::ContactProbe)
        .unwrap();
    sessions.push(session.clone());
    session.transition(LfSessionState::UpperMax).unwrap();
    session
        .set_active(12, 3442, LfActiveKind::ContactProbe)
        .unwrap();
    sessions.push(session.clone());
    session.transition(LfSessionState::UpperHorizontal).unwrap();
    session
        .set_active(12, 3072, LfActiveKind::Commanded)
        .unwrap();
    session
        .hold(StaticTarget {
            motor_id: 12,
            target_tick: 3072,
        })
        .unwrap();
    sessions.push(session.clone());

    session.transition(LfSessionState::LowerMin).unwrap();
    session
        .set_active(11, 3095, LfActiveKind::ContactProbe)
        .unwrap();
    sessions.push(session.clone());
    session.transition(LfSessionState::LowerMax).unwrap();
    session
        .set_active(11, 1668, LfActiveKind::ContactProbe)
        .unwrap();
    sessions.push(session.clone());
    session.transition(LfSessionState::LowerFolded).unwrap();
    session
        .set_active(11, 3038, LfActiveKind::Commanded)
        .unwrap();
    session
        .hold(StaticTarget {
            motor_id: 11,
            target_tick: 3038,
        })
        .unwrap();
    sessions.push(session.clone());

    session.transition(LfSessionState::HipMin).unwrap();
    session
        .set_active(13, 2560, LfActiveKind::ContactProbe)
        .unwrap();
    sessions.push(session.clone());
    session.transition(LfSessionState::HipMax).unwrap();
    session
        .set_active(13, 1536, LfActiveKind::ContactProbe)
        .unwrap();
    sessions.push(session.clone());
    session.transition(LfSessionState::Diagnostics).unwrap();
    sessions.push(session.clone());

    session.transition(LfSessionState::ReturnHip).unwrap();
    session
        .set_active(13, HOME_TICK, LfActiveKind::Commanded)
        .unwrap();
    session
        .hold(StaticTarget {
            motor_id: 13,
            target_tick: HOME_TICK,
        })
        .unwrap();
    sessions.push(session.clone());
    session.transition(LfSessionState::ReturnLowerHeld).unwrap();
    session.release(11);
    session
        .set_active(11, HOME_TICK, LfActiveKind::Commanded)
        .unwrap();
    session
        .hold(StaticTarget {
            motor_id: 11,
            target_tick: HOME_TICK,
        })
        .unwrap();
    sessions.push(session.clone());
    session.transition(LfSessionState::ReturnUpper).unwrap();
    session.release(12);
    session
        .set_active(12, HOME_TICK, LfActiveKind::Commanded)
        .unwrap();
    session
        .hold(StaticTarget {
            motor_id: 12,
            target_tick: HOME_TICK,
        })
        .unwrap();
    sessions.push(session.clone());
    session.transition(LfSessionState::RestoreParking).unwrap();
    session.release(42);
    session
        .set_active(42, HOME_TICK, LfActiveKind::Commanded)
        .unwrap();
    sessions.push(session);
    sessions
}

fn motor_state_from_observation(
    motor_id: u8,
    observed: MotorObservation,
) -> crate::st3215_proto::inference_state::MotorState {
    let mut bytes = vec![0; RamRegister::PresentCurrent.address() as usize + 2];
    bytes[MAX_TEMPERATURE_LIMIT_ADDRESS] = observed.temperature_limit;
    set_register(
        &mut bytes,
        RamRegister::TorqueEnable,
        &[u8::from(observed.torque_enabled)],
    );
    set_register(
        &mut bytes,
        RamRegister::GoalPosition,
        &observed.goal_position.to_le_bytes(),
    );
    set_register(
        &mut bytes,
        RamRegister::TorqueLimit,
        &observed.torque_limit.to_le_bytes(),
    );
    set_register(
        &mut bytes,
        RamRegister::PresentPosition,
        &observed.position.to_le_bytes(),
    );
    set_register(
        &mut bytes,
        RamRegister::PresentSpeed,
        &observed.velocity.to_le_bytes(),
    );
    set_register(
        &mut bytes,
        RamRegister::PresentTemperature,
        &[observed.temperature],
    );
    set_register(&mut bytes, RamRegister::Status, &[observed.status]);
    set_register(
        &mut bytes,
        RamRegister::PresentCurrent,
        &observed.current.to_le_bytes(),
    );
    let mut motor = motor_state(u32::from(motor_id), bytes);
    motor.monotonic_stamp_ns = observed.monotonic_stamp_ns;
    if observed.has_driver_error {
        motor.error = Some(crate::st3215_proto::St3215Error::default());
    }
    motor
}

fn state_for_lf_session(session: &LegSessionStateMachine, now_ns: u64) -> InferenceState {
    let motors = MATDOG_MOTOR_IDS
        .iter()
        .map(|motor_id| {
            let role = session.role_for(*motor_id).unwrap();
            let mut observed = match role {
                LfMotorRole::ActivelyCommanded { target_tick }
                | LfMotorRole::ActivelyHeld { target_tick }
                | LfMotorRole::ContactProbe { target_tick } => {
                    on_observation(target_tick, target_tick)
                }
                LfMotorRole::PassiveTorqueOffSafe { corridor } => {
                    off_observation(HOME_TICK.clamp(corridor.low, corridor.high))
                }
                LfMotorRole::NonParticipatingTorqueOff { entry_tick } => {
                    off_observation(entry_tick)
                }
            };
            observed.monotonic_stamp_ns = now_ns - 1;
            motor_state_from_observation(*motor_id, observed)
        })
        .collect();
    inference_state("matdog-bus", motors)
}

#[test]
fn model_zero_solver_recovers_exact_urdf_home_without_fitting_encoder_scale() {
    for joint in [JointKind::Upper, JointKind::Lower, JointKind::Hip] {
        let spec = *spec_for(Leg::Lf, joint);
        let minimum = build_profile(Leg::Lf, joint, ContactSide::Min).unwrap();
        let maximum = build_profile(Leg::Lf, joint, ContactSide::Max).unwrap();
        let estimate = derive_model_zero(
            spec,
            DualContactResult {
                minimum: contact_result(minimum.urdf_limit_tick, minimum.urdf_limit_tick),
                maximum: contact_result(maximum.urdf_limit_tick, maximum.urdf_limit_tick),
            },
        );
        assert_eq!(estimate.zero_from_minimum_tick, HOME_TICK);
        assert_eq!(estimate.zero_from_maximum_tick, HOME_TICK);
        assert_eq!(estimate.endpoint_disagreement_ticks, 0);
        assert_eq!(estimate.estimated_zero_tick, HOME_TICK);
        assert!(estimate.accepted);
    }
}

#[test]
fn current_lf_hardware_evidence_proves_upper_zero_but_requires_stronger_lower_and_hip_recheck() {
    let upper = derive_model_zero(
        *spec_for(Leg::Lf, JointKind::Upper),
        DualContactResult {
            minimum: contact_result(1443, 1443),
            maximum: contact_result(3443, 3442),
        },
    );
    assert_eq!(upper.zero_from_minimum_tick, 2040);
    assert_eq!(upper.zero_from_maximum_tick, 2048);
    assert_eq!(upper.endpoint_disagreement_ticks, 8);
    assert_eq!(upper.estimated_zero_tick, 2044);
    assert!(upper.accepted);

    let lower = derive_model_zero(
        *spec_for(Leg::Lf, JointKind::Lower),
        DualContactResult {
            minimum: contact_result(3094, 3092),
            maximum: contact_result(1664, 1666),
        },
    );
    assert_eq!(lower.minimum_contact_tick, 3093);
    assert_eq!(lower.maximum_contact_tick, 1665);
    assert_eq!(lower.zero_from_minimum_tick, 2046);
    assert_eq!(lower.zero_from_maximum_tick, 2092);
    assert_eq!(lower.endpoint_disagreement_ticks, 46);
    assert!(!lower.accepted);

    let hip = derive_model_zero(
        *spec_for(Leg::Lf, JointKind::Hip),
        DualContactResult {
            minimum: contact_result(2530, 2530),
            maximum: contact_result(1595, 1595),
        },
    );
    assert_eq!(hip.zero_from_minimum_tick, 2018);
    assert_eq!(hip.zero_from_maximum_tick, 2107);
    assert_eq!(hip.endpoint_disagreement_ticks, 89);
    assert!(!hip.accepted);
}

#[test]
fn model_zero_gate_rejects_endpoint_disagreement_even_when_midpoint_is_near_2048() {
    let spec = *spec_for(Leg::Lf, JointKind::Hip);
    let estimate = derive_model_zero(
        spec,
        DualContactResult {
            minimum: contact_result(2530, 2530),
            maximum: contact_result(1595, 1595),
        },
    );
    assert!(estimate.shift_from_digital_home_ticks <= MODEL_ZERO_MAX_SHIFT_FROM_DIGITAL_HOME_TICKS);
    assert!(estimate.endpoint_disagreement_ticks > MODEL_ZERO_ENDPOINT_CONSISTENCY_TICKS);
    assert!(!estimate.accepted);
}

#[test]
fn full_lf_port_gate_allows_only_exact_home_normalization_for_non_participants() {
    let profile = profile_for_arm_value(LF_FULL_SEQUENCE_ARM_VALUE).unwrap();
    let non_participating = [21_u8, 22, 23, 31, 32, 33, 41, 43];

    for motor_id in non_participating {
        // Same RAM-only q=0 preparation for every canonical joint.
        assert!(ram_write_allowed_for_profile(
            &profile,
            motor_id,
            RamRegister::TorqueEnable.address() as u32,
            &[0],
        ));
        assert!(ram_write_allowed_for_profile(
            &profile,
            motor_id,
            RamRegister::TorqueEnable.address() as u32,
            &[1],
        ));
        assert!(ram_write_allowed_for_profile(
            &profile,
            motor_id,
            RamRegister::Acc.address() as u32,
            &[ACCELERATION],
        ));
        assert!(ram_write_allowed_for_profile(
            &profile,
            motor_id,
            RamRegister::GoalSpeed.address() as u32,
            &GOAL_SPEED.to_le_bytes(),
        ));
        assert!(ram_write_allowed_for_profile(
            &profile,
            motor_id,
            RamRegister::TorqueLimit.address() as u32,
            &TORQUE_LIMIT.to_le_bytes(),
        ));
        assert!(ram_write_allowed_for_profile(
            &profile,
            motor_id,
            RamRegister::GoalPosition.address() as u32,
            &HOME_TICK.to_le_bytes(),
        ));

        // No arbitrary startup target, no copied initial-position command and
        // no operational LF target is available to non-participating joints.
        for target in [0_u16, 2006, 2136, 2200, 2389, protocol::MAX_ANGLE_STEP] {
            assert!(!ram_write_allowed_for_profile(
                &profile,
                motor_id,
                RamRegister::GoalPosition.address() as u32,
                &target.to_le_bytes(),
            ));
        }

        assert!(!ram_write_allowed_for_profile(
            &profile,
            motor_id,
            RamRegister::Acc.address() as u32,
            &[ACCELERATION + 1],
        ));
        assert!(!ram_write_allowed_for_profile(
            &profile,
            motor_id,
            RamRegister::GoalSpeed.address() as u32,
            &(GOAL_SPEED + 1).to_le_bytes(),
        ));
        assert!(!ram_write_allowed_for_profile(
            &profile,
            motor_id,
            RamRegister::TorqueLimit.address() as u32,
            &(TORQUE_LIMIT + 1).to_le_bytes(),
        ));
    }

    for (register, value) in [
        (RamRegister::TorqueEnable, vec![0]),
        (RamRegister::TorqueEnable, vec![1]),
        (RamRegister::Acc, vec![ACCELERATION]),
        (RamRegister::GoalSpeed, GOAL_SPEED.to_le_bytes().to_vec()),
        (
            RamRegister::TorqueLimit,
            TORQUE_LIMIT.to_le_bytes().to_vec(),
        ),
        (RamRegister::GoalPosition, HOME_TICK.to_le_bytes().to_vec()),
    ] {
        assert!(!ram_write_allowed_for_profile(
            &profile,
            99,
            register.address() as u32,
            &value,
        ));
    }
}

#[test]
fn v38_repeatability_compares_two_identical_fine_approaches_not_the_coarse_scout() {
    let source = include_str!("matdog.rs");
    let start = source
        .find("async fn run(&mut self) -> Result<ContactResult, DynError>")
        .unwrap();
    let end = source[start..]
        .find("async fn run_lf_hip_min_max")
        .map(|offset| start + offset)
        .unwrap();
    let body = &source[start..end];
    // Exactly one coarse scouting pass and exactly two identical fine passes.
    // The step size and the scout policy are now derived by probe_advance_step
    // from the pass and the validated mode, so the pass identifiers are what the
    // single-contact session selects.
    assert_eq!(
        body.matches("approach_with_scout(ProbePass::Coarse, baseline)")
            .count(),
        1
    );
    assert_eq!(
        body.matches("approach_with_scout(ProbePass::Fine, baseline)")
            .count(),
        2
    );
    assert!(body.contains("repeatability_spread(first_tick, second_tick)"));
    assert!(!body.contains("repeatability_spread(coarse_scout_tick"));
    // Two backoffs: after the coarse scout and between the two fine passes.
    assert_eq!(body.matches("self.backoff_and_verify(baseline)").count(), 2);
    // The single-contact mode scouts its fine passes; the hip-pair mode does not.
    assert_eq!(
        derived_scout_policy_for(
            LfSessionMode::LfSingleContactLegacy {
                joint: UpperOrLower::Lower,
                side: ContactSide::Min,
            },
            ProbePass::Fine,
            Some(1234)
        ),
        Some(1234)
    );
    assert_eq!(
        derived_scout_policy_for(LfSessionMode::LfHipPairLegacy, ProbePass::Fine, Some(1234)),
        None
    );
    assert_eq!(
        derived_scout_policy_for(
            LfSessionMode::LfFullLegSession,
            ProbePass::Coarse,
            Some(1234)
        ),
        None
    );
}

#[test]
fn v38_2026_08_01_failure_is_coarse_loading_not_mechanical_change() {
    // Actual failed run: coarse scout 1416, fine pass 1440, historical fine
    // endpoint 1443. The old coarse-vs-fine comparison correctly exceeded
    // the gate, but those samples were generated by different envelopes.
    assert_eq!(circular_distance(1416, 1440), 24);
    assert!(repeatability_spread(1416, 1440).is_err());
    assert_eq!(circular_distance(1440, 1443), 3);
    assert!(repeatability_spread(1440, 1443).is_ok());
}

#[test]
fn v41_adaptive_fine_corridor_accepts_observed_hip_max_plateau_without_moving_guard() {
    let profile = lf_hip_sequence_profile(ContactSide::Max).unwrap();
    assert_eq!(contact_acceptance_bounds(&profile), (1472, 1600));
    assert!(position_inside_adaptive_contact_acceptance(
        &profile, 1600, 1617
    ));
    let (low, high) = adaptive_contact_acceptance_bounds(&profile, Some(1600));
    assert_eq!(low, 1472);
    assert_eq!(high, 1632);
    assert_eq!(profile.guard_tick, 1472);
}

#[test]
fn v41_affine_solver_accepts_complete_observed_lf_contact_set() {
    let contact = |first_tick, second_tick| ContactResult {
        coarse_scout_tick: first_tick,
        first_tick,
        second_tick,
        spread_ticks: circular_distance(first_tick, second_tick),
        baseline: BaselineStats {
            median_current: 1,
            mad_current: 0,
        },
    };
    let upper = derive_affine_joint_calibration(
        *spec_for(Leg::Lf, JointKind::Upper),
        DualContactResult {
            minimum: contact(1440, 1434),
            maximum: contact(3442, 3443),
        },
    );
    let lower = derive_affine_joint_calibration(
        *spec_for(Leg::Lf, JointKind::Lower),
        DualContactResult {
            minimum: contact(3092, 3092),
            maximum: contact(1666, 1667),
        },
    );
    let hip = derive_affine_joint_calibration(
        *spec_for(Leg::Lf, JointKind::Hip),
        DualContactResult {
            minimum: contact(2535, 2535),
            maximum: contact(1617, 1617),
        },
    );
    assert!(upper.accepted);
    assert!(lower.accepted);
    assert!(hip.accepted);
    assert_eq!(upper.estimated_zero_tick, 2038);
    assert_eq!(lower.estimated_zero_tick, 2079);
    assert_eq!(hip.estimated_zero_tick, 2076);
    assert!((850..=1150).contains(&upper.scale_permille));
    assert!((850..=1150).contains(&lower.scale_permille));
    assert!((850..=1150).contains(&hip.scale_permille));
}

#[test]
fn lf_state_machine_runs_the_full_simulated_path_with_runtime_roles() {
    let mut session =
        LegSessionStateMachine::new(LfSessionMode::LfFullLegSession, lf_entry_positions()).unwrap();

    // M23=2140 is healthy, torque-OFF and position-irrelevant at entry.
    let m23_role = session.role_for(23).unwrap();
    validate_lf_role_observation(23, off_observation(2140), m23_role, 10_000).unwrap();

    session.transition(LfSessionState::InitialRecovery).unwrap();
    session.transition(LfSessionState::Parking).unwrap();
    session
        .set_active(42, 2389, LfActiveKind::Commanded)
        .unwrap();
    session
        .hold(StaticTarget {
            motor_id: 42,
            target_tick: 2389,
        })
        .unwrap();

    session.transition(LfSessionState::UpperMin).unwrap();
    session
        .set_active(12, 1451, LfActiveKind::ContactProbe)
        .unwrap();
    let passive_m11 = session.role_for(11).unwrap();
    validate_lf_role_observation(11, off_observation(2059), passive_m11, 10_000).unwrap();
    validate_lf_role_observation(23, off_observation(2140), m23_role, 10_000).unwrap();
    session.transition(LfSessionState::UpperMax).unwrap();
    session
        .set_active(12, 3442, LfActiveKind::ContactProbe)
        .unwrap();

    session.transition(LfSessionState::UpperHorizontal).unwrap();
    session
        .set_active(12, 3072, LfActiveKind::Commanded)
        .unwrap();
    session
        .hold(StaticTarget {
            motor_id: 12,
            target_tick: 3072,
        })
        .unwrap();

    session.transition(LfSessionState::LowerMin).unwrap();
    session
        .set_active(11, 3095, LfActiveKind::ContactProbe)
        .unwrap();
    session.transition(LfSessionState::LowerMax).unwrap();
    session
        .set_active(11, 1668, LfActiveKind::ContactProbe)
        .unwrap();
    session.transition(LfSessionState::LowerFolded).unwrap();
    session
        .set_active(11, 3038, LfActiveKind::Commanded)
        .unwrap();
    session
        .hold(StaticTarget {
            motor_id: 11,
            target_tick: 3038,
        })
        .unwrap();

    session.transition(LfSessionState::HipMin).unwrap();
    session
        .set_active(13, 2560, LfActiveKind::ContactProbe)
        .unwrap();
    session.transition(LfSessionState::HipMax).unwrap();
    session
        .set_active(13, 1536, LfActiveKind::ContactProbe)
        .unwrap();
    session.transition(LfSessionState::Diagnostics).unwrap();

    let mut evidences = Vec::new();
    for joint in [JointKind::Hip, JointKind::Upper, JointKind::Lower] {
        let spec = *spec_for(Leg::Lf, joint);
        let contacts = supervised_lf_witness_contacts(joint);
        session.record_contacts(joint, contacts);
        let evidence = derive_joint_evidence(spec, contacts);
        assert!(evidence.accepted);
        session.record_diagnostics(joint, evidence.fixed_scale, evidence.affine);
        evidences.push(evidence);
    }

    session.transition(LfSessionState::ReturnHip).unwrap();
    session
        .set_active(
            13,
            evidences[0].affine.estimated_zero_tick,
            LfActiveKind::Commanded,
        )
        .unwrap();
    session
        .hold(StaticTarget {
            motor_id: 13,
            target_tick: evidences[0].affine.estimated_zero_tick,
        })
        .unwrap();

    session.transition(LfSessionState::ReturnLowerHeld).unwrap();
    session.release(11);
    session
        .set_active(
            11,
            evidences[2].affine.estimated_zero_tick,
            LfActiveKind::Commanded,
        )
        .unwrap();
    session
        .hold(StaticTarget {
            motor_id: 11,
            target_tick: evidences[2].affine.estimated_zero_tick,
        })
        .unwrap();

    session.transition(LfSessionState::ReturnUpper).unwrap();
    session.release(12);
    session
        .set_active(
            12,
            evidences[1].affine.estimated_zero_tick,
            LfActiveKind::Commanded,
        )
        .unwrap();
    let held_m11 = session.role_for(11).unwrap();
    let held_m11_tick = evidences[2].affine.estimated_zero_tick;
    validate_lf_role_observation(
        11,
        on_observation(held_m11_tick, held_m11_tick),
        held_m11,
        10_000,
    )
    .unwrap();
    let mut drifted = on_observation(held_m11_tick + STATIC_TOLERANCE_TICKS + 1, held_m11_tick);
    assert!(validate_lf_role_observation(11, drifted, held_m11, 10_000).is_err());
    drifted.position = held_m11_tick;
    drifted.torque_enabled = false;
    assert!(validate_lf_role_observation(11, drifted, held_m11, 10_000).is_err());
    session
        .hold(StaticTarget {
            motor_id: 12,
            target_tick: evidences[1].affine.estimated_zero_tick,
        })
        .unwrap();

    session.transition(LfSessionState::RestoreParking).unwrap();
    session.release(42);
    session
        .set_active(42, HOME_TICK, LfActiveKind::Commanded)
        .unwrap();
    assert_eq!(
        session.trace,
        vec![
            LfSessionState::Preflight,
            LfSessionState::InitialRecovery,
            LfSessionState::Parking,
            LfSessionState::UpperMin,
            LfSessionState::UpperMax,
            LfSessionState::UpperHorizontal,
            LfSessionState::LowerMin,
            LfSessionState::LowerMax,
            LfSessionState::LowerFolded,
            LfSessionState::HipMin,
            LfSessionState::HipMax,
            LfSessionState::Diagnostics,
            LfSessionState::ReturnHip,
            LfSessionState::ReturnLowerHeld,
            LfSessionState::ReturnUpper,
            LfSessionState::RestoreParking,
        ]
    );
    session.transition(LfSessionState::Cleanup).unwrap();
    session.complete_verified_cleanup().unwrap();
    assert_eq!(session.state, LfSessionState::TorqueOff);
    assert!(session.active.is_none());
    assert!(session.held_targets.is_empty());
}

#[test]
fn production_snapshot_verifier_replays_m23_2140_and_m11_2059_runtime_paths() {
    let now_ns = 50_000;
    let sessions = valid_lf_sessions_by_state();

    let preflight = sessions
        .iter()
        .find(|session| session.state == LfSessionState::Preflight)
        .unwrap();
    let preflight_state = state_for_lf_session(preflight, now_ns);
    validate_lf_session_snapshot(&preflight_state, "matdog-bus", preflight, 0, now_ns).unwrap();
    assert_eq!(
        observation_from_state(&preflight_state, "matdog-bus", 23)
            .unwrap()
            .position,
        2140
    );

    let upper_min = sessions
        .iter()
        .find(|session| session.state == LfSessionState::UpperMin)
        .unwrap();
    let mut runtime_state = state_for_lf_session(upper_min, now_ns);
    let m11 = runtime_state.buses[0]
        .motors
        .iter_mut()
        .find(|motor| motor.id == 11)
        .unwrap();
    let mut m11_bytes = m11.state.to_vec();
    set_register(
        &mut m11_bytes,
        RamRegister::PresentPosition,
        &2059_u16.to_le_bytes(),
    );
    m11.state = m11_bytes.into();
    validate_lf_session_snapshot(&runtime_state, "matdog-bus", upper_min, 12, now_ns).unwrap();

    let m23 = runtime_state.buses[0]
        .motors
        .iter_mut()
        .find(|motor| motor.id == 23)
        .unwrap();
    let mut m23_bytes = m23.state.to_vec();
    set_register(&mut m23_bytes, RamRegister::TorqueEnable, &[1]);
    m23.state = m23_bytes.into();
    assert!(
        validate_lf_session_snapshot(&runtime_state, "matdog-bus", upper_min, 12, now_ns,).is_err()
    );
}

#[test]
fn production_snapshot_verifier_checks_m11_hold_while_m12_is_active() {
    let now_ns = 60_000;
    let sessions = valid_lf_sessions_by_state();
    let return_upper = sessions
        .iter()
        .find(|session| session.state == LfSessionState::ReturnUpper)
        .unwrap();
    let mut state = state_for_lf_session(return_upper, now_ns);
    validate_lf_session_snapshot(&state, "matdog-bus", return_upper, 12, now_ns).unwrap();

    let held_target = match return_upper.role_for(11).unwrap() {
        LfMotorRole::ActivelyHeld { target_tick } => target_tick,
        role => panic!("unexpected M11 role: {role:?}"),
    };
    let m11 = state.buses[0]
        .motors
        .iter_mut()
        .find(|motor| motor.id == 11)
        .unwrap();
    let mut bytes = m11.state.to_vec();
    set_register(
        &mut bytes,
        RamRegister::PresentPosition,
        &(held_target + STATIC_TOLERANCE_TICKS + 1).to_le_bytes(),
    );
    m11.state = bytes.into();
    assert!(validate_lf_session_snapshot(&state, "matdog-bus", return_upper, 12, now_ns).is_err());
}

#[test]
fn active_readback_cannot_cross_the_strict_mechanical_guard() {
    let profile = build_profile(Leg::Lf, JointKind::Upper, ContactSide::Min).unwrap();
    let inside = on_observation(profile.guard_tick, profile.guard_tick);
    validate_lf_active_readback(12, inside, profile.guard_tick).unwrap();

    let outside_position = on_observation(profile.guard_tick - 1, profile.guard_tick);
    assert!(validate_lf_active_readback(12, outside_position, profile.guard_tick).is_err());
}

#[test]
fn final_hold_promotion_requires_stable_fresh_dwell_not_one_crossing_sample() {
    let target = 2081;
    let start = Instant::now();
    let mut gate = StableTargetGate::default();
    assert!(!gate.observe_at(on_observation(target, target), target, 10, start));
    assert!(!gate.observe_at(
        on_observation(target + 13, target),
        target,
        10,
        start + Duration::from_millis(450),
    ));
    for (index, elapsed_ms) in [500_u64, 650, 800].into_iter().enumerate() {
        assert!(
            !gate.observe_at(
                on_observation(target + (2 - index as u16), target),
                target,
                10,
                start + Duration::from_millis(elapsed_ms),
            ),
            "sample {index} promoted the hold too early"
        );
    }
    assert!(gate.observe_at(
        on_observation(target, target),
        target,
        10,
        start + Duration::from_millis(950),
    ));
}

#[test]
fn initial_recovery_skips_safe_passive_m11_2059_and_moves_only_when_needed() {
    let safe_m11 = off_observation(2059);
    assert_eq!(circular_distance(safe_m11.position, HOME_TICK), 11);
    assert!(!lf_initial_recovery_needed(safe_m11));

    let displaced = off_observation(HOME_TICK + PROBE_HOME_TOLERANCE_TICKS + 1);
    assert!(lf_initial_recovery_needed(displaced));

    let mut moving = off_observation(HOME_TICK);
    moving.velocity = LF_HELD_MAX_SPEED_RAW + 1;
    assert!(lf_initial_recovery_needed(moving));
}

#[test]
fn lf_state_model_rejects_wrong_actuator_hold_and_missing_prerequisite() {
    let mut session =
        LegSessionStateMachine::new(LfSessionMode::LfFullLegSession, lf_entry_positions()).unwrap();
    assert!(session
        .set_active(12, HOME_TICK, LfActiveKind::Commanded)
        .is_err());
    session.transition(LfSessionState::InitialRecovery).unwrap();
    assert!(session
        .set_active(42, HOME_TICK, LfActiveKind::Commanded)
        .is_err());
    session.transition(LfSessionState::Parking).unwrap();
    assert!(session
        .set_active(11, HOME_TICK, LfActiveKind::Commanded)
        .is_err());
    assert!(session
        .hold(StaticTarget {
            motor_id: 11,
            target_tick: HOME_TICK,
        })
        .is_err());
    assert!(session.transition(LfSessionState::UpperMin).is_err());

    session
        .set_active(42, 2389, LfActiveKind::Commanded)
        .unwrap();
    session
        .hold(StaticTarget {
            motor_id: 42,
            target_tick: 2389,
        })
        .unwrap();
    session.transition(LfSessionState::UpperMin).unwrap();
    assert!(session
        .set_active(11, HOME_TICK, LfActiveKind::Commanded)
        .is_err());
    assert!(session
        .set_active(12, 1386, LfActiveKind::ContactProbe)
        .is_err());
}

#[test]
fn historical_contacts_use_affine_and_uniform_witness_freeze_gate() {
    let upper = derive_joint_evidence(
        *spec_for(Leg::Lf, JointKind::Upper),
        supervised_lf_witness_contacts(JointKind::Upper),
    );
    let lower = derive_joint_evidence(
        *spec_for(Leg::Lf, JointKind::Lower),
        supervised_lf_witness_contacts(JointKind::Lower),
    );
    let hip = derive_joint_evidence(
        *spec_for(Leg::Lf, JointKind::Hip),
        supervised_lf_witness_contacts(JointKind::Hip),
    );
    for evidence in [upper, lower, hip] {
        assert!(evidence.affine.accepted);
        assert!(evidence.contact_witness_accepted);
        assert!(evidence.accepted);
    }
    assert!(!hip.fixed_scale.accepted);
}

#[test]
fn degree_diagnostics_preserve_lf_direction_sign_range_and_endpoint_residuals() {
    let close = |actual: f64, expected: f64| {
        assert!(
            (actual - expected).abs() < 1.0e-9,
            "actual={actual}, expected={expected}"
        );
    };
    let tick_degrees = 10.0 * 360.0 / 4096.0;

    for joint in [JointKind::Upper, JointKind::Lower, JointKind::Hip] {
        let spec = *spec_for(Leg::Lf, joint);
        let mut contacts = model_consistent_contacts(joint);
        for contact in [&mut contacts.minimum, &mut contacts.maximum] {
            contact.coarse_scout_tick += 10;
            contact.first_tick += 10;
            contact.second_tick += 10;
        }
        let evidence = derive_joint_evidence(spec, contacts);
        assert_eq!(evidence.fixed_scale.estimated_zero_tick, HOME_TICK + 10);
        assert_eq!(evidence.affine.estimated_zero_tick, HOME_TICK + 10);
        assert_eq!(evidence.affine.scale_permille, 1000);

        let expected_signed_correction = f64::from(spec.direction) * tick_degrees;
        close(
            fixed_q0_correction_degrees(evidence.fixed_scale, spec),
            expected_signed_correction,
        );
        close(
            affine_q0_correction_degrees(evidence.affine, spec),
            expected_signed_correction,
        );
        close(
            measured_span_degrees(evidence.affine),
            urdf_span_degrees(spec),
        );
        close(
            affine_ticks_per_degree(evidence.affine, spec),
            4096.0 / 360.0,
        );
        close(
            affine_endpoint_residual_degrees(evidence.affine, spec, ContactSide::Min),
            0.0,
        );
        close(
            affine_endpoint_residual_degrees(evidence.affine, spec, ContactSide::Max),
            0.0,
        );
    }
}

#[test]
fn coarse_scout_is_persisted_but_cannot_change_fine_metrology_or_q0() {
    let spec = *spec_for(Leg::Lf, JointKind::Upper);
    let mut first = model_consistent_contacts(JointKind::Upper);
    let mut second = first;
    first.minimum.coarse_scout_tick = 1416;
    first.maximum.coarse_scout_tick = 3446;
    second.minimum.coarse_scout_tick = 1440;
    second.maximum.coarse_scout_tick = 3400;
    assert_ne!(
        first.minimum.coarse_scout_tick,
        second.minimum.coarse_scout_tick
    );
    assert_eq!(
        derive_model_zero(spec, first),
        derive_model_zero(spec, second)
    );
    assert_eq!(
        derive_affine_joint_calibration(spec, first),
        derive_affine_joint_calibration(spec, second)
    );
}

#[test]
fn every_lf_state_can_fail_into_the_same_verified_cleanup_terminal() {
    let sessions = valid_lf_sessions_by_state();
    assert_eq!(sessions.len(), 16);
    for session in sessions {
        let failed_state = session.state;
        let mut failed = session;
        failed.transition(LfSessionState::Cleanup).unwrap();
        failed.complete_verified_cleanup().unwrap();
        assert_eq!(
            failed.state,
            LfSessionState::TorqueOff,
            "cleanup failed from {}",
            failed_state.label()
        );
        assert!(failed.active.is_none());
        assert!(failed.held_targets.is_empty());
        assert_eq!(global_torque_off_writes().len(), MATDOG_MOTOR_IDS.len());
    }
}

#[tokio::test]
async fn production_global_cleanup_executes_command_and_fresh_readback_from_every_lf_state() {
    static TEST_DIRECTORY_COUNTER: std::sync::atomic::AtomicU64 =
        std::sync::atomic::AtomicU64::new(1);
    let directory_id = TEST_DIRECTORY_COUNTER.fetch_add(1, Ordering::Relaxed);
    let test_directory = std::env::temp_dir().join(format!(
        "matdog-global-cleanup-{}-{directory_id}",
        std::process::id()
    ));
    std::fs::create_dir_all(&test_directory).unwrap();

    let normfs = Arc::new(
        normfs::NormFS::new(test_directory.clone(), normfs::NormFsSettings::default())
            .await
            .unwrap(),
    );
    let rx_queue = normfs.resolve("test-st3215-rx");
    let tx_queue = normfs.resolve("test-st3215-tx");
    let meta_queue = normfs.resolve("test-st3215-meta");
    let inference_queue = normfs.resolve("test-st3215-inference");
    normfs
        .ensure_queue_exists_for_write(&tx_queue)
        .await
        .unwrap();

    let communicator = Arc::new(ST3215BusCommunicator::new(
        normfs.clone(),
        rx_queue,
        tx_queue.clone(),
        meta_queue,
        inference_queue,
    ));
    let initial_session = valid_lf_sessions_by_state().remove(0);
    let (state_tx, state_rx) =
        tokio::sync::watch::channel(state_for_lf_session(&initial_session, 100_000));
    let (command_tx, mut command_rx) = tokio::sync::mpsc::unbounded_channel();
    let subscription_id = normfs
        .subscribe(
            &tx_queue,
            Box::new(move |entries| {
                for (_, data) in entries {
                    let command = TxEnvelope::decode(data.as_ref()).unwrap();
                    if command_tx.send(command).is_err() {
                        return false;
                    }
                }
                true
            }),
        )
        .unwrap();

    let simulator_state_tx = state_tx.clone();
    let simulator = tokio::spawn(async move {
        while let Some(command) = command_rx.recv().await {
            assert!(command.write.is_none());
            assert!(command.reg_write.is_none());
            assert!(command.action.is_none());
            assert!(command.reset.is_none());
            assert!(command.reset_calibration.is_none());
            assert!(command.freeze_calibration.is_none());
            let sync = command.sync_write.as_ref().unwrap();
            assert_eq!(sync.address, RamRegister::TorqueEnable.address() as u32);
            assert_eq!(sync.motors.len(), MATDOG_MOTOR_IDS.len());
            assert!(sync.motors.iter().all(|write| write.value.as_ref() == [0]));

            let mut state = simulator_state_tx.borrow().clone();
            for motor in &mut state.buses[0].motors {
                let mut bytes = motor.state.to_vec();
                set_register(&mut bytes, RamRegister::TorqueEnable, &[0]);
                motor.state = bytes.into();
                motor.monotonic_stamp_ns += 1;
                motor.last_command = None;
            }
            state.buses[0].motors[0].last_command =
                Some(crate::st3215_proto::InferenceCommandState {
                    command: Some(command),
                    result: CommandResult::CrSuccess as i32,
                });
            simulator_state_tx.send(state).unwrap();
        }
    });

    let profile = lf_full_sequence_profile().unwrap();
    let mut calibrator = MatdogRamOnlyCalibrator::new(
        profile,
        "matdog-bus".to_string(),
        communicator.clone(),
        state_rx,
        Arc::new(AtomicBool::new(false)),
    );

    let sessions = valid_lf_sessions_by_state();
    assert_eq!(sessions.len(), 16);
    for (index, session) in sessions.into_iter().enumerate() {
        let failed_state = session.state;
        let state = state_for_lf_session(&session, 200_000 + index as u64 * 100);
        state_tx.send(state).unwrap();
        calibrator.lf_session = Some(session);
        calibrator.global_torque_off_verified().await.unwrap();
        assert_eq!(
            calibrator.lf_session.as_ref().unwrap().state,
            LfSessionState::TorqueOff,
            "production cleanup failed from {}",
            failed_state.label()
        );
        for motor_id in MATDOG_MOTOR_IDS {
            assert!(
                !calibrator
                    .latest_observation(motor_id)
                    .unwrap()
                    .torque_enabled
            );
        }
    }

    normfs.unsubscribe(&tx_queue, subscription_id);
    simulator.abort();
    drop(calibrator);
    drop(communicator);
    normfs.close().await.unwrap();
    drop(normfs);
    std::fs::remove_dir_all(&test_directory).unwrap();
}

#[test]
fn non_participating_and_held_role_failures_are_detected_from_simulated_telemetry() {
    let mut session =
        LegSessionStateMachine::new(LfSessionMode::LfFullLegSession, lf_entry_positions()).unwrap();
    let non_participating = session.role_for(23).unwrap();
    validate_lf_role_observation(23, off_observation(2140), non_participating, 10_000).unwrap();
    assert!(validate_lf_role_observation(
        23,
        off_observation(2140 + NON_PARTICIPATING_MAX_DRIFT_TICKS + 1),
        non_participating,
        10_000,
    )
    .is_err());
    let mut stale = off_observation(2140);
    stale.monotonic_stamp_ns = 1;
    let stale_now = u64::try_from(MAX_TELEMETRY_AGE.as_nanos()).unwrap() + 2;
    assert!(validate_lf_role_observation(23, stale, non_participating, stale_now).is_err());
    let mut status_error = off_observation(2140);
    status_error.status = 1;
    assert!(validate_lf_role_observation(23, status_error, non_participating, 10_000).is_err());

    session.state = LfSessionState::ReturnLowerHeld;
    session
        .set_active(11, 2081, LfActiveKind::Commanded)
        .unwrap();
    session
        .hold(StaticTarget {
            motor_id: 11,
            target_tick: 2081,
        })
        .unwrap();
    let held = session.role_for(11).unwrap();
    let mut bad_goal = on_observation(2081, 2082);
    assert!(validate_lf_role_observation(11, bad_goal, held, 10_000).is_err());
    bad_goal.goal_position = 2081;
    bad_goal.velocity = LF_HELD_MAX_SPEED_RAW + 1;
    assert!(validate_lf_role_observation(11, bad_goal, held, 10_000).is_ok());
    bad_goal.position = 2081 + STATIC_TOLERANCE_TICKS + 1;
    assert!(validate_lf_role_observation(11, bad_goal, held, 10_000).is_err());
    bad_goal.position = 2081;
    bad_goal.velocity = 0;
    bad_goal.temperature = bad_goal.temperature_limit + 1;
    assert!(validate_lf_role_observation(11, bad_goal, held, 10_000).is_err());
    bad_goal.temperature = 25;
    bad_goal.current = HARD_CURRENT_ABORT_RAW;
    assert!(validate_lf_role_observation(11, bad_goal, held, 10_000).is_err());
}

#[test]
fn current_rise_without_kinematic_stall_is_not_contact() {
    let baseline = BaselineStats {
        median_current: 10,
        mad_current: 1,
    };
    let mut detector = HybridContactDetector::new(HOME_TICK, baseline, -1);
    for position in [2016, 1984, 1952, 1920] {
        assert_ne!(
            detector.observe(observation(position, 25, 40, position - 32), position - 32),
            ContactState::ContactConfirmed
        );
    }
}

#[test]
fn hard_abort_inputs_are_direction_independent() {
    let baseline = BaselineStats {
        median_current: 10,
        mad_current: 1,
    };
    for sign in [-1, 1] {
        let mut detector = HybridContactDetector::new(HOME_TICK, baseline, sign);
        let mut status = observation(1984, 0, 20, 1968);
        status.status = 1;
        assert_eq!(detector.observe(status, 1968), ContactState::HardAbort);

        let mut detector = HybridContactDetector::new(HOME_TICK, baseline, sign);
        assert_eq!(
            detector.observe(observation(1984, 0, HARD_CURRENT_ABORT_RAW, 1968), 1968),
            ContactState::HardAbort
        );
    }
}

#[test]
fn wrap_math_is_local_but_goal_targets_remain_unsigned() {
    assert_eq!(circular_distance(4092, 8), 12);
    assert_eq!(signed_tick_delta(8, 4092), 12);
    assert_eq!(signed_tick_delta(4092, 8), -12);
    assert_eq!(advance_tick(2048, -1, 32).unwrap(), 2016);
    assert_eq!(advance_tick(2048, 1, 32).unwrap(), 2080);
    assert!(advance_tick(0, -1, 1).is_err());
    assert!(advance_tick(protocol::MAX_ANGLE_STEP, 1, 1).is_err());
}

#[test]
fn only_ram_motion_registers_are_allowlisted() {
    assert!(is_allowed_matdog_ram_register(RamRegister::TorqueEnable));
    assert!(is_allowed_matdog_ram_register(RamRegister::Acc));
    assert!(is_allowed_matdog_ram_register(RamRegister::GoalPosition));
    assert!(is_allowed_matdog_ram_register(RamRegister::GoalSpeed));
    assert!(is_allowed_matdog_ram_register(RamRegister::TorqueLimit));
    assert!(!is_allowed_matdog_ram_register(RamRegister::Status));
}

#[test]
fn armed_ram_gate_restricts_registers_values_motors_and_goal_windows() {
    let profile = profile_for_arm_value("LF_UPPER_M12_MIN").unwrap();
    let allowed = |motor_id, register: RamRegister, value: &[u8]| {
        ram_write_allowed_for_profile(&profile, motor_id, register.address() as u32, value)
    };

    let goal = 1431_u16.to_le_bytes();
    assert!(allowed(12, RamRegister::GoalPosition, &goal));
    assert!(!allowed(11, RamRegister::GoalPosition, &goal));
    assert!(!allowed(
        12,
        RamRegister::GoalPosition,
        &1000_u16.to_le_bytes()
    ));
    assert!(allowed(
        42,
        RamRegister::GoalPosition,
        &2389_u16.to_le_bytes()
    ));
    assert!(!allowed(
        42,
        RamRegister::GoalPosition,
        &3000_u16.to_le_bytes()
    ));
    assert!(allowed(
        12,
        RamRegister::TorqueLimit,
        &TORQUE_LIMIT.to_le_bytes()
    ));
    assert!(!allowed(
        12,
        RamRegister::TorqueLimit,
        &(TORQUE_LIMIT + 1).to_le_bytes()
    ));
    assert!(!allowed(12, RamRegister::Status, &[0]));
}

#[test]
fn front_lower_restore_order_keeps_rear_parking_until_active_leg_is_home() {
    let profile = profile_for_arm_value("LF_LOWER_M11_MIN").unwrap();
    let order = prerequisite_restore_order(&profile.prerequisites, profile.motor_id);
    assert_eq!(order, vec![12, 13, 42]);

    let profile = profile_for_arm_value("RF_LOWER_M21_MAX").unwrap();
    let order = prerequisite_restore_order(&profile.prerequisites, profile.motor_id);
    assert_eq!(order, vec![22, 23, 32]);
}

#[test]
fn prerequisites_are_unique_and_never_include_the_probe_motor() {
    for profile in all_profiles().unwrap() {
        let ids: BTreeSet<_> = profile
            .prerequisites
            .iter()
            .map(|target| target.motor_id)
            .collect();
        assert_eq!(ids.len(), profile.prerequisites.len());
        assert!(!ids.contains(&profile.motor_id));
        assert!(ids
            .iter()
            .all(|motor_id| profile.allowed_motor_ids.contains(motor_id)));
    }
}

#[test]
fn unsupported_arming_values_are_rejected() {
    assert!(profile_for_arm_value("LF_UPPER_M12_MIN").is_ok());
    assert!(profile_for_arm_value("LF_UPPER_M12_BOTH").is_err());
    assert!(profile_for_arm_value("ALL_24").is_err());
    assert!(profile_for_arm_value("").is_err());
}

#[test]
fn observation_reads_required_live_registers_and_error_state() {
    let profile = profile_for_arm_value("LF_UPPER_M12_MIN").unwrap();
    let mut bytes = vec![0; RamRegister::PresentCurrent.address() as usize + 2];
    bytes[MAX_TEMPERATURE_LIMIT_ADDRESS] = 70;
    set_register(&mut bytes, RamRegister::TorqueEnable, &[1]);
    set_register(
        &mut bytes,
        RamRegister::GoalPosition,
        &profile.urdf_limit_tick.to_le_bytes(),
    );
    set_register(
        &mut bytes,
        RamRegister::TorqueLimit,
        &TORQUE_LIMIT.to_le_bytes(),
    );
    set_register(
        &mut bytes,
        RamRegister::PresentPosition,
        &1460_u16.to_le_bytes(),
    );
    set_register(
        &mut bytes,
        RamRegister::PresentSpeed,
        &0x8007_u16.to_le_bytes(),
    );
    set_register(&mut bytes, RamRegister::Status, &[0x04]);
    set_register(&mut bytes, RamRegister::PresentTemperature, &[31]);
    set_register(
        &mut bytes,
        RamRegister::PresentCurrent,
        &123_u16.to_le_bytes(),
    );

    let mut motor = motor_state(profile.motor_id as u32, bytes);
    motor.monotonic_stamp_ns = 42;
    motor.error = Some(crate::st3215_proto::St3215Error::default());
    let state = inference_state("matdog-bus", vec![motor]);
    let observed = observation_from_state(&state, "matdog-bus", profile.motor_id).unwrap();

    assert_eq!(observed.monotonic_stamp_ns, 42);
    assert_eq!(observed.position, 1460);
    assert_eq!(speed_magnitude(observed.velocity), 7);
    assert_eq!(observed.current, 123);
    assert_eq!(observed.temperature, 31);
    assert_eq!(observed.temperature_limit, 70);
    assert_eq!(observed.goal_position, profile.urdf_limit_tick);
    assert_eq!(observed.torque_limit, TORQUE_LIMIT);
    assert!(observed.torque_enabled);
    assert_eq!(observed.status, 0x04);
    assert!(observed.has_driver_error);
}

#[test]
fn command_result_and_ram_readback_are_matched_exactly() {
    let command_id = make_command_id(1, 2, 3);
    let mut bytes = vec![0; RamRegister::PresentCurrent.address() as usize + 2];
    set_register(
        &mut bytes,
        RamRegister::GoalPosition,
        &1451_u16.to_le_bytes(),
    );
    let mut motor = motor_state(12, bytes);
    motor.last_command = Some(crate::st3215_proto::InferenceCommandState {
        command: Some(TxEnvelope {
            command_id: command_id.clone(),
            target_bus_serial: "matdog-bus".to_string(),
            ..Default::default()
        }),
        result: CommandResult::CrSuccess as i32,
    });
    let state = inference_state("matdog-bus", vec![motor]);
    let motor = find_motor(&state, "matdog-bus", 12).unwrap();

    assert_eq!(
        command_result_for(&state, "matdog-bus", &command_id),
        Some(CommandResult::CrSuccess as i32)
    );
    assert_eq!(command_result_for(&state, "other-bus", &command_id), None);
    assert_eq!(
        command_result_for(&state, "matdog-bus", &make_command_id(1, 2, 4)),
        None
    );
    assert!(motor_ram_register_matches(
        motor,
        RamRegister::GoalPosition,
        &1451_u16.to_le_bytes()
    ));
    assert!(!motor_ram_register_matches(
        motor,
        RamRegister::GoalPosition,
        &HOME_TICK.to_le_bytes()
    ));
}

#[test]
fn command_ids_are_scoped_and_monotonic() {
    let first = make_command_id(10, 20, 1);
    let second = make_command_id(10, 20, 2);
    let other_run = make_command_id(10, 21, 1);
    assert_eq!(first.len(), 24);
    assert_ne!(first, second);
    assert_ne!(first, other_run);
}

#[test]
fn global_torque_off_cleanup_is_exact() {
    let writes = global_torque_off_writes();
    assert_eq!(writes.len(), MATDOG_MOTOR_IDS.len());
    assert!(is_exact_matdog_motor_set(
        &writes
            .iter()
            .map(|(motor_id, _)| *motor_id)
            .collect::<Vec<_>>()
    ));
    assert!(writes.iter().all(|(_, value)| value.as_slice() == &[0]));
}

#[test]
fn repeatability_uses_circular_distance_and_preserves_unsigned_goals() {
    assert_eq!(repeatability_spread(4092, 8).unwrap(), 12);
    assert_eq!(
        repeatability_spread(1000, 1000 + REPEATABILITY_TOLERANCE_TICKS).unwrap(),
        REPEATABILITY_TOLERANCE_TICKS
    );
    assert!(repeatability_spread(1000, 1001 + REPEATABILITY_TOLERANCE_TICKS).is_err());
}

#[test]
fn canonical_matdog_source_has_no_eeprom_reset_offset_regwrite_action_or_freeze_path() {
    let source = include_str!("matdog.rs");
    for forbidden in [
        "EepromRegister",
        "RamRegister::Lock",
        "ST3215Request::",
        "reg_write: Some",
        "reset: Some",
        "reset_calibration: Some",
        "freeze_calibration: Some",
        "action: Some",
        "Offset.address",
    ] {
        assert!(!source.contains(forbidden), "forbidden token: {forbidden}");
    }
}

#[test]
fn v19_m13_2405_is_early_stall_not_contact() {
    let profile = profile_for_arm_value("LF_HIP_M13_MIN").unwrap();
    let baseline = BaselineStats {
        median_current: 0,
        mad_current: 0,
    };
    let mut detector = HybridContactDetector::new_for_profile(HOME_TICK, baseline, &profile);
    let target = 2464;
    assert_eq!(
        detector.observe(observation(2405, 0, 1, target), target),
        ContactState::FreeMotion
    );
    for _ in 0..TARGET_STARTUP_SAMPLES {
        assert_eq!(
            detector.observe(observation(2405, 0, 1, target), target),
            ContactState::FreeMotion
        );
    }
    assert_eq!(
        detector.observe(observation(2405, 0, 1, target), target),
        ContactState::ContactSuspected
    );
    assert_eq!(
        detector.observe(observation(2405, 0, 1, target), target),
        ContactState::ContactSuspected
    );
    assert_eq!(
        detector.observe(observation(2405, 0, 1, target), target),
        ContactState::EarlyStall
    );
}

#[test]
fn detector_confirms_only_persistent_stall_inside_profile_corridor() {
    let profile = profile_for_arm_value("LF_HIP_M13_MIN").unwrap();
    let baseline = BaselineStats {
        median_current: 0,
        mad_current: 0,
    };
    let mut detector = HybridContactDetector::new_for_profile(HOME_TICK, baseline, &profile);
    let target = 2568;
    assert_eq!(
        detector.observe(observation(2520, 0, 1, target), target),
        ContactState::FreeMotion
    );
    for _ in 0..TARGET_STARTUP_SAMPLES {
        assert_eq!(
            detector.observe(observation(2520, 0, 1, target), target),
            ContactState::FreeMotion
        );
    }
    assert_eq!(
        detector.observe(observation(2520, 0, 1, target), target),
        ContactState::ContactSuspected
    );
    assert_eq!(
        detector.observe(observation(2520, 0, 1, target), target),
        ContactState::ContactSuspected
    );
    assert_eq!(
        detector.observe(observation(2520, 0, 1, target), target),
        ContactState::ContactConfirmed
    );
}

#[test]
fn lf_hip_combined_sequence_is_the_only_unblocked_hip_hardware_path() {
    let combined = profile_for_arm_value(LF_HIP_SEQUENCE_ARM_VALUE).unwrap();
    assert!(is_lf_hip_sequence(&combined));
    assert_eq!(combined.side, ContactSide::Min);
    assert!(hardware_profile_allowed(&combined).is_ok());

    for token in ["LF_HIP_M13_MIN", "LF_HIP_M13_MAX", "RF_HIP_M23_MIN"] {
        let isolated = profile_for_arm_value(token).unwrap();
        assert!(!is_lf_hip_sequence(&isolated));
        assert!(hardware_profile_allowed(&isolated).is_err());
    }
}

#[test]
fn lf_hip_sequence_uses_one_horizontal_parallel_pose_for_both_contacts() {
    let minimum = lf_hip_sequence_profile(ContactSide::Min).unwrap();
    let maximum = lf_hip_sequence_profile(ContactSide::Max).unwrap();

    let expected = vec![
        StaticTarget {
            motor_id: 42,
            target_tick: 2389,
        },
        StaticTarget {
            motor_id: 12,
            target_tick: 3072,
        },
        StaticTarget {
            motor_id: 11,
            target_tick: 3038,
        },
    ];
    assert_eq!(minimum.prerequisites, expected);
    assert_eq!(maximum.prerequisites, expected);

    assert_eq!(minimum.motor_id, 13);
    assert_eq!(minimum.probe_sign, 1);
    assert_eq!(minimum.urdf_limit_tick, 2560);
    assert_eq!(minimum.guard_tick, 2624);
    assert_eq!(contact_acceptance_bounds(&minimum), (2496, 2624));

    assert_eq!(maximum.motor_id, 13);
    assert_eq!(maximum.probe_sign, -1);
    assert_eq!(maximum.urdf_limit_tick, 1536);
    assert_eq!(maximum.guard_tick, 1472);
    assert_eq!(contact_acceptance_bounds(&maximum), (1472, 1600));
}

#[test]
fn lf_hip_sequence_gate_is_bounded_across_min_and_max_and_restart_safe() {
    let profile = profile_for_arm_value(LF_HIP_SEQUENCE_ARM_VALUE).unwrap();
    assert_eq!(startup_envelope(&profile, 13), (1462, 2634));
    for target in [1472, 1536, HOME_TICK, 2560, 2624] {
        assert!(armed_goal_target_allowed(&profile, 13, target));
    }
    assert!(!armed_goal_target_allowed(&profile, 13, 1461));
    assert!(!armed_goal_target_allowed(&profile, 13, 2635));

    assert!(armed_goal_target_allowed(&profile, 42, 2389));
    assert!(armed_goal_target_allowed(&profile, 12, 3072));
    assert!(armed_goal_target_allowed(&profile, 11, 3038));
    assert!(armed_goal_target_allowed(&profile, 12, 3015));
}

#[test]
fn lf_hip_sequence_orders_min_then_max_before_single_home_recovery() {
    let source = include_str!("matdog.rs");
    let start = source
        .find("    async fn run_lf_hip_min_max(")
        .expect("LF HIP sequence method");
    let inspect = source[start..]
        .find("    async fn inspect_profile_entry(")
        .map(|offset| start + offset)
        .expect("next method");
    let body = &source[start..inspect];

    let shared_pose = body.find("Set M12 horizontal and M11 parallel").unwrap();
    let min_coarse = body.find("LF HIP MIN coarse approach").unwrap();
    let between_home = body.find("Return M13 home between MIN and MAX").unwrap();
    let max_coarse = body.find("LF HIP MAX coarse approach").unwrap();
    let final_home = body.find("Return LF HIP M13 home").unwrap();
    let restore = body.find("Restore M11, M12 and M42 to home").unwrap();
    let final_off = body.find("Final verified global torque OFF").unwrap();

    assert!(
        shared_pose < min_coarse
            && min_coarse < between_home
            && between_home < max_coarse
            && max_coarse < final_home
            && final_home < restore
            && restore < final_off
    );
    assert_eq!(
        body.matches("self.establish_prerequisites_restart_safe(&entry_plan)")
            .count(),
        1
    );
    assert_eq!(
        body.matches("self.restore_prerequisites().await?;").count(),
        1
    );
}

#[test]
fn lf_hip_sequence_preserves_ram_only_unsigned_contract() {
    let profile = profile_for_arm_value(LF_HIP_SEQUENCE_ARM_VALUE).unwrap();
    assert!(ram_write_allowed_for_profile(
        &profile,
        13,
        RamRegister::GoalPosition.address() as u32,
        &2624_u16.to_le_bytes(),
    ));
    assert!(ram_write_allowed_for_profile(
        &profile,
        13,
        RamRegister::GoalPosition.address() as u32,
        &1472_u16.to_le_bytes(),
    ));
    assert!(!ram_write_allowed_for_profile(
        &profile,
        13,
        RamRegister::GoalPosition.address() as u32,
        &4095_u16.to_le_bytes(),
    ));
    assert!(!is_allowed_matdog_ram_register(RamRegister::Status));

    let source = include_str!("matdog.rs");
    assert!(!source.contains("EepromRegister"));
    assert!(!source.contains("i16::from_le_bytes"));
}

#[test]
fn full_lf_startup_home_normalization_is_uniform_for_all_canonical_joints() {
    let profile = profile_for_arm_value(LF_FULL_SEQUENCE_ARM_VALUE).unwrap();

    // Entry acceptance is based on valid fresh telemetry, not distance from q=0.
    for position in [0_u16, 1, 2006, 2048, 2136, protocol::MAX_ANGLE_STEP] {
        assert!(startup_home_initial_position_valid(position));
    }
    assert!(!startup_home_initial_position_valid(
        protocol::MAX_ANGLE_STEP + 1
    ));

    for motor_id in MATDOG_MOTOR_IDS {
        // The generic normalization target is exactly q=0 for every joint.
        assert!(armed_goal_target_allowed(&profile, motor_id, HOME_TICK));
        assert!(ram_write_allowed_for_profile(
            &profile,
            motor_id,
            RamRegister::TorqueEnable.address() as u32,
            &[1],
        ));
        assert!(ram_write_allowed_for_profile(
            &profile,
            motor_id,
            RamRegister::Acc.address() as u32,
            &[ACCELERATION],
        ));
        assert!(ram_write_allowed_for_profile(
            &profile,
            motor_id,
            RamRegister::GoalSpeed.address() as u32,
            &GOAL_SPEED.to_le_bytes(),
        ));
        assert!(ram_write_allowed_for_profile(
            &profile,
            motor_id,
            RamRegister::TorqueLimit.address() as u32,
            &TORQUE_LIMIT.to_le_bytes(),
        ));
    }

    // Non-participating joints may be commanded only to exact HOME; their
    // observed initial position is not converted into a command allowance.
    for motor_id in [21_u8, 22, 23, 31, 32, 33, 41, 43] {
        for target in [0_u16, 2006, 2136, 2200, protocol::MAX_ANGLE_STEP] {
            assert!(!armed_goal_target_allowed(&profile, motor_id, target));
        }
    }
}

#[test]
fn full_lf_normalizes_all_twelve_joints_before_creating_strict_session_roles() {
    let source = include_str!("matdog.rs");
    let start = source
        .find("async fn run_lf_state_machine")
        .expect("LF state machine");
    let end = source[start..]
        .find("async fn move_lf_session_motor_to")
        .map(|offset| start + offset)
        .expect("next LF method");
    let body = &source[start..end];

    let normalize = body
        .find("self.normalize_all_matdog_joints_to_q0().await?;")
        .expect("uniform q0 normalization");
    let create_session = body
        .find("self.inspect_lf_native_session_entry()?;")
        .expect("strict LF session creation");
    let parking = body
        .find("Park LH upper M42 once for the complete LF session")
        .expect("M42 operational parking");

    assert!(normalize < create_session);
    assert!(create_session < parking);
    assert!(!body.contains("for motor_id in [13_u8, 11, 12]"));
}

#[test]
fn full_lf_q0_normalization_has_no_distance_admission_window() {
    let source = include_str!("matdog.rs");
    let start = source
        .find("fn verify_uniform_startup_home_snapshot")
        .expect("uniform snapshot helper");
    let end = source[start..]
        .find("async fn startup_home_goal_policy_write")
        .map(|offset| start + offset)
        .expect("end of q0 startup helpers");
    let startup = &source[start..end];

    assert!(!startup.contains("STARTUP_HOME_RECOVERY_LIMIT_TICKS"));
    assert!(!startup.contains("outside the uniform startup-home recovery window"));
    assert!(startup.contains("startup_home_initial_position_valid(initial.position)"));

    // The q=0 prime itself is now the W1/W2 engine operation. It still commands
    // exactly HOME_TICK, never the present position, and it still refuses an
    // invalid unsigned encoder position before writing.
    let prime_start = source
        .find("async fn home_normalization_prime")
        .expect("W1/W2 engine operation");
    let prime_end = source[prime_start..]
        .find("async fn home_reassert_torque_on")
        .map(|offset| prime_start + offset)
        .expect("end of the W1/W2 operation");
    let prime = &source[prime_start..prime_end];
    assert!(prime.contains("self.startup_home_goal_policy_write(motor_id, HOME_TICK)"));
    assert!(!prime.contains("startup_home_goal_policy_write(motor_id, observation.position)"));
    assert_eq!(
        prime.matches("startup_home_goal_policy_write(").count(),
        1,
        "the q=0 prime emits exactly once"
    );
    assert!(prime.contains("startup_home_initial_position_valid(observation.position)"));
    assert!(prime.contains("if observation.torque_enabled"));
}

#[test]
fn accepted_endpoint_q0_is_used_only_for_transactional_staging() {
    let source = include_str!("matdog.rs");
    assert!(source.contains("MATDOG LF URDF FREEZE GATE: PASS"));
    assert!(source.contains("hip_staged_q0"));
    assert!(source.contains("lower_staged_q0"));
    assert!(source.contains("upper_staged_q0"));
    assert!(source.contains("let hip_staged_q0 = outcome.joints[0].affine.estimated_zero_tick;"));
    assert!(source.contains("let lower_staged_q0 = outcome.joints[2].affine.estimated_zero_tick;"));
    assert!(source.contains("let upper_staged_q0 = outcome.joints[1].affine.estimated_zero_tick;"));
    assert!(source.contains("movement_RAM_only=true, EEPROM_written=false"));
    assert!(!source.contains("reg_write: Some"));
    assert!(!source.contains("freeze_calibration: Some"));
}

#[test]
fn full_lf_final_order_stages_m13_m11_m12_then_restores_m42() {
    let source = include_str!("matdog.rs");
    let hip = source.find("let hip_staged_q0").unwrap();
    let lower = source.find("let lower_staged_q0").unwrap();
    let upper = source.find("let upper_staged_q0").unwrap();
    let parking = source
        .find("Restore LH upper M42 once at end of LF calibration")
        .unwrap();
    assert!(hip < lower && lower < upper && upper < parking);
}

#[test]
fn lf_parking_goal_gate_accepts_q0_settle_priming_without_widening_beyond_static_tolerance() {
    let profile = profile_for_arm_value(LF_FULL_SEQUENCE_ARM_VALUE).unwrap();
    let parking = static_target(Leg::Lh, JointKind::Upper, UPPER_30_DELTA).unwrap();
    let lowest_q0_prime = HOME_TICK.saturating_sub(STATIC_TOLERANCE_TICKS);

    for target in lowest_q0_prime..=HOME_TICK {
        assert!(
            armed_goal_target_allowed(&profile, 42, target),
            "M42 q0-settled prime target {target} must be admitted"
        );
        assert!(ram_write_allowed_for_profile(
            &profile,
            42,
            RamRegister::GoalPosition.address() as u32,
            &target.to_le_bytes(),
        ));
    }

    assert!(armed_goal_target_allowed(&profile, 42, parking.target_tick));
    assert!(!armed_goal_target_allowed(
        &profile,
        42,
        lowest_q0_prime.saturating_sub(1)
    ));
}

// ===========================================================================
// P2A-G3-A — structural foundation regression
//
// These tests pin the structural boundaries introduced for the generic
// extraction. They prove admission, rejection and sealing only; the
// twenty-three historical GoalPosition write paths, the three mode traces and
// every startup/detector/threshold behaviour stay covered by the existing
// tests above, unchanged.
// ===========================================================================

fn reviewed_lf_runtime_arms() -> [(&'static str, LfSessionMode); 6] {
    [
        ("LF_LEG_STATE_MACHINE", LfSessionMode::LfFullLegSession),
        ("LF_HIP_M13_MIN_MAX", LfSessionMode::LfHipPairLegacy),
        (
            "LF_UPPER_M12_MIN",
            LfSessionMode::LfSingleContactLegacy {
                joint: UpperOrLower::Upper,
                side: ContactSide::Min,
            },
        ),
        (
            "LF_UPPER_M12_MAX",
            LfSessionMode::LfSingleContactLegacy {
                joint: UpperOrLower::Upper,
                side: ContactSide::Max,
            },
        ),
        (
            "LF_LOWER_M11_MIN",
            LfSessionMode::LfSingleContactLegacy {
                joint: UpperOrLower::Lower,
                side: ContactSide::Min,
            },
        ),
        (
            "LF_LOWER_M11_MAX",
            LfSessionMode::LfSingleContactLegacy {
                joint: UpperOrLower::Lower,
                side: ContactSide::Max,
            },
        ),
    ]
}

#[test]
fn lf_runtime_arm_sentinels_match_the_immutable_profile_constants() {
    assert_eq!(LF_FULL_SEQUENCE_ARM_VALUE, "LF_LEG_STATE_MACHINE");
    assert_eq!(LF_HIP_SEQUENCE_ARM_VALUE, "LF_HIP_M13_MIN_MAX");
    for (arm_value, _) in reviewed_lf_runtime_arms() {
        if arm_value == LF_FULL_SEQUENCE_ARM_VALUE || arm_value == LF_HIP_SEQUENCE_ARM_VALUE {
            continue;
        }
        let profile = profile_for_arm_value(arm_value).unwrap();
        assert_eq!(profile.arm_value, arm_value);
        assert_eq!(profile.leg, Leg::Lf);
    }
}

#[test]
fn lf_runtime_resolver_admits_exactly_the_six_reviewed_arm_values() {
    let mut candidates = all_profiles()
        .unwrap()
        .into_iter()
        .map(|profile| profile.arm_value)
        .collect::<Vec<_>>();
    assert_eq!(candidates.len(), 24);
    candidates.push(LF_HIP_SEQUENCE_ARM_VALUE.to_string());
    candidates.push(LF_FULL_SEQUENCE_ARM_VALUE.to_string());
    for malformed in [
        "",
        " ",
        "lf_leg_state_machine",
        "LF_LEG_STATE_MACHINE ",
        "LF_HIP_M13_MIN_MAXX",
        "LF_HIP_M13_MIN_MA",
        "LF_UPPER_M12_MID",
        "LF_UPPER_M11_MIN",
        "LF_LOWER_M12_MAX",
        "MATDOG",
    ] {
        candidates.push(malformed.to_string());
    }

    let admitted = candidates
        .iter()
        .filter(|value| lf_runtime_session_mode(value).is_ok())
        .cloned()
        .collect::<Vec<_>>();

    assert_eq!(
        admitted,
        vec![
            "LF_UPPER_M12_MIN".to_string(),
            "LF_UPPER_M12_MAX".to_string(),
            "LF_LOWER_M11_MIN".to_string(),
            "LF_LOWER_M11_MAX".to_string(),
            "LF_HIP_M13_MIN_MAX".to_string(),
            "LF_LEG_STATE_MACHINE".to_string(),
        ]
    );
}

#[test]
fn lf_runtime_resolver_maps_each_reviewed_token_to_its_exact_session_mode() {
    for (arm_value, mode) in reviewed_lf_runtime_arms() {
        assert_eq!(
            lf_runtime_session_mode(arm_value).unwrap(),
            mode,
            "{arm_value}"
        );
    }

    // The four single-contact modes stay four distinct runtime modes.
    let single = reviewed_lf_runtime_arms()
        .into_iter()
        .filter(|(_, mode)| matches!(mode, LfSessionMode::LfSingleContactLegacy { .. }))
        .map(|(_, mode)| mode)
        .collect::<Vec<_>>();
    assert_eq!(single.len(), 4);
    for (index, first) in single.iter().enumerate() {
        for second in single.iter().skip(index + 1) {
            assert_ne!(first, second);
        }
    }
}

#[test]
fn lf_isolated_hip_arm_values_stay_recognized_but_never_resolve_to_a_runtime_session() {
    for token in ["LF_HIP_M13_MIN", "LF_HIP_M13_MAX"] {
        // recognized historical profile data (ARM-3r, ARM-5)
        let profile = profile_for_arm_value(token).expect("historical profile data");
        assert_eq!(profile.leg, Leg::Lf);
        assert_eq!(profile.joint, JointKind::Hip);
        assert_eq!(profile.motor_id, 13);

        // still hardware-blocked, exactly as immutable V25 requires
        let blocked = hardware_profile_allowed(&profile).unwrap_err();
        assert!(blocked.contains(HIP_HARDWARE_BLOCK_REASON), "{blocked}");

        // and never a runtime session
        let refused = lf_runtime_session_mode(token).unwrap_err();
        assert!(refused.contains(HIP_HARDWARE_BLOCK_REASON), "{refused}");

        // it is NOT the reviewed pair sequence, and it cannot borrow its brand
        assert_ne!(token, LF_HIP_SEQUENCE_ARM_VALUE);
        let raw = canonical_lf_v25_raw_spec(token);
        for (_, mode) in reviewed_lf_runtime_arms() {
            assert_eq!(
                validate_lf_v25(&raw, mode).err().unwrap(),
                LfIdentityError::SessionMode,
                "{token} must never brand a runtime session"
            );
        }
    }

    // the reviewed shared-geometry pair remains admitted, unchanged
    let pair = profile_for_arm_value(LF_HIP_SEQUENCE_ARM_VALUE).unwrap();
    assert!(hardware_profile_allowed(&pair).is_ok());
    assert_eq!(
        lf_runtime_session_mode(LF_HIP_SEQUENCE_ARM_VALUE).unwrap(),
        LfSessionMode::LfHipPairLegacy
    );
}

#[test]
fn rf_rh_lh_arm_values_never_reach_the_lf_runtime_or_the_sealed_brand() {
    let non_lf = all_profiles()
        .unwrap()
        .into_iter()
        .filter(|profile| profile.leg != Leg::Lf)
        .collect::<Vec<_>>();
    assert_eq!(non_lf.len(), 18);
    for leg in [Leg::Rf, Leg::Rh, Leg::Lh] {
        assert_eq!(
            non_lf.iter().filter(|profile| profile.leg == leg).count(),
            6,
            "{leg:?}"
        );
    }

    for profile in &non_lf {
        // ARM-5: offline enumeration is untouched
        assert!(profile_for_arm_value(&profile.arm_value).is_ok());

        // ARM-1: no runtime session
        let refused = lf_runtime_session_mode(&profile.arm_value).unwrap_err();
        assert!(refused.contains(&profile.arm_value), "{refused}");

        // no branded spec, under any reviewed mode, whether the arm value or
        // the leg identity is the non-LF part
        for (reviewed_arm_value, mode) in reviewed_lf_runtime_arms() {
            // the non-LF token is never the requested arm value of any mode
            let by_arm_value = canonical_lf_v25_raw_spec(&profile.arm_value);
            assert_eq!(
                validate_lf_v25(&by_arm_value, mode).err().unwrap(),
                LfIdentityError::SessionMode
            );

            // even with a reviewed LF token, a non-LF leg fails at LFID-1,
            // before any oracle or probe authority exists
            let mut by_leg = canonical_lf_v25_raw_spec(reviewed_arm_value);
            by_leg.leg = profile.leg;
            assert_eq!(
                validate_lf_v25(&by_leg, mode).err().unwrap(),
                LfIdentityError::Leg
            );

            // an "LF"-labelled hybrid carrying this leg's real joint identity
            // fails on the identity itself, not on the label
            let mut hybrid = canonical_lf_v25_raw_spec(reviewed_arm_value);
            let foreign = spec_for(profile.leg, JointKind::Upper);
            hybrid.joints[1].name = foreign.name.to_string();
            hybrid.joints[1].motor_id = foreign.motor_id;
            assert_eq!(
                validate_lf_v25(&hybrid, mode).err().unwrap(),
                LfIdentityError::JointName
            );
        }
    }
}

#[test]
fn armable_lf_session_spec_has_no_producer_other_than_validate_lf_v25() {
    let source = include_str!("matdog.rs");

    // exactly one declaration, one construction site and one producer signature
    assert_eq!(
        source
            .matches("pub(super) struct ArmableLfSessionSpec {")
            .count(),
        1
    );
    assert_eq!(source.matches("Ok(ArmableLfSessionSpec {").count(), 1);
    assert_eq!(
        source
            .matches("-> Result<ArmableLfSessionSpec, LfIdentityError>")
            .count(),
        1
    );
    assert_eq!(source.matches("pub(super) fn validate_lf_v25(").count(), 1);

    // no trait impl, no conversion and no escape hatch
    assert!(!source.contains("for ArmableLfSessionSpec"));
    assert!(!source.contains("into_armable"));
    assert!(!source.contains("fn inner("));

    // the brand carries no derive at all: not Clone, Copy, Default or Deserialize
    let declaration = source
        .find("pub(super) struct ArmableLfSessionSpec {")
        .unwrap();
    let preamble = &source[declaration.saturating_sub(600)..declaration];
    assert!(!preamble[preamble.rfind("///").unwrap_or(0)..].contains("#[derive"));

    // the inner representation is private to the sealed module
    assert_eq!(source.matches("struct ArmableInner {").count(), 1);
    assert!(!source.contains("pub(super) struct ArmableInner"));
    assert!(!source.contains("pub(crate) struct ArmableInner"));
}

#[test]
fn canonical_lf_v25_identity_is_accepted_and_derives_its_own_choreography() {
    for (arm_value, mode) in reviewed_lf_runtime_arms() {
        let raw = canonical_lf_v25_raw_spec(arm_value);
        let armed = validate_lf_v25(&raw, mode).expect("canonical LF V25 identity");

        assert_eq!(armed.mode(), mode);
        assert_eq!(armed.participants(), &[11u8, 12, 13, 42]);

        let joints = armed.joints();
        assert_eq!(joints[0].name, "lf_hip_joint");
        assert_eq!(joints[0].motor_id, 13);
        assert_eq!(joints[0].direction, -1);
        assert_eq!((joints[0].min_delta, joints[0].max_delta), (-512, 512));
        assert_eq!(joints[1].name, "lf_upper_leg_joint");
        assert_eq!(joints[1].motor_id, 12);
        assert_eq!(joints[1].direction, 1);
        assert_eq!((joints[1].min_delta, joints[1].max_delta), (-597, 1394));
        assert_eq!(joints[2].name, "lf_lower_leg_joint");
        assert_eq!(joints[2].motor_id, 11);
        assert_eq!(joints[2].direction, -1);
        assert_eq!((joints[2].min_delta, joints[2].max_delta), (-1047, 427));

        // the oracle is reachable only through the brand, keyed by JointKind
        let oracle = armed.oracle();
        assert_eq!(oracle.reference_contact_ticks(JointKind::Hip), (2535, 1617));
        assert_eq!(
            oracle.reference_contact_ticks(JointKind::Upper),
            (1443, 3442)
        );
        assert_eq!(
            oracle.reference_contact_ticks(JointKind::Lower),
            (3093, 1666)
        );
        assert_eq!(oracle.tolerance_ticks(), LF_CONTACT_WITNESS_TOLERANCE_TICKS);
        assert_eq!(oracle.tolerance_ticks(), 24);
    }
}

#[test]
fn lf_v25_oracle_preserves_the_immutable_release_witness_and_reconciliation() {
    let oracle = lf_v25_oracle();
    assert_eq!(
        oracle.physical_evidence_sha256(),
        "6eae3201a00b5299550028d5b4e1e73d67520deccf5a85e548f3b07b1777cab4"
    );
    assert_eq!(
        oracle.reconciler_source_sha256(),
        "111da4045c983c4f3a5acd0061dbbe3c7b77fbda7e120693a927f5f0a40f5dfe"
    );
    // the runtime witness delegates to the immutable contact-witness values
    for joint in [JointKind::Hip, JointKind::Upper, JointKind::Lower] {
        assert_eq!(
            oracle.reference_contact_ticks(joint),
            lf_reference_contact_ticks(joint)
        );
    }

    // six recorded joint/side records; hardware evidence never upgrades a domain
    let expected: [(JointKind, ContactSide, f64, bool); 6] = [
        (JointKind::Hip, ContactSide::Min, -42.803, false),
        (JointKind::Hip, ContactSide::Max, 39.375, false),
        (JointKind::Upper, ContactSide::Min, -53.525, true),
        (JointKind::Upper, ContactSide::Max, 122.607, true),
        (JointKind::Lower, ContactSide::Min, -91.846, true),
        (JointKind::Lower, ContactSide::Max, 34.277, false),
    ];
    for (joint, side, hardware_degrees, agrees) in expected {
        let record = oracle.reconciliation(joint, side);
        assert_eq!(record.joint, joint);
        assert_eq!(record.side, side);
        assert_eq!(record.hardware_degrees, hardware_degrees);
        assert_eq!(record.agrees, agrees);
        // delta = geometry - hardware, as recorded
        assert!(
            (record.geometry_degrees - record.hardware_degrees - record.delta_degrees).abs()
                < 0.002,
            "{joint:?} {side:?} reconciliation delta is not self-consistent"
        );
    }
}

#[test]
fn lf_session_participants_and_holds_are_derived_from_the_mode_alone() {
    for (_, mode) in reviewed_lf_runtime_arms() {
        assert_eq!(
            lf_session_participants(mode).unwrap(),
            vec![11u8, 12, 13, 42]
        );
    }

    // the derived holds reuse the immutable historical poses only
    let parking = static_target(Leg::Lh, JointKind::Upper, UPPER_30_DELTA).unwrap();
    assert_eq!(
        lf_mode_hold_targets(LfSessionMode::LfFullLegSession).unwrap(),
        vec![parking]
    );
    assert_eq!(
        lf_mode_hold_targets(LfSessionMode::LfHipPairLegacy).unwrap(),
        vec![
            parking,
            static_target(Leg::Lf, JointKind::Upper, UPPER_90_DELTA).unwrap(),
            static_target(Leg::Lf, JointKind::Lower, LOWER_FOLDED_DELTA).unwrap(),
        ]
    );
    assert_eq!(
        lf_mode_hold_targets(LfSessionMode::LfSingleContactLegacy {
            joint: UpperOrLower::Lower,
            side: ContactSide::Min,
        })
        .unwrap(),
        vec![
            parking,
            static_target(Leg::Lf, JointKind::Hip, 0).unwrap(),
            static_target(Leg::Lf, JointKind::Upper, UPPER_90_DELTA).unwrap(),
        ]
    );
}

#[test]
fn mutated_lf_identity_is_rejected_per_rule_while_choreography_hints_stay_unread() {
    let arm_value = LF_FULL_SEQUENCE_ARM_VALUE;
    let mode = LfSessionMode::LfFullLegSession;
    let mutate = |edit: &dyn Fn(&mut RawLegCalibrationSpec)| {
        let mut raw = canonical_lf_v25_raw_spec(arm_value);
        edit(&mut raw);
        validate_lf_v25(&raw, mode).err().unwrap()
    };

    // LFID-1
    assert_eq!(mutate(&|raw| raw.leg = Leg::Rf), LfIdentityError::Leg);
    // LFID-2
    assert_eq!(
        mutate(&|raw| raw.joints[1].name = "rf_upper_leg_joint".to_string()),
        LfIdentityError::JointName
    );
    assert_eq!(
        mutate(&|raw| raw.joints.swap(0, 1)),
        LfIdentityError::JointName
    );
    // LFID-3
    assert_eq!(
        mutate(&|raw| raw.joints[1].motor_id = 22),
        LfIdentityError::MotorId
    );
    // LFID-4
    assert_eq!(
        mutate(&|raw| raw.joints[1].direction = -1),
        LfIdentityError::Direction
    );
    // LFID-5
    assert_eq!(
        mutate(&|raw| raw.joints[1].max_delta = UPPER_MAX_DELTA + 1),
        LfIdentityError::Limits
    );
    assert_eq!(
        mutate(&|raw| raw.joints[0].min_delta = HIP_MIN_DELTA + 1),
        LfIdentityError::Limits
    );
    // LFID-6
    assert_eq!(
        mutate(&|raw| raw.oracle_identity.physical_evidence_sha256 = "0".repeat(64)),
        LfIdentityError::OracleIdentity
    );
    assert_eq!(
        mutate(&|raw| raw.oracle_identity.reconciler_source_sha256 = "0".repeat(64)),
        LfIdentityError::OracleIdentity
    );
    assert_eq!(
        mutate(&|raw| raw.oracle_identity.tolerance_ticks = LF_CONTACT_WITNESS_TOLERANCE_TICKS + 1),
        LfIdentityError::OracleIdentity
    );
    assert_eq!(
        mutate(&|raw| raw.oracle_identity.reference_contact_ticks[0] = (JointKind::Hip, 2536, 1617)),
        LfIdentityError::OracleIdentity
    );
    assert_eq!(
        mutate(
            &|raw| raw.oracle_identity.reference_contact_ticks[0] = (JointKind::Upper, 1443, 3442)
        ),
        LfIdentityError::OracleIdentity
    );
    // LFID-7
    assert_eq!(
        mutate(&|raw| raw.requested_arm_value = "LF_HIP_M13_MIN_MAX".to_string()),
        LfIdentityError::SessionMode
    );
    assert_eq!(
        mutate(&|raw| raw.requested_arm_value = "RF_UPPER_M22_MIN".to_string()),
        LfIdentityError::SessionMode
    );
    // LFID-8
    assert_eq!(
        mutate(&|raw| raw.parking_reference.as_mut().unwrap().motor_id = 32),
        LfIdentityError::ParkingIdentity
    );
    assert_eq!(
        mutate(&|raw| raw.parking_reference.as_mut().unwrap().leg = Leg::Rh),
        LfIdentityError::ParkingIdentity
    );
    assert_eq!(
        mutate(&|raw| raw.parking_reference.as_mut().unwrap().joint_name =
            "rh_upper_leg_joint".to_string()),
        LfIdentityError::ParkingIdentity
    );

    // every rule is attributable
    for error in [
        LfIdentityError::Leg,
        LfIdentityError::JointName,
        LfIdentityError::MotorId,
        LfIdentityError::Direction,
        LfIdentityError::Limits,
        LfIdentityError::OracleIdentity,
        LfIdentityError::SessionMode,
        LfIdentityError::ParkingIdentity,
        LfIdentityError::Participants,
    ] {
        assert!(error.rule().starts_with("LFID-"));
    }

    // BRAND-1 / BRAND-2: a mutated motion-bearing hint changes nothing, because
    // validate_lf_v25 never reads it.
    let canonical = canonical_lf_v25_raw_spec(arm_value);
    let mut hinted = canonical.clone();
    hinted.choreography = RawChoreographyHints {
        joint_order: vec![JointKind::Hip, JointKind::Hip, JointKind::Hip],
        side_order: vec![[ContactSide::Max, ContactSide::Min]],
        prerequisite_pose_deltas: vec![i16::MAX, i16::MIN, 0],
        parking_held_whole_session: false,
        restore_order: vec![13, 13, 13],
        historical_pose_source: "not-the-lf-historical-source".to_string(),
    };
    assert_ne!(canonical.choreography, hinted.choreography);

    let from_canonical = validate_lf_v25(&canonical, mode).unwrap();
    let from_hinted = validate_lf_v25(&hinted, mode).unwrap();
    assert_eq!(from_canonical.mode(), from_hinted.mode());
    assert_eq!(from_canonical.joints(), from_hinted.joints());
    assert_eq!(from_canonical.participants(), from_hinted.participants());
    assert_eq!(
        from_canonical
            .oracle()
            .reference_contact_ticks(JointKind::Hip),
        from_hinted.oracle().reference_contact_ticks(JointKind::Hip)
    );
}

#[test]
fn one_engine_only_and_no_per_leg_state_machine_family_exists() {
    let source = include_str!("matdog.rs");

    for forbidden in [
        "LfSessionStateMachine",
        "RfSessionStateMachine",
        "RhSessionStateMachine",
        "LhSessionStateMachine",
        "LegSessionStateMachineFor",
        "RfSessionState",
        "RhSessionState",
        "LhSessionState",
        "run_rf_",
        "run_rh_",
        "run_lh_",
    ] {
        assert!(
            !source.contains(forbidden),
            "per-leg engine family token: {forbidden}"
        );
    }
    assert_eq!(source.matches("struct LegSessionStateMachine").count(), 1);
    assert_eq!(source.matches("impl LegSessionStateMachine").count(), 1);

    for forbidden in [
        "MotionGrant",
        "AuthorizedGoal",
        "MotionToken",
        "AuthorizationToken",
        "DirectGeometryTarget",
        "TrustedGeometryManifest",
        "ExpectedGeometryV5DW4",
        "OfflineLegCalibrationSpec",
        "validate_offline",
    ] {
        assert!(
            !source.contains(forbidden),
            "forbidden concept: {forbidden}"
        );
    }

    // the one engine is the mode-parameterised session type
    for (_, mode) in reviewed_lf_runtime_arms() {
        let session = LegSessionStateMachine::new(mode, lf_entry_positions()).unwrap();
        assert_eq!(session.mode(), mode);
        assert_eq!(session.role_for(13).unwrap(), session.role_for(13).unwrap());
    }
}

#[test]
fn engine_contexts_are_constructible_only_through_the_intended_lifecycle() {
    let source = include_str!("matdog.rs");
    // the only constructor requires the sealed brand
    assert_eq!(
        source
            .matches("pub(super) fn enter(spec: &ArmableLfSessionSpec)")
            .count(),
        1
    );
    assert!(!source.contains("pub(crate) enum EngineContext"));
    assert!(!source.contains("pub enum EngineContext"));

    let raw = canonical_lf_v25_raw_spec(LF_FULL_SEQUENCE_ARM_VALUE);
    let armed = validate_lf_v25(&raw, LfSessionMode::LfFullLegSession).unwrap();
    let mut entry = EngineContext::enter(&armed);

    assert_eq!(entry.mode(), LfSessionMode::LfFullLegSession);
    assert_eq!(entry.entry_step(), Some(EntryStep::ExactSetVerify));
    assert_eq!(entry.grammar_node(), None);
    assert_eq!(entry.active_motor(), None);
    // no session-owned state before the session exists
    assert!(entry.set_active(Some(12)).is_err());
    assert!(entry.enter_node(GrammarNode::Parking).is_err());

    entry.advance_entry(EntryStep::GlobalTorqueOff).unwrap();
    entry
        .advance_entry(EntryStep::FullNormalization {
            motor: 21,
            stage: NormalizationStage::Prime,
        })
        .unwrap();
    entry
        .advance_entry(EntryStep::FullNormalization {
            motor: 21,
            stage: NormalizationStage::Settle,
        })
        .unwrap();
    // CTX-3: the legacy entry steps are not reachable in the full session
    assert!(entry
        .advance_entry(EntryStep::LegacyProfileEntry {
            motor: 11,
            stage: LegacyEntryStage::Prime,
        })
        .is_err());

    // Full mode owns InitialRecovery; the legacy Prerequisites node is not its
    assert!(EngineContext::enter(&armed)
        .begin_session(GrammarNode::Prerequisites)
        .is_err());
    let mut session = entry.begin_session(GrammarNode::InitialRecovery).unwrap();
    assert_eq!(session.entry_step(), None);
    assert_eq!(session.grammar_node(), Some(GrammarNode::InitialRecovery));
    assert!(session.advance_entry(EntryStep::ExactSetVerify).is_err());
    session.enter_node(GrammarNode::Parking).unwrap();
    session.set_active(Some(42)).unwrap();
    assert_eq!(session.active_motor(), Some(42));
    // entering a new node clears the active motor; the session is created once
    session.enter_node(GrammarNode::Diagnostics).unwrap();
    assert_eq!(session.active_motor(), None);
    assert!(session.begin_session(GrammarNode::Cleanup).is_err());

    // the two legacy modes keep their own entry and their own grammar
    let hip_raw = canonical_lf_v25_raw_spec(LF_HIP_SEQUENCE_ARM_VALUE);
    let hip_armed = validate_lf_v25(&hip_raw, LfSessionMode::LfHipPairLegacy).unwrap();
    let mut hip_entry = EngineContext::enter(&hip_armed);
    assert!(hip_entry
        .advance_entry(EntryStep::FullNormalization {
            motor: 11,
            stage: NormalizationStage::Prime,
        })
        .is_err());
    for stage in [
        LegacyEntryStage::Prime,
        LegacyEntryStage::Reassert,
        LegacyEntryStage::Prerequisite,
    ] {
        hip_entry
            .advance_entry(EntryStep::LegacyProfileEntry { motor: 11, stage })
            .unwrap();
    }
    assert!(EngineContext::enter(&hip_armed)
        .begin_session(GrammarNode::InitialRecovery)
        .is_err());
    hip_entry.begin_session(GrammarNode::Prerequisites).unwrap();

    // mode-scoped grammar, straight from contract sections 6.1 and 6.2
    let single = LfSessionMode::LfSingleContactLegacy {
        joint: UpperOrLower::Lower,
        side: ContactSide::Min,
    };
    assert!(grammar_node_admissible(
        LfSessionMode::LfHipPairLegacy,
        GrammarNode::ContactSearch {
            joint: JointKind::Hip,
            side: ContactSide::Max,
        }
    ));
    assert!(!grammar_node_admissible(
        LfSessionMode::LfHipPairLegacy,
        GrammarNode::ContactSearch {
            joint: JointKind::Upper,
            side: ContactSide::Max,
        }
    ));
    assert!(grammar_node_admissible(
        single,
        GrammarNode::ContactSearch {
            joint: JointKind::Lower,
            side: ContactSide::Min,
        }
    ));
    assert!(!grammar_node_admissible(
        single,
        GrammarNode::ContactSearch {
            joint: JointKind::Lower,
            side: ContactSide::Max,
        }
    ));
    assert!(!grammar_node_admissible(single, GrammarNode::Diagnostics));
    assert!(!grammar_node_admissible(
        LfSessionMode::LfFullLegSession,
        GrammarNode::Prerequisites
    ));
    assert!(grammar_node_admissible(
        LfSessionMode::LfFullLegSession,
        GrammarNode::RestoreParking
    ));
    // GRM-5: Cleanup is reachable in every mode
    for (_, mode) in reviewed_lf_runtime_arms() {
        assert!(grammar_node_admissible(mode, GrammarNode::Cleanup));
        assert!(grammar_node_admissible(mode, GrammarNode::TorqueOff));
    }
}

// ===========================================================================
// P2A-G3-B — GoalPosition authority-graph regression
//
// These tests pin the frozen call graph of G2 section 8.3:
//
//   23 historical LF V25 write paths
//        -> 12 intent-specific private engine operations
//        -> 2 private policy writers
//        -> 1 private raw GoalPosition constructor / emitter
//        -> RamRegister::GoalPosition
//
// They walk the production source mechanically. matdog_test.rs is not part of
// the scanned production scope, and matdog.rs has no #[cfg(test)] block that
// constructs a GoalPosition write.
// ===========================================================================

/// Attribute every production line of matdog.rs to the enclosing function, with
/// comment lines stripped so a doc comment naming a symbol is never mistaken for
/// a call to it.
fn production_functions() -> Vec<(String, String)> {
    let source = include_str!("matdog.rs");
    let mut functions: Vec<(String, String)> = Vec::new();
    let mut current: Option<(String, String)> = None;
    for line in source.lines() {
        let trimmed = line.trim_start();
        let starts_item = (trimmed.starts_with("fn ")
            || trimmed.starts_with("async fn ")
            || trimmed.starts_with("const fn ")
            || trimmed.starts_with("pub(super) fn ")
            || trimmed.starts_with("pub(super) async fn ")
            || trimmed.starts_with("pub(super) const fn ")
            || trimmed.starts_with("pub(crate) fn ")
            || trimmed.starts_with("pub(crate) async fn "))
            && trimmed.contains('(');
        if starts_item {
            if let Some(entry) = current.take() {
                functions.push(entry);
            }
            let name = trimmed
                .trim_start_matches("pub(crate) ")
                .trim_start_matches("pub(super) ")
                .trim_start_matches("async ")
                .trim_start_matches("const ")
                .trim_start_matches("fn ")
                .split(['(', '<'])
                .next()
                .unwrap_or_default()
                .to_string();
            current = Some((name, String::new()));
        }
        if trimmed.starts_with("//") {
            continue;
        }
        if let Some((_, body)) = current.as_mut() {
            body.push_str(line);
            body.push('\n');
        }
    }
    if let Some(entry) = current {
        functions.push(entry);
    }
    functions
}

fn functions_containing(needle: &str) -> Vec<String> {
    production_functions()
        .into_iter()
        .filter(|(_, body)| body.contains(needle))
        .map(|(name, _)| name)
        .collect()
}

/// The twelve intent-specific engine operations of G2 section 8.2, with the
/// W-rows of section 7 each covers. Every historical write path appears exactly
/// once across this table.
fn engine_operation_w_closure() -> [(&'static str, &'static [&'static str]); 12] {
    [
        ("home_normalization_prime", &["W1", "W2"] as &[&'static str]),
        ("home_reassert_torque_on", &["W3"]),
        ("prime_at_present", &["W4"]),
        ("prerequisite_or_parking_move", &["W5", "W6"]),
        ("moving_baseline_step", &["W7", "W8"]),
        ("probe_advance_step", &["W9", "W10", "W11"]),
        ("stop_pressure_at_observation", &["W12"]),
        ("stop_pressure_at_recorded_contact", &["W13"]),
        ("backoff_step", &["W14"]),
        ("static_hold_transition", &["W15", "W16"]),
        ("staged_affine_q0", &["W17", "W18", "W19"]),
        ("return_home", &["W20", "W21", "W22", "W23"]),
    ]
}

#[test]
fn exactly_one_production_goal_position_write_construction_site_exists() {
    // A construction site passes RamRegister::GoalPosition TOGETHER WITH a value
    // payload to a RAM-write helper. Bare enum references inside register
    // allowlists and register-policy predicates are not construction sites and
    // legitimately remain.
    let mentioning = functions_containing("RamRegister::GoalPosition");
    assert_eq!(
        mentioning,
        vec![
            "ram_write_allowed_for_profile".to_string(),
            "is_allowed_matdog_ram_register".to_string(),
            "construct_goal_position_write".to_string(),
        ]
    );

    let constructing = production_functions()
        .into_iter()
        .filter(|(_, body)| {
            body.contains("RamRegister::GoalPosition") && body.contains("to_le_bytes().to_vec()")
        })
        .map(|(name, _)| name)
        .collect::<Vec<_>>();
    assert_eq!(
        constructing,
        vec!["construct_goal_position_write".to_string()]
    );
}

#[test]
fn the_raw_goal_constructor_has_exactly_two_direct_policy_writer_callers() {
    let callers = production_functions()
        .into_iter()
        .filter(|(name, body)| {
            name != "construct_goal_position_write"
                && body.contains("self.construct_goal_position_write(")
        })
        .map(|(name, _)| name)
        .collect::<Vec<_>>();
    assert_eq!(
        callers,
        vec![
            "startup_home_goal_policy_write".to_string(),
            "armed_goal_policy_write".to_string(),
        ]
    );

    // The construction boundary is reached only through the two writers: no
    // third caller and no direct bypass of either of them.
    let envelope_callers = production_functions()
        .into_iter()
        .filter(|(name, body)| {
            name != "emit_goal_write_envelope" && body.contains("self.emit_goal_write_envelope(")
        })
        .map(|(name, _)| name)
        .collect::<Vec<_>>();
    assert_eq!(
        envelope_callers,
        vec!["construct_goal_position_write".to_string()]
    );
}

#[test]
fn the_startup_writer_covers_only_w1_w2_w3_and_the_normal_writer_only_w4_to_w23() {
    let startup = production_functions()
        .into_iter()
        .filter(|(name, body)| {
            name != "startup_home_goal_policy_write"
                && body.contains("self.startup_home_goal_policy_write(")
        })
        .map(|(name, _)| name)
        .collect::<Vec<_>>();
    assert_eq!(
        startup,
        vec![
            "home_normalization_prime".to_string(),
            "home_reassert_torque_on".to_string(),
        ]
    );

    let armed = production_functions()
        .into_iter()
        .filter(|(name, body)| {
            name != "armed_goal_policy_write" && body.contains("self.armed_goal_policy_write(")
        })
        .map(|(name, _)| name)
        .collect::<Vec<_>>();
    assert_eq!(
        armed,
        vec![
            "prime_at_present".to_string(),
            "prerequisite_or_parking_move".to_string(),
            "moving_baseline_step".to_string(),
            "probe_advance_step".to_string(),
            "stop_pressure_at_observation".to_string(),
            "stop_pressure_at_recorded_contact".to_string(),
            "backoff_step".to_string(),
            "static_hold_transition".to_string(),
            "staged_affine_q0".to_string(),
            "return_home".to_string(),
        ]
    );

    // W-row coverage: the startup writer's operations own exactly W1..W3 and the
    // normal writer's operations own exactly W4..W23.
    let closure = engine_operation_w_closure();
    let rows_for = |operations: &[String]| {
        let mut rows = operations
            .iter()
            .flat_map(|operation| {
                closure
                    .iter()
                    .find(|(name, _)| name == operation)
                    .expect("every writer caller is a reviewed engine operation")
                    .1
                    .iter()
                    .copied()
            })
            .collect::<Vec<_>>();
        rows.sort_by_key(|row| row[1..].parse::<u8>().unwrap());
        rows
    };
    assert_eq!(rows_for(&startup), vec!["W1", "W2", "W3"]);
    assert_eq!(
        rows_for(&armed),
        (4..=23).map(|n| format!("W{n}")).collect::<Vec<_>>()
    );
}

#[test]
fn the_policy_writer_caller_union_is_exactly_the_twelve_engine_operations() {
    let mut union = production_functions()
        .into_iter()
        .filter(|(name, body)| {
            name != "startup_home_goal_policy_write"
                && name != "armed_goal_policy_write"
                && (body.contains("self.startup_home_goal_policy_write(")
                    || body.contains("self.armed_goal_policy_write("))
        })
        .map(|(name, _)| name)
        .collect::<Vec<_>>();
    union.sort();
    union.dedup();
    assert_eq!(union.len(), 12, "no thirteenth authority-bearing operation");

    let mut expected = engine_operation_w_closure()
        .iter()
        .map(|(name, _)| name.to_string())
        .collect::<Vec<_>>();
    expected.sort();
    assert_eq!(union, expected);
}

#[test]
fn every_historical_write_path_w1_to_w23_is_represented_exactly_once() {
    let mut rows = engine_operation_w_closure()
        .iter()
        .flat_map(|(_, rows)| rows.iter().copied())
        .collect::<Vec<_>>();
    assert_eq!(rows.len(), 23, "the source write path count is 23");
    rows.sort_by_key(|row| row[1..].parse::<u8>().unwrap());
    rows.dedup();
    assert_eq!(rows.len(), 23, "no W-row is duplicated");
    assert_eq!(rows, (1..=23).map(|n| format!("W{n}")).collect::<Vec<_>>());
}

#[test]
fn w2_torque_off_prime_and_w3_torque_on_reassertion_stay_two_distinct_writes() {
    let functions = production_functions();
    let body_of = |name: &str| {
        functions
            .iter()
            .find(|(function, _)| function == name)
            .map(|(_, body)| body.clone())
            .unwrap_or_else(|| panic!("{name} exists"))
    };

    // W2 requires torque OFF; W3 requires torque ON. Neither can stand in for
    // the other, and each emits exactly once.
    let prime = body_of("home_normalization_prime");
    assert!(prime.contains("if observation.torque_enabled"));
    assert!(prime.contains("requires torque OFF before the write"));
    assert_eq!(prime.matches("startup_home_goal_policy_write(").count(), 1);

    let reassert = body_of("home_reassert_torque_on");
    assert!(reassert.contains("if !observation.torque_enabled"));
    assert!(reassert.contains("requires torque ON after the preceding prime"));
    assert!(reassert.contains("does not match the motor primed by W2"));
    assert_eq!(
        reassert.matches("startup_home_goal_policy_write(").count(),
        1
    );

    // The legacy recovery still performs prime -> torque enable -> reassertion,
    // in that order, for the same motor.
    let recovery = body_of("recover_home_only_joints");
    let prime_stage = recovery
        .find("stage: LegacyEntryStage::Prime")
        .expect("W2 prime stage");
    let reassert_stage = recovery
        .find("stage: LegacyEntryStage::Reassert")
        .expect("W3 reassert stage");
    assert!(prime_stage < reassert_stage);
    // The torque enable that separates them lives in the prepare helper.
    let prepare = body_of("prepare_startup_home_recovery_motor");
    let prepare_prime = prepare
        .find("self.home_normalization_prime()")
        .expect("W1/W2 prime");
    let prepare_torque_on = prepare
        .find("set_startup_home_torque_verified(motor_id, true)")
        .expect("torque enable");
    assert!(prepare_prime < prepare_torque_on);

    // The full session never reasserts: it writes once and dwells on that goal.
    let normalization = body_of("normalize_all_matdog_joints_to_q0");
    assert!(normalization.contains("stage: NormalizationStage::Prime"));
    assert!(normalization.contains("stage: NormalizationStage::Settle"));
    assert!(!normalization.contains("LegacyEntryStage"));
}

#[test]
fn the_two_policy_writers_keep_their_different_motor_admission_gates() {
    let functions = production_functions();
    let body_of = |name: &str| {
        functions
            .iter()
            .find(|(function, _)| function == name)
            .map(|(_, body)| body.clone())
            .unwrap_or_else(|| panic!("{name} exists"))
    };

    // Startup transport: any canonical MATDOG motor, plus the startup-local RAM
    // policy. It must NOT be narrowed to the armed profile allowlist, because
    // W1 normalizes all twelve motors and W2/W3 recover home-only joints that
    // are not participants of the armed profile.
    let startup = body_of("write_startup_home_ram_verified");
    assert!(startup.contains("MATDOG_MOTOR_IDS.contains(&motor_id)"));
    assert!(startup.contains("ram_write_allowed_for_profile("));
    assert!(!startup.contains("self.profile.allowed_motor_ids"));

    // Normal transport: the armed profile allowlist, exactly as before.
    let armed = body_of("write_motor_ram_verified");
    assert!(armed.contains("self.profile.allowed_motor_ids.contains(&motor_id)"));
    assert!(!armed.contains("MATDOG_MOTOR_IDS.contains(&motor_id)"));

    // The routes are selected below the construction boundary and never merged.
    let envelope = body_of("emit_goal_write_envelope");
    assert!(envelope.contains("GoalWriteRoute::StartupHome"));
    assert!(envelope.contains("GoalWriteRoute::ArmedProfile"));
    assert!(envelope.contains("write_startup_home_ram_verified"));
    assert!(envelope.contains("write_motor_ram_verified"));

    // The all-twelve startup admission itself is unchanged.
    let startup_profile = lf_full_sequence_profile().unwrap();
    for motor_id in MATDOG_MOTOR_IDS {
        assert!(ram_write_allowed_for_profile(
            &startup_profile,
            motor_id,
            RamRegister::GoalPosition.address() as u32,
            &HOME_TICK.to_le_bytes(),
        ));
    }
}

#[test]
fn rf_rh_lh_and_isolated_hip_cannot_reach_the_goal_authority_graph() {
    // Every one of the six reviewed LF runtime values receives an authority
    // context; nothing else does.
    for (arm_value, mode) in reviewed_lf_runtime_arms() {
        let context = engine_authority_for_arm_value(arm_value)
            .unwrap_or_else(|| panic!("{arm_value} is a reviewed LF runtime value"));
        assert_eq!(context.mode(), mode);
        assert_eq!(context.entry_step(), Some(EntryStep::ExactSetVerify));
    }

    for profile in all_profiles().unwrap() {
        if reviewed_lf_runtime_arms()
            .iter()
            .any(|(arm_value, _)| *arm_value == profile.arm_value)
        {
            continue;
        }
        assert!(
            engine_authority_for_arm_value(&profile.arm_value).is_none(),
            "{} must not receive engine motion authority",
            profile.arm_value
        );
    }

    // Named explicitly: the eighteen non-LF tokens and the two isolated LF hip
    // tokens.
    for token in ["RF_UPPER_M22_MIN", "RH_LOWER_M31_MIN", "LH_LOWER_M41_MAX"] {
        assert!(engine_authority_for_arm_value(token).is_none());
    }
    for token in ["LF_HIP_M13_MIN", "LF_HIP_M13_MAX"] {
        assert!(engine_authority_for_arm_value(token).is_none());
        // still recognized offline, still hardware-blocked
        let profile = profile_for_arm_value(token).unwrap();
        assert!(hardware_profile_allowed(&profile).is_err());
    }

    // Without a context every operation fails closed: each one begins by
    // resolving the authority context or the node that only a session provides.
    let functions = production_functions();
    for (operation, _) in engine_operation_w_closure() {
        let body = functions
            .iter()
            .find(|(name, _)| name == operation)
            .map(|(_, body)| body.clone())
            .unwrap_or_else(|| panic!("{operation} exists"));
        // CTX-4 / AUTH-4 / AUTH-6: the live EngineContext must be resolved
        // BEFORE the policy-writer call. Engine bookkeeping such as the motion
        // node is NOT a substitute: only these primitives read the context.
        let up_to_writer = body
            .split("_policy_write(")
            .next()
            .expect("every operation reaches a policy writer");
        assert!(
            up_to_writer.contains("self.session_motion_authority()?")
                || up_to_writer.contains("self.session_probe_authority()?")
                || up_to_writer.contains("self.authority_context()?")
                || up_to_writer.contains("self.current_entry_step()?")
                || up_to_writer.contains("self.session_mode()?"),
            "{operation} must resolve the live EngineContext before it can write"
        );

        // DER-3: identifiers and evidence only. No operation accepts a raw
        // tick, motor id, step size, scout flag or pose as a caller choice.
        let signature = body
            .lines()
            .take_while(|line| !line.contains("-> Result<"))
            .chain(
                body.lines()
                    .filter(|line| line.contains("-> Result<"))
                    .take(1),
            )
            .collect::<Vec<_>>()
            .join(" ");
        for forbidden in [
            "motor_id: u8",
            "target: u16",
            "tick: u16",
            "position: u16",
            "step_ticks: u16",
            "scout",
            "pose",
        ] {
            assert!(
                !signature.contains(forbidden),
                "{operation} must not accept `{forbidden}` as a caller choice"
            );
        }
    }
}

#[test]
fn the_authority_graph_introduces_no_motion_token_and_no_geometry_v5_target() {
    let source = include_str!("matdog.rs");
    for forbidden in [
        "MotionGrant",
        "AuthorizedGoal",
        "GoalAuthorization",
        "MotionToken",
        "AuthorizationToken",
        "DirectGeometryTarget",
        "TrustedGeometryManifest",
        "ExpectedGeometryV5DW4",
        "OfflineLegCalibrationSpec",
        "validate_offline",
        "GeometryV5",
        "geometry_v5",
    ] {
        assert!(
            !source.contains(forbidden),
            "forbidden concept: {forbidden}"
        );
    }

    // The engine contexts are still state, not permission: no operation stores a
    // goal for later emission, and every operation emits inside its own call.
    let functions = production_functions();
    for (operation, _) in engine_operation_w_closure() {
        let body = functions
            .iter()
            .find(|(name, _)| name == operation)
            .map(|(_, body)| body.clone())
            .unwrap_or_else(|| panic!("{operation} exists"));
        assert!(
            body.contains("_policy_write("),
            "{operation} must emit within its own call"
        );
    }

    // The only runtime pose source stays the LF historical one: the staged
    // targets come from this session's affine evidence and the held poses from
    // the immutable LF delta constants.
    let staged = functions
        .iter()
        .find(|(name, _)| name == "staged_affine_q0")
        .map(|(_, body)| body.clone())
        .unwrap();
    assert!(staged.contains("affine.estimated_zero_tick"));
    assert!(!staged.contains("fixed_scale"));
    assert!(staged.contains("MODEL_ZERO_MAX_SHIFT_FROM_DIGITAL_HOME_TICKS"));
    let hold = functions
        .iter()
        .find(|(name, _)| name == "static_hold_transition")
        .map(|(_, body)| body.clone())
        .unwrap();
    assert!(hold.contains("UPPER_90_DELTA"));
    assert!(hold.contains("LOWER_FOLDED_DELTA"));
    assert!(!hold.contains("UPPER_85_DELTA"));
}

#[test]
fn progress_counts_and_phase_strings_are_unchanged_by_the_authority_extraction() {
    let source = include_str!("matdog.rs");
    // G3-C: the totals are derived from the reviewed grammar, never assigned as
    // literals. The numbers themselves are unchanged.
    // The two always-LF wrappers derive their total from the grammar directly;
    // the legacy single-contact wrapper publishes the authority-free structural
    // envelope and the grammar total is validated in mark_done.
    assert_eq!(
        source
            .matches("calibrator.total_steps = calibrator.derive_expected_progress_total()?;")
            .count(),
        2
    );
    assert_eq!(
        source
            .matches(
                "calibrator.total_steps = single_contact_progress_envelope(&calibrator.profile);"
            )
            .count(),
        1
    );
    assert_eq!(
        lf_expected_progress_total(LfSessionMode::LfFullLegSession),
        58
    );
    assert_eq!(
        lf_expected_progress_total(LfSessionMode::LfHipPairLegacy),
        20
    );
    assert_eq!(
        lf_expected_progress_total(LfSessionMode::LfSingleContactLegacy {
            joint: UpperOrLower::Upper,
            side: ContactSide::Min,
        }),
        16
    );
    assert!(source.contains("\"single-session LF native calibration preflight\""));
    assert!(source.contains("\"LF HIP MIN+MAX shared-geometry preflight\""));
    assert!(source.contains("\"MATDOG native profile preflight\""));
    assert!(source.contains("format!(\"{}: {phase}\", self.profile.label)"));
    assert!(source.contains(
        "self.publish_progress(self.total_steps, \"completed\", CalibrationStatus::Done, None)"
    ));

    // The three modes keep their own next_phase counts.
    let count_in = |start_anchor: &str, end_anchor: &str| {
        let start = source.find(start_anchor).expect(start_anchor);
        let end = source[start..]
            .find(end_anchor)
            .map(|offset| start + offset)
            .expect(end_anchor);
        source[start..end].matches("self.next_phase(").count()
    };
    assert_eq!(
        count_in(
            "async fn run(&mut self) -> Result<ContactResult, DynError>",
            "async fn run_lf_hip_min_max"
        ),
        16
    );
    assert_eq!(
        count_in(
            "    async fn run_lf_hip_min_max(",
            "    async fn run_lf_state_machine("
        ),
        20
    );
    assert_eq!(
        count_in(
            "    async fn run_lf_state_machine(",
            "    fn inspect_lf_native_session_entry("
        ),
        16
    );
    // Full: 16 direct next_phase calls plus 7 per contact side x 6 sides = 58.
    assert_eq!(
        count_in(
            "    async fn measure_lf_contact_side_efficient(",
            "    async fn acquire_moving_current_baseline_forward("
        ),
        7
    );
}

#[test]
fn w7_derives_its_relative_baseline_from_a_single_entry_observation() {
    let functions = production_functions();
    let body_of = |name: &str| {
        functions
            .iter()
            .find(|(function, _)| function == name)
            .map(|(_, body)| body.clone())
            .unwrap_or_else(|| panic!("{name} exists"))
    };

    // The W7 caller performs NO telemetry read of its own. It consumes the one
    // entry observation the operation acquired, safety-checked and derived the
    // relative target from, so a single snapshot serves both uses exactly as
    // the immutable source did.
    let relative_caller = body_of("acquire_moving_current_baseline_forward");
    assert_eq!(
        relative_caller.matches("self.latest_observation(").count(),
        0,
        "the W7 caller must not acquire a second observation"
    );
    assert!(relative_caller.contains("let step = self.moving_baseline_step().await?;"));
    assert!(relative_caller.contains("step.relative_entry"));
    assert!(relative_caller.contains("let mut last_stamp = initial.monotonic_stamp_ns;"));
    assert!(relative_caller.contains("let mut previous_position = initial.position;"));

    // The operation acquires exactly one observation for the relative rule, and
    // that same observation passes the safety gate, produces the target and is
    // the one reported back.
    let operation = body_of("moving_baseline_step");
    assert_eq!(
        operation.matches("self.latest_observation(").count(),
        1,
        "W7 acquires exactly one entry observation"
    );
    assert!(operation.contains("self.ensure_observation_safe(motor_id, initial, true, None)?"));
    assert!(operation.contains("initial.position,"));
    assert!(operation.contains("(relative, Some(initial))"));
    // The guard is still checked immediately before the write, and the write is
    // still the last thing the operation does.
    let guard = operation.find("passed_guard(relative").expect("W7 guard");
    let emit = operation
        .find("self.armed_goal_policy_write(motor_id, target)")
        .expect("W7 emission");
    assert!(guard < emit);

    // W8 is untouched: the absolute rule reads no telemetry at all, and its
    // caller keeps its own single historical bookkeeping observation.
    assert!(operation.contains("(self.profile.baseline_target_tick, None)"));
    let absolute_caller = body_of("acquire_moving_current_baseline");
    assert_eq!(
        absolute_caller.matches("self.latest_observation(").count(),
        1,
        "W8 keeps its single historical bookkeeping observation"
    );
    assert!(absolute_caller.contains("let mut last_stamp = initial.monotonic_stamp_ns;"));
    assert!(absolute_caller.contains("let mut previous_position = initial.position;"));
}

#[test]
fn staged_affine_evidence_and_outcome_originate_from_the_same_accepted_result() {
    // The provenance chain, proven end to end:
    //   outcome.joints = [Hip, Upper, Lower]     (derive_joint_evidence carries
    //                                             the spec it was given)
    //   record_diagnostics stores at lf_joint_index(evidence.spec.kind)
    //   staged_affine_q0 reads session.affine[lf_joint_index(joint)]
    // so session.affine[i] IS outcome.joints[i].affine for every joint.
    assert_eq!(lf_joint_index(JointKind::Hip), 0);
    assert_eq!(lf_joint_index(JointKind::Upper), 1);
    assert_eq!(lf_joint_index(JointKind::Lower), 2);

    let hip = derive_joint_evidence(
        *spec_for(Leg::Lf, JointKind::Hip),
        supervised_lf_witness_contacts(JointKind::Hip),
    );
    let upper = derive_joint_evidence(
        *spec_for(Leg::Lf, JointKind::Upper),
        supervised_lf_witness_contacts(JointKind::Upper),
    );
    let lower = derive_joint_evidence(
        *spec_for(Leg::Lf, JointKind::Lower),
        supervised_lf_witness_contacts(JointKind::Lower),
    );
    let outcome = LfCalibrationOutcome {
        joints: [hip, upper, lower],
    };

    // Every joint's evidence carries the spec it was derived from, so the kind
    // used to record it is the kind that produced it.
    assert_eq!(outcome.joints[0].spec.kind, JointKind::Hip);
    assert_eq!(outcome.joints[1].spec.kind, JointKind::Upper);
    assert_eq!(outcome.joints[2].spec.kind, JointKind::Lower);

    let mut session =
        LegSessionStateMachine::new(LfSessionMode::LfFullLegSession, lf_entry_positions()).unwrap();
    for evidence in outcome.joints {
        session.record_contacts(evidence.spec.kind, evidence.contacts);
        session.record_diagnostics(evidence.spec.kind, evidence.fixed_scale, evidence.affine);
    }
    assert!(session.has_complete_evidence());

    // What staged_affine_q0 would derive equals what the staging call sites
    // pass to the consistency check, for all three joints.
    for (joint, outcome_index) in [
        (JointKind::Hip, 0usize),
        (JointKind::Lower, 2usize),
        (JointKind::Upper, 1usize),
    ] {
        let session_affine = session.affine[lf_joint_index(joint)].unwrap();
        assert_eq!(
            session_affine.estimated_zero_tick,
            outcome.joints[outcome_index].affine.estimated_zero_tick
        );
        assert_eq!(
            session_affine.motor_id,
            spec_for(Leg::Lf, joint).motor_id,
            "the derived motor is the canonical motor of that joint"
        );
        assert!(session_affine.accepted);
        assert!(lf_contact_witness_accepted(
            joint,
            session.contacts[lf_joint_index(joint)].unwrap()
        ));
    }

    // The staging call sites pass exactly those outcome slots, so the assertion
    // compares a value to itself on every valid trace and can only fail closed
    // on an inconsistent one. It runs after the move: it cannot alter the
    // staged target or the trace.
    let source = include_str!("matdog.rs");
    assert!(source.contains("let hip_staged_q0 = outcome.joints[0].affine.estimated_zero_tick;"));
    assert!(source.contains("let lower_staged_q0 = outcome.joints[2].affine.estimated_zero_tick;"));
    assert!(source.contains("let upper_staged_q0 = outcome.joints[1].affine.estimated_zero_tick;"));
    for (joint, binding) in [
        ("JointKind::Hip", "hip_staged"),
        ("JointKind::Lower", "lower_staged"),
        ("JointKind::Upper", "upper_staged"),
    ] {
        let move_site = source
            .find(&format!("let {binding} = self"))
            .unwrap_or_else(|| panic!("{binding} move site"));
        let check_site = source
            .find(&format!(
                "self.assert_staged_matches_outcome({joint}, {binding}, {binding}_q0)?;"
            ))
            .unwrap_or_else(|| panic!("{binding} consistency check"));
        assert!(
            move_site < check_site,
            "{binding} is checked after the move"
        );
    }
}

/// Offline bus harness that counts the GoalPosition writes that actually reach
/// the wire, so "zero writes" is proven against real emissions rather than
/// inferred from source shape.
struct GoalWriteProbe {
    normfs: Arc<normfs::NormFS>,
    tx_queue: normfs::QueueId,
    subscription: usize,
    communicator: Arc<ST3215BusCommunicator>,
    state_tx: tokio::sync::watch::Sender<InferenceState>,
    state_rx: tokio::sync::watch::Receiver<InferenceState>,
    goal_writes: Arc<std::sync::atomic::AtomicUsize>,
    goal_targets: Arc<std::sync::Mutex<Vec<(u32, u16)>>>,
    directory: std::path::PathBuf,
}

impl GoalWriteProbe {
    async fn new(label: &str) -> Self {
        static PROBE_COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);
        let id = PROBE_COUNTER.fetch_add(1, Ordering::Relaxed);
        let directory = std::env::temp_dir().join(format!(
            "matdog-goal-probe-{label}-{}-{id}",
            std::process::id()
        ));
        std::fs::create_dir_all(&directory).unwrap();
        let normfs = Arc::new(
            normfs::NormFS::new(directory.clone(), normfs::NormFsSettings::default())
                .await
                .unwrap(),
        );
        let rx_queue = normfs.resolve("probe-rx");
        let tx_queue = normfs.resolve("probe-tx");
        let meta_queue = normfs.resolve("probe-meta");
        let inference_queue = normfs.resolve("probe-inference");
        for queue in [&rx_queue, &tx_queue, &meta_queue, &inference_queue] {
            normfs.ensure_queue_exists_for_write(queue).await.unwrap();
        }

        let goal_writes = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let goal_targets = Arc::new(std::sync::Mutex::new(Vec::new()));
        let counter = goal_writes.clone();
        let targets = goal_targets.clone();
        let goal_address = RamRegister::GoalPosition.address() as u32;
        let subscription = normfs
            .subscribe(
                &tx_queue,
                Box::new(move |entries| {
                    for (_, data) in entries {
                        let command = TxEnvelope::decode(data.as_ref()).unwrap();
                        if let Some(write) = command.write.as_ref() {
                            if write.address == goal_address {
                                counter.fetch_add(1, Ordering::Relaxed);
                                let bytes = write.value.as_ref();
                                targets.lock().unwrap().push((
                                    write.motor_id,
                                    u16::from_le_bytes([bytes[0], bytes[1]]),
                                ));
                            }
                        }
                        if let Some(sync) = command.sync_write.as_ref() {
                            if sync.address == goal_address {
                                counter.fetch_add(sync.motors.len(), Ordering::Relaxed);
                            }
                        }
                    }
                    true
                }),
            )
            .unwrap();

        let communicator = Arc::new(ST3215BusCommunicator::new(
            normfs.clone(),
            rx_queue,
            tx_queue.clone(),
            meta_queue,
            inference_queue,
        ));
        let session = valid_lf_sessions_by_state().remove(2);
        let (state_tx, state_rx) = tokio::sync::watch::channel(state_for_lf_session(
            &session,
            systime::get_monotonic_stamp_ns(),
        ));
        Self {
            normfs,
            tx_queue,
            subscription,
            communicator,
            state_tx,
            state_rx,
            goal_writes,
            goal_targets,
            directory,
        }
    }

    fn refresh_state(&self) {
        self.refresh_state_from(2);
    }

    /// Re-publish fresh telemetry consistent with one of the reviewed LF
    /// session states.
    fn refresh_state_from(&self, index: usize) {
        let session = valid_lf_sessions_by_state().remove(index);
        self.state_tx
            .send(state_for_lf_session(
                &session,
                systime::get_monotonic_stamp_ns(),
            ))
            .unwrap();
    }

    fn targets(&self) -> Vec<(u32, u16)> {
        self.goal_targets.lock().unwrap().clone()
    }

    fn reset(&self) {
        self.goal_writes.store(0, Ordering::Relaxed);
        self.goal_targets.lock().unwrap().clear();
    }

    fn calibrator(&self, profile: ContactProfile) -> MatdogRamOnlyCalibrator {
        MatdogRamOnlyCalibrator::new(
            profile,
            "matdog-bus".to_string(),
            self.communicator.clone(),
            self.state_rx.clone(),
            Arc::new(AtomicBool::new(false)),
        )
    }

    /// Let any published envelope reach the counting subscription.
    async fn settle(&self) {
        for _ in 0..20 {
            tokio::task::yield_now().await;
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }

    fn writes(&self) -> usize {
        self.goal_writes.load(Ordering::Relaxed)
    }

    async fn close(self) {
        self.normfs.unsubscribe(&self.tx_queue, self.subscription);
        drop(self.communicator);
        drop(self.state_rx);
        drop(self.state_tx);
        self.normfs.close().await.unwrap();
        drop(self.normfs);
        let _ = std::fs::remove_dir_all(&self.directory);
    }
}

fn is_lf_authority_rejection(message: &str) -> bool {
    message.contains("reviewed LF runtime session authorit")
}

#[test]
fn the_entry_context_is_advanced_before_every_write_bearing_entry_operation() {
    let functions = production_functions();
    let body_of = |name: &str| {
        functions
            .iter()
            .find(|(function, _)| function == name)
            .map(|(_, body)| body.clone())
            .unwrap_or_else(|| panic!("{name} exists"))
    };

    // W4 then W6: the entry step is advanced BEFORE prepare_motor, which is the
    // W4-emitting wrapper, and therefore before the W6 move as well.
    let establish = body_of("establish_prerequisites_restart_safe");
    let advance = establish
        .find("self.advance_entry_step(EntryStep::LegacyProfileEntry {")
        .expect("entry step advance");
    let prepare = establish
        .find("self.prepare_motor(target.motor_id)")
        .expect("W4 wrapper");
    let mover = establish
        .find("self.move_profile_entry_motor_to_target(")
        .expect("W6 move");
    assert!(
        advance < prepare && prepare < mover,
        "the authority context must be advanced before the write-bearing operations"
    );

    // W2 then torque enable then W3: the entry step is advanced before each.
    let recover = body_of("recover_home_only_joints");
    let prime_stage = recover
        .find("stage: LegacyEntryStage::Prime")
        .expect("W2 stage");
    let prime_call = recover
        .find("self.prepare_startup_home_recovery_motor(motor_id)")
        .expect("W2 wrapper");
    let reassert_stage = recover
        .find("stage: LegacyEntryStage::Reassert")
        .expect("W3 stage");
    let reassert_call = recover
        .find("self.move_profile_entry_motor_to_target(")
        .expect("W3 move");
    assert!(prime_stage < prime_call && prime_call < reassert_stage);
    assert!(reassert_stage < reassert_call);

    // W1: the full session advances its normalization step before the prime.
    let normalize = body_of("normalize_all_matdog_joints_to_q0");
    let normalization_stage = normalize
        .find("stage: NormalizationStage::Prime")
        .expect("W1 stage");
    let normalization_call = normalize
        .find("self.prepare_startup_home_recovery_motor(motor_id)")
        .expect("W1 wrapper");
    assert!(normalization_stage < normalization_call);
}

#[tokio::test]
async fn without_lf_authority_no_arm_value_can_emit_a_goal_position_write() {
    let probe = GoalWriteProbe::new("no-authority").await;

    // ---- 1. Every non-LF token and both isolated LF hip tokens ----
    // The historical bypass ran establish_prerequisites_restart_safe with an
    // EMPTY home-recovery list, so the entry-step check in recover_home_only_joints
    // never executed and prepare_motor reached W4. That exact plan is used here.
    let mut refused = Vec::new();
    for profile in all_profiles().unwrap() {
        if reviewed_lf_runtime_arms()
            .iter()
            .any(|(arm_value, _)| *arm_value == profile.arm_value)
        {
            continue;
        }
        let arm_value = profile.arm_value.clone();
        let probe_motor = profile.motor_id;
        let mut calibrator = probe.calibrator(profile);
        assert!(
            calibrator.context.is_none(),
            "{arm_value} must obtain no engine authority"
        );

        let plan = StartupEntryPlan {
            home_recovery_motors: Vec::new(),
            home_ready_motors: MATDOG_MOTOR_IDS.into_iter().collect(),
        };
        probe.refresh_state();
        // Fails closed. Some profiles are refused even earlier, by the
        // read-only entry-hold verification; either way nothing is written.
        calibrator
            .establish_prerequisites_restart_safe(&plan)
            .await
            .expect_err("prerequisite establishment must be refused without LF authority");

        // The narrowest production wrapper for W4, called directly.
        let prime_error = calibrator
            .prepare_motor(probe_motor)
            .await
            .expect_err("W4 must be refused without LF authority")
            .to_string();
        assert!(
            is_lf_authority_rejection(&prime_error),
            "{arm_value}: {prime_error}"
        );

        // The operation itself, with a well-formed fresh observation.
        let observation = calibrator.latest_observation(probe_motor).unwrap();
        let operation_error = calibrator
            .prime_at_present(&FreshObservation {
                motor_id: probe_motor,
                observation,
            })
            .await
            .expect_err("prime_at_present must be refused without LF authority")
            .to_string();
        assert!(
            is_lf_authority_rejection(&operation_error),
            "{arm_value}: {operation_error}"
        );

        refused.push(arm_value);
    }

    // 18 RF/RH/LH tokens + LF_HIP_M13_MIN + LF_HIP_M13_MAX
    assert_eq!(refused.len(), 20);
    for leg in ["RF_", "RH_", "LH_"] {
        assert_eq!(
            refused.iter().filter(|arm| arm.starts_with(leg)).count(),
            6,
            "{leg}"
        );
    }
    assert!(refused.iter().any(|arm| arm == "LF_HIP_M13_MIN"));
    assert!(refused.iter().any(|arm| arm == "LF_HIP_M13_MAX"));

    probe.settle().await;
    assert_eq!(
        probe.writes(),
        0,
        "no GoalPosition write may reach the bus without LF authority"
    );

    // ---- 2. The six reviewed LF arms keep W4 reachable ----
    // With a live authority context the operation passes the authority gate and
    // only then evaluates the observation, so it fails on telemetry, never on
    // authority.
    for (arm_value, _) in reviewed_lf_runtime_arms() {
        let profile = profile_for_arm_value(arm_value).unwrap();
        let probe_motor = profile.motor_id;
        let mut calibrator = probe.calibrator(profile);
        assert!(calibrator.context.is_some(), "{arm_value}");
        calibrator
            .begin_engine_session(GrammarNode::Cleanup)
            .unwrap();

        let mut stale = calibrator.latest_observation(probe_motor).unwrap();
        stale.monotonic_stamp_ns = 1;
        let error = calibrator
            .prime_at_present(&FreshObservation {
                motor_id: probe_motor,
                observation: stale,
            })
            .await
            .expect_err("a stale observation must still be refused")
            .to_string();
        assert!(
            !is_lf_authority_rejection(&error),
            "{arm_value} must pass the authority gate: {error}"
        );
        assert!(error.contains("telemetry stale"), "{arm_value}: {error}");
    }

    probe.settle().await;
    assert_eq!(probe.writes(), 0);

    // ---- 3. Positive control: the probe really does observe a write ----
    // A reviewed LF arm, a live Session context and a fresh observation reach
    // the raw constructor and put a GoalPosition envelope on the bus.
    let mut calibrator = probe.calibrator(lf_full_sequence_profile().unwrap());
    calibrator
        .begin_engine_session(GrammarNode::InitialRecovery)
        .unwrap();
    calibrator.enter_goal_node(GoalNode::Parking).unwrap();
    calibrator.lf_session = Some(valid_lf_sessions_by_state().remove(2));
    probe.refresh_state();
    let fresh = calibrator.latest_observation(42).unwrap();
    let _ = tokio::time::timeout(
        Duration::from_millis(400),
        calibrator.prime_at_present(&FreshObservation {
            motor_id: 42,
            observation: fresh,
        }),
    )
    .await;
    probe.settle().await;
    assert_eq!(
        probe.writes(),
        1,
        "the probe must be able to observe a genuine GoalPosition write"
    );

    probe.close().await;
}

#[tokio::test]
async fn pressure_release_requires_session_authority_and_stays_reachable_in_session() {
    let probe = GoalWriteProbe::new("pressure-release").await;
    // The full-session sentinel profile probes M12, and reviewed session state 3
    // is UpperMin with M12 active as the contact probe at tick 1451.
    let profile = lf_full_sequence_profile().unwrap();
    let probe_motor = profile.motor_id;
    assert_eq!(probe_motor, 12);
    let contact_tick = 1451u16;
    let upper_min_session = || valid_lf_sessions_by_state().remove(3);

    // ---- Entry context rejects BOTH pressure releases, with zero writes ----
    {
        let mut calibrator = probe.calibrator(profile.clone());
        assert!(matches!(
            calibrator.context.as_ref().expect("reviewed LF arm"),
            EngineContext::Entry(_)
        ));
        calibrator.lf_session = Some(upper_min_session());
        calibrator.recorded_contact_tick = Some(contact_tick);
        probe.refresh_state_from(3);
        let observation = calibrator.latest_observation(probe_motor).unwrap();
        assert_eq!(observation.position, contact_tick);

        let w12 = calibrator
            .stop_pressure_at_observation(&ContactVerdict {
                motor_id: probe_motor,
                observation,
            })
            .await
            .expect_err("W12 must be refused from the pre-session entry context")
            .to_string();
        assert!(
            w12.contains("unreachable from the pre-session entry context"),
            "{w12}"
        );

        let w13 = calibrator
            .stop_pressure_at_recorded_contact()
            .await
            .expect_err("W13 must be refused from the pre-session entry context")
            .to_string();
        assert!(
            w13.contains("unreachable from the pre-session entry context"),
            "{w13}"
        );
    }
    probe.settle().await;
    assert_eq!(
        probe.writes(),
        0,
        "an entry context must emit no pressure release"
    );

    // ---- W12 positive control: Session context, correct active probe ----
    probe.reset();
    {
        let mut calibrator = probe.calibrator(profile.clone());
        calibrator
            .begin_engine_session(GrammarNode::InitialRecovery)
            .unwrap();
        calibrator.lf_session = Some(upper_min_session());
        probe.refresh_state_from(3);
        let observation = calibrator.latest_observation(probe_motor).unwrap();
        let _ = tokio::time::timeout(
            Duration::from_millis(400),
            calibrator.stop_pressure_at_observation(&ContactVerdict {
                motor_id: probe_motor,
                observation,
            }),
        )
        .await;
    }
    probe.settle().await;
    assert_eq!(
        probe.targets(),
        vec![(u32::from(probe_motor), contact_tick)],
        "W12 must still emit the verdict observation's own position"
    );

    // ---- W13 positive control: Session context, contact recorded here ----
    probe.reset();
    {
        let mut calibrator = probe.calibrator(profile.clone());
        calibrator
            .begin_engine_session(GrammarNode::InitialRecovery)
            .unwrap();
        calibrator.lf_session = Some(upper_min_session());
        calibrator.recorded_contact_tick = Some(contact_tick);
        probe.refresh_state_from(3);
        let _ = tokio::time::timeout(
            Duration::from_millis(400),
            calibrator.stop_pressure_at_recorded_contact(),
        )
        .await;
    }
    probe.settle().await;
    assert_eq!(
        probe.targets(),
        vec![(u32::from(probe_motor), contact_tick)],
        "W13 must still emit this session's recorded contact tick"
    );

    // ---- The gate binds the active probe, not just the phase ----
    probe.reset();
    {
        let mut calibrator = probe.calibrator(profile.clone());
        calibrator
            .begin_engine_session(GrammarNode::InitialRecovery)
            .unwrap();
        // Session state 2 is Parking: M42 is active, not the M12 probe.
        calibrator.lf_session = Some(valid_lf_sessions_by_state().remove(2));
        calibrator.recorded_contact_tick = Some(contact_tick);
        probe.refresh_state_from(2);
        let error = calibrator
            .stop_pressure_at_recorded_contact()
            .await
            .expect_err("the probe must be the session's active motor")
            .to_string();
        assert!(
            error.contains("is not the active LF motor of this session"),
            "{error}"
        );
    }
    probe.settle().await;
    assert_eq!(probe.writes(), 0);

    probe.close().await;
}

#[test]
fn both_pressure_release_operations_reject_the_entry_variant_before_writing() {
    let functions = production_functions();
    let body_of = |name: &str| {
        functions
            .iter()
            .find(|(function, _)| function == name)
            .map(|(_, body)| body.clone())
            .unwrap_or_else(|| panic!("{name} exists"))
    };

    // The shared resolver requires Session and rejects Entry.
    let resolver = body_of("session_probe_authority");
    assert!(resolver.contains("EngineContext::Session(_) => {}"));
    assert!(resolver.contains("EngineContext::Entry(_) =>"));
    assert!(resolver.contains("unreachable from the pre-session entry context"));
    assert!(resolver.contains("self.derived_participants()?"));
    assert!(resolver.contains("active.motor_id == motor_id"));

    // Both operations resolve it before they can reach the policy writer, and
    // neither keeps a mode-only check that would accept an entry context.
    for operation in [
        "stop_pressure_at_observation",
        "stop_pressure_at_recorded_contact",
    ] {
        let body = body_of(operation);
        let gate = body
            .find("self.session_probe_authority()?")
            .unwrap_or_else(|| panic!("{operation} must resolve the session probe authority"));
        let writer = body
            .find("self.armed_goal_policy_write(")
            .unwrap_or_else(|| panic!("{operation} must emit through the normal policy writer"));
        assert!(gate < writer, "{operation}: gate must precede the write");
        assert!(
            !body.contains("let _mode = self.session_mode()?;"),
            "{operation} must not keep a variant-blind mode check"
        );
    }
}

// ===========================================================================
// P2A-G3-C — deterministic offline session harness
//
// A simulated ST3215 bus that answers every MATDOG command, keeps a per-motor
// register model, and stops each LF probe at a mechanical limit so the REAL
// production detector reaches its historical contacts. Nothing in production
// motion logic is bypassed: the engine runs its own choreography and the
// progress events compared by the regressions are the ones production actually
// published.
// ===========================================================================

/// Mechanical stop of each LF probe, per side. Chosen inside the reviewed model
/// acceptance corridor AND within the LF V25 witness tolerance of 24 ticks, so
/// the historical contact-witness gate accepts the simulated session.
fn simulated_mechanical_stops(motor_id: u8) -> Option<(u16, u16)> {
    match motor_id {
        11 => Some((1666, 3093)), // lower: max side 1666, min side 3093
        12 => Some((1443, 3442)), // upper: min side 1443, max side 3442
        13 => Some((1600, 2535)), // hip: max side 1600 (corridor top), min 2535
        _ => None,
    }
}

const SIMULATED_TICKS_PER_FRAME: u16 = 8;
const SIMULATED_FREE_CURRENT: u16 = 3;
const SIMULATED_CONTACT_CURRENT: u16 = 60;

struct SimulatedBus {
    /// Authoritative per-motor RAM register file: every MATDOG write lands here
    /// verbatim, so every production readback verification sees what it wrote.
    registers: std::collections::BTreeMap<u8, Vec<u8>>,
    stamp: u64,
}

impl SimulatedBus {
    fn new() -> Self {
        let stamp = systime::get_monotonic_stamp_ns();
        let registers = MATDOG_MOTOR_IDS
            .into_iter()
            .map(|motor_id| {
                let observed = MotorObservation {
                    monotonic_stamp_ns: stamp,
                    position: HOME_TICK,
                    velocity: 0,
                    current: SIMULATED_FREE_CURRENT,
                    temperature: 30,
                    temperature_limit: EXPECTED_TEMPERATURE_LIMIT_C,
                    goal_position: HOME_TICK,
                    torque_limit: TORQUE_LIMIT,
                    torque_enabled: false,
                    status: 0,
                    has_driver_error: false,
                };
                let state = motor_state_from_observation(motor_id, observed);
                (motor_id, state.state.to_vec())
            })
            .collect();
        Self { registers, stamp }
    }

    fn apply_write(&mut self, motor_id: u8, address: u32, value: &[u8]) {
        let Some(bytes) = self.registers.get_mut(&motor_id) else {
            return;
        };
        let start = address as usize;
        if start + value.len() <= bytes.len() {
            bytes[start..start + value.len()].copy_from_slice(value);
        }
    }

    fn read_u16(bytes: &[u8], register: RamRegister) -> u16 {
        let address = register.address() as usize;
        u16::from_le_bytes([bytes[address], bytes[address + 1]])
    }

    /// Advance every motor one frame toward its commanded goal, stopping at the
    /// mechanical limit where one exists.
    fn step(&mut self) {
        // Track real time so the production freshness gate sees live telemetry,
        // while staying strictly monotonic for the observation-after waits.
        self.stamp = systime::get_monotonic_stamp_ns().max(self.stamp + 1);
        for (motor_id, bytes) in self.registers.iter_mut() {
            let torque_enabled = bytes[RamRegister::TorqueEnable.address() as usize] == 1;
            set_register(bytes, RamRegister::PresentSpeed, &0u16.to_le_bytes());
            if !torque_enabled {
                set_register(
                    bytes,
                    RamRegister::PresentCurrent,
                    &SIMULATED_FREE_CURRENT.to_le_bytes(),
                );
                continue;
            }
            let goal = Self::read_u16(bytes, RamRegister::GoalPosition);
            let mut position = Self::read_u16(bytes, RamRegister::PresentPosition);
            let (low, high) = simulated_mechanical_stops(*motor_id).unwrap_or((0, 4095));
            let reachable = goal.clamp(low, high);
            if position != reachable {
                let delta = i32::from(reachable) - i32::from(position);
                let step = delta.signum() * i32::from(SIMULATED_TICKS_PER_FRAME).min(delta.abs());
                position = (i32::from(position) + step) as u16;
                set_register(bytes, RamRegister::PresentPosition, &position.to_le_bytes());
            }
            let current = if position == goal {
                SIMULATED_FREE_CURRENT
            } else {
                // Held against a mechanical stop: elevated, far below the hard
                // current abort.
                SIMULATED_CONTACT_CURRENT
            };
            set_register(bytes, RamRegister::PresentCurrent, &current.to_le_bytes());
        }
    }

    fn publish(&self, bus_serial: &str, last: Option<TxEnvelope>) -> InferenceState {
        let mut motors = self
            .registers
            .iter()
            .map(|(motor_id, bytes)| {
                let mut motor = motor_state(u32::from(*motor_id), bytes.clone());
                motor.monotonic_stamp_ns = self.stamp;
                motor
            })
            .collect::<Vec<_>>();
        if let Some(command) = last {
            motors[0].last_command = Some(crate::st3215_proto::InferenceCommandState {
                command: Some(command),
                result: CommandResult::CrSuccess as i32,
            });
        }
        inference_state(bus_serial, motors)
    }
}

/// One complete offline session: drives the real production entry point for a
/// mode and returns the ordered progress events production actually emitted.
struct OfflineSession {
    events: Vec<(String, u32, CalibrationStatus)>,
    outcome: Result<(), String>,
    /// GoalPosition write frames that actually reached the simulated bus.
    goal_writes: usize,
    /// Verified global torque-OFF frames: a sync write of TorqueEnable=0 to
    /// every canonical MATDOG motor.
    verified_torque_off_frames: usize,
}

async fn run_offline_session(arm_value: &str) -> OfflineSession {
    static SESSION_COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);
    let id = SESSION_COUNTER.fetch_add(1, Ordering::Relaxed);
    let directory = std::env::temp_dir().join(format!(
        "matdog-offline-session-{}-{id}",
        std::process::id()
    ));
    std::fs::create_dir_all(&directory).unwrap();
    let normfs = Arc::new(
        normfs::NormFS::new(directory.clone(), normfs::NormFsSettings::default())
            .await
            .unwrap(),
    );
    let rx_queue = normfs.resolve("session-rx");
    let tx_queue = normfs.resolve("session-tx");
    let meta_queue = normfs.resolve("session-meta");
    let inference_queue = normfs.resolve("session-inference");
    for queue in [&rx_queue, &tx_queue, &meta_queue, &inference_queue] {
        normfs.ensure_queue_exists_for_write(queue).await.unwrap();
    }

    let (command_tx, mut command_rx) = tokio::sync::mpsc::unbounded_channel();
    let goal_writes = Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let torque_off_frames = Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let observed_goals = goal_writes.clone();
    let observed_torque_off = torque_off_frames.clone();
    let goal_address = RamRegister::GoalPosition.address() as u32;
    let torque_address = RamRegister::TorqueEnable.address() as u32;
    let subscription = normfs
        .subscribe(
            &tx_queue,
            Box::new(move |entries| {
                for (_, data) in entries {
                    let command = TxEnvelope::decode(data.as_ref()).unwrap();
                    if let Some(write) = command.write.as_ref() {
                        if write.address == goal_address {
                            observed_goals.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                    if let Some(sync) = command.sync_write.as_ref() {
                        if sync.address == goal_address {
                            observed_goals.fetch_add(sync.motors.len(), Ordering::Relaxed);
                        }
                        if sync.address == torque_address
                            && sync.motors.len() == MATDOG_MOTOR_IDS.len()
                            && sync.motors.iter().all(|write| write.value.as_ref() == [0])
                        {
                            observed_torque_off.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                    if command_tx.send(command).is_err() {
                        return false;
                    }
                }
                true
            }),
        )
        .unwrap();

    let communicator = Arc::new(ST3215BusCommunicator::new(
        normfs.clone(),
        rx_queue,
        tx_queue.clone(),
        meta_queue,
        inference_queue,
    ));

    let bus = SimulatedBus::new();
    let (state_tx, state_rx) = tokio::sync::watch::channel(bus.publish("matdog-bus", None));

    let simulator = tokio::spawn(async move {
        let mut bus = bus;
        loop {
            let mut last = None;
            tokio::select! {
                received = command_rx.recv() => match received {
                    Some(command) => {
                        if let Some(write) = command.write.as_ref() {
                            bus.apply_write(write.motor_id as u8, write.address, write.value.as_ref());
                        }
                        if let Some(sync) = command.sync_write.as_ref() {
                            for write in &sync.motors {
                                bus.apply_write(
                                    write.motor_id as u8,
                                    sync.address,
                                    write.value.as_ref(),
                                );
                            }
                        }
                        last = Some(command);
                    }
                    None => break,
                },
                _ = tokio::time::sleep(Duration::from_millis(1)) => {}
            }
            bus.step();
            if state_tx.send(bus.publish("matdog-bus", last)).is_err() {
                break;
            }
        }
    });

    let profile = profile_for_arm_value(arm_value).unwrap();
    let mut calibrator = MatdogRamOnlyCalibrator::new(
        profile.clone(),
        "matdog-bus".to_string(),
        communicator.clone(),
        state_rx,
        Arc::new(AtomicBool::new(false)),
    );

    // Drive exactly the production entry point for this mode.
    let outcome = if is_lf_full_sequence(&profile) {
        drive_full_session(&mut calibrator).await
    } else if is_lf_hip_sequence(&profile) {
        drive_hip_pair_session(&mut calibrator).await
    } else {
        drive_single_session(&mut calibrator).await
    };

    let events = calibrator.emitted_progress.lock().unwrap().clone();
    // Let the counting subscription drain the final cleanup frames.
    for _ in 0..20 {
        tokio::task::yield_now().await;
        tokio::time::sleep(Duration::from_millis(5)).await;
    }

    normfs.unsubscribe(&tx_queue, subscription);
    simulator.abort();
    drop(calibrator);
    drop(communicator);
    normfs.close().await.unwrap();
    drop(normfs);
    let _ = std::fs::remove_dir_all(&directory);

    OfflineSession {
        events,
        outcome,
        goal_writes: goal_writes.load(Ordering::Relaxed),
        verified_torque_off_frames: torque_off_frames.load(Ordering::Relaxed),
    }
}

async fn drive_single_session(calibrator: &mut MatdogRamOnlyCalibrator) -> Result<(), String> {
    execute_single_contact_session(calibrator)
        .await
        .map_err(|error| error.to_string())
}

async fn drive_hip_pair_session(calibrator: &mut MatdogRamOnlyCalibrator) -> Result<(), String> {
    let maximum_profile = lf_hip_sequence_profile(ContactSide::Max).unwrap();
    execute_hip_pair_session(calibrator, maximum_profile)
        .await
        .map_err(|error| error.to_string())
}

async fn drive_full_session(calibrator: &mut MatdogRamOnlyCalibrator) -> Result<(), String> {
    execute_full_leg_session(calibrator)
        .await
        .map_err(|error| error.to_string())
}

// ---------------------------------------------------------------------------
// Frozen G2 Revision 2.2d progress oracles.
//
// Independent TEST DATA, transcribed from the contract. Nothing here is derived
// from captured output, and none of it is runtime command authority.
// ---------------------------------------------------------------------------

/// §14.5 — the seven doubled contact-side strings of the full session. The
/// label appears twice because the phase argument embeds it and
/// `publish_progress` prefixes it again. This duplication is frozen.
fn full_contact_side_oracle(label: &str) -> Vec<String> {
    [
        "moving baseline from current pose",
        "coarse scouting pass",
        "coarse backoff",
        "fine metrology pass 1",
        "fine metrology backoff",
        "fine metrology pass 2",
        "fine-to-fine repeatability",
    ]
    .into_iter()
    .map(|phase| format!("{label}: {label} {phase}"))
    .collect()
}

/// §14.2 — the 58 operational increments of `LfFullLegSession`, in order.
fn full_operational_oracle() -> Vec<String> {
    let mut oracle: Vec<String> = [
        "LF_LEG_STATE_MACHINE: Verify exact MATDOG ID set once",
        "LF_LEG_STATE_MACHINE: Verified global torque OFF once at session entry",
        "LF_LEG_STATE_MACHINE: Normalize every displaced MATDOG joint to q=0 with one uniform rule",
        "LF_LEG_STATE_MACHINE: Create LF state machine from verified q=0 session entry",
        "LF_LEG_STATE_MACHINE: Park LH upper M42 once for the complete LF session",
        "LF_LEG_STATE_MACHINE: Prepare LF UPPER M12 once",
    ]
    .into_iter()
    .map(String::from)
    .collect();
    oracle.extend(full_contact_side_oracle("LF_UPPER_M12_MIN"));
    oracle.extend(full_contact_side_oracle("LF_UPPER_M12_MAX"));
    // #21 keeps the MAX prefix: it fires before the profile is reassigned.
    oracle.push(
        "LF_UPPER_M12_MAX: Transition M12 directly from MAX contact to horizontal hold".into(),
    );
    oracle.push("LF_LOWER_M11_MIN: Prepare LF LOWER M11 once".into());
    oracle.extend(full_contact_side_oracle("LF_LOWER_M11_MIN"));
    oracle.extend(full_contact_side_oracle("LF_LOWER_M11_MAX"));
    oracle.push(
        "LF_LOWER_M11_MAX: Transition M11 directly from MAX contact to HIP parallel hold".into(),
    );
    // #38 keeps the LOWER MAX prefix: no profile assignment separates them.
    oracle.push("LF_LOWER_M11_MAX: Prepare LF HIP M13 once".into());
    oracle.extend(full_contact_side_oracle("LF_HIP_M13_MIN_MAX"));
    oracle.extend(full_contact_side_oracle("LF_HIP_M13_MIN_MAX"));
    oracle.push(
        "LF_HIP_M13_MIN_MAX: Derive endpoint and affine q0 diagnostics from all fine contacts"
            .into(),
    );
    // #54 keeps the HIP sentinel: it fires before the sentinel is restored.
    oracle.push(
        "LF_HIP_M13_MIN_MAX: Move LF HIP M13 from MAX contact to URDF-derived staged q=0".into(),
    );
    oracle
        .push("LF_LEG_STATE_MACHINE: Move LF LOWER M11 to URDF-derived staged q=0 and hold".into());
    oracle.push(
        "LF_LEG_STATE_MACHINE: Move LF UPPER M12 to URDF-derived staged q=0 while M11 holds".into(),
    );
    oracle.push("LF_LEG_STATE_MACHINE: Restore LH upper M42 once at end of LF calibration".into());
    oracle.push("LF_LEG_STATE_MACHINE: Final verified global torque OFF".into());
    oracle
}

/// §14.3 — the 20 operational increments of `LfHipPairLegacy`, in order.
fn hip_pair_operational_oracle() -> Vec<String> {
    [
        "Verify exact MATDOG ID set",
        "Verified global torque OFF",
        "Inspect restart-safe LF HIP sequence entry",
        "Recover home-only joints to digital home",
        "Set M12 horizontal and M11 parallel",
        "Prime LF HIP M13 at digital home",
        "LF HIP MIN moving-current baseline",
        "LF HIP MIN coarse approach",
        "LF HIP MIN backoff and recovery",
        "LF HIP MIN fine repeat approach",
        "LF HIP MIN repeatability",
        "Return M13 home between MIN and MAX",
        "LF HIP MAX moving-current baseline",
        "LF HIP MAX coarse approach",
        "LF HIP MAX backoff and recovery",
        "LF HIP MAX fine repeat approach",
        "LF HIP MAX repeatability",
        "Return LF HIP M13 home",
        "Restore M11, M12 and M42 to home",
        "Final verified global torque OFF",
    ]
    .into_iter()
    .map(|phase| format!("LF_HIP_M13_MIN_MAX: {phase}"))
    .collect()
}

/// §14.4 — the 16 operational increments of `LfSingleContactLegacy`, in order.
/// The label never changes during a single-contact session.
fn single_contact_operational_oracle(label: &str) -> Vec<String> {
    [
        "Verify exact MATDOG ID set",
        "Verified global torque OFF",
        "Inspect restart-safe profile entry",
        "Recover home-only joints to digital home",
        "Establish geometry prerequisites from restart-safe state",
        "Prime and return probing joint home",
        "Acquire moving-current baseline",
        "Coarse scouting approach — measurement discarded",
        "Backoff after coarse scout",
        "First fine metrology approach",
        "Backoff between identical fine approaches",
        "Second fine metrology approach",
        "Verify fine-to-fine repeatability",
        "Return probing joint home",
        "Restore prerequisite joints one at a time",
        "Final verified global torque OFF",
    ]
    .into_iter()
    .map(|phase| format!("{label}: {phase}"))
    .collect()
}

/// Assert an ACTUAL captured session against the frozen oracle: first the
/// complete external stream (§14.0), then the operational increments obtained
/// with the contract's own filter.
fn assert_offline_trace(
    session: &OfflineSession,
    label: &str,
    preflight_phase: &str,
    operational: &[String],
) {
    assert_eq!(session.outcome, Ok(()), "{label}: session must complete");
    let expected_total = operational.len() as u32;

    // ---- complete external stream: 1 preflight + N increments + 1 Done ----
    assert_eq!(
        session.events.len(),
        operational.len() + 2,
        "{label}: external stream length"
    );
    assert_eq!(
        session.events[0],
        (
            format!("{label}: {preflight_phase}"),
            0,
            CalibrationStatus::InProgress
        ),
        "{label}: step-zero preflight"
    );
    for (index, expected) in operational.iter().enumerate() {
        assert_eq!(
            session.events[index + 1],
            (
                expected.clone(),
                index as u32 + 1,
                CalibrationStatus::InProgress
            ),
            "{label}: external event {}",
            index + 1
        );
    }
    assert_eq!(
        *session.events.last().unwrap(),
        (
            format!("{label}: completed"),
            expected_total,
            CalibrationStatus::Done
        ),
        "{label}: terminal event"
    );

    // ---- operational increments, via the §14.0 filter ----
    let filtered = session
        .events
        .iter()
        .filter(|(_, current, status)| {
            *status == CalibrationStatus::InProgress && *current >= 1 && *current <= expected_total
        })
        .cloned()
        .collect::<Vec<_>>();
    assert_eq!(
        filtered.len(),
        operational.len(),
        "{label}: operational increment count"
    );
    for (index, (string, current, status)) in filtered.iter().enumerate() {
        assert_eq!(
            string,
            &operational[index],
            "{label}: increment {}",
            index + 1
        );
        assert_eq!(*current, index as u32 + 1, "{label}: increment current");
        assert_eq!(*status, CalibrationStatus::InProgress);
    }

    // PRG-5: the grammar-derived total equals the observed increment count.
    let mode = lf_runtime_session_mode(label).unwrap_or(LfSessionMode::LfFullLegSession);
    assert_eq!(lf_expected_progress_total(mode), expected_total);
}

#[test]
fn the_reviewed_grammar_expansion_derives_every_progress_total() {
    // The totals fall out of the expansion's length, not out of a literal.
    let full = lf_progress_plan(LfSessionMode::LfFullLegSession);
    let hip_pair = lf_progress_plan(LfSessionMode::LfHipPairLegacy);
    assert_eq!(full.len(), 58);
    assert_eq!(hip_pair.len(), 20);
    for (joint, side) in [
        (UpperOrLower::Upper, ContactSide::Min),
        (UpperOrLower::Upper, ContactSide::Max),
        (UpperOrLower::Lower, ContactSide::Min),
        (UpperOrLower::Lower, ContactSide::Max),
    ] {
        let mode = LfSessionMode::LfSingleContactLegacy { joint, side };
        assert_eq!(lf_progress_plan(mode).len(), 16);
        assert_eq!(lf_expected_progress_total(mode), 16);
    }
    assert_eq!(
        lf_expected_progress_total(LfSessionMode::LfFullLegSession),
        58
    );
    assert_eq!(
        lf_expected_progress_total(LfSessionMode::LfHipPairLegacy),
        20
    );

    // The expansion is structural: the full session's own components account for
    // the total, and the three modes expand differently.
    assert_eq!(
        full.iter()
            .filter(|step| matches!(step, ProgressStep::ContactSidePhase { .. }))
            .count(),
        42,
        "six full contact sides of seven phases"
    );
    assert_eq!(
        hip_pair
            .iter()
            .filter(|step| matches!(step, ProgressStep::ContactSidePhase { .. }))
            .count(),
        10,
        "two hip-pair sides of five phases"
    );
    assert_eq!(
        full.iter()
            .filter(|step| matches!(step, ProgressStep::StaticHold(_)))
            .count(),
        2
    );
    assert_eq!(
        full.iter()
            .filter(|step| matches!(step, ProgressStep::ReturnStaged(_)))
            .count(),
        3
    );
    assert!(full.contains(&ProgressStep::Diagnostics));
    assert!(!hip_pair.contains(&ProgressStep::Diagnostics));
    assert!(hip_pair.contains(&ProgressStep::BetweenSideReturnHome));

    // No standalone numeric assignment remains as the source of truth.
    let source = include_str!("matdog.rs");
    for forbidden in ["total_steps = 58", "total_steps = 20", "total_steps = 16"] {
        assert!(
            !source.contains(forbidden),
            "stale progress literal: {forbidden}"
        );
    }
    assert_eq!(
        source
            .matches("calibrator.total_steps = calibrator.derive_expected_progress_total()?;")
            .count(),
        2
    );
    assert_eq!(
        source
            .matches(
                "calibrator.total_steps = single_contact_progress_envelope(&calibrator.profile);"
            )
            .count(),
        1
    );
    // The envelope is the SAME structural shape the LF grammar expands, so no
    // numeric total exists anywhere.
    assert!(source.contains("fn single_contact_progress_shape(joint: JointKind, side: ContactSide) -> Vec<ProgressStep> {"));
    assert!(source
        .contains("(2 + single_contact_progress_shape(profile.joint, profile.side).len()) as u32"));
    assert!(
        source.contains("plan.extend(single_contact_progress_shape(joint.joint_kind(), side));")
    );
    assert!(source.contains("fn lf_expected_progress_total(mode: LfSessionMode) -> u32 {"));
    assert!(source.contains("lf_progress_plan(mode).len() as u32"));
}

#[tokio::test]
async fn offline_full_leg_session_matches_the_frozen_progress_oracle() {
    let session = run_offline_session(LF_FULL_SEQUENCE_ARM_VALUE).await;
    let oracle = full_operational_oracle();
    assert_eq!(oracle.len(), 58);
    assert_offline_trace(
        &session,
        LF_FULL_SEQUENCE_ARM_VALUE,
        "single-session LF native calibration preflight",
        &oracle,
    );

    // The frozen label subtleties, asserted individually.
    assert_eq!(
        session.events[21].0,
        "LF_UPPER_M12_MAX: Transition M12 directly from MAX contact to horizontal hold"
    );
    assert_eq!(
        session.events[38].0,
        "LF_LOWER_M11_MAX: Prepare LF HIP M13 once"
    );
    assert_eq!(
        session.events[54].0,
        "LF_HIP_M13_MIN_MAX: Move LF HIP M13 from MAX contact to URDF-derived staged q=0"
    );
    // The doubled contact-side form survives verbatim.
    assert_eq!(
        session.events[7].0,
        "LF_UPPER_M12_MIN: LF_UPPER_M12_MIN moving baseline from current pose"
    );
}

#[tokio::test]
async fn offline_hip_pair_session_matches_the_frozen_progress_oracle() {
    let session = run_offline_session(LF_HIP_SEQUENCE_ARM_VALUE).await;
    let oracle = hip_pair_operational_oracle();
    assert_eq!(oracle.len(), 20);
    assert_offline_trace(
        &session,
        LF_HIP_SEQUENCE_ARM_VALUE,
        "LF HIP MIN+MAX shared-geometry preflight",
        &oracle,
    );
    // The sentinel label holds for the whole session, both sides included.
    assert!(session
        .events
        .iter()
        .all(|(string, _, _)| string.starts_with("LF_HIP_M13_MIN_MAX: ")));
}

#[tokio::test]
async fn offline_single_contact_sessions_match_the_frozen_progress_oracle() {
    for label in [
        "LF_UPPER_M12_MIN",
        "LF_UPPER_M12_MAX",
        "LF_LOWER_M11_MIN",
        "LF_LOWER_M11_MAX",
    ] {
        let session = run_offline_session(label).await;
        let oracle = single_contact_operational_oracle(label);
        assert_eq!(oracle.len(), 16);
        assert_offline_trace(&session, label, "MATDOG native profile preflight", &oracle);
        // The label never changes during a single-contact session.
        assert!(session
            .events
            .iter()
            .all(|(string, _, _)| string.starts_with(&format!("{label}: "))));
    }
}

#[test]
fn done_is_refused_when_any_reviewed_operation_is_skipped() {
    // The narrowest progress-accounting hook: no motion check is weakened.
    for (arm_value, expected_total) in [
        (LF_FULL_SEQUENCE_ARM_VALUE, 58u32),
        (LF_HIP_SEQUENCE_ARM_VALUE, 20),
        ("LF_UPPER_M12_MIN", 16),
        ("LF_LOWER_M11_MAX", 16),
    ] {
        let mode = lf_runtime_session_mode(arm_value).unwrap();
        assert_eq!(lf_expected_progress_total(mode), expected_total);
    }

    let source = include_str!("matdog.rs");
    // PRG-3 lives in production: mark_done compares executed against expected
    // and refuses the terminal event when they differ.
    let start = source
        .find("fn mark_done(&self) -> Result<(), DynError> {")
        .unwrap();
    let end = source[start..].find("\n    }\n").unwrap() + start;
    let body = &source[start..end];
    assert!(body.contains("if self.current_step != self.total_steps"));
    assert!(body.contains("terminal Done refused"));
    let refusal = body.find("return Err").unwrap();
    let publish = body.find("self.publish_progress(").unwrap();
    assert!(
        refusal < publish,
        "the refusal must precede any terminal publication"
    );
    // Every wrapper routes an incomplete session to Failed, never to Done.
    assert_eq!(source.matches("match calibrator.mark_done() {").count(), 3);
    assert_eq!(
        source.matches("Err(incomplete) => {").count(),
        3,
        "every wrapper routes an incomplete session to Failed"
    );
}

#[tokio::test]
async fn done_completeness_is_enforced_at_runtime_for_every_mode() {
    let probe = GoalWriteProbe::new("done-completeness").await;
    for arm_value in [
        LF_FULL_SEQUENCE_ARM_VALUE,
        LF_HIP_SEQUENCE_ARM_VALUE,
        "LF_UPPER_M12_MIN",
        "LF_UPPER_M12_MAX",
        "LF_LOWER_M11_MIN",
        "LF_LOWER_M11_MAX",
    ] {
        let profile = profile_for_arm_value(arm_value).unwrap();
        let label = profile.label.clone();
        let expected_total =
            lf_expected_progress_total(lf_runtime_session_mode(arm_value).unwrap());

        // A complete trace allows the terminal Done.
        let mut complete = probe.calibrator(profile.clone());
        complete.total_steps = complete.derive_expected_progress_total().unwrap();
        assert_eq!(complete.total_steps, expected_total, "{arm_value}");
        complete.current_step = expected_total;
        complete.mark_done().expect("a complete trace may finish");
        let emitted = complete.emitted_progress.lock().unwrap().clone();
        assert_eq!(
            emitted,
            vec![(
                format!("{label}: completed"),
                expected_total,
                CalibrationStatus::Done
            )],
            "{arm_value}: complete trace publishes exactly the terminal event"
        );

        // One missing operational increment refuses Done, and NO terminal event
        // is published.
        let mut skipped = probe.calibrator(profile.clone());
        skipped.total_steps = expected_total;
        skipped.current_step = expected_total - 1;
        let error = skipped
            .mark_done()
            .expect_err("a skipped operation must refuse Done")
            .to_string();
        assert!(
            error.contains("progress incomplete"),
            "{arm_value}: {error}"
        );
        let emitted = skipped.emitted_progress.lock().unwrap().clone();
        assert!(
            emitted.is_empty(),
            "{arm_value}: an incomplete trace publishes nothing"
        );
        assert!(
            !emitted
                .iter()
                .any(|(_, _, status)| *status == CalibrationStatus::Done),
            "{arm_value}: no terminal Done on an incomplete trace"
        );
    }
    probe.close().await;
}

/// The twelve RF/RH/LH Upper/Lower profiles the legacy wrapper still
/// recognizes, derived from the real profile table rather than hard-coded.
fn recognized_non_lf_legacy_labels() -> Vec<String> {
    let labels = all_profiles()
        .unwrap()
        .into_iter()
        .filter(|profile| profile.leg != Leg::Lf && profile.joint != JointKind::Hip)
        .map(|profile| profile.arm_value)
        .collect::<Vec<_>>();
    assert_eq!(labels.len(), 12);
    labels
}

#[tokio::test]
async fn recognized_non_lf_profiles_keep_their_preflight_and_verified_cleanup() {
    for label in recognized_non_lf_legacy_labels() {
        // 1. still recognized by the same legacy wrapper surface
        let profile = profile_for_arm_value(&label).unwrap();
        assert!(!is_lf_full_sequence(&profile));
        assert!(!is_lf_hip_sequence(&profile));
        assert!(hardware_profile_allowed(&profile).is_ok());
        // 2. no LF engine authority exists for it
        assert!(engine_authority_for_arm_value(&label).is_none(), "{label}");
        // the authority-free envelope equals the parent's historical value, so
        // the published progress envelope is unchanged for these profiles
        assert_eq!(
            single_contact_progress_envelope(&profile),
            16,
            "{label}: historical single-contact progress envelope"
        );

        let session = run_offline_session(&label).await;

        // 3./4. the historical preflight is published BEFORE the rejection
        assert!(!session.events.is_empty(), "{label}: preflight missing");
        assert_eq!(
            session.events[0],
            (
                format!("{label}: MATDOG native profile preflight"),
                0,
                CalibrationStatus::InProgress
            ),
            "{label}: exact historical preflight"
        );

        // 5. zero GoalPosition writes reached the bus
        assert_eq!(session.goal_writes, 0, "{label}: GoalPosition writes");

        // 6. verified global torque-OFF cleanup was actually issued on the bus
        assert!(
            session.verified_torque_off_frames >= 1,
            "{label}: verified cleanup frames = {}",
            session.verified_torque_off_frames
        );

        // 7. a Failed event follows, and 8. no Done event exists
        let failed = session
            .events
            .iter()
            .filter(|(_, _, status)| *status == CalibrationStatus::Failed)
            .count();
        assert_eq!(failed, 1, "{label}: exactly one Failed event");
        assert_eq!(
            session
                .events
                .iter()
                .filter(|(_, _, status)| *status == CalibrationStatus::Done)
                .count(),
            0,
            "{label}: no terminal Done"
        );
        assert_eq!(
            session.events.last().unwrap().2,
            CalibrationStatus::Failed,
            "{label}: Failed is the terminal event"
        );

        // 9. the wrapper returns Err
        assert!(session.outcome.is_err(), "{label}: must fail closed");
    }
}

#[test]
fn the_legacy_wrapper_keeps_its_preflight_execute_cleanup_ordering() {
    let source = include_str!("matdog.rs");
    let start = source
        .find("async fn execute_single_contact_session(")
        .expect("legacy wrapper");
    let end = source[start..]
        .find("\nasync fn run_lf_hip_min_max(")
        .map(|offset| start + offset)
        .expect("end of wrapper");
    let wrapper = &source[start..end];

    // The progress envelope needs no LF authority, so a rejected profile still
    // reaches the preflight.
    let envelope = wrapper
        .find("single_contact_progress_envelope(&calibrator.profile)")
        .expect("authority-free envelope");
    assert!(
        !wrapper.contains("calibrator.total_steps = calibrator.derive_expected_progress_total()?;"),
        "the legacy wrapper must not require LF authority before its preflight"
    );
    let preflight = wrapper
        .find("\"MATDOG native profile preflight\"")
        .expect("historical preflight");
    let execute = wrapper
        .find("calibrator.run().await")
        .expect("production single-contact executor");
    let cleanup = wrapper
        .find("calibrator\n        .global_torque_off_verified()")
        .or_else(|| wrapper.find("global_torque_off_verified()"))
        .expect("verified cleanup");
    let failure = wrapper
        .find("calibrator.mark_failed(&run_err);")
        .expect("failure publication");

    assert!(envelope < preflight, "envelope before preflight");
    assert!(preflight < execute, "preflight before execution");
    assert!(execute < cleanup, "execution before cleanup");
    assert!(cleanup < failure, "cleanup before failure publication");

    // Cleanup is unconditional: it is not inside any `if`, and it is bound
    // before the result is matched.
    let cleanup_line = wrapper[cleanup..].lines().next().unwrap();
    assert!(!cleanup_line.contains("if "));
    assert!(wrapper[..cleanup].contains("let result = calibrator.run().await"));
    assert!(wrapper.contains("match (result, cleanup)"));
}
