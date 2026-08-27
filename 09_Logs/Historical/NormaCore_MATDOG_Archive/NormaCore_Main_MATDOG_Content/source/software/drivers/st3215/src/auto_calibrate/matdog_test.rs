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
        .find("    async fn approach(")
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
    let rehome_move = run[rehome_prepare..]
        .find("STATIC_TOLERANCE_TICKS")
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

fn valid_lf_sessions_by_state() -> Vec<LfSessionStateMachine> {
    let mut sessions = Vec::new();
    let mut session = LfSessionStateMachine::new(lf_entry_positions()).unwrap();
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

fn state_for_lf_session(session: &LfSessionStateMachine, now_ns: u64) -> InferenceState {
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
    assert!(
        body.contains("let coarse_scout_tick = self.approach(COARSE_STEP_TICKS, baseline).await?;")
    );
    assert_eq!(
        body.matches("approach_with_scout(FINE_STEP_TICKS, baseline, Some(coarse_scout_tick))")
            .count(),
        2
    );
    assert!(body.contains("repeatability_spread(first_tick, second_tick)"));
    assert!(!body.contains("repeatability_spread(coarse_scout_tick"));
    assert!(body.contains("backoff_and_verify(first_tick, baseline)"));
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
    let mut session = LfSessionStateMachine::new(lf_entry_positions()).unwrap();

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
    let mut session = LfSessionStateMachine::new(lf_entry_positions()).unwrap();
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
    let mut session = LfSessionStateMachine::new(lf_entry_positions()).unwrap();
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
        .find("async fn set_startup_home_goal_verified")
        .map(|offset| start + offset)
        .expect("end of q0 startup helpers");
    let startup = &source[start..end];

    assert!(!startup.contains("STARTUP_HOME_RECOVERY_LIMIT_TICKS"));
    assert!(!startup.contains("outside the uniform startup-home recovery window"));
    assert!(!startup.contains("set_startup_home_goal_verified(motor_id, initial.position)"));
    assert!(startup.contains("set_startup_home_goal_verified(motor_id, HOME_TICK)"));
    assert!(startup.contains("startup_home_initial_position_valid(initial.position)"));
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
