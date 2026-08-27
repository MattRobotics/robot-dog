from __future__ import annotations

from dataclasses import replace
import unittest

from tools.matdog import matdog_headless_auto_calibrate as runner
from tools.matdog import matdog_native_observer_contract as observer_contract


APP_START_ID = 1234


def motor(
    motor_id: int,
    *,
    position: int = 2048,
    goal: int = 2048,
    torque: bool = False,
    torque_limit: int = 0,
    speed: int = 0,
    current: int = 0,
    temperature: int = 30,
    temperature_limit: int = 70,
    status: int = 0,
    driver_error: bool = False,
) -> runner.MotorSample:
    stamp = runner.station_monotonic_stamp_ns()
    return runner.MotorSample(
        motor_id=motor_id,
        monotonic_stamp_ns=stamp,
        system_stamp_ns=stamp,
        app_start_id=APP_START_ID,
        state_length=runner.MIN_STATE_LENGTH,
        rx_pointer_hex="01",
        rx_pointer_decimal=1,
        raw_ram_0x28_0x46_hex="00" * (
            runner.MIN_STATE_LENGTH - runner.TORQUE_ENABLE
        ),
        position=position,
        speed_raw=speed,
        current_raw=current,
        voltage_raw=111,
        voltage_v=11.1,
        temperature_c=temperature,
        temperature_limit_c=temperature_limit,
        status=status,
        torque_raw=int(torque),
        torque_enabled=torque,
        goal_position=goal,
        torque_limit=torque_limit,
        driver_error_present=driver_error,
    )


def samples() -> dict[int, runner.MotorSample]:
    return {
        motor_id: motor(motor_id)
        for motor_id in runner.EXPECTED_MOTOR_IDS
    }


def frame(
    motors: dict[int, runner.MotorSample] | None = None,
    *,
    phase: str = "LF_LEG_STATE_MACHINE: runtime",
    step: int = 1,
    status: int = 1,
    status_name: str = "IN_PROGRESS",
    app_start_id: int = APP_START_ID,
) -> runner.FrameSample:
    return runner.FrameSample(
        bus_serial=runner.EXPECTED_BUS_SERIAL,
        bus_monotonic_stamp_ns=runner.station_monotonic_stamp_ns(),
        bus_system_stamp_ns=1,
        bus_app_start_id=app_start_id,
        calibration=runner.CalibrationSample(
            status=status,
            status_name=status_name,
            current_step=step,
            total_steps=runner.EXPECTED_FULL_TOTAL_STEPS if status else 0,
            phase=phase,
            error_message="",
        ),
        motors=motors or samples(),
    )


def initialized_contract():
    contract_class = observer_contract.build_native_authority_contract(runner)
    contract = contract_class()
    contract.validate_preflight(
        frame(
            phase="",
            step=0,
            status=0,
            status_name="IDLE",
        )
    )
    return contract


class NativeAuthorityObserverContractTests(unittest.TestCase):
    def test_prime_then_operational_target_is_general_for_every_lf_participant(self) -> None:
        transitions = {
            42: (2047, 2389),
            12: (2052, 3506),
            11: (2047, 3159),
            13: (2048, 1472),
        }

        for motor_id, (prime_tick, operational_tick) in transitions.items():
            with self.subTest(motor_id=motor_id):
                contract = initialized_contract()
                primed = samples()
                primed[motor_id] = motor(
                    motor_id,
                    position=prime_tick,
                    goal=prime_tick,
                    torque=True,
                    torque_limit=runner.EXPECTED_ACTIVE_TORQUE_LIMIT,
                )
                contract.validate_running(
                    frame(
                        primed,
                        phase=f"arbitrary native phase for M{motor_id}",
                        step=17,
                    )
                )

                retargeted = dict(primed)
                retargeted[motor_id] = motor(
                    motor_id,
                    position=prime_tick,
                    goal=operational_tick,
                    torque=True,
                    torque_limit=runner.EXPECTED_ACTIVE_TORQUE_LIMIT,
                )
                contract.validate_running(
                    frame(
                        retargeted,
                        phase=f"different native phase for M{motor_id}",
                        step=18,
                    )
                )

    def test_observer_does_not_reconstruct_joint_roles_from_phase_or_step(self) -> None:
        contract = initialized_contract()
        observed = samples()
        observed[12] = motor(
            12,
            position=2050,
            goal=2050,
            torque=True,
            torque_limit=runner.EXPECTED_ACTIVE_TORQUE_LIMIT,
        )
        observed[42] = motor(
            42,
            position=2389,
            goal=2389,
            torque=True,
            torque_limit=runner.EXPECTED_ACTIVE_TORQUE_LIMIT,
        )
        contract.validate_running(
            frame(
                observed,
                phase="text deliberately unrelated to joint choreography",
                step=57,
            )
        )

    def test_hard_current_remains_fail_closed(self) -> None:
        contract = initialized_contract()
        observed = samples()
        observed[11] = motor(11, current=runner.MAX_SAFE_CURRENT_RAW + 1)
        with self.assertRaisesRegex(runner.RunnerError, "M11 current"):
            contract.validate_running(frame(observed))

    def test_servo_status_remains_fail_closed(self) -> None:
        contract = initialized_contract()
        observed = samples()
        observed[12] = motor(12, status=1)
        with self.assertRaisesRegex(runner.RunnerError, "M12 servo status"):
            contract.validate_running(frame(observed))

    def test_driver_error_remains_fail_closed(self) -> None:
        contract = initialized_contract()
        observed = samples()
        observed[13] = motor(13, driver_error=True)
        with self.assertRaisesRegex(runner.RunnerError, "M13 driver error"):
            contract.validate_running(frame(observed))

    def test_temperature_contract_remains_fail_closed(self) -> None:
        contract = initialized_contract()
        wrong_limit = samples()
        wrong_limit[21] = motor(21, temperature_limit=71)
        with self.assertRaisesRegex(
            runner.RunnerError,
            "M21 configured temperature limit changed",
        ):
            contract.validate_running(frame(wrong_limit))

        over_limit = samples()
        over_limit[22] = motor(22, temperature=71)
        with self.assertRaisesRegex(runner.RunnerError, "M22 temperature"):
            contract.validate_running(frame(over_limit))

    def test_station_restart_remains_fail_closed(self) -> None:
        contract = initialized_contract()
        with self.assertRaisesRegex(runner.RunnerError, "Station restart detected"):
            contract.validate_running(frame(app_start_id=APP_START_ID + 1))

    def test_final_global_torque_off_is_still_verified(self) -> None:
        contract = initialized_contract()
        observed = samples()
        observed[42] = replace(
            observed[42],
            torque_raw=1,
            torque_enabled=True,
            torque_limit=runner.EXPECTED_ACTIVE_TORQUE_LIMIT,
        )
        with self.assertRaisesRegex(runner.RunnerError, "global torque-OFF"):
            contract.validate_torque_off(frame(observed))


if __name__ == "__main__":
    unittest.main()
