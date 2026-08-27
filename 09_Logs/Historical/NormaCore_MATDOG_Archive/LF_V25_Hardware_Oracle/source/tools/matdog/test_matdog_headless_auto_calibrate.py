from __future__ import annotations

import argparse
import asyncio
import signal
import unittest

from tools.matdog import matdog_headless_auto_calibrate as runner


APP_START_ID = 42


def make_motor(
    motor_id: int,
    stamp_ns: int,
    *,
    temperature_c: int = 40,
    torque_enabled: bool = False,
) -> runner.MotorSample:
    rx_pointer = int(stamp_ns & 0xFFFFFFFF).to_bytes(4, "little")
    return runner.MotorSample(
        motor_id=motor_id,
        monotonic_stamp_ns=stamp_ns,
        system_stamp_ns=1_800_000_000_000_000_000 + stamp_ns,
        app_start_id=APP_START_ID,
        state_length=runner.MIN_STATE_LENGTH,
        rx_pointer_hex=rx_pointer.hex(),
        rx_pointer_decimal=int.from_bytes(rx_pointer, "little"),
        raw_ram_0x28_0x46_hex="00" * (
            runner.MIN_STATE_LENGTH - runner.TORQUE_ENABLE
        ),
        position=2048,
        speed_raw=0,
        current_raw=0,
        voltage_raw=111,
        voltage_v=11.1,
        temperature_c=temperature_c,
        temperature_limit_c=70,
        status=0,
        torque_raw=int(torque_enabled),
        torque_enabled=torque_enabled,
        goal_position=2048,
        torque_limit=0,
        driver_error_present=False,
    )


def make_frame(
    stamps: dict[int, int] | int,
    *,
    temperatures: dict[int, int] | None = None,
) -> runner.FrameSample:
    if isinstance(stamps, int):
        stamps = {motor_id: stamps for motor_id in runner.EXPECTED_MOTOR_IDS}
    temperatures = temperatures or {}
    motors = {
        motor_id: make_motor(
            motor_id,
            stamps[motor_id],
            temperature_c=temperatures.get(motor_id, 40),
        )
        for motor_id in runner.EXPECTED_MOTOR_IDS
    }
    return runner.FrameSample(
        bus_serial=runner.EXPECTED_BUS_SERIAL,
        bus_monotonic_stamp_ns=max(stamps.values()),
        bus_system_stamp_ns=1_800_000_000_000_000_000 + max(stamps.values()),
        bus_app_start_id=APP_START_ID,
        calibration=runner.CalibrationSample(
            status=int(runner.st3215.AutoCalibrationState_Status.IDLE),
            status_name="IDLE",
            current_step=0,
            total_steps=0,
            phase="",
            error_message="",
        ),
        motors=motors,
    )


def observe(
    collector: runner.LatestSamplePreflight,
    frame: runner.FrameSample,
    now_ns: int,
    sequence: int,
) -> None:
    collector.observe_frame(
        frame,
        inference_queue_entry_id_hex=sequence.to_bytes(4, "little").hex(),
        received_boottime_ns=now_ns,
        received_utc="2026-08-01T00:00:00+00:00",
    )


class BacklogConcurrencyTests(unittest.TestCase):
    def test_thousands_of_old_frames_are_superseded_by_source_latest(self) -> None:
        now_ns = 20_000_000_000
        telemetry = runner.ThermalSeriesRecorder()
        collector = runner.LatestSamplePreflight(telemetry)
        start_stamp = now_ns - 4_000_000_000

        for sequence in range(3_001):
            stamp = start_stamp + sequence * 1_300_000
            observe(collector, make_frame(stamp), now_ns, sequence)

        final_stamps = {
            motor_id: now_ns - 10_000_000
            for motor_id in runner.EXPECTED_MOTOR_IDS
        }
        final_stamps[43] = now_ns - 17_000_000
        observe(collector, make_frame(final_stamps), now_ns, 3_002)
        window = collector.validate_window(now_ns)

        self.assertEqual(
            collector.latest_by_motor[43].monotonic_stamp_ns,
            now_ns - 17_000_000,
        )
        self.assertEqual(window["latest_source_age_ns"]["43"], 17_000_000)
        self.assertGreater(
            collector.max_observed_backlog_ns[43],
            runner.MAX_TELEMETRY_AGE_NS,
        )
        # Evidence is deliberately decimated: every source frame still
        # reaches the latest-sample safety contract, while the persisted M11
        # series records phase changes and at most four samples per second.
        self.assertGreaterEqual(len(telemetry.m11_records), 15)
        self.assertLessEqual(len(telemetry.m11_records), 20)

    def test_slow_deferred_writer_keeps_latest_gate_live(self) -> None:
        async def scenario() -> None:
            telemetry = runner.ThermalSeriesRecorder()
            collector = runner.LatestSamplePreflight(telemetry)
            initial_stamp = 30_000_000_000
            observe(
                collector,
                make_frame(initial_stamp),
                initial_stamp + 10_000_000,
                0,
            )
            collector.validate_window(initial_stamp + 10_000_000)

            writer_release = asyncio.Event()

            async def slow_writer() -> str:
                await writer_release.wait()
                return "persisted"

            persistence_task = asyncio.create_task(slow_writer())
            calls = 0
            last_stamp = initial_stamp

            async def observe_window() -> None:
                nonlocal calls, last_stamp
                calls += 1
                if calls == 1:
                    gate_now = last_stamp + 4_100_000_000
                    for index in range(1, 1_001):
                        stamp = last_stamp + index * 4_000_000
                        observe(
                            collector,
                            make_frame(stamp),
                            gate_now,
                            index,
                        )
                    last_stamp += 4_000_000_000
                else:
                    gate_now = last_stamp + 200_000_000
                    for index in range(1, 11):
                        stamp = last_stamp + index * 10_000_000
                        observe(
                            collector,
                            make_frame(stamp),
                            gate_now,
                            2_000 + index,
                        )
                    last_stamp += 100_000_000
                    writer_release.set()
                collector.validate_window(gate_now)
                await asyncio.sleep(0)

            result, observed_windows = (
                await runner.finish_deferred_write_while_observing(
                    persistence_task,
                    observe_window,
                )
            )
            self.assertEqual(result, "persisted")
            self.assertGreaterEqual(observed_windows, 2)
            self.assertGreater(
                collector.max_observed_backlog_ns[43],
                runner.MAX_TELEMETRY_AGE_NS,
            )
            self.assertLessEqual(
                collector.max_latest_source_age_ns[43],
                100_000_000,
            )

        asyncio.run(scenario())


class LatestSampleContractTests(unittest.TestCase):
    def test_interleaved_updates_require_every_motor_to_advance(self) -> None:
        telemetry = runner.ThermalSeriesRecorder()
        collector = runner.LatestSamplePreflight(telemetry)
        base_stamp = 40_000_000_000
        stamps = {
            motor_id: base_stamp for motor_id in runner.EXPECTED_MOTOR_IDS
        }
        observe(collector, make_frame(stamps), base_stamp + 1_000_000, 0)
        collector.validate_window(base_stamp + 1_000_000)

        for sequence, motor_id in enumerate(runner.EXPECTED_MOTOR_IDS, start=1):
            stamps = dict(stamps)
            stamps[motor_id] += 10_000_000
            observe(
                collector,
                make_frame(stamps),
                base_stamp + 20_000_000,
                sequence,
            )
            if motor_id != runner.EXPECTED_MOTOR_IDS[-1]:
                self.assertFalse(collector.window_ready())

        self.assertTrue(collector.window_ready())
        collector.validate_window(base_stamp + 20_000_000)
        self.assertTrue(
            all(count == 2 for count in collector.unique_samples.values())
        )

    def test_m43_without_new_source_sample_for_over_three_seconds_fails(self) -> None:
        telemetry = runner.ThermalSeriesRecorder()
        collector = runner.LatestSamplePreflight(telemetry)
        first_now = 50_000_000_000
        first_stamp = first_now - 10_000_000
        observe(collector, make_frame(first_stamp), first_now, 0)
        collector.validate_window(first_now)

        second_now = first_now + 3_100_000_000
        stamps = {
            motor_id: second_now - 10_000_000
            for motor_id in runner.EXPECTED_MOTOR_IDS
        }
        stamps[43] = first_stamp
        observe(collector, make_frame(stamps), second_now, 1)
        with self.assertRaisesRegex(
            runner.RunnerError,
            r"M43 latest telemetry is not fresh",
        ):
            collector.validate_window(second_now)

    def test_thermal_spike_cannot_be_hidden_by_a_newer_sample(self) -> None:
        telemetry = runner.ThermalSeriesRecorder()
        collector = runner.LatestSamplePreflight(telemetry)
        now_ns = 60_000_000_000
        with self.assertRaisesRegex(
            runner.RunnerError,
            r"M11 preflight temperature 65 C",
        ):
            observe(
                collector,
                make_frame(
                    now_ns - 20_000_000,
                    temperatures={11: 65},
                ),
                now_ns,
                0,
            )
        self.assertEqual(telemetry.m11_records[-1]["temperature_c"], 65)
        self.assertEqual(len(telemetry.anomalies), 1)

    def test_consecutive_thermal_jump_over_five_degrees_fails(self) -> None:
        telemetry = runner.ThermalSeriesRecorder()
        collector = runner.LatestSamplePreflight(telemetry)
        now_ns = 70_000_000_000
        observe(collector, make_frame(now_ns - 20_000_000), now_ns, 0)
        collector.validate_window(now_ns)
        with self.assertRaisesRegex(
            runner.RunnerError,
            r"M11 consecutive temperature jump 6 C",
        ):
            observe(
                collector,
                make_frame(
                    now_ns + 10_000_000,
                    temperatures={11: 46},
                ),
                now_ns + 20_000_000,
                1,
            )


class RuntimeLatestHeadTests(unittest.TestCase):
    @staticmethod
    def make_run() -> runner.HeadlessRun:
        class Evidence:
            io_errors: list[str] = []

            def emit(self, _event: str, **_payload: object) -> None:
                return

        run = runner.HeadlessRun(
            argparse.Namespace(),
            Evidence(),  # type: ignore[arg-type]
        )
        run.stream_errors = asyncio.Queue()
        run._parse_stream_entry = (  # type: ignore[method-assign]
            lambda entry, *, phase: entry
        )
        return run

    def test_runtime_backlog_is_drained_before_freshness_validation(self) -> None:
        async def scenario() -> None:
            run = self.make_run()
            now_ns = runner.station_monotonic_stamp_ns()
            old_start = now_ns - 4_000_000_000

            for sequence in range(1_000):
                run.entries.put_nowait(
                    make_frame(old_start + sequence * 1_000_000)
                )
            latest_stamp = now_ns - 10_000_000
            latest_frame = make_frame(latest_stamp)
            run.entries.put_nowait(latest_frame)

            run.contract.initial_goals = {
                motor_id: sample.goal_position
                for motor_id, sample in latest_frame.motors.items()
            }
            run.contract.initial_positions = {
                motor_id: sample.position
                for motor_id, sample in latest_frame.motors.items()
            }

            observed = await run.next_frame(
                1.0, phase="during_calibration"
            )
            self.assertEqual(
                observed.motors[43].monotonic_stamp_ns,
                latest_stamp,
            )
            run.contract.validate_running(observed)
            # LatestOnlyQueue discards superseded entries at ingestion,
            # so next_frame consumes exactly the current head, not a backlog.
            self.assertEqual(run.max_frames_per_head_drain, 1)

        asyncio.run(scenario())

    def test_superseded_bulk_temperature_is_replaced_by_confirmed_latest(self) -> None:
        async def scenario() -> None:
            run = self.make_run()
            now_ns = runner.station_monotonic_stamp_ns()
            run.entries.put_nowait(
                make_frame(
                    now_ns - 20_000_000,
                    temperatures={12: 73},
                )
            )
            latest = make_frame(now_ns - 10_000_000)
            run.entries.put_nowait(latest)

            observed = await run.next_frame(
                1.0, phase="during_calibration"
            )
            self.assertEqual(
                observed.motors[12].temperature_c,
                latest.motors[12].temperature_c,
            )
            self.assertEqual(run.max_frames_per_head_drain, 1)

            # A confirmed current over-temperature is still rejected. The
            # Rust driver produces that confirmed current value from dedicated
            # 0x3F reads; the Python runner validates only the latest frame.
            confirmed = make_frame(
                now_ns - 5_000_000,
                temperatures={12: 73},
            )
            with self.assertRaisesRegex(
                runner.RunnerError,
                r"M12 temperature 73 C",
            ):
                run.contract.validate_payload(confirmed)

        asyncio.run(scenario())

    def test_runtime_requires_exact_70c_configured_limit(self) -> None:
        run = self.make_run()
        now_ns = runner.station_monotonic_stamp_ns()
        frame = make_frame(now_ns - 10_000_000)
        motor = frame.motors[12]
        frame.motors[12] = runner.MotorSample(
            **{
                **runner.asdict(motor),
                "temperature_limit_c": 71,
            }
        )
        with self.assertRaisesRegex(
            runner.RunnerError,
            r"M12 configured temperature limit changed",
        ):
            run.contract.validate_payload(frame)


class ShutdownTests(unittest.TestCase):
    def test_sigint_graceful_shutdown_has_no_sigkill(self) -> None:
        async def scenario() -> None:
            identity = runner.ProcessIdentity(123, "/station", 456, "S")
            signals: list[signal.Signals] = []
            live_checks = 0

            def is_live(_identity: runner.ProcessIdentity) -> bool:
                nonlocal live_checks
                live_checks += 1
                return live_checks < 3

            def send(_pid: int, sent_signal: signal.Signals) -> None:
                signals.append(sent_signal)

            result = await runner.controlled_station_shutdown(
                identity,
                timeout_s=0.1,
                poll_interval_s=0,
                signal_sender=send,
                identity_is_live=is_live,
            )
            self.assertTrue(result["stopped"])
            self.assertTrue(result["graceful"])
            self.assertEqual(signals, [signal.SIGINT])
            self.assertFalse(result["sigkill_sent"])

        asyncio.run(scenario())

    def test_sigkill_is_only_used_after_sigint_timeout(self) -> None:
        async def scenario() -> None:
            identity = runner.ProcessIdentity(123, "/station", 456, "S")
            live = True
            signals: list[signal.Signals] = []

            def is_live(_identity: runner.ProcessIdentity) -> bool:
                return live

            def send(_pid: int, sent_signal: signal.Signals) -> None:
                nonlocal live
                signals.append(sent_signal)
                if sent_signal == signal.SIGKILL:
                    live = False

            result = await runner.controlled_station_shutdown(
                identity,
                timeout_s=0.001,
                poll_interval_s=0,
                signal_sender=send,
                identity_is_live=is_live,
            )
            self.assertTrue(result["stopped"])
            self.assertFalse(result["graceful"])
            self.assertEqual(signals, [signal.SIGINT, signal.SIGKILL])
            self.assertTrue(result["sigkill_sent"])

        asyncio.run(scenario())


if __name__ == "__main__":
    unittest.main()
