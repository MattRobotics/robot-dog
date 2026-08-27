from __future__ import annotations

import argparse
import json
from pathlib import Path
import tempfile
import unittest

from tools.matdog import matdog_lf_profile as profile


def machine_record(
    *,
    joint: str,
    joint_name: str,
    motor_id: int,
    direction: int,
    urdf_min_delta: int,
    urdf_max_delta: int,
    urdf_min_tick: int,
    urdf_max_tick: int,
    q0: int = 2048,
    q0_affine: int | None = None,
    accepted: bool = True,
) -> str:
    values = {
        "joint": joint,
        "joint_name": joint_name,
        "motor_id": motor_id,
        "direction": direction,
        "urdf_min_delta": urdf_min_delta,
        "urdf_max_delta": urdf_max_delta,
        "urdf_min_tick": urdf_min_tick,
        "urdf_max_tick": urdf_max_tick,
        "coarse_min": urdf_min_tick,
        "coarse_max": urdf_max_tick,
        "fine_min_1": urdf_min_tick,
        "fine_min_2": urdf_min_tick + 1,
        "fine_max_1": urdf_max_tick,
        "fine_max_2": urdf_max_tick + 1,
        "repeatability_min": 1,
        "repeatability_max": 1,
        "contact_min": urdf_min_tick,
        "contact_max": urdf_max_tick,
        "q0_fixed": q0,
        "q0_affine": q0 if q0_affine is None else q0_affine,
        "endpoint_disagreement": 1,
        "q0_shift": abs(q0 - 2048),
        "scale_permille": 1000,
        "safe_min_tick": min(urdf_min_tick, urdf_max_tick),
        "safe_max_tick": max(urdf_min_tick, urdf_max_tick),
        "accepted": str(accepted).lower(),
    }
    return profile.PREFIX + "|".join(f"{key}={value}" for key, value in values.items())


class MatdogLfProfileTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.urdf = self.root / "matdog.urdf"
        self.urdf.write_text(
            """<robot name="matdog">
<joint name="lf_hip_joint" type="revolute"><limit lower="-0.7853981633974483" upper="0.7853981633974483" effort="1" velocity="1"/></joint>
<joint name="lf_upper_leg_joint" type="revolute"><limit lower="-0.915864323" upper="2.138181" effort="1" velocity="1"/></joint>
<joint name="lf_lower_leg_joint" type="revolute"><limit lower="-1.606318" upper="0.654913" effort="1" velocity="1"/></joint>
</robot>\n""",
            encoding="utf-8",
        )
        self.log = self.root / "station.log"
        self.log.write_text(
            "\n".join(
                (
                    machine_record(
                        joint="HIP",
                        joint_name="lf_hip_joint",
                        motor_id=13,
                        direction=-1,
                        urdf_min_delta=-512,
                        urdf_max_delta=512,
                        urdf_min_tick=2560,
                        urdf_max_tick=1536,
                    ),
                    machine_record(
                        joint="UPPER",
                        joint_name="lf_upper_leg_joint",
                        motor_id=12,
                        direction=1,
                        urdf_min_delta=-597,
                        urdf_max_delta=1394,
                        urdf_min_tick=1451,
                        urdf_max_tick=3442,
                    ),
                    machine_record(
                        joint="LOWER",
                        joint_name="lf_lower_leg_joint",
                        motor_id=11,
                        direction=-1,
                        urdf_min_delta=-1047,
                        urdf_max_delta=427,
                        urdf_min_tick=3095,
                        urdf_max_tick=1621,
                        q0=2040,
                        q0_affine=2055,
                    ),
                )
            )
            + "\n",
            encoding="utf-8",
        )

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def test_stage_and_finalize_profile(self) -> None:
        stage_json = self.root / "stage.json"
        stage_plan = self.root / "stage.env"
        profile.stage_profile(
            argparse.Namespace(
                station_log=str(self.log),
                urdf=str(self.urdf),
                bus_serial=profile.BUS_SERIAL_DEFAULT,
                calibrator_commit="abc123",
                output_json=str(stage_json),
                output_plan=str(stage_plan),
            )
        )
        staged = json.loads(stage_json.read_text(encoding="utf-8"))
        self.assertEqual(staged["status"], "LF_STAGED")
        self.assertEqual(set(staged["motors"]), {"11", "12", "13"})
        plan_text = stage_plan.read_text(encoding="utf-8")
        self.assertIn("motor.11.estimated_q0_tick=2055", plan_text)
        self.assertNotIn("motor.11.estimated_q0_tick=2040", plan_text)

        freeze = self.root / "freeze.env"
        lines = [
            "format=MATDOG_LF_FREEZE_RESULT_V1",
            "status=LF_FROZEN",
            f"bus_serial={profile.BUS_SERIAL_DEFAULT}",
            f"urdf_sha256={staged['urdf']['sha256']}",
        ]
        for motor_id in (11, 12, 13):
            lines.extend(
                (
                    f"motor.{motor_id}.joint_name={profile.JOINTS[motor_id]}",
                    f"motor.{motor_id}.old_offset=0",
                    f"motor.{motor_id}.new_offset=0",
                    f"motor.{motor_id}.position=2048",
                    f"motor.{motor_id}.lock=1",
                    f"motor.{motor_id}.torque_enabled=0",
                )
            )
        freeze.write_text("\n".join(lines) + "\n", encoding="utf-8")
        backup = self.root / "backup.json"
        backup.write_text("{}\n", encoding="utf-8")
        frozen = self.root / "lf-frozen.json"
        global_index = self.root / "global.json"
        profile.finalize_profile(
            argparse.Namespace(
                stage_json=str(stage_json),
                freeze_result=str(freeze),
                backup=str(backup),
                output_json=str(frozen),
                global_index=str(global_index),
            )
        )
        self.assertEqual(json.loads(frozen.read_text())["status"], "LF_FROZEN")
        self.assertEqual(json.loads(global_index.read_text())["frozen_legs"], 1)

    def test_rejected_joint_cannot_be_staged(self) -> None:
        line = machine_record(
            joint="HIP",
            joint_name="lf_hip_joint",
            motor_id=13,
            direction=-1,
            urdf_min_delta=-512,
            urdf_max_delta=512,
            urdf_min_tick=2560,
            urdf_max_tick=1536,
            accepted=False,
        )
        with self.assertRaises(profile.ProfileError):
            profile.parse_record(line)


if __name__ == "__main__":
    unittest.main()
