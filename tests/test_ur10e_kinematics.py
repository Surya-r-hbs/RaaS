import importlib.util
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONTROLLER_DIR = PROJECT_ROOT / "webots/controllers/robot_controller"
KINEMATICS_PATH = (
    CONTROLLER_DIR / "adapters/ur10e_kinematics.py"
)
SPEC = importlib.util.spec_from_file_location(
    "ur10e_kinematics",
    KINEMATICS_PATH,
)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"Cannot load UR10e kinematics from {KINEMATICS_PATH}")
KINEMATICS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(KINEMATICS)
forward_kinematics = KINEMATICS.forward_kinematics
inverse_kinematics = KINEMATICS.inverse_kinematics


class UR10eKinematicsTests(unittest.TestCase):
    def assert_pose_close(self, actual, expected):
        for row in range(3):
            self.assertAlmostEqual(
                actual[row][3],
                expected[row][3],
                delta=0.005,
            )
            for column in range(3):
                self.assertAlmostEqual(
                    actual[row][column],
                    expected[row][column],
                    delta=0.035,
                )

    def test_inverse_kinematics_recovers_reachable_pose_from_home(self):
        expected_joints = [0.2, -1.0, 1.2, -1.0, -1.2, 0.3]
        target = forward_kinematics(expected_joints)

        actual_joints = inverse_kinematics(target, [0.0] * 6)

        self.assert_pose_close(forward_kinematics(actual_joints), target)

    def test_inverse_kinematics_uses_bent_fallback_seed_from_zero_pose(self):
        root_position = (3.21, -4.06, 0.74)
        root_rotation = (
            (0.0, -1.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0),
        )
        world_targets = (
            (3.94, -4.73, 1.18),
            (3.94, -4.73, 1.06),
            (3.94, -4.73, 1.18),
            (4.0, -4.3, 1.18),
            (4.0, -4.3, 0.74),
            (4.0, -4.3, 0.86),
        )
        world_down = (
            (1.0, 0.0, 0.0),
            (0.0, -1.0, 0.0),
            (0.0, 0.0, -1.0),
        )
        base_down = tuple(
            tuple(
                sum(
                    root_rotation[axis][row] * world_down[axis][column]
                    for axis in range(3)
                )
                for column in range(3)
            )
            for row in range(3)
        )
        seed = [0.0] * 6
        for world_target in world_targets:
            difference = tuple(
                world_target[index] - root_position[index]
                for index in range(3)
            )
            base_target = (
                sum(
                    root_rotation[column][0] * difference[column]
                    for column in range(3)
                ),
                sum(
                    root_rotation[column][1] * difference[column]
                    for column in range(3)
                ),
                difference[2],
            )
            target = [
                list(base_down[row]) + [base_target[row]]
                for row in range(3)
            ] + [[0.0, 0.0, 0.0, 1.0]]

            seed = inverse_kinematics(target, seed)

            self.assert_pose_close(forward_kinematics(seed), target)

    def test_inverse_kinematics_rejects_unreachable_pose(self):
        target = [
            [1.0, 0.0, 0.0, 3.0],
            [0.0, 1.0, 0.0, 3.0],
            [0.0, 0.0, 1.0, 3.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

        with self.assertRaisesRegex(ValueError, "No UR10e solution"):
            inverse_kinematics(target, [0.0] * 6)

    def test_inverse_kinematics_requires_six_joint_values(self):
        with self.assertRaisesRegex(ValueError, "six seed joint angles"):
            inverse_kinematics(forward_kinematics([0.0] * 6), [0.0] * 5)


if __name__ == "__main__":
    unittest.main()
