import importlib
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONTROLLER_DIR = PROJECT_ROOT / "webots/controllers/robot_controller"
sys.path.insert(0, str(CONTROLLER_DIR))

saved_config = sys.modules.pop("config", None)
try:
    YoubotAdapter = importlib.import_module(
        "adapters.youbot_adapter"
    ).YoubotAdapter
finally:
    if saved_config is not None:
        sys.modules["config"] = saved_config


class Motor:
    def __init__(self, name):
        self.name = name
        self.position = None
        self.velocity = None

    def getName(self):
        return self.name

    def setPosition(self, position):
        self.position = position

    def setVelocity(self, velocity):
        self.velocity = velocity


class Robot:
    def __init__(self, heading=0.0):
        self.position = [0.0, 0.0, 0.0]
        cosine = math.cos(heading)
        sine = math.sin(heading)
        self.orientation = [
            cosine, -sine, 0.0,
            sine, cosine, 0.0,
            0.0, 0.0, 1.0,
        ]
        self.motors = [
            Motor(f"wheel{index}")
            for index in range(1, 5)
        ]

    def getProtoField(self, name):
        if name == "numberOfArms":
            return type("IntField", (), {"getSFInt32": lambda _self: 0})()
        return None

    def getSelf(self):
        return self

    def getPosition(self):
        return self.position

    def getOrientation(self):
        return self.orientation

    def getNumberOfDevices(self):
        return len(self.motors)

    def getDeviceByIndex(self, index):
        return self.motors[index]


class NodeManager:
    def __init__(self, target):
        self.target = target

    def get_node(self, node_id):
        return self.target if node_id == "goal" else None


class YoubotNavigationTests(unittest.TestCase):
    def create_adapter(self, target, heading=0.0):
        robot = Robot(heading)
        adapter = YoubotAdapter(robot, NodeManager(target), "Youbot")
        return adapter, robot

    def assert_velocities(self, robot, expected):
        for motor, velocity in zip(robot.motors, expected):
            self.assertAlmostEqual(motor.velocity, velocity, places=6)

    def test_forward_motion_uses_all_four_wheels(self):
        adapter, robot = self.create_adapter({"x": 2.0, "y": 0.0})

        result = adapter.move_to_node("goal")

        self.assertEqual(result, {"reached": False, "error": False})
        self.assert_velocities(robot, [7.0, 7.0, 7.0, 7.0])

    def test_lateral_motion_uses_omnidirectional_wheel_pattern(self):
        adapter, robot = self.create_adapter({"x": 0.0, "y": 2.0})

        adapter.move_to_node("goal")

        self.assert_velocities(robot, [7.0, -7.0, -7.0, 7.0])

    def test_target_vector_is_transformed_to_robot_frame(self):
        adapter, robot = self.create_adapter(
            {"x": 2.0, "y": 0.0},
            heading=math.pi / 2,
        )

        adapter.move_to_node("goal")

        self.assert_velocities(robot, [-7.0, 7.0, 7.0, -7.0])

    def test_navigation_keeps_moving_after_small_progress(self):
        adapter, robot = self.create_adapter({"x": 2.0, "y": 0.0})

        first_result = adapter.move_to_node("goal")
        robot.position[0] = 0.1
        next_result = adapter.move_to_node("goal")

        self.assertFalse(first_result["reached"])
        self.assertFalse(next_result["reached"])
        self.assertTrue(all(motor.velocity > 0.0 for motor in robot.motors))

    def test_arrival_and_stop_zero_all_wheel_motors(self):
        adapter, robot = self.create_adapter({"x": 0.05, "y": 0.0})

        result = adapter.move_to_node("goal")

        self.assertTrue(result["reached"])
        self.assert_velocities(robot, [0.0, 0.0, 0.0, 0.0])

    def test_transport_is_unverified_by_default_and_simulation_only_when_enabled(self):
        import adapters.youbot_adapter as youbot_module

        adapter, _robot = self.create_adapter({"x": 2.0, "y": 0.0})
        self.assertEqual(adapter.transport_capability["status"], "UNVERIFIED")

        with patch.object(
            youbot_module,
            "WEBOTS_SIMULATION_MODE",
            True,
        ):
            capability = adapter.transport_capability

        self.assertEqual(capability["status"], "SIMULATION_ONLY")
        self.assertEqual(capability["mode"], "SIMULATION_ONLY")
        self.assertEqual(capability["maxPayloadKg"], 0.5)
        self.assertIn("not a physical rating", capability["reason"])


if __name__ == "__main__":
    unittest.main()
