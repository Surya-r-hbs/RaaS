import importlib
import math
import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONTROLLER_DIR = PROJECT_ROOT / "webots/controllers/robot_controller"
sys.path.insert(0, str(CONTROLLER_DIR))

saved_config = sys.modules.pop("config", None)
try:
    Pioneer3atAdapter = importlib.import_module(
        "adapters.pioneer3at_adapter"
    ).Pioneer3atAdapter
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


class Sonar:
    def __init__(self, name, value):
        self.name = name
        self.value = value
        self.enabled = None

    def getName(self):
        return self.name

    def enable(self, time_step):
        self.enabled = time_step

    def getValue(self):
        return self.value


class Robot:
    def __init__(self, position=(0.0, 0.0, 0.0), heading=math.pi / 2):
        self.position = list(position)
        cosine = math.cos(heading)
        sine = math.sin(heading)
        self.orientation = [
            cosine, -sine, 0.0,
            sine, cosine, 0.0,
            0.0, 0.0, 1.0,
        ]
        self.devices = [
            Motor("back left wheel"),
            Motor("front left wheel"),
            Motor("back right wheel"),
            Motor("front right wheel"),
            Sonar("so", 0.0),
        ]

    def getSelf(self):
        return self

    def getPosition(self):
        return self.position

    def getOrientation(self):
        return self.orientation

    def getNumberOfDevices(self):
        return len(self.devices)

    def getDeviceByIndex(self, index):
        return self.devices[index]


class NodeManager:
    def __init__(self, target):
        self.target = target

    def get_node(self, node_id):
        return self.target if node_id == "goal" else None


class Pioneer3atAdapterTests(unittest.TestCase):
    def create_adapter(self, target, position=(0.0, 0.0, 0.0)):
        robot = Robot(position=position)
        adapter = Pioneer3atAdapter(robot, NodeManager(target), "Pioneer3at")
        return adapter, robot

    def test_initializes_four_wheels_and_enables_declared_forward_sonar(self):
        adapter, robot = self.create_adapter({"x": 0.0, "y": 2.0})

        self.assertEqual(adapter.capability_names, ["NAVIGATION"])
        for motor in robot.devices[:4]:
            self.assertEqual(motor.position, float("inf"))
            self.assertEqual(motor.velocity, 0.0)
        self.assertIsNotNone(robot.devices[4].enabled)

    def test_navigates_forward_in_xy_using_pioneer_forward_axis(self):
        adapter, robot = self.create_adapter({"x": 0.0, "y": 2.0})

        result = adapter.move_to_node("goal")

        self.assertEqual(result, {"reached": False, "error": False})
        self.assertEqual(
            [motor.velocity for motor in robot.devices[:4]],
            [1.0, 1.0, 1.0, 1.0],
        )

    def test_turns_toward_target_before_driving(self):
        adapter, robot = self.create_adapter({"x": 2.0, "y": 0.0})

        adapter.move_to_node("goal")

        self.assertGreater(robot.devices[0].velocity, 0.0)
        self.assertGreater(robot.devices[1].velocity, 0.0)
        self.assertLess(robot.devices[2].velocity, 0.0)
        self.assertLess(robot.devices[3].velocity, 0.0)

    def test_nearby_forward_sonar_stops_and_reports_obstacle(self):
        adapter, robot = self.create_adapter({"x": 0.0, "y": 2.0})
        robot.devices[4].value = 1024.0 * (1.0 - 0.2 / 5.0)

        result = adapter.move_to_node("goal")

        self.assertTrue(result["obstacleDetected"])
        self.assertAlmostEqual(result["obstacleDistanceMeters"], 0.2)
        self.assertEqual(
            [motor.velocity for motor in robot.devices[:4]],
            [0.0, 0.0, 0.0, 0.0],
        )

    def test_arrival_and_unknown_node_stop_all_wheels(self):
        adapter, robot = self.create_adapter(
            {"x": 0.05, "y": 0.0},
            position=(0.0, 0.0, 0.0),
        )

        self.assertTrue(adapter.move_to_node("goal")["reached"])
        self.assertIn("error", adapter.move_to_node("missing"))
        self.assertEqual(
            [motor.velocity for motor in robot.devices[:4]],
            [0.0, 0.0, 0.0, 0.0],
        )


if __name__ == "__main__":
    unittest.main()
