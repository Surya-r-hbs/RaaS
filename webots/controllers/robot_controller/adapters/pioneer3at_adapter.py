"""Pioneer3at differential-drive navigation with sonar stop protection."""

import math
import time

from adapters.base_robot import BaseRobot
from config import POSITION_TOLERANCE, TIME_STEP
from navigation_planner import ProgressMonitor


WHEEL_MOTOR_NAMES = (
    "back left wheel",
    "front left wheel",
    "back right wheel",
    "front right wheel",
)
SONAR_MAX_RANGE_METERS = 5.0
SONAR_MAX_OUTPUT = 1024.0
OBSTACLE_STOP_DISTANCE_METERS = 0.45
MAX_WHEEL_SPEED = 1.0
HEADING_DEADBAND_RADIANS = math.radians(4.0)


class Pioneer3atAdapter(BaseRobot):
    def __init__(self, robot, node_manager, robot_type):
        super().__init__(robot, node_manager, robot_type)
        self.wheel_motors = [
            self.require_device(name)
            for name in WHEEL_MOTOR_NAMES
        ]
        for motor in self.wheel_motors:
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

        self.sonars = {
            name: device
            for name, device in self.devices.items()
            if name == "so" or name.startswith("so") and name[2:].isdigit()
        }
        if not self.sonars:
            raise RuntimeError(
                "Pioneer3at requires at least one Pioneer distance sensor"
            )
        for sensor in self.sonars.values():
            sensor.enable(TIME_STEP)

        self.forward_sonar = self.sonars.get("so")
        self.front_sonars = {
            name: sensor
            for name, sensor in self.sonars.items()
            if name in {"so", "so0", "so1", "so2", "so3"}
        }
        self.capabilities.add("NAVIGATION")
        self._last_obstacle_distance = None
        self._target_node = None
        self._path = []
        self._path_index = 0
        self._planned_snapshot = None
        self._goal = None
        self._approach = False
        self._arrival_samples = 0
        self._arrived_node = None
        self._progress_monitor = ProgressMonitor()
        self._obstacle_since = None

    @staticmethod
    def _heading(rotation):
        # Pioneer3at's wheel axle is local Y; its chassis forward axis is +X.
        try:
            forward_x = float(rotation[0])
            forward_y = float(rotation[3])
        except (TypeError, IndexError, ValueError):
            raise RuntimeError("Pioneer3at forward orientation is invalid")
        if not math.isfinite(forward_x) or not math.isfinite(forward_y):
            raise RuntimeError("Pioneer3at forward orientation is invalid")
        magnitude = math.hypot(forward_x, forward_y)
        if magnitude <= 1e-9:
            raise RuntimeError("Pioneer3at forward orientation is invalid")
        heading = math.atan2(forward_y / magnitude, forward_x / magnitude)
        if not math.isfinite(heading):
            raise RuntimeError("Pioneer3at forward orientation is invalid")
        return heading

    @staticmethod
    def _sonar_distance(value):
        """Convert the Pioneer PROTO's 0..1024, 0..5 m lookup table."""
        numeric = float(value)
        if not math.isfinite(numeric):
            return None
        output = min(SONAR_MAX_OUTPUT, max(0.0, numeric))
        return SONAR_MAX_RANGE_METERS * (
            1.0 - output / SONAR_MAX_OUTPUT
        )

    def _obstacle_distance(self):
        sensors = self.front_sonars or self.sonars
        distances = []
        for sensor in sensors.values():
            try:
                distance = self._sonar_distance(sensor.getValue())
            except (TypeError, ValueError, OverflowError):
                continue
            if distance is not None:
                distances.append(distance)
        return min(distances) if distances else None

    def _set_drive(self, left_speed, right_speed):
        left_speed = max(
            -MAX_WHEEL_SPEED,
            min(MAX_WHEEL_SPEED, left_speed),
        )
        right_speed = max(
            -MAX_WHEEL_SPEED,
            min(MAX_WHEEL_SPEED, right_speed),
        )
        self.wheel_motors[0].setVelocity(left_speed)
        self.wheel_motors[1].setVelocity(left_speed)
        self.wheel_motors[2].setVelocity(right_speed)
        self.wheel_motors[3].setVelocity(right_speed)

    def stop(self):
        self._set_drive(0.0, 0.0)

    def move_to_node(self, node_id):
        target = self.node_manager.get_node(node_id)
        if target is None:
            self.stop()
            return {
                "reached": False,
                "error": f"Node not found: {node_id}",
            }

        if node_id != self._target_node:
            self._target_node = node_id
            self._path = []
            self._path_index = 0
            self._planned_snapshot = None
            self._goal = None
            self._approach = False
            self._arrival_samples = 0
            self._arrived_node = None
            self._progress_monitor.reset()
            self._obstacle_since = None
        if self._arrived_node == node_id:
            self.stop()
            return {"reached": True, "error": False}

        position = self.get_position()
        generation = getattr(self.node_manager, "_snapshot_generation", None)
        if not self._path or generation != self._planned_snapshot:
            try:
                plan = self.node_manager.plan_navigation_path(position, node_id)
            except (KeyError, TypeError, ValueError, RuntimeError) as error:
                self.stop()
                return {"reached": False, "error": str(error)}
            self._path = plan["path"]
            self._path_index = min(1, len(self._path) - 1)
            self._planned_snapshot = generation
            self._goal = plan["goal"]
            self._approach = plan["approach"]

        dx = self._goal["x"] - position["x"]
        dy = self._goal["y"] - position["y"]
        distance = math.hypot(dx, dy)
        if not math.isfinite(distance):
            self.stop()
            return {"reached": False, "error": "Target distance is invalid"}
        if distance <= POSITION_TOLERANCE:
            self.stop()
            self._arrival_samples += 1
            if self._arrival_samples >= 3:
                self._arrived_node = node_id
                return {"reached": True, "error": False}
            return {"reached": False, "error": False}
        self._arrival_samples = 0

        obstacle_distance = self._obstacle_distance()
        self._last_obstacle_distance = obstacle_distance
        if obstacle_distance is None:
            self.stop()
            return {
                "reached": False,
                "error": "Pioneer3at forward sonar readings are unavailable",
            }
        if (
            obstacle_distance <= OBSTACLE_STOP_DISTANCE_METERS
        ):
            self.stop()
            if self._obstacle_since is None:
                self._obstacle_since = time.monotonic()
            if time.monotonic() - self._obstacle_since > 10.0:
                return {
                    "reached": False,
                    "error": (
                        "Pioneer3at remained blocked by an obstacle for "
                        "10 seconds"
                    ),
                }
            return {
                "reached": False,
                "error": False,
                "obstacleDetected": True,
                "obstacleDistanceMeters": obstacle_distance,
            }
        self._obstacle_since = None

        while self._path_index < len(self._path) - 1:
            waypoint = self._path[self._path_index]
            if math.hypot(
                waypoint[0] - position["x"],
                waypoint[1] - position["y"],
            ) > POSITION_TOLERANCE:
                break
            self._path_index += 1
        waypoint = self._path[self._path_index]
        dx = waypoint[0] - position["x"]
        dy = waypoint[1] - position["y"]
        waypoint_distance = math.hypot(dx, dy)
        route_remaining = waypoint_distance + sum(
            math.hypot(second[0] - first[0], second[1] - first[1])
            for first, second in zip(
                self._path[self._path_index:],
                self._path[self._path_index + 1:],
            )
        )
        if self._progress_monitor.stalled(route_remaining):
            self.stop()
            return {
                "reached": False,
                "error": "Navigation made insufficient progress; route is blocked",
            }
        if waypoint_distance <= POSITION_TOLERANCE:
            dx = self._goal["x"] - position["x"]
            dy = self._goal["y"] - position["y"]

        try:
            current_heading = self._heading(self.get_orientation())
            desired_heading = math.atan2(dy, dx)
            heading_error = math.atan2(
                math.sin(desired_heading - current_heading),
                math.cos(desired_heading - current_heading),
            )
        except (TypeError, ValueError, RuntimeError):
            self.stop()
            return {
                "reached": False,
                "error": "Robot orientation is unavailable or invalid",
            }

        if not math.isfinite(heading_error):
            self.stop()
            return {
                "reached": False,
                "error": "Robot heading is unavailable or invalid",
            }

        if abs(heading_error) > HEADING_DEADBAND_RADIANS:
            turn_speed = min(MAX_WHEEL_SPEED, abs(heading_error))
            direction = 1.0 if heading_error > 0 else -1.0
            self._set_drive(-direction * turn_speed, direction * turn_speed)
        else:
            self._last_obstacle_distance = None
            forward_speed = min(MAX_WHEEL_SPEED, waypoint_distance)
            self._set_drive(forward_speed, forward_speed)

        return {"reached": False, "error": False}
