"""Pioneer3at differential-drive navigation with sonar stop protection."""

import math

from adapters.base_robot import BaseRobot
from config import POSITION_TOLERANCE, TIME_STEP


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

    @staticmethod
    def _heading(rotation):
        # Pioneer3at's wheel axle is local Y; its chassis forward axis is +X.
        forward_x = rotation[0]
        forward_y = rotation[3]
        magnitude = math.hypot(forward_x, forward_y)
        if magnitude <= 1e-9:
            raise RuntimeError("Pioneer3at forward orientation is invalid")
        return math.atan2(forward_y / magnitude, forward_x / magnitude)

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
        distances = [
            distance
            for sensor in sensors.values()
            if (distance := self._sonar_distance(sensor.getValue())) is not None
        ]
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

        position = self.get_position()
        dx = float(target["x"]) - position["x"]
        dy = float(target["y"]) - position["y"]
        distance = math.hypot(dx, dy)
        if distance <= POSITION_TOLERANCE:
            self.stop()
            return {"reached": True, "error": False}

        obstacle_distance = self._obstacle_distance()
        self._last_obstacle_distance = obstacle_distance
        if (
            obstacle_distance is not None
            and obstacle_distance <= OBSTACLE_STOP_DISTANCE_METERS
        ):
            self.stop()
            return {
                "reached": False,
                "error": False,
                "obstacleDetected": True,
                "obstacleDistanceMeters": obstacle_distance,
            }

        desired_heading = math.atan2(dy, dx)
        heading_error = math.atan2(
            math.sin(desired_heading - self._heading(self.get_orientation())),
            math.cos(desired_heading - self._heading(self.get_orientation())),
        )

        if abs(heading_error) > HEADING_DEADBAND_RADIANS:
            turn_speed = min(MAX_WHEEL_SPEED, abs(heading_error))
            direction = 1.0 if heading_error > 0 else -1.0
            self._set_drive(-direction * turn_speed, direction * turn_speed)
        else:
            self._last_obstacle_distance = None
            forward_speed = min(MAX_WHEEL_SPEED, distance)
            self._set_drive(forward_speed, forward_speed)

        return {"reached": False, "error": False}
