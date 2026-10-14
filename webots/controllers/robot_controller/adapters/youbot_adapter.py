"""KUKA YouBot omnidirectional base navigation."""

import math

from adapters.base_robot import BaseRobot
from navigation_planner import ProgressMonitor
from config import (
    POSITION_TOLERANCE,
    WEBOTS_SIMULATION_MODE,
    YOUBOT_SIMULATION_TRANSPORT_PROFILE,
    YOUBOT_SUPPORT_SURFACE,
)


class _YoubotNavigation:
    WHEEL_RADIUS = 0.05
    MAX_LINEAR_SPEED = 0.35
    SPEED_GAIN = 1.5

    def __init__(self, robot, wheel_motors, node_manager):
        self.robot = robot
        self.wheel_motors = wheel_motors
        self.node_manager = node_manager
        self.target_node = None
        self.path = []
        self.path_index = 0
        self.planned_snapshot = None
        self.goal = None
        self.arrival_samples = 0
        self.arrived_node = None
        self.progress_monitor = ProgressMonitor()

    def get_robot_position(self):
        try:
            values = self.robot.getSelf().getPosition()
        except (AttributeError, RuntimeError):
            return self.node_manager.get_robot_position()
        return {
            "x": float(values[0]),
            "y": float(values[1]),
            "z": float(values[2]),
        }

    def get_robot_heading(self):
        try:
            orientation = self.robot.getSelf().getOrientation()
        except (AttributeError, RuntimeError):
            orientation = self.node_manager.get_robot_orientation()
        if not orientation or len(orientation) < 9:
            raise RuntimeError("Robot orientation is unavailable or invalid")
        forward_x = float(orientation[0])
        forward_y = float(orientation[3])
        if not math.isfinite(forward_x) or not math.isfinite(forward_y):
            raise RuntimeError("Robot orientation is unavailable or invalid")
        magnitude = math.hypot(forward_x, forward_y)
        if not math.isfinite(magnitude) or magnitude <= 1e-9:
            raise RuntimeError("Robot orientation is unavailable or invalid")
        return math.atan2(forward_y / magnitude, forward_x / magnitude)

    def stop(self):
        for motor in self.wheel_motors:
            motor.setVelocity(0.0)

    def _set_wheel_speeds(self, forward_speed, lateral_speed):
        wheel_speeds = (
            forward_speed + lateral_speed,
            forward_speed - lateral_speed,
            forward_speed - lateral_speed,
            forward_speed + lateral_speed,
        )
        for motor, speed in zip(self.wheel_motors, wheel_speeds):
            motor.setVelocity(speed / self.WHEEL_RADIUS)

    def move_to_node(self, node_id):
        target = self.node_manager.get_node(node_id)
        if target is None:
            self.stop()
            return {"reached": False, "error": f"Node not found: {node_id}"}

        if node_id != self.target_node:
            self.target_node = node_id
            self.path = []
            self.path_index = 0
            self.planned_snapshot = None
            self.goal = None
            self.arrival_samples = 0
            self.arrived_node = None
            self.progress_monitor.reset()
        if self.arrived_node == node_id:
            self.stop()
            return {"reached": True, "error": False}

        position = self.get_robot_position()
        generation = getattr(self.node_manager, "_snapshot_generation", None)
        if not self.path or generation != self.planned_snapshot:
            try:
                plan = self.node_manager.plan_navigation_path(position, node_id)
            except (KeyError, TypeError, ValueError, RuntimeError) as error:
                self.stop()
                return {"reached": False, "error": str(error)}
            self.path = plan["path"]
            self.path_index = min(1, len(self.path) - 1)
            self.planned_snapshot = generation
            self.goal = plan["goal"]
            self.approach = plan["approach"]

        dx = self.goal["x"] - position["x"]
        dy = self.goal["y"] - position["y"]
        distance = math.hypot(dx, dy)
        if distance <= POSITION_TOLERANCE:
            self.stop()
            self.arrival_samples += 1
            if self.arrival_samples >= 3:
                self.arrived_node = node_id
                return {"reached": True, "error": False}
            return {"reached": False, "error": False}
        self.arrival_samples = 0

        while self.path_index < len(self.path) - 1:
            waypoint = self.path[self.path_index]
            if math.hypot(
                waypoint[0] - position["x"],
                waypoint[1] - position["y"],
            ) > POSITION_TOLERANCE:
                break
            self.path_index += 1
        waypoint = self.path[self.path_index]
        dx = waypoint[0] - position["x"]
        dy = waypoint[1] - position["y"]
        waypoint_distance = math.hypot(dx, dy)
        route_remaining = waypoint_distance + sum(
            math.hypot(second[0] - first[0], second[1] - first[1])
            for first, second in zip(
                self.path[self.path_index:],
                self.path[self.path_index + 1:],
            )
        )
        if self.progress_monitor.stalled(route_remaining):
            self.stop()
            return {
                "reached": False,
                "error": "Navigation made insufficient progress; route is blocked",
            }
        if waypoint_distance <= POSITION_TOLERANCE:
            dx = self.goal["x"] - position["x"]
            dy = self.goal["y"] - position["y"]

        speed = min(
            self.MAX_LINEAR_SPEED,
            waypoint_distance * self.SPEED_GAIN,
        )
        world_vx = dx / distance * speed
        world_vy = dy / distance * speed
        heading = self.get_robot_heading()

        forward_speed = (
            math.cos(heading) * world_vx
            + math.sin(heading) * world_vy
        )
        lateral_speed = (
            -math.sin(heading) * world_vx
            + math.cos(heading) * world_vy
        )
        self._set_wheel_speeds(forward_speed, lateral_speed)
        return {"reached": False, "error": False}


class YoubotAdapter(BaseRobot):
    WHEEL_MOTOR_NAMES = ("wheel1", "wheel2", "wheel3", "wheel4")
    SUPPORT_SURFACE = YOUBOT_SUPPORT_SURFACE

    def __init__(self, robot, node_manager, robot_type):
        super().__init__(robot, node_manager, robot_type)
        self.wheel_motors = [
            self.require_device(name)
            for name in self.WHEEL_MOTOR_NAMES
        ]
        for motor in self.wheel_motors:
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

        self.navigation = _YoubotNavigation(
            robot,
            self.wheel_motors,
            node_manager,
        )
        self.capabilities.add("NAVIGATION")

    @property
    def transport_capability(self):
        capability = {
            "status": "UNVERIFIED",
            "mode": "UNVERIFIED",
            "reason": (
                "The Webots YouBot model exposes support geometry, but this "
                "world requires explicit simulation-only transport configuration."
            ),
        }
        get_self = getattr(self.robot, "getSelf", None)
        node = get_self() if callable(get_self) else None
        get_proto_field = getattr(node, "getProtoField", None)
        field = get_proto_field("numberOfArms") if callable(get_proto_field) else None
        if field is None or field.getSFInt32() > 1:
            capability["reason"] = (
                "The configured YouBot model does not expose an unobstructed "
                "single-arm support surface."
            )
            return capability

        capability["supportSurface"] = self.SUPPORT_SURFACE
        capability["platformDimensions"] = self.SUPPORT_SURFACE["dimensions"]
        if WEBOTS_SIMULATION_MODE:
            capability.update(YOUBOT_SIMULATION_TRANSPORT_PROFILE)
            capability.update(
                {
                    "status": "SIMULATION_ONLY",
                    "mode": "SIMULATION_ONLY",
                    "canCarryPreloaded": True,
                    "reason": (
                        "Enabled by the configured Webots-only YouBot "
                        "simulation profile; not a physical rating."
                    ),
                }
            )
        return capability

    def get_position(self):
        return self.navigation.get_robot_position()

    def move_to_node(self, node_id):
        return self.navigation.move_to_node(node_id)

    def stop(self):
        self.navigation.stop()
        super().stop()
