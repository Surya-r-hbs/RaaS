"""KUKA YouBot omnidirectional base navigation."""

import math

from adapters.base_robot import BaseRobot
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
            return 0.0
        return math.atan2(orientation[3], orientation[0])

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

        position = self.get_robot_position()
        dx = target["x"] - position["x"]
        dy = target["y"] - position["y"]
        distance = math.hypot(dx, dy)
        if distance <= POSITION_TOLERANCE:
            self.stop()
            return {"reached": True, "error": False}

        speed = min(self.MAX_LINEAR_SPEED, distance * self.SPEED_GAIN)
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
