"""E-puck hardware integration using the existing navigation implementation."""

from adapters.base_robot import BaseRobot
from config import (
    HEADING_TOLERANCE_DEGREES,
    MAX_SPEED,
    POSITION_TOLERANCE,
    TIME_STEP,
    TURN_SPEED,
)
from navigation import Navigation


class EpuckAdapter(BaseRobot):
    def __init__(self, robot, node_manager, robot_type):
        super().__init__(robot, node_manager, robot_type)

        self.left_motor = self.require_device("left wheel motor")
        self.right_motor = self.require_device("right wheel motor")
        self.gps = self.require_device("gps")
        self.compass = self.require_device("compass")

        for motor in (self.left_motor, self.right_motor):
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

        self.gps.enable(TIME_STEP)
        self.compass.enable(TIME_STEP)
        self.navigation = Navigation(
            robot,
            self.left_motor,
            self.right_motor,
            self.gps,
            self.compass,
            node_manager,
            position_tolerance=POSITION_TOLERANCE,
            max_speed=MAX_SPEED,
            turn_speed=TURN_SPEED,
            heading_tolerance_degrees=HEADING_TOLERANCE_DEGREES,
        )
        self.capabilities.add("NAVIGATION")

    def get_position(self):
        return self.navigation.get_robot_position()

    def move_to_node(self, node_id):
        return self.navigation.move_to_node(node_id)

    def stop(self):
        self.navigation.stop()
