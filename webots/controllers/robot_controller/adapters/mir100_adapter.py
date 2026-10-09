"""MiR100 drive and waypoint navigation."""

from adapters.base_robot import BaseRobot
from config import POSITION_TOLERANCE
from navigation import Navigation


class Mir100Adapter(BaseRobot):
    WHEEL_MOTOR_NAMES = (
        "middle_left_wheel_joint",
        "middle_right_wheel_joint",
    )

    def __init__(self, robot, node_manager, robot_type):
        super().__init__(robot, node_manager, robot_type)
        self.wheel_motors = [
            self.require_device(name)
            for name in self.WHEEL_MOTOR_NAMES
        ]
        for motor in self.wheel_motors:
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)
        self.navigation = Navigation(
            robot,
            self.wheel_motors[0],
            self.wheel_motors[1],
            None,
            None,
            node_manager,
            position_tolerance=POSITION_TOLERANCE,
            max_speed=3.0,
            turn_speed=1.5,
        )
        self.capabilities.add("NAVIGATION")

    def get_position(self):
        return self.navigation.get_robot_position()

    def move_to_node(self, node_id):
        return self.navigation.move_to_node(node_id)

    def stop(self):
        self.navigation.stop()
        super().stop()
