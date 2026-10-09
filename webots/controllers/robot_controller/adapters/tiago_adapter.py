"""TIAGo base drive and waypoint navigation."""

from adapters.base_robot import BaseRobot
from config import POSITION_TOLERANCE
from navigation import Navigation


class TiagoAdapter(BaseRobot):
    BASE_MOTOR_NAMES = (
        "wheel_left_joint",
        "wheel_right_joint",
    )

    def __init__(self, robot, node_manager, robot_type):
        super().__init__(robot, node_manager, robot_type)
        self.base_motors = [
            self.require_device(name)
            for name in self.BASE_MOTOR_NAMES
        ]
        for motor in self.base_motors:
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)
        self.navigation = Navigation(
            robot,
            self.base_motors[0],
            self.base_motors[1],
            None,
            None,
            node_manager,
            position_tolerance=POSITION_TOLERANCE,
            max_speed=2.0,
            turn_speed=1.0,
        )
        self.capabilities.add("NAVIGATION")

    def get_position(self):
        return self.navigation.get_robot_position()

    def move_to_node(self, node_id):
        return self.navigation.move_to_node(node_id)

    def stop(self):
        self.navigation.stop()
        super().stop()
