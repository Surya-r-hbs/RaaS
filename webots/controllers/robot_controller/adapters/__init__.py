"""Robot-specific Webots hardware adapters."""

from adapters.base_robot import BaseRobot
from adapters.epuck_adapter import EpuckAdapter
from adapters.mir100_adapter import Mir100Adapter
from adapters.tiago_adapter import TiagoAdapter
from adapters.youbot_adapter import YoubotAdapter


_ADAPTERS = {
    "epuck": EpuckAdapter,
    "mir100": Mir100Adapter,
    "tiago": TiagoAdapter,
    "youbot": YoubotAdapter,
}


def create_robot_adapter(robot_type, robot, node_manager):
    adapter_type = "".join(
        character
        for character in str(robot_type).lower()
        if character.isalnum()
    )
    adapter_class = _ADAPTERS.get(adapter_type, BaseRobot)
    return adapter_class(robot, node_manager, robot_type)
