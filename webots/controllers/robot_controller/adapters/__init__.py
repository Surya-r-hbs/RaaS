"""Robot-specific Webots hardware adapters."""

from adapters.base_robot import BaseRobot
from adapters.epuck_adapter import EpuckAdapter
from adapters.mir100_adapter import Mir100Adapter
from adapters.pioneer3at_adapter import Pioneer3atAdapter
from adapters.tiago_adapter import TiagoAdapter
from adapters.ur10e_adapter import UR10eAdapter
from adapters.youbot_adapter import YoubotAdapter


_ADAPTERS = {
    "epuck": EpuckAdapter,
    "mir100": Mir100Adapter,
    "pioneer3at": Pioneer3atAdapter,
    "tiago": TiagoAdapter,
    "ur10e": UR10eAdapter,
    "youbot": YoubotAdapter,
}


def create_robot_adapter(robot_type, robot, node_manager):
    adapter_type = "".join(
        character
        for character in str(robot_type).lower()
        if character.isalnum()
    )
    adapter_class = _ADAPTERS.get(adapter_type, BaseRobot)
    if adapter_class is BaseRobot:
        matching_types = [
            name
            for name in _ADAPTERS
            if name in adapter_type
        ]
        if len(matching_types) == 1:
            adapter_class = _ADAPTERS[matching_types[0]]
    return adapter_class(robot, node_manager, robot_type)
