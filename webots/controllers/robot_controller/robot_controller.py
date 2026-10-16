import math
import os
import queue
import sys
import time
import uuid

from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from raas_logging import get_logger, log_event

logger = get_logger("raas.robot_controller")

# --------------------
# ROBOT CONTROLLER
# --------------------

try:
    from controller import Robot
except ImportError:  # pragma: no cover - only used outside Webots runtime.
    class Robot:
        def __init__(self, *args, **kwargs):
            self._devices = {}

        def getDevice(self, name):
            if name not in self._devices:
                raise RuntimeError(f"Device '{name}' is unavailable outside Webots.")
            return self._devices[name]

        def step(self, _time_step):
            return -1

from config import (
    MANUFACTURER,
    TIME_STEP,
    WEBOTS_SIMULATION_MODE,
)

from robot_state import RobotState
from node_manager import NodeManager
from mqtt_handler import MQTTHandler
from robot_discovery import discover_local_robot
from adapters import create_robot_adapter
from adapters.base_robot import UnsupportedRobotOperation
from robot_commands import (
    discard_queued_manipulation_commands as discard_commands,
    handle_command as process_command,
    handle_recovery_command as process_recovery_command,
    handle_stop_command as process_stop_command,
)
from robot_status import publish_status as publish_robot_status


def resolve_current_node(nodes, position, proximity_radius=0.75):
    """Return the node ID only when the robot is physically close to it."""
    if not nodes or position is None:
        return None

    best_node_id = None
    best_distance = None
    for node_id, node_position in nodes.items():
        if not isinstance(node_position, dict):
            continue
        dx = float(node_position.get("x", 0.0)) - float(position.get("x", 0.0))
        dy = float(node_position.get("y", 0.0)) - float(position.get("y", 0.0))
        distance = math.hypot(dx, dy)
        if best_distance is None or distance < best_distance:
            best_distance = distance
            best_node_id = node_id

    if best_node_id is None or best_distance is None:
        return None
    if best_distance > proximity_radius:
        return None
    return best_node_id


class RobotController:

    def __init__(self):
        self.logger = logger
        log_event(logger, "SYSTEM", "robot.controller.initialized", "Robot controller initialized", source=__file__, robot_id=None)

        # --------------------
        # WEBOTS
        # --------------------

        self.robot = Robot()
        local_robot = discover_local_robot(self.robot, MANUFACTURER)
        self.robot_id = local_robot["robotId"]
        self.robot_type = local_robot["robotType"]
        self.world_robots = []
        self.manufacturer = local_robot["manufacturer"]

        # --------------------
        # NODE MANAGER
        # --------------------

        self.node_manager = NodeManager(self.robot_id)

        # --------------------
        # ROBOT ADAPTER
        # --------------------

        self.adapter = create_robot_adapter(
            self.robot_type,
            self.robot,
            self.node_manager,
        )
        log_event(logger, "ROBOTS", "robot.adapter_ready", "Robot adapter ready", robot_id=self.robot_id, robot_type=self.robot_type, adapter=type(self.adapter).__name__, devices=self.adapter.available_devices, capabilities=self.adapter.capability_names)

        # --------------------
        # ROBOT STATE
        # --------------------

        self.state = RobotState()
        self.last_error = None
        self.manipulation_commands = queue.Queue()
        self.agent_session_id = uuid.uuid4().hex
        self.status_sequence = 0
        self.simulation_mode_enabled = WEBOTS_SIMULATION_MODE

        # --------------------
        # MQTT
        # --------------------

        self.mqtt = MQTTHandler(
            self.handle_command,
            self.robot_id,
            self.manufacturer,
            self.robot_type,
            self.world_robots,
            {
                "capabilities": self.adapter.capability_names,
                "availableDevices": self.adapter.available_devices,
                "transportCapability": self.adapter.transport_capability,
                "runtimeMode": "WEBOTS_SIMULATION",
                "simulationModeEnabled": self.simulation_mode_enabled,
                "navigationAwareness": {
                    "status": "SUPERVISOR_SIMULATION_ONLY",
                    "source": (
                        "Webots Supervisor map and robot poses with "
                        "R2025a model footprint profiles where needed"
                    ),
                    "physicalObstacleAvoidance": False,
                    "localSensorResponse": (
                        "FORWARD_SONAR_STOP_ONLY"
                        if "".join(
                            character
                            for character in self.robot_type.lower()
                            if character.isalnum()
                        )
                        == "pioneer3at"
                        else "NOT_CONFIGURED"
                    ),
                },
                "agentSessionId": self.agent_session_id,
            },
            world_snapshot_callback=self.handle_world_snapshot,
        )

        # --------------------
        # STARTUP
        # --------------------

        log_event(logger, "SYSTEM", "robot.started", "Robot controller started", robot_id=self.robot_id, manufacturer=self.manufacturer, robot_type=self.robot_type)

        self.mqtt.connect()

    def handle_world_snapshot(self, payload):
        try:
            self.node_manager.update_world_snapshot(payload)
        except (TypeError, ValueError, KeyError, RuntimeError) as error:
            log_event(logger, "SIMULATION", "world_snapshot.invalid", "Rejected invalid Webots world snapshot", level="WARNING", robot_id=self.robot_id, error=str(error))
            return
        self.world_robots = payload["worldRobots"]
        self.mqtt.world_robots = self.world_robots
        if not any(
            record.get("robotId") == self.robot_id
            for record in self.world_robots
        ):
            log_event(logger, "SIMULATION", "world_snapshot.missing_robot", "World snapshot missing this robot", level="WARNING", robot_id=self.robot_id)

    # --------------------
    # COMMAND HANDLER
    # --------------------

    def handle_command(self, command):
        return process_command(self, command)

    # --------------------
    # STOP COMMAND
    # --------------------

    def handle_stop_command(self, command):
        return process_stop_command(self, command)

    def handle_recovery_command(self, command):
        return process_recovery_command(self, command)

    def _discard_queued_manipulation_commands(self):
        return discard_commands(self)

    # --------------------
    # STATUS
    # --------------------

    def publish_status(self, recovered_order_id=None):
        return publish_robot_status(
            self,
            recovered_order_id,
            simulation_mode_enabled=WEBOTS_SIMULATION_MODE,
        )

   # --------------------
# UPDATE
# --------------------

    def update(self):

        try:
            command = self.manipulation_commands.get_nowait()
        except queue.Empty:
            command = None
        if command is not None:
            if self.state.state == "EXECUTING":
                self.last_error = (
                    "Cannot start pick-and-place while the robot is navigating"
                )
                self.publish_status()
            else:
                try:
                    self.adapter.start_pick_and_place(command)
                except (TypeError, ValueError, RuntimeError) as error:
                    self.last_error = str(error)
                else:
                    self.last_error = None
                self.publish_status()

        if getattr(self.adapter, "task_active", False):
            previous_manipulation = self.adapter.manipulation_status
            self.adapter.update_manipulation()
            if self.adapter.manipulation_status != previous_manipulation:
                if (
                    self.adapter.manipulation_status
                    and self.adapter.manipulation_status["state"]
                    in {"COMPLETED", "FAILED", "CANCELLED"}
                ):
                    self.last_error = self.adapter.manipulation_status["error"]
                self.publish_status()

        if self.state.state != "EXECUTING":
            return

        target_node = self.state.target_node

        if not target_node:
            return

        try:
            result = self.adapter.move_to_node(target_node)
        except (UnsupportedRobotOperation, RuntimeError) as error:
            self.last_error = str(error)
            self.adapter.stop()
            self.state.set_error()
            self.publish_status()
            return

        # --------------------
        # HANDLE NAVIGATION RESULT
        # --------------------

        # Current navigation implementation returns bool.
        # Newer implementation may return a dictionary.
        if isinstance(result, bool):

            if not result:
                # Still navigating
                return

            # True means target reached
            reached = True

        elif isinstance(result, dict):

            if result.get("error"):
                self.adapter.stop()
                self.state.set_error()
                self.last_error = str(result["error"])
                log_event(logger, "NAVIGATION", "navigation.error", "Navigation returned an error", level="ERROR", robot_id=self.robot_id, error=self.last_error)
                self.publish_status()
                return

            if result.get("obstacleDetected"):
                self.adapter.stop()
                obstacle_message = (
                    "Navigation stopped by Pioneer3at sonar at "
                    f"{result.get('obstacleDistanceMeters', 'unknown')} m"
                )
                if self.last_error != obstacle_message:
                    self.last_error = obstacle_message
                    self.publish_status()
                return

            if (
                isinstance(self.last_error, str)
                and self.last_error.startswith(
                    "Navigation stopped by Pioneer3at sonar at "
                )
            ):
                self.last_error = None
                self.publish_status()

            reached = result.get("reached", False)

        else:
            self.adapter.stop()
            self.state.set_error()
            self.last_error = f"Unexpected navigation result: {result!r}"
            log_event(logger, "NAVIGATION", "navigation.unexpected_result", "Navigation returned an unexpected result", level="ERROR", robot_id=self.robot_id, result=str(result))
            self.publish_status()
            return

        # --------------------
        # NODE REACHED
        # --------------------

        if reached:
            self.adapter.stop()
            log_event(logger, "ROBOTS", "navigation.node_reached", "Navigation target node reached", robot_id=self.robot_id, node_id=target_node, edge_id=self.state.target_edge)
            self.state.node_reached()
            self.publish_status()

    # --------------------
    # RUN
    # --------------------

    def run(self):

        last_idle_telemetry = 0.0

        while self.robot.step(TIME_STEP) != -1:

            self.update()

            if time.monotonic() - last_idle_telemetry >= 1.0:
                self.publish_status()
                last_idle_telemetry = time.monotonic()


# --------------------
# ENTRY POINT
# --------------------

if __name__ == "__main__":

    controller = RobotController()

    controller.run()