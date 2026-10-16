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
                "simulationModeEnabled": WEBOTS_SIMULATION_MODE,
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

        log_event(logger, "MQTT", "command.received", "Command received by robot controller", robot_id=self.robot_id, command_type=command.get("command") if isinstance(command, dict) else None, command=command if isinstance(command, dict) else None)

        if not isinstance(command, dict):
            log_event(logger, "MQTT", "command.invalid", "Robot command must be a JSON object", level="ERROR", robot_id=self.robot_id)
            return

        command_type = command.get("command")

        if command_type == "PICK_AND_PLACE":
            if "PICK_AND_PLACE" not in self.adapter.capabilities:
                self.last_error = (
                    f"Pick-and-place is not supported by {self.robot_type}"
                )
                self.publish_status()
                return
            self.manipulation_commands.put(command)
            return

        # --------------------
        # STOP ROBOT
        # --------------------

        if command_type == "STOP_ROBOT":

            self.handle_stop_command(command)

            return

        if command_type == "RECOVER_ROBOT":
            self.handle_recovery_command(command)
            return

        # --------------------
        # MOVE TO NODE
        # --------------------

        if command_type != "MOVE_TO_NODE":
            log_event(logger, "MQTT", "command.unsupported", "Unsupported robot command", level="WARNING", robot_id=self.robot_id, command_type=command_type)
            return

        order_id = command.get("orderId")
        node_id = command.get("nodeId")
        edge_id = command.get("edgeId")

        if not order_id:
            log_event(logger, "MQTT", "command.missing_order", "Robot command missing orderId", level="WARNING", robot_id=self.robot_id)
            return

        if getattr(self.adapter, "task_active", False):
            self.last_error = (
                "Cannot start navigation while pick-and-place is active"
            )
            self.publish_status()
            return

        if not node_id:
            log_event(logger, "MQTT", "command.missing_node", "Robot command missing nodeId", level="WARNING", robot_id=self.robot_id, order_id=order_id)
            return

        if not edge_id:
            log_event(logger, "MQTT", "command.missing_edge", "Robot command missing edgeId", level="WARNING", robot_id=self.robot_id, order_id=order_id, node_id=node_id)
            return

        # --------------------
        # CHECK NODE
        # --------------------

        if not self.node_manager.exists(node_id):
            log_event(logger, "ROBOTS", "navigation.invalid_node", "Robot navigation target node is unknown", level="ERROR", robot_id=self.robot_id, order_id=order_id, node_id=node_id)
            self.state.order_id = order_id
            self.state.set_error()
            self.publish_status()
            return

        if "NAVIGATION" not in self.adapter.capabilities:
            self.last_error = (
                f"Navigation is not supported by {self.robot_type}"
            )
            self.state.order_id = order_id
            self.state.set_error()
            self.publish_status()
            return

        # --------------------
        # CHECK BUSY
        # --------------------

        if self.state.state == "EXECUTING":
            log_event(logger, "ROBOTS", "navigation.busy", "Robot is already executing a navigation command", level="WARNING", robot_id=self.robot_id, target_node=self.state.target_node, order_id=self.state.order_id)
            return

        self.last_error = None

        # --------------------
        # START NAVIGATION
        # --------------------

        self.state.start_navigation(
            order_id,
            node_id,
            edge_id,
        )

        log_event(logger, "ROBOTS", "navigation.started", "Navigation started", robot_id=self.robot_id, order_id=order_id, target_node=node_id, edge_id=edge_id)

        self.publish_status()

    # --------------------
    # STOP COMMAND
    # --------------------

    def handle_stop_command(self, command):

        command_robot_id = command.get("robotId")

        # --------------------
        # VALIDATE ROBOT
        # --------------------

        if command_robot_id and command_robot_id != self.robot_id:
            log_event(logger, "ROBOTS", "stop.ignored", "Stop command ignored for another robot", level="WARNING", robot_id=self.robot_id, command_robot_id=command_robot_id)
            return

        # --------------------
        # STOP MOTION
        # --------------------

        log_event(logger, "ROBOTS", "stop.requested", "Stop command received", robot_id=self.robot_id, current_state=self.state.state, order_id=self.state.order_id, current_node=self.state.current_node, target_node=self.state.target_node)

        self._discard_queued_manipulation_commands()

        # This directly stops the navigation/motors.
        try:
            self.adapter.stop()
        except Exception as error:
            self.last_error = f"Failed to stop robot safely: {error}"
            self.state.set_error()
            log_event(logger, "ROBOTS", "stop.failed", "Robot stop failed", level="ERROR", robot_id=self.robot_id, error=self.last_error)
            self.publish_status()
            return

        # --------------------
        # UPDATE STATE
        # --------------------

        self.state.stop()
        log_event(logger, "ROBOTS", "stop.completed", "Robot stopped", robot_id=self.robot_id, order_id=self.state.order_id)

        # --------------------
        # NOTIFY FLEET + ERP
        # --------------------

        self.publish_status()

    def handle_recovery_command(self, command):
        command_robot_id = command.get("robotId")
        order_id = command.get("orderId")

        if command_robot_id and command_robot_id != self.robot_id:
            return
        if not order_id:
            log_event(logger, "RECOVERY", "recovery.rejected", "Recovery rejected because orderId is missing", level="WARNING", robot_id=self.robot_id)
            return
        if self.state.order_id and self.state.order_id != order_id:
            log_event(logger, "RECOVERY", "recovery.rejected", "Recovery rejected because active order does not match request", level="WARNING", robot_id=self.robot_id, active_order=self.state.order_id, requested_order=order_id)
            return

        self._discard_queued_manipulation_commands()
        try:
            self.adapter.stop()
        except Exception as error:
            self.last_error = f"Robot recovery stop failed: {error}"
            self.state.set_error()
            self.publish_status()
            return

        self.state.clear_order()
        self.last_error = None
        self.publish_status(recovered_order_id=order_id)

    def _discard_queued_manipulation_commands(self):
        commands = getattr(self, "manipulation_commands", None)
        if commands is None:
            return
        while True:
            try:
                commands.get_nowait()
            except queue.Empty:
                return

    # --------------------
    # STATUS
    # --------------------

    def publish_status(self, recovered_order_id=None):

        self.status_sequence += 1

        state = self.state.state
        pose_error = None
        try:
            position = self.adapter.get_position()
            orientation = self.adapter.get_orientation()
        except RuntimeError as error:
            position = None
            orientation = None
            pose_error = f"Webots world discovery is unavailable: {error}"
        nearest_node = None
        nearest_node_distance = None
        if position is not None:
            try:
                nearest_node, nearest_node_distance = (
                    self.node_manager.get_nearest_node(position)
                )
            except RuntimeError:
                pass
        current_node = (
            nearest_node
            if nearest_node_distance is not None
            and nearest_node_distance <= 0.75
            else None
        )

        message = {
            "robotId": self.robot_id,
            "manufacturer": self.manufacturer,
            "robotType": self.robot_type,
            "capabilities": self.adapter.capability_names,
            "availableDevices": self.adapter.available_devices,
            "transportCapability": self.adapter.transport_capability,
            "runtimeMode": "WEBOTS_SIMULATION",
            "simulationModeEnabled": WEBOTS_SIMULATION_MODE,

            # Fleet controller expects this.
            "state": state,

            # Compatibility/debugging.
            "status": state,

            # Mission correlation.
            "orderId": self.state.order_id,
            "currentNode": (
                self.state.current_node
                if state == "NODE_REACHED"
                else current_node
            ),
            "nearestNode": nearest_node,
            "nearestNodeDistance": nearest_node_distance,
            "position": position,
            "orientation": orientation,
            "worldRobots": self.world_robots,
            "error": self.last_error or pose_error,
            "agentSessionId": self.agent_session_id,
            "statusSequence": self.status_sequence,

            "nodeId": (
                self.state.current_node
                if state in {"NODE_REACHED", "STOPPED"}
                else self.state.target_node
            ),

            "edgeId": (
                self.state.current_edge
                if state in {"NODE_REACHED", "STOPPED"}
                else self.state.target_edge
            ),
        }
        if recovered_order_id:
            message["recoveredOrderId"] = recovered_order_id
        manipulation = getattr(self.adapter, "manipulation_status", None)
        if manipulation is not None:
            message["manipulation"] = manipulation

        log_event(logger, "ROBOTS", "status.published", "Robot status published", robot_id=self.robot_id, state=state, order_id=self.state.order_id, current_node=current_node, target_node=self.state.target_node, edge_id=self.state.target_edge)
        self.mqtt.publish_status(message)

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