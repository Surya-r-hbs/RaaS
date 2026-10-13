import os
import queue
import time
import uuid

print("--------------------")
print("ROBOT CONTROLLER FILE")
print(os.path.abspath(__file__))
print("--------------------")

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


class RobotController:

    def __init__(self):

        print("--------------------")
        print("MODULAR CONTROLLER VERSION: 2026-10-06")
        print(f"CONTROLLER FILE: {__file__}")
        print("--------------------")

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
        self.node_manager.print_nodes()

        # --------------------
        # ROBOT ADAPTER
        # --------------------

        self.adapter = create_robot_adapter(
            self.robot_type,
            self.robot,
            self.node_manager,
        )
        print("--------------------")
        print("ROBOT ADAPTER READY")
        print(f"Robot type       : {self.robot_type}")
        print(f"Adapter          : {type(self.adapter).__name__}")
        print(f"Devices          : {self.adapter.available_devices}")
        print(f"Capabilities     : {self.adapter.capability_names}")
        print("--------------------")

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
                "agentSessionId": self.agent_session_id,
            },
            world_snapshot_callback=self.handle_world_snapshot,
        )

        # --------------------
        # STARTUP
        # --------------------

        print("--------------------")
        print("ROBOT CONTROLLER STARTED")
        print(f"Robot       : {self.robot_id}")
        print(f"Manufacturer: {self.manufacturer}")
        print("--------------------")

        self.mqtt.connect()

    def handle_world_snapshot(self, payload):
        try:
            self.node_manager.update_world_snapshot(payload)
        except (TypeError, ValueError, KeyError) as error:
            print(f"Rejected invalid Webots world snapshot: {error}")
            return
        self.world_robots = payload["worldRobots"]
        self.mqtt.world_robots = self.world_robots
        if not any(
            record.get("robotId") == self.robot_id
            for record in self.world_robots
        ):
            print(
                f"World snapshot does not contain this robot "
                f"({self.robot_id}); pose-dependent tasks will be rejected"
            )

    # --------------------
    # COMMAND HANDLER
    # --------------------

    def handle_command(self, command):

        print("--------------------")
        print("COMMAND RECEIVED")
        print(command)
        print("--------------------")

        if not isinstance(command, dict):
            print("ERROR: Robot command must be a JSON object")
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

            print(f"ERROR: Unsupported command: {command_type}")

            return

        order_id = command.get("orderId")
        node_id = command.get("nodeId")
        edge_id = command.get("edgeId")

        if not order_id:

            print("ERROR: Missing orderId")

            return

        if getattr(self.adapter, "task_active", False):
            self.last_error = (
                "Cannot start navigation while pick-and-place is active"
            )
            self.publish_status()
            return

        if not node_id:

            print("ERROR: Missing nodeId")

            return

        if not edge_id:

            print("ERROR: Missing edgeId")

            return

        # --------------------
        # CHECK NODE
        # --------------------

        if not self.node_manager.exists(node_id):

            print("--------------------")
            print("ERROR: UNKNOWN NODE")
            print(f"Node: {node_id}")
            print("--------------------")

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

            print("--------------------")
            print("ERROR: ROBOT BUSY")
            print(f"Current target: {self.state.target_node}")
            print("--------------------")

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

        print("--------------------")
        print("STARTING NAVIGATION")
        print(f"Order ID   : {order_id}")
        print(f"Target node: {node_id}")
        print(f"Target edge: {edge_id}")
        print("--------------------")

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

            print("--------------------")
            print("STOP COMMAND IGNORED")
            print(f"Command robot : {command_robot_id}")
            print(f"This robot    : {self.robot_id}")
            print("--------------------")

            return

        # --------------------
        # STOP MOTION
        # --------------------

        print("--------------------")
        print("STOP ROBOT COMMAND")
        print("--------------------")

        print(f"Current state : {self.state.state}")
        print(f"Order ID      : {self.state.order_id}")
        print(f"Current node  : {self.state.current_node}")
        print(f"Current edge  : {self.state.current_edge}")
        print(f"Target node   : {self.state.target_node}")
        print(f"Target edge   : {self.state.target_edge}")

        self._discard_queued_manipulation_commands()

        # This directly stops the navigation/motors.
        try:
            self.adapter.stop()
        except Exception as error:
            self.last_error = f"Failed to stop robot safely: {error}"
            self.state.set_error()
            print(self.last_error)
            self.publish_status()
            return

        # --------------------
        # UPDATE STATE
        # --------------------

        self.state.stop()

        print("--------------------")
        print("ROBOT STOPPED")
        print("--------------------")

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
            print("RECOVERY REJECTED: Missing orderId")
            return
        if self.state.order_id and self.state.order_id != order_id:
            print(
                "RECOVERY REJECTED: Active order does not match "
                f"requested order {order_id}"
            )
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
        nodes = self.node_manager.get_nodes()
        current_node = min(
            nodes,
            key=lambda node_id: (
                (nodes[node_id]["x"] - position["x"]) ** 2
                + (nodes[node_id]["y"] - position["y"]) ** 2
            ),
        ) if nodes and position is not None else None

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

        print("--------------------")
        print("ROBOT STATUS")
        print(message)
        print("--------------------")

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

                print("--------------------")
                print("NAVIGATION ERROR")
                print(result["error"])
                print("--------------------")

                self.adapter.stop()

                self.state.set_error()
                self.last_error = str(result["error"])

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

            print("--------------------")
            print("NAVIGATION ERROR")
            print("Unexpected navigation result:")
            print(result)
            print("--------------------")

            self.adapter.stop()

            self.state.set_error()
            self.last_error = f"Unexpected navigation result: {result!r}"

            self.publish_status()

            return

        # --------------------
        # NODE REACHED
        # --------------------

        if reached:

            self.adapter.stop()

            print("--------------------")
            print("NODE REACHED")
            print(f"Node: {target_node}")
            print(f"Edge: {self.state.target_edge}")
            print("--------------------")

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