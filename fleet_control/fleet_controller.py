"""
VDA-5050 fleet controller for the RaaS lab.

The ERP frontend publishes VDA-5050 orders to ORDER_TOPIC.
This controller validates the order, commands the Webots robot through
the internal robot command topic, and publishes VDA-5050 state.
"""

# --------------------
# IMPORTS
# --------------------

import json
import threading
import time

from datetime import datetime, timezone

import paho.mqtt.client as mqtt

from config import (
    BROKER,
    PORT,
    MANUFACTURER,
    SERIAL_NUMBER,
    ORDER_TOPIC,
    ROBOT_COMMAND_TOPIC,
    ROBOT_STATUS_TOPIC,
    STATE_TOPIC,
    VDA_VERSION,
    MQTT_KEEPALIVE,
    order_topic,
    state_topic,
    robot_command_topic,
    robot_status_topic,
)

from mission import Mission, validate_order
from robot_registry import RobotRegistry


# --------------------
# GLOBAL MISSION
# --------------------

missions: dict[tuple[str, str], Mission] = {}
robot_registry = RobotRegistry()

state_lock = threading.Lock()
REGISTRY_TOPIC = "fleet/robots"


def publish_robot_registry() -> None:
    result = mqtt_client.publish(
        REGISTRY_TOPIC,
        json.dumps({"robots": robot_registry.snapshot()}),
        qos=1,
        retain=True,
    )
    if result.rc != mqtt.MQTT_ERR_SUCCESS:
        log(f"ERROR: Failed to publish robot registry: {result.rc}")


# --------------------
# TIME
# --------------------

def now_utc() -> str:

    return datetime.now(
        timezone.utc
    ).isoformat(
        timespec="milliseconds"
    ).replace(
        "+00:00",
        "Z",
    )


# --------------------
# LOGGING
# --------------------

def log(message: str) -> None:

    print(
        message,
        flush=True,
    )


def separator() -> None:

    print(
        "-" * 60,
        flush=True,
    )


# --------------------
# PUBLISH VDA STATE
# --------------------

def publish_state(robot_key: tuple[str, str]) -> None:

    with state_lock:

        current = missions.get(robot_key)

        if current is None:

            return

        nodes = current.nodes
        edges = current.edges

        node_states = []
        edge_states = []

        if 0 <= current.node_index < len(nodes):

            node = nodes[current.node_index]

            node_states.append(
                {
                    "nodeId": node["nodeId"],
                    "sequenceId": node["sequenceId"],
                    "released": node.get(
                        "released",
                        True,
                    ),
                }
            )

        if (
            current.state not in {"COMPLETED", "STOPPED"}
            and 0 <= current.edge_index < len(edges)
        ):

            edge = edges[current.edge_index]

            edge_states.append(
                {
                    "edgeId": edge["edgeId"],
                    "sequenceId": edge["sequenceId"],
                    "released": edge.get(
                        "released",
                        True,
                    ),
                }
            )

        manufacturer = current.order.get("manufacturer", MANUFACTURER)
        serial_number = current.order.get("serialNumber", SERIAL_NUMBER)
        payload = {
            "headerId": current.header_id,
            "timestamp": now_utc(),
            "version": current.order.get(
                "version",
                VDA_VERSION,
            ),
            "manufacturer": manufacturer,
            "serialNumber": serial_number,
            "orderId": current.order_id,
            "orderUpdateId": current.order.get(
                "orderUpdateId",
                0,
            ),
            "lastNodeId": current.last_node_id,
            "lastNodeSequenceId": current.last_node_sequence_id,
            "nodeStates": node_states,
            "edgeStates": edge_states,
        }

        current.header_id += 1

        mission_state = current.state
        current_edge_id = current.current_edge_id
        current_edge_sequence_id = current.current_edge_sequence_id
        state_topic_name = state_topic(manufacturer, serial_number)

    result = mqtt_client.publish(
        state_topic_name,
        json.dumps(payload),
        qos=0,
    )

    separator()

    log("PUBLISHING VDA-5050 STATE")

    separator()

    log(f"Topic        : {state_topic_name}")
    log(f"Mission      : {mission_state}")
    log(f"Order ID     : {payload['orderId']}")
    log(f"Last node    : {payload['lastNodeId']}")
    log(f"Node seq     : {payload['lastNodeSequenceId']}")
    log(f"Current edge : {current_edge_id}")
    log(f"Edge seq     : {current_edge_sequence_id}")
    log(f"Payload      : {json.dumps(payload)}")

    separator()

    if result.rc == mqtt.MQTT_ERR_SUCCESS:

        log("VDA-5050 STATE PUBLISHED")

    else:

        log(
            f"ERROR: Failed to publish VDA-5050 state: "
            f"{result.rc}"
        )


# --------------------
# SEND ROBOT COMMAND
# --------------------

def send_robot_command(
    order_id: str,
    start_node: dict,
    target_node: dict,
    edge: dict,
    manufacturer: str | None = None,
    serial_number: str | None = None,
) -> bool:

    manufacturer = manufacturer or MANUFACTURER
    serial_number = serial_number or SERIAL_NUMBER
    command_topic_name = robot_command_topic(manufacturer, serial_number)

    command = {
        "command": "MOVE_TO_NODE",
        "orderId": order_id,
        "nodeId": target_node["nodeId"],
        "edgeId": edge["edgeId"],
    }

    payload = json.dumps(command)

    separator()

    log("SENDING ROBOT COMMAND")

    separator()

    log(f"Topic      : {command_topic_name}")
    log(f"Order ID   : {order_id}")
    log(f"Start node : {start_node['nodeId']}")
    log(f"Target     : {target_node['nodeId']}")
    log(f"Edge       : {edge['edgeId']}")
    log(f"Payload    : {payload}")

    separator()

    result = mqtt_client.publish(
        command_topic_name,
        payload,
        qos=0,
    )

    if result.rc != mqtt.MQTT_ERR_SUCCESS:

        log(
            f"ERROR: Failed to send robot command: "
            f"{result.rc}"
        )

        return False

    log(
        "ROBOT COMMAND SENT SUCCESSFULLY"
    )

    return True


# --------------------
# START NEXT SEGMENT
# --------------------

def start_next_segment(robot_key: tuple[str, str]) -> None:

    with state_lock:

        current = missions.get(robot_key)

        if current is None:

            return

        # --------------------
        # DO NOT CONTINUE STOPPED MISSION
        # --------------------

        if current.state == "STOPPED":

            log(
                "MISSION IS STOPPED - "
                "NO NEXT SEGMENT WILL BE SENT"
            )

            return

        # --------------------
        # FINAL NODE
        # --------------------

        if current.node_index >= len(current.nodes) - 1:

            current.state = "COMPLETED"

            current.current_edge_id = None
            current.current_edge_sequence_id = 0

            final_node = current.last_node_id
            order_id = current.order_id

            total_nodes = len(current.nodes)
            total_edges = len(current.edges)

            should_send = False

        else:

            edge = current.edges[current.edge_index]

            start_node = current.nodes[current.node_index]

            target_node = current.nodes[
                current.node_index + 1
            ]

            current.current_edge_id = edge["edgeId"]

            current.current_edge_sequence_id = edge[
                "sequenceId"
            ]

            current.state = "EXECUTING"

            order_id = current.order_id

            should_send = True

    # --------------------
    # MISSION COMPLETED
    # --------------------

    if not should_send:

        separator()

        log("MISSION COMPLETED")

        separator()

        log(f"Order ID   : {order_id}")
        log(f"Final node : {final_node}")
        log(f"Total nodes: {total_nodes}")
        log(f"Total edges: {total_edges}")

        separator()

        robot_registry.set_state(
            robot_key[0],
            robot_key[1],
            "IDLE",
            current_node=final_node,
        )
        publish_state(robot_key)
        publish_robot_registry()

        return

    # --------------------
    # SEND NEXT SEGMENT
    # --------------------

    robot_registry.set_state(
        robot_key[0],
        robot_key[1],
        "DISPATCHED",
        current_order=order_id,
        current_node=start_node["nodeId"],
        current_edge=edge["edgeId"],
    )
    publish_state(robot_key)
    publish_robot_registry()

    command_sent = send_robot_command(
        order_id,
        start_node,
        target_node,
        edge,
        robot_key[0],
        robot_key[1],
    )
    if not command_sent:
        with state_lock:
            current = missions.get(robot_key)
            if current is not None and current.order_id == order_id:
                current.state = "ERROR"
        robot_registry.set_state(
            robot_key[0],
            robot_key[1],
            "DISPATCHED",
            current_order=order_id,
            current_node=start_node["nodeId"],
            current_edge=edge["edgeId"],
        )
        robot_registry.mark_recovery_required(
            robot_key[0],
            robot_key[1],
            "Robot command delivery failed; verify the robot before recovery",
        )
        publish_robot_registry()


# --------------------
# HANDLE VDA ORDER
# --------------------

def handle_order(order: dict) -> None:

    manufacturer = order.get("manufacturer", MANUFACTURER)
    serial_number = order.get("serialNumber", SERIAL_NUMBER)
    robot_key = (manufacturer, serial_number)

    valid, reason = validate_order(
        order,
        manufacturer,
        serial_number,
    )

    separator()

    log("VALIDATING VDA-5050 ORDER")

    separator()

    log(reason)

    if not valid:

        log("ORDER REJECTED")

        return

    registered_robot = robot_registry.get(manufacturer, serial_number)
    if registered_robot is None:
        log(
            f"ORDER REJECTED: robot {manufacturer}/{serial_number} "
            "has not registered with the fleet"
        )
        return

    if not registered_robot["available"]:
        log(
            f"ORDER REJECTED: robot {manufacturer}/{serial_number} "
            f"is unavailable ({registered_robot['availabilityReason']})"
        )
        return

    with state_lock:
        current_mission = missions.get(robot_key)

        if (
            current_mission is not None
            and current_mission.state in {
                "EXECUTING",
                "PAUSED",
            }
            and current_mission.order_id != order["orderId"]
        ):

            log(
                f"ORDER REJECTED: robot is already "
                f"executing {current_mission.order_id}"
            )

            return

        if (
            current_mission is not None
            and current_mission.order_id == order["orderId"]
        ):
            log(f"Duplicate order ignored: {order['orderId']}")
            return

        first_node = order["nodes"][0]
        first_edge = order["edges"][0]

        missions[robot_key] = Mission(
            order=order,
            node_index=0,
            edge_index=0,
            last_node_id=first_node["nodeId"],
            last_node_sequence_id=first_node[
                "sequenceId"
            ],
            current_edge_id=first_edge["edgeId"],
            current_edge_sequence_id=first_edge[
                "sequenceId"
            ],
            state="EXECUTING",
            header_id=0,
        )

    separator()

    log("MISSION INITIALIZED")

    separator()

    log(f"Order ID    : {order['orderId']}")
    log(f"Start node  : {first_node['nodeId']}")
    log(
        f"Target node : "
        f"{order['nodes'][1]['nodeId']}"
    )
    log(f"First edge  : {first_edge['edgeId']}")

    separator()

    start_next_segment(robot_key)


# --------------------
# HANDLE ROBOT STATUS
# --------------------

def handle_robot_status(status: dict) -> None:

    robot_id = str(
        status.get("robotId")
        or status.get("serialNumber")
        or ""
    ).strip()
    manufacturer = str(status.get("manufacturer") or "UNKNOWN").strip()
    robot_key = (manufacturer, robot_id)
    state = status.get("state")
    order_id = status.get("orderId")
    node_id = status.get("nodeId")
    edge_id = status.get("edgeId")

    if not robot_id:
        log("Ignoring robot status without a robot ID")
        return

    if not robot_registry.update_status(status):
        log(
            "Ignoring duplicate, delayed, or mismatched robot status "
            f"for {manufacturer}/{robot_id} order={order_id!r}"
        )
        return
    robot_registry.discover_world(status.get("worldRobots", []))

    separator()

    log("ROBOT STATUS")

    separator()

    log(f"Robot    : {robot_id}")
    log(f"State    : {state}")
    log(f"Order ID : {order_id}")
    log(f"Node ID  : {node_id}")
    log(f"Edge ID  : {edge_id}")

    separator()

    if state == "IDLE":
        recovered_order_id = status.get("recoveredOrderId")
        if recovered_order_id:
            with state_lock:
                current = missions.get(robot_key)
                if current is not None and current.order_id == recovered_order_id:
                    current.state = "STOPPED"
        publish_robot_registry()
        return

    with state_lock:
        previous_mission = missions.get(robot_key)
        completed_mission_heartbeat = (
            state == "NODE_REACHED"
            and previous_mission is not None
            and previous_mission.order_id == order_id
            and previous_mission.state == "COMPLETED"
            and previous_mission.last_node_id == node_id
        )

    if completed_mission_heartbeat:
        robot_registry.set_state(
            manufacturer,
            robot_id,
            "IDLE",
            current_node=node_id,
        )
        publish_robot_registry()
        return

    with state_lock:
        previous_mission = missions.get(robot_key)
        repeated_terminal_status = (
            state in {"STOPPED", "ERROR"}
            and previous_mission is not None
            and previous_mission.order_id == order_id
            and previous_mission.state == state
        )

    if repeated_terminal_status:
        publish_robot_registry()
        return

    with state_lock:

        current = missions.get(robot_key)

        if current is None or order_id != current.order_id:

            current = None

        if current is not None:
            nodes = current.nodes
            edges = current.edges

        reached_index = None

        # --------------------
        # EXECUTING
        # --------------------

        if current is None:
            unmatched_status = True
        else:
            unmatched_status = False

        if unmatched_status:
            pass

        elif state == "EXECUTING":

            current.state = "EXECUTING"

        elif state == "NODE_REACHED":

            expected_index = current.node_index + 1
            expected_edge_index = current.edge_index

            if (
                expected_index >= len(nodes)
                or expected_edge_index >= len(edges)
                or nodes[expected_index]["nodeId"] != node_id
                or edges[expected_edge_index]["edgeId"] != edge_id
            ):

                log(
                    "Ignoring unexpected NODE_REACHED status "
                    f"(node={node_id}, edge={edge_id}); "
                    "it does not match the next mission segment."
                )

                invalid_reached_status = True

            else:
                invalid_reached_status = False

            if invalid_reached_status:
                mission_completed = False
                mission_failed = False
                mission_stopped = False
                next_segment = False
            else:
                reached_index = expected_index
                current.node_index = reached_index

                current.last_node_id = node_id

                current.last_node_sequence_id = nodes[
                    reached_index
                ]["sequenceId"]

                mission_completed = reached_index == len(nodes) - 1
                mission_failed = False
                mission_stopped = False
                next_segment = not mission_completed

                if mission_completed:
                    current.state = "COMPLETED"
                    current.current_edge_id = None
                    current.current_edge_sequence_id = 0
                    order_id = current.order_id
                    final_node = current.last_node_id
                    total_nodes = len(nodes)
                    total_edges = len(edges)
                else:
                    current.edge_index = reached_index
                    next_edge = edges[current.edge_index]
                    next_node = nodes[reached_index + 1]
                    current.current_edge_id = next_edge["edgeId"]
                    current.current_edge_sequence_id = next_edge["sequenceId"]
                    current.state = "EXECUTING"
                    start_node = nodes[current.node_index]
                    order_id = current.order_id

        elif state == "STOPPED":

            current.state = "STOPPED"
            current.current_edge_id = edge_id
            current.current_edge_sequence_id = 0
            mission_stopped = True
            mission_completed = False
            mission_failed = False

        elif state == "ERROR":

            current.state = "ERROR"
            mission_failed = True
            mission_completed = False
            mission_stopped = False

        else:

            log(
                f"Ignoring unsupported robot state: "
                f"{state}"
            )

            return

    if unmatched_status:
        log("Robot status updated; no matching active mission.")
        if state in {"EXECUTING", "NODE_REACHED", "DISPATCHED"}:
            robot_registry.mark_recovery_required(
                manufacturer,
                robot_id,
                "Robot reports an active order unknown to this Fleet session",
            )
        publish_robot_registry()
        return

    if state == "EXECUTING":
        robot_registry.set_state(
            manufacturer,
            robot_id,
            "EXECUTING",
            current_order=order_id,
            current_node=status.get("currentNode") or node_id,
            current_edge=edge_id,
        )
        publish_robot_registry()
        log("ROBOT IS EXECUTING MISSION")
        return

    if state == "NODE_REACHED":
        if invalid_reached_status:
            publish_robot_registry()
            return

        if mission_completed:
            separator()
            log("MISSION COMPLETED")
            separator()
            log(f"Order ID   : {order_id}")
            log(f"Final node : {final_node}")
            log(f"Total nodes: {total_nodes}")
            log(f"Total edges: {total_edges}")
            separator()
            robot_registry.set_state(
                manufacturer,
                robot_id,
                "IDLE",
                current_node=final_node,
            )
            publish_state(robot_key)
            publish_robot_registry()
            return

        robot_registry.set_state(
            manufacturer,
            robot_id,
            "DISPATCHED",
            current_order=order_id,
            current_node=node_id,
            current_edge=next_edge["edgeId"],
        )
        publish_state(robot_key)
        publish_robot_registry()
        send_robot_command(
            order_id,
            start_node,
            next_node,
            next_edge,
            manufacturer,
            robot_id,
        )
        return

    if state == "STOPPED":
        robot_registry.set_state(
            manufacturer,
            robot_id,
            "STOPPED",
            current_order=order_id,
            current_node=node_id,
            current_edge=edge_id,
        )
        publish_robot_registry()
        publish_state(robot_key)
        return

    if state == "ERROR":
        robot_registry.set_state(
            manufacturer,
            robot_id,
            "ERROR",
            current_order=order_id,
            current_node=node_id,
            current_edge=edge_id,
        )
        publish_robot_registry()
        publish_state(robot_key)
        return

# --------------------
# MQTT CONNECT
# --------------------

def on_connect(
    client,
    userdata,
    flags,
    reason_code,
    properties,
):

    separator()

    log(
        "FLEET CONTROLLER MQTT CONNECTED"
    )

    separator()

    log(f"Broker : {BROKER}")
    log(f"Port   : {PORT}")

    separator()

    if reason_code != 0:

        log(
            f"MQTT connection failed: "
            f"{reason_code}"
        )

        return

    for topic, label in (
        (
            "robot/+/+/status",
            "ROBOT STATUS",
        ),
        (
            "vda5050/v3/+/+/order",
            "VDA-5050 ORDER",
        ),
        (
            "robot/discovery",
            "ROBOT DISCOVERY",
        ),
    ):

        result, message_id = client.subscribe(
            topic,
            qos=0,
        )

        separator()

        log(
            f"SUBSCRIBED TO {label}"
        )

        separator()

        log(f"Topic      : {topic}")
        log(f"Result     : {result}")
        log(f"Message ID : {message_id}")

        separator()

    publish_robot_registry()


# --------------------
# MQTT DISCONNECT
# --------------------

def on_disconnect(
    client,
    userdata,
    disconnect_flags,
    reason_code,
    properties,
):

    separator()

    log(
        "FLEET CONTROLLER MQTT DISCONNECTED"
    )

    separator()

    log(
        f"Reason code: {reason_code}"
    )

    separator()


# --------------------
# MQTT MESSAGE
# --------------------

def on_message(
    client,
    userdata,
    message,
):

    raw_payload = message.payload.decode(
        "utf-8",
        errors="replace",
    )

    separator()

    log("MQTT MESSAGE RECEIVED")

    separator()

    log(f"Topic   : {message.topic}")
    log(f"Payload : {raw_payload}")

    separator()

    try:

        payload = json.loads(
            raw_payload
        )

    except json.JSONDecodeError as error:

        log(
            f"ERROR: Invalid JSON: {error}"
        )

        return

    if message.topic == "robot/discovery":
        robot_registry.discover_world(payload.get("worldRobots", []))
        if not robot_registry.register_agent(payload):
            log("Ignoring robot discovery without a robot ID")
            return
        log(
            f"Robot registered: {payload.get('robotId')} "
            f"({payload.get('manufacturer')}, "
            f"{payload.get('robotType', 'Unknown')})"
        )
        publish_robot_registry()
        return

    if message.topic.endswith("/order"):
        if message.retain:
            log("Ignoring retained VDA-5050 order; it cannot be safely replayed")
            return
        handle_order(payload)

    elif message.topic.endswith("/status"):
        handle_robot_status(payload)


# --------------------
# MQTT CLIENT
# --------------------

mqtt_client = mqtt.Client(
    mqtt.CallbackAPIVersion.VERSION2,
    client_id="fleet-controller",
)

mqtt_client.on_connect = on_connect
mqtt_client.on_disconnect = on_disconnect
mqtt_client.on_message = on_message


# --------------------
# MAIN
# --------------------

def main() -> None:

    separator()

    log("FLEET CONTROLLER")

    separator()

    log(f"Order topic          : vda5050/v3/+/+/order")
    log(f"Robot command topic  : robot/<manufacturer>/<robotId>/command")
    log(f"Robot status topic   : robot/+/+/status")
    log(f"Fleet registry topic : {REGISTRY_TOPIC}")

    separator()

    try:

        mqtt_client.connect(
            BROKER,
            PORT,
            MQTT_KEEPALIVE,
        )

    except Exception as error:

        log(
            f"MQTT CONNECTION ERROR: {error}"
        )

        raise

    mqtt_client.loop_start()

    try:

        while True:

            time.sleep(1)
            if robot_registry.expire_stale_agents():
                publish_robot_registry()

    except KeyboardInterrupt:

        separator()

        log(
            "FLEET CONTROLLER STOPPING"
        )

        separator()

    finally:

        mqtt_client.loop_stop()

        mqtt_client.disconnect()

        separator()

        log(
            "FLEET CONTROLLER STOPPED"
        )

        separator()


# --------------------
# ENTRY POINT
# --------------------

if __name__ == "__main__":

    main()