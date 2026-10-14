from __future__ import annotations

import json
import os
import re
import sys
import tempfile
import threading
import time
import uuid

from datetime import datetime, timezone
from pathlib import Path

import paho.mqtt.client as mqtt

from flask import (
    Flask,
    jsonify,
    render_template,
    request,
)

# --------------------
# PROJECT PATHS
# --------------------

PROJECT_ROOT = Path(
    __file__
).resolve().parents[1]

FLEET_CONTROL = (
    PROJECT_ROOT / "fleet_control"
)

sys.path.insert(
    0,
    str(FLEET_CONTROL),
)


from config import (
    BROKER,
    PORT,
    MANUFACTURER,
    SERIAL_NUMBER,
    ORDER_TOPIC,
    ROBOT_STATUS_TOPIC,
    STATE_TOPIC,
    VDA_VERSION,
    MAP_CONNECTIONS,
    MAP_NODE_POSITIONS,
    SIMULATION_TRANSPORT_ENABLED,
)
from transport_tasks import TransportConflictError, TransportTaskManager


# --------------------
# FLASK
# --------------------

app = Flask(__name__)


# --------------------
# DATA FILE
# --------------------

DATA_FILE = Path(
    __file__
).with_name(
    "orders.json"
)
TRANSPORT_TASKS_FILE = Path(__file__).with_name("transport_tasks.json")


# --------------------
# LOCK
# --------------------

data_lock = threading.Lock()
persistence_lock = threading.Lock()


# --------------------
# DATA
# --------------------

orders = {}

events = []


robots = {}
dispatching_robots = set()
recovery_requests = {}
object_inventory_snapshot = None
object_inventory_received_at = None
OBJECT_INVENTORY_MAX_AGE_SECONDS = 5.0
RECOVERY_ORDER_STATES = {
    "DISPATCHED",
    "EXECUTING",
    "STOPPING",
    "RECOVERY_REQUIRED",
}
transport_task_manager = TransportTaskManager(
    MAP_NODE_POSITIONS,
    TRANSPORT_TASKS_FILE,
)


# --------------------
# TIME
# --------------------

def now_utc():

    return (
        datetime.now(
            timezone.utc
        )
        .isoformat(
            timespec="milliseconds"
        )
        .replace(
            "+00:00",
            "Z",
        )
    )


def robot_status_is_fresh(robot, max_age_seconds=8.0):
    last_update = robot.get("lastUpdate")
    if not last_update:
        return False
    try:
        parsed = datetime.fromisoformat(
            last_update.replace("Z", "+00:00")
        )
    except (AttributeError, TypeError, ValueError):
        return False
    age = (datetime.now(timezone.utc) - parsed).total_seconds()
    return 0 <= age <= max_age_seconds


def robot_dispatchability_reason(robot, allowed_transport_task_id=None):
    if not robot:
        return "Robot is not registered"
    if not robot.get("agentOnline"):
        return "Robot is disconnected"
    if not robot_status_is_fresh(robot):
        return "Robot status is stale"
    if robot.get("recoveryRequired") or robot.get("recoveryPending"):
        return robot.get("recoveryReason") or "Robot requires recovery"
    if transport_task_manager.has_unreleased_load(
        robot.get("manufacturer"),
        robot.get("robotId"),
    ) and not transport_task_manager.task_authorizes_carrier(
        allowed_transport_task_id,
        robot.get("manufacturer"),
        robot.get("robotId"),
    ):
        return "Robot has a verified load reserved for object transport"
    if robot.get("currentOrder"):
        return f"Active mission {robot['currentOrder']}"
    if robot.get("available") is not True:
        return robot.get("availabilityReason") or "Robot is unavailable"
    if robot.get("status") not in {"IDLE", "STOPPED"}:
        return f"Robot is not operational ({robot.get('status', 'UNKNOWN')})"
    if "NAVIGATION" not in robot.get("capabilities", []):
        return "Navigation capability is not implemented"
    return None


def update_object_inventory(payload):
    global object_inventory_snapshot, object_inventory_received_at

    if (
        not isinstance(payload, dict)
        or not isinstance(payload.get("objects"), list)
        or not isinstance(payload.get("relationships"), list)
    ):
        add_event(
            "OBJECT_INVENTORY_ERROR",
            "Fleet published an invalid object inventory snapshot",
        )
        return

    with data_lock:
        object_inventory_snapshot = json.loads(json.dumps(payload))
        object_inventory_received_at = time.monotonic()
        robot_list = [dict(robot) for robot in robots.values()]

    inventory, status_code = get_object_inventory_response()
    if status_code == 200:
        transport_task_manager.observe_inventory(inventory, robot_list)
        transport_task_manager.verify_deliveries(inventory, robot_list)


def get_object_inventory_response():
    with data_lock:
        payload = (
            json.loads(json.dumps(object_inventory_snapshot))
            if object_inventory_snapshot is not None
            else None
        )
        received_at = object_inventory_received_at

    if payload is None:
        return {
            "objects": [],
            "relationships": [],
            "objectCount": 0,
            "relationshipCount": 0,
            "lastRefresh": None,
            "coordinateSystem": "X/Y floor plane; Z height",
            "simulationConnected": False,
            "error": "No object inventory has been received from Webots",
        }, 503

    last_refresh = payload.get("lastRefresh")
    fresh = False
    if received_at is not None and last_refresh:
        try:
            parsed = datetime.fromisoformat(
                last_refresh.replace("Z", "+00:00")
            )
            source_age = (
                datetime.now(timezone.utc) - parsed
            ).total_seconds()
            receive_age = time.monotonic() - received_at
            fresh = (
                0 <= source_age <= OBJECT_INVENTORY_MAX_AGE_SECONDS
                and 0 <= receive_age <= OBJECT_INVENTORY_MAX_AGE_SECONDS
            )
        except (AttributeError, TypeError, ValueError):
            fresh = False

    payload["objectCount"] = len(payload["objects"])
    payload["relationshipCount"] = len(payload["relationships"])
    payload["simulationConnected"] = fresh
    if not fresh:
        payload["error"] = (
            "Webots object inventory is stale; check the simulation supervisor"
        )
    return payload, 200 if fresh else 503


# --------------------
# EVENTS
# --------------------

def add_event(
    event_type,
    message,
    payload=None,
):

    with data_lock:

        events.insert(
            0,
            {
                "timestamp": now_utc(),
                "type": event_type,
                "message": message,
                "payload": payload,
            },
        )

        del events[100:]


# --------------------
# LOAD ORDERS
# --------------------

def load_orders():

    global orders

    if not DATA_FILE.exists():

        return

    try:

        data = json.loads(
            DATA_FILE.read_text(
                encoding="utf-8"
            )
        )

        modified = False
        with data_lock:
            orders = {
                order["orderId"]: order
                for order in data
                if "orderId" in order
            }
            for order in orders.values():
                if order.get("status") in {
                    "DISPATCHED",
                    "EXECUTING",
                    "STOPPING",
                }:
                    order["status"] = "RECOVERY_REQUIRED"
                    order["recoveryReason"] = (
                        "ERP restarted before a terminal robot state was confirmed"
                    )
                    modified = True
        if modified:
            save_orders()

    except Exception as error:

        print(
            f"WARNING: Could not load "
            f"orders.json: {error}"
        )


# --------------------
# SAVE ORDERS
# --------------------

def save_orders():

    with persistence_lock:
        with data_lock:
            payload = json.dumps(list(orders.values()), indent=2)

        DATA_FILE.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                dir=DATA_FILE.parent,
                prefix=f".{DATA_FILE.name}.",
                suffix=".tmp",
                delete=False,
            ) as temporary_file:
                temporary_path = temporary_file.name
                temporary_file.write(payload)
                temporary_file.flush()
                os.fsync(temporary_file.fileno())
            os.replace(temporary_path, DATA_FILE)
        except OSError:
            if temporary_path and os.path.exists(temporary_path):
                os.unlink(temporary_path)
            raise


# --------------------
# ROBOT STATUS
# --------------------

def update_order_from_robot_status(
    status,
):

    order_id = status.get(
        "orderId"
    )

    state = status.get(
        "state"
    )

    node_id = status.get(
        "nodeId"
    )

    with data_lock:

        order = orders.get(
            order_id
        )

        if order is None:
            return

        if state == "EXECUTING":
            order["status"] = "EXECUTING"

        elif state == "NODE_REACHED":
            nodes = order.get("nodes", [])
            final_node = nodes[-1]["nodeId"] if nodes else None

            if node_id == final_node:
                order["status"] = "COMPLETED"
            else:
                order["status"] = "EXECUTING"

        elif state == "STOPPED":
            order["status"] = "STOPPED"

        elif state == "ERROR":
            order["status"] = "FAILED"

        task_state = (
            "COMPLETED"
            if order.get("status") == "COMPLETED"
            else state
        )

    transport_task_manager.update_order_state(
        order_id,
        task_state,
        node_id,
    )
    save_orders()


# --------------------
# VDA STATE
# --------------------

def update_from_vda_state(
    state_payload,
):

    order_id = state_payload.get(
        "orderId"
    )

    last_node = state_payload.get(
        "lastNodeId"
    )

    edge_states = state_payload.get(
        "edgeStates",
        [],
    )

    manufacturer = state_payload.get("manufacturer")
    robot_id = state_payload.get("serialNumber")
    header_id = state_payload.get("headerId")
    changed = False

    with data_lock:
        order = orders.get(order_id)
        robot = robots.get((manufacturer, robot_id))
        if (
            order is None
            or order.get("manufacturer") != manufacturer
            or order.get("serialNumber") != robot_id
            or robot is None
            or robot.get("currentOrder") != order_id
        ):
            return

        try:
            header_id = int(header_id)
        except (TypeError, ValueError):
            return
        if header_id <= order.get("_lastVdaHeaderId", -1):
            return
        order["_lastVdaHeaderId"] = header_id

        nodes = order.get("nodes", [])
        final_node = nodes[-1]["nodeId"] if nodes else None

        if last_node == final_node and not edge_states:
            order["status"] = "COMPLETED"
            order["recoveryReason"] = None
            changed = True

    transport_task_manager.update_order_state(
        order_id,
        "COMPLETED" if order.get("status") == "COMPLETED" else "EXECUTING",
        last_node,
    )
    if changed or order is not None:
        save_orders()


def update_robot_registry(payload):
    entries = payload.get("robots")
    if not isinstance(entries, list):
        add_event("FLEET_ERROR", "Fleet registry payload has no robots list")
        return

    refreshed = {}
    for entry in entries:
        if not isinstance(entry, dict):
            continue
        robot_id = str(entry.get("robotId") or "").strip()
        manufacturer = str(entry.get("manufacturer") or "").strip()
        if robot_id and manufacturer:
            refreshed[(manufacturer, robot_id)] = entry

    orders_changed = False
    recovery_confirmations = []

    with data_lock:
        previous_robots = dict(robots)
        robots.clear()
        robots.update(refreshed)
        for key in dispatching_robots:
            previous_robot = previous_robots.get(key)
            current_robot = robots.get(key)
            if (
                previous_robot is not None
                and current_robot is not None
                and previous_robot.get("currentOrder")
            ):
                current_robot.update(
                    {
                        "status": "DISPATCHED",
                        "available": False,
                        "availabilityReason": (
                            f"Active mission {previous_robot['currentOrder']}"
                        ),
                        "currentOrder": previous_robot["currentOrder"],
                        "lastUpdate": previous_robot["lastUpdate"],
                    }
                )
        for entry in refreshed.values():
            key = (entry["manufacturer"], entry["robotId"])
            status_is_fresh = robot_status_is_fresh(entry)
            requested_order_id = recovery_requests.get(key)
            if requested_order_id:
                recovery_acknowledged = (
                    status_is_fresh
                    and entry.get("status") == "IDLE"
                    and entry.get("currentOrder") is None
                    and entry.get("lastOrderId") == requested_order_id
                    and entry.get("lastOrderStatus") == "INTERRUPTED"
                )
                if recovery_acknowledged:
                    recovery_requests.pop(key, None)
                    recovery_confirmations.append((key, requested_order_id))
                    recovered_order = orders.get(requested_order_id)
                    if recovered_order is not None:
                        recovered_order["status"] = "INTERRUPTED"
                        recovered_order["recoveryReason"] = None
                        recovery_history = recovered_order.setdefault(
                            "recoveryHistory", []
                        )
                        recovery_history.append(
                            {
                                "timestamp": now_utc(),
                                "robotId": key[1],
                                "manufacturer": key[0],
                                "orderId": requested_order_id,
                                "result": "CONFIRMED",
                            }
                        )
                        orders_changed = True
                else:
                    entry["recoveryPending"] = True
                    entry["available"] = False
                    entry["availabilityReason"] = (
                        "Waiting for robot recovery acknowledgement"
                    )

            order_id = entry.get("currentOrder")
            robot_state = entry.get("status")
            if order_id:
                order = orders.get(order_id)
                if (
                    order is not None
                    and (order.get("manufacturer"), order.get("serialNumber"))
                    == key
                    and status_is_fresh
                    and (
                        robot_state in {"DISPATCHED", "EXECUTING", "STOPPING"}
                        or entry.get("recoveryRequired")
                    )
                ):
                    target_status = (
                        "RECOVERY_REQUIRED"
                        if entry.get("recoveryRequired")
                        else robot_state
                    )
                    reason = entry.get("recoveryReason")
                    if (
                        order.get("status") != target_status
                        or order.get("recoveryReason") != reason
                    ):
                        order["status"] = target_status
                        order["recoveryReason"] = reason
                        orders_changed = True
                continue

            last_order_id = entry.get("lastOrderId")
            last_order_status = entry.get("lastOrderStatus")
            last_order = orders.get(last_order_id)
            if (
                last_order is not None
                and (last_order.get("manufacturer"), last_order.get("serialNumber"))
                == key
                and status_is_fresh
                and last_order_status in {
                    "COMPLETED",
                    "STOPPED",
                    "FAILED",
                    "INTERRUPTED",
                }
                and last_order.get("status") != last_order_status
            ):
                last_order["status"] = last_order_status
                last_order["recoveryReason"] = None
                orders_changed = True

            if (
                entry.get("agentOnline")
                and status_is_fresh
                and not entry.get("recoveryRequired")
                and robot_state in {"IDLE", "STOPPED"}
            ):
                for order in orders.values():
                    if (
                        (order.get("manufacturer"), order.get("serialNumber"))
                        == key
                        and order.get("status") in RECOVERY_ORDER_STATES
                    ):
                        order["status"] = "INTERRUPTED"
                        order["recoveryReason"] = (
                            "A fresh robot report confirmed no active assignment; "
                            "the previous mission was not resumed"
                        )
                        orders_changed = True

    if orders_changed:
        save_orders()
    with data_lock:
        task_order_states = [
            (
                order.get("orderId"),
                order.get("status"),
                (
                    robots.get(
                        (order.get("manufacturer"), order.get("serialNumber")),
                        {},
                    ).get("currentNode")
                ),
            )
            for order in orders.values()
            if order.get("orderId")
        ]
    for order_id, order_state, current_node in task_order_states:
        if order_state in {
            "DISPATCHED",
            "EXECUTING",
            "COMPLETED",
            "FAILED",
            "STOPPED",
            "RECOVERY_REQUIRED",
        }:
            transport_task_manager.update_order_state(
                order_id,
                order_state,
                current_node,
            )
    inventory, inventory_status = get_object_inventory_response()
    if inventory_status == 200:
        transport_task_manager.reconcile(
            inventory,
            list(refreshed.values()),
        )
    for key, recovered_order_id in recovery_confirmations:
        add_event(
            "ROBOT_RECOVERY_CONFIRMED",
            f"Robot {key[0]}/{key[1]} confirmed recovery of "
            f"order {recovered_order_id}",
        )


# --------------------
# MQTT CONNECT
# --------------------

def mqtt_on_connect(
    client,
    userdata,
    flags,
    reason_code,
    properties,
):

    if reason_code != 0:

        add_event(
            "MQTT_ERROR",
            f"MQTT connection failed: "
            f"{reason_code}",
        )

        return

    for topic in (
        "fleet/robots",
        "vda5050/v3/+/+/state",
        "warehouse/objects",
    ):
        client.subscribe(topic, qos=0)

    add_event(
        "MQTT_CONNECTED",
        "ERP subscribed to the fleet robot registry and VDA-5050 state",
    )


# --------------------
# MQTT DISCONNECT
# --------------------

def mqtt_on_disconnect(
    client,
    userdata,
    disconnect_flags,
    reason_code,
    properties,
):

    add_event(
        "MQTT_DISCONNECTED",
        f"ERP MQTT disconnected: "
        f"{reason_code}",
    )


# --------------------
# MQTT MESSAGE
# --------------------

def mqtt_on_message(
    client,
    userdata,
    message,
):

    try:

        payload = json.loads(
            message.payload.decode(
                "utf-8",
                errors="replace",
            )
        )

    except json.JSONDecodeError:

        add_event(
            "MQTT_ERROR",
            "Received invalid JSON",
        )

        return

    if message.topic == "fleet/robots":
        update_robot_registry(payload)
        return

    if message.topic == "warehouse/objects":
        update_object_inventory(payload)
        return

    if message.topic.endswith("/state"):
        if getattr(message, "retain", False):
            add_event(
                "VDA_STATE_IGNORED",
                "Ignored retained VDA-5050 state; waiting for live robot telemetry",
            )
            return
        update_from_vda_state(payload)
        add_event(
            "VDA_STATE",
            f"VDA-5050 state: {payload.get('lastNodeId')}",
            payload,
        )


# --------------------
# MQTT CLIENT
# --------------------

mqtt_client = mqtt.Client(
    mqtt.CallbackAPIVersion.VERSION2,
    client_id="erp-frontend",
)

mqtt_client.on_connect = (
    mqtt_on_connect
)

mqtt_client.on_disconnect = (
    mqtt_on_disconnect
)

mqtt_client.on_message = (
    mqtt_on_message
)


# --------------------
# BUILD ORDER
# --------------------

def build_order(
    order_id,
    robot_id,
    priority,
    route_nodes,
    planning_mode,
    requested_route,
    destination,
    manufacturer,
):

    nodes = [
        {
            "nodeId": node_id,
            "sequenceId": index * 2,
            "released": True,
            "actions": [],
        }
        for index, node_id
        in enumerate(route_nodes)
    ]

    edges = [
        {
            "edgeId": f"E{index + 1}",
            "sequenceId": index * 2 + 1,
            "released": True,
            "actions": [],
        }
        for index in range(
            len(route_nodes) - 1
        )
    ]

    return {
        "headerId": 0,
        "timestamp": now_utc(),
        "version": VDA_VERSION,
        "manufacturer": manufacturer,
        "serialNumber": robot_id,
        "orderId": order_id,
        "orderUpdateId": 0,
        "nodes": nodes,
        "edges": edges,
        "erpPriority": priority,
        "erpStatus": "CREATED",
        "status": "CREATED",
        "erpPlanningMode": planning_mode,
        "erpRoute": requested_route,
        "erpDestination": destination,
    }


def shortest_node_path(start_node, end_node):
    """Return the least-distance route through the configured map graph."""
    if start_node not in MAP_NODE_POSITIONS:
        raise ValueError(f"Unknown start node: {start_node}")
    if end_node not in MAP_NODE_POSITIONS:
        raise ValueError(f"Unknown destination node: {end_node}")

    adjacency = {node_id: [] for node_id in MAP_NODE_POSITIONS}
    for first, second in MAP_CONNECTIONS:
        first_position = MAP_NODE_POSITIONS[first]
        second_position = MAP_NODE_POSITIONS[second]
        distance = (
            (first_position[0] - second_position[0]) ** 2
            + (first_position[1] - second_position[1]) ** 2
        ) ** 0.5
        adjacency[first].append((second, distance))
        adjacency[second].append((first, distance))

    distances = {start_node: 0.0}
    previous = {}
    unvisited = set(adjacency)

    while unvisited:
        current = min(
            unvisited,
            key=lambda node_id: (
                distances.get(node_id, float("inf")),
                node_id,
            ),
        )
        if distances.get(current, float("inf")) == float("inf"):
            break
        if current == end_node:
            break

        unvisited.remove(current)
        for neighbor, edge_distance in adjacency[current]:
            candidate = distances[current] + edge_distance
            if candidate < distances.get(neighbor, float("inf")):
                distances[neighbor] = candidate
                previous[neighbor] = current

    if end_node not in distances:
        raise ValueError(
            f"No map path exists from {start_node} to {end_node}"
        )

    path = [end_node]
    while path[-1] != start_node:
        path.append(previous[path[-1]])
    path.reverse()
    return path


def plan_order_nodes(start_node, route_nodes, destination):
    """Route through required waypoints, then add destination as a direct leg."""
    if not route_nodes and not destination:
        raise ValueError("Select a route, a destination, or both")

    planned_nodes = [start_node]
    current_node = start_node
    for stop in route_nodes:
        leg = shortest_node_path(current_node, stop)
        planned_nodes.extend(leg[1:])
        current_node = stop

    if destination and destination != current_node:
        planned_nodes.append(destination)

    return planned_nodes


# --------------------
# HOME
# --------------------

@app.get("/")
def index():

    return render_template(
        "index.html"
    )


# --------------------
# DASHBOARD
# --------------------

@app.get("/api/dashboard")
def dashboard():

    with data_lock:

        order_list = list(orders.values())
        robot_list = []
        for robot in robots.values():
            robot_view = dict(robot)
            reason = robot_dispatchability_reason(robot_view)
            robot_view["available"] = reason is None
            if reason:
                robot_view["availabilityReason"] = reason
            robot_list.append(robot_view)
        event_list = list(events)

    return jsonify(
        {
            "orders": order_list,
            "robots": robot_list,
            "transportTasks": transport_task_manager.list_tasks(),
            "events": event_list,
            "summary": {
                "totalOrders": len(order_list),
                "activeOrders": len(
                    {
                        robot.get("currentOrder")
                        for robot in robot_list
                        if robot.get("currentOrder")
                    }
                ),
                "completedOrders": sum(
                    order.get("status") == "COMPLETED"
                    for order in order_list
                ),
                "failedOrders": sum(
                    order.get("status") == "FAILED"
                    for order in order_list
                ),
                "availableRobots": sum(
                    robot.get("available") is True
                    for robot in robot_list
                ),
            },
        }
    )


@app.get("/api/transport-options")
def transport_options():
    inventory, status_code = get_object_inventory_response()
    if status_code != 200:
        return jsonify(inventory), status_code
    with data_lock:
        carrier_list = [
            dict(robot)
            for robot in robots.values()
        ]
    return jsonify(
        {
            "objects": inventory.get("objects", []),
            "destinations": [
                {"nodeId": node_id, "position": {"x": position[0], "y": position[1]}}
                for node_id, position in MAP_NODE_POSITIONS.items()
            ],
            "carriers": carrier_list,
            "simulationTransportEnabled": SIMULATION_TRANSPORT_ENABLED,
            "handlingModes": ["PRELOADED"],
        }
    )


@app.get("/api/transport-tasks")
def list_transport_tasks():
    return jsonify({"tasks": transport_task_manager.list_tasks()})


@app.get("/api/transport-tasks/<task_id>")
def get_transport_task(task_id):
    task = transport_task_manager.get(task_id)
    if task is None:
        return jsonify({"error": "Transport task not found"}), 404
    return jsonify(task)


@app.post("/api/transport-tasks")
def create_transport_task():
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        return jsonify({"error": "A JSON transport task request is required"}), 400
    if payload.get("taskType") != "TRANSPORT_OBJECT":
        return jsonify({"error": "taskType must be TRANSPORT_OBJECT"}), 400
    if payload.get("handlingMode") != "PRELOADED":
        return jsonify(
            {"error": "Only PRELOADED transport is supported; no handling robot is configured"}
        ), 400

    object_id = str(payload.get("objectId") or "").strip()
    destination = str(payload.get("destinationNode") or "").strip()
    if not object_id:
        return jsonify({"error": "objectId is required"}), 400
    if destination not in MAP_NODE_POSITIONS:
        return jsonify({"error": f"Unknown destination node: {destination or '(empty)'}"}), 400

    inventory, status_code = get_object_inventory_response()
    if status_code != 200:
        return jsonify(inventory), status_code
    object_record = next(
        (
            item
            for item in inventory["objects"]
            if item.get("object_id") == object_id
        ),
        None,
    )
    if object_record is None:
        return jsonify({"error": f"Object not found in live inventory: {object_id}"}), 404

    carrier_id = str(payload.get("carrierRobotId") or "").strip() or None
    manufacturer = str(payload.get("manufacturer") or "").strip() or None
    with data_lock:
        carrier_list = [dict(robot) for robot in robots.values()]
    if carrier_id and not any(
        robot.get("robotId") == carrier_id
        and (manufacturer is None or robot.get("manufacturer") == manufacturer)
        for robot in carrier_list
    ):
        return jsonify({"error": f"Unknown carrier robot: {carrier_id}"}), 404

    try:
        transport_task_manager.observe_inventory(inventory, carrier_list)
        task = transport_task_manager.create_task(
            object_record,
            destination,
            carrier_list,
            carrier_id,
            manufacturer,
        )
    except TransportConflictError as error:
        return jsonify({"error": str(error)}), 409
    except OSError as error:
        add_event("TRANSPORT_TASK_PERSISTENCE_ERROR", str(error))
        return jsonify({"error": "Could not persist transport task"}), 503
    except ValueError as error:
        add_event("TRANSPORT_TASK_ERROR", str(error))
        return jsonify({"error": str(error)}), 400
    add_event(
        "TRANSPORT_TASK_CREATED",
        f"Transport task {task['taskId']} created with state {task['status']}",
        task,
    )
    return jsonify(task), 201


@app.post("/api/transport-tasks/<task_id>/dispatch")
def dispatch_transport_task(task_id):
    task = transport_task_manager.get(task_id)
    if task is None:
        return jsonify({"error": "Transport task not found"}), 404
    if task.get("status") != "READY_FOR_DISPATCH":
        return jsonify(
            {
                "error": task.get("message")
                or f"Transport task cannot dispatch from {task.get('status')}",
                "task": task,
            }
        ), 409

    inventory, status_code = get_object_inventory_response()
    if status_code != 200:
        return jsonify(inventory), status_code
    object_record = next(
        (
            item
            for item in inventory["objects"]
            if item.get("object_id") == task.get("objectId")
        ),
        None,
    )
    if object_record is None:
        return jsonify({"error": "Transport object is absent from live inventory"}), 409

    robot_key = (task.get("manufacturer"), task.get("carrierRobotId"))
    with data_lock:
        carrier = robots.get(robot_key)
        carrier = dict(carrier) if carrier is not None else None
    if carrier is None:
        return jsonify({"error": "Verified carrier is not in the current fleet"}), 409
    reason = robot_dispatchability_reason(carrier, task_id)
    if reason:
        return jsonify({"error": f"Carrier is unavailable: {reason}"}), 409
    start_node = carrier.get("currentNode")
    if not start_node:
        return jsonify({"error": "Carrier location is not available yet"}), 409
    try:
        route_nodes = plan_order_nodes(
            start_node,
            [],
            task["destinationNode"],
        )
    except ValueError as error:
        return jsonify({"error": str(error)}), 400
    if len(route_nodes) < 2:
        return jsonify({"error": "Carrier is already at the destination"}), 409
    if not transport_task_manager.reserve(task_id, object_record, carrier):
        return jsonify(
            {"error": "Load, carrier, or task reservation changed; refresh and retry"}
        ), 409

    order_id = f"transport-{uuid.uuid4().hex}"
    order = build_order(
        order_id,
        carrier["robotId"],
        "NORMAL",
        route_nodes,
        "destination",
        [],
        task["destinationNode"],
        carrier["manufacturer"],
    )
    with data_lock:
        if order_id in orders:
            transport_task_manager.rollback_dispatch(task_id)
            return jsonify({"error": "Generated transport order ID already exists"}), 409
        orders[order_id] = order
    try:
        save_orders()
    except OSError as error:
        with data_lock:
            orders.pop(order_id, None)
        transport_task_manager.rollback_dispatch(task_id)
        add_event("ORDER_PERSISTENCE_ERROR", str(error))
        return jsonify({"error": "Could not persist transport order"}), 503

    if not transport_task_manager.attach_order(task_id, order_id):
        transport_task_manager.mark_recovery_required(
            task_id,
            "Could not correlate the transport task with its persisted navigation order",
        )
        return jsonify({"error": "Could not reserve transport task for dispatch"}), 503

    dispatch_result = dispatch_order(order_id)
    if isinstance(dispatch_result, tuple):
        response, dispatch_status = dispatch_result
    else:
        response, dispatch_status = dispatch_result, dispatch_result.status_code
    if dispatch_status == 200:
        if not transport_task_manager.mark_dispatched(task_id):
            transport_task_manager.mark_recovery_required(
                task_id,
                "Navigation was dispatched but task state could not be persisted",
            )
            return jsonify({"error": "Dispatch outcome requires recovery"}), 503
        transport_task_manager.update_order_state(order_id, "DISPATCHED")
        return jsonify(
            {
                "task": transport_task_manager.get(task_id),
                "order": order,
            }
        ), 202
    dispatch_error = (
        response.get_json(silent=True) or {}
    ).get("error", "")
    safe_dispatch_rejection = (
        dispatch_error.startswith("MQTT publish failed:")
        or dispatch_error == "Could not persist order before dispatch"
    )
    if dispatch_status >= 500 and not safe_dispatch_rejection:
        transport_task_manager.mark_recovery_required(
            task_id,
            "Order publish or persistence outcome is uncertain; load association retained",
        )
    else:
        transport_task_manager.rollback_dispatch(task_id)
        with data_lock:
            order["status"] = "FAILED"
            order["dispatchError"] = "Transport dispatch was rejected before movement"
        save_orders()
    return response, dispatch_status


@app.get("/api/objects")
def list_objects():
    payload, status_code = get_object_inventory_response()
    return jsonify(payload), status_code


@app.get("/api/objects/relationships")
def list_object_relationships():
    payload, status_code = get_object_inventory_response()
    return jsonify(
        {
            "relationships": payload.get("relationships", []),
            "lastRefresh": payload.get("lastRefresh"),
            "simulationConnected": payload.get("simulationConnected", False),
            **({"error": payload["error"]} if "error" in payload else {}),
        }
    ), status_code


@app.get("/api/objects/<object_id>")
def get_object(object_id):
    if not re.fullmatch(r"[A-Za-z0-9_.:-]{1,128}", object_id):
        return jsonify({"error": "Invalid object ID"}), 400

    payload, status_code = get_object_inventory_response()
    record = next(
        (
            item
            for item in payload.get("objects", [])
            if item.get("object_id") == object_id
        ),
        None,
    )
    if status_code != 200:
        return jsonify({"error": payload["error"], "object": record}), status_code
    if record is None:
        return jsonify({"error": "Object not found"}), 404
    return jsonify(record)


@app.post("/api/objects/refresh")
def refresh_objects():
    if not mqtt_client.is_connected():
        return jsonify(
            {"error": "Cannot request inventory refresh: MQTT is disconnected"}
        ), 503

    try:
        result = mqtt_client.publish(
            "warehouse/objects/refresh",
            json.dumps({"requestedAt": now_utc()}),
            qos=0,
        )
    except Exception as error:
        add_event(
            "OBJECT_INVENTORY_ERROR",
            f"Could not request Webots inventory refresh: {error}",
        )
        return jsonify({"error": "Could not request inventory refresh"}), 503

    if result.rc != mqtt.MQTT_ERR_SUCCESS:
        return jsonify(
            {"error": f"Could not request inventory refresh: {result.rc}"}
        ), 503

    return jsonify(
        {
            "status": "REFRESH_REQUESTED",
            "message": "Webots will publish the next inventory snapshot shortly",
        }
    ), 202


# --------------------
# CREATE ORDER
# --------------------

@app.post("/api/orders")
def create_order():

    payload = (
        request.get_json(
            silent=True
        )
        or {}
    )

    order_id = str(
        payload.get(
            "orderId",
            "",
        )
    ).strip()

    robot_id = str(
        payload.get(
            "robotId",
            "",
        )
    ).strip()
    manufacturer = str(payload.get("manufacturer", "")).strip()

    priority = str(
        payload.get(
            "priority",
            "NORMAL",
        )
    ).upper()

    planning_mode = str(
        payload.get("planningMode", "route")
    ).lower()
    route = payload.get("route", [])
    destination = str(payload.get("destination", "")).strip()

    if not order_id:

        return jsonify(
            {
                "error":
                    "orderId is required"
            }
        ), 400

    with data_lock:
        selected_robot = robots.get((manufacturer, robot_id))

    if selected_robot is None:
        return jsonify({"error": "Unknown robot selection"}), 400

    reason = robot_dispatchability_reason(selected_robot)
    if reason:
        return jsonify(
            {
                "error": f"Robot {robot_id} is unavailable: {reason}"
            }
        ), 409

    if planning_mode not in {"route", "destination", "both"}:
        return jsonify({"error": "Invalid planning mode"}), 400

    if not isinstance(route, list):
        return jsonify({"error": "Route must be a list of node IDs"}), 400

    route = [
        str(node).strip()
        for node in route
        if str(node).strip()
    ]

    if planning_mode in {"route", "both"} and not route:
        return jsonify(
            {"error": "Enter at least one required route node"}
        ), 400
    if planning_mode in {"destination", "both"} and not destination:
        return jsonify({"error": "Enter a destination node"}), 400
    if planning_mode == "route" and destination:
        return jsonify(
            {"error": "Destination is not used in route-only mode"}
        ), 400
    if planning_mode == "destination":
        route = []

    requested_nodes = route + (
        [destination]
        if destination
        else []
    )
    unknown_nodes = [
        node_id
        for node_id in requested_nodes
        if node_id not in MAP_NODE_POSITIONS
    ]
    if unknown_nodes:
        return jsonify(
            {
                "error":
                    f"Unknown map node(s): "
                    f"{', '.join(unknown_nodes)}"
            }
        ), 400

    with data_lock:
        start_node = selected_robot.get("currentNode")

    if not start_node:
        return jsonify(
            {
                "error": (
                    "Robot location is not available yet. "
                    "Wait for robot telemetry and refresh."
                )
            }
        ), 409

    try:
        planned_nodes = plan_order_nodes(
            start_node,
            route,
            destination,
        )
    except ValueError as error:
        return jsonify({"error": str(error)}), 400

    if len(planned_nodes) < 2:
        return jsonify(
            {
                "error":
                    "The selected destination is already "
                    "the robot's current node"
            }
        ), 400

    with data_lock:

        if order_id in orders:

            return jsonify(
                {
                    "error":
                        "Order already exists"
                }
            ), 409

        order = build_order(
            order_id,
            robot_id,
            priority,
            planned_nodes,
            planning_mode,
            route,
            destination or None,
            manufacturer,
        )

        orders[order_id] = order

    save_orders()

    add_event(
        "ORDER_CREATED",
        f"Order {order_id} created",
        order,
    )

    return jsonify(order), 201


# --------------------
# DISPATCH ORDER
# --------------------

@app.post(
    "/api/orders/<order_id>/dispatch"
)
def dispatch_order(
    order_id
):

    with data_lock:
        order = orders.get(order_id)
        if order is None:
            return jsonify({"error": "Order not found"}), 404
        if order.get("status") not in {"CREATED", "FAILED", "STOPPED"}:
            return jsonify(
                {"error": f"Order cannot be dispatched from {order['status']}"}
            ), 409

        robot_key = (order["manufacturer"], order["serialNumber"])
        selected_robot = robots.get(robot_key)
        transport_task_id = transport_task_manager.task_id_for_order(order_id)
        reason = robot_dispatchability_reason(
            selected_robot,
            transport_task_id,
        )
        if reason:
            return jsonify(
                {"error": f"Selected robot is unavailable: {reason}"}
            ), 409
        if robot_key in dispatching_robots:
            return jsonify(
                {"error": "Selected robot already has an active order"}
            ), 409

        previous_robot = dict(selected_robot)
        previous_status = order.get("status")
        order["status"] = "DISPATCHED"
        order["dispatchError"] = None
        selected_robot["status"] = "DISPATCHED"
        selected_robot["available"] = False
        selected_robot["availabilityReason"] = f"Active mission {order_id}"
        selected_robot["currentOrder"] = order_id
        selected_robot["lastUpdate"] = now_utc()
        dispatching_robots.add(robot_key)

    try:
        save_orders()
    except OSError as error:
        with data_lock:
            dispatching_robots.discard(robot_key)
            if orders.get(order_id) is order:
                order["status"] = previous_status
            robots[robot_key] = previous_robot
        add_event(
            "ORDER_PERSISTENCE_ERROR",
            f"Could not persist order {order_id} before dispatch: {error}",
        )
        return jsonify({"error": "Could not persist order before dispatch"}), 503

    try:
        result = mqtt_client.publish(
            f"vda5050/v3/{order['manufacturer']}/{order['serialNumber']}/order",
            json.dumps(order),
            qos=1,
        )
    except Exception as error:
        with data_lock:
            dispatching_robots.discard(robot_key)
            selected_robot = robots.get(robot_key)
            if selected_robot is not None:
                selected_robot["recoveryRequired"] = True
                selected_robot["recoveryReason"] = (
                    "Order publish outcome is uncertain; await robot recovery"
                )
                selected_robot["available"] = False
            order["status"] = "RECOVERY_REQUIRED"
            order["recoveryReason"] = (
                "Order publish outcome is uncertain; robot acknowledgement is required"
            )
        save_orders()
        add_event(
            "ORDER_DISPATCH_UNCERTAIN",
            f"MQTT publish outcome for {order_id} is uncertain: {error}",
        )
        return jsonify(
            {"error": "MQTT publish outcome is uncertain; robot recovery is required"}
        ), 503

    if (
        result.rc
        != mqtt.MQTT_ERR_SUCCESS
    ):
        with data_lock:
            dispatching_robots.discard(robot_key)
            if orders.get(order_id) is order:
                order["status"] = previous_status
                order["dispatchError"] = f"MQTT publish failed: {result.rc}"
            current_robot = robots.get(robot_key)
            if (
                current_robot is not None
                and current_robot.get("currentOrder") == order_id
                and current_robot.get("status") == "DISPATCHED"
            ):
                robots[robot_key] = previous_robot
        try:
            save_orders()
        except OSError as error:
            add_event(
                "ORDER_PERSISTENCE_ERROR",
                f"Could not persist dispatch rollback for {order_id}: {error}",
            )

        return jsonify(
            {
                "error": f"MQTT publish failed: {result.rc}"
            }
        ), 503

    with data_lock:
        dispatching_robots.discard(robot_key)
        order["erpStatus"] = "DISPATCHED"
    try:
        save_orders()
    except OSError as error:
        with data_lock:
            order["status"] = "RECOVERY_REQUIRED"
            order["recoveryReason"] = (
                "Order was published but its dispatch update could not be persisted"
            )
            active_robot = robots.get(robot_key)
            if active_robot is not None:
                active_robot["available"] = False
                active_robot["recoveryRequired"] = True
                active_robot["recoveryReason"] = order["recoveryReason"]
        add_event(
            "ORDER_PERSISTENCE_ERROR",
            f"Order {order_id} was published but its dispatch update could not be saved: {error}",
        )
        try:
            save_orders()
        except OSError as recovery_error:
            add_event(
                "ORDER_PERSISTENCE_ERROR",
                f"Could not persist recovery state for {order_id}: {recovery_error}",
            )
        return jsonify(
            {
                "error": order["recoveryReason"],
                "orderId": order_id,
            }
        ), 503

    add_event(
        "ORDER_DISPATCHED",
        f"Order {order_id} published "
        "to VDA-5050 order topic",
        order,
    )

    return jsonify(order)


# --------------------
# STOP ROBOT
# --------------------

@app.post(
    "/api/robots/<manufacturer>/<robot_id>/stop"
)
def stop_robot(
    manufacturer,
    robot_id
):

    with data_lock:
        robot_entry = robots.get((manufacturer, robot_id))

    if robot_entry is None:
        return jsonify({"error": "Unknown robot"}), 404

    with data_lock:
        current_status = robot_entry["status"]
        current_order_id = robot_entry["currentOrder"]
        current_node = robot_entry["currentNode"]
        current_edge = robot_entry["currentEdge"]

    # --------------------
    # IDEMPOTENT STOP
    # --------------------

    if current_status == "STOPPED":

        add_event(
            "ROBOT_STOP",
            f"Robot {manufacturer}/{robot_id} "
            "is already stopped",
        )

        return jsonify(
            {
                "robotId": robot_id,
                "status": "STOPPED",
                "orderId": current_order_id,
                "nodeId": current_node,
                "edgeId": current_edge,
                "message":
                    "Robot is already stopped",
            }
        )

    # --------------------
    # CHECK ACTIVE STATE
    # --------------------

    if current_status not in {"EXECUTING", "DISPATCHED"}:

        return jsonify(
            {
                "error":
                    f"Robot cannot be stopped "
                    f"from state "
                    f"{current_status}"
            }
        ), 409

    # --------------------
    # STOP COMMAND
    # --------------------

    command = {
        "command": "STOP_ROBOT",
        "robotId": robot_id,
        "reason": "USER_REQUEST",
    }

    result = mqtt_client.publish(
        f"robot/{robot_entry['manufacturer']}/{robot_id}/command",
        json.dumps(command),
        qos=0,
    )

    if (
        result.rc
        != mqtt.MQTT_ERR_SUCCESS
    ):

        add_event(
            "ROBOT_STOP_ERROR",
            f"Failed to send stop command "
            f"to {manufacturer}/{robot_id}",
        )

        return jsonify(
            {
                "error":
                    "Failed to send "
                    "STOP_ROBOT command"
            }
        ), 503

    # --------------------
    # OPTIMISTIC ERP STATE
    # --------------------

    with data_lock:

        robot_entry["status"] = "STOPPING"
        robot_entry["lastUpdate"] = now_utc()
        robot_entry["recoveryRequired"] = bool(current_order_id)
        robot_entry["recoveryReason"] = (
            "Operator requested a stop; recovery is required to resume"
            if current_order_id
            else "Robot was stopped by operator request"
        )
        robot_entry["available"] = False

        if current_order_id:
            order = orders.get(current_order_id)
            if order is not None:
                order["status"] = "STOPPING"
                order["recoveryReason"] = robot_entry["recoveryReason"]

    save_orders()

    add_event(
        "ROBOT_STOP_REQUESTED",
        f"Stop requested for "
        f"Robot {manufacturer}/{robot_id}",
        command,
    )

    return jsonify(
        {
            "robotId": robot_id,
            "status": "STOPPING",
            "orderId": current_order_id,
            "nodeId": current_node,
            "edgeId": current_edge,
            "message":
                "Stop command sent",
        }
    )


@app.post("/api/robots/<manufacturer>/<robot_id>/recover")
def recover_robot(manufacturer, robot_id):
    payload = request.get_json(silent=True) or {}
    reason = str(payload.get("reason") or "").strip()
    confirm_safe = payload.get("confirmSafe") is True
    robot_key = (manufacturer, robot_id)

    if not confirm_safe or not reason:
        return jsonify(
            {"error": "Confirm the robot is safe to recover and provide a reason"}
        ), 400

    with data_lock:
        robot = robots.get(robot_key)
        if robot is None:
            return jsonify({"error": "Unknown robot"}), 404
        if not robot.get("agentOnline") or not robot_status_is_fresh(robot):
            return jsonify(
                {
                    "error": (
                        "Robot is disconnected or its status is stale. "
                        "Assignment was not cleared."
                    )
                }
            ), 409
        if not robot.get("recoveryRequired"):
            return jsonify(
                {"error": "Robot does not currently require recovery"}
            ), 409

        order_id = robot.get("currentOrder") or robot.get("lastOrderId")
        if not order_id:
            return jsonify(
                {"error": "No recoverable order is associated with this robot"}
            ), 409
        order = orders.get(order_id)
        if order is None or (
            order.get("manufacturer"), order.get("serialNumber")
        ) != robot_key:
            return jsonify(
                {
                    "error": (
                        "The assigned order is not present in order history; "
                        "assignment was not cleared."
                    )
                }
            ), 409
        if (
            robot_key in recovery_requests
            and recovery_requests[robot_key] != order_id
        ):
            return jsonify(
                {"error": "A different order is already being recovered"}
            ), 409

        command = {
            "command": "RECOVER_ROBOT",
            "robotId": robot_id,
            "orderId": order_id,
            "reason": reason,
        }
        first_request = robot_key not in recovery_requests
        recovery_requests[robot_key] = order_id
        robot["recoveryPending"] = True
        robot["availabilityReason"] = (
            "Waiting for robot recovery acknowledgement"
        )
        robot["available"] = False
        if first_request:
            order.setdefault("recoveryHistory", []).append(
                {
                    "timestamp": now_utc(),
                    "robotId": robot_id,
                    "manufacturer": manufacturer,
                    "orderId": order_id,
                    "reason": reason,
                    "result": "REQUESTED",
                }
            )

    try:
        save_orders()
        result = mqtt_client.publish(
            f"robot/{manufacturer}/{robot_id}/command",
            json.dumps(command),
            qos=1,
        )
    except Exception as error:
        with data_lock:
            recovery_requests.pop(robot_key, None)
            robot = robots.get(robot_key)
            if robot is not None:
                robot["recoveryPending"] = False
                robot["available"] = False
        add_event(
            "ROBOT_RECOVERY_ERROR",
            f"Failed to request recovery for {manufacturer}/{robot_id}: {error}",
        )
        return jsonify({"error": f"Recovery command failed: {error}"}), 503

    if result.rc != mqtt.MQTT_ERR_SUCCESS:
        with data_lock:
            recovery_requests.pop(robot_key, None)
            robot = robots.get(robot_key)
            if robot is not None:
                robot["recoveryPending"] = False
                robot["available"] = False
        return jsonify(
            {"error": f"Recovery command publish failed: {result.rc}"}
        ), 503

    add_event(
        "ROBOT_RECOVERY_REQUESTED",
        f"Recovery requested for {manufacturer}/{robot_id}, order {order_id}",
        command,
    )
    return jsonify(
        {
            "robotId": robot_id,
            "orderId": order_id,
            "status": "RECOVERY_PENDING",
            "message": "Waiting for robot recovery acknowledgement",
        }
    ), 202


# --------------------
# EVENTS
# --------------------

@app.get("/api/events")
def get_events():

    with data_lock:

        return jsonify(
            events
        )


# --------------------
# MQTT START
# --------------------

def start_mqtt():

    try:

        mqtt_client.connect(
            BROKER,
            PORT,
            60,
        )

        mqtt_client.loop_start()

    except Exception as error:

        print(
            f"MQTT CONNECTION ERROR: "
            f"{error}"
        )

        add_event(
            "MQTT_ERROR",
            str(error),
        )


# --------------------
# STARTUP
# --------------------

if __name__ == "__main__":

    load_orders()
    transport_task_manager.load()
    start_mqtt()

    app.run(
        host="0.0.0.0",
        port=5000,
        debug=False,
        use_reloader=False,
    )