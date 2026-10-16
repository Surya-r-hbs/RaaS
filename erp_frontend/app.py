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
from manipulation_tasks import ManipulationTaskService
from order_planning import (
    build_order,
    plan_order_nodes,
    plan_robot_order_nodes,
    shortest_node_path,
)

if __name__ == "__main__":
    # Route modules import "app"; alias the script module to avoid loading it twice.
    sys.modules.setdefault("app", sys.modules[__name__])

from routes.manipulation import bp as manipulation_routes
from time_utils import now_utc


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
MANIPULATION_TASKS_FILE = Path(__file__).with_name("manipulation_tasks.json")

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
logs = []

LOG_CATEGORIES = {
    "SYSTEM": "System & Simulation",
    "MQTT": "MQTT & Messaging",
    "ROBOTS": "Robots & Navigation",
    "ORDERS": "Orders & Tasks",
    "OBJECTS": "Objects & Inventory",
    "ERP": "ERP & API",
}
LOG_SEVERITY_LEVELS = ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]


def normalize_log_category(category):
    text = str(category or "ERP").strip().upper()
    if text in LOG_CATEGORIES:
        return text

    if "MQTT" in text or "MESSAGE" in text or "BROKER" in text:
        return "MQTT"
    if "ROBOT" in text or "NAVIGATION" in text or "DISCOVERY" in text or "STATE" in text or "STATUS" in text:
        return "ROBOTS"
    if "ORDER" in text or "TASK" in text or "DISPATCH" in text:
        return "ORDERS"
    if "OBJECT" in text or "INVENTORY" in text or "RELATION" in text:
        return "OBJECTS"
    if "APP" in text or "SYSTEM" in text or "SIMULATION" in text or "SUPERVISOR" in text:
        return "SYSTEM"

    return "ERP"


def normalize_severity(severity):
    text = str(severity or "INFO").strip().upper()
    return text if text in LOG_SEVERITY_LEVELS else "INFO"


def build_log_entry(category, message, severity="INFO", source=None, payload=None, **metadata):
    category_name = normalize_log_category(category)
    return {
        "timestamp": now_utc(),
        "category": category_name,
        "categoryLabel": LOG_CATEGORIES.get(category_name, category_name.title()),
        "severity": normalize_severity(severity),
        "source": str(source or "erp").strip() or "erp",
        "message": str(message),
        "payload": payload,
        **metadata,
    }


def log_event(category, message, severity="INFO", source=None, payload=None, **metadata):
    entry = build_log_entry(
        category, message, severity=severity, source=source, payload=payload, **metadata
    )
    with data_lock:
        logs.insert(0, entry)
        del logs[300:]
    return entry


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
manipulation_task_service = ManipulationTaskService(
    MANIPULATION_TASKS_FILE,
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
    event_severity = "ERROR"
    event_text = str(event_type or "EVENT").upper()
    if any(token in event_text for token in ("ERROR", "FAILED", "DISCONNECTED", "INVALID", "STALE")):
        event_severity = "ERROR"
    elif any(token in event_text for token in ("WARNING", "RECOVERY", "INTERRUPTED")):
        event_severity = "WARNING"
    elif any(token in event_text for token in ("CONNECTED", "COMPLETED", "CREATED")):
        event_severity = "INFO"
    else:
        event_severity = "INFO"

    log_payload = build_log_entry(
        category=event_type,
        message=message,
        severity=event_severity,
        source=event_type,
        payload=payload,
        type=event_type,
    )

    with data_lock:
        logs.insert(0, log_payload)
        del logs[300:]
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


def get_manipulation_erp_state():
    return {
        "get_object_inventory_response": get_object_inventory_response,
        "robot_status_is_fresh": robot_status_is_fresh,
        "manipulation_task_service": manipulation_task_service,
        "data_lock": data_lock,
        "robots": robots,
        "add_event": add_event,
    }


app.config["ERP_STATE"] = get_manipulation_erp_state

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
# HOME
# --------------------



# --------------------
# DASHBOARD
# --------------------























# --------------------
# CREATE ORDER
# --------------------



# --------------------
# DISPATCH ORDER
# --------------------



# --------------------
# STOP ROBOT
# --------------------





# --------------------
# EVENTS
# --------------------



from routes.dashboard import bp as dashboard_routes, index, dashboard, get_logs, get_events
from routes.transport import bp as transport_routes, transport_options, list_transport_tasks, get_transport_task, create_transport_task, dispatch_transport_task, list_objects, list_object_relationships, get_object, refresh_objects
from routes.orders import bp as order_routes, create_order, dispatch_order
from routes.robots import bp as robot_routes, stop_robot, recover_robot

app.register_blueprint(dashboard_routes)
app.register_blueprint(transport_routes)
app.register_blueprint(order_routes)
app.register_blueprint(robot_routes)
app.register_blueprint(manipulation_routes)

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
    manipulation_task_service.load()
    start_mqtt()

    app.run(
        host="0.0.0.0",
        port=5000,
        debug=False,
        use_reloader=False,
    )