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
import os
import sys
import threading
import time

from datetime import datetime, timezone
from pathlib import Path

import paho.mqtt.client as mqtt

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from raas_logging import get_logger, log_event

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
from load_verification import annotate_load_verification
from object_registry import ObjectRegistry
from robot_registry import RobotRegistry
from mqtt_messages import route_message
from mqtt_lifecycle import handle_connect, handle_disconnect
from robot_dispatch import send_robot_command as dispatch_robot_command
from vda5050_state import publish_state as publish_vda_state
from robot_status_handler import handle_robot_status as process_robot_status
from mission_lifecycle import (
    handle_order as process_order,
    start_next_segment as process_next_segment,
)


# --------------------
# GLOBAL MISSION
# --------------------

missions: dict[tuple[str, str], Mission] = {}
robot_registry = RobotRegistry()
object_registry = ObjectRegistry()

state_lock = threading.Lock()
REGISTRY_TOPIC = "fleet/robots"
OBJECTS_RAW_TOPIC = "warehouse/objects/raw"
OBJECTS_TOPIC = "warehouse/objects"


def publish_robot_registry() -> None:
    result = mqtt_client.publish(
        REGISTRY_TOPIC,
        json.dumps({"robots": robot_registry.snapshot()}),
        qos=1,
        retain=True,
    )
    if result.rc != mqtt.MQTT_ERR_SUCCESS:
        log(f"ERROR: Failed to publish robot registry: {result.rc}")


def publish_object_registry() -> None:
    result = mqtt_client.publish(
        OBJECTS_TOPIC,
        json.dumps(object_registry.snapshot()),
        qos=1,
        retain=True,
    )
    if result.rc != mqtt.MQTT_ERR_SUCCESS:
        log(f"ERROR: Failed to publish object inventory: {result.rc}")


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

logger = get_logger("raas.fleet_controller")


def log(message: str) -> None:
    logger.info(message)


def separator() -> None:
    # Deliberately quiet in normal mode; keep operational messages in structured logs.
    return None


# --------------------
# PUBLISH VDA STATE
# --------------------

def publish_state(robot_key: tuple[str, str]) -> None:
    publish_vda_state(sys.modules[__name__], robot_key)


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
    return dispatch_robot_command(
        sys.modules[__name__],
        order_id,
        start_node,
        target_node,
        edge,
        manufacturer,
        serial_number,
    )


# --------------------
# START NEXT SEGMENT
# --------------------

def start_next_segment(robot_key: tuple[str, str]) -> None:
    process_next_segment(sys.modules[__name__], robot_key)


def handle_order(order: dict) -> None:
    process_order(sys.modules[__name__], order)


def handle_robot_status(status: dict) -> None:
    process_robot_status(sys.modules[__name__], status)


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
    handle_connect(
        client,
        userdata,
        flags,
        reason_code,
        properties,
        broker=BROKER,
        port=PORT,
        objects_raw_topic=OBJECTS_RAW_TOPIC,
        publish_robot_registry=publish_robot_registry,
        log=log,
    )


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
    handle_disconnect(
        client,
        userdata,
        disconnect_flags,
        reason_code,
        properties,
        log=log,
    )


# --------------------
# MQTT MESSAGE
# --------------------

def on_message(
    client,
    userdata,
    message,
):
    route_message(
        message,
        objects_raw_topic=OBJECTS_RAW_TOPIC,
        robot_registry=robot_registry,
        object_registry=object_registry,
        annotate_load_verification=annotate_load_verification,
        publish_robot_registry=publish_robot_registry,
        publish_object_registry=publish_object_registry,
        handle_order=handle_order,
        handle_robot_status=handle_robot_status,
        log=log,
        logger=logger,
    )


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