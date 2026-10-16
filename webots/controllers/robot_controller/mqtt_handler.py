import json
import sys
from pathlib import Path

import paho.mqtt.client as mqtt

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from raas_logging import get_logger, log_event
from config import (
    MQTT_BROKER,
    MQTT_PORT,
    COMMAND_TOPIC,
    STATUS_TOPIC,
    robot_topic_prefix,
)

logger = get_logger("raas.robot_mqtt")


# --------------------
# MQTT HANDLER
# --------------------

class MQTTHandler:

    def __init__(
        self,
        command_callback,
        robot_id=None,
        manufacturer="DemoRobot",
        robot_type="Unknown",
        world_robots=None,
        robot_info=None,
        world_snapshot_callback=None,
    ):

        self.command_callback = (
            command_callback
        )
        self.robot_id = robot_id or "robot001"
        self.manufacturer = manufacturer or "DemoRobot"
        self.robot_type = robot_type or "Unknown"
        self.world_robots = world_robots or []
        self.robot_info = robot_info or {}
        self.world_snapshot_callback = world_snapshot_callback
        self.command_topic = (
            f"robot/{self.manufacturer}/{self.robot_id}/command"
        )
        self.status_topic = (
            f"robot/{self.manufacturer}/{self.robot_id}/status"
        )
        self.discovery_topic = "robot/discovery"

        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"webots-{self.robot_id}"
        )

        self.client.on_connect = (
            self.on_connect
        )

        self.client.on_message = (
            self.on_message
        )

    # --------------------
    # CONNECT
    # --------------------

    def connect(self):

        self.client.connect(
            MQTT_BROKER,
            MQTT_PORT,
            60
        )

        self.client.loop_start()


    # --------------------
    # CONNECT CALLBACK
    # --------------------

    def on_connect(
        self,
        client,
        userdata,
        flags,
        reason_code,
        properties=None
    ):

        if reason_code != 0:
            log_event(logger, "MQTT", "mqtt.connect_failed", "Robot MQTT connection failed", level="ERROR", robot_id=self.robot_id, broker=f"{MQTT_BROKER}:{MQTT_PORT}", reason=reason_code)
            return

        client.subscribe(self.command_topic)
        client.subscribe(self.discovery_topic, qos=0)
        log_event(logger, "MQTT", "mqtt.connected", "Robot MQTT connected", robot_id=self.robot_id, command_topic=self.command_topic, discovery_topic=self.discovery_topic)
        self.publish_discovery()

    # --------------------
    # MESSAGE CALLBACK
    # --------------------

    def on_message(
        self,
        client,
        userdata,
        message
    ):

        try:

            payload = json.loads(
                message.payload.decode(
                    "utf-8"
                )
            )

        except Exception as error:
            log_event(logger, "MQTT", "mqtt.invalid_message", "Ignored invalid MQTT payload", level="WARNING", robot_id=self.robot_id, topic=getattr(message, "topic", None), error=str(error))
            return

        if message.topic == self.discovery_topic:
            if payload.get("discoveryType") != "WORLD_SNAPSHOT":
                return
            if self.world_snapshot_callback is None:
                log_event(logger, "SIMULATION", "world_snapshot.no_handler", "Received world snapshot without a handler", level="WARNING", robot_id=self.robot_id, topic=message.topic)
                return
            self.world_snapshot_callback(payload)
            return

        if message.topic != self.command_topic:
            return

        log_event(logger, "MQTT", "command.received", "Robot command received", robot_id=self.robot_id, command=payload.get("command"), order_id=payload.get("orderId"), topic=message.topic)
        self.command_callback(payload)

    # --------------------
    # PUBLISH STATUS
    # --------------------

    def publish_status(
        self,
        message
    ):

        message = dict(message)
        message["robotId"] = self.robot_id
        message["manufacturer"] = self.manufacturer
        message["robotType"] = self.robot_type
        message.update(self.robot_info)

        self.client.publish(
            self.status_topic,
            json.dumps(message)
        )

    def publish_discovery(self):
        self.client.publish(
            self.discovery_topic,
            json.dumps(
                {
                    "robotId": self.robot_id,
                    "manufacturer": self.manufacturer,
                    "status": "ONLINE",
                    "state": "IDLE",
                    "robotType": self.robot_type,
                    "worldRobots": self.world_robots,
                    **self.robot_info,
                }
            ),
            qos=0,
        )