"""Publish a live warehouse-object inventory from a Webots Supervisor."""

import json
import os
import sys
import threading
import time

from pathlib import Path

from controller import Supervisor
import paho.mqtt.client as mqtt

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from raas_logging import get_logger, log_event
from object_discovery import discover_objects
from world_discovery import discover_world_snapshot

logger = get_logger("raas.object_supervisor")


TIME_STEP = 32
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
OBJECTS_TOPIC = "warehouse/objects/raw"
ROBOT_DISCOVERY_TOPIC = "robot/discovery"
REFRESH_TOPIC = "warehouse/objects/refresh"
PUBLISH_INTERVAL_SECONDS = 5.0
ROBOT_SNAPSHOT_INTERVAL_SECONDS = 0.75


class ObjectInventorySupervisor:
    def __init__(self, supervisor=None, mqtt_client=None):
        self.supervisor = supervisor or Supervisor()
        self.mqtt = mqtt_client or mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id="warehouse-object-supervisor",
        )
        self._refresh_requested = threading.Event()
        self._connected = False
        self.mqtt.on_connect = self._on_connect
        self.mqtt.on_disconnect = self._on_disconnect
        self.mqtt.on_message = self._on_message
        self.mqtt.reconnect_delay_set(min_delay=1, max_delay=5)

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code != 0:
            log_event(logger, "MQTT", "mqtt.connect_failed", "Object inventory MQTT connection failed", level="ERROR", topic=REFRESH_TOPIC, reason=reason_code)
            self._connected = False
            return
        self._connected = True
        client.subscribe(REFRESH_TOPIC, qos=0)
        log_event(logger, "MQTT", "mqtt.connected", "Object inventory connected", topic=REFRESH_TOPIC, source="object_supervisor")

    def _on_disconnect(
        self, client, userdata, disconnect_flags, reason_code, properties
    ):
        self._connected = False
        log_event(logger, "MQTT", "mqtt.disconnected", "Object inventory MQTT disconnected", level="WARNING", reason=reason_code)

    def _on_message(self, client, userdata, message):
        if message.topic == REFRESH_TOPIC:
            self._refresh_requested.set()

    def publish_snapshot(self):
        payload = discover_objects(self.supervisor)
        result = self.mqtt.publish(
            OBJECTS_TOPIC,
            json.dumps(payload),
            qos=1,
            retain=True,
        )
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            log_event(logger, "OBJECTS_INVENTORY", "inventory.publish_failed", "Object inventory publish failed", level="ERROR", topic=OBJECTS_TOPIC, rc=result.rc)
        return payload

    def publish_robot_snapshot(self):
        payload = discover_world_snapshot(self.supervisor)
        result = self.mqtt.publish(
            ROBOT_DISCOVERY_TOPIC,
            json.dumps(payload),
            qos=1,
            retain=True,
        )

        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            log_event(logger, "MQTT", "robot_snapshot.publish_failed", "Robot world snapshot publish failed", level="ERROR", topic=ROBOT_DISCOVERY_TOPIC, rc=result.rc)
        else:
            log_event(logger, "SIMULATION", "robot_snapshot.published", "Robot world snapshot published", topic=ROBOT_DISCOVERY_TOPIC, timestamp=payload.get("timestamp"))

        return payload

    def run(self):
        self.mqtt.connect_async(MQTT_BROKER, MQTT_PORT, keepalive=60)
        self.mqtt.loop_start()
        last_publish = 0.0
        last_robot_snapshot = 0.0
        while self.supervisor.step(TIME_STEP) != -1:
            now = time.monotonic()
            if (
                self._connected
                and now - last_robot_snapshot >= ROBOT_SNAPSHOT_INTERVAL_SECONDS
            ):
                # A single transient Webots discovery failure must not terminate
                # the Supervisor loop. Keep the last published snapshot in place
                # and retry discovery on the next interval.
                try:
                    self.publish_robot_snapshot()
                except Exception as error:
                    log_event(
                        logger,
                        "SIMULATION",
                        "robot_snapshot.discovery_failed",
                        "Robot world snapshot discovery failed; will retry",
                        level="ERROR",
                        topic=ROBOT_DISCOVERY_TOPIC,
                        error_type=type(error).__name__,
                        error=str(error),
                    )
                finally:
                    last_robot_snapshot = now
            if (
                self._connected
                and (
                    now - last_publish >= PUBLISH_INTERVAL_SECONDS
                    or self._refresh_requested.is_set()
                )
            ):
                self._refresh_requested.clear()
                self.publish_snapshot()
                last_publish = now
        self.mqtt.loop_stop()
        self.mqtt.disconnect()


if __name__ == "__main__":
    ObjectInventorySupervisor().run()