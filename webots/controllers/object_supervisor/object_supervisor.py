"""Publish a live warehouse-object inventory from a Webots Supervisor."""

import json
import threading
import time

from controller import Supervisor
import paho.mqtt.client as mqtt

from object_discovery import discover_objects
from world_discovery import discover_world_snapshot


TIME_STEP = 32
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
OBJECTS_TOPIC = "warehouse/objects/raw"
ROBOT_DISCOVERY_TOPIC = "robot/discovery"
REFRESH_TOPIC = "warehouse/objects/refresh"
PUBLISH_INTERVAL_SECONDS = 1.0
ROBOT_SNAPSHOT_INTERVAL_SECONDS = 0.1


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
            print(f"Object inventory MQTT connection failed: {reason_code}")
            self._connected = False
            return
        self._connected = True
        client.subscribe(REFRESH_TOPIC, qos=0)
        print(f"Object inventory connected; subscribed to {REFRESH_TOPIC}")

    def _on_disconnect(
        self, client, userdata, disconnect_flags, reason_code, properties
    ):
        self._connected = False
        print(f"Object inventory MQTT disconnected: {reason_code}")

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
            print(f"Object inventory publish failed: {result.rc}")
        return payload

    def publish_robot_snapshot(self):
        payload = discover_world_snapshot(self.supervisor)

        print(
            f"[DEBUG] Generated robot snapshot timestamp: "
            f"{payload.get('timestamp')}",
            flush=True,
        )

        result = self.mqtt.publish(
            ROBOT_DISCOVERY_TOPIC,
            json.dumps(payload),
            qos=1,
            retain=True,
        )

        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            print(f"Robot world snapshot publish failed: {result.rc}")
            
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
                self.publish_robot_snapshot()
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
