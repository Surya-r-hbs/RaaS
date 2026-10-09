import json

import paho.mqtt.client as mqtt

from config import (
    MQTT_BROKER,
    MQTT_PORT,
    COMMAND_TOPIC,
    STATUS_TOPIC,
    robot_topic_prefix,
)


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
    ):

        self.command_callback = (
            command_callback
        )
        self.robot_id = robot_id or "robot001"
        self.manufacturer = manufacturer or "DemoRobot"
        self.robot_type = robot_type or "Unknown"
        self.world_robots = world_robots or []
        self.robot_info = robot_info or {}
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

        print("--------------------")
        print("MQTT CONNECTED")
        print(
            f"Broker: "
            f"{MQTT_BROKER}:{MQTT_PORT}"
        )
        print(f"Robot: {self.robot_id}")
        print("--------------------")

        client.subscribe(
            self.command_topic
        )

        print(
            f"Subscribed: "
            f"{self.command_topic}"
        )
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

            print("--------------------")
            print("INVALID MQTT MESSAGE")
            print(error)
            print("--------------------")

            return

        if message.topic != self.command_topic:
            return

        print("--------------------")
        print("COMMAND RECEIVED")
        print(
            json.dumps(
                payload,
                indent=2
            )
        )
        print("--------------------")

        self.command_callback(
            payload
        )

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