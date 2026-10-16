"""Shared configuration for the RaaS VDA-5050 lab."""

import os

BROKER = "localhost"
PORT = 1883

MANUFACTURER = "DemoRobot"
SERIAL_NUMBER = "robot001"

VDA_VERSION = "3.0.0"
MQTT_KEEPALIVE = 60

# Compatibility aliases for the Webots robot runtime, which imports config by
# module name when the test harness preloads the fleet config path.
MANUFACTURER = "DemoRobot"
ROBOT_ID = SERIAL_NUMBER
WEBOTS_SIMULATION_MODE = (
    os.environ.get("RAAS_WEBOTS_SIMULATION_MODE", "").strip() == "1"
)
MQTT_BROKER = BROKER
MQTT_PORT = PORT


def robot_topic_prefix(manufacturer: str, robot_id: str) -> str:
    return f"robot/{manufacturer}/{robot_id}"


COMMAND_TOPIC = robot_topic_prefix(MANUFACTURER, ROBOT_ID) + "/command"
STATUS_TOPIC = robot_topic_prefix(MANUFACTURER, ROBOT_ID) + "/status"
TIME_STEP = 32
POSITION_TOLERANCE = 0.12
MAX_SPEED = 4.0
TURN_SPEED = 2.0
HEADING_TOLERANCE_DEGREES = 5.0


def order_topic(manufacturer: str, serial_number: str) -> str:
    return f"vda5050/v3/{manufacturer}/{serial_number}/order"


def state_topic(manufacturer: str, serial_number: str) -> str:
    return f"vda5050/v3/{manufacturer}/{serial_number}/state"


def robot_command_topic(manufacturer: str, serial_number: str) -> str:
    return f"robot/{manufacturer}/{serial_number}/command"


def robot_status_topic(manufacturer: str, serial_number: str) -> str:
    return f"robot/{manufacturer}/{serial_number}/status"


ORDER_TOPIC = order_topic(MANUFACTURER, SERIAL_NUMBER)
ROBOT_COMMAND_TOPIC = robot_command_topic(MANUFACTURER, SERIAL_NUMBER)
ROBOT_STATUS_TOPIC = robot_status_topic(MANUFACTURER, SERIAL_NUMBER)
STATE_TOPIC = state_topic(MANUFACTURER, SERIAL_NUMBER)

# Simulation map topology. Edges represent traversable warehouse segments;
# destination planning uses their geometric lengths, not straight-line shortcuts.
MAP_NODE_POSITIONS = {
    "N0": (-3.0, 0.0),
    "N1": (0.0, 0.0),
    "N2": (3.0, 0.0),
    "N3": (6.0, 0.0),
    "N4": (5.0, 2.0),
    "N5": (6.0, 3.0),
}

MAP_CONNECTIONS = (
    ("N0", "N1"),
    ("N1", "N2"),
    ("N2", "N3"),
    ("N3", "N4"),
    ("N4", "N5"),
)

# Simulation assumptions must be enabled explicitly in the fleet process.
SIMULATION_TRANSPORT_ENABLED = (
    os.environ.get("RAAS_WEBOTS_SIMULATION_MODE", "").strip() == "1"
)
LOAD_CONTACT_TOLERANCE_METERS = 0.015
LOAD_FOOTPRINT_TOLERANCE_METERS = 0.005
