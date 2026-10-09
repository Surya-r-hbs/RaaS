"""Shared configuration for the RaaS VDA-5050 lab."""

BROKER = "localhost"
PORT = 1883

MANUFACTURER = "DemoRobot"
SERIAL_NUMBER = "robot001"

VDA_VERSION = "3.0.0"
MQTT_KEEPALIVE = 60


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
