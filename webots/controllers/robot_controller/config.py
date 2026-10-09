# --------------------
# ROBOT CONFIGURATION
# --------------------

MANUFACTURER = "DemoRobot"
ROBOT_ID = "robot001"


def resolve_robot_identity(robot=None):
    """Return the robot identity from the live Webots world when available."""

    if robot is not None:
        self_robot = getattr(robot, "getSelf", lambda: None)()
        if self_robot is not None:
            for getter_name in ("getName", "getDef"):
                getter = getattr(self_robot, getter_name, None)
                if callable(getter):
                    try:
                        value = getter()
                    except Exception:
                        continue
                    if value:
                        return str(value), MANUFACTURER

        for getter_name in ("getName", "getDef"):
            getter = getattr(robot, getter_name, None)
            if callable(getter):
                try:
                    value = getter()
                except Exception:
                    continue
                if value:
                    return str(value), MANUFACTURER

    return ROBOT_ID, MANUFACTURER


# --------------------
# MQTT CONFIGURATION
# --------------------

MQTT_BROKER = "localhost"
MQTT_PORT = 1883


def robot_topic_prefix(manufacturer, robot_id):
    return f"robot/{manufacturer}/{robot_id}"


COMMAND_TOPIC = (
    f"robot/{MANUFACTURER}/{ROBOT_ID}/command"
)

STATUS_TOPIC = (
    f"robot/{MANUFACTURER}/{ROBOT_ID}/status"
)


# --------------------
# WEBOTS CONFIGURATION
# --------------------

TIME_STEP = 32


# --------------------
# NAVIGATION CONFIGURATION
# --------------------

POSITION_TOLERANCE = 0.12

MAX_SPEED = 4.0

TURN_SPEED = 2.0

HEADING_TOLERANCE_DEGREES = 5.0