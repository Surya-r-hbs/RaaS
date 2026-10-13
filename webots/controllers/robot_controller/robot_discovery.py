"""Resolve this controller's identity using only its local Webots Robot API."""

import re


def discover_local_robot(robot, manufacturer="DemoRobot"):
    """Return this robot's configured ID and model without world privileges."""
    get_name = getattr(robot, "getName", None)
    robot_id = str(get_name() or "").strip() if callable(get_name) else ""
    if not robot_id:
        raise RuntimeError(
            "Webots did not provide this robot's name; set a unique name "
            "on the robot node before starting its controller"
        )

    get_model = getattr(robot, "getModel", None)
    robot_type = str(get_model() or "").strip() if callable(get_model) else ""
    if not robot_type:
        robot_type = re.split(r"[-_](?:N\d+|SOURCE|DESTINATION)$", robot_id, 1)[0]
    if not robot_type:
        raise RuntimeError(
            f"Webots did not provide a model for robot {robot_id!r}"
        )
    return {
        "robotId": robot_id,
        "manufacturer": manufacturer,
        "robotType": robot_type,
    }
