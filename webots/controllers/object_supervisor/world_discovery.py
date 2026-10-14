"""Publish authoritative robot poses and navigation targets from Webots."""

import math
import re
from datetime import datetime, timezone

from object_discovery import (
    discover_navigation_map,
    discover_robot_footprint_radius,
    robot_model_footprint_radius,
)

NAVIGATION_DEF = re.compile(r"N\d+")
SUPERVISOR_CONTROLLER = "object_supervisor"
ROBOT_CONTROLLER = "robot_controller"


def _field(node, name):
    getter = getattr(node, "getField", None)
    return getter(name) if callable(getter) else None


def _field_string(node, name):
    field = _field(node, name)
    if field is None:
        return ""
    try:
        return field.getSFString().strip()
    except (AttributeError, TypeError, ValueError):
        return ""


def _children(node):
    field = _field(node, "children")
    if field is None:
        return []
    return [
        field.getMFNode(index)
        for index in range(field.getCount())
    ]


def discover_world_snapshot(supervisor, manufacturer="DemoRobot"):
    """Return robot records and N-node poses using the dedicated Supervisor."""
    root = supervisor.getRoot()
    children = _field(root, "children")
    if children is None:
        raise RuntimeError(
            "The dedicated object Supervisor cannot inspect Webots world "
            "children; check that its Robot node has supervisor TRUE"
        )

    stack = [
        (children.getMFNode(index), None)
        for index in reversed(range(children.getCount()))
    ]
    robots = []
    navigation_nodes = {}
    name_counts = {}
    while stack:
        node, station_id = stack.pop()
        if node is None:
            continue
        try:
            type_name = str(node.getTypeName())
            base_type = str(node.getBaseTypeName())
        except (AttributeError, TypeError, ValueError):
            continue

        controller = _field_string(node, "controller")
        if (
            type_name == "Robot"
            or base_type == "Robot"
            or controller == ROBOT_CONTROLLER
        ):
            if controller != SUPERVISOR_CONTROLLER:
                name = _field_string(node, "name")
                definition = node.getDef()
                base_id = name or definition
                if not base_id:
                    raise RuntimeError(
                        "A Webots robot has neither a name nor DEF; assign "
                        "a stable unique name before publishing discovery"
                    )
                name_counts[base_id] = name_counts.get(base_id, 0) + 1
                robot_id = (
                    base_id
                    if name_counts[base_id] == 1
                    else f"{base_id}-{name_counts[base_id]}"
                )
                position = tuple(float(value) for value in node.getPosition())
                orientation = tuple(
                    float(value) for value in node.getOrientation()
                )
                if (
                    len(position) != 3
                    or len(orientation) != 9
                    or not all(math.isfinite(value) for value in position + orientation)
                ):
                    raise RuntimeError(
                        f"Webots returned invalid pose for robot {robot_id!r}"
                    )
                proto_type = getattr(node, "getProtoTypeName", None)
                try:
                    robot_type = (
                        str(proto_type() or "").strip()
                        if callable(proto_type)
                        else ""
                    )
                except (AttributeError, TypeError, ValueError):
                    robot_type = ""
                if robot_type == "Robot":
                    robot_type = ""
                custom_data = _field_string(node, "customData")
                custom_station = next(
                    (
                        value.partition("=")[2].strip()
                        for value in custom_data.split(";")
                        if value.strip().startswith("stationId=")
                    ),
                    "",
                )
                record = {
                    "robotId": robot_id,
                    "manufacturer": manufacturer,
                    "position": dict(zip(("x", "y", "z"), position)),
                    "orientation": list(orientation),
                    "present": True,
                }
                if robot_type:
                    record["robotType"] = robot_type
                normalized_type = re.sub(r"[^a-z0-9]", "", robot_type.lower())
                is_mobile = normalized_type in {
                    "epuck",
                    "mir100",
                    "pioneer3at",
                    "tiago",
                    "youbot",
                }
                record["mobile"] = is_mobile
                if is_mobile:
                    footprint_radius = discover_robot_footprint_radius(node)
                    footprint_source = "SUPERVISOR_COLLISION_GEOMETRY"
                    footprint_status = "VERIFIED"
                    if footprint_radius is None:
                        footprint_radius = robot_model_footprint_radius(robot_type)
                        footprint_source = "WEBOTS_R2025A_MODEL_PROFILE"
                        footprint_status = "MODEL_PROFILE"
                    record["footprintStatus"] = (
                        footprint_status
                        if footprint_radius is not None
                        else "UNKNOWN"
                    )
                    record["footprintSource"] = (
                        footprint_source if footprint_radius is not None else "UNKNOWN"
                    )
                    if footprint_radius is not None:
                        record["footprintRadius"] = footprint_radius
                if station_id or custom_station:
                    record["stationId"] = station_id or custom_station
                robots.append(record)
            continue

        definition = node.getDef()
        if definition and NAVIGATION_DEF.fullmatch(definition):
            position = tuple(float(value) for value in node.getPosition())
            if len(position) == 3 and all(map(math.isfinite, position)):
                navigation_nodes[definition] = dict(
                    zip(("x", "y", "z"), position)
                )

        next_station = station_id
        if type_name == "Solid" or base_type == "Solid":
            next_station = definition or station_id
        stack.extend(
            (child, next_station)
            for child in reversed(_children(node))
        )

    return {
        "discoveryType": "WORLD_SNAPSHOT",
        "timestamp": (
            datetime.now(timezone.utc)
            .isoformat(timespec="milliseconds")
            .replace("+00:00", "Z")
        ),
        "manufacturer": manufacturer,
        "worldRobots": robots,
        "navigationNodes": navigation_nodes,
        "navigationMap": discover_navigation_map(supervisor),
    }
