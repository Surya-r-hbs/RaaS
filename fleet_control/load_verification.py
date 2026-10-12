"""Verify preloaded objects against fresh Webots poses and carrier geometry."""

from datetime import datetime, timezone
import math


LOAD_EVIDENCE_METHOD = "SUPERVISOR_GEOMETRY"
MAX_EVIDENCE_AGE_SECONDS = 5.0
MAX_POSE_AGE_SECONDS = 5.0
CONTACT_TOLERANCE_METERS = 0.015
FOOTPRINT_TOLERANCE_METERS = 0.005


def _number(value):
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _timestamp_age(value, maximum_age=MAX_EVIDENCE_AGE_SECONDS):
    if not isinstance(value, str):
        return None
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None
    if parsed.tzinfo is None:
        return None
    age = (datetime.now(timezone.utc) - parsed).total_seconds()
    return age if 0 <= age <= maximum_age else None


def _vector(record):
    if not isinstance(record, dict):
        return None
    result = tuple(_number(record.get(axis)) for axis in ("x", "y", "z"))
    return result if all(value is not None for value in result) else None


def _matrix(record):
    if not isinstance(record, (list, tuple)) or len(record) != 9:
        return None
    values = tuple(_number(value) for value in record)
    if any(value is None for value in values):
        return None
    for row in range(3):
        for other_row in range(3):
            dot_product = sum(
                values[row * 3 + column]
                * values[other_row * 3 + column]
                for column in range(3)
            )
            expected = 1.0 if row == other_row else 0.0
            if abs(dot_product - expected) > 0.001:
                return None
    determinant = (
        values[0] * (values[4] * values[8] - values[5] * values[7])
        - values[1] * (values[3] * values[8] - values[5] * values[6])
        + values[2] * (values[3] * values[7] - values[4] * values[6])
    )
    return values if abs(determinant - 1.0) <= 0.001 else None


def _transform(matrix, vector):
    return tuple(
        sum(matrix[row * 3 + column] * vector[column] for column in range(3))
        for row in range(3)
    )


def _object_supported(object_record, robot):
    capability = robot.get("transportCapability")
    surface = (
        capability.get("supportSurface")
        if isinstance(capability, dict)
        else None
    )
    surface_center = _vector(surface.get("center")) if isinstance(surface, dict) else None
    surface_size = _vector(surface.get("dimensions")) if isinstance(surface, dict) else None
    robot_position = _vector(robot.get("position"))
    robot_rotation = _matrix(robot.get("orientation"))
    object_position = _vector(object_record.get("position"))
    object_rotation = _matrix(object_record.get("rotation"))
    geometry = object_record.get("geometry")
    bounds = geometry.get("bounds") if isinstance(geometry, dict) else None
    minimum = _vector(bounds.get("min")) if isinstance(bounds, dict) else None
    maximum = _vector(bounds.get("max")) if isinstance(bounds, dict) else None
    if any(
        value is None
        for value in (
            surface_center,
            surface_size,
            robot_position,
            robot_rotation,
            object_position,
            object_rotation,
            minimum,
            maximum,
        )
    ):
        return False

    if (
        surface_size[0] <= 0
        or surface_size[1] <= 0
        or any(minimum[axis] >= maximum[axis] for axis in range(3))
    ):
        return False

    corners = []
    for x in (minimum[0], maximum[0]):
        for y in (minimum[1], maximum[1]):
            for z in (minimum[2], maximum[2]):
                world_offset = _transform(object_rotation, (x, y, z))
                world_point = tuple(
                    object_position[axis] + world_offset[axis]
                    for axis in range(3)
                )
                carrier_offset = tuple(
                    world_point[axis] - robot_position[axis]
                    for axis in range(3)
                )
                local_point = tuple(
                    sum(
                        robot_rotation[row * 3 + axis] * carrier_offset[row]
                        for row in range(3)
                    )
                    for axis in range(3)
                )
                corners.append(local_point)

    bottom_z = min(point[2] for point in corners)
    bottom_points = [
        point for point in corners
        if point[2] <= bottom_z + FOOTPRINT_TOLERANCE_METERS
    ]
    if len(bottom_points) != 4:
        return False
    if any(
        abs(point[2] - surface_center[2]) > CONTACT_TOLERANCE_METERS
        for point in bottom_points
    ):
        return False
    half_x = surface_size[0] / 2.0
    half_y = surface_size[1] / 2.0
    return all(
        abs(point[0] - surface_center[0])
        <= half_x + FOOTPRINT_TOLERANCE_METERS
        and abs(point[1] - surface_center[1])
        <= half_y + FOOTPRINT_TOLERANCE_METERS
        for point in bottom_points
    )


def annotate_load_verification(snapshot, robots):
    """Attach evidence only when one live, modeled support surface fits."""
    if not isinstance(snapshot, dict) or not isinstance(
        snapshot.get("objects"), list
    ):
        return snapshot

    for object_record in snapshot["objects"]:
        if isinstance(object_record, dict):
            object_record.pop("load_verification", None)

    observed_at = snapshot.get("timestamp")
    if _timestamp_age(observed_at) is None:
        return snapshot

    fresh_robots = [
        robot
        for robot in robots
        if isinstance(robot, dict)
        and robot.get("agentOnline") is True
        and robot.get("status") in {"IDLE", "STOPPED", "NODE_REACHED"}
        and robot.get("robotId")
        and robot.get("manufacturer")
        and _timestamp_age(
            robot.get("lastUpdate"),
            MAX_POSE_AGE_SECONDS,
        ) is not None
    ]
    for object_record in snapshot["objects"]:
        if (
            not isinstance(object_record, dict)
            or not isinstance(object_record.get("object_id"), str)
            or not object_record["object_id"].strip()
        ):
            continue
        matches = [
            robot
            for robot in fresh_robots
            if _object_supported(object_record, robot)
        ]
        if len(matches) != 1:
            continue
        carrier = matches[0]
        object_record["load_verification"] = {
            "verified": True,
            "method": LOAD_EVIDENCE_METHOD,
            "load_state": "LOADED",
            "object_id": object_record.get("object_id"),
            "carrier_robot_id": carrier["robotId"],
            "carrier_manufacturer": carrier["manufacturer"],
            "observed_at": observed_at,
        }
    return snapshot
