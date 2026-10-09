"""Geometry-based warehouse object relationship analysis.

World coordinates follow this project convention: X/Y are the floor plane and
Z is height. Object bounds are local to each object's PROTO origin.
"""

from datetime import datetime, timezone


SURFACE_GAP_TOLERANCE = 0.025
CONTAINMENT_TOLERANCE = 0.005
MIN_SUPPORT_OVERLAP_RATIO = 0.6
MIN_SUPPORT_NORMAL_Z = 0.95


def _timestamp():
    return (
        datetime.now(timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


def _corners(bounds):
    minimum = bounds["min"]
    maximum = bounds["max"]
    return [
        (x, y, z)
        for x in (minimum["x"], maximum["x"])
        for y in (minimum["y"], maximum["y"])
        for z in (minimum["z"], maximum["z"])
    ]


def _transform(point, position, rotation):
    x, y, z = point
    return (
        position["x"] + rotation[0] * x + rotation[1] * y + rotation[2] * z,
        position["y"] + rotation[3] * x + rotation[4] * y + rotation[5] * z,
        position["z"] + rotation[6] * x + rotation[7] * y + rotation[8] * z,
    )


def _world_corners(obj, bounds):
    return [
        _transform(point, obj["position"], obj["rotation"])
        for point in _corners(bounds)
    ]


def _axis_bounds(points, axis):
    values = [point[axis] for point in points]
    return min(values), max(values)


def _overlap_ratio(upper_points, surface_points):
    upper_x = _axis_bounds(upper_points, 0)
    upper_y = _axis_bounds(upper_points, 1)
    surface_x = _axis_bounds(surface_points, 0)
    surface_y = _axis_bounds(surface_points, 1)
    overlap_x = max(
        0.0,
        min(upper_x[1], surface_x[1]) - max(upper_x[0], surface_x[0]),
    )
    overlap_y = max(
        0.0,
        min(upper_y[1], surface_y[1]) - max(upper_y[0], surface_y[0]),
    )
    upper_area = (upper_x[1] - upper_x[0]) * (upper_y[1] - upper_y[0])
    if upper_area <= 0:
        return 0.0
    return overlap_x * overlap_y / upper_area


def _is_on_top(upper, lower):
    geometry = upper.get("geometry")
    lower_geometry = lower.get("geometry")
    if not geometry or not lower_geometry:
        return False

    surface = lower_geometry.get("support_surface")
    if not surface or upper.get("object_id") == lower.get("object_id"):
        return False
    if lower["rotation"][8] < MIN_SUPPORT_NORMAL_Z:
        return False

    upper_points = _world_corners(upper, geometry["bounds"])
    surface_bounds = {
        "min": {
            **surface["min"],
            "z": surface["z"],
        },
        "max": {
            **surface["max"],
            "z": surface["z"],
        },
    }
    surface_points = _world_corners(lower, surface_bounds)
    upper_bottom = min(point[2] for point in upper_points)
    surface_height = sum(point[2] for point in surface_points) / len(
        surface_points
    )

    if abs(upper_bottom - surface_height) > SURFACE_GAP_TOLERANCE:
        return False
    return _overlap_ratio(upper_points, surface_points) >= (
        MIN_SUPPORT_OVERLAP_RATIO
    )


def _is_inside(obj, container):
    geometry = obj.get("geometry")
    container_geometry = container.get("geometry")
    if not geometry or not container_geometry:
        return False

    interior = container_geometry.get("container_interior")
    if not interior or obj.get("object_id") == container.get("object_id"):
        return False

    object_world_corners = _world_corners(obj, geometry["bounds"])
    container_position = container["position"]
    container_rotation = container["rotation"]
    interior_min = interior["min"]
    interior_max = interior["max"]

    for world_x, world_y, world_z in object_world_corners:
        relative = (
            world_x - container_position["x"],
            world_y - container_position["y"],
            world_z - container_position["z"],
        )
        local = (
            container_rotation[0] * relative[0]
            + container_rotation[3] * relative[1]
            + container_rotation[6] * relative[2],
            container_rotation[1] * relative[0]
            + container_rotation[4] * relative[1]
            + container_rotation[7] * relative[2],
            container_rotation[2] * relative[0]
            + container_rotation[5] * relative[1]
            + container_rotation[8] * relative[2],
        )
        for index, axis in enumerate(("x", "y", "z")):
            if (
                local[index] < interior_min[axis] - CONTAINMENT_TOLERANCE
                or local[index] > interior_max[axis] + CONTAINMENT_TOLERANCE
            ):
                return False
    return True


def analyze_relationships(objects):
    """Recompute all supported relationships from the current geometry."""
    relationships = {obj["object_id"]: [] for obj in objects}
    verified_at = _timestamp()
    inside_pairs = {
        (obj["object_id"], container["object_id"])
        for obj in objects
        for container in objects
        if _is_inside(obj, container)
    }
    support_pairs = {
        (obj["object_id"], support["object_id"])
        for obj in objects
        for support in objects
        if _is_on_top(obj, support)
    }

    for object_id, container_id in inside_pairs:
        support_ids = [
            support_id
            for supported_object_id, support_id in support_pairs
            if supported_object_id == container_id
        ]
        for support_id in support_ids:
            support_pairs.discard((object_id, support_id))

    for object_id, container_id in inside_pairs:
        relationships[object_id].append(
            {
                "type": "INSIDE",
                "target_object_id": container_id,
                "certainty": "GEOMETRY_CHECKED",
                "verified_at": verified_at,
            }
        )

    for object_id, support_id in support_pairs:
        if object_id != support_id:
            relationships[object_id].append(
                {
                    "type": "ON_TOP_OF",
                    "target_object_id": support_id,
                    "certainty": "GEOMETRY_CHECKED",
                    "verified_at": verified_at,
                }
            )

    for obj in objects:
        obj["relationships"] = relationships[obj["object_id"]]
    return objects
