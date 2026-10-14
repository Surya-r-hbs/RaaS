"""Discover scene solids and extract geometry exposed by Webots Supervisor.

Positions and local bounds use Webots coordinates: X/Y are the floor plane and
Z is height. PROTO implementations that hide geometry from Supervisor are
reported with unknown geometry rather than assigned estimated dimensions.
"""

from datetime import datetime, timezone
import math
import re


OBJECT_TYPES = {
    "Table": "table",
    "PlasticCrate": "plastic_crate",
    "WaterBottle": "water_bottle",
    "WoodenBox": "wooden_box",
}

CRATE_WALL_THICKNESS_FACTOR = 0.035
NAVIGATION_DEF = re.compile(r"N\d+")
INFRASTRUCTURE_TYPES = {"Floor"}
NON_COLLIDING_PROTOS = {"BeerBottle"}
WEBOTS_R2025A_ROBOT_FOOTPRINT_RADII = {
    "epuck": 0.05,
    "mir100": 0.55,
    "pioneer3at": 0.32,
    "tiago": 0.30,
    "youbot": 0.40,
}


def _timestamp():
    return (
        datetime.now(timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


def _proto_type(node):
    getter = getattr(node, "getProtoTypeName", None)
    if callable(getter):
        try:
            value = getter()
            if value:
                return str(value)
        except Exception:
            pass
    return str(node.getTypeName())


def _field(node, field_name):
    for getter_name in ("getProtoField", "getField"):
        getter = getattr(node, getter_name, None)
        if not callable(getter):
            continue
        try:
            field = getter(field_name)
        except Exception:
            continue
        if field is not None:
            return field
    return None


def _node_from_field(node, field_name):
    field = _field(node, field_name)
    if field is None:
        return None
    try:
        return field.getSFNode()
    except (AttributeError, TypeError, ValueError):
        return None


def _object_name(node, proto_type):
    field = _field(node, "name")
    if field is not None:
        try:
            value = field.getSFString().strip()
            if value:
                return value
        except (AttributeError, TypeError, ValueError):
            pass
    return proto_type


def _vector_field(node, field_name):
    field = _field(node, field_name)
    if field is None:
        return None
    for getter_name in ("getSFVec3f", "getSFVec3d"):
        getter = getattr(field, getter_name, None)
        if not callable(getter):
            continue
        try:
            values = tuple(float(value) for value in getter())
        except (TypeError, ValueError, IndexError):
            continue
        if len(values) == 3 and all(math.isfinite(value) for value in values):
            return values
    return None


def _size(node):
    values = _vector_field(node, "size")
    if values is None or any(value <= 0 for value in values):
        return None
    return dict(zip(("x", "y", "z"), values))


def _floor_size(node):
    field = _field(node, "size")
    getter = getattr(field, "getSFVec2f", None)
    if callable(getter):
        try:
            values = tuple(float(value) for value in getter())
        except (TypeError, ValueError):
            values = ()
        if (
            len(values) == 2
            and all(math.isfinite(value) and value > 0 for value in values)
        ):
            return dict(zip(("x", "y"), values))

    size = _size(node)
    if size is not None:
        return {"x": size["x"], "y": size["y"]}
    return None


def _mass(node):
    field = _field(node, "mass")
    if field is None:
        return None
    try:
        mass = float(field.getSFFloat())
    except (AttributeError, TypeError, ValueError):
        return None
    return mass if math.isfinite(mass) and mass >= 0 else None


def _bounds(minimum, maximum):
    return {
        "min": {axis: float(minimum[axis]) for axis in ("x", "y", "z")},
        "max": {axis: float(maximum[axis]) for axis in ("x", "y", "z")},
    }


def _field_nodes(node, field_name):
    field = _field(node, field_name)
    if field is None:
        return []
    try:
        count = field.getCount()
        return [
            child for child in (field.getMFNode(index) for index in range(count))
            if child is not None
        ]
    except (AttributeError, TypeError, ValueError):
        child = _node_from_field(node, field_name)
        return [child] if child is not None else []


def _rotation_matrix(node):
    field = _field(node, "rotation")
    if field is None:
        return (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    try:
        axis_x, axis_y, axis_z, angle = (
            float(value) for value in field.getSFRotation()
        )
    except (AttributeError, TypeError, ValueError):
        return (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    length = math.sqrt(axis_x**2 + axis_y**2 + axis_z**2)
    if not math.isfinite(angle) or length <= 0:
        return (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    x, y, z = axis_x / length, axis_y / length, axis_z / length
    cosine = math.cos(angle)
    sine = math.sin(angle)
    complement = 1.0 - cosine
    return (
        cosine + x * x * complement,
        x * y * complement - z * sine,
        x * z * complement + y * sine,
        y * x * complement + z * sine,
        cosine + y * y * complement,
        y * z * complement - x * sine,
        z * x * complement - y * sine,
        z * y * complement + x * sine,
        cosine + z * z * complement,
    )


def _multiply_matrices(left, right):
    return tuple(
        sum(left[row * 3 + index] * right[index * 3 + column] for index in range(3))
        for row in range(3)
        for column in range(3)
    )


def _apply_matrix(matrix, point):
    return tuple(
        sum(matrix[row * 3 + column] * point[column] for column in range(3))
        for row in range(3)
    )


def _geometry_points(geometry_node):
    """Return points in a geometry node's local coordinates when exposed."""
    kind = str(geometry_node.getTypeName())
    if kind == "Box":
        size = _vector_field(geometry_node, "size")
        if size and all(value > 0 for value in size):
            return [
                (sx * size[0] / 2, sy * size[1] / 2, sz * size[2] / 2)
                for sx in (-1, 1)
                for sy in (-1, 1)
                for sz in (-1, 1)
            ]
    if kind in {"Cylinder", "Capsule"}:
        radius_field = _field(geometry_node, "radius")
        height_field = _field(geometry_node, "height")
        try:
            radius = float(radius_field.getSFFloat())
            height = float(height_field.getSFFloat())
        except (AttributeError, TypeError, ValueError):
            return []
        if not all(math.isfinite(value) and value > 0 for value in (radius, height)):
            return []
        return [
            (x, y, z)
            for x in (-radius, radius)
            for y in (-height / 2 - (radius if kind == "Capsule" else 0),
                      height / 2 + (radius if kind == "Capsule" else 0))
            for z in (-radius, radius)
        ]
    if kind == "Sphere":
        field = _field(geometry_node, "radius")
        try:
            radius = float(field.getSFFloat())
        except (AttributeError, TypeError, ValueError):
            return []
        if not math.isfinite(radius) or radius <= 0:
            return []
        return [
            (x * radius, y * radius, z * radius)
            for x in (-1, 1)
            for y in (-1, 1)
            for z in (-1, 1)
        ]
    if kind == "IndexedFaceSet":
        coordinate = _node_from_field(geometry_node, "coord")
        points_field = _field(coordinate, "point") if coordinate else None
        if points_field is None:
            return []
        try:
            return [
                tuple(float(value) for value in points_field.getMFVec3f(index))
                for index in range(points_field.getCount())
            ]
        except (AttributeError, TypeError, ValueError, IndexError):
            return []
    return []


def _geometry_from_tree(
    node,
    visited=None,
    parent_position=(0.0, 0.0, 0.0),
    parent_rotation=(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
    is_root=True,
    geometry_complete=None,
):
    """Extract a local axis-aligned bound from exposed shapes or collision geometry."""
    visited = visited if visited is not None else set()
    geometry_complete = geometry_complete if geometry_complete is not None else [True]
    try:
        node_id = node.getId()
    except (AttributeError, TypeError, ValueError):
        node_id = id(node)
    if node_id in visited:
        return []
    visited.add(node_id)

    translation = _vector_field(node, "translation")
    local_position = (
        (0.0, 0.0, 0.0) if is_root or translation is None else translation
    )
    local_rotation = (
        parent_rotation
        if is_root
        else _multiply_matrices(parent_rotation, _rotation_matrix(node))
    )
    offset = _apply_matrix(parent_rotation, local_position)
    world_position = tuple(
        parent_position[index] + offset[index] for index in range(3)
    )

    points = []
    node_type = str(node.getTypeName())
    if node_type == "Shape":
        geometry = _node_from_field(node, "geometry")
        if geometry is not None:
            points.extend(_geometry_points(geometry))
        if not points:
            geometry_complete[0] = False
    elif node_type in {
        "Box", "Capsule", "Cylinder", "IndexedFaceSet", "Sphere",
    }:
        points.extend(_geometry_points(node))
        if not points:
            geometry_complete[0] = False
    points = [
        tuple(
            world_position[axis]
            + _apply_matrix(local_rotation, point)[axis]
            for axis in range(3)
        )
        for point in points
    ]

    for field_name in ("children", "boundingObject"):
        for child in _field_nodes(node, field_name):
            points.extend(
                _geometry_from_tree(
                    child,
                    visited,
                    world_position,
                    local_rotation,
                    is_root=False,
                    geometry_complete=geometry_complete,
                )
            )
    return points


def _generic_geometry(node):
    geometry_complete = [True]
    points = _geometry_from_tree(node, geometry_complete=geometry_complete)
    if not geometry_complete[0] or not points or any(
        len(point) != 3
        or not all(math.isfinite(value) for value in point)
        for point in points
    ):
        return None
    minimum = {
        axis: min(point[index] for point in points)
        for index, axis in enumerate(("x", "y", "z"))
    }
    maximum = {
        axis: max(point[index] for point in points)
        for index, axis in enumerate(("x", "y", "z"))
    }
    if any(minimum[axis] >= maximum[axis] for axis in minimum):
        return None
    return {"bounds": _bounds(minimum, maximum), "source": "SUPERVISOR_GEOMETRY"}


def _legacy_support_geometry(node, proto_type):
    """Retain existing explicit model geometry where the PROTO hides its shapes."""
    if proto_type == "WaterBottle":
        return {
            "bounds": _bounds(
                {"x": -0.044, "y": -0.044, "z": 0.002},
                {"x": 0.044, "y": 0.044, "z": 0.3055},
            ),
            "source": "R2025A_PROTO_BOUNDING_OBJECT",
        }
    size = _size(node)
    if size is None or proto_type not in {"Table", "PlasticCrate", "WoodenBox"}:
        return None

    if proto_type == "WoodenBox":
        minimum = {axis: -size[axis] / 2.0 for axis in ("x", "y", "z")}
        maximum = {axis: size[axis] / 2.0 for axis in ("x", "y", "z")}
        return {
            "bounds": _bounds(minimum, maximum),
            "support_surface": {
                "min": minimum,
                "max": maximum,
                "z": maximum["z"],
            },
            "source": "PROTO_SIZE",
        }

    minimum = {"x": -size["x"] / 2, "y": -size["y"] / 2, "z": 0.0}
    maximum = {
        "x": size["x"] / 2,
        "y": size["y"] / 2,
        "z": size["z"],
    }
    geometry = {"bounds": _bounds(minimum, maximum), "source": "PROTO_SIZE"}
    if proto_type == "Table":
        geometry["support_surface"] = {
            "min": {"x": minimum["x"], "y": minimum["y"]},
            "max": {"x": maximum["x"], "y": maximum["y"]},
            "z": maximum["z"],
        }
    elif proto_type == "PlasticCrate":
        wall = {axis: size[axis] * CRATE_WALL_THICKNESS_FACTOR for axis in size}
        geometry["container_interior"] = {
            "min": {
                "x": minimum["x"] + wall["x"],
                "y": minimum["y"] + wall["y"],
                "z": wall["z"],
            },
            "max": {
                "x": maximum["x"] - wall["x"],
                "y": maximum["y"] - wall["y"],
                "z": maximum["z"],
            },
        }
    return geometry


def _geometry(node, proto_type):
    return (
        _legacy_support_geometry(node, proto_type)
        or _generic_geometry(node)
    )


def discover_robot_footprint_radius(node):
    """Return a conservative planar radius from the exposed collision shape."""
    bounding_object = _node_from_field(node, "boundingObject")
    if bounding_object is None:
        return None
    geometry_complete = [True]
    points = _geometry_from_tree(
        bounding_object,
        geometry_complete=geometry_complete,
    )
    if not geometry_complete[0] or not points or any(
        len(point) != 3
        or not all(math.isfinite(value) for value in point)
        for point in points
    ):
        return None
    radius = max(math.hypot(point[0], point[1]) for point in points)
    return radius if math.isfinite(radius) and radius > 0 else None


def robot_model_footprint_radius(proto_type):
    """Return the conservative R2025a base envelope for opaque robot PROTOs."""
    normalized_type = re.sub(r"[^a-z0-9]", "", str(proto_type).lower())
    return WEBOTS_R2025A_ROBOT_FOOTPRINT_RADII.get(normalized_type)


def _base_type(node):
    getter = getattr(node, "getBaseTypeName", None)
    if callable(getter):
        try:
            return str(getter())
        except Exception:
            return ""
    return ""


def _is_robot(node):
    return _base_type(node) == "Robot" or str(node.getTypeName()) == "Robot"


def _is_scene_solid(node):
    base_type = _base_type(node)
    if base_type:
        return base_type == "Solid"
    return str(node.getTypeName()) in {"Solid", "Robot"} or bool(
        getattr(node, "proto_type", "")
    )


def _children(node):
    return _field_nodes(node, "children")


def _walk_scene(nodes, include_station_tables=False):
    """Walk scene children, but emit solids rather than their visual subnodes."""
    visited = set()
    stack = [(node, None) for node in reversed(nodes)]
    while stack:
        node, parent_id = stack.pop()
        try:
            identifier = node.getId()
        except (AttributeError, TypeError, ValueError):
            identifier = id(node)
        if identifier in visited:
            continue
        visited.add(identifier)
        if _is_robot(node):
            continue
        is_solid = _is_scene_solid(node)
        stable_id = None
        if is_solid:
            try:
                definition = node.getDef()
                stable_id = (
                    f"def:{definition}"
                    if definition
                    else f"webots-node:{node.getId()}"
                )
            except (AttributeError, TypeError, ValueError):
                stable_id = None
            is_station_table = (
                include_station_tables
                and NAVIGATION_DEF.fullmatch(definition or "")
                and _proto_type(node) == "Table"
            )
            if not NAVIGATION_DEF.fullmatch(definition or "") or is_station_table:
                yield node, parent_id
        next_parent = stable_id or parent_id
        stack.extend(
            (child, next_parent) for child in reversed(_children(node))
        )


def _category(node, proto_type, definition):
    field = _field(node, "raasCategory")
    if field is not None:
        try:
            value = field.getSFString().strip().lower()
            if value:
                return value
        except (AttributeError, TypeError, ValueError):
            pass
    if NAVIGATION_DEF.fullmatch(definition or ""):
        return "navigation"
    if proto_type in INFRASTRUCTURE_TYPES:
        return "infrastructure"
    if _field(node, "mass") is not None:
        return "cargo"
    return "unknown"


def discover_objects(supervisor, timestamp=None):
    root = supervisor.getRoot()
    children = root.getField("children")
    if children is None:
        raise RuntimeError("Cannot inspect Webots world children for objects")

    snapshot_time = timestamp or _timestamp()
    root_nodes = [
        children.getMFNode(index)
        for index in range(children.getCount())
    ]
    session_id = None
    try:
        session_id = f"webots-session:{supervisor.getSelf().getId()}"
    except (AttributeError, TypeError, ValueError):
        pass

    objects = []
    for node, parent_id in _walk_scene(root_nodes):
        if not _is_scene_solid(node):
            continue
        try:
            proto_type = _proto_type(node)
            node_type = str(node.getTypeName())
            definition = node.getDef()
            position = node.getPosition()
            rotation = node.getOrientation()
        except Exception:
            continue
        category = _category(node, proto_type, definition)
        if category in {"navigation", "infrastructure"}:
            continue
        stable_id = f"def:{definition}" if definition else f"webots-node:{node.getId()}"
        geometry = _geometry(node, proto_type)
        if geometry and geometry.get("support_surface"):
            category = "support_surface"
        elif geometry and geometry.get("container_interior"):
            category = "container"
        record = {
            "object_id": stable_id,
            "name": _object_name(node, proto_type),
            "object_type": OBJECT_TYPES.get(
                proto_type,
                re.sub(r"(?<!^)(?=[A-Z])", "_", proto_type).lower() or "unknown",
            ),
            "proto_type": proto_type,
            "node_type": node_type,
            "semantic_category": category,
            "source": "webots_simulation",
            "position": {
                "x": float(position[0]),
                "y": float(position[1]),
                "z": float(position[2]),
            },
            "position_reference": "PROTO_ORIGIN",
            "rotation": [float(value) for value in rotation],
            "geometry": geometry,
            "geometry_status": "VERIFIED" if geometry else "UNKNOWN",
            "parent_id": parent_id,
            "last_seen": snapshot_time,
            "simulation_session_id": session_id,
            "status": "PRESENT",
        }
        mass = _mass(node)
        if mass is not None:
            record["mass_kg"] = mass
        objects.append(record)

    return {
        "timestamp": snapshot_time,
        "coordinateSystem": "X/Y floor plane; Z height",
        "objects": objects,
    }


def discover_navigation_map(supervisor):
    """Return conservative floor bounds and collidable scene geometry."""
    root = supervisor.getRoot()
    children = root.getField("children")
    if children is None:
        raise RuntimeError("Cannot inspect Webots world children for navigation")

    root_nodes = [
        children.getMFNode(index)
        for index in range(children.getCount())
    ]
    obstacles = []
    issues = []
    floor_bounds = None
    floor_area = 0.0

    for node, _parent_id in _walk_scene(
        root_nodes,
        include_station_tables=True,
    ):
        try:
            proto_type = _proto_type(node)
            definition = node.getDef()
        except (AttributeError, TypeError, ValueError):
            continue

        if proto_type == "Floor":
            size = _floor_size(node)
            if size is None:
                issues.append("Floor geometry is unavailable")
                continue
            try:
                candidate_bounds = _world_xy_bounds(
                    node,
                    (
                        (-size["x"] / 2, -size["y"] / 2, 0.0),
                        (-size["x"] / 2, size["y"] / 2, 0.0),
                        (size["x"] / 2, -size["y"] / 2, 0.0),
                        (size["x"] / 2, size["y"] / 2, 0.0),
                    ),
                )
            except ValueError as error:
                issues.append(f"Floor geometry is invalid: {error}")
                continue
            candidate_area = size["x"] * size["y"]
            # Worlds may include small floor tiles nested in station wrapper Solids.
            if floor_bounds is None or candidate_area > floor_area:
                floor_bounds = candidate_bounds
                floor_area = candidate_area
            continue

        if (
            proto_type == "Solid"
            and _node_from_field(node, "boundingObject") is None
        ):
            continue

        if proto_type in NON_COLLIDING_PROTOS:
            continue

        if proto_type not in {
            "Table",
            "PlasticCrate",
            "WaterBottle",
            "WoodenBox",
        }:
            issues.append(
                f"Collision geometry is not classified for "
                f"{definition or proto_type}"
            )
            continue

        geometry = _geometry(node, proto_type)
        if geometry is None:
            issues.append(
                f"Collision geometry is unavailable for "
                f"{definition or _object_name(node, proto_type)}"
            )
            continue

        local_bounds = geometry["bounds"]
        corners = [
            (x, y, z)
            for x in (
                local_bounds["min"]["x"],
                local_bounds["max"]["x"],
            )
            for y in (
                local_bounds["min"]["y"],
                local_bounds["max"]["y"],
            )
            for z in (
                local_bounds["min"]["z"],
                local_bounds["max"]["z"],
            )
        ]
        name = _object_name(node, proto_type)
        try:
            world_bounds = _world_xy_bounds(node, corners)
        except ValueError as error:
            issues.append(
                f"Collision geometry is invalid for "
                f"{definition or name}: {error}"
            )
            continue
        try:
            node_id = node.getId()
        except (AttributeError, TypeError, ValueError):
            node_id = None
        obstacle_id = (
            f"webots-node:{node_id}"
            if node_id is not None
            else f"def:{definition or name}"
        )
        obstacles.append(
            {
                "id": obstacle_id,
                "name": name,
                "def": definition or None,
                "type": proto_type,
                "bounds": world_bounds,
                "geometrySource": geometry["source"],
                "approachAllowed": (
                    proto_type == "Table"
                    and NAVIGATION_DEF.fullmatch(definition or "") is not None
                ),
            }
        )

    if floor_bounds is None:
        issues.append("A bounded floor surface could not be identified")
    return {
        "floorBounds": floor_bounds,
        "obstacles": obstacles,
        "complete": not issues,
        "issues": issues,
        "source": "WEBOTS_SUPERVISOR_SIMULATION",
    }


def _world_xy_bounds(node, local_points):
    try:
        position = tuple(float(value) for value in node.getPosition())
        orientation = tuple(float(value) for value in node.getOrientation())
    except (AttributeError, TypeError, ValueError) as error:
        raise ValueError("Supervisor did not provide finite world geometry") from error
    if (
        len(position) != 3
        or len(orientation) != 9
        or not all(math.isfinite(value) for value in position + orientation)
    ):
        raise ValueError("Supervisor returned invalid world geometry")

    world_points = [
        tuple(
            position[row]
            + sum(orientation[row * 3 + column] * point[column] for column in range(3))
            for row in range(3)
        )
        for point in local_points
    ]
    if not world_points:
        raise ValueError("Supervisor returned empty geometry")
    return {
        "min": {
            "x": min(point[0] for point in world_points),
            "y": min(point[1] for point in world_points),
        },
        "max": {
            "x": max(point[0] for point in world_points),
            "y": max(point[1] for point in world_points),
        },
    }
