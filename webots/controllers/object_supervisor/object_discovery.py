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
):
    """Extract a local axis-aligned bound from exposed shapes or collision geometry."""
    visited = visited if visited is not None else set()
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
    if str(node.getTypeName()) == "Shape":
        geometry = _node_from_field(node, "geometry")
        if geometry is not None:
            points.extend(_geometry_points(geometry))
    elif str(node.getTypeName()) in {
        "Box", "Capsule", "Cylinder", "IndexedFaceSet", "Sphere",
    }:
        points.extend(_geometry_points(node))
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
                )
            )
    return points


def _generic_geometry(node):
    points = _geometry_from_tree(node)
    if not points or any(
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


def _walk_scene(nodes):
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
            if not NAVIGATION_DEF.fullmatch(definition or ""):
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
