"""Discover supported top-level warehouse objects through Webots Supervisor."""

from datetime import datetime, timezone
import math


OBJECT_TYPES = {
    "Table": "table",
    "PlasticCrate": "plastic_crate",
    "WaterBottle": "water_bottle",
    "WoodenBox": "wooden_box",
}

CRATE_WALL_THICKNESS_FACTOR = 0.035


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


def _object_name(node):
    field = _field(node, "name")
    if field is not None:
        try:
            value = field.getSFString().strip()
            if value:
                return value
        except Exception:
            pass
    return _proto_type(node)


def _size(node):
    field = _field(node, "size")
    if field is None:
        return None
    try:
        values = field.getSFVec3f()
        size = {
            "x": float(values[0]),
            "y": float(values[1]),
            "z": float(values[2]),
        }
    except (AttributeError, TypeError, ValueError, IndexError):
        return None
    if any(
        not math.isfinite(value) or value <= 0
        for value in size.values()
    ):
        return None
    return size


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


def _geometry(node, proto_type):
    size = _size(node)
    if size is not None and proto_type in {
        "Table",
        "PlasticCrate",
        "WoodenBox",
    }:
        if proto_type == "WoodenBox":
            minimum = {
                axis: -size[axis] / 2.0
                for axis in ("x", "y", "z")
            }
            maximum = {
                axis: size[axis] / 2.0
                for axis in ("x", "y", "z")
            }
            return {
                "bounds": _bounds(minimum, maximum),
                "support_surface": {
                    "min": minimum,
                    "max": maximum,
                    "z": maximum["z"],
                },
            }

        minimum = {
            "x": -size["x"] / 2.0,
            "y": -size["y"] / 2.0,
            "z": 0.0,
        }
        maximum = {
            "x": size["x"] / 2.0,
            "y": size["y"] / 2.0,
            "z": size["z"],
        }
        geometry = {"bounds": _bounds(minimum, maximum)}
        if proto_type == "Table":
            geometry["support_surface"] = {
                "min": {"x": minimum["x"], "y": minimum["y"]},
                "max": {"x": maximum["x"], "y": maximum["y"]},
                "z": maximum["z"],
            }
        elif proto_type == "PlasticCrate":
            interior_wall = {
                axis: size[axis] * CRATE_WALL_THICKNESS_FACTOR
                for axis in ("x", "y", "z")
            }
            geometry["container_interior"] = {
                "min": {
                    "x": minimum["x"] + interior_wall["x"],
                    "y": minimum["y"] + interior_wall["y"],
                    "z": interior_wall["z"],
                },
                "max": {
                    "x": maximum["x"] - interior_wall["x"],
                    "y": maximum["y"] - interior_wall["y"],
                    "z": maximum["z"],
                },
            }
        return geometry

    if proto_type == "WaterBottle":
        # Bounds come from the Cylinder/Sphere shapes in the R2025a
        # WaterBottle.proto boundingObject, relative to its base origin.
        return {
            "bounds": _bounds(
                {"x": -0.044, "y": -0.044, "z": 0.002},
                {"x": 0.044, "y": 0.044, "z": 0.3055},
            )
        }
    return None


def discover_objects(supervisor, timestamp=None):
    root = supervisor.getRoot()
    children = root.getField("children")
    if children is None:
        raise RuntimeError("Cannot inspect Webots world children for objects")

    snapshot_time = timestamp or _timestamp()
    objects = []
    for index in range(children.getCount()):
        node = children.getMFNode(index)
        if node is None:
            continue
        try:
            proto_type = _proto_type(node)
        except Exception:
            continue
        object_type = OBJECT_TYPES.get(proto_type)
        if object_type is None:
            continue

        position = node.getPosition()
        rotation = node.getOrientation()
        definition = node.getDef()
        stable_id = (
            f"def:{definition}"
            if definition
            else f"webots-node:{node.getId()}"
        )
        record = {
            "object_id": stable_id,
            "name": _object_name(node),
            "object_type": object_type,
            "source": "webots_simulation",
            "position": {
                "x": float(position[0]),
                "y": float(position[1]),
                "z": float(position[2]),
            },
            "rotation": [float(value) for value in rotation],
            "geometry": _geometry(node, proto_type),
            "last_seen": snapshot_time,
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
