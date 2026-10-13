"""Current Webots object inventory and relationship registry."""

from datetime import datetime
import math
import threading

from spatial_analyzer import analyze_relationships


COORDINATE_SYSTEM = "X/Y floor plane; Z height"


def _number(value):
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _vector(value, fields):
    if not isinstance(value, dict):
        return None
    normalized = {field: _number(value.get(field)) for field in fields}
    return normalized if all(item is not None for item in normalized.values()) else None


def _normalize_bounds(value):
    if not isinstance(value, dict):
        return None
    minimum = _vector(value.get("min"), ("x", "y", "z"))
    maximum = _vector(value.get("max"), ("x", "y", "z"))
    if minimum is None or maximum is None:
        return None
    if any(minimum[axis] > maximum[axis] for axis in ("x", "y", "z")):
        return None
    return {"min": minimum, "max": maximum}


def _normalize_geometry(value):
    if not isinstance(value, dict):
        return None
    bounds = _normalize_bounds(value.get("bounds"))
    if bounds is None:
        return None

    geometry = {"bounds": bounds}
    support = value.get("support_surface")
    if isinstance(support, dict):
        support_min = _vector(support.get("min"), ("x", "y"))
        support_max = _vector(support.get("max"), ("x", "y"))
        support_z = _number(support.get("z"))
        if (
            support_min is not None
            and support_max is not None
            and support_z is not None
            and support_min["x"] <= support_max["x"]
            and support_min["y"] <= support_max["y"]
        ):
            geometry["support_surface"] = {
                "min": support_min,
                "max": support_max,
                "z": support_z,
            }

    interior = value.get("container_interior")
    if isinstance(interior, dict):
        interior_min = _vector(interior.get("min"), ("x", "y", "z"))
        interior_max = _vector(interior.get("max"), ("x", "y", "z"))
        if (
            interior_min is not None
            and interior_max is not None
            and all(
                interior_min[axis] <= interior_max[axis]
                for axis in ("x", "y", "z")
            )
        ):
            geometry["container_interior"] = {
                "min": interior_min,
                "max": interior_max,
            }
    return geometry


def _normalize_load_verification(value, object_id):
    if not isinstance(value, dict):
        return None
    if (
        value.get("verified") is not True
        or value.get("method") != "SUPERVISOR_GEOMETRY"
        or value.get("load_state") not in {"LOADED", "UNLOADED"}
        or value.get("object_id") != object_id
        or not isinstance(value.get("carrier_robot_id"), str)
        or not value["carrier_robot_id"].strip()
        or not isinstance(value.get("carrier_manufacturer"), str)
        or not value["carrier_manufacturer"].strip()
        or not isinstance(value.get("observed_at"), str)
    ):
        return None
    try:
        observed_at = datetime.fromisoformat(
            value["observed_at"].replace("Z", "+00:00")
        )
    except ValueError:
        return None
    if observed_at.tzinfo is None:
        return None
    evidence_age = (
        datetime.now(observed_at.tzinfo) - observed_at
    ).total_seconds()
    if not 0 <= evidence_age <= 5.0:
        return None
    return {
        "verified": True,
        "method": "SUPERVISOR_GEOMETRY",
        "load_state": value["load_state"],
        "object_id": object_id,
        "carrier_robot_id": value["carrier_robot_id"].strip(),
        "carrier_manufacturer": value["carrier_manufacturer"].strip(),
        "observed_at": value["observed_at"],
    }


def _normalize_object(record, snapshot_time):
    if not isinstance(record, dict):
        return None

    object_id = str(record.get("object_id") or "").strip()
    name = str(record.get("name") or "").strip()
    object_type = str(record.get("object_type") or "").strip()
    position = _vector(record.get("position"), ("x", "y", "z"))
    if not object_id or not name or not object_type or position is None:
        return None

    rotation = record.get("rotation")
    if rotation is None:
        rotation = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    elif not isinstance(rotation, (list, tuple)) or len(rotation) != 9:
        return None
    else:
        rotation = [_number(value) for value in rotation]
        if any(value is None for value in rotation):
            return None

    geometry = _normalize_geometry(record.get("geometry"))
    dimensions = None
    if geometry is not None:
        dimensions = {
            axis: geometry["bounds"]["max"][axis]
            - geometry["bounds"]["min"][axis]
            for axis in ("x", "y", "z")
        }

    mass = _number(record.get("mass_kg"))
    normalized = {
        "object_id": object_id,
        "name": name,
        "object_type": object_type,
        "proto_type": str(record.get("proto_type") or ""),
        "node_type": str(record.get("node_type") or ""),
        "semantic_category": str(record.get("semantic_category") or "unknown"),
        "source": str(record.get("source") or "webots_simulation"),
        "position": position,
        "position_reference": "PROTO_ORIGIN",
        "rotation": rotation,
        "dimensions": dimensions,
        "geometry": geometry,
        "geometry_status": (
            "VERIFIED" if geometry is not None else "UNKNOWN"
        ),
        "parent_id": record.get("parent_id"),
        "simulation_session_id": record.get("simulation_session_id"),
        "load_verification_reason": record.get("load_verification_reason"),
        "relationships": [],
        "last_seen": record.get("last_seen") or snapshot_time,
        "status": "PRESENT",
    }
    if mass is not None and mass >= 0:
        normalized["mass_kg"] = mass
    load_verification = _normalize_load_verification(
        record.get("load_verification"),
        object_id,
    )
    if load_verification is not None:
        normalized["load_verification"] = load_verification
    return normalized


class ObjectRegistry:
    def __init__(self):
        self._lock = threading.Lock()
        self._objects = {}
        self._last_refresh = None

    def update(self, payload):
        """Replace the current scene inventory from a validated supervisor scan."""
        if not isinstance(payload, dict) or not isinstance(
            payload.get("objects"), list
        ):
            raise ValueError("Object snapshot must contain an objects list")

        snapshot_time = payload.get("timestamp")
        if not isinstance(snapshot_time, str) or not snapshot_time.strip():
            raise ValueError("Object snapshot must include an ISO-8601 timestamp")
        try:
            parsed_time = datetime.fromisoformat(
                snapshot_time.replace("Z", "+00:00")
            )
        except ValueError as error:
            raise ValueError("Object snapshot timestamp is invalid") from error
        if parsed_time.tzinfo is None:
            raise ValueError("Object snapshot timestamp must include a timezone")

        objects = {}
        for index, record in enumerate(payload["objects"]):
            normalized = _normalize_object(record, snapshot_time)
            if normalized is None:
                raise ValueError(f"Object record at index {index} is invalid")
            if normalized["object_id"] in objects:
                raise ValueError(
                    f"Duplicate object ID: {normalized['object_id']}"
                )
            objects[normalized["object_id"]] = normalized

        analyzed = analyze_relationships(list(objects.values()))
        for item in analyzed:
            evidence = item.get("load_verification")
            if (
                evidence is None
                or evidence.get("load_state") != "LOADED"
            ):
                continue
            item["relationships"].append(
                {
                    "type": "ON_TOP_OF",
                    "target_object_id": evidence["carrier_robot_id"],
                    "certainty": "SUPERVISOR_GEOMETRY",
                    "verified_at": evidence["observed_at"],
                }
            )
        with self._lock:
            self._objects = {
                item["object_id"]: item
                for item in analyzed
            }
            self._last_refresh = snapshot_time
        return self.snapshot()

    def snapshot(self):
        with self._lock:
            objects = [
                dict(record)
                for record in self._objects.values()
            ]
            objects.sort(key=lambda record: record["object_id"])
            relationships = [
                {
                    "object_id": record["object_id"],
                    **relationship,
                }
                for record in objects
                for relationship in record["relationships"]
            ]
            return {
                "objects": objects,
                "relationships": relationships,
                "lastRefresh": self._last_refresh,
                "coordinateSystem": COORDINATE_SYSTEM,
                "simulationConnected": bool(self._last_refresh),
            }

    def get(self, object_id):
        with self._lock:
            record = self._objects.get(object_id)
            return dict(record) if record is not None else None
