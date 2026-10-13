"""Persistent business-task and verified-load state for object transport."""

from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import tempfile
import threading
import uuid

from config import SIMULATION_TRANSPORT_ENABLED


ACTIVE_TASK_STATES = {
    "DISPATCHING",
    "DISPATCHED",
    "EXECUTING",
    "VERIFYING_DELIVERY",
}
LOAD_EVIDENCE_METHOD = "SUPERVISOR_GEOMETRY"
LOAD_EVIDENCE_MAX_AGE_SECONDS = 5.0


def _timestamp():
    return (
        datetime.now(timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


def _number(value):
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


class TransportConflictError(ValueError):
    """A task conflicts with an active or uncertain object reservation."""


class LoadManager:
    """Track exclusive, Webots-verified object-to-carrier associations."""

    def __init__(self, records=None, lock=None):
        self._records = records if records is not None else {}
        self._lock = lock or threading.RLock()

    @staticmethod
    def verify_evidence(
        object_record,
        carrier_robot_id,
        manufacturer=None,
    ):
        evidence = object_record.get("load_verification")
        if not isinstance(evidence, dict):
            return None
        if (
            evidence.get("verified") is not True
            or evidence.get("method") != LOAD_EVIDENCE_METHOD
            or evidence.get("load_state") not in {"LOADED", "UNLOADED"}
            or evidence.get("object_id") != object_record.get("object_id")
            or evidence.get("carrier_robot_id") != carrier_robot_id
            or (
                manufacturer is not None
                and evidence.get("carrier_manufacturer") != manufacturer
            )
            or not isinstance(evidence.get("carrier_manufacturer"), str)
            or not isinstance(evidence.get("observed_at"), str)
        ):
            return None
        try:
            observed_at = datetime.fromisoformat(
                evidence["observed_at"].replace("Z", "+00:00")
            )
        except ValueError:
            return None
        if observed_at.tzinfo is None:
            return None
        age = (datetime.now(timezone.utc) - observed_at).total_seconds()
        if not 0 <= age <= LOAD_EVIDENCE_MAX_AGE_SECONDS:
            return None
        return evidence

    @staticmethod
    def check_compatibility(object_record, robot):
        capability = robot.get("transportCapability")
        if not isinstance(capability, dict):
            return "Carrier transport capability is unverified"
        capability_mode = capability.get("mode")
        verified_profile = (
            capability.get("status") == "VERIFIED"
            and capability.get("canCarryPreloaded") is True
            and capability.get("stabilityVerified") is True
        )
        simulation_profile = (
            capability.get("status") == "SIMULATION_ONLY"
            and capability_mode == "SIMULATION_ONLY"
            and SIMULATION_TRANSPORT_ENABLED
            and robot.get("runtimeMode") == "WEBOTS_SIMULATION"
            and robot.get("simulationModeEnabled") is True
            and capability.get("canCarryPreloaded") is True
            and capability.get("stabilityMode") == "SIMULATION_ASSUMED"
            and isinstance(capability.get("stabilityAssumption"), str)
            and bool(capability["stabilityAssumption"].strip())
        )
        if not (verified_profile or simulation_profile):
            reason = capability.get("reason")
            if isinstance(reason, str) and reason.strip():
                return f"Carrier transport capability is unverified: {reason.strip()}"
            if capability.get("status") == "SIMULATION_ONLY":
                if not SIMULATION_TRANSPORT_ENABLED:
                    return (
                        "Simulation-only transport profile is disabled; set "
                        "RAAS_WEBOTS_SIMULATION_MODE=1 in both Webots and Fleet"
                    )
                if (
                    robot.get("runtimeMode") != "WEBOTS_SIMULATION"
                    or robot.get("simulationModeEnabled") is not True
                ):
                    return (
                        "Simulation-only transport is restricted to a Webots "
                        "robot with simulation mode explicitly enabled"
                    )
                return "Simulation-only carrier capability profile is incomplete"
            return (
                "Carrier carrying or stability capability is unverified "
                "or unsupported"
            )
        if "NAVIGATION" not in robot.get("capabilities", []):
            return "Carrier navigation capability is not implemented"

        max_payload = _number(capability.get("maxPayloadKg"))
        object_mass = _number(object_record.get("mass_kg"))
        platform = capability.get("platformDimensions")
        geometry = object_record.get("geometry")
        bounds = geometry.get("bounds") if isinstance(geometry, dict) else None
        rotation = object_record.get("rotation")
        if (
            max_payload is None
            or object_mass is None
            or not isinstance(platform, dict)
            or not isinstance(bounds, dict)
            or not isinstance(bounds.get("min"), dict)
            or not isinstance(bounds.get("max"), dict)
            or not isinstance(rotation, list)
            or len(rotation) != 9
            or any(_number(value) is None for value in rotation)
        ):
            return (
                "Carrier payload, platform dimensions, object geometry, or orientation "
                "are unknown"
            )
        rotation = [_number(value) for value in rotation]
        platform_x = _number(platform.get("x"))
        platform_y = _number(platform.get("y"))
        if platform_x is None or platform_y is None:
            return "Carrier platform dimensions are unknown"
        minimum = {
            axis: _number(bounds["min"].get(axis))
            for axis in ("x", "y", "z")
        }
        maximum = {
            axis: _number(bounds["max"].get(axis))
            for axis in ("x", "y", "z")
        }
        if any(
            minimum[axis] is None
            or maximum[axis] is None
            or minimum[axis] >= maximum[axis]
            for axis in ("x", "y", "z")
        ):
            return "Object geometry bounds are invalid"
        robot_rotation = robot.get("orientation")
        if (
            not isinstance(robot_rotation, (list, tuple))
            or len(robot_rotation) != 9
            or any(_number(value) is None for value in robot_rotation)
        ):
            return (
                "Carrier orientation is unavailable for object footprint "
                "validation"
            )
        relative_rotation = [
            sum(
                _number(robot_rotation[k * 3 + row])
                * _number(rotation[k * 3 + column])
                for k in range(3)
            )
            for row in range(3)
            for column in range(3)
        ]
        points = [
            (x, y, z)
            for x in (minimum["x"], maximum["x"])
            for y in (minimum["y"], maximum["y"])
            for z in (minimum["z"], maximum["z"])
        ]
        transformed = [
            (
                relative_rotation[0] * x
                + relative_rotation[1] * y
                + relative_rotation[2] * z,
                relative_rotation[3] * x
                + relative_rotation[4] * y
                + relative_rotation[5] * z,
            )
            for x, y, z in points
        ]
        object_x = max(point[0] for point in transformed) - min(
            point[0] for point in transformed
        )
        object_y = max(point[1] for point in transformed) - min(
            point[1] for point in transformed
        )
        if object_mass <= 0:
            return "Object mass must be positive for transport"
        if max_payload <= 0 or platform_x <= 0 or platform_y <= 0:
            return "Configured carrier payload/platform limits must be positive"
        if object_mass > max_payload:
            return (
                f"Object mass {object_mass:g} kg exceeds the configured "
                f"{max_payload:g} kg carrier payload limit"
            )
        if object_x > platform_x or object_y > platform_y:
            return "Object footprint exceeds the carrier support platform"
        return None

    def observe_verified(self, object_record, robot, evidence):
        object_id = object_record["object_id"]
        carrier_id = evidence["carrier_robot_id"]
        manufacturer = evidence["carrier_manufacturer"]
        with self._lock:
            existing = self._records.get(object_id)
            if (
                existing is not None
                and (
                    existing.get("carrierRobotId") != carrier_id
                    or existing.get("manufacturer") != manufacturer
                )
                and existing.get("state")
                in {"LOADED", "IN_TRANSIT", "RECOVERY_REQUIRED"}
            ):
                existing["state"] = "RECOVERY_REQUIRED"
                existing["updatedAt"] = _timestamp()
                return True
            if evidence["load_state"] == "UNLOADED":
                if (
                    existing is not None
                    and existing.get("carrierRobotId") == carrier_id
                    and existing.get("manufacturer") == manufacturer
                    and existing.get("state")
                    in {"LOADED", "IN_TRANSIT", "RECOVERY_REQUIRED"}
                ):
                    existing.update(
                        {
                            "lastCarrierRobotId": carrier_id,
                            "lastManufacturer": manufacturer,
                            "carrierRobotId": None,
                            "manufacturer": None,
                            "state": "NOT_LOADED",
                            "verificationStatus": "VERIFIED_UNLOADED",
                            "verifiedAt": evidence["observed_at"],
                            "updatedAt": _timestamp(),
                        }
                    )
                    return True
                return False
            load_state = "LOADED"
            if existing and existing.get("state") in {
                "IN_TRANSIT",
                "RECOVERY_REQUIRED",
            }:
                load_state = existing["state"]
            current = {
                "objectId": object_id,
                "carrierRobotId": carrier_id,
                "manufacturer": robot.get("manufacturer"),
                "state": load_state,
                "loadingMethod": "PRELOADED",
                "verificationStatus": "VERIFIED",
                "verificationMethod": LOAD_EVIDENCE_METHOD,
                "verifiedAt": evidence["observed_at"],
                "transportTaskId": (
                    existing.get("transportTaskId")
                    if existing
                    else None
                ),
                "updatedAt": _timestamp(),
            }
            if existing and all(
                existing.get(field) == current[field]
                for field in (
                    "carrierRobotId",
                    "manufacturer",
                    "state",
                    "loadingMethod",
                    "verificationStatus",
                    "verificationMethod",
                    "transportTaskId",
                )
            ):
                return False
            if (
                existing
                and existing.get("carrierRobotId") == carrier_id
                and existing.get("manufacturer") == manufacturer
                and existing.get("state")
                in {"LOADED", "IN_TRANSIT", "RECOVERY_REQUIRED"}
            ):
                current["verifiedAt"] = existing.get(
                    "verifiedAt",
                    evidence["observed_at"],
                )
            self._records[object_id] = current
            return True

    def get(self, object_id):
        with self._lock:
            record = self._records.get(object_id)
            return json.loads(json.dumps(record)) if record else None

    def has_unreleased_load(self, manufacturer, robot_id):
        with self._lock:
            return any(
                record.get("manufacturer") == manufacturer
                and record.get("carrierRobotId") == robot_id
                and record.get("state")
                in {"LOADED", "IN_TRANSIT", "RECOVERY_REQUIRED"}
                for record in self._records.values()
            )

    def reserve(self, record, task_id):
        with self._lock:
            if not record or record.get("state") != "LOADED":
                return False
            record["state"] = "IN_TRANSIT"
            record["transportTaskId"] = task_id
            record["updatedAt"] = _timestamp()
            return True

    def set_state(self, record, state):
        with self._lock:
            if record is None:
                return
            record["state"] = state
            record["updatedAt"] = _timestamp()


class TransportTaskManager:
    """Own transport task history and persisted load reservations."""

    def __init__(self, node_positions, data_file=None):
        self._node_positions = dict(node_positions)
        self._data_file = Path(data_file) if data_file else None
        self._lock = threading.RLock()
        self._tasks = {}
        self._loads = {}
        self.load_manager = LoadManager(self._loads, self._lock)

    def load(self):
        if self._data_file is None or not self._data_file.exists():
            return
        try:
            payload = json.loads(self._data_file.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise RuntimeError(
                f"Could not read transport task state: {error}"
            ) from error
        if not isinstance(payload, dict):
            raise RuntimeError("Transport task state must be a JSON object")
        tasks = payload.get("tasks", [])
        loads = payload.get("loads", {})
        if not isinstance(tasks, list) or not isinstance(loads, dict):
            raise RuntimeError("Transport task state has an invalid schema")
        with self._lock:
            self._tasks = {
                task["taskId"]: task
                for task in tasks
                if isinstance(task, dict) and task.get("taskId")
            }
            self._loads.clear()
            self._loads.update(
                {
                    object_id: record
                    for object_id, record in loads.items()
                    if isinstance(object_id, str) and isinstance(record, dict)
                }
            )
            changed = False
            for task in self._tasks.values():
                if task.get("status") in ACTIVE_TASK_STATES:
                    self._transition(
                        task,
                        "RECOVERY_REQUIRED",
                        "ERP restarted while transport or delivery verification was active",
                    )
                    changed = True
            for record in self._loads.values():
                if record.get("state") == "IN_TRANSIT":
                    self.load_manager.set_state(record, "RECOVERY_REQUIRED")
                    changed = True
            if changed:
                self._save_locked()

    def _save_locked(self):
        if self._data_file is None:
            return
        self._data_file.parent.mkdir(parents=True, exist_ok=True)
        payload = json.dumps(
            {
                "tasks": list(self._tasks.values()),
                "loads": self._loads,
            },
            indent=2,
        )
        temporary_path = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                dir=self._data_file.parent,
                prefix=f".{self._data_file.name}.",
                suffix=".tmp",
                delete=False,
            ) as temporary_file:
                temporary_path = temporary_file.name
                temporary_file.write(payload)
                temporary_file.flush()
                os.fsync(temporary_file.fileno())
            os.replace(temporary_path, self._data_file)
        except OSError:
            if temporary_path and os.path.exists(temporary_path):
                os.unlink(temporary_path)
            raise

    @staticmethod
    def _transition(task, status, reason=None):
        previous = task.get("status")
        previous_message = task.get("message")
        if previous == status and previous_message == reason:
            return False
        task["status"] = status
        task["updatedAt"] = _timestamp()
        task["message"] = reason
        task.setdefault("history", []).append(
            {
                "status": status,
                "timestamp": task["updatedAt"],
                "message": reason,
            }
        )
        return previous != status or previous_message != reason

    @staticmethod
    def _valid_load_evidence(
        object_record,
        carrier_robot_id,
        manufacturer=None,
    ):
        return LoadManager.verify_evidence(
            object_record,
            carrier_robot_id,
            manufacturer,
        )

    def observe_inventory(self, inventory, robots):
        objects = inventory.get("objects", [])
        robot_by_id = {
            (robot.get("manufacturer"), robot.get("robotId")): robot
            for robot in robots
            if isinstance(robot, dict) and robot.get("robotId")
        }
        with self._lock:
            changed = False
            for object_record in objects:
                if not isinstance(object_record, dict):
                    continue
                evidence = object_record.get("load_verification")
                if not isinstance(evidence, dict):
                    continue
                carrier_id = evidence.get("carrier_robot_id")
                manufacturer = evidence.get("carrier_manufacturer")
                if not isinstance(carrier_id, str) or not carrier_id:
                    continue
                if not isinstance(manufacturer, str) or not manufacturer:
                    continue
                if self._valid_load_evidence(object_record, carrier_id) is None:
                    continue
                robot = robot_by_id.get((manufacturer, carrier_id))
                if robot is None and evidence.get("load_state") == "LOADED":
                    continue
                changed = self.load_manager.observe_verified(
                    object_record,
                    robot or {"robotId": carrier_id, "manufacturer": manufacturer},
                    evidence,
                ) or changed
            changed = self._reconcile_waiting_locked(objects, robots) or changed
            if changed:
                self._save_locked()

    def _reconcile_waiting_locked(self, objects, robots):
        objects_by_id = {
            item.get("object_id"): item
            for item in objects
            if isinstance(item, dict)
        }
        robots_by_key = {
            (item.get("manufacturer"), item.get("robotId")): item
            for item in robots
            if isinstance(item, dict)
        }
        changed = False
        for task in self._tasks.values():
            if task.get("status") not in {
                "LOAD_REQUIRED",
                "CREATED",
                "READY_FOR_DISPATCH",
            }:
                continue
            load = self._loads.get(task.get("objectId"))
            if load and load.get("state") in {
                "IN_TRANSIT",
                "RECOVERY_REQUIRED",
            }:
                changed = self._transition(
                    task,
                    "RECOVERY_REQUIRED",
                    "The object's carrier load is active or uncertain",
                ) or changed
                continue
            if load is None or load.get("state") != "LOADED":
                if task.get("status") == "READY_FOR_DISPATCH":
                    task["loadState"] = "NOT_LOADED"
                    changed = self._transition(
                        task,
                        "LOAD_REQUIRED",
                        "Fresh Webots evidence no longer confirms the object on its carrier",
                    ) or changed
                continue
            carrier_id = load.get("carrierRobotId")
            object_record = objects_by_id.get(task.get("objectId"))
            if object_record is None:
                if task.get("status") == "READY_FOR_DISPATCH":
                    changed = self._transition(
                        task,
                        "LOAD_REQUIRED",
                        "Transport object is absent from the fresh Webots inventory",
                    ) or changed
                continue
            if task.get("carrierRobotId") and task["carrierRobotId"] != carrier_id:
                changed = self._transition(
                    task,
                    "FAILED",
                    "The object was verified on a different carrier; transfer is not implemented",
                ) or changed
                continue
            robot = robots_by_key.get((load.get("manufacturer"), carrier_id))
            if robot is None:
                if task.get("status") == "READY_FOR_DISPATCH":
                    changed = self._transition(
                        task,
                        "CREATED",
                        "Verified carrier is not present in the current fleet",
                    ) or changed
                continue
            if not robot.get("available") and task.get("status") == "READY_FOR_DISPATCH":
                changed = self._transition(
                    task,
                    "CREATED",
                    robot.get("availabilityReason") or "Carrier is unavailable",
                ) or changed
                continue
            if object_record is None:
                continue
            evidence = self._valid_load_evidence(
                object_record,
                carrier_id,
                robot.get("manufacturer"),
            )
            if evidence is None or evidence.get("load_state") != "LOADED":
                if task.get("status") == "READY_FOR_DISPATCH":
                    task["loadState"] = "NOT_LOADED"
                    changed = self._transition(
                        task,
                        "LOAD_REQUIRED",
                        "Fresh Webots evidence no longer confirms the object on its carrier",
                    ) or changed
                continue
            reason = self._check_carrier(object_record, robot)
            if reason:
                changed = self._transition(task, "FAILED", reason) or changed
                continue
            if not robot.get("available"):
                continue
            task["carrierRobotId"] = carrier_id
            task["manufacturer"] = robot.get("manufacturer")
            task["loadState"] = "LOADED"
            changed = self._transition(task, "READY_FOR_DISPATCH") or changed
        return changed

    def reconcile(self, inventory, robots):
        with self._lock:
            changed = self._reconcile_waiting_locked(
                inventory.get("objects", []),
                robots,
            )
            if changed:
                self._save_locked()

    def _check_carrier(self, object_record, robot):
        return self.load_manager.check_compatibility(object_record, robot)

    def create_task(
        self,
        object_record,
        destination_node,
        robots,
        carrier_robot_id=None,
        manufacturer=None,
    ):
        if not isinstance(object_record, dict) or not object_record.get("object_id"):
            raise ValueError("Object not found in the live inventory")
        if destination_node not in self._node_positions:
            raise ValueError(f"Unknown destination node: {destination_node}")

        object_id = object_record["object_id"]
        with self._lock:
            stored_load = self._loads.get(object_id)
            if stored_load and stored_load.get("state") in {
                "IN_TRANSIT",
                "RECOVERY_REQUIRED",
            }:
                raise TransportConflictError(
                    f"Object {object_id} already has an active or uncertain transport"
                )
            current_evidence = (
                self._valid_load_evidence(
                    object_record,
                    stored_load.get("carrierRobotId"),
                    stored_load.get("manufacturer"),
                )
                if stored_load
                else None
            )
            load = (
                json.loads(json.dumps(stored_load))
                if stored_load
                and current_evidence
                and current_evidence.get("load_state") == "LOADED"
                else None
            )
        if load is None:
            carrier_id = None
            carrier = None
        else:
            carrier_id = load["carrierRobotId"]
            expected_manufacturer = manufacturer or load.get("manufacturer")
            carrier = next(
                (
                    robot
                    for robot in robots
                    if robot.get("robotId") == carrier_id
                    and robot.get("manufacturer") == expected_manufacturer
                ),
                None,
            )
        if carrier_robot_id and carrier_id and (
            carrier_robot_id != carrier_id
            or (
                manufacturer
                and load
                and manufacturer != load.get("manufacturer")
            )
        ):
            raise ValueError(
                "A loaded object must remain with its verified carrier; "
                "handling/transfer is not implemented"
            )
        if carrier_robot_id and not carrier_id:
            carrier_id = carrier_robot_id

        now = _timestamp()
        task_id = f"transport-{uuid.uuid4().hex}"
        task = {
            "taskId": task_id,
            "taskType": "TRANSPORT_OBJECT",
            "objectId": object_id,
            "objectName": object_record.get("name"),
            "destinationNode": destination_node,
            "handlingMode": "PRELOADED",
            "status": "CREATED",
            "createdAt": now,
            "updatedAt": now,
            "message": None,
            "carrierRobotId": carrier_id,
            "manufacturer": (
                carrier.get("manufacturer") if carrier else manufacturer
            ),
            "orderId": None,
            "loadState": load.get("state") if load else "NOT_LOADED",
            "history": [],
        }
        self._transition(task, "VALIDATING")
        if load is None or load.get("state") not in {
            "LOADED",
            "IN_TRANSIT",
        }:
            self._transition(
                task,
                "LOAD_REQUIRED",
                object_record.get("load_verification_reason")
                or "The object has no current Webots-verified load association. "
                "Physically load it and refresh the Webots inventory.",
            )
        elif carrier is None:
            self._transition(
                task,
                "FAILED",
                "The verified carrier is not present in the current fleet registry",
            )
        else:
            reason = self._check_carrier(object_record, carrier)
            if reason:
                self._transition(task, "FAILED", reason)
            elif not carrier.get("available"):
                self._transition(
                    task,
                    "CREATED",
                    carrier.get("availabilityReason") or "Carrier is unavailable",
                )
            else:
                task["loadState"] = "LOADED"
                self._transition(task, "READY_FOR_DISPATCH")

        with self._lock:
            self._tasks[task_id] = task
            self._save_locked()
            return json.loads(json.dumps(task))

    def get(self, task_id):
        with self._lock:
            task = self._tasks.get(task_id)
            return json.loads(json.dumps(task)) if task else None

    def list_tasks(self):
        with self._lock:
            return [
                json.loads(json.dumps(task))
                for task in sorted(
                    self._tasks.values(),
                    key=lambda item: item["createdAt"],
                    reverse=True,
                )
            ]

    def get_load(self, object_id):
        return self.load_manager.get(object_id)

    def has_unreleased_load(self, manufacturer, robot_id):
        return self.load_manager.has_unreleased_load(manufacturer, robot_id)

    def task_authorizes_carrier(self, task_id, manufacturer, robot_id):
        if not task_id:
            return False
        with self._lock:
            task = self._tasks.get(task_id)
            load = (
                self._loads.get(task.get("objectId"))
                if task is not None
                else None
            )
            return bool(
                task
                and task.get("manufacturer") == manufacturer
                and task.get("carrierRobotId") == robot_id
                and task.get("status")
                in {"READY_FOR_DISPATCH", *ACTIVE_TASK_STATES}
                and load
                and load.get("carrierRobotId") == robot_id
                and load.get("manufacturer") == manufacturer
            )

    def task_id_for_order(self, order_id):
        if not order_id:
            return None
        with self._lock:
            task = next(
                (
                    item
                    for item in self._tasks.values()
                    if item.get("orderId") == order_id
                ),
                None,
            )
            return task.get("taskId") if task else None

    def reserve(self, task_id, object_record, robot):
        with self._lock:
            task = self._tasks.get(task_id)
            if not isinstance(robot, dict):
                return False
            load = self._loads.get(task.get("objectId")) if task else None
            evidence = (
                self._valid_load_evidence(
                    object_record,
                    robot.get("robotId"),
                    robot.get("manufacturer"),
                )
                if isinstance(object_record, dict)
                else None
            )
            if (
                task is None
                or task.get("status") != "READY_FOR_DISPATCH"
                or load is None
                or evidence is None
                or evidence.get("load_state") != "LOADED"
                or load.get("state") != "LOADED"
                or load.get("carrierRobotId") != robot.get("robotId")
                or load.get("manufacturer") != robot.get("manufacturer")
                or self._check_carrier(object_record, robot)
                or not robot.get("available")
                or robot.get("currentOrder")
            ):
                return False
            if any(
                other.get("taskId") != task_id
                and other.get("carrierRobotId") == robot.get("robotId")
                and other.get("manufacturer") == robot.get("manufacturer")
                and other.get("status") in ACTIVE_TASK_STATES
                for other in self._tasks.values()
            ):
                return False
            if not self.load_manager.reserve(load, task_id):
                return False
            task["loadState"] = "IN_TRANSIT"
            self._transition(task, "DISPATCHING")
            self._save_locked()
            return True

    def attach_order(self, task_id, order_id):
        with self._lock:
            task = self._tasks.get(task_id)
            if task is None or task.get("status") != "DISPATCHING":
                return False
            task["orderId"] = order_id
            task.setdefault("orderHistory", []).append(order_id)
            task["updatedAt"] = _timestamp()
            self._save_locked()
            return True

    def mark_dispatched(self, task_id):
        with self._lock:
            task = self._tasks.get(task_id)
            if (
                task is None
                or task.get("status") != "DISPATCHING"
                or not task.get("orderId")
            ):
                return False
            self._transition(task, "DISPATCHED")
            self._save_locked()
            return True

    def rollback_dispatch(self, task_id):
        with self._lock:
            task = self._tasks.get(task_id)
            if task is None or task.get("status") != "DISPATCHING":
                return False
            load = self._loads.get(task.get("objectId"))
            if load is not None and load.get("transportTaskId") == task_id:
                self.load_manager.set_state(load, "LOADED")
            if task.get("orderId"):
                task.setdefault("failedOrderIds", []).append(task["orderId"])
                task["orderId"] = None
            task["loadState"] = "LOADED"
            self._transition(task, "READY_FOR_DISPATCH")
            self._save_locked()
            return True

    def mark_recovery_required(self, task_id, reason):
        with self._lock:
            task = self._tasks.get(task_id)
            if task is None:
                return False
            self._transition(task, "RECOVERY_REQUIRED", reason)
            load = self._loads.get(task.get("objectId"))
            if load is not None and load.get("transportTaskId") == task_id:
                self.load_manager.set_state(load, "RECOVERY_REQUIRED")
            self._save_locked()
            return True

    def update_order_state(self, order_id, state, node_id=None):
        with self._lock:
            task = next(
                (
                    item for item in self._tasks.values()
                    if item.get("orderId") == order_id
                ),
                None,
            )
            if task is None:
                return None
            previous_status = task.get("status")
            previous_message = task.get("message")
            previous_node = task.get("currentNode")
            if state == "DISPATCHED":
                self._transition(task, "DISPATCHED")
            elif state in {"EXECUTING", "NODE_REACHED"}:
                task["currentNode"] = node_id or task.get("currentNode")
                if node_id == task.get("destinationNode"):
                    self._transition(
                        task,
                        "VERIFYING_DELIVERY",
                        "Navigation reached the destination; awaiting fresh object/load evidence",
                    )
                else:
                    self._transition(task, "EXECUTING")
            elif state == "COMPLETED":
                self._transition(
                    task,
                    "VERIFYING_DELIVERY",
                    "Navigation completed; awaiting fresh object/load evidence",
                )
            elif state in {
                "ERROR",
                "FAILED",
                "STOPPED",
                "RECOVERY_REQUIRED",
            }:
                self._transition(
                    task,
                    "RECOVERY_REQUIRED",
                    f"Carrier mission ended in {state}; load association is retained",
                )
                load = self._loads.get(task["objectId"])
                if load is not None:
                    self.load_manager.set_state(load, "RECOVERY_REQUIRED")
            if (
                task.get("status") != previous_status
                or task.get("message") != previous_message
                or task.get("currentNode") != previous_node
            ):
                self._save_locked()
            return json.loads(json.dumps(task))

    def verify_deliveries(self, inventory, robots):
        objects_by_id = {
            item.get("object_id"): item
            for item in inventory.get("objects", [])
            if isinstance(item, dict)
        }
        robot_by_id = {
            (item.get("manufacturer"), item.get("robotId")): item
            for item in robots
            if isinstance(item, dict)
        }
        completed = []
        with self._lock:
            for task in self._tasks.values():
                if task.get("status") != "VERIFYING_DELIVERY":
                    continue
                object_record = objects_by_id.get(task.get("objectId"))
                if object_record is None:
                    continue
                robot = robot_by_id.get(
                    (task.get("manufacturer"), task["carrierRobotId"])
                )
                if robot is None:
                    continue
                evidence = self._valid_load_evidence(
                    object_record,
                    task["carrierRobotId"],
                    task.get("manufacturer"),
                )
                if evidence is None:
                    continue
                if evidence.get("load_state") != "LOADED":
                    continue
                if robot.get("currentNode") != task.get("destinationNode"):
                    continue
                position = object_record.get("position")
                dimensions = object_record.get("dimensions")
                target = self._node_positions.get(task["destinationNode"])
                if not isinstance(position, dict) or target is None:
                    continue
                x = _number(position.get("x"))
                y = _number(position.get("y"))
                if isinstance(dimensions, dict):
                    dx = _number(dimensions.get("x"))
                    dy = _number(dimensions.get("y"))
                    if None in (dx, dy, x, y):
                        continue
                    extent = math.hypot(dx, dy) / 2.0
                else:
                    continue
                distance = math.hypot(x - target[0], y - target[1])
                if distance > extent + 0.2:
                    continue
                task["deliveryVerification"] = {
                    "verified": True,
                    "method": "WEBOTS_OBJECT_POSE_AND_LOAD_EVIDENCE",
                    "objectPosition": position,
                    "carrierNode": robot.get("currentNode"),
                    "verifiedAt": _timestamp(),
                }
                task["loadState"] = "LOADED"
                load = self._loads.get(task["objectId"])
                if load is not None:
                    self.load_manager.set_state(load, "LOADED")
                self._transition(
                    task,
                    "COMPLETED",
                    "Object pose and verified carrier load were confirmed at the destination",
                )
                completed.append(task["taskId"])
            if completed:
                self._save_locked()
        return completed
