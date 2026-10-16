"""ERP API for creating and inspecting UR10e manipulation tasks.

Production task creation remains disabled until an approved manipulation
location/geometry configuration source is provided.
"""

import re

from flask import (
    Blueprint,
    current_app,
    jsonify,
    request,
)

from manipulation_tasks import (
    ManipulationTaskConflictError,
    ManipulationTaskError,
)

bp = Blueprint("manipulation", __name__)

TASK_TYPE = "MANIPULATE_OBJECT"
REQUEST_FIELDS = (
    "taskType",
    "requestId",
    "objectId",
    "manufacturer",
    "robotId",
    "sourceId",
    "destinationId",
    "geometryProfileId",
)
ID_PATTERN = re.compile(r"^[A-Za-z0-9_.:-]{1,128}$")
MAX_LIST_LIMIT = 200
DEFAULT_LIST_LIMIT = 50


def _erp():
    """Access the current ERP state without importing app.py."""
    return current_app.config["ERP_STATE"]()


def _error(message, status_code):
    return jsonify({"error": message}), status_code


def _parse_request(payload):
    if not isinstance(payload, dict):
        raise ManipulationTaskError(
            "A JSON manipulation-task request is required"
        )

    unexpected = sorted(set(payload) - set(REQUEST_FIELDS))
    if unexpected:
        raise ManipulationTaskError(
            "Unsupported request field(s): " + ", ".join(unexpected)
        )

    if payload.get("taskType") != TASK_TYPE:
        raise ManipulationTaskError(
            "taskType must be MANIPULATE_OBJECT"
        )

    normalized = {}
    for field in REQUEST_FIELDS[1:]:
        value = payload.get(field)

        if not isinstance(value, str):
            raise ManipulationTaskError(
                f"{field} is required and must be a string"
            )

        value = value.strip()

        if not value:
            raise ManipulationTaskError(
                f"{field} is required and must be a non-empty string"
            )

        if len(value) > 128:
            raise ManipulationTaskError(
                f"{field} must be at most 128 characters"
            )

        if not ID_PATTERN.fullmatch(value):
            raise ManipulationTaskError(
                f"{field} contains unsupported characters"
            )

        normalized[field] = value

    normalized["taskType"] = TASK_TYPE
    return normalized


def _request_fields_for_idempotency(task):
    return {
        field: task.get(field)
        for field in REQUEST_FIELDS[1:]
    }


def _existing_request_result(service, normalized):
    """Return (existing task, conflict) before live-state checks on retries."""
    for task in service.list_tasks():
        if task.get("requestId") != normalized["requestId"]:
            continue

        expected = {
            field: normalized[field]
            for field in REQUEST_FIELDS[1:]
        }

        if _request_fields_for_idempotency(task) != expected:
            return None, True

        return task, False

    return None, False


def _find_live_object(object_id):
    erp = _erp()
    inventory, status_code = erp["get_object_inventory_response"]()

    if status_code != 200:
        return None, (
            inventory.get("error") or "Object inventory is unavailable",
            503,
        )

    record = next(
        (
            item
            for item in inventory.get("objects", [])
            if isinstance(item, dict)
            and item.get("object_id") == object_id
        ),
        None,
    )

    if record is None:
        return None, (
            f"Object not found in live inventory: {object_id}",
            404,
        )

    return record, None


def _robot_unavailability_reason(robot):
    erp = _erp()

    if robot is None:
        return "Robot is not registered"

    if robot.get("agentOnline") is not True:
        return "Robot agent is offline"

    if not erp["robot_status_is_fresh"](robot):
        return "Robot status is stale"

    if robot.get("present") is False:
        return "Robot is not present in the Webots world"

    if robot.get("recoveryRequired") or robot.get("recoveryPending"):
        return robot.get("recoveryReason") or "Robot requires recovery"

    if robot.get("currentOrder"):
        return f"Robot has active assignment {robot['currentOrder']}"

    if robot.get("status") not in {"IDLE", "STOPPED"}:
        return (
            "Robot is not operational "
            f"({robot.get('status', 'UNKNOWN')})"
        )

    if robot.get("available") is not True:
        return robot.get("availabilityReason") or "Robot is unavailable"

    capabilities = robot.get("capabilities")
    if (
        not isinstance(capabilities, list)
        or "PICK_AND_PLACE" not in capabilities
    ):
        return "Robot does not advertise PICK_AND_PLACE capability"

    return None


def validate_approved_manipulation_configuration(normalized):
    """Fail closed until approved location and geometry configuration exists."""
    return (
        "Approved manipulation location and geometry configuration "
        "is not configured",
        503,
    )


@bp.post("/api/manipulation-tasks")
def create_manipulation_task():
    erp = _erp()
    payload = request.get_json(silent=True)

    if not isinstance(payload, dict):
        return _error(
            "A valid JSON manipulation-task request is required",
            400,
        )

    try:
        normalized = _parse_request(payload)
    except ManipulationTaskError as error:
        return _error(str(error), 400)

    service = erp["manipulation_task_service"]

    # Resolve retries before live-state checks.
    existing, conflicting = _existing_request_result(
        service, normalized
    )

    if conflicting:
        return _error(
            "requestId has already been used with different task data",
            409,
        )

    if existing is not None:
        return jsonify(existing), 200

    _, inventory_error = _find_live_object(normalized["objectId"])
    if inventory_error:
        message, status_code = inventory_error
        return _error(message, status_code)

    robot_key = (
        normalized["manufacturer"],
        normalized["robotId"],
    )

    with erp["data_lock"]:
        robot = erp["robots"].get(robot_key)
        robot = dict(robot) if robot is not None else None

    if robot is None:
        return _error(
            "Unknown robot: "
            f"{normalized['manufacturer']}/{normalized['robotId']}",
            404,
        )

    reason = _robot_unavailability_reason(robot)
    if reason:
        return _error(
            f"Selected robot is unavailable: {reason}",
            409,
        )

    configuration_result = (
        validate_approved_manipulation_configuration(normalized)
    )

    if configuration_result is not None:
        message, status_code = configuration_result
        return _error(message, status_code)

    try:
        task = service.create_task(normalized)

    except ManipulationTaskConflictError as error:
        return _error(str(error), 409)

    except ManipulationTaskError as error:
        return _error(str(error), 400)

    except (OSError, RuntimeError) as error:
        erp["add_event"](
            "MANIPULATION_TASK_PERSISTENCE_ERROR",
            str(error),
        )
        return _error(
            "Could not persist manipulation task",
            503,
        )

    erp["add_event"](
        "MANIPULATION_TASK_CREATED",
        (
            f"Manipulation task {task['taskId']} created "
            f"in state {task['status']}"
        ),
        task,
    )

    return jsonify(task), 201


@bp.get("/api/manipulation-tasks")
def list_manipulation_tasks():
    erp = _erp()

    try:
        limit = int(request.args.get("limit", DEFAULT_LIST_LIMIT))
        offset = int(request.args.get("offset", 0))
    except (TypeError, ValueError):
        return _error("limit and offset must be integers", 400)

    if not 1 <= limit <= MAX_LIST_LIMIT or offset < 0:
        return _error(
            f"limit must be 1-{MAX_LIST_LIMIT} "
            "and offset must be non-negative",
            400,
        )

    tasks = erp["manipulation_task_service"].list_tasks()

    return jsonify(
        {
            "tasks": tasks[offset:offset + limit],
            "total": len(tasks),
            "limit": limit,
            "offset": offset,
        }
    )


@bp.get("/api/manipulation-tasks/<task_id>")
def get_manipulation_task(task_id):
    if not ID_PATTERN.fullmatch(task_id or ""):
        return _error("Invalid manipulation task ID", 400)

    service = _erp()["manipulation_task_service"]
    task = service.get_task(task_id)

    if task is None:
        return _error("Manipulation task not found", 404)

    return jsonify(task)