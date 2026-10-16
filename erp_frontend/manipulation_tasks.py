
"""Persistent ERP-side task service for UR10e object manipulation.

This module owns the business-task lifecycle. It deliberately does not
publish MQTT commands or claim physical placement has been verified.
"""

from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import tempfile
import threading
import uuid


TASK_STATES = {
    "QUEUED",
    "DISPATCHING",
    "DISPATCHED",
    "EXECUTING",
    "VERIFYING",
    "COMPLETED",
    "FAILED",
    "UNKNOWN",
}

ACTIVE_TASK_STATES = {
    "DISPATCHING",
    "DISPATCHED",
    "EXECUTING",
    "VERIFYING",
}

TERMINAL_TASK_STATES = {
    "COMPLETED",
    "FAILED",
}

ALLOWED_TRANSITIONS = {
    "QUEUED": {"DISPATCHING", "FAILED"},
    "DISPATCHING": {"DISPATCHED", "FAILED", "UNKNOWN"},
    "DISPATCHED": {"EXECUTING", "VERIFYING", "FAILED", "UNKNOWN"},
    "EXECUTING": {"VERIFYING", "FAILED", "UNKNOWN"},
    "VERIFYING": {"COMPLETED", "FAILED", "UNKNOWN"},
    "UNKNOWN": {"VERIFYING", "FAILED", "COMPLETED"},
    "COMPLETED": set(),
    "FAILED": set(),
}

REQUIRED_REQUEST_FIELDS = (
    "requestId",
    "objectId",
    "manufacturer",
    "robotId",
    "sourceId",
    "destinationId",
    "geometryProfileId",
)

MUTABLE_RESULT_FIELDS = {
    "robotTaskId",
    "dispatchState",
    "executionResult",
    "verificationResult",
    "failureReason",
    "recoveryReason",
}


def _timestamp() -> str:
    return (
        datetime.now(timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


class ManipulationTaskError(ValueError):
    """Base error for invalid manipulation-task operations."""


class ManipulationTaskConflictError(ManipulationTaskError):
    """Raised when a request conflicts with an existing task."""


class ManipulationTaskService:
    """Own durable ERP task records and allowed lifecycle transitions.

    The persistence lock coordinates threads in this process. This JSON
    implementation assumes a single writer process; it does not provide
    cross-process locking.
    """

    def __init__(self, data_file: str | Path | None = None):
        self._data_file = (
            Path(data_file)
            if data_file is not None
            else Path(__file__).with_name("manipulation_tasks.json")
        )
        self._lock = threading.RLock()
        self._tasks: dict[str, dict] = {}

    def load(self) -> None:
        """Load records and conservatively recover interrupted operations.

        Tasks that had reached dispatch or execution are changed to UNKNOWN.
        They must be reconciled with the fleet ledger before any further action.
        """
        if not self._data_file.exists():
            return

        try:
            payload = json.loads(
                self._data_file.read_text(encoding="utf-8")
            )
        except (OSError, json.JSONDecodeError) as error:
            raise RuntimeError(
                f"Could not read manipulation task state: {error}"
            ) from error

        if not isinstance(payload, dict):
            raise RuntimeError(
                "Manipulation task state must be a JSON object"
            )

        records = payload.get("tasks")
        if not isinstance(records, list):
            raise RuntimeError(
                "Manipulation task state must contain a tasks list"
            )

        loaded: dict[str, dict] = {}
        for task in records:
            if (
                not isinstance(task, dict)
                or not isinstance(task.get("taskId"), str)
                or not task["taskId"].strip()
                or task.get("status") not in TASK_STATES
            ):
                raise RuntimeError(
                    "Manipulation task state contains an invalid task record"
                )

            task_id = task["taskId"]
            if task_id in loaded:
                raise RuntimeError(
                    f"Duplicate manipulation task ID in storage: {task_id}"
                )
            loaded[task_id] = deepcopy(task)

        with self._lock:
            self._tasks = loaded
            changed = False

            for task in self._tasks.values():
                if task["status"] in ACTIVE_TASK_STATES:
                    reason = (
                        "ERP restarted while the task was active; "
                        "reconcile with fleet dispatch state before proceeding"
                    )
                    self._record_transition(
                        task,
                        "UNKNOWN",
                        reason,
                    )
                    task["recoveryRequired"] = True
                    task["recoveryReason"] = reason
                    changed = True

            if changed:
                self._save_locked()

    def create_task(self, request: dict) -> dict:
        """Create a queued task, or return the prior task for an identical request.

        requestId is the client idempotency key. Reusing it with different
        request fields is rejected rather than creating a second task.
        """
        normalized = self._validate_request(request)
        request_id = normalized["requestId"]

        with self._lock:
            for existing in self._tasks.values():
                if existing.get("requestId") != request_id:
                    continue

                existing_request = {
                    field: existing.get(field)
                    for field in REQUIRED_REQUEST_FIELDS
                }
                if existing_request != normalized:
                    raise ManipulationTaskConflictError(
                        "requestId has already been used with different task data"
                    )
                return deepcopy(existing)

            created_at = _timestamp()
            task = {
                "taskId": f"manip-{uuid.uuid4().hex}",
                **normalized,
                "taskType": "MANIPULATE_OBJECT",
                "status": "QUEUED",
                "createdAt": created_at,
                "updatedAt": created_at,
                "robotTaskId": None,
                "dispatchState": "NOT_DISPATCHED",
                "executionResult": None,
                "verificationResult": None,
                "failureReason": None,
                "recoveryRequired": False,
                "recoveryReason": None,
                "history": [
                    {
                        "status": "QUEUED",
                        "timestamp": created_at,
                        "message": "Manipulation task created",
                    }
                ],
            }

            self._tasks[task["taskId"]] = task
            try:
                self._save_locked()
            except OSError:
                self._tasks.pop(task["taskId"], None)
                raise

            return deepcopy(task)

    def transition_task(
        self,
        task_id: str,
        new_status: str,
        *,
        message: str | None = None,
        updates: dict | None = None,
    ) -> dict:
        """Persist an allowed state transition and optional result fields."""
        if new_status not in TASK_STATES:
            raise ManipulationTaskError(
                f"Unknown manipulation task state: {new_status}"
            )

        updates = updates or {}
        if not isinstance(updates, dict):
            raise ManipulationTaskError("updates must be an object")

        unexpected = set(updates) - MUTABLE_RESULT_FIELDS
        if unexpected:
            raise ManipulationTaskError(
                "Unsupported task update fields: "
                + ", ".join(sorted(unexpected))
            )

        with self._lock:
            task = self._tasks.get(task_id)
            if task is None:
                raise KeyError(f"Manipulation task not found: {task_id}")

            current_status = task["status"]
            if new_status not in ALLOWED_TRANSITIONS[current_status]:
                raise ManipulationTaskConflictError(
                    f"Invalid manipulation task transition: "
                    f"{current_status} -> {new_status}"
                )

            candidate = deepcopy(task)
            candidate.update(deepcopy(updates))

            if new_status == "COMPLETED":
                self._validate_completion_evidence(candidate)

            previous = deepcopy(task)
            self._record_transition(candidate, new_status, message)

            if new_status in {"COMPLETED", "FAILED"}:
                candidate["recoveryRequired"] = False
                candidate["recoveryReason"] = None
            elif new_status == "UNKNOWN":
                candidate["recoveryRequired"] = True
                candidate["recoveryReason"] = (
                    message or "Task outcome requires reconciliation"
                )

            self._tasks[task_id] = candidate
            try:
                self._save_locked()
            except OSError:
                self._tasks[task_id] = previous
                raise

            return deepcopy(candidate)

    def get_task(self, task_id: str) -> dict | None:
        with self._lock:
            task = self._tasks.get(task_id)
            return deepcopy(task) if task else None

    def list_tasks(self) -> list[dict]:
        with self._lock:
            records = deepcopy(list(self._tasks.values()))

        return sorted(
            records,
            key=lambda task: task.get("createdAt", ""),
            reverse=True,
        )

    @staticmethod
    def _validate_request(request: dict) -> dict:
        if not isinstance(request, dict):
            raise ManipulationTaskError(
                "A JSON manipulation-task request is required"
            )

        normalized = {}
        for field in REQUIRED_REQUEST_FIELDS:
            value = request.get(field)
            if not isinstance(value, str) or not value.strip():
                raise ManipulationTaskError(
                    f"{field} is required and must be a non-empty string"
                )
            normalized[field] = value.strip()

        if request.get("taskType", "MANIPULATE_OBJECT") != "MANIPULATE_OBJECT":
            raise ManipulationTaskError(
                "taskType must be MANIPULATE_OBJECT"
            )

        return normalized

    @staticmethod
    def _validate_completion_evidence(task: dict) -> None:
        evidence = task.get("verificationResult")
        if not isinstance(evidence, dict):
            raise ManipulationTaskError(
                "Cannot complete a task without placement verification evidence"
            )

        if evidence.get("verified") is not True:
            raise ManipulationTaskError(
                "Placement verification must explicitly be successful"
            )

        if evidence.get("objectId") != task.get("objectId"):
            raise ManipulationTaskError(
                "Verification evidence does not match the requested object"
            )

        if evidence.get("destinationId") != task.get("destinationId"):
            raise ManipulationTaskError(
                "Verification evidence does not match the requested destination"
            )

        if not isinstance(evidence.get("method"), str) or not evidence["method"].strip():
            raise ManipulationTaskError(
                "Verification evidence must identify its method"
            )

        if not isinstance(evidence.get("observedAt"), str) or not evidence["observedAt"].strip():
            raise ManipulationTaskError(
                "Verification evidence must include observedAt"
            )

    @staticmethod
    def _record_transition(
        task: dict,
        new_status: str,
        message: str | None,
    ) -> None:
        timestamp = _timestamp()
        task["status"] = new_status
        task["updatedAt"] = timestamp
        task["message"] = message
        task.setdefault("history", []).append(
            {
                "status": new_status,
                "timestamp": timestamp,
                "message": message,
            }
        )

    def _save_locked(self) -> None:
        self._data_file.parent.mkdir(parents=True, exist_ok=True)
        payload = json.dumps(
            {"tasks": list(self._tasks.values())},
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
