
"""API-level tests for ERP manipulation task creation and retrieval.

These tests mock the live Webots inventory, robot registry and the not-yet-
implemented approved manipulation configuration source. They do not test robot
motion, MQTT dispatch, or physical placement verification.
"""

import sys
import tempfile
import time
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path
from unittest.mock import patch

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ERP_DIR = PROJECT_ROOT / "erp_frontend"
FLEET_DIR = PROJECT_ROOT / "fleet_control"

for path in (str(FLEET_DIR), str(ERP_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

import app as erp
import routes.manipulation as manipulation_routes
from manipulation_tasks import ManipulationTaskService


def timestamp(delta_seconds=0):
    value = datetime.now(timezone.utc) + timedelta(
        seconds=delta_seconds
    )
    return value.isoformat(timespec="milliseconds").replace("+00:00", "Z")


def valid_request(**updates):
    payload = {
        "taskType": "MANIPULATE_OBJECT",
        "requestId": "api-request-001",
        "objectId": "webots-node:40972",
        "manufacturer": "DemoRobot",
        "robotId": "UR10e-source",
        "sourceId": "source-table",
        "destinationId": "destination-table",
        "geometryProfileId": "ur10e-source-bottle-v1",
    }
    payload.update(updates)
    return payload


def validate_test_configuration(payload):
    # Test-only allowlist; not a production geometry configuration.
    if payload["sourceId"] != "source-table":
        return (
            f"Unknown or unapproved source location: {payload['sourceId']}",
            400,
        )

    if payload["destinationId"] != "destination-table":
        return (
            "Unknown or unapproved destination location: "
            f"{payload['destinationId']}",
            400,
        )

    if payload["geometryProfileId"] != "ur10e-source-bottle-v1":
        return (
            "Unknown or unapproved geometry profile: "
            f"{payload['geometryProfileId']}",
            400,
        )

    return None


class ManipulationTaskApiTests(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.original = {
            "inventory": erp.object_inventory_snapshot,
            "received": erp.object_inventory_received_at,
            "robots": erp.robots,
            "service": erp.manipulation_task_service,
            "events": erp.events,
        }

        erp.robots = {}
        erp.events = []
        erp.manipulation_task_service = ManipulationTaskService(
            Path(self.tempdir.name) / "manipulation_tasks.json"
        )
        erp.manipulation_task_service.load()

        self.client = erp.app.test_client()
        self.config_patch = patch.object(
            manipulation_routes,
            "validate_approved_manipulation_configuration",
            side_effect=validate_test_configuration,
        )
        self.config_patch.start()

        self.set_inventory()
        erp.robots[("DemoRobot", "UR10e-source")] = {
            "robotId": "UR10e-source",
            "manufacturer": "DemoRobot",
            "robotType": "UR10e",
            "agentOnline": True,
            "present": True,
            "lastUpdate": timestamp(),
            "status": "IDLE",
            "available": True,
            "availabilityReason": None,
            "currentOrder": None,
            "recoveryRequired": False,
            "recoveryPending": False,
            "capabilities": ["PICK_AND_PLACE"],
        }

    def tearDown(self):
        self.config_patch.stop()
        erp.object_inventory_snapshot = self.original["inventory"]
        erp.object_inventory_received_at = self.original["received"]
        erp.robots = self.original["robots"]
        erp.manipulation_task_service = self.original["service"]
        erp.events = self.original["events"]
        self.tempdir.cleanup()

    def set_inventory(self, object_id="webots-node:40972", last_refresh=None):
        erp.object_inventory_snapshot = {
            "objects": [
                {"object_id": object_id, "name": "TestObject"}
            ],
            "relationships": [],
            "lastRefresh": last_refresh or timestamp(),
            "coordinateSystem": "X/Y floor plane; Z height",
        }
        erp.object_inventory_received_at = time.monotonic()

    def post(self, payload=None, raw=None):
        if raw is not None:
            return self.client.post(
                "/api/manipulation-tasks",
                data=raw,
                content_type="application/json",
            )
        return self.client.post("/api/manipulation-tasks", json=payload)

    def test_valid_request_creates_queued_task_without_dispatch(self):
        response = self.post(valid_request())

        self.assertEqual(response.status_code, 201, response.json)
        self.assertEqual(response.json["status"], "QUEUED")
        self.assertEqual(response.json["dispatchState"], "NOT_DISPATCHED")
        self.assertIsNone(response.json["robotTaskId"])
        self.assertEqual(
            len(erp.manipulation_task_service.list_tasks()), 1
        )

    def test_missing_fields_are_rejected(self):
        fields = (
            "taskType", "requestId", "objectId", "manufacturer",
            "robotId", "sourceId", "destinationId", "geometryProfileId",
        )

        for field in fields:
            with self.subTest(field=field):
                payload = valid_request()
                payload.pop(field)
                response = self.post(payload)
                self.assertEqual(response.status_code, 400)

    def test_malformed_json_is_rejected(self):
        response = self.post(raw='{"taskType": ')

        self.assertEqual(response.status_code, 400)
        self.assertIn("error", response.json)

    def test_unsupported_task_type_is_rejected(self):
        response = self.post(
            valid_request(taskType="TRANSPORT_OBJECT")
        )

        self.assertEqual(response.status_code, 400)

    def test_extra_coordinate_or_result_fields_are_rejected(self):
        response = self.post(
            valid_request(
                pickPosition={"x": 1, "y": 2, "z": 3}
            )
        )

        self.assertEqual(response.status_code, 400)

    def test_unknown_object_is_rejected(self):
        response = self.post(
            valid_request(objectId="missing-object")
        )

        self.assertEqual(response.status_code, 404)

    def test_stale_object_inventory_is_rejected(self):
        self.set_inventory(last_refresh=timestamp(-30))
        response = self.post(valid_request())

        self.assertEqual(response.status_code, 503)
        self.assertIn("stale", response.json["error"].lower())

    def test_unknown_robot_is_rejected(self):
        response = self.post(
            valid_request(robotId="missing-robot")
        )

        self.assertEqual(response.status_code, 404)

    def test_unavailable_robot_is_rejected(self):
        erp.robots[
            ("DemoRobot", "UR10e-source")
        ]["available"] = False

        response = self.post(valid_request())

        self.assertEqual(response.status_code, 409)

    def test_stale_robot_status_is_rejected(self):
        erp.robots[
            ("DemoRobot", "UR10e-source")
        ]["lastUpdate"] = timestamp(-30)

        response = self.post(valid_request())

        self.assertEqual(response.status_code, 409)
        self.assertIn("stale", response.json["error"].lower())

    def test_robot_without_manipulation_capability_is_rejected(self):
        erp.robots[
            ("DemoRobot", "UR10e-source")
        ]["capabilities"] = ["NAVIGATION"]

        response = self.post(valid_request())

        self.assertEqual(response.status_code, 409)
        self.assertIn("PICK_AND_PLACE", response.json["error"])

    def test_invalid_source_destination_and_profile_are_rejected(self):
        cases = (
            ("sourceId", "unknown-source"),
            ("destinationId", "unknown-destination"),
            ("geometryProfileId", "unknown-profile"),
        )

        for field, value in cases:
            with self.subTest(field=field):
                response = self.post(valid_request(**{field: value}))
                self.assertEqual(response.status_code, 400)

    def test_missing_approved_configuration_fails_closed(self):
        with patch.object(
            manipulation_routes,
            "validate_approved_manipulation_configuration",
            return_value=(
                "Approved manipulation location and geometry "
                "configuration is not configured",
                503,
            ),
        ):
            response = self.post(valid_request())

        self.assertEqual(response.status_code, 503)
        self.assertIn("configuration", response.json["error"].lower())

    def test_identical_request_id_and_payload_returns_existing_task(self):
        first = self.post(valid_request())
        retry = self.post(valid_request())

        self.assertEqual(first.status_code, 201)
        self.assertEqual(retry.status_code, 200)
        self.assertEqual(first.json["taskId"], retry.json["taskId"])
        self.assertEqual(
            len(erp.manipulation_task_service.list_tasks()), 1
        )

    def test_conflicting_payload_with_same_request_id_returns_conflict(self):
        self.assertEqual(self.post(valid_request()).status_code, 201)

        response = self.post(
            valid_request(objectId="another-object")
        )

        self.assertEqual(response.status_code, 409)
        self.assertIn("requestId", response.json["error"])

    def test_idempotent_retry_does_not_depend_on_current_robot_or_inventory(self):
        first = self.post(valid_request())
        erp.robots.clear()
        self.set_inventory(object_id="different-object")

        retry = self.post(valid_request())

        self.assertEqual(first.status_code, 201)
        self.assertEqual(retry.status_code, 200)
        self.assertEqual(first.json["taskId"], retry.json["taskId"])

    def test_existing_and_nonexistent_task_retrieval(self):
        created = self.post(valid_request())
        found = self.client.get(
            f"/api/manipulation-tasks/{created.json['taskId']}"
        )
        missing = self.client.get(
            "/api/manipulation-tasks/manip-does-not-exist"
        )

        self.assertEqual(found.status_code, 200)
        self.assertEqual(
            found.json["taskId"], created.json["taskId"]
        )
        self.assertEqual(missing.status_code, 404)

    def test_task_list_supports_pagination(self):
        self.post(valid_request())
        self.post(valid_request(requestId="api-request-002"))

        response = self.client.get(
            "/api/manipulation-tasks?limit=1&offset=1"
        )

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json["total"], 2)
        self.assertEqual(len(response.json["tasks"]), 1)

    def test_invalid_pagination_is_rejected(self):
        response = self.client.get(
            "/api/manipulation-tasks?limit=0"
        )

        self.assertEqual(response.status_code, 400)

    def test_persistence_error_returns_service_unavailable(self):
        with patch.object(
            erp.manipulation_task_service,
            "create_task",
            side_effect=OSError("disk unavailable"),
        ):
            response = self.post(valid_request())

        self.assertEqual(response.status_code, 503)
        self.assertIn("persist", response.json["error"].lower())

    def test_service_runtime_error_returns_service_unavailable(self):
        with patch.object(
            erp.manipulation_task_service,
            "create_task",
            side_effect=RuntimeError("storage failure"),
        ):
            response = self.post(valid_request())

        self.assertEqual(response.status_code, 503)


if __name__ == "__main__":
    unittest.main()
