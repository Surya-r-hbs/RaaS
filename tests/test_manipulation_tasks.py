
import sys
import tempfile
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ERP_DIR = PROJECT_ROOT / "erp_frontend"

if str(ERP_DIR) not in sys.path:
    sys.path.insert(0, str(ERP_DIR))

from manipulation_tasks import (
    ManipulationTaskConflictError,
    ManipulationTaskError,
    ManipulationTaskService,
)


def make_request(**updates):
    request = {
        "requestId": "client-request-001",
        "objectId": "webots-node:40972",
        "manufacturer": "DemoRobot",
        "robotId": "UR10e-source",
        "sourceId": "source-table",
        "destinationId": "destination-table",
        "geometryProfileId": "ur10e-source-bottle-v1",
        "taskType": "MANIPULATE_OBJECT",
    }
    request.update(updates)
    return request


def make_verification(task):
    return {
        "verified": True,
        "objectId": task["objectId"],
        "destinationId": task["destinationId"],
        "method": "TEST_FIXTURE",
        "observedAt": "2026-10-16T10:00:00.000Z",
    }


class ManipulationTaskServiceTests(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.data_file = Path(self.temporary_directory.name) / "tasks.json"
        self.service = ManipulationTaskService(self.data_file)
        self.service.load()

    def tearDown(self):
        self.temporary_directory.cleanup()

    def test_create_task_persists_queued_record(self):
        task = self.service.create_task(make_request())

        self.assertEqual(task["status"], "QUEUED")
        self.assertEqual(task["objectId"], "webots-node:40972")
        self.assertEqual(task["dispatchState"], "NOT_DISPATCHED")
        self.assertTrue(self.data_file.exists())

        reloaded = ManipulationTaskService(self.data_file)
        reloaded.load()
        self.assertEqual(reloaded.get_task(task["taskId"])["status"], "QUEUED")

    def test_same_request_id_and_payload_are_idempotent(self):
        first = self.service.create_task(make_request())
        second = self.service.create_task(make_request())

        self.assertEqual(first["taskId"], second["taskId"])
        self.assertEqual(len(self.service.list_tasks()), 1)

    def test_reusing_request_id_with_different_object_is_rejected(self):
        self.service.create_task(make_request())

        with self.assertRaises(ManipulationTaskConflictError):
            self.service.create_task(
                make_request(objectId="webots-node:another-object")
            )

    def test_required_fields_are_validated(self):
        with self.assertRaisesRegex(ManipulationTaskError, "objectId"):
            self.service.create_task(make_request(objectId=" "))

    def test_invalid_lifecycle_transition_is_rejected(self):
        task = self.service.create_task(make_request())

        with self.assertRaises(ManipulationTaskConflictError):
            self.service.transition_task(task["taskId"], "COMPLETED")

        self.assertEqual(
            self.service.get_task(task["taskId"])["status"],
            "QUEUED",
        )

    def test_completion_requires_matching_verification_evidence(self):
        task = self.service.create_task(make_request())
        self.service.transition_task(task["taskId"], "DISPATCHING")
        self.service.transition_task(task["taskId"], "DISPATCHED")
        self.service.transition_task(task["taskId"], "EXECUTING")
        self.service.transition_task(task["taskId"], "VERIFYING")

        with self.assertRaisesRegex(ManipulationTaskError, "verification"):
            self.service.transition_task(task["taskId"], "COMPLETED")

        with self.assertRaisesRegex(ManipulationTaskError, "object"):
            self.service.transition_task(
                task["taskId"],
                "COMPLETED",
                updates={
                    "verificationResult": {
                        **make_verification(task),
                        "objectId": "wrong-object",
                    }
                },
            )

        completed = self.service.transition_task(
            task["taskId"],
            "COMPLETED",
            updates={"verificationResult": make_verification(task)},
        )
        self.assertEqual(completed["status"], "COMPLETED")

    def test_restart_marks_active_task_unknown_and_does_not_replay_it(self):
        task = self.service.create_task(make_request())
        self.service.transition_task(task["taskId"], "DISPATCHING")
        self.service.transition_task(task["taskId"], "DISPATCHED")
        self.service.transition_task(task["taskId"], "EXECUTING")

        recovered = ManipulationTaskService(self.data_file)
        recovered.load()

        result = recovered.get_task(task["taskId"])
        self.assertEqual(result["status"], "UNKNOWN")
        self.assertTrue(result["recoveryRequired"])
        self.assertEqual(len(recovered.list_tasks()), 1)

        with self.assertRaises(ManipulationTaskConflictError):
            recovered.transition_task(task["taskId"], "DISPATCHING")

    def test_unknown_task_can_enter_verification_after_reconciliation(self):
        task = self.service.create_task(make_request())
        self.service.transition_task(task["taskId"], "DISPATCHING")
        self.service.transition_task(task["taskId"], "UNKNOWN")

        result = self.service.transition_task(
            task["taskId"],
            "VERIFYING",
            message="Fleet reconciliation confirmed execution ended",
        )
        self.assertEqual(result["status"], "VERIFYING")

    def test_terminal_task_cannot_be_restarted(self):
        task = self.service.create_task(make_request())
        self.service.transition_task(task["taskId"], "FAILED")

        with self.assertRaises(ManipulationTaskConflictError):
            self.service.transition_task(task["taskId"], "DISPATCHING")


if __name__ == "__main__":
    unittest.main()