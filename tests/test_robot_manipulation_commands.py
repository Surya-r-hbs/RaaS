
"""Tests for safe acceptance of UR10e manipulation commands."""

import queue
import sys
import types
import unittest
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONTROLLER_DIR = (
    PROJECT_ROOT / "webots/controllers/robot_controller"
)
sys.path.insert(0, str(CONTROLLER_DIR))

# Stub the deployment logger so these unit tests can run independently.
logging_stub = types.ModuleType("raas_logging")
logging_stub.log_event = lambda *args, **kwargs: None
sys.modules.setdefault("raas_logging", logging_stub)

from robot_commands import handle_command


class Adapter:
    capabilities = {"PICK_AND_PLACE"}
    task_active = False
    manipulation_status = None


class Controller:
    def __init__(
        self,
        state="IDLE",
        capabilities=None,
        task_active=False,
        queued=False,
    ):
        self.logger = SimpleNamespace(
            info=lambda *args, **kwargs: None,
            log=lambda *args, **kwargs: None,
        )
        self.robot_id = "UR10e-N0"
        self.robot_type = "UR10e"
        self.adapter = Adapter()
        self.adapter.capabilities = (
            {"PICK_AND_PLACE"}
            if capabilities is None
            else capabilities
        )
        self.adapter.task_active = task_active
        self.state = SimpleNamespace(state=state)
        self.manipulation_commands = queue.Queue()

        if queued:
            self.manipulation_commands.put(
                {"taskId": "already-queued"}
            )

        self.last_error = None
        self.published = 0

    def publish_status(self):
        self.published += 1


class RobotManipulationCommandTests(unittest.TestCase):
    def command(self, task_id="task-1"):
        return {
            "command": "PICK_AND_PLACE",
            "taskId": task_id,
            "pickPosition": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.2,
            },
            "placePosition": {
                "x": 0.4,
                "y": 0.0,
                "z": 0.2,
            },
        }

    def test_accepts_supported_task_once(self):
        controller = Controller()

        handle_command(controller, self.command())

        self.assertEqual(
            controller.manipulation_commands.qsize(), 1
        )
        self.assertEqual(
            controller.last_manipulation_task_id, "task-1"
        )
        self.assertIsNone(controller.last_error)

    def test_rejects_missing_task_id(self):
        controller = Controller()
        command = self.command()
        command.pop("taskId")

        handle_command(controller, command)

        self.assertTrue(
            controller.manipulation_commands.empty()
        )
        self.assertIn("requires taskId", controller.last_error)

    def test_rejects_robot_without_capability(self):
        controller = Controller(capabilities=set())

        handle_command(controller, self.command())

        self.assertTrue(
            controller.manipulation_commands.empty()
        )
        self.assertIn("not supported", controller.last_error)

    def test_rejects_while_navigating(self):
        controller = Controller(state="EXECUTING")

        handle_command(controller, self.command())

        self.assertTrue(
            controller.manipulation_commands.empty()
        )
        self.assertIn("navigating", controller.last_error)

    def test_rejects_while_manipulation_active(self):
        controller = Controller(task_active=True)

        handle_command(controller, self.command())

        self.assertTrue(
            controller.manipulation_commands.empty()
        )
        self.assertIn("active manipulation", controller.last_error)

    def test_duplicate_task_id_is_not_queued_twice(self):
        controller = Controller()

        handle_command(controller, self.command())
        handle_command(controller, self.command())

        self.assertEqual(
            controller.manipulation_commands.qsize(), 1
        )

    def test_rejects_second_different_queued_task(self):
        controller = Controller(queued=True)

        handle_command(controller, self.command())

        self.assertEqual(
            controller.manipulation_commands.qsize(), 1
        )
        self.assertIn("already queued", controller.last_error)


if __name__ == "__main__":
    unittest.main()
