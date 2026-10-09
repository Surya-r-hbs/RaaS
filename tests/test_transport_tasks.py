import sys
import time
import unittest
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

PROJECT_ROOT = Path(__file__).resolve().parents[1]
FLEET_DIR = PROJECT_ROOT / "fleet_control"
ERP_DIR = PROJECT_ROOT / "erp_frontend"
for path in (str(FLEET_DIR), str(ERP_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from config import MAP_NODE_POSITIONS
from transport_tasks import TransportTaskManager
import app as erp


def timestamp():
    return (
        datetime.now(timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


def object_record(object_id="wood-box", load_state="LOADED", position=None):
    record = {
        "object_id": object_id,
        "name": "WoodenBox",
        "object_type": "wooden_box",
        "source": "webots_simulation",
        "position": position or {"x": 0.0, "y": 0.0, "z": 0.3},
        "rotation": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "dimensions": {"x": 0.6, "y": 0.6, "z": 0.6},
        "mass_kg": 10.0,
        "geometry": {
            "bounds": {
                "min": {"x": -0.3, "y": -0.3, "z": -0.3},
                "max": {"x": 0.3, "y": 0.3, "z": 0.3},
            },
        },
        "relationships": [],
        "load_verification": {
            "verified": True,
            "method": "SUPERVISOR_GEOMETRY",
            "load_state": load_state,
            "object_id": object_id,
            "carrier_robot_id": "Youbot",
            "carrier_manufacturer": "DemoRobot",
            "observed_at": timestamp(),
        },
    }
    return record


def robot_record(
    robot_id="Youbot",
    current_node="N2",
    available=True,
    capability=True,
):
    record = {
        "robotId": robot_id,
        "manufacturer": "DemoRobot",
        "robotType": "Youbot",
        "agentOnline": True,
        "lastUpdate": timestamp(),
        "status": "IDLE" if available else "EXECUTING",
        "available": available,
        "currentOrder": None if available else "other-order",
        "currentNode": current_node,
        "capabilities": ["NAVIGATION"],
    }
    if capability:
        record["transportCapability"] = {
            "status": "VERIFIED",
            "canCarryPreloaded": True,
            "stabilityVerified": True,
            "maxPayloadKg": 20.0,
            "platformDimensions": {"x": 1.0, "y": 1.0},
        }
    return record


class TransportTaskManagerTests(unittest.TestCase):
    def setUp(self):
        self.manager = TransportTaskManager(MAP_NODE_POSITIONS)
        self.robot = robot_record()
        self.object = object_record()
        self.inventory = {
            "objects": [self.object],
            "lastRefresh": timestamp(),
        }
        self.manager.observe_inventory(self.inventory, [self.robot])

    def create_ready_task(self, object_value=None, robot=None):
        return self.manager.create_task(
            object_value or self.object,
            "N5",
            [robot or self.robot],
        )

    def test_valid_object_and_destination_create_task(self):
        task = self.create_ready_task(object_record(load_state="UNLOADED"))

        self.assertEqual(task["status"], "LOAD_REQUIRED")
        self.assertEqual(task["objectId"], "wood-box")
        self.assertEqual(task["destinationNode"], "N5")

    def test_unknown_object_and_destination_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "Object not found"):
            self.manager.create_task({}, "N5", [self.robot])
        with self.assertRaisesRegex(ValueError, "Unknown destination"):
            self.manager.create_task(self.object, "N99", [self.robot])

    def test_unloaded_object_produces_load_required(self):
        self.manager = TransportTaskManager(MAP_NODE_POSITIONS)
        task = self.manager.create_task(
            object_record(load_state="UNLOADED"),
            "N5",
            [self.robot],
        )

        self.assertEqual(task["status"], "LOAD_REQUIRED")
        self.assertIn("no current Webots-verified load association", task["message"])

    def test_verified_preloaded_object_becomes_dispatchable(self):
        task = self.create_ready_task()

        self.assertEqual(task["status"], "READY_FOR_DISPATCH")
        self.assertEqual(task["carrierRobotId"], "Youbot")
        self.assertEqual(task["loadState"], "LOADED")

    def test_repeated_load_evidence_does_not_rewrite_persisted_state(self):
        with patch.object(self.manager, "_save_locked") as save:
            self.manager.observe_inventory(self.inventory, [self.robot])

        save.assert_not_called()

    def test_reloading_after_verified_unload_uses_fresh_evidence_time(self):
        unloaded = object_record(load_state="UNLOADED")
        self.manager.observe_inventory({"objects": [unloaded]}, [self.robot])

        loaded = object_record(load_state="LOADED")
        observed_at = timestamp()
        loaded["load_verification"]["observed_at"] = observed_at
        self.manager.observe_inventory({"objects": [loaded]}, [self.robot])

        load = self.manager.get_load("wood-box")
        self.assertEqual(load["state"], "LOADED")
        self.assertEqual(load["verifiedAt"], observed_at)

    def test_unavailable_or_unverified_carrier_cannot_be_reserved(self):
        unavailable = robot_record(available=False)
        task = self.create_ready_task(robot=unavailable)
        self.assertEqual(task["status"], "CREATED")
        self.assertFalse(self.manager.reserve(task["taskId"], self.object, unavailable))

        unverified = robot_record(capability=False)
        self.manager = TransportTaskManager(MAP_NODE_POSITIONS)
        self.manager.observe_inventory(self.inventory, [unverified])
        incompatible = self.manager.create_task(
            self.object,
            "N5",
            [unverified],
        )
        self.assertEqual(incompatible["status"], "FAILED")
        self.assertIn("unverified", incompatible["message"])

    def test_incompatible_payload_or_footprint_is_rejected(self):
        too_heavy = robot_record()
        too_heavy["transportCapability"]["maxPayloadKg"] = 9
        self.manager = TransportTaskManager(MAP_NODE_POSITIONS)
        self.manager.observe_inventory(self.inventory, [too_heavy])
        task = self.manager.create_task(self.object, "N5", [too_heavy])

        self.assertEqual(task["status"], "FAILED")
        self.assertIn("exceeds", task["message"])

    def test_two_tasks_cannot_reserve_the_same_object(self):
        first = self.create_ready_task()
        second = self.create_ready_task()

        self.assertTrue(self.manager.reserve(first["taskId"], self.object, self.robot))
        self.assertFalse(self.manager.reserve(second["taskId"], self.object, self.robot))

    def test_two_tasks_cannot_reserve_the_same_robot(self):
        another_object = object_record("crate", position={"x": 1, "y": 0, "z": 0.3})
        another_object["load_verification"]["object_id"] = "crate"
        self.manager.observe_inventory(
            {"objects": [self.object, another_object]},
            [self.robot],
        )
        first = self.manager.create_task(self.object, "N5", [self.robot])
        second = self.manager.create_task(another_object, "N4", [self.robot])

        self.assertTrue(self.manager.reserve(first["taskId"], self.object, self.robot))
        self.assertFalse(
            self.manager.reserve(second["taskId"], another_object, self.robot)
        )

    def test_navigation_requires_fresh_delivery_evidence(self):
        task = self.create_ready_task()
        self.assertTrue(self.manager.reserve(task["taskId"], self.object, self.robot))
        order_id = "transport-order"
        self.assertTrue(self.manager.attach_order(task["taskId"], order_id))
        self.assertTrue(self.manager.mark_dispatched(task["taskId"]))
        self.manager.update_order_state(order_id, "COMPLETED", "N5")

        self.assertEqual(
            self.manager.get(task["taskId"])["status"],
            "VERIFYING_DELIVERY",
        )
        at_destination = object_record(
            position={"x": 6.0, "y": 3.0, "z": 0.3}
        )
        completed = self.manager.verify_deliveries(
            {"objects": [at_destination]},
            [robot_record(current_node="N5")],
        )

        self.assertEqual(completed, [task["taskId"]])
        self.assertEqual(self.manager.get(task["taskId"])["status"], "COMPLETED")
        self.assertEqual(
            self.manager.get_load("wood-box")["state"],
            "LOADED",
        )

    def test_failure_never_marks_object_delivered_or_clears_load(self):
        task = self.create_ready_task()
        self.manager.reserve(task["taskId"], self.object, self.robot)
        self.manager.attach_order(task["taskId"], "failed-order")
        self.manager.mark_dispatched(task["taskId"])
        self.manager.update_order_state("failed-order", "ERROR")
        completed = self.manager.verify_deliveries(
            {"objects": [object_record(position={"x": 6, "y": 3, "z": 0.3})]},
            [robot_record(current_node="N5")],
        )

        self.assertEqual(completed, [])
        self.assertEqual(
            self.manager.get(task["taskId"])["status"],
            "RECOVERY_REQUIRED",
        )
        self.assertEqual(
            self.manager.get_load("wood-box")["state"],
            "RECOVERY_REQUIRED",
        )

    def test_inventory_position_is_not_mutated_by_task_completion(self):
        task = self.create_ready_task()
        self.manager.reserve(task["taskId"], self.object, self.robot)
        self.manager.attach_order(task["taskId"], "delivery-order")
        self.manager.mark_dispatched(task["taskId"])
        self.manager.update_order_state("delivery-order", "COMPLETED", "N5")
        observed = object_record(position={"x": 6.0, "y": 3.0, "z": 0.3})
        original_position = dict(observed["position"])

        self.manager.verify_deliveries(
            {"objects": [observed]},
            [robot_record(current_node="N5")],
        )

        self.assertEqual(observed["position"], original_position)
        self.assertEqual(
            self.manager.get(task["taskId"])["deliveryVerification"]["method"],
            "WEBOTS_OBJECT_POSE_AND_LOAD_EVIDENCE",
        )

    def test_restart_marks_active_task_and_load_for_recovery(self):
        import json
        import tempfile

        with tempfile.TemporaryDirectory() as directory:
            state_path = Path(directory) / "transport_tasks.json"
            first_manager = TransportTaskManager(MAP_NODE_POSITIONS, state_path)
            first_manager.observe_inventory(self.inventory, [self.robot])
            task = first_manager.create_task(self.object, "N5", [self.robot])
            first_manager.reserve(task["taskId"], self.object, self.robot)
            first_manager.attach_order(task["taskId"], "restart-order")
            first_manager.mark_dispatched(task["taskId"])

            restarted = TransportTaskManager(MAP_NODE_POSITIONS, state_path)
            restarted.load()

            self.assertEqual(
                restarted.get(task["taskId"])["status"],
                "RECOVERY_REQUIRED",
            )
            self.assertEqual(
                restarted.get_load("wood-box")["state"],
                "RECOVERY_REQUIRED",
            )
            self.assertTrue(json.loads(state_path.read_text())["loads"])

    def test_verified_unload_releases_carrier_without_erasing_history(self):
        task = self.create_ready_task()
        self.assertTrue(self.manager.get_load("wood-box"))
        unloaded = object_record(load_state="UNLOADED")
        self.manager.observe_inventory({"objects": [unloaded]}, [self.robot])

        self.assertEqual(
            self.manager.get_load("wood-box")["state"],
            "NOT_LOADED",
        )
        self.assertEqual(
            self.manager.get(task["taskId"])["status"],
            "LOAD_REQUIRED",
        )


class TransportTaskApiTests(unittest.TestCase):
    def setUp(self):
        self.client = erp.app.test_client()
        self.previous = {
            "inventory": erp.object_inventory_snapshot,
            "received": erp.object_inventory_received_at,
            "robots": erp.robots,
            "tasks": erp.transport_task_manager,
            "orders": erp.orders,
            "dispatching": erp.dispatching_robots,
            "events": erp.events,
        }
        erp.robots = {}
        erp.orders = {}
        erp.dispatching_robots = set()
        erp.transport_task_manager = TransportTaskManager(MAP_NODE_POSITIONS)
        self.set_inventory(object_record(load_state="UNLOADED"))

    def tearDown(self):
        erp.object_inventory_snapshot = self.previous["inventory"]
        erp.object_inventory_received_at = self.previous["received"]
        erp.robots = self.previous["robots"]
        erp.transport_task_manager = self.previous["tasks"]
        erp.orders = self.previous["orders"]
        erp.dispatching_robots = self.previous["dispatching"]
        erp.events = self.previous["events"]

    def set_inventory(self, record):
        erp.object_inventory_snapshot = {
            "objects": [record],
            "relationships": [],
            "lastRefresh": timestamp(),
            "coordinateSystem": "X/Y floor plane; Z height",
        }
        erp.object_inventory_received_at = time.monotonic()

    def test_api_rejects_unknown_object_and_destination(self):
        unknown_object = self.client.post(
            "/api/transport-tasks",
            json={
                "taskType": "TRANSPORT_OBJECT",
                "objectId": "missing",
                "destinationNode": "N5",
                "handlingMode": "PRELOADED",
            },
        )
        unknown_node = self.client.post(
            "/api/transport-tasks",
            json={
                "taskType": "TRANSPORT_OBJECT",
                "objectId": "wood-box",
                "destinationNode": "N99",
                "handlingMode": "PRELOADED",
            },
        )

        self.assertEqual(unknown_object.status_code, 404)
        self.assertEqual(unknown_node.status_code, 400)

    def test_api_creates_load_required_task_and_exposes_world_nodes(self):
        response = self.client.post(
            "/api/transport-tasks",
            json={
                "taskType": "TRANSPORT_OBJECT",
                "objectId": "wood-box",
                "destinationNode": "N5",
                "handlingMode": "PRELOADED",
            },
        )
        options = self.client.get("/api/transport-options")

        self.assertEqual(response.status_code, 201)
        self.assertEqual(response.json["status"], "LOAD_REQUIRED")
        self.assertIn(
            "N5",
            {node["nodeId"] for node in options.json["destinations"]},
        )

    def test_verified_load_dispatch_uses_existing_vda_order_path(self):
        loaded = object_record()
        self.set_inventory(loaded)
        robot = robot_record()
        erp.robots = {("DemoRobot", "Youbot"): robot}

        created = self.client.post(
            "/api/transport-tasks",
            json={
                "taskType": "TRANSPORT_OBJECT",
                "objectId": "wood-box",
                "destinationNode": "N5",
                "handlingMode": "PRELOADED",
            },
        )
        self.assertEqual(created.status_code, 201)
        self.assertEqual(created.json["status"], "READY_FOR_DISPATCH")

        with patch.object(erp, "save_orders"), patch.object(
            erp.mqtt_client,
            "publish",
            return_value=SimpleNamespace(rc=erp.mqtt.MQTT_ERR_SUCCESS),
        ):
            dispatched = self.client.post(
                f"/api/transport-tasks/{created.json['taskId']}/dispatch"
            )

        self.assertEqual(dispatched.status_code, 202)
        self.assertEqual(dispatched.json["task"]["status"], "DISPATCHED")
        sent_order = dispatched.json["order"]
        self.assertEqual(
            [node["nodeId"] for node in sent_order["nodes"]],
            ["N2", "N5"],
        )
        self.assertNotIn("taskId", sent_order)
        self.assertNotIn("objectId", sent_order)

    def test_definite_publish_rejection_releases_task_reservations(self):
        self.set_inventory(object_record())
        erp.robots = {("DemoRobot", "Youbot"): robot_record()}
        created = self.client.post(
            "/api/transport-tasks",
            json={
                "taskType": "TRANSPORT_OBJECT",
                "objectId": "wood-box",
                "destinationNode": "N5",
                "handlingMode": "PRELOADED",
            },
        )

        with patch.object(erp, "save_orders"), patch.object(
            erp.mqtt_client,
            "publish",
            return_value=SimpleNamespace(rc=erp.mqtt.MQTT_ERR_NO_CONN),
        ):
            response = self.client.post(
                f"/api/transport-tasks/{created.json['taskId']}/dispatch"
            )

        self.assertEqual(response.status_code, 503)
        self.assertEqual(
            erp.transport_task_manager.get(created.json["taskId"])["status"],
            "READY_FOR_DISPATCH",
        )
        self.assertEqual(
            erp.transport_task_manager.get_load("wood-box")["state"],
            "LOADED",
        )

    def test_uncertain_publish_outcome_keeps_load_reserved_for_recovery(self):
        self.set_inventory(object_record())
        erp.robots = {("DemoRobot", "Youbot"): robot_record()}
        created = self.client.post(
            "/api/transport-tasks",
            json={
                "taskType": "TRANSPORT_OBJECT",
                "objectId": "wood-box",
                "destinationNode": "N5",
                "handlingMode": "PRELOADED",
            },
        )

        with patch.object(erp, "save_orders"), patch.object(
            erp.mqtt_client,
            "publish",
            side_effect=RuntimeError("connection outcome unknown"),
        ):
            response = self.client.post(
                f"/api/transport-tasks/{created.json['taskId']}/dispatch"
            )

        self.assertEqual(response.status_code, 503)
        self.assertEqual(
            erp.transport_task_manager.get(created.json["taskId"])["status"],
            "RECOVERY_REQUIRED",
        )
        self.assertEqual(
            erp.transport_task_manager.get_load("wood-box")["state"],
            "RECOVERY_REQUIRED",
        )


if __name__ == "__main__":
    unittest.main()
