import importlib
import json
import sys
import tempfile
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ERP_DIR = PROJECT_ROOT / "erp_frontend"
FLEET_DIR = PROJECT_ROOT / "fleet_control"

sys.path.insert(0, str(ERP_DIR))
sys.path.insert(0, str(FLEET_DIR))

erp = importlib.import_module("app")
from robot_registry import RobotRegistry
fleet = importlib.import_module("fleet_controller")


def make_order(order_id, status="CREATED", robot_id="youBot"):
    return {
        "orderId": order_id,
        "manufacturer": "DemoRobot",
        "serialNumber": robot_id,
        "status": status,
        "nodes": [
            {"nodeId": "N1", "sequenceId": 0},
            {"nodeId": "N2", "sequenceId": 2},
        ],
        "edges": [{"edgeId": "E1", "sequenceId": 1}],
    }


def make_robot(**updates):
    robot = {
        "robotId": "youBot",
        "manufacturer": "DemoRobot",
        "robotType": "Youbot",
        "status": "IDLE",
        "available": True,
        "availabilityReason": None,
        "agentOnline": True,
        "present": True,
        "currentNode": "N1",
        "currentEdge": None,
        "currentOrder": None,
        "lastOrderId": None,
        "lastOrderStatus": None,
        "recoveryRequired": False,
        "recoveryPending": False,
        "capabilities": ["NAVIGATION"],
        "lastUpdate": erp.now_utc(),
    }
    robot.update(updates)
    return robot


class OrderLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.original_data_file = erp.DATA_FILE
        erp.DATA_FILE = Path(self.temporary_directory.name) / "orders.json"
        with erp.data_lock:
            erp.orders.clear()
            erp.robots.clear()
            erp.events.clear()
            erp.recovery_requests.clear()
            erp.dispatching_robots.clear()
        self.client = erp.app.test_client()
        self.publish = patch.object(
            erp.mqtt_client,
            "publish",
            return_value=SimpleNamespace(rc=erp.mqtt.MQTT_ERR_SUCCESS),
        )
        self.mock_publish = self.publish.start()
        erp.robots[("DemoRobot", "youBot")] = make_robot()

    def tearDown(self):
        self.publish.stop()
        erp.DATA_FILE = self.original_data_file
        self.temporary_directory.cleanup()

    def add_order(self, order_id, status="CREATED"):
        order = make_order(order_id, status)
        erp.orders[order_id] = order
        return order

    def dispatch(self, order_id):
        return self.client.post(f"/api/orders/{order_id}/dispatch")

    def test_created_order_dispatches_without_historical_reservation(self):
        self.add_order("old", "COMPLETED")
        order = self.add_order("new")

        response = self.dispatch("new")

        self.assertEqual(response.status_code, 200)
        self.assertEqual(order["status"], "DISPATCHED")
        self.assertEqual(
            erp.robots[("DemoRobot", "youBot")]["currentOrder"],
            "new",
        )

    def test_active_assignment_blocks_second_order(self):
        robot = erp.robots[("DemoRobot", "youBot")]
        robot.update(
            status="EXECUTING",
            available=False,
            availabilityReason="Active mission running",
            currentOrder="active",
        )
        self.add_order("active", "EXECUTING")
        self.add_order("second")

        response = self.dispatch("second")

        self.assertEqual(response.status_code, 409)
        self.assertEqual(erp.orders["active"]["status"], "EXECUTING")
        self.assertEqual(robot["currentOrder"], "active")

    def test_completed_stopped_and_created_history_do_not_reserve(self):
        self.add_order("completed", "COMPLETED")
        self.add_order("stopped", "STOPPED")
        self.add_order("created", "CREATED")

        response = self.dispatch("created")

        self.assertEqual(response.status_code, 200)

    def test_robot_dispatches_consecutive_orders_after_confirmed_completion(self):
        original_registry = fleet.robot_registry
        original_missions = fleet.missions
        fleet.robot_registry = RobotRegistry()
        fleet.missions = {}
        self.addCleanup(
            setattr,
            fleet,
            "robot_registry",
            original_registry,
        )
        self.addCleanup(
            setattr,
            fleet,
            "missions",
            original_missions,
        )

        publish_result = SimpleNamespace(rc=erp.mqtt.MQTT_ERR_SUCCESS)
        with patch.object(
            fleet.mqtt_client,
            "publish",
            return_value=publish_result,
        ):
            fleet.robot_registry.register_agent(
                {
                    "robotId": "youBot",
                    "manufacturer": "DemoRobot",
                    "agentSessionId": "session",
                    "capabilities": ["NAVIGATION"],
                }
            )
            fleet.robot_registry.update_status(
                {
                    "robotId": "youBot",
                    "manufacturer": "DemoRobot",
                    "agentSessionId": "session",
                    "statusSequence": 1,
                    "state": "IDLE",
                }
            )
            erp.update_robot_registry(
                {"robots": fleet.robot_registry.snapshot()}
            )

            first_order = self.add_order("first")
            first_order["orderUpdateId"] = 0
            first_order["version"] = "3.0.0"
            self.assertEqual(self.dispatch("first").status_code, 200)
            fleet.handle_order(first_order)
            fleet.handle_robot_status(
                {
                    "robotId": "youBot",
                    "manufacturer": "DemoRobot",
                    "agentSessionId": "session",
                    "statusSequence": 2,
                    "state": "EXECUTING",
                    "orderId": "first",
                    "nodeId": "N2",
                    "edgeId": "E1",
                }
            )
            fleet.handle_robot_status(
                {
                    "robotId": "youBot",
                    "manufacturer": "DemoRobot",
                    "agentSessionId": "session",
                    "statusSequence": 3,
                    "state": "NODE_REACHED",
                    "orderId": "first",
                    "nodeId": "N2",
                    "edgeId": "E1",
                }
            )

            completed_robot = fleet.robot_registry.get(
                "DemoRobot",
                "youBot",
            )
            self.assertIsNone(completed_robot["currentOrder"])
            self.assertEqual(completed_robot["lastOrderStatus"], "COMPLETED")
            self.assertTrue(completed_robot["available"])
            erp.update_robot_registry(
                {"robots": fleet.robot_registry.snapshot()}
            )
            self.assertEqual(erp.orders["first"]["status"], "COMPLETED")
            fleet.handle_robot_status(
                {
                    "robotId": "youBot",
                    "manufacturer": "DemoRobot",
                    "agentSessionId": "session",
                    "statusSequence": 4,
                    "state": "NODE_REACHED",
                    "orderId": "first",
                    "nodeId": "N2",
                    "edgeId": "E1",
                }
            )
            post_completion_heartbeat = fleet.robot_registry.get(
                "DemoRobot",
                "youBot",
            )
            self.assertTrue(post_completion_heartbeat["agentOnline"])
            self.assertTrue(post_completion_heartbeat["available"])

            second_order = self.add_order("second")
            second_order["orderUpdateId"] = 0
            second_order["version"] = "3.0.0"
            self.assertEqual(self.dispatch("second").status_code, 200)
            fleet.handle_order(second_order)

            second_assignment = fleet.robot_registry.get(
                "DemoRobot",
                "youBot",
            )
            self.assertEqual(second_assignment["currentOrder"], "second")
            self.assertFalse(second_assignment["available"])

            accepted = fleet.robot_registry.update_status(
                {
                    "robotId": "youBot",
                    "manufacturer": "DemoRobot",
                    "agentSessionId": "session",
                    "statusSequence": 5,
                    "state": "NODE_REACHED",
                    "orderId": "first",
                    "nodeId": "N2",
                    "edgeId": "E1",
                }
            )
            self.assertFalse(accepted)
            self.assertEqual(
                fleet.robot_registry.get("DemoRobot", "youBot")[
                    "currentOrder"
                ],
                "second",
            )

            fleet.handle_robot_status(
                {
                    "robotId": "youBot",
                    "manufacturer": "DemoRobot",
                    "agentSessionId": "session",
                    "statusSequence": 6,
                    "state": "EXECUTING",
                    "orderId": "second",
                    "nodeId": "N2",
                    "edgeId": "E1",
                }
            )
            fleet.handle_robot_status(
                {
                    "robotId": "youBot",
                    "manufacturer": "DemoRobot",
                    "agentSessionId": "session",
                    "statusSequence": 7,
                    "state": "NODE_REACHED",
                    "orderId": "second",
                    "nodeId": "N2",
                    "edgeId": "E1",
                }
            )

        final_robot = fleet.robot_registry.get("DemoRobot", "youBot")
        self.assertIsNone(final_robot["currentOrder"])
        self.assertEqual(final_robot["lastOrderStatus"], "COMPLETED")
        self.assertTrue(final_robot["available"])

    def test_startup_does_not_restore_unverified_executing_order(self):
        erp.DATA_FILE.write_text(
            json.dumps([make_order("saved", "EXECUTING")]),
            encoding="utf-8",
        )

        erp.load_orders()

        self.assertEqual(erp.orders["saved"]["status"], "RECOVERY_REQUIRED")
        self.assertIn("restarted", erp.orders["saved"]["recoveryReason"])
        self.assertIsNone(erp.robots[("DemoRobot", "youBot")]["currentOrder"])

    def test_live_assignment_reconciles_persisted_order(self):
        erp.orders["saved"] = make_order("saved", "RECOVERY_REQUIRED")
        erp.update_robot_registry(
            {
                "robots": [
                    make_robot(
                        status="EXECUTING",
                        available=False,
                        currentOrder="saved",
                    )
                ]
            }
        )

        self.assertEqual(erp.orders["saved"]["status"], "EXECUTING")

    def test_stale_registry_assignment_does_not_reconcile_order(self):
        erp.orders["saved"] = make_order("saved", "RECOVERY_REQUIRED")
        erp.update_robot_registry(
            {
                "robots": [
                    make_robot(
                        status="EXECUTING",
                        available=False,
                        currentOrder="saved",
                        lastUpdate="2000-01-01T00:00:00Z",
                    )
                ]
            }
        )

        self.assertEqual(erp.orders["saved"]["status"], "RECOVERY_REQUIRED")

    def test_uncertain_registry_assignment_blocks_dispatch(self):
        robot = erp.robots[("DemoRobot", "youBot")]
        robot.update(
            available=False,
            recoveryRequired=True,
            recoveryReason="Unknown mission state",
            currentOrder="saved",
        )
        self.add_order("saved", "RECOVERY_REQUIRED")
        self.add_order("new")

        response = self.dispatch("new")

        self.assertEqual(response.status_code, 409)
        self.assertIn("Unknown mission state", response.json["error"])

    def test_publish_failure_releases_its_own_reservation(self):
        self.add_order("failed")
        self.mock_publish.return_value = SimpleNamespace(rc=1)

        response = self.dispatch("failed")

        self.assertEqual(response.status_code, 503)
        self.assertEqual(erp.orders["failed"]["status"], "CREATED")
        self.assertIsNone(
            erp.robots[("DemoRobot", "youBot")]["currentOrder"]
        )
        self.assertNotIn(("DemoRobot", "youBot"), erp.dispatching_robots)

    def test_delayed_terminal_state_cannot_clear_newer_assignment(self):
        registry = RobotRegistry()
        registry.register_agent(
            {
                "robotId": "youBot",
                "manufacturer": "DemoRobot",
                "agentSessionId": "session-a",
                "capabilities": ["NAVIGATION"],
            }
        )
        self.assertTrue(
            registry.update_status(
                {
                    "robotId": "youBot",
                    "manufacturer": "DemoRobot",
                    "agentSessionId": "session-a",
                    "statusSequence": 1,
                    "state": "EXECUTING",
                    "orderId": "older",
                }
            )
        )
        registry.update_status(
            {
                "robotId": "youBot",
                "manufacturer": "DemoRobot",
                "agentSessionId": "session-a",
                "statusSequence": 2,
                "state": "STOPPED",
                "orderId": "older",
            }
        )
        registry.set_state(
            "DemoRobot",
            "youBot",
            "DISPATCHED",
            current_order="newer",
        )

        accepted = registry.update_status(
            {
                "robotId": "youBot",
                "manufacturer": "DemoRobot",
                "agentSessionId": "session-a",
                "statusSequence": 3,
                "state": "STOPPED",
                "orderId": "older",
            }
        )

        self.assertFalse(accepted)
        self.assertEqual(
            registry.get("DemoRobot", "youBot")["currentOrder"],
            "newer",
        )

    def test_manual_stop_keeps_active_order_recoverable(self):
        registry = RobotRegistry()
        registry.register_agent(
            {
                "robotId": "youBot",
                "manufacturer": "DemoRobot",
                "agentSessionId": "session",
                "capabilities": ["NAVIGATION"],
            }
        )
        registry.update_status(
            {
                "robotId": "youBot",
                "manufacturer": "DemoRobot",
                "agentSessionId": "session",
                "statusSequence": 1,
                "state": "EXECUTING",
                "orderId": "active",
            }
        )

        registry.update_status(
            {
                "robotId": "youBot",
                "manufacturer": "DemoRobot",
                "agentSessionId": "session",
                "statusSequence": 2,
                "state": "STOPPED",
                "orderId": "active",
            }
        )

        record = registry.get("DemoRobot", "youBot")
        self.assertEqual(record["currentOrder"], "active")
        self.assertTrue(record["recoveryRequired"])
        self.assertEqual(record["lastOrderStatus"], "STOPPED")
        self.assertFalse(record["available"])

    def test_stop_endpoint_marks_robot_for_recovery(self):
        order = self.add_order("active", "EXECUTING")
        robot = erp.robots[("DemoRobot", "youBot")]
        robot.update(
            status="EXECUTING",
            currentOrder="active",
            available=False,
            recoveryRequired=False,
            recoveryReason=None,
        )

        response = self.client.post("/api/robots/DemoRobot/youBot/stop")

        self.assertEqual(response.status_code, 200)
        self.assertEqual(robot["currentOrder"], "active")
        self.assertTrue(robot["recoveryRequired"])
        self.assertIn("recovery", robot["recoveryReason"].lower())
        self.assertFalse(robot["available"])
        self.assertEqual(order["status"], "STOPPING")

    def test_robot_navigation_error_is_preserved_for_recovery_ui(self):
        original_registry = fleet.robot_registry
        original_missions = fleet.missions
        fleet.robot_registry = RobotRegistry()
        fleet.missions = {
            ("DemoRobot", "youBot"): fleet.Mission(
                make_order("active", "EXECUTING"),
                state="EXECUTING",
            )
        }
        self.addCleanup(setattr, fleet, "robot_registry", original_registry)
        self.addCleanup(setattr, fleet, "missions", original_missions)

        fleet.robot_registry.register_agent(
            {
                "robotId": "youBot",
                "manufacturer": "DemoRobot",
                "agentSessionId": "session",
            }
        )
        navigation_error = "No collision-free route to N2"
        with (
            patch.object(fleet, "publish_robot_registry"),
            patch.object(fleet, "publish_state"),
        ):
            fleet.handle_robot_status(
                {
                    "robotId": "youBot",
                    "manufacturer": "DemoRobot",
                    "agentSessionId": "session",
                    "statusSequence": 1,
                    "state": "ERROR",
                    "orderId": "active",
                    "error": navigation_error,
                }
            )

        robot = fleet.robot_registry.get("DemoRobot", "youBot")
        self.assertEqual(robot["lastError"], navigation_error)
        self.assertEqual(robot["recoveryReason"], navigation_error)
        self.assertTrue(robot["recoveryRequired"])
        self.assertFalse(robot["available"])

    def test_completed_mission_heartbeat_keeps_robot_online_and_available(self):
        registry = RobotRegistry(heartbeat_timeout_seconds=8.0)
        clock = [1.0]
        with patch("robot_registry.time.monotonic", side_effect=lambda: clock[0]):
            registry.register_agent(
                {
                    "robotId": "youBot",
                    "manufacturer": "DemoRobot",
                    "agentSessionId": "session",
                    "capabilities": ["NAVIGATION"],
                }
            )
            self.assertTrue(
                registry.update_status(
                    {
                        "robotId": "youBot",
                        "manufacturer": "DemoRobot",
                        "agentSessionId": "session",
                        "statusSequence": 1,
                        "state": "EXECUTING",
                        "orderId": "completed-order",
                    }
                )
            )
            registry.set_state(
                "DemoRobot",
                "youBot",
                "IDLE",
                current_node="N2",
            )

            clock[0] = 20.0
            accepted = registry.update_status(
                {
                    "robotId": "youBot",
                    "manufacturer": "DemoRobot",
                    "agentSessionId": "session",
                    "statusSequence": 2,
                    "state": "NODE_REACHED",
                    "orderId": "completed-order",
                    "nodeId": "N2",
                    "position": {"x": 3.0, "y": 0.0, "z": 0.0},
                }
            )
            record = registry.get("DemoRobot", "youBot")

        self.assertTrue(accepted)
        self.assertEqual(record["status"], "IDLE")
        self.assertEqual(record["lastOrderStatus"], "COMPLETED")
        self.assertIsNone(record["currentOrder"])
        self.assertTrue(record["agentOnline"])
        self.assertTrue(record["available"])
        self.assertEqual(record["currentNode"], "N2")

    def test_controller_restart_marks_existing_assignment_for_recovery(self):
        registry = RobotRegistry()
        registry.register_agent(
            {
                "robotId": "youBot",
                "manufacturer": "DemoRobot",
                "agentSessionId": "session-old",
                "capabilities": ["NAVIGATION"],
            }
        )
        registry.update_status(
            {
                "robotId": "youBot",
                "manufacturer": "DemoRobot",
                "agentSessionId": "session-old",
                "statusSequence": 1,
                "state": "EXECUTING",
                "orderId": "active",
            }
        )

        registry.register_agent(
            {
                "robotId": "youBot",
                "manufacturer": "DemoRobot",
                "agentSessionId": "session-new",
                "capabilities": ["NAVIGATION"],
            }
        )

        record = registry.get("DemoRobot", "youBot")
        self.assertEqual(record["currentOrder"], "active")
        self.assertTrue(record["recoveryRequired"])
        self.assertFalse(record["available"])
        self.assertFalse(
            registry.update_status(
                {
                    "robotId": "youBot",
                    "manufacturer": "DemoRobot",
                    "agentSessionId": "session-old",
                    "statusSequence": 2,
                    "state": "IDLE",
                }
            )
        )

    def test_matching_recovery_ack_releases_only_that_assignment(self):
        registry = RobotRegistry()
        registry.register_agent(
            {
                "robotId": "youBot",
                "manufacturer": "DemoRobot",
                "agentSessionId": "session",
                "capabilities": ["NAVIGATION"],
            }
        )
        registry.update_status(
            {
                "robotId": "youBot",
                "manufacturer": "DemoRobot",
                "agentSessionId": "session",
                "statusSequence": 1,
                "state": "EXECUTING",
                "orderId": "assigned",
            }
        )
        registry.mark_recovery_required(
            "DemoRobot", "youBot", "Unknown Fleet mission"
        )

        accepted = registry.update_status(
            {
                "robotId": "youBot",
                "manufacturer": "DemoRobot",
                "agentSessionId": "session",
                "statusSequence": 2,
                "state": "IDLE",
                "orderId": None,
                "recoveredOrderId": "assigned",
            }
        )

        record = registry.get("DemoRobot", "youBot")
        self.assertTrue(accepted)
        self.assertIsNone(record["currentOrder"])
        self.assertEqual(record["lastOrderId"], "assigned")
        self.assertEqual(record["lastOrderStatus"], "INTERRUPTED")
        self.assertFalse(record["recoveryRequired"])
        self.assertTrue(record["available"])

    def test_wrong_recovery_ack_does_not_release_assignment(self):
        registry = RobotRegistry()
        registry.register_agent(
            {
                "robotId": "youBot",
                "manufacturer": "DemoRobot",
                "agentSessionId": "session",
                "capabilities": ["NAVIGATION"],
            }
        )
        registry.update_status(
            {
                "robotId": "youBot",
                "manufacturer": "DemoRobot",
                "agentSessionId": "session",
                "statusSequence": 1,
                "state": "EXECUTING",
                "orderId": "assigned",
            }
        )

        accepted = registry.update_status(
            {
                "robotId": "youBot",
                "manufacturer": "DemoRobot",
                "agentSessionId": "session",
                "statusSequence": 2,
                "state": "IDLE",
                "orderId": None,
                "recoveredOrderId": "different",
            }
        )

        self.assertFalse(accepted)
        self.assertEqual(
            registry.get("DemoRobot", "youBot")["currentOrder"],
            "assigned",
        )

    def test_concurrent_dispatch_reserves_robot_once(self):
        self.add_order("one")
        self.add_order("two")
        publish_entered = threading.Event()
        release_publish = threading.Event()

        def blocked_publish(*_args, **_kwargs):
            publish_entered.set()
            if not release_publish.wait(timeout=3):
                raise TimeoutError("test did not release MQTT publish")
            return SimpleNamespace(rc=erp.mqtt.MQTT_ERR_SUCCESS)

        self.mock_publish.side_effect = blocked_publish

        with ThreadPoolExecutor(max_workers=2) as pool:
            first = pool.submit(self.dispatch, "one")
            self.assertTrue(publish_entered.wait(timeout=3))
            second = self.dispatch("two")
            release_publish.set()
            first_response = first.result(timeout=3)

        self.assertEqual(first_response.status_code, 200)
        self.assertEqual(second.status_code, 409)
        self.assertEqual(
            erp.robots[("DemoRobot", "youBot")]["currentOrder"],
            "one",
        )
        self.assertEqual(self.mock_publish.call_count, 1)

    def test_manual_recovery_waits_for_ack_and_preserves_history(self):
        order = self.add_order("stale", "RECOVERY_REQUIRED")
        robot = erp.robots[("DemoRobot", "youBot")]
        robot.update(
            status="IDLE",
            available=False,
            recoveryRequired=True,
            recoveryReason="Controller restarted during an active assignment",
            currentOrder="stale",
        )

        response = self.client.post(
            "/api/robots/DemoRobot/youBot/recover",
            json={"confirmSafe": True, "reason": "Area inspected and clear"},
        )

        self.assertEqual(response.status_code, 202)
        self.assertEqual(order["status"], "RECOVERY_REQUIRED")
        self.assertEqual(erp.recovery_requests[("DemoRobot", "youBot")], "stale")
        self.assertEqual(order["recoveryHistory"][0]["result"], "REQUESTED")
        self.assertFalse(robot["available"])

        erp.update_robot_registry(
            {
                "robots": [
                    make_robot(
                        lastOrderId="stale",
                        lastOrderStatus="INTERRUPTED",
                    )
                ]
            }
        )

        self.assertEqual(order["status"], "INTERRUPTED")
        self.assertIsNone(erp.recovery_requests.get(("DemoRobot", "youBot")))
        self.assertEqual(len(order["recoveryHistory"]), 2)

    def test_recovery_is_idempotent_and_requires_live_robot(self):
        order = self.add_order("stale", "RECOVERY_REQUIRED")
        robot = erp.robots[("DemoRobot", "youBot")]
        robot.update(
            available=False,
            recoveryRequired=True,
            currentOrder="stale",
        )
        request_payload = {
            "confirmSafe": True,
            "reason": "Operator verified safe",
        }

        self.assertEqual(
            self.client.post(
                "/api/robots/DemoRobot/youBot/recover",
                json=request_payload,
            ).status_code,
            202,
        )
        self.assertEqual(
            self.client.post(
                "/api/robots/DemoRobot/youBot/recover",
                json=request_payload,
            ).status_code,
            202,
        )
        self.assertEqual(len(order["recoveryHistory"]), 1)
        self.assertEqual(self.mock_publish.call_count, 2)

        robot["agentOnline"] = False
        response = self.client.post(
            "/api/robots/DemoRobot/youBot/recover",
            json=request_payload,
        )
        self.assertEqual(response.status_code, 409)
        self.assertEqual(order["status"], "RECOVERY_REQUIRED")

    def test_stale_recovery_ack_does_not_release_assignment(self):
        order = self.add_order("stale", "RECOVERY_REQUIRED")
        robot = erp.robots[("DemoRobot", "youBot")]
        robot.update(
            available=False,
            recoveryRequired=True,
            currentOrder="stale",
        )
        response = self.client.post(
            "/api/robots/DemoRobot/youBot/recover",
            json={"confirmSafe": True, "reason": "Area inspected"},
        )
        self.assertEqual(response.status_code, 202)

        stale_report = make_robot(
            lastOrderId="stale",
            lastOrderStatus="INTERRUPTED",
            lastUpdate="2000-01-01T00:00:00Z",
        )
        erp.update_robot_registry({"robots": [stale_report]})

        self.assertEqual(order["status"], "RECOVERY_REQUIRED")
        self.assertEqual(
            erp.recovery_requests[("DemoRobot", "youBot")],
            "stale",
        )

    def test_dashboard_does_not_show_stale_robot_as_available(self):
        erp.robots[("DemoRobot", "youBot")]["lastUpdate"] = (
            "2000-01-01T00:00:00Z"
        )

        response = self.client.get("/api/dashboard")

        robot = response.json["robots"][0]
        self.assertFalse(robot["available"])
        self.assertEqual(robot["availabilityReason"], "Robot status is stale")

    def test_robot_state_transition_contract_remains_unchanged(self):
        controller_dir = str(
            PROJECT_ROOT / "webots/controllers/robot_controller"
        )
        sys.path.insert(0, controller_dir)
        from robot_state import RobotState

        state = RobotState()
        state.start_navigation("order", "N2", "E1")
        self.assertEqual(state.state, "EXECUTING")
        state.stop()
        self.assertEqual(state.state, "STOPPED")
        self.assertEqual(state.order_id, "order")
        state.clear_order()
        self.assertEqual(state.state, "IDLE")
        self.assertIsNone(state.order_id)

    def test_navigation_waypoint_and_stop_contract_remains_unchanged(self):
        controller_dir = str(
            PROJECT_ROOT / "webots/controllers/robot_controller"
        )
        sys.path.insert(0, controller_dir)
        from navigation import Navigation

        class Motor:
            def __init__(self):
                self.velocity = None

            def setVelocity(self, velocity):
                self.velocity = velocity

        class Robot:
            def getSelf(self):
                return self

            def getPosition(self):
                return [0.0, 0.0, 0.0]

            def getOrientation(self):
                return [1, 0, 0, 0, 1, 0, 0, 0, 1]

        class Nodes:
            def get_node(self, node_id):
                return {"x": 2.0, "y": 0.0, "z": 0.0}

            def plan_navigation_path(self, start, node_id):
                target = self.get_node(node_id)
                goal = {"x": target["x"], "y": target["y"]}
                return {
                    "path": [(start["x"], start["y"]), (goal["x"], goal["y"])],
                    "goal": goal,
                    "approach": False,
                }

        left = Motor()
        right = Motor()
        navigation = Navigation(
            Robot(),
            left,
            right,
            None,
            None,
            Nodes(),
        )

        result = navigation.move_to_node("N2")
        self.assertFalse(result["reached"])
        self.assertGreater(left.velocity, 0)
        self.assertEqual(left.velocity, right.velocity)
        navigation.stop()
        self.assertEqual(left.velocity, 0.0)
        self.assertEqual(right.velocity, 0.0)

    def test_robot_recovery_command_stops_before_acknowledging(self):
        controller_dir = str(
            PROJECT_ROOT / "webots/controllers/robot_controller"
        )
        sys.path.insert(0, controller_dir)
        saved_config = sys.modules.pop("config", None)
        try:
            controller_module = importlib.import_module("robot_controller")
        finally:
            if saved_config is not None:
                sys.modules["config"] = saved_config

        class Adapter:
            stopped = False

            def stop(self):
                self.stopped = True

        controller = controller_module.RobotController.__new__(
            controller_module.RobotController
        )
        controller.robot_id = "youBot"
        controller.state = __import__("robot_state").RobotState()
        controller.state.start_navigation("assigned", "N2", "E1")
        controller.adapter = Adapter()
        controller.last_error = None
        acknowledgements = []
        controller.publish_status = lambda recovered_order_id=None: (
            acknowledgements.append(recovered_order_id)
        )

        controller.handle_recovery_command(
            {
                "robotId": "youBot",
                "orderId": "assigned",
            }
        )

        self.assertTrue(controller.adapter.stopped)
        self.assertEqual(controller.state.state, "IDLE")
        self.assertEqual(acknowledgements, ["assigned"])


if __name__ == "__main__":
    unittest.main()
