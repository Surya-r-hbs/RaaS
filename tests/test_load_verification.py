import json
import sys
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

PROJECT_ROOT = Path(__file__).resolve().parents[1]
FLEET_DIR = PROJECT_ROOT / "fleet_control"
for path in (str(FLEET_DIR),):
    if path not in sys.path:
        sys.path.insert(0, path)

from config import MAP_NODE_POSITIONS
from load_verification import annotate_load_verification
from object_registry import ObjectRegistry
from transport_tasks import TransportTaskManager
import fleet_controller as fleet


def timestamp(delta=timedelta(0)):
    return (
        (datetime.now(timezone.utc) + delta)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


def carrier(robot_id="youBot", position=None, orientation=None):
    return {
        "robotId": robot_id,
        "manufacturer": "DemoRobot",
        "agentOnline": True,
        "status": "IDLE",
        "available": True,
        "capabilities": ["NAVIGATION"],
        "lastUpdate": timestamp(),
        "position": position or {"x": 0.0, "y": 0.0, "z": 0.1},
        "orientation": orientation or [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ],
        "transportCapability": {
            "status": "UNVERIFIED",
            "reason": "Payload and stability limits are unknown.",
            "supportSurface": {
                "center": {"x": -0.147, "y": 0.0, "z": 0.0485},
                "dimensions": {"x": 0.25, "y": 0.17, "z": 0.007},
            },
        },
    }


def object_record(
    object_id="small-box",
    position=None,
    rotation=None,
    size=(0.1, 0.1, 0.1),
):
    center_x, center_y, center_z = position or (-0.147, 0.0, 0.1985)
    half_x, half_y, half_z = (value / 2.0 for value in size)
    return {
        "object_id": object_id,
        "name": "small box",
        "object_type": "box",
        "position": {"x": center_x, "y": center_y, "z": center_z},
        "rotation": rotation or [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ],
        "geometry": {
            "bounds": {
                "min": {"x": -half_x, "y": -half_y, "z": -half_z},
                "max": {"x": half_x, "y": half_y, "z": half_z},
            },
        },
        "mass_kg": 0.5,
    }


class LoadVerificationTests(unittest.TestCase):
    def snapshot(self, record):
        return {"timestamp": timestamp(), "objects": [record]}

    def test_fresh_unique_geometric_support_generates_load_evidence(self):
        snapshot = self.snapshot(object_record())

        result = annotate_load_verification(snapshot, [carrier()])

        evidence = result["objects"][0]["load_verification"]
        self.assertEqual(evidence["load_state"], "LOADED")
        self.assertEqual(evidence["carrier_robot_id"], "youBot")
        self.assertEqual(evidence["method"], "SUPERVISOR_GEOMETRY")

    def test_robot_orientation_is_applied_to_support_geometry(self):
        rotation = [
            0.0, -1.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        record = object_record(
            position=(0.0, -0.147, 0.1985),
            rotation=rotation,
        )

        result = annotate_load_verification(
            self.snapshot(record),
            [carrier(orientation=rotation)],
        )

        self.assertIn("load_verification", result["objects"][0])

    def test_object_outside_support_patch_does_not_generate_evidence(self):
        snapshot = self.snapshot(object_record(size=(0.6, 0.6, 0.6)))

        result = annotate_load_verification(snapshot, [carrier()])

        self.assertNotIn("load_verification", result["objects"][0])

    def test_moving_carrier_does_not_generate_load_evidence(self):
        moving = carrier()
        moving["status"] = "EXECUTING"

        result = annotate_load_verification(
            self.snapshot(object_record()),
            [moving],
        )

        self.assertNotIn("load_verification", result["objects"][0])

    def test_ambiguous_or_stale_carrier_pose_does_not_generate_evidence(self):
        snapshot = self.snapshot(object_record())
        two_carriers = [carrier("youBot"), carrier("youBot-copy")]

        ambiguous = annotate_load_verification(snapshot, two_carriers)
        self.assertNotIn("load_verification", ambiguous["objects"][0])

        stale = carrier()
        stale["lastUpdate"] = timestamp(timedelta(seconds=-6))
        stale_result = annotate_load_verification(
            self.snapshot(object_record()),
            [stale],
        )
        self.assertNotIn("load_verification", stale_result["objects"][0])

    def test_stale_snapshot_removes_untrusted_embedded_evidence(self):
        record = object_record()
        record["load_verification"] = {"verified": True}
        snapshot = {
            "timestamp": timestamp(timedelta(seconds=-6)),
            "objects": [record],
        }

        annotate_load_verification(snapshot, [carrier()])

        self.assertNotIn("load_verification", record)

    def test_unknown_payload_and_stability_still_block_transport(self):
        robot = carrier()
        record = object_record()
        snapshot = annotate_load_verification(
            self.snapshot(record),
            [robot],
        )
        manager = TransportTaskManager(MAP_NODE_POSITIONS)
        manager.observe_inventory(snapshot, [robot])

        task = manager.create_task(
            snapshot["objects"][0],
            "N5",
            [robot],
        )

        self.assertEqual(task["status"], "FAILED")
        self.assertIn("Payload and stability limits are unknown", task["message"])


class FleetLoadBridgeTests(unittest.TestCase):
    def setUp(self):
        self.previous_registry = fleet.object_registry
        fleet.object_registry = ObjectRegistry()

    def tearDown(self):
        fleet.object_registry = self.previous_registry

    def test_raw_inventory_is_verified_using_live_carrier_pose(self):
        payload = {
            "timestamp": timestamp(),
            "objects": [object_record()],
        }
        message = SimpleNamespace(
            topic=fleet.OBJECTS_RAW_TOPIC,
            payload=json.dumps(payload).encode("utf-8"),
            retain=False,
        )

        with patch.object(fleet.robot_registry, "snapshot", return_value=[carrier()]), patch.object(
            fleet,
            "publish_object_registry",
        ):
            fleet.on_message(None, None, message)

        verified = fleet.object_registry.get("small-box")["load_verification"]
        self.assertEqual(verified["carrier_robot_id"], "youBot")
        self.assertEqual(verified["load_state"], "LOADED")


if __name__ == "__main__":
    unittest.main()
