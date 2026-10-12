import importlib
import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

PROJECT_ROOT = Path(__file__).resolve().parents[1]
FLEET_DIR = PROJECT_ROOT / "fleet_control"
ERP_DIR = PROJECT_ROOT / "erp_frontend"
SUPERVISOR_DIR = PROJECT_ROOT / "webots/controllers/object_supervisor"
ROBOT_CONTROLLER_DIR = PROJECT_ROOT / "webots/controllers/robot_controller"
for path in (str(FLEET_DIR), str(ERP_DIR), str(SUPERVISOR_DIR), str(ROBOT_CONTROLLER_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from object_discovery import discover_objects
from object_registry import ObjectRegistry
from robot_discovery import discover_world_robots

erp = importlib.import_module("app")
fleet = importlib.import_module("fleet_controller")


class Field:
    def __init__(self, value):
        self.value = value

    def getSFString(self):
        return self.value

    def getSFVec3f(self):
        return self.value

    def getSFFloat(self):
        return self.value


class SceneObject:
    def __init__(
        self,
        object_id,
        proto_type,
        name,
        position,
        size=None,
        rotation=None,
        definition="",
        mass=None,
    ):
        self.object_id = object_id
        self.proto_type = proto_type
        self.name = name
        self.position = position
        self.size = size
        self.rotation = rotation or [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        self.definition = definition
        self.mass = mass

    def getProtoTypeName(self):
        return self.proto_type

    def getTypeName(self):
        return "Solid"

    def getId(self):
        return self.object_id

    def getDef(self):
        return self.definition

    def getPosition(self):
        return self.position

    def getOrientation(self):
        return self.rotation

    def getProtoField(self, name):
        if name == "name":
            return Field(self.name)
        if name == "size" and self.size is not None:
            return Field(self.size)
        if name == "mass" and self.mass is not None:
            return Field(self.mass)
        return None

    def getField(self, name):
        return None

    def getBaseTypeName(self):
        return "Solid"


class Children:
    def __init__(self, nodes):
        self.nodes = nodes

    def getCount(self):
        return len(self.nodes)

    def getMFNode(self, index):
        return self.nodes[index]


class Root:
    def __init__(self, nodes):
        self.children = Children(nodes)

    def getField(self, name):
        return self.children if name == "children" else None


class Supervisor:
    def __init__(self, nodes):
        self.root = Root(nodes)

    def getRoot(self):
        return self.root


def world_objects():
    return [
        SceneObject(
            1, "Table", "table",
            [4.01, -4.01, 0.0],
            [1.0, 1.8, 0.74],
        ),
        SceneObject(
            2, "PlasticCrate", "plastic crate",
            [4.1400008, -4.6099994, 0.7356838],
            [0.6, 0.6, 0.6],
        ),
        SceneObject(
            3, "WaterBottle", "water bottle",
            [3.9399887, -4.7322309, 0.7543518],
        ),
        SceneObject(
            4, "WoodenBox", "wooden box",
            [4.19, -3.71, 1.036076],
            [0.6, 0.6, 0.6],
            mass=10.0,
        ),
    ]


def fresh_snapshot(objects):
    return {
        "timestamp": erp.now_utc(),
        "objects": objects,
    }


class ObjectDiscoveryTests(unittest.TestCase):
    def test_world_objects_are_discovered_without_duplicate_entries(self):
        supervisor = Supervisor(world_objects())

        first = discover_objects(supervisor)
        second = discover_objects(supervisor)

        self.assertEqual(len(first["objects"]), 4)
        self.assertEqual(
            [item["object_id"] for item in first["objects"]],
            [item["object_id"] for item in second["objects"]],
        )
        self.assertEqual(
            {item["name"] for item in first["objects"]},
            {"table", "plastic crate", "water bottle", "wooden box"},
        )

    def test_position_changes_are_reflected_by_each_discovery_scan(self):
        objects = world_objects()
        supervisor = Supervisor(objects)

        first = discover_objects(supervisor)
        objects[2].position[0] = 1.25
        second = discover_objects(supervisor)

        bottle_id = first["objects"][2]["object_id"]
        moved = next(item for item in second["objects"] if item["object_id"] == bottle_id)
        self.assertEqual(moved["position"]["x"], 1.25)

    def test_def_is_used_as_stable_id_when_available(self):
        supervisor = Supervisor(
            [SceneObject(99, "WoodenBox", "box", [0, 0, 0], [1, 1, 1], definition="BoxA")]
        )

        record = discover_objects(supervisor)["objects"][0]

        self.assertEqual(record["object_id"], "def:BoxA")

    def test_mass_from_world_proto_is_preserved(self):
        record = discover_objects(Supervisor(world_objects()))["objects"][-1]

        self.assertEqual(record["mass_kg"], 10.0)

    def test_beer_bottle_and_unlisted_solid_are_discovered_generically(self):
        nodes = [
            SceneObject(10, "BeerBottle", "beer bottle", [0, 0, 0]),
            SceneObject(11, "CustomPart", "custom part", [1, 0, 0]),
        ]

        records = discover_objects(Supervisor(nodes))["objects"]

        self.assertEqual(len(records), 2)
        self.assertEqual(records[0]["object_type"], "beer_bottle")
        self.assertEqual(records[0]["semantic_category"], "unknown")
        self.assertIsNone(records[0]["geometry"])
        self.assertEqual(records[1]["object_type"], "custom_part")

    def test_duplicate_display_names_keep_distinct_node_identities(self):
        nodes = [
            SceneObject(20, "Solid", "same name", [0, 0, 0]),
            SceneObject(21, "Solid", "same name", [1, 0, 0]),
        ]

        records = discover_objects(Supervisor(nodes))["objects"]

        self.assertEqual(
            [record["object_id"] for record in records],
            ["webots-node:20", "webots-node:21"],
        )

    def test_navigation_and_infrastructure_nodes_are_excluded_from_cargo(self):
        nodes = [
            SceneObject(30, "Solid", "N5", [0, 0, 0], definition="N5"),
            SceneObject(31, "Floor", "floor", [0, 0, 0]),
            SceneObject(32, "BeerBottle", "beer bottle", [0, 0, 0]),
        ]

        records = discover_objects(Supervisor(nodes))["objects"]

        self.assertEqual([record["object_id"] for record in records], ["webots-node:32"])

    def test_nested_scene_solids_are_discovered(self):
        child = SceneObject(42, "BeerBottle", "nested bottle", [1, 2, 3])
        parent = SceneObject(41, "Solid", "holder", [0, 0, 0])

        class ChildField:
            def getCount(self):
                return 1

            def getMFNode(self, index):
                return child if index == 0 else None

        parent.getField = lambda name: ChildField() if name == "children" else None

        records = discover_objects(Supervisor([parent]))["objects"]

        by_id = {record["object_id"]: record for record in records}
        self.assertEqual(set(by_id), {"webots-node:41", "webots-node:42"})
        self.assertEqual(by_id["webots-node:42"]["parent_id"], "webots-node:41")

    def test_exposed_indexed_mesh_geometry_is_extracted_without_proto_name_rules(self):
        class ValueField:
            def __init__(self, value):
                self.value = value

            def getSFNode(self):
                return self.value

            def getCount(self):
                return len(self.value)

            def getMFNode(self, index):
                return self.value[index]

            def getMFVec3f(self, index):
                return self.value[index]

            def getSFVec3f(self):
                return self.value

            def getSFRotation(self):
                return self.value

        class GenericNode:
            def __init__(self, node_id, type_name, fields=None):
                self.node_id = node_id
                self.type_name = type_name
                self.fields = fields or {}

            def getId(self):
                return self.node_id

            def getTypeName(self):
                return self.type_name

            def getBaseTypeName(self):
                return "Solid"

            def getProtoTypeName(self):
                return "CustomMesh"

            def getDef(self):
                return ""

            def getPosition(self):
                return [0.0, 0.0, 0.0]

            def getOrientation(self):
                return [
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0,
                ]

            def getProtoField(self, name):
                return None

            def getField(self, name):
                return self.fields.get(name)

        coordinates = GenericNode(
            104,
            "Coordinate",
            {"point": ValueField([
                (-0.1, -0.05, 0.0),
                (0.1, 0.05, 0.0),
                (-0.1, -0.05, 0.3),
                (0.1, 0.05, 0.3),
            ])},
        )
        mesh = GenericNode(103, "IndexedFaceSet", {"coord": ValueField(coordinates)})
        shape = GenericNode(
            102,
            "Shape",
            {
                "geometry": ValueField(mesh),
                "translation": ValueField((1.0, 0.0, 0.0)),
                "rotation": ValueField((0.0, 0.0, 1.0, 0.0)),
            },
        )
        solid = GenericNode(101, "Solid", {"children": ValueField([shape])})

        record = discover_objects(Supervisor([solid]))["objects"][0]

        self.assertEqual(record["geometry_status"], "VERIFIED")
        self.assertEqual(record["geometry"]["bounds"]["min"]["z"], 0.0)
        self.assertEqual(record["geometry"]["bounds"]["max"]["z"], 0.3)
        self.assertAlmostEqual(record["geometry"]["bounds"]["min"]["x"], 0.9)

    def test_object_supervisor_is_excluded_from_robot_fleet_discovery(self):
        class RobotNode:
            def __init__(self, node_id, name, controller, type_name):
                self.node_id = node_id
                self.name = name
                self.controller = controller
                self.type_name = type_name

            def getTypeName(self):
                return self.type_name

            def getBaseTypeName(self):
                return "Robot"

            def getField(self, name):
                if name == "name":
                    return Field(self.name)
                if name == "controller":
                    return Field(self.controller)
                return None

            def getDef(self):
                return ""

            def getId(self):
                return self.node_id

            def getPosition(self):
                return [0.0, 0.0, 0.0]

        supervisor_robot = RobotNode(
            1,
            "object-inventory-supervisor",
            "object_supervisor",
            "Robot",
        )
        warehouse_robot = RobotNode(
            2,
            "Mir100",
            "robot_controller",
            "Mir100",
        )

        class RobotSupervisor(Supervisor):
            def getSelf(self):
                return supervisor_robot

        world = RobotSupervisor([supervisor_robot, warehouse_robot])
        discovered = discover_world_robots(world, "DemoRobot")

        self.assertEqual([record["robotId"] for record, _ in discovered], ["Mir100"])


class ObjectRelationshipTests(unittest.TestCase):
    def setUp(self):
        self.registry = ObjectRegistry()
        discovered = discover_objects(Supervisor(world_objects()))
        self.payload = fresh_snapshot(discovered["objects"])
        self.registry.update(self.payload)

    def test_actual_scene_geometry_generates_expected_relationships(self):
        snapshot = self.registry.snapshot()
        by_id = {obj["object_id"]: obj for obj in snapshot["objects"]}
        relations = {
            (
                obj["name"],
                relation["type"],
                by_id[relation["target_object_id"]]["name"],
            )
            for obj in snapshot["objects"]
            for relation in obj["relationships"]
        }

        self.assertIn(("plastic crate", "ON_TOP_OF", "table"), relations)
        self.assertIn(("wooden box", "ON_TOP_OF", "table"), relations)
        self.assertIn(("water bottle", "INSIDE", "plastic crate"), relations)
        self.assertNotIn(("water bottle", "ON_TOP_OF", "table"), relations)

    def test_moving_an_object_removes_old_relationships(self):
        records = json.loads(json.dumps(self.payload["objects"]))
        records[2]["position"]["x"] = 8.0

        self.registry.update(fresh_snapshot(records))

        bottle = self.registry.get("webots-node:3")
        self.assertEqual(bottle["relationships"], [])

    def test_moving_crate_off_table_removes_support_relationship(self):
        records = json.loads(json.dumps(self.payload["objects"]))
        records[1]["position"]["x"] = 8.0

        self.registry.update(fresh_snapshot(records))

        crate = self.registry.get("webots-node:2")
        self.assertNotIn(
            "ON_TOP_OF",
            [relation["type"] for relation in crate["relationships"]],
        )

    def test_missing_geometry_does_not_fabricate_relationships(self):
        self.registry.update(
            fresh_snapshot(
                [
                    {
                        "object_id": "a",
                        "name": "object-a",
                        "object_type": "box",
                        "position": {"x": 0, "y": 0, "z": 0},
                    },
                    {
                        "object_id": "b",
                        "name": "object-b",
                        "object_type": "table",
                        "position": {"x": 0, "y": 0, "z": 0},
                    },
                ]
            )
        )

        self.assertEqual(self.registry.get("a")["relationships"], [])
        self.assertIsNone(self.registry.get("a")["dimensions"])

    def test_repeated_updates_replace_instead_of_duplicate_inventory(self):
        self.registry.update(self.payload)
        self.registry.update(self.payload)

        snapshot = self.registry.snapshot()
        self.assertEqual(len(snapshot["objects"]), 4)
        self.assertEqual(len({item["object_id"] for item in snapshot["objects"]}), 4)

    def test_verified_load_evidence_adds_dynamic_carrier_relationship(self):
        records = json.loads(json.dumps(self.payload["objects"]))
        box = next(item for item in records if item["name"] == "wooden box")
        box["load_verification"] = {
            "verified": True,
            "method": "SUPERVISOR_GEOMETRY",
            "load_state": "LOADED",
            "object_id": box["object_id"],
            "carrier_robot_id": "Youbot",
            "carrier_manufacturer": "DemoRobot",
            "observed_at": erp.now_utc(),
        }
        snapshot = self.registry.update(fresh_snapshot(records))

        relation = next(
            item
            for item in snapshot["relationships"]
            if item["object_id"] == box["object_id"]
            and item["target_object_id"] == "Youbot"
        )
        self.assertEqual(relation["type"], "ON_TOP_OF")
        self.assertEqual(relation["certainty"], "SUPERVISOR_GEOMETRY")

    def test_invalid_snapshot_timestamp_is_rejected(self):
        payload = dict(self.payload, timestamp="not-a-timestamp")

        with self.assertRaisesRegex(ValueError, "timestamp is invalid"):
            self.registry.update(payload)

    def test_duplicate_object_ids_are_rejected(self):
        records = [
            self.payload["objects"][0],
            dict(self.payload["objects"][0], name="duplicate"),
        ]

        with self.assertRaisesRegex(ValueError, "Duplicate object ID"):
            self.registry.update(fresh_snapshot(records))

    def test_source_is_preserved_for_future_inventory_producers(self):
        record = dict(self.payload["objects"][0], source="camera")
        self.registry.update(fresh_snapshot([record]))

        self.assertEqual(
            self.registry.get(record["object_id"])["source"],
            "camera",
        )

    def test_malformed_rotation_is_rejected(self):
        record = dict(self.payload["objects"][0], rotation=[1.0, 0.0])

        with self.assertRaisesRegex(ValueError, "record at index 0 is invalid"):
            self.registry.update(fresh_snapshot([record]))


class ObjectInventoryApiTests(unittest.TestCase):
    def setUp(self):
        self.client = erp.app.test_client()
        self.previous_snapshot = erp.object_inventory_snapshot
        self.previous_received_at = erp.object_inventory_received_at
        erp.object_inventory_snapshot = None
        erp.object_inventory_received_at = None

    def tearDown(self):
        erp.object_inventory_snapshot = self.previous_snapshot
        erp.object_inventory_received_at = self.previous_received_at

    def publish_fixture(self):
        registry = ObjectRegistry()
        discovered = discover_objects(Supervisor(world_objects()))
        erp.update_object_inventory(
            registry.update(fresh_snapshot(discovered["objects"]))
        )

    def test_api_reports_disconnected_simulation_without_fabricating_data(self):
        response = self.client.get("/api/objects")

        self.assertEqual(response.status_code, 503)
        self.assertFalse(response.json["simulationConnected"])
        self.assertEqual(response.json["objects"], [])

    def test_list_detail_and_relationship_api_return_live_registry_data(self):
        self.publish_fixture()

        listing = self.client.get("/api/objects")
        object_id = next(
            item["object_id"]
            for item in listing.json["objects"]
            if item["name"] == "water bottle"
        )
        detail = self.client.get(f"/api/objects/{object_id}")
        relations = self.client.get("/api/objects/relationships")

        self.assertEqual(listing.status_code, 200)
        self.assertEqual(listing.json["objectCount"], 4)
        self.assertTrue(listing.json["simulationConnected"])
        self.assertEqual(detail.status_code, 200)
        self.assertEqual(detail.json["name"], "water bottle")
        self.assertIn(
            "INSIDE",
            [relation["type"] for relation in detail.json["relationships"]],
        )
        self.assertGreaterEqual(len(relations.json["relationships"]), 3)

    def test_api_returns_dynamically_discovered_unknown_geometry_object(self):
        discovered = discover_objects(
            Supervisor([
                SceneObject(77, "BeerBottle", "beer bottle", [0, 0, 0]),
            ])
        )
        registry = ObjectRegistry()
        erp.update_object_inventory(
            registry.update(fresh_snapshot(discovered["objects"]))
        )

        response = self.client.get("/api/objects")

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json["objects"][0]["object_type"], "beer_bottle")
        self.assertEqual(
            response.json["objects"][0]["geometry_status"],
            "UNKNOWN",
        )

    def test_object_api_rejects_invalid_identifier(self):
        response = self.client.get("/api/objects/%21")

        self.assertEqual(response.status_code, 400)

    def test_refresh_api_is_nonblocking_and_reports_broker_errors(self):
        with patch.object(erp.mqtt_client, "is_connected", return_value=True), patch.object(
            erp.mqtt_client,
            "publish",
            return_value=SimpleNamespace(rc=erp.mqtt.MQTT_ERR_SUCCESS),
        ):
            accepted = self.client.post("/api/objects/refresh")
        with patch.object(erp.mqtt_client, "is_connected", return_value=False):
            disconnected = self.client.post("/api/objects/refresh")

        self.assertEqual(accepted.status_code, 202)
        self.assertEqual(disconnected.status_code, 503)


class FleetObjectBridgeTests(unittest.TestCase):
    def setUp(self):
        self.previous_registry = fleet.object_registry
        fleet.object_registry = ObjectRegistry()

    def tearDown(self):
        fleet.object_registry = self.previous_registry

    def test_raw_supervisor_snapshot_is_processed_and_published(self):
        discovered = discover_objects(Supervisor(world_objects()))
        payload = fresh_snapshot(discovered["objects"])
        message = SimpleNamespace(
            topic=fleet.OBJECTS_RAW_TOPIC,
            payload=json.dumps(payload).encode("utf-8"),
            retain=False,
        )

        with patch.object(fleet, "publish_object_registry") as publish:
            fleet.on_message(None, None, message)

        snapshot = fleet.object_registry.snapshot()
        self.assertEqual(len(snapshot["objects"]), 4)
        self.assertGreaterEqual(len(snapshot["relationships"]), 3)
        publish.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
