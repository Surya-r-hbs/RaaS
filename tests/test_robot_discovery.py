import sys
import unittest
from datetime import datetime, timezone
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONTROLLER_DIR = PROJECT_ROOT / "webots/controllers/robot_controller"
SUPERVISOR_DIR = PROJECT_ROOT / "webots/controllers/object_supervisor"
FLEET_DIR = PROJECT_ROOT / "fleet_control"
sys.path[:0] = [str(CONTROLLER_DIR), str(SUPERVISOR_DIR), str(FLEET_DIR)]

from robot_discovery import discover_local_robot
from robot_registry import RobotRegistry
from node_manager import NodeManager
from world_discovery import discover_world_snapshot


class Field:
    def __init__(self, value=None):
        self.value = value

    def getSFString(self):
        return self.value

    def getMFNode(self, index):
        return self.value[index]

    def getCount(self):
        return len(self.value)


class Node:
    def __init__(
        self,
        node_id,
        type_name,
        base_type,
        name="",
        definition="",
        proto_type=None,
        children=(),
        controller="",
        custom_data="",
        position=(0.0, 0.0, 0.0),
        orientation=(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
    ):
        self.node_id = node_id
        self.type_name = type_name
        self.base_type = base_type
        self.name = name
        self.definition = definition
        self.proto_type = proto_type
        self.children = list(children)
        self.controller = controller
        self.custom_data = custom_data
        self.position = position
        self.orientation = orientation

    def getTypeName(self):
        return self.type_name

    def getBaseTypeName(self):
        return self.base_type

    def getProtoTypeName(self):
        return self.proto_type

    def getField(self, name):
        fields = {
            "children": self.children,
            "name": self.name,
            "controller": self.controller,
            "customData": self.custom_data,
        }
        return Field(fields[name]) if name in fields else None

    def getDef(self):
        return self.definition

    def getId(self):
        return self.node_id

    def getPosition(self):
        return self.position

    def getOrientation(self):
        return self.orientation


class Supervisor:
    def __init__(self, root_children, self_node):
        self.root_children = root_children
        self.self_node = self_node

    def getRoot(self):
        return Node(
            -1,
            "Root",
            "Root",
            children=self.root_children,
        )

    def getSelf(self):
        return self.self_node


class RobotDiscoveryTests(unittest.TestCase):
    def test_local_identity_does_not_require_world_access(self):
        class LocalRobot:
            def getName(self):
                return "Pioneer3at"

            def getModel(self):
                return "Pioneer 3-AT"

        self.assertEqual(
            discover_local_robot(LocalRobot()),
            {
                "robotId": "Pioneer3at",
                "manufacturer": "DemoRobot",
                "robotType": "Pioneer 3-AT",
            },
        )

    def test_discovers_nested_arm_type_and_station_association(self):
        arm = Node(
            2,
            "Robot",
            "Robot",
            name="UR10e-N4",
            proto_type="UR10e",
            position=(1.0, 2.0, 3.0),
        )
        station = Node(
            1,
            "Solid",
            "Solid",
            definition="N4",
            children=[arm],
        )
        supervisor = Supervisor([station], arm)

        snapshot = discover_world_snapshot(supervisor)

        self.assertEqual(snapshot["discoveryType"], "WORLD_SNAPSHOT")
        self.assertEqual(len(snapshot["worldRobots"]), 1)
        record = snapshot["worldRobots"][0]
        self.assertEqual(record["robotId"], "UR10e-N4")
        self.assertEqual(record["robotType"], "UR10e")
        self.assertEqual(record["stationId"], "N4")
        self.assertEqual(record["position"], {"x": 1.0, "y": 2.0, "z": 3.0})
        self.assertEqual(len(record["orientation"]), 9)

    def test_discovers_robot_proto_by_controller_when_base_type_is_solid(self):
        robot = Node(
            3,
            "PROTO",
            "Solid",
            name="Youbot",
            proto_type="Youbot",
            controller="robot_controller",
            position=(1.0, 2.0, 0.1),
        )

        snapshot = discover_world_snapshot(Supervisor([robot], robot))

        self.assertEqual(len(snapshot["worldRobots"]), 1)
        self.assertEqual(
            snapshot["worldRobots"][0]["position"],
            {"x": 1.0, "y": 2.0, "z": 0.1},
        )

    def test_missing_proto_model_does_not_abort_world_snapshot(self):
        robots = [
            Node(
                node_id,
                "PROTO",
                "Solid",
                name=name,
                proto_type=proto_type,
                controller="robot_controller",
            )
            for node_id, name, proto_type in (
                (5, "MiR100", None),
                (6, "Tiago", "Robot"),
            )
        ]

        snapshot = discover_world_snapshot(Supervisor(robots, robots[0]))

        self.assertEqual(len(snapshot["worldRobots"]), 2)
        self.assertEqual(
            [robot["robotId"] for robot in snapshot["worldRobots"]],
            ["MiR100", "Tiago"],
        )
        self.assertTrue(
            all("robotType" not in robot for robot in snapshot["worldRobots"])
        )

    def test_reads_station_from_world_authored_custom_data(self):
        arm = Node(
            4,
            "Robot",
            "Robot",
            name="UR10e-source",
            proto_type="UR10e",
            custom_data="stationId=SOURCE_TABLE",
        )
        supervisor = Supervisor([arm], arm)

        discovered = discover_world_snapshot(supervisor)

        self.assertEqual(
            discovered["worldRobots"][0]["stationId"],
            "SOURCE_TABLE",
        )

    def test_station_association_is_available_in_fleet_registry(self):
        arm = Node(
            6,
            "Robot",
            "Robot",
            name="UR10e-N4",
            proto_type="UR10e",
        )
        station = Node(
            7,
            "Solid",
            "Solid",
            definition="N4",
            children=[arm],
        )
        discovered = discover_world_snapshot(Supervisor([station], arm))
        registry = RobotRegistry()

        registry.discover_world(discovered["worldRobots"])

        arm_record = registry.get("DemoRobot", "UR10e-N4")
        self.assertEqual(arm_record["stationId"], "N4")

    def test_excludes_object_supervisor(self):
        supervisor_robot = Node(
            5,
            "Robot",
            "Robot",
            name="object-inventory-supervisor",
            controller="object_supervisor",
        )
        supervisor = Supervisor([supervisor_robot], supervisor_robot)

        self.assertEqual(
            discover_world_snapshot(supervisor)["worldRobots"],
            [],
        )

    def test_world_discovery_fails_clearly_without_supervisor_privilege(self):
        class NoWorldAccess:
            def getRoot(self):
                class InaccessibleRoot:
                    def getField(self, _name):
                        return None

                return InaccessibleRoot()

        with self.assertRaisesRegex(
            RuntimeError,
            "dedicated object Supervisor cannot inspect",
        ):
            discover_world_snapshot(NoWorldAccess())

    def test_world_snapshot_supplies_navigation_and_local_robot_pose(self):
        manager = NodeManager("Pioneer3at")
        manager.update_world_snapshot(
            {
                "discoveryType": "WORLD_SNAPSHOT",
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "navigationNodes": {
                    "N4": {"x": 1.0, "y": 2.0, "z": 0.0},
                },
                "worldRobots": [
                    {
                        "robotId": "Pioneer3at",
                        "position": {"x": 3.0, "y": 4.0, "z": 0.0},
                        "orientation": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                    },
                ],
            }
        )

        self.assertEqual(
            manager.get_node("N4"),
            {"x": 1.0, "y": 2.0, "z": 0.0},
        )
        self.assertEqual(
            manager.get_robot_position(),
            {"x": 3.0, "y": 4.0, "z": 0.0},
        )
        self.assertEqual(len(manager.get_robot_orientation()), 9)

    def test_world_snapshot_requires_robot_identity_for_pose(self):
        manager = NodeManager("missing-robot")
        manager.update_world_snapshot(
            {
                "discoveryType": "WORLD_SNAPSHOT",
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "navigationNodes": {},
                "worldRobots": [],
            }
        )
        with self.assertRaisesRegex(RuntimeError, "no world snapshot record"):
            manager.get_robot_position()


if __name__ == "__main__":
    unittest.main()
