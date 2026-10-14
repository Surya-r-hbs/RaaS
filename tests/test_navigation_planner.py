import sys
import unittest
from datetime import datetime, timezone
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONTROLLER_DIR = PROJECT_ROOT / "webots/controllers/robot_controller"
if str(CONTROLLER_DIR) not in sys.path:
    sys.path.insert(0, str(CONTROLLER_DIR))

from navigation_planner import plan_route
from node_manager import NodeManager


FLOOR = {
    "min": {"x": -5.0, "y": -5.0},
    "max": {"x": 5.0, "y": 5.0},
}


def rectangle(name, min_x, min_y, max_x, max_y, **metadata):
    return {
        "id": name,
        "name": name,
        "bounds": {
            "min": {"x": min_x, "y": min_y},
            "max": {"x": max_x, "y": max_y},
        },
        **metadata,
    }


class NavigationPlannerTests(unittest.TestCase):
    def test_clear_route_remains_direct_in_x_y(self):
        result = plan_route(
            {"x": -2.0, "y": -1.0},
            {"x": 2.0, "y": 1.0},
            [],
            FLOOR,
            footprint_radius=0.2,
        )

        self.assertEqual(
            result["path"],
            [(-2.0, -1.0), (2.0, 1.0)],
        )
        self.assertFalse(result["approach"])

    def test_route_goes_around_a_table_between_robot_and_goal(self):
        table = rectangle("table", -0.75, -0.6, 0.75, 0.6)

        result = plan_route(
            {"x": -3.0, "y": 0.0},
            {"x": 3.0, "y": 0.0},
            [table],
            FLOOR,
            footprint_radius=0.3,
        )

        self.assertGreater(len(result["path"]), 2)
        self.assertTrue(
            any(abs(point[1]) > 0.8 for point in result["path"][1:-1])
        )

    def test_goal_inside_unknown_obstacle_is_rejected(self):
        wall = rectangle("wall", -0.5, -0.5, 0.5, 0.5)

        with self.assertRaisesRegex(ValueError, "conflicts with wall"):
            plan_route(
                {"x": -2.0, "y": 0.0},
                {"x": 0.0, "y": 0.0},
                [wall],
                FLOOR,
                footprint_radius=0.2,
            )

    def test_station_table_goal_uses_a_reachable_approach_pose(self):
        table = rectangle(
            "N0",
            -0.75,
            -0.6,
            0.75,
            0.6,
            approachAllowed=True,
        )

        result = plan_route(
            {"x": -3.0, "y": 0.0},
            {"x": 0.0, "y": 0.0},
            [table],
            FLOOR,
            footprint_radius=0.3,
        )

        self.assertTrue(result["approach"])
        self.assertGreater(len(result["path"]), 1)
        self.assertGreater(
            abs(result["goal"]["x"]) + abs(result["goal"]["y"]),
            0.9,
        )

    def test_no_route_is_returned_when_an_obstacle_seals_the_floor(self):
        wall = rectangle("wall", -0.1, -5.0, 0.1, 5.0)

        with self.assertRaisesRegex(ValueError, "No collision-free route"):
            plan_route(
                {"x": -2.0, "y": 0.0},
                {"x": 2.0, "y": 0.0},
                [wall],
                FLOOR,
                footprint_radius=0.2,
            )

    def test_inflated_footprint_rejects_a_gap_that_is_too_narrow(self):
        obstacles = [
            rectangle("lower-wall", -0.1, -5.0, 0.1, -0.25),
            rectangle("upper-wall", -0.1, 0.25, 0.1, 5.0),
        ]

        with self.assertRaisesRegex(ValueError, "No collision-free route"):
            plan_route(
                {"x": -2.0, "y": 0.0},
                {"x": 2.0, "y": 0.0},
                obstacles,
                FLOOR,
                footprint_radius=0.3,
                clearance=0.1,
            )

    def test_new_dynamic_robot_pose_is_planned_as_an_obstacle(self):
        manager = NodeManager("robot-a")
        manager.update_world_snapshot(
            {
                "discoveryType": "WORLD_SNAPSHOT",
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "navigationNodes": {
                    "N1": {"x": 2.0, "y": 0.0, "z": 0.0},
                },
                "worldRobots": [
                    {
                        "robotId": "robot-a",
                        "robotType": "Youbot",
                        "mobile": True,
                        "footprintRadius": 0.25,
                        "position": {"x": -2.0, "y": 0.0, "z": 0.0},
                        "orientation": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                    },
                    {
                        "robotId": "robot-b",
                        "robotType": "Pioneer3at",
                        "mobile": True,
                        "footprintRadius": 0.3,
                        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "orientation": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                    },
                ],
                "navigationMap": {
                    "floorBounds": FLOOR,
                    "obstacles": [],
                    "complete": True,
                    "issues": [],
                },
            }
        )

        result = manager.plan_navigation_path(
            {"x": -2.0, "y": 0.0, "z": 0.0},
            "N1",
        )

        self.assertGreater(len(result["path"]), 2)
        self.assertTrue(
            any(abs(point[1]) > 0.6 for point in result["path"][1:-1])
        )

    def test_missing_robot_footprint_fails_closed(self):
        manager = NodeManager("robot-a")
        manager.update_world_snapshot(
            {
                "discoveryType": "WORLD_SNAPSHOT",
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "navigationNodes": {
                    "N1": {"x": 2.0, "y": 0.0, "z": 0.0},
                },
                "worldRobots": [
                    {
                        "robotId": "robot-a",
                        "mobile": True,
                        "position": {"x": -2.0, "y": 0.0, "z": 0.0},
                        "orientation": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                    },
                ],
                "navigationMap": {
                    "floorBounds": FLOOR,
                    "obstacles": [],
                    "complete": True,
                    "issues": [],
                },
            }
        )

        with self.assertRaisesRegex(RuntimeError, "footprint.*unknown"):
            manager.plan_navigation_path(
                {"x": -2.0, "y": 0.0, "z": 0.0},
                "N1",
            )


if __name__ == "__main__":
    unittest.main()
