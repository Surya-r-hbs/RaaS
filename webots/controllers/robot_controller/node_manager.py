import math
import re
import time
from datetime import datetime, timezone

from navigation_planner import plan_route


# --------------------
# NODE MANAGER
# --------------------

class NodeManager:
    MAX_SNAPSHOT_AGE_SECONDS = 5.0
    MAX_FUTURE_SKEW_SECONDS = 5.0
    MAX_DYNAMIC_OBSTACLE_AGE_SECONDS = 0.5

    def __init__(self, robot_id=None):
        self.active_robot_id = robot_id
        self.nodes = {}
        self.robots = {}
        self.navigation_map = None
        self._snapshot_received_at = None
        self._last_accepted_snapshot = None
        self._snapshot_generation = 0

    @staticmethod
    def _parse_snapshot_timestamp(timestamp):
        if timestamp is None:
            raise ValueError("World snapshot timestamp is missing")
        timestamp_text = str(timestamp).strip()
        if not timestamp_text:
            raise ValueError("World snapshot timestamp is missing")
        try:
            parsed = datetime.fromisoformat(timestamp_text.replace("Z", "+00:00"))
        except (TypeError, ValueError) as error:
            raise ValueError("World snapshot has no valid UTC timestamp") from error
        if parsed.tzinfo is None:
            raise ValueError("World snapshot timestamp must include a UTC offset")
        return parsed.astimezone(timezone.utc)

    def update_world_snapshot(self, payload):
        if payload.get("discoveryType") != "WORLD_SNAPSHOT":
            raise ValueError("Expected a Webots WORLD_SNAPSHOT discovery message")
        snapshot_time = self._parse_snapshot_timestamp(payload.get("timestamp"))
        now_utc = datetime.now(timezone.utc)
        age = (now_utc - snapshot_time).total_seconds()
        future_skew = (snapshot_time - now_utc).total_seconds()
        if age > self.MAX_SNAPSHOT_AGE_SECONDS:
            raise ValueError(
                f"World snapshot is stale ({age:.2f}s older than the allowed "
                f"{self.MAX_SNAPSHOT_AGE_SECONDS:.1f}s)"
            )
        if future_skew > self.MAX_FUTURE_SKEW_SECONDS:
            raise ValueError(
                f"World snapshot is from the future ({future_skew:.2f}s ahead)"
            )
        if (
            self._last_accepted_snapshot is not None
            and snapshot_time < self._last_accepted_snapshot
        ):
            raise ValueError(
                "World snapshot is older than the last accepted snapshot"
            )
        nodes = payload.get("navigationNodes")
        robots = payload.get("worldRobots")
        if not isinstance(nodes, dict) or not isinstance(robots, list):
            raise ValueError("World snapshot is missing navigationNodes or worldRobots")
        discovered_nodes = {}
        for node_id, position in nodes.items():
            if not re.fullmatch(r"N\d+", str(node_id)):
                raise ValueError(f"Invalid navigation node ID: {node_id!r}")
            discovered_nodes[node_id] = self._vector(
                position,
                ("x", "y", "z"),
                f"navigation node {node_id}",
            )

        discovered_robots = {}
        for record in robots:
            if not isinstance(record, dict):
                raise ValueError("World snapshot contains an invalid robot record")
            robot_id = str(record.get("robotId") or "").strip()
            if not robot_id or robot_id in discovered_robots:
                raise ValueError(
                    f"World snapshot has a missing or duplicate robot ID: {robot_id!r}"
                )
            self._vector(
                record.get("position"),
                ("x", "y", "z"),
                f"robot {robot_id} position",
            )
            orientation = record.get("orientation")
            if (
                not isinstance(orientation, list)
                or len(orientation) != 9
                or not all(
                    isinstance(value, (int, float)) and math.isfinite(value)
                    for value in orientation
                )
            ):
                raise ValueError(
                    f"World snapshot has invalid orientation for {robot_id!r}"
                )
            footprint_radius = record.get("footprintRadius")
            if footprint_radius is not None and (
                not isinstance(footprint_radius, (int, float))
                or not math.isfinite(footprint_radius)
                or footprint_radius <= 0
            ):
                raise ValueError(
                    f"World snapshot has invalid footprint for {robot_id!r}"
                )
            discovered_robots[robot_id] = record
        navigation_map = payload.get("navigationMap")
        if navigation_map is not None:
            self._validate_navigation_map(navigation_map)
        self.nodes = discovered_nodes
        self.robots = discovered_robots
        self.navigation_map = navigation_map
        self._last_accepted_snapshot = snapshot_time
        self._snapshot_received_at = time.monotonic()
        self._snapshot_generation += 1

    @classmethod
    def _validate_navigation_map(cls, navigation_map):
        if not isinstance(navigation_map, dict):
            raise ValueError("World snapshot navigationMap must be an object")
        if not isinstance(navigation_map.get("complete"), bool):
            raise ValueError(
                "World snapshot navigationMap must declare whether it is complete"
            )
        obstacles = navigation_map.get("obstacles")
        if not isinstance(obstacles, list):
            raise ValueError("World snapshot navigationMap has no obstacle list")
        if navigation_map.get("complete"):
            floor_bounds = navigation_map.get("floorBounds")
            if not isinstance(floor_bounds, dict):
                raise ValueError("Complete navigationMap has no floor bounds")
            cls._validate_bounds(floor_bounds, "floor")
        for obstacle in obstacles:
            if not isinstance(obstacle, dict):
                raise ValueError("World snapshot contains an invalid obstacle")
            cls._validate_bounds(
                obstacle.get("bounds"),
                str(obstacle.get("name") or obstacle.get("id") or "obstacle"),
            )

    @staticmethod
    def _validate_bounds(bounds, label):
        if not isinstance(bounds, dict):
            raise ValueError(f"World snapshot has no bounds for {label}")
        try:
            minimum = {
                axis: float(bounds["min"][axis])
                for axis in ("x", "y")
            }
            maximum = {
                axis: float(bounds["max"][axis])
                for axis in ("x", "y")
            }
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"World snapshot has invalid bounds for {label}") from error
        if (
            not all(math.isfinite(value) for value in (*minimum.values(), *maximum.values()))
            or minimum["x"] >= maximum["x"]
            or minimum["y"] >= maximum["y"]
        ):
            raise ValueError(f"World snapshot has invalid bounds for {label}")

    def plan_navigation_path(self, start_position, node_id):
        self._require_fresh_snapshot()
        if self.navigation_map is None:
            raise RuntimeError(
                "Safe navigation is unavailable: the Webots snapshot has no "
                "obstacle map"
            )
        if not self.navigation_map.get("complete"):
            issues = self.navigation_map.get("issues") or ["unknown map geometry"]
            raise RuntimeError(
                "Safe navigation is unavailable: " + "; ".join(map(str, issues))
            )
        target = self.nodes.get(node_id)
        if target is None:
            raise RuntimeError(f"Navigation node is unavailable: {node_id}")
        robot = self.get_robot_record()
        footprint_radius = robot.get("footprintRadius")
        if footprint_radius is None:
            raise RuntimeError(
                f"Safe navigation is unavailable: collision footprint for "
                f"{robot.get('robotType') or robot['robotId']} is unknown"
            )

        obstacles = list(self.navigation_map["obstacles"])
        for other in self.robots.values():
            if (
                other["robotId"] == self.active_robot_id
                or not other.get("mobile")
                or other.get("stationId")
            ):
                continue
            if (
                self._snapshot_received_at is None
                or time.monotonic() - self._snapshot_received_at
                > self.MAX_DYNAMIC_OBSTACLE_AGE_SECONDS
            ):
                raise RuntimeError(
                    "Safe navigation is unavailable: other robot poses are stale"
                )
            radius = other.get("footprintRadius")
            if radius is None:
                raise RuntimeError(
                    f"Safe navigation is unavailable: collision footprint for "
                    f"robot {other['robotId']} is unknown"
                )
            position = self._vector(
                other.get("position"),
                ("x", "y"),
                f"robot {other['robotId']} position",
            )
            obstacles.append(
                {
                    "id": f"robot:{other['robotId']}",
                    "name": f"robot {other['robotId']}",
                    "type": "dynamic_robot",
                    "bounds": {
                        "min": {
                            "x": position["x"] - radius,
                            "y": position["y"] - radius,
                        },
                        "max": {
                            "x": position["x"] + radius,
                            "y": position["y"] + radius,
                        },
                    },
                    "approachAllowed": False,
                }
            )

        try:
            return plan_route(
                start_position,
                target,
                obstacles,
                self.navigation_map["floorBounds"],
                footprint_radius,
            )
        except ValueError as error:
            raise RuntimeError(str(error)) from error

    @staticmethod
    def _vector(value, axes, label):
        if not isinstance(value, dict):
            raise ValueError(f"World snapshot has no {label}")
        try:
            vector = {axis: float(value[axis]) for axis in axes}
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"World snapshot has invalid {label}") from error
        if not all(math.isfinite(component) for component in vector.values()):
            raise ValueError(f"World snapshot has invalid {label}")
        return vector

    def _require_fresh_snapshot(self):
        if self._snapshot_received_at is None:
            raise RuntimeError("No Webots world-discovery snapshot has been received")
        age = time.monotonic() - self._snapshot_received_at
        if age > self.MAX_SNAPSHOT_AGE_SECONDS:
            raise RuntimeError(
                f"Webots world-discovery snapshot is stale ({age:.2f}s)"
            )

    def get_robot_record(self, robot_id=None):
        self._require_fresh_snapshot()
        target_id = robot_id or self.active_robot_id
        if not target_id:
            raise RuntimeError("Robot pose is unavailable: active robot ID is unset")
        record = self.robots.get(target_id)
        if record is None:
            raise RuntimeError(
                f"Robot pose is unavailable: no world snapshot record for {target_id!r}"
            )
        return record

    def get_robot_position(self, robot_id=None):
        position = self.get_robot_record(robot_id).get("position")
        if not isinstance(position, dict):
            raise RuntimeError("World snapshot robot record has no position")
        return self._vector(position, ("x", "y", "z"), "robot position")

    def get_robot_orientation(self, robot_id=None):
        orientation = self.get_robot_record(robot_id).get("orientation")
        if not isinstance(orientation, list) or len(orientation) != 9:
            raise RuntimeError("World snapshot robot record has no valid orientation")
        return [float(value) for value in orientation]

    # --------------------
    # CHECK NODE
    # --------------------

    def exists(self, node_id):

        return node_id in self.nodes

    # --------------------
    # GET NODE
    # --------------------

    def get_node(self, node_id):

        return self.nodes.get(node_id)

    # --------------------
    # GET ALL NODES
    # --------------------

    def get_nodes(self):

        return self.nodes.copy()

    # --------------------
    # PRINT NODES
    # --------------------

    def print_nodes(self):

        print("--------------------")
        print("DISCOVERED WEBOTS NODES")

        if not self.nodes:

            print("Waiting for the dedicated Webots world-discovery snapshot.")

        else:

            def node_number(name):
                return int(name[1:])

            for node_id in sorted(
                self.nodes.keys(),
                key=node_number
            ):

                position = self.nodes[node_id]

                print(
                    f"{node_id} -> "
                    f"x={position['x']:.2f}, "
                    f"y={position['y']:.2f}, "
                    f"z={position['z']:.2f}"
                )

        print("--------------------")