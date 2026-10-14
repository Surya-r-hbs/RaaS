import math
import re
import time
from datetime import datetime, timezone


# --------------------
# NODE MANAGER
# --------------------

class NodeManager:
    MAX_SNAPSHOT_AGE_SECONDS = 5.0
    MAX_FUTURE_SKEW_SECONDS = 5.0

    def __init__(self, robot_id=None):
        self.active_robot_id = robot_id
        self.nodes = {}
        self.robots = {}
        self._snapshot_received_at = None
        self._last_accepted_snapshot = None

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
            discovered_robots[robot_id] = record
        self.nodes = discovered_nodes
        self.robots = discovered_robots
        self._last_accepted_snapshot = snapshot_time
        self._snapshot_received_at = time.monotonic()

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