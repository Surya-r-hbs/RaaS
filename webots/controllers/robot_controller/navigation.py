import math

from navigation_planner import ProgressMonitor


class Navigation:

    def __init__(
        self,
        robot,
        left_motor,
        right_motor,
        gps,
        compass,
        node_manager,
        position_tolerance=0.12,
        max_speed=4.0,
        turn_speed=2.0,
        heading_tolerance_degrees=5.0,
    ):

        # --------------------
        # ROBOT
        # --------------------

        self.robot = robot

        # --------------------
        # MOTORS
        # --------------------

        self.left_motor = left_motor
        self.right_motor = right_motor

        # --------------------
        # SENSORS
        # --------------------

        self.gps = gps
        self.compass = compass

        # --------------------
        # NODE MANAGER
        # --------------------

        self.node_manager = node_manager

        # --------------------
        # NAVIGATION SETTINGS
        # --------------------

        self.position_tolerance = position_tolerance
        self.max_speed = max_speed
        self.turn_speed = turn_speed
        self.heading_tolerance_degrees = heading_tolerance_degrees
        self._target_node = None
        self._arrival_samples = 0
        self._arrived_node = None
        self._planned_snapshot = None
        self._path = []
        self._path_index = 0
        self.arrival_samples_required = 3
        self._progress_monitor = ProgressMonitor()

        # --------------------
        # DEBUG
        # --------------------

        self.debug_counter = 0
        self.debug_interval = 20

    # --------------------
    # STOP ROBOT
    # --------------------

    def stop(self):

        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    # --------------------
    # ANGLE NORMALIZATION
    # --------------------

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    # --------------------
    # GET ROBOT POSITION
    #
    # Project convention:
    # X = horizontal
    # Y = floor/depth
    # Z = height
    # --------------------

    def get_robot_position(self):

        if self.gps is not None:
            values = self.gps.getValues()
        else:
            try:
                values = self.robot.getSelf().getPosition()
            except (AttributeError, RuntimeError):
                position = self.node_manager.get_robot_position()
                return position

        position = {
            "x": float(values[0]),
            "y": float(values[1]),
            "z": float(values[2]),
        }
        if not all(math.isfinite(value) for value in position.values()):
            raise RuntimeError("Robot position is unavailable or invalid")
        return position

    # --------------------
    # GET ROBOT FORWARD VECTOR
    #
    # The e-puck wheels are mounted along
    # its local Y axis, so its drive-forward
    # axis is local +X.
    # --------------------

    def get_robot_forward_vector(self):

        if self.robot is None:
            raise RuntimeError("Robot orientation is unavailable")

        try:
            orientation = self.robot.getSelf().getOrientation()
        except (AttributeError, RuntimeError):
            orientation = self.node_manager.get_robot_orientation()

        if not orientation or len(orientation) < 9:
            raise RuntimeError("Robot orientation is unavailable or invalid")

        # Webots returns the 3x3 orientation matrix in row-major order.
        # Its first column transforms local +X into world coordinates.
        forward_x = orientation[0]
        forward_y = orientation[3]
        magnitude = math.hypot(forward_x, forward_y)

        if (
            not math.isfinite(forward_x)
            or not math.isfinite(forward_y)
            or not math.isfinite(magnitude)
            or magnitude <= 1e-9
        ):
            raise RuntimeError("Robot orientation is unavailable or invalid")

        return forward_x / magnitude, forward_y / magnitude

    # --------------------
    # GET ROBOT HEADING
    #
    # Heading convention:
    #
    #   0 deg   = +X
    #   +90 deg = +Y
    #   -90 deg = -Y
    #   180 deg = -X
    #
    # This matches the X/Y warehouse
    # coordinate system used by this project.
    # --------------------

    def get_robot_heading(self):

        forward_x, forward_y = (
            self.get_robot_forward_vector()
        )

        return math.atan2(forward_y, forward_x)

    # --------------------
    # GET TARGET HEADING
    # --------------------

    def get_target_heading(
        self,
        dx,
        dy,
    ):

        return math.atan2(dy, dx)

    # --------------------
    # SET WHEEL SPEED
    # --------------------

    def set_wheels(
        self,
        left_speed,
        right_speed,
    ):

        left_speed = max(
            -self.max_speed,
            min(self.max_speed, left_speed)
        )

        right_speed = max(
            -self.max_speed,
            min(self.max_speed, right_speed)
        )

        self.left_motor.setVelocity(
            left_speed
        )

        self.right_motor.setVelocity(
            right_speed
        )

    # --------------------
    # MOVE TO NODE
    # --------------------

    def move_to_node(self, node_id):
        target = self.node_manager.get_node(node_id)
        if target is None:
            self.stop()
            return {"reached": False, "error": f"Node not found: {node_id}"}

        if node_id != self._target_node:
            self._target_node = node_id
            self._arrival_samples = 0
            self._arrived_node = None
            self._planned_snapshot = None
            self._path = []
            self._path_index = 0
            self._progress_monitor.reset()

        if self._arrived_node == node_id:
            self.stop()
            return {"reached": True, "error": False}

        try:
            robot_position = self.get_robot_position()
            generation = getattr(
                self.node_manager,
                "_snapshot_generation",
                None,
            )
            if not self._path or generation != self._planned_snapshot:
                plan = self.node_manager.plan_navigation_path(
                    robot_position,
                    node_id,
                )
                self._path = plan["path"]
                self._path_index = min(1, len(self._path) - 1)
                self._planned_snapshot = generation
                self._goal = plan["goal"]
                self._approach = plan["approach"]
        except (KeyError, TypeError, ValueError, RuntimeError) as error:
            self.stop()
            return {"reached": False, "error": str(error)}

        target_x = self._goal["x"]
        target_y = self._goal["y"]
        dx = target_x - robot_position["x"]
        dy = target_y - robot_position["y"]
        distance = math.hypot(dx, dy)
        if not math.isfinite(distance):
            self.stop()
            return {"reached": False, "error": "Robot-to-goal distance is invalid"}

        if distance <= self.position_tolerance:
            self.stop()
            self._arrival_samples += 1
            if self._arrival_samples >= self.arrival_samples_required:
                self._arrived_node = node_id
                return {"reached": True, "error": False}
            return {"reached": False, "error": False}

        self._arrival_samples = 0
        while self._path_index < len(self._path) - 1:
            waypoint = self._path[self._path_index]
            waypoint_distance = math.hypot(
                waypoint[0] - robot_position["x"],
                waypoint[1] - robot_position["y"],
            )
            if waypoint_distance > self.position_tolerance:
                break
            self._path_index += 1
        waypoint = self._path[self._path_index]
        dx = waypoint[0] - robot_position["x"]
        dy = waypoint[1] - robot_position["y"]
        waypoint_distance = math.hypot(dx, dy)
        route_remaining = waypoint_distance + sum(
            math.hypot(
                second[0] - first[0],
                second[1] - first[1],
            )
            for first, second in zip(
                self._path[self._path_index:],
                self._path[self._path_index + 1:],
            )
        )
        if self._progress_monitor.stalled(route_remaining):
            self.stop()
            return {
                "reached": False,
                "error": "Navigation made insufficient progress; route is blocked",
            }
        if waypoint_distance <= self.position_tolerance:
            dx = target_x - robot_position["x"]
            dy = target_y - robot_position["y"]

        try:
            robot_heading = self.get_robot_heading()
        except (TypeError, ValueError, RuntimeError) as error:
            self.stop()
            return {"reached": False, "error": str(error)}

        heading_error = self.normalize_angle(
            self.get_target_heading(dx, dy) - robot_heading
        )
        if not math.isfinite(heading_error):
            self.stop()
            return {"reached": False, "error": "Robot heading is invalid"}

        if abs(math.degrees(heading_error)) > self.heading_tolerance_degrees:
            turn_speed = min(self.turn_speed, max(0.25, abs(heading_error)))
            direction = 1.0 if heading_error > 0 else -1.0
            self.set_wheels(-direction * turn_speed, direction * turn_speed)
        else:
            self.set_wheels(
                min(self.max_speed, max(0.25, distance * 2.0)),
                min(self.max_speed, max(0.25, distance * 2.0)),
            )

        return {"reached": False, "error": False}