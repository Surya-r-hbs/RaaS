import math


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

        return {
            "x": float(values[0]),
            "y": float(values[1]),
            "z": float(values[2]),
        }

    # --------------------
    # GET ROBOT FORWARD VECTOR
    #
    # The e-puck wheels are mounted along
    # its local Y axis, so its drive-forward
    # axis is local +X.
    # --------------------

    def get_robot_forward_vector(self):

        if self.robot is None:
            return 1.0, 0.0

        try:
            orientation = self.robot.getSelf().getOrientation()
        except (AttributeError, RuntimeError):
            orientation = self.node_manager.get_robot_orientation()

        if not orientation:
            return 1.0, 0.0

        if len(orientation) < 9:
            return 1.0, 0.0

        # Webots returns the 3x3 orientation matrix in row-major order.
        # Its first column transforms local +X into world coordinates.
        forward_x = orientation[0]
        forward_y = orientation[3]
        magnitude = math.hypot(forward_x, forward_y)

        if magnitude <= 1e-9:
            return 1.0, 0.0

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

        # --------------------
        # GET TARGET NODE
        # --------------------

        target = self.node_manager.get_node(
            node_id
        )

        if target is None:

            print("--------------------")
            print("NAVIGATION ERROR")
            print(f"Node not found: {node_id}")
            print("--------------------")

            self.stop()

            return {
                "reached": False,
                "error": True,
            }

        # --------------------
        # ROBOT POSITION
        # --------------------

        robot_position = (
            self.get_robot_position()
        )

        robot_x = robot_position["x"]
        robot_y = robot_position["y"]

        # --------------------
        # TARGET POSITION
        # --------------------

        target_x = target["x"]
        target_y = target["y"]

        # --------------------
        # TARGET VECTOR
        #
        # IMPORTANT:
        # Navigation uses X/Y.
        # Z is height.
        # --------------------

        dx = target_x - robot_x
        dy = target_y - robot_y

        distance = math.sqrt(
            dx * dx +
            dy * dy
        )

        # --------------------
        # TARGET REACHED
        # --------------------

        if distance <= self.position_tolerance:

            self.stop()

            print("--------------------")
            print("TARGET REACHED")
            print(f"Node     : {node_id}")
            print(f"Distance : {distance:.3f}")
            print("--------------------")

            return {
                "reached": True,
                "error": False,
            }

        # --------------------
        # ROBOT HEADING
        # --------------------

        robot_heading = (
            self.get_robot_heading()
        )

        # --------------------
        # TARGET HEADING
        # --------------------

        target_heading = (
            self.get_target_heading(
                dx,
                dy,
            )
        )

        # --------------------
        # HEADING ERROR
        # --------------------

        heading_error = (
            self.normalize_angle(
                target_heading -
                robot_heading
            )
        )

        heading_error_degrees = math.degrees(
            heading_error
        )

        # --------------------
        # DEBUG OUTPUT
        # --------------------

        self.debug_counter += 1

        if self.debug_counter >= self.debug_interval:

            self.debug_counter = 0

            forward_x, forward_y = (
                self.get_robot_forward_vector()
            )

            print()
            print("--------------------")
            print("NAVIGATION")
            print("--------------------")

            print(
                f"Target node     : {node_id}"
            )

            print(
                f"Robot position  : "
                f"X={robot_x:.3f}, "
                f"Y={robot_y:.3f}, "
                f"Z={robot_position['z']:.3f}"
            )

            print(
                f"Target position : "
                f"X={target_x:.3f}, "
                f"Y={target_y:.3f}"
            )

            print(
                f"Delta           : "
                f"X={dx:.3f}, "
                f"Y={dy:.3f}"
            )

            print(
                f"Distance        : "
                f"{distance:.3f}"
            )

            print(
                f"Forward vector  : "
                f"X={forward_x:.3f}, "
                f"Y={forward_y:.3f}"
            )

            print(
                f"Robot heading   : "
                f"{math.degrees(robot_heading):.2f} deg"
            )

            print(
                f"Target heading  : "
                f"{math.degrees(target_heading):.2f} deg"
            )

            print(
                f"Heading error   : "
                f"{heading_error_degrees:.2f} deg"
            )

            print("--------------------")

        # --------------------
        # ROTATE
        # --------------------

        if (
            abs(heading_error_degrees)
            > self.heading_tolerance_degrees
        ):

            # Positive heading rotates from +X toward +Y. For the e-puck,
            # this requires the right wheel to move forward relative to the left.

            if heading_error > 0:

                self.set_wheels(
                    -self.turn_speed,
                    self.turn_speed,
                )

            else:

                self.set_wheels(
                    self.turn_speed,
                    -self.turn_speed,
                )

            return {
                "reached": False,
                "error": False,
            }

        # --------------------
        # MOVE FORWARD
        # --------------------

        # Slow down as we approach the node.

        forward_speed = min(
            self.max_speed,
            max(
                0.5,
                distance * 2.0
            )
        )

        self.set_wheels(
            forward_speed,
            forward_speed,
        )

        return {
            "reached": False,
            "error": False,
        }