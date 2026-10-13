"""Cartesian pick-and-place adapter for the Webots UR10e."""

import math

from adapters.base_robot import BaseRobot
from adapters.ur10e_kinematics import inverse_kinematics
from config import TIME_STEP


JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
JOINT_SENSOR_NAMES = tuple(f"{name}_sensor" for name in JOINT_NAMES)
APPROACH_CLEARANCE = 0.12
MOVE_TOLERANCE = 0.025
MOVE_TIMEOUT_STEPS = 1600
ATTACH_TIMEOUT_STEPS = 40
RELEASE_STEPS = 5
MAX_BASE_REACH = 1.25


def _vector(value, field_name):
    if not isinstance(value, dict):
        raise ValueError(f"{field_name} must be an object with x, y, and z")
    try:
        result = tuple(float(value[axis]) for axis in ("x", "y", "z"))
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            f"{field_name} must contain numeric x, y, and z values"
        ) from error
    if not all(math.isfinite(component) for component in result):
        raise ValueError(f"{field_name} values must be finite")
    return result


def _transpose_rotation(rotation):
    return [
        [rotation[column][row] for column in range(3)]
        for row in range(3)
    ]


def _rotate(rotation, vector):
    return tuple(
        sum(rotation[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )


def _multiply_rotation(left, right):
    return [
        [
            sum(left[row][index] * right[index][column] for index in range(3))
            for column in range(3)
        ]
        for row in range(3)
    ]


class UR10eAdapter(BaseRobot):
    def __init__(self, robot, node_manager, robot_type):
        super().__init__(robot, node_manager, robot_type)
        vacuum_motors = [
            device
            for name, device in self.devices.items()
            if name.endswith(" vacuum joint")
            and callable(getattr(device, "setPosition", None))
        ]
        if len(vacuum_motors) != 1:
            raise RuntimeError(
                "UR10e requires exactly one EPick vacuum-joint motor; "
                f"found {len(vacuum_motors)}"
            )
        # EPick's SliderJoint has a valid range of [-0.01, 0]. Webots can
        # restore a tiny positive saved target, which is outside that range.
        vacuum_motors[0].setPosition(0.0)

        self.joint_motors = [
            self.require_device(name)
            for name in JOINT_NAMES
        ]
        self.joint_sensors = [
            self.require_device(name)
            for name in JOINT_SENSOR_NAMES
        ]
        for motor in self.joint_motors:
            motor.setVelocity(0.7)

        self.vacuum_gripper = next(
            (
                device
                for device in self.devices.values()
                if callable(getattr(device, "turnOn", None))
                and callable(getattr(device, "turnOff", None))
                and callable(getattr(device, "enablePresence", None))
                and callable(getattr(device, "getPresence", None))
            ),
            None,
        )
        if self.vacuum_gripper is None:
            raise RuntimeError(
                "UR10e requires a Webots VacuumGripper device; attach the "
                "Robotiq EPick gripper in the world file"
            )
        self.vacuum_gripper.enablePresence(TIME_STEP)

        self.base_position = None
        self.base_rotation = None
        self.base_rotation_inverse = None
        self.task = None
        self.capabilities.add("PICK_AND_PLACE")

    @property
    def manipulation_status(self):
        if self.task is None:
            return None
        return {
            "taskId": self.task["task_id"],
            "state": self.task["state"],
            "phase": self.task["phase"],
            "error": self.task["error"],
        }

    @property
    def task_active(self):
        return self.task is not None and self.task["state"] == "EXECUTING"

    def start_pick_and_place(self, command):
        if self.task_active:
            raise RuntimeError("UR10e already has an active manipulation task")
        self._ensure_base_pose()
        if self.vacuum_gripper.getPresence():
            raise RuntimeError(
                "UR10e gripper is already attached to an object; release it "
                "before starting a new task"
            )

        pick_position = _vector(command.get("pickPosition"), "pickPosition")
        place_position = _vector(command.get("placePosition"), "placePosition")
        task_id = str(command.get("taskId") or command.get("orderId") or "")
        if not task_id:
            raise ValueError("PICK_AND_PLACE requires taskId or orderId")

        for label, position in (
            ("pickPosition", pick_position),
            ("placePosition", place_position),
        ):
            base_point = self._world_to_base(position)
            if math.sqrt(sum(value * value for value in base_point[:2])) > MAX_BASE_REACH:
                raise ValueError(
                    f"{label} is outside the UR10e's configured horizontal "
                    "reach"
                )

        transfer_height = (
            max(pick_position[2], place_position[2]) + APPROACH_CLEARANCE
        )
        self.task = {
            "task_id": task_id,
            "state": "EXECUTING",
            "phase": "APPROACH_PICK",
            "error": None,
            "targets": (
                (pick_position[0], pick_position[1], pick_position[2] + APPROACH_CLEARANCE),
                pick_position,
                (pick_position[0], pick_position[1], transfer_height),
                (place_position[0], place_position[1], transfer_height),
                place_position,
                (place_position[0], place_position[1], place_position[2] + APPROACH_CLEARANCE),
            ),
            "target_index": 0,
            "step_count": 0,
            "release_count": 0,
            "target_joints": None,
        }
        for motor in self.joint_motors:
            motor.setVelocity(0.7)
        try:
            self._move_to_current_target()
        except (ValueError, RuntimeError) as error:
            self._fail(str(error))
            raise
        return self.manipulation_status

    def _ensure_base_pose(self):
        position = self.get_position()
        orientation = self.get_orientation()
        self.base_position = tuple(
            position[axis] for axis in ("x", "y", "z")
        )
        self.base_rotation = [
            [orientation[row * 3 + column] for column in range(3)]
            for row in range(3)
        ]
        self.base_rotation_inverse = _transpose_rotation(self.base_rotation)

    def _world_to_base(self, point):
        difference = tuple(
            point[axis] - self.base_position[axis]
            for axis in range(3)
        )
        return _rotate(self.base_rotation_inverse, difference)

    def _target_transform(self, world_position):
        base_position = self._world_to_base(world_position)
        tool_down_world = [
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
        ]
        tool_down_base = _multiply_rotation(
            self.base_rotation_inverse,
            tool_down_world,
        )
        return [
            [tool_down_base[row][column] for column in range(3)]
            + [base_position[row]]
            for row in range(3)
        ] + [[0.0, 0.0, 0.0, 1.0]]

    def _move_to_current_target(self):
        target_position = self.task["targets"][self.task["target_index"]]
        self.task["target_joints"] = inverse_kinematics(
            self._target_transform(target_position),
            [sensor.getValue() for sensor in self.joint_sensors],
        )
        for motor, angle in zip(
            self.joint_motors,
            self.task["target_joints"],
        ):
            motor.setPosition(angle)
        self.task["step_count"] = 0

    def update_manipulation(self):
        if not self.task_active:
            return False

        task = self.task
        task["step_count"] += 1
        if task["phase"] == "WAIT_FOR_GRASP":
            if self.vacuum_gripper.getPresence():
                self._advance_target("LIFT_OBJECT")
            elif task["step_count"] >= ATTACH_TIMEOUT_STEPS:
                self._fail("No object was detected by the suction cup")
            return True

        if task["phase"] == "WAIT_FOR_RELEASE":
            task["release_count"] -= 1
            if task["release_count"] <= 0:
                self._advance_target("RETREAT_PLACE")
            return True

        target = task["target_joints"]
        actual = [sensor.getValue() for sensor in self.joint_sensors]
        if max(abs(goal - current) for goal, current in zip(target, actual)) > MOVE_TOLERANCE:
            if task["step_count"] >= MOVE_TIMEOUT_STEPS:
                self._fail(f"Timed out while moving during {task['phase']}")
            return False

        if task["phase"] == "DESCEND_PICK":
            self.vacuum_gripper.turnOn()
            task["phase"] = "WAIT_FOR_GRASP"
            task["step_count"] = 0
            return True
        if task["phase"] == "DESCEND_PLACE":
            self.vacuum_gripper.turnOff()
            task["phase"] = "WAIT_FOR_RELEASE"
            task["release_count"] = RELEASE_STEPS
            return True
        if task["target_index"] == len(task["targets"]) - 1:
            task["state"] = "COMPLETED"
            task["phase"] = "COMPLETED"
            return True

        next_phase = {
            "APPROACH_PICK": "DESCEND_PICK",
            "LIFT_OBJECT": "TRANSFER",
            "TRANSFER": "DESCEND_PLACE",
        }.get(task["phase"], "TRANSFER")
        self._advance_target(next_phase)
        return True

    def _advance_target(self, phase):
        self.task["target_index"] += 1
        self.task["phase"] = phase
        try:
            self._move_to_current_target()
        except (ValueError, RuntimeError) as error:
            self._fail(str(error))

    def _fail(self, message):
        self.vacuum_gripper.turnOff()
        for motor, sensor in zip(self.joint_motors, self.joint_sensors):
            motor.setPosition(sensor.getValue())
            motor.setVelocity(0.0)
        self.task["state"] = "FAILED"
        self.task["phase"] = "FAILED"
        self.task["error"] = message

    def stop(self):
        if self.vacuum_gripper.isOn():
            self.vacuum_gripper.turnOff()
        for motor, sensor in zip(self.joint_motors, self.joint_sensors):
            motor.setPosition(sensor.getValue())
            motor.setVelocity(0.0)
        if self.task_active:
            self.task["state"] = "CANCELLED"
            self.task["phase"] = "CANCELLED"
        super().stop()
