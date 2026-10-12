"""Dependency-free inverse kinematics for the Webots UR10e model."""

import math


DH_PARAMETERS = (
    (0.1807, 0.0, math.pi / 2.0),
    (0.0, -0.6127, 0.0),
    (0.0, -0.57155, 0.0),
    (0.17415, 0.0, math.pi / 2.0),
    (0.11985, 0.0, -math.pi / 2.0),
    (0.11655, 0.0, 0.0),
)
TOOL_OFFSET = 0.232
JOINT_LIMIT = 6.27
_EPSILON = 1e-5


def _multiply(left, right):
    return [
        [
            sum(left[row][index] * right[index][column] for index in range(4))
            for column in range(4)
        ]
        for row in range(4)
    ]


def _dh_transform(theta, distance, length, twist):
    cosine_theta = math.cos(theta)
    sine_theta = math.sin(theta)
    cosine_twist = math.cos(twist)
    sine_twist = math.sin(twist)
    return [
        [
            cosine_theta,
            -sine_theta * cosine_twist,
            sine_theta * sine_twist,
            length * cosine_theta,
        ],
        [
            sine_theta,
            cosine_theta * cosine_twist,
            -cosine_theta * sine_twist,
            length * sine_theta,
        ],
        [0.0, sine_twist, cosine_twist, distance],
        [0.0, 0.0, 0.0, 1.0],
    ]


def forward_kinematics(joints):
    """Return the suction-cup frame for six UR10e joint angles."""

    if len(joints) != 6:
        raise ValueError("UR10e kinematics require six joint angles")

    transform = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    for angle, (distance, length, twist) in zip(joints, DH_PARAMETERS):
        transform = _multiply(
            transform,
            _dh_transform(angle, distance, length, twist),
        )

    transform[0][3] += TOOL_OFFSET * transform[0][2]
    transform[1][3] += TOOL_OFFSET * transform[1][2]
    transform[2][3] += TOOL_OFFSET * transform[2][2]
    return transform


def _rotation_error(current, target):
    relative = [
        [
            sum(target[row][axis] * current[column][axis] for axis in range(3))
            for column in range(3)
        ]
        for row in range(3)
    ]
    cosine = max(
        -1.0,
        min(1.0, (relative[0][0] + relative[1][1] + relative[2][2] - 1.0) / 2.0),
    )
    angle = math.acos(cosine)
    if angle < 1e-8:
        return [0.0, 0.0, 0.0]

    scale = angle / (2.0 * math.sin(angle))
    return [
        scale * (relative[2][1] - relative[1][2]),
        scale * (relative[0][2] - relative[2][0]),
        scale * (relative[1][0] - relative[0][1]),
    ]


def _pose_error(current, target):
    return [
        target[row][3] - current[row][3]
        for row in range(3)
    ] + _rotation_error(current, target)


def _solve_linear(matrix, vector):
    size = len(vector)
    augmented = [
        list(matrix[row]) + [vector[row]]
        for row in range(size)
    ]
    for column in range(size):
        pivot = max(
            range(column, size),
            key=lambda row: abs(augmented[row][column]),
        )
        if abs(augmented[pivot][column]) < 1e-12:
            raise ValueError("Inverse-kinematics Jacobian is singular")
        augmented[column], augmented[pivot] = (
            augmented[pivot],
            augmented[column],
        )
        divisor = augmented[column][column]
        augmented[column] = [
            value / divisor for value in augmented[column]
        ]
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                value - factor * pivot_value
                for value, pivot_value in zip(
                    augmented[row],
                    augmented[column],
                )
            ]
    return [augmented[row][-1] for row in range(size)]


def inverse_kinematics(target, seed, max_iterations=400):
    """Solve for a target homogeneous transform in the UR10e base frame."""

    if len(seed) != 6:
        raise ValueError("UR10e kinematics require six seed joint angles")
    if len(target) != 4 or any(len(row) != 4 for row in target):
        raise ValueError("Target pose must be a 4-by-4 transform")

    initial = [float(angle) for angle in seed]
    home = [0.0, -1.0, 1.2, -1.8, -math.pi / 2.0, 0.0]
    seeds = [initial]
    if max(abs(angle - home_angle) for angle, home_angle in zip(initial, home)) > 0.1:
        seeds.append(home)

    for start in seeds:
        joints = list(start)
        damping = 0.04
        for _iteration in range(max_iterations):
            current = forward_kinematics(joints)
            error = _pose_error(current, target)
            position_error = math.sqrt(
                sum(value * value for value in error[:3])
            )
            rotation_error = math.sqrt(
                sum(value * value for value in error[3:])
            )
            if position_error <= 0.005 and rotation_error <= 0.035:
                return joints

            jacobian = [[0.0] * 6 for _row in range(6)]
            for joint_index in range(6):
                positive = list(joints)
                negative = list(joints)
                positive[joint_index] += _EPSILON
                negative[joint_index] -= _EPSILON
                positive_pose = forward_kinematics(positive)
                negative_pose = forward_kinematics(negative)
                for row in range(3):
                    jacobian[row][joint_index] = (
                        positive_pose[row][3] - negative_pose[row][3]
                    ) / (2.0 * _EPSILON)
                angular = _rotation_error(negative_pose, positive_pose)
                for row in range(3):
                    jacobian[row + 3][joint_index] = (
                        angular[row] / (2.0 * _EPSILON)
                    )

            normal = [
                [
                    sum(
                        jacobian[row][joint] * jacobian[column][joint]
                        for joint in range(6)
                    )
                    + (damping * damping if row == column else 0.0)
                    for column in range(6)
                ]
                for row in range(6)
            ]
            correction = _solve_linear(normal, error)
            delta = [
                sum(
                    jacobian[row][joint] * correction[row]
                    for row in range(6)
                )
                for joint in range(6)
            ]
            joints = [
                max(
                    -JOINT_LIMIT,
                    min(JOINT_LIMIT, angle + max(-0.2, min(0.2, change))),
                )
                for angle, change in zip(joints, delta)
            ]

    raise ValueError(
        "No UR10e solution found for the requested suction-cup pose"
    )
