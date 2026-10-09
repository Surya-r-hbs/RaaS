"""Shared lifecycle and device discovery for Webots robot adapters."""


class UnsupportedRobotOperation(RuntimeError):
    """Raised when a robot adapter does not implement a requested operation."""


class BaseRobot:
    def __init__(self, robot, node_manager, robot_type):
        self.robot = robot
        self.node_manager = node_manager
        self.robot_type = robot_type
        self.devices = self._discover_devices()
        self.capabilities = set()
        self._motors = {
            name: device
            for name, device in self.devices.items()
            if callable(getattr(device, "setVelocity", None))
        }

    def _discover_devices(self):
        devices = {}
        for index in range(self.robot.getNumberOfDevices()):
            device = self.robot.getDeviceByIndex(index)
            devices[device.getName()] = device
        return devices

    def require_device(self, name):
        device = self.devices.get(name)
        if device is None:
            raise RuntimeError(
                f"{self.robot_type} adapter requires Webots device {name!r}; "
                f"available devices: {', '.join(sorted(self.devices)) or 'none'}"
            )
        return device

    @property
    def available_devices(self):
        return sorted(self.devices)

    @property
    def capability_names(self):
        return sorted(self.capabilities)

    def get_position(self):
        position = self.robot.getSelf().getPosition()
        return {
            "x": float(position[0]),
            "y": float(position[1]),
            "z": float(position[2]),
        }

    def move_to_node(self, node_id):
        raise UnsupportedRobotOperation(
            f"Navigation is not implemented for {self.robot_type}"
        )

    def stop(self):
        failures = []
        for name, motor in self._motors.items():
            try:
                motor.setVelocity(0.0)
            except Exception as error:
                failures.append(f"{name}: {error}")
        if failures:
            raise RuntimeError(
                "Could not stop all robot motors: " + "; ".join(failures)
            )

    def update(self):
        return None

    def execute_action(self, action):
        raise UnsupportedRobotOperation(
            f"Action execution is not implemented for {self.robot_type}: "
            f"{action}"
        )
