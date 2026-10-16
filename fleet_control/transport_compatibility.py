"""Payload, geometry, and carrier compatibility checks for transport tasks."""

import math


def number(value):
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def check_compatibility(
    object_record,
    robot,
    simulation_transport_enabled,
):
    capability = robot.get("transportCapability")
    if not isinstance(capability, dict):
        return "Carrier transport capability is unverified"
    capability_mode = capability.get("mode")
    verified_profile = (
        capability.get("status") == "VERIFIED"
        and capability.get("canCarryPreloaded") is True
        and capability.get("stabilityVerified") is True
    )
    simulation_profile = (
        capability.get("status") == "SIMULATION_ONLY"
        and capability_mode == "SIMULATION_ONLY"
        and simulation_transport_enabled
        and robot.get("runtimeMode") == "WEBOTS_SIMULATION"
        and robot.get("simulationModeEnabled") is True
        and capability.get("canCarryPreloaded") is True
        and capability.get("stabilityMode") == "SIMULATION_ASSUMED"
        and isinstance(capability.get("stabilityAssumption"), str)
        and bool(capability["stabilityAssumption"].strip())
    )
    if not (verified_profile or simulation_profile):
        reason = capability.get("reason")
        if isinstance(reason, str) and reason.strip():
            return f"Carrier transport capability is unverified: {reason.strip()}"
        if capability.get("status") == "SIMULATION_ONLY":
            if not simulation_transport_enabled:
                return (
                    "Simulation-only transport profile is disabled; set "
                    "RAAS_WEBOTS_SIMULATION_MODE=1 in both Webots and Fleet"
                )
            if (
                robot.get("runtimeMode") != "WEBOTS_SIMULATION"
                or robot.get("simulationModeEnabled") is not True
            ):
                return (
                    "Simulation-only transport is restricted to a Webots "
                    "robot with simulation mode explicitly enabled"
                )
            return "Simulation-only carrier capability profile is incomplete"
        return (
            "Carrier carrying or stability capability is unverified "
            "or unsupported"
        )
    if "NAVIGATION" not in robot.get("capabilities", []):
        return "Carrier navigation capability is not implemented"

    max_payload = number(capability.get("maxPayloadKg"))
    object_mass = number(object_record.get("mass_kg"))
    platform = capability.get("platformDimensions")
    geometry = object_record.get("geometry")
    bounds = geometry.get("bounds") if isinstance(geometry, dict) else None
    rotation = object_record.get("rotation")
    if (
        max_payload is None
        or object_mass is None
        or not isinstance(platform, dict)
        or not isinstance(bounds, dict)
        or not isinstance(bounds.get("min"), dict)
        or not isinstance(bounds.get("max"), dict)
        or not isinstance(rotation, list)
        or len(rotation) != 9
        or any(number(value) is None for value in rotation)
    ):
        return (
            "Carrier payload, platform dimensions, object geometry, or orientation "
            "are unknown"
        )
    rotation = [number(value) for value in rotation]
    platform_x = number(platform.get("x"))
    platform_y = number(platform.get("y"))
    if platform_x is None or platform_y is None:
        return "Carrier platform dimensions are unknown"
    minimum = {
        axis: number(bounds["min"].get(axis))
        for axis in ("x", "y", "z")
    }
    maximum = {
        axis: number(bounds["max"].get(axis))
        for axis in ("x", "y", "z")
    }
    if any(
        minimum[axis] is None
        or maximum[axis] is None
        or minimum[axis] >= maximum[axis]
        for axis in ("x", "y", "z")
    ):
        return "Object geometry bounds are invalid"
    robot_rotation = robot.get("orientation")
    if (
        not isinstance(robot_rotation, (list, tuple))
        or len(robot_rotation) != 9
        or any(number(value) is None for value in robot_rotation)
    ):
        return (
            "Carrier orientation is unavailable for object footprint "
            "validation"
        )
    relative_rotation = [
        sum(
            number(robot_rotation[k * 3 + row])
            * number(rotation[k * 3 + column])
            for k in range(3)
        )
        for row in range(3)
        for column in range(3)
    ]
    points = [
        (x, y, z)
        for x in (minimum["x"], maximum["x"])
        for y in (minimum["y"], maximum["y"])
        for z in (minimum["z"], maximum["z"])
    ]
    transformed = [
        (
            relative_rotation[0] * x
            + relative_rotation[1] * y
            + relative_rotation[2] * z,
            relative_rotation[3] * x
            + relative_rotation[4] * y
            + relative_rotation[5] * z,
        )
        for x, y, z in points
    ]
    object_x = max(point[0] for point in transformed) - min(
        point[0] for point in transformed
    )
    object_y = max(point[1] for point in transformed) - min(
        point[1] for point in transformed
    )
    if object_mass <= 0:
        return "Object mass must be positive for transport"
    if max_payload <= 0 or platform_x <= 0 or platform_y <= 0:
        return "Configured carrier payload/platform limits must be positive"
    if object_mass > max_payload:
        return (
            f"Object mass {object_mass:g} kg exceeds the configured "
            f"{max_payload:g} kg carrier payload limit"
        )
    if object_x > platform_x or object_y > platform_y:
        return "Object footprint exceeds the carrier support platform"
    return None
