"""Decode and route MQTT messages received by the fleet controller."""

import json


def route_message(
    message,
    *,
    objects_raw_topic,
    robot_registry,
    object_registry,
    annotate_load_verification,
    publish_robot_registry,
    publish_object_registry,
    handle_order,
    handle_robot_status,
    log,
    logger,
):
    raw_payload = message.payload.decode("utf-8", errors="replace")
    logger.debug("MQTT message received: topic=%s payload=%s", message.topic, raw_payload)

    try:
        payload = json.loads(raw_payload)
    except json.JSONDecodeError as error:
        log(f"ERROR: Invalid JSON: {error}")
        return

    if message.topic == "robot/discovery":
        if payload.get("discoveryType") == "WORLD_SNAPSHOT":
            robot_registry.discover_world(payload.get("worldRobots", []))
            publish_robot_registry()
            return
        if payload.get("worldRobots"):
            robot_registry.discover_world(payload["worldRobots"])
        if not robot_registry.register_agent(payload):
            log("Ignoring robot discovery without a robot ID")
            return
        log(
            f"Robot registered: {payload.get('robotId')} "
            f"({payload.get('manufacturer')}, "
            f"{payload.get('robotType', 'Unknown')})"
        )
        publish_robot_registry()
        return

    if message.topic == objects_raw_topic:
        try:
            payload = annotate_load_verification(
                payload,
                robot_registry.snapshot(),
            )
            object_registry.update(payload)
        except ValueError as error:
            log(f"Invalid Webots object inventory: {error}")
            return
        publish_object_registry()
        return

    if message.topic.endswith("/order"):
        if message.retain:
            log("Ignoring retained VDA-5050 order; it cannot be safely replayed")
            return
        handle_order(payload)
    elif message.topic.endswith("/status"):
        handle_robot_status(payload)
