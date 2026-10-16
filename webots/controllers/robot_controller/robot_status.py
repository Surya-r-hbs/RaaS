"""Build and publish robot telemetry for the fleet."""

from raas_logging import log_event


def publish_status(
    controller, recovered_order_id=None, *, simulation_mode_enabled
):
    controller.status_sequence += 1

    state = controller.state.state
    pose_error = None
    try:
        position = controller.adapter.get_position()
        orientation = controller.adapter.get_orientation()
    except RuntimeError as error:
        position = None
        orientation = None
        pose_error = f"Webots world discovery is unavailable: {error}"

    nearest_node = None
    nearest_node_distance = None
    if position is not None:
        try:
            nearest_node, nearest_node_distance = (
                controller.node_manager.get_nearest_node(position)
            )
        except RuntimeError:
            pass
    current_node = (
        nearest_node
        if nearest_node_distance is not None
        and nearest_node_distance <= 0.75
        else None
    )

    message = {
        "robotId": controller.robot_id,
        "manufacturer": controller.manufacturer,
        "robotType": controller.robot_type,
        "capabilities": controller.adapter.capability_names,
        "availableDevices": controller.adapter.available_devices,
        "transportCapability": controller.adapter.transport_capability,
        "runtimeMode": "WEBOTS_SIMULATION",
        "simulationModeEnabled": simulation_mode_enabled,
        "state": state,
        "status": state,
        "orderId": controller.state.order_id,
        "currentNode": (
            controller.state.current_node
            if state == "NODE_REACHED"
            else current_node
        ),
        "nearestNode": nearest_node,
        "nearestNodeDistance": nearest_node_distance,
        "position": position,
        "orientation": orientation,
        "worldRobots": controller.world_robots,
        "error": controller.last_error or pose_error,
        "agentSessionId": controller.agent_session_id,
        "statusSequence": controller.status_sequence,
        "nodeId": (
            controller.state.current_node
            if state in {"NODE_REACHED", "STOPPED"}
            else controller.state.target_node
        ),
        "edgeId": (
            controller.state.current_edge
            if state in {"NODE_REACHED", "STOPPED"}
            else controller.state.target_edge
        ),
    }
    if recovered_order_id:
        message["recoveredOrderId"] = recovered_order_id
    manipulation = getattr(controller.adapter, "manipulation_status", None)
    if manipulation is not None:
        message["manipulation"] = manipulation

    log_event(
        controller.logger,
        "ROBOTS",
        "status.published",
        "Robot status published",
        robot_id=controller.robot_id,
        state=state,
        order_id=controller.state.order_id,
        current_node=current_node,
        target_node=controller.state.target_node,
        edge_id=controller.state.target_edge,
    )
    controller.mqtt.publish_status(message)
