"""Apply robot telemetry to Fleet mission and registry state."""


def handle_robot_status(controller, status: dict) -> None:
    logger = controller.logger
    log = controller.log
    state_lock = controller.state_lock
    missions = controller.missions
    robot_registry = controller.robot_registry
    publish_robot_registry = controller.publish_robot_registry
    publish_state = controller.publish_state
    send_robot_command = controller.send_robot_command
    separator = controller.separator

    robot_id = str(
        status.get("robotId")
        or status.get("serialNumber")
        or ""
    ).strip()
    manufacturer = str(status.get("manufacturer") or "UNKNOWN").strip()
    robot_key = (manufacturer, robot_id)
    state = status.get("state")
    order_id = status.get("orderId")
    node_id = status.get("nodeId")
    edge_id = status.get("edgeId")

    if not robot_id:
        logger.warning("Ignoring robot status without a robot ID")
        return

    previous_robot = robot_registry.get(manufacturer, robot_id)
    if not robot_registry.update_status(status):
        logger.debug(
            "Ignoring duplicate, delayed, or mismatched robot status "
            "for %s/%s order=%r",
            manufacturer,
            robot_id,
            order_id,
        )
        return
    robot_registry.discover_world(status.get("worldRobots", []))

    status_changed = (
        previous_robot is None
        or previous_robot.get("status") != state
        or previous_robot.get("currentOrder") != order_id
        or (
            node_id is not None
            and previous_robot.get("currentNode") != node_id
        )
        or (
            edge_id is not None
            and previous_robot.get("currentEdge") != edge_id
        )
    )
    if status_changed:
        log(
            f"Robot status changed: {manufacturer}/{robot_id} "
            f"state={state} order={order_id or '-'} "
            f"node={node_id or '-'} edge={edge_id or '-'}"
        )

    if state == "IDLE":
        recovered_order_id = status.get("recoveredOrderId")
        if recovered_order_id:
            with state_lock:
                current = missions.get(robot_key)
                if current is not None and current.order_id == recovered_order_id:
                    current.state = "STOPPED"
        publish_robot_registry()
        return

    with state_lock:
        previous_mission = missions.get(robot_key)
        completed_mission_heartbeat = (
            state == "NODE_REACHED"
            and previous_mission is not None
            and previous_mission.order_id == order_id
            and previous_mission.state == "COMPLETED"
            and previous_mission.last_node_id == node_id
        )

    if completed_mission_heartbeat:
        robot_registry.set_state(
            manufacturer,
            robot_id,
            "IDLE",
            current_node=node_id,
        )
        publish_robot_registry()
        return

    with state_lock:
        previous_mission = missions.get(robot_key)
        repeated_terminal_status = (
            state in {"STOPPED", "ERROR"}
            and previous_mission is not None
            and previous_mission.order_id == order_id
            and previous_mission.state == state
        )

    if repeated_terminal_status:
        publish_robot_registry()
        return

    with state_lock:
        current = missions.get(robot_key)
        if current is None or order_id != current.order_id:
            current = None

        if current is not None:
            nodes = current.nodes
            edges = current.edges

        if current is None:
            unmatched_status = True
        else:
            unmatched_status = False

        if unmatched_status:
            pass
        elif state == "EXECUTING":
            current.state = "EXECUTING"
        elif state == "NODE_REACHED":
            expected_index = current.node_index + 1
            expected_edge_index = current.edge_index
            if (
                expected_index >= len(nodes)
                or expected_edge_index >= len(edges)
                or nodes[expected_index]["nodeId"] != node_id
                or edges[expected_edge_index]["edgeId"] != edge_id
            ):
                log(
                    "Ignoring unexpected NODE_REACHED status "
                    f"(node={node_id}, edge={edge_id}); "
                    "it does not match the next mission segment."
                )
                invalid_reached_status = True
            else:
                invalid_reached_status = False

            if invalid_reached_status:
                mission_completed = False
                mission_failed = False
                mission_stopped = False
                next_segment = False
            else:
                reached_index = expected_index
                current.node_index = reached_index
                current.last_node_id = node_id
                current.last_node_sequence_id = nodes[
                    reached_index
                ]["sequenceId"]

                mission_completed = reached_index == len(nodes) - 1
                mission_failed = False
                mission_stopped = False
                next_segment = not mission_completed

                if mission_completed:
                    current.state = "COMPLETED"
                    current.current_edge_id = None
                    current.current_edge_sequence_id = 0
                    order_id = current.order_id
                    final_node = current.last_node_id
                    total_nodes = len(nodes)
                    total_edges = len(edges)
                else:
                    current.edge_index = reached_index
                    next_edge = edges[current.edge_index]
                    next_node = nodes[reached_index + 1]
                    current.current_edge_id = next_edge["edgeId"]
                    current.current_edge_sequence_id = next_edge["sequenceId"]
                    current.state = "EXECUTING"
                    start_node = nodes[current.node_index]
                    order_id = current.order_id
        elif state == "STOPPED":
            current.state = "STOPPED"
            current.current_edge_id = edge_id
            current.current_edge_sequence_id = 0
            mission_stopped = True
            mission_completed = False
            mission_failed = False
        elif state == "ERROR":
            current.state = "ERROR"
            mission_failed = True
            mission_completed = False
            mission_stopped = False
        else:
            log(f"Ignoring unsupported robot state: {state}")
            return

    if unmatched_status:
        log("Robot status updated; no matching active mission.")
        if state in {"EXECUTING", "NODE_REACHED", "DISPATCHED"}:
            robot_registry.mark_recovery_required(
                manufacturer,
                robot_id,
                "Robot reports an active order unknown to this Fleet session",
            )
        publish_robot_registry()
        return

    if state == "EXECUTING":
        robot_registry.set_state(
            manufacturer,
            robot_id,
            "EXECUTING",
            current_order=order_id,
            current_node=status.get("currentNode") or node_id,
            current_edge=edge_id,
        )
        publish_robot_registry()
        log("ROBOT IS EXECUTING MISSION")
        return

    if state == "NODE_REACHED":
        if invalid_reached_status:
            publish_robot_registry()
            return

        if mission_completed:
            separator()
            log("MISSION COMPLETED")
            separator()
            log(f"Order ID   : {order_id}")
            log(f"Final node : {final_node}")
            log(f"Total nodes: {total_nodes}")
            log(f"Total edges: {total_edges}")
            separator()
            robot_registry.set_state(
                manufacturer,
                robot_id,
                "IDLE",
                current_node=final_node,
            )
            publish_state(robot_key)
            publish_robot_registry()
            return

        robot_registry.set_state(
            manufacturer,
            robot_id,
            "DISPATCHED",
            current_order=order_id,
            current_node=node_id,
            current_edge=next_edge["edgeId"],
        )
        publish_state(robot_key)
        publish_robot_registry()
        send_robot_command(
            order_id,
            start_node,
            next_node,
            next_edge,
            manufacturer,
            robot_id,
        )
        return

    if state == "STOPPED":
        robot_registry.set_state(
            manufacturer,
            robot_id,
            "STOPPED",
            current_order=order_id,
            current_node=node_id,
            current_edge=edge_id,
        )
        publish_robot_registry()
        publish_state(robot_key)
        return

    if state == "ERROR":
        robot_registry.set_state(
            manufacturer,
            robot_id,
            "ERROR",
            current_order=order_id,
            current_node=node_id,
            current_edge=edge_id,
            error=status.get("error"),
        )
        publish_robot_registry()
        publish_state(robot_key)
        return
