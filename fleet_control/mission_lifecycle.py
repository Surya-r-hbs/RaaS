"""Validate incoming VDA-5050 orders and advance Fleet missions."""


def start_next_segment(controller, robot_key: tuple[str, str]) -> None:
    state_lock = controller.state_lock
    missions = controller.missions
    log = controller.log
    separator = controller.separator
    robot_registry = controller.robot_registry
    publish_state = controller.publish_state
    publish_robot_registry = controller.publish_robot_registry
    send_robot_command = controller.send_robot_command

    with state_lock:
        current = missions.get(robot_key)
        if current is None:
            return

        if current.state == "STOPPED":
            log("MISSION IS STOPPED - NO NEXT SEGMENT WILL BE SENT")
            return

        if current.node_index >= len(current.nodes) - 1:
            current.state = "COMPLETED"
            current.current_edge_id = None
            current.current_edge_sequence_id = 0
            final_node = current.last_node_id
            order_id = current.order_id
            total_nodes = len(current.nodes)
            total_edges = len(current.edges)
            should_send = False
        else:
            edge = current.edges[current.edge_index]
            start_node = current.nodes[current.node_index]
            target_node = current.nodes[current.node_index + 1]
            current.current_edge_id = edge["edgeId"]
            current.current_edge_sequence_id = edge["sequenceId"]
            current.state = "EXECUTING"
            order_id = current.order_id
            should_send = True

    if not should_send:
        log("MISSION COMPLETED")
        log(f"Order ID   : {order_id}")
        log(f"Final node : {final_node}")
        log(f"Total nodes: {total_nodes}")
        log(f"Total edges: {total_edges}")
        robot_registry.set_state(
            robot_key[0],
            robot_key[1],
            "IDLE",
            current_node=final_node,
        )
        publish_state(robot_key)
        publish_robot_registry()
        return

    robot_registry.set_state(
        robot_key[0],
        robot_key[1],
        "DISPATCHED",
        current_order=order_id,
        current_node=start_node["nodeId"],
        current_edge=edge["edgeId"],
    )
    publish_state(robot_key)
    publish_robot_registry()

    command_sent = send_robot_command(
        order_id,
        start_node,
        target_node,
        edge,
        robot_key[0],
        robot_key[1],
    )
    if not command_sent:
        with state_lock:
            current = missions.get(robot_key)
            if current is not None and current.order_id == order_id:
                current.state = "ERROR"
        robot_registry.set_state(
            robot_key[0],
            robot_key[1],
            "DISPATCHED",
            current_order=order_id,
            current_node=start_node["nodeId"],
            current_edge=edge["edgeId"],
        )
        robot_registry.mark_recovery_required(
            robot_key[0],
            robot_key[1],
            "Robot command delivery failed; verify the robot before recovery",
        )
        publish_robot_registry()


def handle_order(controller, order: dict) -> None:
    state_lock = controller.state_lock
    missions = controller.missions
    validate_order = controller.validate_order
    Mission = controller.Mission
    MANUFACTURER = controller.MANUFACTURER
    SERIAL_NUMBER = controller.SERIAL_NUMBER
    robot_registry = controller.robot_registry
    log = controller.log
    separator = controller.separator
    start_next_segment = controller.start_next_segment

    manufacturer = order.get("manufacturer", MANUFACTURER)
    serial_number = order.get("serialNumber", SERIAL_NUMBER)
    robot_key = (manufacturer, serial_number)
    valid, reason = validate_order(order, manufacturer, serial_number)

    log("VALIDATING VDA-5050 ORDER")
    log(reason)
    if not valid:
        log("ORDER REJECTED")
        return

    registered_robot = robot_registry.get(manufacturer, serial_number)
    if registered_robot is None:
        log(
            f"ORDER REJECTED: robot {manufacturer}/{serial_number} "
            "has not registered with the fleet"
        )
        return
    if not registered_robot["available"]:
        log(
            f"ORDER REJECTED: robot {manufacturer}/{serial_number} "
            f"is unavailable ({registered_robot['availabilityReason']})"
        )
        return

    with state_lock:
        current_mission = missions.get(robot_key)
        if (
            current_mission is not None
            and current_mission.state in {"EXECUTING", "PAUSED"}
            and current_mission.order_id != order["orderId"]
        ):
            log(
                f"ORDER REJECTED: robot is already "
                f"executing {current_mission.order_id}"
            )
            return

        if (
            current_mission is not None
            and current_mission.order_id == order["orderId"]
        ):
            log(f"Duplicate order ignored: {order['orderId']}")
            return

        first_node = order["nodes"][0]
        first_edge = order["edges"][0]
        missions[robot_key] = Mission(
            order=order,
            node_index=0,
            edge_index=0,
            last_node_id=first_node["nodeId"],
            last_node_sequence_id=first_node["sequenceId"],
            current_edge_id=first_edge["edgeId"],
            current_edge_sequence_id=first_edge["sequenceId"],
            state="EXECUTING",
            header_id=0,
        )

    log("MISSION INITIALIZED")
    log(f"Order ID    : {order['orderId']}")
    log(f"Start node  : {first_node['nodeId']}")
    log(f"Target node : {order['nodes'][1]['nodeId']}")
    log(f"First edge  : {first_edge['edgeId']}")
    start_next_segment(robot_key)
