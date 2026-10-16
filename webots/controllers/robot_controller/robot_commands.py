
"""Robot command validation and stop/recovery handling."""

import queue

from raas_logging import log_event


def handle_command(controller, command):
    log_event(
        controller.logger,
        "MQTT",
        "command.received",
        "Command received by robot controller",
        robot_id=controller.robot_id,
        command_type=(
            command.get("command") if isinstance(command, dict) else None
        ),
        command=command if isinstance(command, dict) else None,
    )

    if not isinstance(command, dict):
        log_event(
            controller.logger,
            "MQTT",
            "command.invalid",
            "Robot command must be a JSON object",
            level="ERROR",
            robot_id=controller.robot_id,
        )
        return

    command_type = command.get("command")

    if command_type == "PICK_AND_PLACE":
        _handle_pick_and_place_command(controller, command)
        return

    if command_type == "STOP_ROBOT":
        handle_stop_command(controller, command)
        return

    if command_type == "RECOVER_ROBOT":
        handle_recovery_command(controller, command)
        return

    if command_type != "MOVE_TO_NODE":
        log_event(
            controller.logger,
            "MQTT",
            "command.unsupported",
            "Unsupported robot command",
            level="WARNING",
            robot_id=controller.robot_id,
            command_type=command_type,
        )
        return

    order_id = command.get("orderId")
    node_id = command.get("nodeId")
    edge_id = command.get("edgeId")

    if not order_id:
        log_event(
            controller.logger,
            "MQTT",
            "command.missing_order",
            "Robot command missing orderId",
            level="WARNING",
            robot_id=controller.robot_id,
        )
        return

    if getattr(controller.adapter, "task_active", False):
        controller.last_error = (
            "Cannot start navigation while pick-and-place is active"
        )
        controller.publish_status()
        return

    if not node_id:
        log_event(
            controller.logger,
            "MQTT",
            "command.missing_node",
            "Robot command missing nodeId",
            level="WARNING",
            robot_id=controller.robot_id,
        )
        return

    if not edge_id:
        log_event(
            controller.logger,
            "MQTT",
            "command.missing_edge",
            "Robot command missing edgeId",
            level="WARNING",
            robot_id=controller.robot_id,
            order_id=order_id,
            node_id=node_id,
        )
        return

    if not controller.node_manager.exists(node_id):
        log_event(
            controller.logger,
            "ROBOTS",
            "navigation.invalid_node",
            "Robot navigation target node is unknown",
            level="ERROR",
            robot_id=controller.robot_id,
            order_id=order_id,
            node_id=node_id,
        )
        controller.state.order_id = order_id
        controller.state.set_error()
        controller.publish_status()
        return

    if "NAVIGATION" not in controller.adapter.capabilities:
        controller.last_error = (
            f"Navigation is not supported by {controller.robot_type}"
        )
        controller.state.order_id = order_id
        controller.state.set_error()
        controller.publish_status()
        return

    if controller.state.state == "EXECUTING":
        log_event(
            controller.logger,
            "ROBOTS",
            "navigation.busy",
            "Robot is already executing a navigation command",
            level="WARNING",
            robot_id=controller.robot_id,
            target_node=controller.state.target_node,
            order_id=controller.state.order_id,
        )
        return

    controller.last_error = None
    controller.state.start_navigation(order_id, node_id, edge_id)

    log_event(
        controller.logger,
        "ROBOTS",
        "navigation.started",
        "Navigation started",
        robot_id=controller.robot_id,
        order_id=order_id,
        target_node=node_id,
        edge_id=edge_id,
    )
    controller.publish_status()


def _handle_pick_and_place_command(controller, command):
    task_id = str(
        command.get("taskId") or command.get("orderId") or ""
    ).strip()

    if not task_id:
        controller.last_error = (
            "PICK_AND_PLACE requires taskId or orderId"
        )
        controller.publish_status()
        return

    if "PICK_AND_PLACE" not in controller.adapter.capabilities:
        controller.last_error = (
            f"Pick-and-place is not supported by {controller.robot_type}"
        )
        controller.publish_status()
        return

    if controller.state.state == "EXECUTING":
        controller.last_error = (
            "Cannot start pick-and-place while the robot is navigating"
        )
        controller.publish_status()
        return

    if getattr(controller.adapter, "task_active", False):
        controller.last_error = (
            "Robot already has an active manipulation task"
        )
        controller.publish_status()
        return

    if not hasattr(controller, "last_manipulation_task_id"):
        controller.last_manipulation_task_id = None

    active_status = getattr(
        controller.adapter, "manipulation_status", None
    )
    known_task_id = (
        active_status.get("taskId")
        if isinstance(active_status, dict)
        else None
    )

    # Prevent duplicate MQTT deliveries from repeating physical actions.
    if task_id in {
        controller.last_manipulation_task_id,
        known_task_id,
    }:
        controller.last_error = None
        controller.publish_status()
        return

    commands = getattr(controller, "manipulation_commands", None)

    if commands is None:
        controller.last_error = (
            "Robot controller has no manipulation command queue"
        )
        controller.publish_status()
        return

    if not commands.empty():
        controller.last_error = (
            "A manipulation command is already queued"
        )
        controller.publish_status()
        return

    controller.last_manipulation_task_id = task_id
    commands.put(command)
    controller.last_error = None
    controller.publish_status()


def handle_stop_command(controller, command):
    command_robot_id = command.get("robotId")

    if command_robot_id and command_robot_id != controller.robot_id:
        log_event(
            controller.logger,
            "ROBOTS",
            "stop.ignored",
            "Stop command ignored for another robot",
            level="WARNING",
            robot_id=controller.robot_id,
            command_robot_id=command_robot_id,
        )
        return

    log_event(
        controller.logger,
        "ROBOTS",
        "stop.requested",
        "Stop command received",
        robot_id=controller.robot_id,
        current_state=controller.state.state,
        order_id=controller.state.order_id,
        current_node=controller.state.current_node,
        target_node=controller.state.target_node,
    )

    discard_queued_manipulation_commands(controller)

    try:
        controller.adapter.stop()
    except Exception as error:
        controller.last_error = f"Failed to stop robot safely: {error}"
        controller.state.set_error()

        log_event(
            controller.logger,
            "ROBOTS",
            "stop.failed",
            "Robot stop failed",
            level="ERROR",
            robot_id=controller.robot_id,
            error=controller.last_error,
        )
        controller.publish_status()
        return

    controller.state.stop()

    log_event(
        controller.logger,
        "ROBOTS",
        "stop.completed",
        "Robot stopped",
        robot_id=controller.robot_id,
        order_id=controller.state.order_id,
    )
    controller.publish_status()


def handle_recovery_command(controller, command):
    command_robot_id = command.get("robotId")
    order_id = command.get("orderId")

    if command_robot_id and command_robot_id != controller.robot_id:
        return

    if not order_id:
        log_event(
            controller.logger,
            "RECOVERY",
            "recovery.rejected",
            "Recovery rejected because orderId is missing",
            level="WARNING",
            robot_id=controller.robot_id,
        )
        return

    if controller.state.order_id and controller.state.order_id != order_id:
        log_event(
            controller.logger,
            "RECOVERY",
            "recovery.rejected",
            "Recovery rejected because active order does not match request",
            level="WARNING",
            robot_id=controller.robot_id,
            active_order=controller.state.order_id,
            requested_order=order_id,
        )
        return

    discard_queued_manipulation_commands(controller)

    try:
        controller.adapter.stop()
    except Exception as error:
        controller.last_error = f"Robot recovery stop failed: {error}"
        controller.state.set_error()
        controller.publish_status()
        return

    controller.state.clear_order()
    controller.last_error = None
    controller.publish_status(recovered_order_id=order_id)


def discard_queued_manipulation_commands(controller):
    commands = getattr(controller, "manipulation_commands", None)

    if commands is None:
        return

    while True:
        try:
            commands.get_nowait()
        except queue.Empty:
            return
