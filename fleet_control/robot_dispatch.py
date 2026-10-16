"""Dispatch navigation commands from Fleet to registered robots."""

import json

from raas_logging import log_event


def send_robot_command(
    controller,
    order_id: str,
    start_node: dict,
    target_node: dict,
    edge: dict,
    manufacturer: str | None = None,
    serial_number: str | None = None,
) -> bool:
    manufacturer = manufacturer or controller.MANUFACTURER
    serial_number = serial_number or controller.SERIAL_NUMBER
    command_topic_name = controller.robot_command_topic(
        manufacturer, serial_number
    )
    command = {
        "command": "MOVE_TO_NODE",
        "orderId": order_id,
        "nodeId": target_node["nodeId"],
        "edgeId": edge["edgeId"],
    }
    payload = json.dumps(command)

    log_event(
        controller.logger,
        "MQTT",
        "robot_command.publish",
        "Sending robot command",
        topic=command_topic_name,
        order_id=order_id,
        start_node=start_node["nodeId"],
        target_node=target_node["nodeId"],
        edge_id=edge["edgeId"],
    )
    result = controller.mqtt_client.publish(
        command_topic_name,
        payload,
        qos=0,
    )
    if result.rc != controller.mqtt.MQTT_ERR_SUCCESS:
        log_event(
            controller.logger,
            "MQTT",
            "robot_command.publish_failed",
            "Failed to send robot command",
            level="ERROR",
            topic=command_topic_name,
            order_id=order_id,
            rc=result.rc,
        )
        return False

    log_event(
        controller.logger,
        "MQTT",
        "robot_command.sent",
        "Robot command sent",
        topic=command_topic_name,
        order_id=order_id,
    )
    return True
