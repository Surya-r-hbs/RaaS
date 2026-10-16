"""Publish mission state messages using the VDA-5050 state schema."""

import json

from raas_logging import log_event


def publish_state(controller, robot_key: tuple[str, str]) -> None:
    with controller.state_lock:
        current = controller.missions.get(robot_key)
        if current is None:
            return

        nodes = current.nodes
        edges = current.edges
        node_states = []
        edge_states = []

        if 0 <= current.node_index < len(nodes):
            node = nodes[current.node_index]
            node_states.append(
                {
                    "nodeId": node["nodeId"],
                    "sequenceId": node["sequenceId"],
                    "released": node.get("released", True),
                }
            )

        if (
            current.state not in {"COMPLETED", "STOPPED"}
            and 0 <= current.edge_index < len(edges)
        ):
            edge = edges[current.edge_index]
            edge_states.append(
                {
                    "edgeId": edge["edgeId"],
                    "sequenceId": edge["sequenceId"],
                    "released": edge.get("released", True),
                }
            )

        manufacturer = current.order.get(
            "manufacturer", controller.MANUFACTURER
        )
        serial_number = current.order.get(
            "serialNumber", controller.SERIAL_NUMBER
        )
        payload = {
            "headerId": current.header_id,
            "timestamp": controller.now_utc(),
            "version": current.order.get("version", controller.VDA_VERSION),
            "manufacturer": manufacturer,
            "serialNumber": serial_number,
            "orderId": current.order_id,
            "orderUpdateId": current.order.get("orderUpdateId", 0),
            "lastNodeId": current.last_node_id,
            "lastNodeSequenceId": current.last_node_sequence_id,
            "nodeStates": node_states,
            "edgeStates": edge_states,
        }
        current.header_id += 1

        mission_state = current.state
        current_edge_id = current.current_edge_id
        state_topic_name = controller.state_topic(manufacturer, serial_number)

    result = controller.mqtt_client.publish(
        state_topic_name,
        json.dumps(payload),
        qos=0,
    )
    if result.rc == controller.mqtt.MQTT_ERR_SUCCESS:
        log_event(
            controller.logger,
            "MQTT",
            "vda_state.published",
            "VDA-5050 state published",
            topic=state_topic_name,
            order_id=payload["orderId"],
            mission_state=mission_state,
            last_node_id=payload["lastNodeId"],
            current_edge_id=current_edge_id,
        )
    else:
        log_event(
            controller.logger,
            "MQTT",
            "vda_state.publish_failed",
            "Failed to publish VDA-5050 state",
            level="ERROR",
            topic=state_topic_name,
            order_id=payload["orderId"],
            rc=result.rc,
        )
