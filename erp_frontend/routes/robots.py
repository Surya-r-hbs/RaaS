from flask import Blueprint
import app as erp

bp = Blueprint("robots", __name__)


@bp.post(
    "/api/robots/<manufacturer>/<robot_id>/stop"
)
def stop_robot(
    manufacturer,
    robot_id
):

    with erp.data_lock:
        robot_entry = erp.robots.get((manufacturer, robot_id))

    if robot_entry is None:
        return erp.jsonify({"error": "Unknown robot"}), 404

    with erp.data_lock:
        current_status = robot_entry["status"]
        current_order_id = robot_entry["currentOrder"]
        current_node = robot_entry["currentNode"]
        current_edge = robot_entry["currentEdge"]

    # --------------------
    # IDEMPOTENT STOP
    # --------------------

    if current_status == "STOPPED":

        erp.add_event(
            "ROBOT_STOP",
            f"Robot {manufacturer}/{robot_id} "
            "is already stopped",
        )

        return erp.jsonify(
            {
                "robotId": robot_id,
                "status": "STOPPED",
                "orderId": current_order_id,
                "nodeId": current_node,
                "edgeId": current_edge,
                "message":
                    "Robot is already stopped",
            }
        )

    # --------------------
    # CHECK ACTIVE STATE
    # --------------------

    if current_status not in {"EXECUTING", "DISPATCHED"}:

        return erp.jsonify(
            {
                "error":
                    f"Robot cannot be stopped "
                    f"from state "
                    f"{current_status}"
            }
        ), 409

    # --------------------
    # STOP COMMAND
    # --------------------

    command = {
        "command": "STOP_ROBOT",
        "robotId": robot_id,
        "reason": "USER_REQUEST",
    }

    result = erp.mqtt_client.publish(
        f"robot/{robot_entry['manufacturer']}/{robot_id}/command",
        erp.json.dumps(command),
        qos=0,
    )

    if (
        result.rc
        != erp.mqtt.MQTT_ERR_SUCCESS
    ):

        erp.add_event(
            "ROBOT_STOP_ERROR",
            f"Failed to send stop command "
            f"to {manufacturer}/{robot_id}",
        )

        return erp.jsonify(
            {
                "error":
                    "Failed to send "
                    "STOP_ROBOT command"
            }
        ), 503

    # --------------------
    # OPTIMISTIC ERP STATE
    # --------------------

    with erp.data_lock:

        robot_entry["status"] = "STOPPING"
        robot_entry["lastUpdate"] = erp.now_utc()
        robot_entry["recoveryRequired"] = bool(current_order_id)
        robot_entry["recoveryReason"] = (
            "Operator requested a stop; recovery is required to resume"
            if current_order_id
            else "Robot was stopped by operator request"
        )
        robot_entry["available"] = False

        if current_order_id:
            order = erp.orders.get(current_order_id)
            if order is not None:
                order["status"] = "STOPPING"
                order["recoveryReason"] = robot_entry["recoveryReason"]

    erp.save_orders()

    erp.add_event(
        "ROBOT_STOP_REQUESTED",
        f"Stop requested for "
        f"Robot {manufacturer}/{robot_id}",
        command,
    )

    return erp.jsonify(
        {
            "robotId": robot_id,
            "status": "STOPPING",
            "orderId": current_order_id,
            "nodeId": current_node,
            "edgeId": current_edge,
            "message":
                "Stop command sent",
        }
    )

@bp.post("/api/robots/<manufacturer>/<robot_id>/recover")
def recover_robot(manufacturer, robot_id):
    payload = erp.request.get_json(silent=True) or {}
    reason = str(payload.get("reason") or "").strip()
    confirm_safe = payload.get("confirmSafe") is True
    robot_key = (manufacturer, robot_id)

    if not confirm_safe or not reason:
        return erp.jsonify(
            {"error": "Confirm the robot is safe to recover and provide a reason"}
        ), 400

    with erp.data_lock:
        robot = erp.robots.get(robot_key)
        if robot is None:
            return erp.jsonify({"error": "Unknown robot"}), 404
        if not robot.get("agentOnline") or not erp.robot_status_is_fresh(robot):
            return erp.jsonify(
                {
                    "error": (
                        "Robot is disconnected or its status is stale. "
                        "Assignment was not cleared."
                    )
                }
            ), 409
        if not robot.get("recoveryRequired"):
            return erp.jsonify(
                {"error": "Robot does not currently require recovery"}
            ), 409

        order_id = robot.get("currentOrder") or robot.get("lastOrderId")
        if not order_id:
            return erp.jsonify(
                {"error": "No recoverable order is associated with this robot"}
            ), 409
        order = erp.orders.get(order_id)
        if order is None or (
            order.get("manufacturer"), order.get("serialNumber")
        ) != robot_key:
            return erp.jsonify(
                {
                    "error": (
                        "The assigned order is not present in order history; "
                        "assignment was not cleared."
                    )
                }
            ), 409
        if (
            robot_key in erp.recovery_requests
            and erp.recovery_requests[robot_key] != order_id
        ):
            return erp.jsonify(
                {"error": "A different order is already being recovered"}
            ), 409

        command = {
            "command": "RECOVER_ROBOT",
            "robotId": robot_id,
            "orderId": order_id,
            "reason": reason,
        }
        first_request = robot_key not in erp.recovery_requests
        erp.recovery_requests[robot_key] = order_id
        robot["recoveryPending"] = True
        robot["availabilityReason"] = (
            "Waiting for robot recovery acknowledgement"
        )
        robot["available"] = False
        if first_request:
            order.setdefault("recoveryHistory", []).append(
                {
                    "timestamp": erp.now_utc(),
                    "robotId": robot_id,
                    "manufacturer": manufacturer,
                    "orderId": order_id,
                    "reason": reason,
                    "result": "REQUESTED",
                }
            )

    try:
        erp.save_orders()
        result = erp.mqtt_client.publish(
            f"robot/{manufacturer}/{robot_id}/command",
            erp.json.dumps(command),
            qos=1,
        )
    except Exception as error:
        with erp.data_lock:
            erp.recovery_requests.pop(robot_key, None)
            robot = erp.robots.get(robot_key)
            if robot is not None:
                robot["recoveryPending"] = False
                robot["available"] = False
        erp.add_event(
            "ROBOT_RECOVERY_ERROR",
            f"Failed to request recovery for {manufacturer}/{robot_id}: {error}",
        )
        return erp.jsonify({"error": f"Recovery command failed: {error}"}), 503

    if result.rc != erp.mqtt.MQTT_ERR_SUCCESS:
        with erp.data_lock:
            erp.recovery_requests.pop(robot_key, None)
            robot = erp.robots.get(robot_key)
            if robot is not None:
                robot["recoveryPending"] = False
                robot["available"] = False
        return erp.jsonify(
            {"error": f"Recovery command publish failed: {result.rc}"}
        ), 503

    erp.add_event(
        "ROBOT_RECOVERY_REQUESTED",
        f"Recovery requested for {manufacturer}/{robot_id}, order {order_id}",
        command,
    )
    return erp.jsonify(
        {
            "robotId": robot_id,
            "orderId": order_id,
            "status": "RECOVERY_PENDING",
            "message": "Waiting for robot recovery acknowledgement",
        }
    ), 202
