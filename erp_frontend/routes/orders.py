from flask import Blueprint
import app as erp

bp = Blueprint("orders", __name__)


@bp.post("/api/orders")
def create_order():

    payload = (
        erp.request.get_json(
            silent=True
        )
        or {}
    )

    order_id = str(
        payload.get(
            "orderId",
            "",
        )
    ).strip()

    robot_id = str(
        payload.get(
            "robotId",
            "",
        )
    ).strip()
    manufacturer = str(payload.get("manufacturer", "")).strip()

    priority = str(
        payload.get(
            "priority",
            "NORMAL",
        )
    ).upper()

    planning_mode = str(
        payload.get("planningMode", "route")
    ).lower()
    route = payload.get("route", [])
    destination = str(payload.get("destination", "")).strip()

    if not order_id:

        return erp.jsonify(
            {
                "error":
                    "orderId is required"
            }
        ), 400

    with erp.data_lock:
        selected_robot = erp.robots.get((manufacturer, robot_id))

    if selected_robot is None:
        return erp.jsonify({"error": "Unknown robot selection"}), 400

    reason = erp.robot_dispatchability_reason(selected_robot)
    if reason:
        return erp.jsonify(
            {
                "error": f"Robot {robot_id} is unavailable: {reason}"
            }
        ), 409

    if planning_mode not in {"route", "destination", "both"}:
        return erp.jsonify({"error": "Invalid planning mode"}), 400

    if not isinstance(route, list):
        return erp.jsonify({"error": "Route must be a list of node IDs"}), 400

    route = [
        str(node).strip()
        for node in route
        if str(node).strip()
    ]

    if planning_mode in {"route", "both"} and not route:
        return erp.jsonify(
            {"error": "Enter at least one required route node"}
        ), 400
    if planning_mode in {"destination", "both"} and not destination:
        return erp.jsonify({"error": "Enter a destination node"}), 400
    if planning_mode == "route" and destination:
        return erp.jsonify(
            {"error": "Destination is not used in route-only mode"}
        ), 400
    if planning_mode == "destination":
        route = []

    requested_nodes = route + (
        [destination]
        if destination
        else []
    )
    unknown_nodes = [
        node_id
        for node_id in requested_nodes
        if node_id not in erp.MAP_NODE_POSITIONS
    ]
    if unknown_nodes:
        return erp.jsonify(
            {
                "error":
                    f"Unknown map node(s): "
                    f"{', '.join(unknown_nodes)}"
            }
        ), 400

    try:
        planned_nodes = erp.plan_robot_order_nodes(
            selected_robot,
            route,
            destination,
        )
    except ValueError as error:
        if str(error) == "Robot location is not available yet":
            return erp.jsonify(
                {
                    "error": (
                        "Robot location is not available yet. "
                        "Wait for robot telemetry and refresh."
                    )
                }
            ), 409
        return erp.jsonify({"error": str(error)}), 400

    if len(planned_nodes) < 2:
        return erp.jsonify(
            {
                "error":
                    "The selected destination is already "
                    "the robot's current node"
            }
        ), 400

    with erp.data_lock:

        if order_id in erp.orders:

            return erp.jsonify(
                {
                    "error":
                        "Order already exists"
                }
            ), 409

        order = erp.build_order(
            order_id,
            robot_id,
            priority,
            planned_nodes,
            planning_mode,
            route,
            destination or None,
            manufacturer,
        )

        erp.orders[order_id] = order

    erp.save_orders()

    erp.add_event(
        "ORDER_CREATED",
        f"Order {order_id} created",
        order,
    )

    return erp.jsonify(order), 201

@bp.post(
    "/api/orders/<order_id>/dispatch"
)
def dispatch_order(
    order_id
):

    with erp.data_lock:
        order = erp.orders.get(order_id)
        if order is None:
            return erp.jsonify({"error": "Order not found"}), 404
        if order.get("status") not in {"CREATED", "FAILED", "STOPPED"}:
            return erp.jsonify(
                {"error": f"Order cannot be dispatched from {order['status']}"}
            ), 409

        robot_key = (order["manufacturer"], order["serialNumber"])
        selected_robot = erp.robots.get(robot_key)
        transport_task_id = erp.transport_task_manager.task_id_for_order(order_id)
        reason = erp.robot_dispatchability_reason(
            selected_robot,
            transport_task_id,
        )
        if reason:
            return erp.jsonify(
                {"error": f"Selected robot is unavailable: {reason}"}
            ), 409
        if robot_key in erp.dispatching_robots:
            return erp.jsonify(
                {"error": "Selected robot already has an active order"}
            ), 409

        previous_robot = dict(selected_robot)
        previous_status = order.get("status")
        order["status"] = "DISPATCHED"
        order["dispatchError"] = None
        selected_robot["status"] = "DISPATCHED"
        selected_robot["available"] = False
        selected_robot["availabilityReason"] = f"Active mission {order_id}"
        selected_robot["currentOrder"] = order_id
        selected_robot["lastUpdate"] = erp.now_utc()
        erp.dispatching_robots.add(robot_key)

    try:
        erp.save_orders()
    except OSError as error:
        with erp.data_lock:
            erp.dispatching_robots.discard(robot_key)
            if erp.orders.get(order_id) is order:
                order["status"] = previous_status
            erp.robots[robot_key] = previous_robot
        erp.add_event(
            "ORDER_PERSISTENCE_ERROR",
            f"Could not persist order {order_id} before dispatch: {error}",
        )
        return erp.jsonify({"error": "Could not persist order before dispatch"}), 503

    try:
        result = erp.mqtt_client.publish(
            f"vda5050/v3/{order['manufacturer']}/{order['serialNumber']}/order",
            erp.json.dumps(order),
            qos=1,
        )
    except Exception as error:
        with erp.data_lock:
            erp.dispatching_robots.discard(robot_key)
            selected_robot = erp.robots.get(robot_key)
            if selected_robot is not None:
                selected_robot["recoveryRequired"] = True
                selected_robot["recoveryReason"] = (
                    "Order publish outcome is uncertain; await robot recovery"
                )
                selected_robot["available"] = False
            order["status"] = "RECOVERY_REQUIRED"
            order["recoveryReason"] = (
                "Order publish outcome is uncertain; robot acknowledgement is required"
            )
        erp.save_orders()
        erp.add_event(
            "ORDER_DISPATCH_UNCERTAIN",
            f"MQTT publish outcome for {order_id} is uncertain: {error}",
        )
        return erp.jsonify(
            {"error": "MQTT publish outcome is uncertain; robot recovery is required"}
        ), 503

    if (
        result.rc
        != erp.mqtt.MQTT_ERR_SUCCESS
    ):
        with erp.data_lock:
            erp.dispatching_robots.discard(robot_key)
            if erp.orders.get(order_id) is order:
                order["status"] = previous_status
                order["dispatchError"] = f"MQTT publish failed: {result.rc}"
            current_robot = erp.robots.get(robot_key)
            if (
                current_robot is not None
                and current_robot.get("currentOrder") == order_id
                and current_robot.get("status") == "DISPATCHED"
            ):
                erp.robots[robot_key] = previous_robot
        try:
            erp.save_orders()
        except OSError as error:
            erp.add_event(
                "ORDER_PERSISTENCE_ERROR",
                f"Could not persist dispatch rollback for {order_id}: {error}",
            )

        return erp.jsonify(
            {
                "error": f"MQTT publish failed: {result.rc}"
            }
        ), 503

    with erp.data_lock:
        erp.dispatching_robots.discard(robot_key)
        order["erpStatus"] = "DISPATCHED"
    try:
        erp.save_orders()
    except OSError as error:
        with erp.data_lock:
            order["status"] = "RECOVERY_REQUIRED"
            order["recoveryReason"] = (
                "Order was published but its dispatch update could not be persisted"
            )
            active_robot = erp.robots.get(robot_key)
            if active_robot is not None:
                active_robot["available"] = False
                active_robot["recoveryRequired"] = True
                active_robot["recoveryReason"] = order["recoveryReason"]
        erp.add_event(
            "ORDER_PERSISTENCE_ERROR",
            f"Order {order_id} was published but its dispatch update could not be saved: {error}",
        )
        try:
            erp.save_orders()
        except OSError as recovery_error:
            erp.add_event(
                "ORDER_PERSISTENCE_ERROR",
                f"Could not persist recovery state for {order_id}: {recovery_error}",
            )
        return erp.jsonify(
            {
                "error": order["recoveryReason"],
                "orderId": order_id,
            }
        ), 503

    erp.add_event(
        "ORDER_DISPATCHED",
        f"Order {order_id} published "
        "to VDA-5050 order topic",
        order,
    )

    return erp.jsonify(order)
