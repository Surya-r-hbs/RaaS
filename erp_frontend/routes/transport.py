from flask import Blueprint
import app as erp

bp = Blueprint("transport", __name__)


@bp.get("/api/transport-options")
def transport_options():
    inventory, status_code = erp.get_object_inventory_response()
    if status_code != 200:
        return erp.jsonify(inventory), status_code
    with erp.data_lock:
        carrier_list = [
            dict(robot)
            for robot in erp.robots.values()
        ]
    return erp.jsonify(
        {
            "objects": inventory.get("objects", []),
            "destinations": [
                {"nodeId": node_id, "position": {"x": position[0], "y": position[1]}}
                for node_id, position in erp.MAP_NODE_POSITIONS.items()
            ],
            "carriers": carrier_list,
            "simulationTransportEnabled": erp.SIMULATION_TRANSPORT_ENABLED,
            "handlingModes": ["PRELOADED"],
        }
    )

@bp.get("/api/transport-tasks")
def list_transport_tasks():
    return erp.jsonify({"tasks": erp.transport_task_manager.list_tasks()})

@bp.get("/api/transport-tasks/<task_id>")
def get_transport_task(task_id):
    task = erp.transport_task_manager.get(task_id)
    if task is None:
        return erp.jsonify({"error": "Transport task not found"}), 404
    return erp.jsonify(task)

@bp.post("/api/transport-tasks")
def create_transport_task():
    payload = erp.request.get_json(silent=True)
    if not isinstance(payload, dict):
        return erp.jsonify({"error": "A JSON transport task request is required"}), 400
    if payload.get("taskType") != "TRANSPORT_OBJECT":
        return erp.jsonify({"error": "taskType must be TRANSPORT_OBJECT"}), 400
    if payload.get("handlingMode") != "PRELOADED":
        return erp.jsonify(
            {"error": "Only PRELOADED transport is supported; no handling robot is configured"}
        ), 400

    object_id = str(payload.get("objectId") or "").strip()
    destination = str(payload.get("destinationNode") or "").strip()
    if not object_id:
        return erp.jsonify({"error": "objectId is required"}), 400
    if destination not in erp.MAP_NODE_POSITIONS:
        return erp.jsonify({"error": f"Unknown destination node: {destination or '(empty)'}"}), 400

    inventory, status_code = erp.get_object_inventory_response()
    if status_code != 200:
        return erp.jsonify(inventory), status_code
    object_record = next(
        (
            item
            for item in inventory["objects"]
            if item.get("object_id") == object_id
        ),
        None,
    )
    if object_record is None:
        return erp.jsonify({"error": f"Object not found in live inventory: {object_id}"}), 404

    carrier_id = str(payload.get("carrierRobotId") or "").strip() or None
    manufacturer = str(payload.get("manufacturer") or "").strip() or None
    with erp.data_lock:
        carrier_list = [dict(robot) for robot in erp.robots.values()]
    if carrier_id and not any(
        robot.get("robotId") == carrier_id
        and (manufacturer is None or robot.get("manufacturer") == manufacturer)
        for robot in carrier_list
    ):
        return erp.jsonify({"error": f"Unknown carrier robot: {carrier_id}"}), 404

    try:
        erp.transport_task_manager.observe_inventory(inventory, carrier_list)
        task = erp.transport_task_manager.create_task(
            object_record,
            destination,
            carrier_list,
            carrier_id,
            manufacturer,
        )
    except erp.TransportConflictError as error:
        return erp.jsonify({"error": str(error)}), 409
    except OSError as error:
        erp.add_event("TRANSPORT_TASK_PERSISTENCE_ERROR", str(error))
        return erp.jsonify({"error": "Could not persist transport task"}), 503
    except ValueError as error:
        erp.add_event("TRANSPORT_TASK_ERROR", str(error))
        return erp.jsonify({"error": str(error)}), 400
    erp.add_event(
        "TRANSPORT_TASK_CREATED",
        f"Transport task {task['taskId']} created with state {task['status']}",
        task,
    )
    return erp.jsonify(task), 201

@bp.post("/api/transport-tasks/<task_id>/dispatch")
def dispatch_transport_task(task_id):
    task = erp.transport_task_manager.get(task_id)
    if task is None:
        return erp.jsonify({"error": "Transport task not found"}), 404
    if task.get("status") != "READY_FOR_DISPATCH":
        return erp.jsonify(
            {
                "error": task.get("message")
                or f"Transport task cannot dispatch from {task.get('status')}",
                "task": task,
            }
        ), 409

    inventory, status_code = erp.get_object_inventory_response()
    if status_code != 200:
        return erp.jsonify(inventory), status_code
    object_record = next(
        (
            item
            for item in inventory["objects"]
            if item.get("object_id") == task.get("objectId")
        ),
        None,
    )
    if object_record is None:
        return erp.jsonify({"error": "Transport object is absent from live inventory"}), 409

    robot_key = (task.get("manufacturer"), task.get("carrierRobotId"))
    with erp.data_lock:
        carrier = erp.robots.get(robot_key)
        carrier = dict(carrier) if carrier is not None else None
    if carrier is None:
        return erp.jsonify({"error": "Verified carrier is not in the current fleet"}), 409
    reason = erp.robot_dispatchability_reason(carrier, task_id)
    if reason:
        return erp.jsonify({"error": f"Carrier is unavailable: {reason}"}), 409
    try:
        route_nodes = erp.plan_robot_order_nodes(
            carrier,
            [],
            task["destinationNode"],
        )
    except ValueError as error:
        if str(error) == "Robot location is not available yet":
            return erp.jsonify({"error": "Carrier location is not available yet"}), 409
        return erp.jsonify({"error": str(error)}), 400
    if len(route_nodes) < 2:
        return erp.jsonify({"error": "Carrier is already at the destination"}), 409
    if not erp.transport_task_manager.reserve(task_id, object_record, carrier):
        return erp.jsonify(
            {"error": "Load, carrier, or task reservation changed; refresh and retry"}
        ), 409

    order_id = f"transport-{erp.uuid.uuid4().hex}"
    order = erp.build_order(
        order_id,
        carrier["robotId"],
        "NORMAL",
        route_nodes,
        "destination",
        [],
        task["destinationNode"],
        carrier["manufacturer"],
    )
    with erp.data_lock:
        if order_id in erp.orders:
            erp.transport_task_manager.rollback_dispatch(task_id)
            return erp.jsonify({"error": "Generated transport order ID already exists"}), 409
        erp.orders[order_id] = order
    try:
        erp.save_orders()
    except OSError as error:
        with erp.data_lock:
            erp.orders.pop(order_id, None)
        erp.transport_task_manager.rollback_dispatch(task_id)
        erp.add_event("ORDER_PERSISTENCE_ERROR", str(error))
        return erp.jsonify({"error": "Could not persist transport order"}), 503

    if not erp.transport_task_manager.attach_order(task_id, order_id):
        erp.transport_task_manager.mark_recovery_required(
            task_id,
            "Could not correlate the transport task with its persisted navigation order",
        )
        return erp.jsonify({"error": "Could not reserve transport task for dispatch"}), 503

    dispatch_result = erp.dispatch_order(order_id)
    if isinstance(dispatch_result, tuple):
        response, dispatch_status = dispatch_result
    else:
        response, dispatch_status = dispatch_result, dispatch_result.status_code
    if dispatch_status == 200:
        if not erp.transport_task_manager.mark_dispatched(task_id):
            erp.transport_task_manager.mark_recovery_required(
                task_id,
                "Navigation was dispatched but task state could not be persisted",
            )
            return erp.jsonify({"error": "Dispatch outcome requires recovery"}), 503
        erp.transport_task_manager.update_order_state(order_id, "DISPATCHED")
        return erp.jsonify(
            {
                "task": erp.transport_task_manager.get(task_id),
                "order": order,
            }
        ), 202
    dispatch_error = (
        response.get_json(silent=True) or {}
    ).get("error", "")
    safe_dispatch_rejection = (
        dispatch_error.startswith("MQTT publish failed:")
        or dispatch_error == "Could not persist order before dispatch"
    )
    if dispatch_status >= 500 and not safe_dispatch_rejection:
        erp.transport_task_manager.mark_recovery_required(
            task_id,
            "Order publish or persistence outcome is uncertain; load association retained",
        )
    else:
        erp.transport_task_manager.rollback_dispatch(task_id)
        with erp.data_lock:
            order["status"] = "FAILED"
            order["dispatchError"] = "Transport dispatch was rejected before movement"
        erp.save_orders()
    return response, dispatch_status

@bp.get("/api/objects")
def list_objects():
    payload, status_code = erp.get_object_inventory_response()
    return erp.jsonify(payload), status_code

@bp.get("/api/objects/relationships")
def list_object_relationships():
    payload, status_code = erp.get_object_inventory_response()
    return erp.jsonify(
        {
            "relationships": payload.get("relationships", []),
            "lastRefresh": payload.get("lastRefresh"),
            "simulationConnected": payload.get("simulationConnected", False),
            **({"error": payload["error"]} if "error" in payload else {}),
        }
    ), status_code

@bp.get("/api/objects/<object_id>")
def get_object(object_id):
    if not erp.re.fullmatch(r"[A-Za-z0-9_.:-]{1,128}", object_id):
        return erp.jsonify({"error": "Invalid object ID"}), 400

    payload, status_code = erp.get_object_inventory_response()
    record = next(
        (
            item
            for item in payload.get("objects", [])
            if item.get("object_id") == object_id
        ),
        None,
    )
    if status_code != 200:
        return erp.jsonify({"error": payload["error"], "object": record}), status_code
    if record is None:
        return erp.jsonify({"error": "Object not found"}), 404
    return erp.jsonify(record)

@bp.post("/api/objects/refresh")
def refresh_objects():
    if not erp.mqtt_client.is_connected():
        return erp.jsonify(
            {"error": "Cannot request inventory refresh: MQTT is disconnected"}
        ), 503

    try:
        result = erp.mqtt_client.publish(
            "warehouse/objects/refresh",
            erp.json.dumps({"requestedAt": erp.now_utc()}),
            qos=0,
        )
    except Exception as error:
        erp.add_event(
            "OBJECT_INVENTORY_ERROR",
            f"Could not request Webots inventory refresh: {error}",
        )
        return erp.jsonify({"error": "Could not request inventory refresh"}), 503

    if result.rc != erp.mqtt.MQTT_ERR_SUCCESS:
        return erp.jsonify(
            {"error": f"Could not request inventory refresh: {result.rc}"}
        ), 503

    return erp.jsonify(
        {
            "status": "REFRESH_REQUESTED",
            "message": "Webots will publish the next inventory snapshot shortly",
        }
    ), 202
