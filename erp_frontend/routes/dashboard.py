from flask import Blueprint
import app as erp

bp = Blueprint("dashboard", __name__)


@bp.get("/")
def index():

    return erp.render_template(
        "index.html"
    )

@bp.get("/api/dashboard")
def dashboard():

    with erp.data_lock:

        order_list = list(erp.orders.values())
        robot_list = []
        for robot in erp.robots.values():
            robot_view = dict(robot)
            reason = erp.robot_dispatchability_reason(robot_view)
            robot_view["available"] = reason is None
            if reason:
                robot_view["availabilityReason"] = reason
            robot_list.append(robot_view)
        event_list = list(erp.events)
        recent_logs = list(erp.logs)

    return erp.jsonify(
        {
            "orders": order_list,
            "robots": robot_list,
            "transportTasks": erp.transport_task_manager.list_tasks(),
            "events": event_list,
            "logs": recent_logs,
            "summary": {
                "totalOrders": len(order_list),
                "activeOrders": len(
                    {
                        robot.get("currentOrder")
                        for robot in robot_list
                        if robot.get("currentOrder")
                    }
                ),
                "completedOrders": sum(
                    order.get("status") == "COMPLETED"
                    for order in order_list
                ),
                "failedOrders": sum(
                    order.get("status") == "FAILED"
                    for order in order_list
                ),
                "availableRobots": sum(
                    robot.get("available") is True
                    for robot in robot_list
                ),
            },
        }
    )

@bp.get("/api/logs")
def get_logs():
    category = str(erp.request.args.get("category", "")).strip()
    severity = str(erp.request.args.get("severity", "")).strip()
    search = str(erp.request.args.get("search", "")).strip().lower()
    try:
        limit = max(1, min(int(erp.request.args.get("limit", "100")), 500))
    except ValueError:
        limit = 100

    with erp.data_lock:
        filtered_logs = []
        for entry in erp.logs:
            if category and erp.normalize_log_category(entry.get("category")) != erp.normalize_log_category(category):
                continue
            if severity and entry.get("severity") != erp.normalize_severity(severity):
                continue
            payload_text = " ".join(
                [
                    str(entry.get("category", "")),
                    str(entry.get("severity", "")),
                    str(entry.get("source", "")),
                    str(entry.get("message", "")),
                ]
            ).lower()
            if search and search not in payload_text:
                continue
            filtered_logs.append(entry)

    return erp.jsonify(
        {
            "logs": filtered_logs[:limit],
            "categories": list(erp.LOG_CATEGORIES.keys()),
            "severities": erp.LOG_SEVERITY_LEVELS,
        }
    )

@bp.get("/api/events")
def get_events():

    with erp.data_lock:

        return erp.jsonify(
            erp.events
        )
