import math

from config import MAP_CONNECTIONS, MAP_NODE_POSITIONS, VDA_VERSION
from time_utils import now_utc


def build_order(
    order_id,
    robot_id,
    priority,
    route_nodes,
    planning_mode,
    requested_route,
    destination,
    manufacturer,
):
    nodes = [
        {
            "nodeId": node_id,
            "sequenceId": index * 2,
            "released": True,
            "actions": [],
        }
        for index, node_id in enumerate(route_nodes)
    ]
    edges = [
        {
            "edgeId": f"E{index + 1}",
            "sequenceId": index * 2 + 1,
            "released": True,
            "actions": [],
        }
        for index in range(len(route_nodes) - 1)
    ]

    return {
        "headerId": 0,
        "timestamp": now_utc(),
        "version": VDA_VERSION,
        "manufacturer": manufacturer,
        "serialNumber": robot_id,
        "orderId": order_id,
        "orderUpdateId": 0,
        "nodes": nodes,
        "edges": edges,
        "erpPriority": priority,
        "erpStatus": "CREATED",
        "status": "CREATED",
        "erpPlanningMode": planning_mode,
        "erpRoute": requested_route,
        "erpDestination": destination,
    }


def shortest_node_path(start_node, end_node):
    """Return the least-distance route through the configured map graph."""
    if start_node not in MAP_NODE_POSITIONS:
        raise ValueError(f"Unknown start node: {start_node}")
    if end_node not in MAP_NODE_POSITIONS:
        raise ValueError(f"Unknown destination node: {end_node}")

    adjacency = {node_id: [] for node_id in MAP_NODE_POSITIONS}
    for first, second in MAP_CONNECTIONS:
        first_position = MAP_NODE_POSITIONS[first]
        second_position = MAP_NODE_POSITIONS[second]
        distance = (
            (first_position[0] - second_position[0]) ** 2
            + (first_position[1] - second_position[1]) ** 2
        ) ** 0.5
        adjacency[first].append((second, distance))
        adjacency[second].append((first, distance))

    distances = {start_node: 0.0}
    previous = {}
    unvisited = set(adjacency)

    while unvisited:
        current = min(
            unvisited,
            key=lambda node_id: (
                distances.get(node_id, float("inf")),
                node_id,
            ),
        )
        if distances.get(current, float("inf")) == float("inf"):
            break
        if current == end_node:
            break

        unvisited.remove(current)
        for neighbor, edge_distance in adjacency[current]:
            candidate = distances[current] + edge_distance
            if candidate < distances.get(neighbor, float("inf")):
                distances[neighbor] = candidate
                previous[neighbor] = current

    if end_node not in distances:
        raise ValueError(
            f"No map path exists from {start_node} to {end_node}"
        )

    path = [end_node]
    while path[-1] != start_node:
        path.append(previous[path[-1]])
    path.reverse()
    return path


def plan_order_nodes(start_node, route_nodes, destination):
    """Route through required waypoints, then add destination as a direct leg."""
    if not route_nodes and not destination:
        raise ValueError("Select a route, a destination, or both")

    planned_nodes = [start_node]
    current_node = start_node
    for stop in route_nodes:
        leg = shortest_node_path(current_node, stop)
        planned_nodes.extend(leg[1:])
        current_node = stop

    if destination and destination != current_node:
        planned_nodes.append(destination)

    return planned_nodes


def plan_robot_order_nodes(robot, route_nodes, destination):
    start_node = robot.get("currentNode")
    inferred_start = False
    if start_node not in MAP_NODE_POSITIONS:
        nearest_node = robot.get("nearestNode")
        nearest_distance = robot.get("nearestNodeDistance")
        position = robot.get("position")
        try:
            coordinates = [float(position[axis]) for axis in ("x", "y", "z")]
        except (KeyError, TypeError, ValueError):
            coordinates = []
        if (
            nearest_node not in MAP_NODE_POSITIONS
            or len(coordinates) != 3
            or not all(math.isfinite(value) for value in coordinates)
            or not isinstance(nearest_distance, (int, float))
            or not math.isfinite(nearest_distance)
            or nearest_distance < 0
        ):
            raise ValueError("Robot location is not available yet")
        start_node = nearest_node
        inferred_start = True

    planned_nodes = plan_order_nodes(start_node, route_nodes, destination)
    if len(planned_nodes) < 2 and inferred_start:
        alternate_starts = [
            first if second == start_node else second
            for first, second in MAP_CONNECTIONS
            if first == start_node or second == start_node
        ]
        for alternate_start in alternate_starts:
            candidate = plan_order_nodes(
                alternate_start,
                route_nodes,
                destination,
            )
            if len(candidate) >= 2:
                return candidate
    return planned_nodes
