"""Mission model and VDA-5050 order validation."""

from dataclasses import dataclass


@dataclass
class Mission:
    order: dict
    node_index: int = 0
    edge_index: int = 0
    last_node_id: str | None = None
    last_node_sequence_id: int = 0
    current_edge_id: str | None = None
    current_edge_sequence_id: int = 0
    state: str = "IDLE"
    header_id: int = 0

    @property
    def nodes(self) -> list[dict]:
        return self.order.get("nodes", [])

    @property
    def edges(self) -> list[dict]:
        return self.order.get("edges", [])

    @property
    def order_id(self) -> str:
        return self.order["orderId"]


def validate_order(
    order: dict,
    manufacturer: str,
    serial_number: str,
) -> tuple[bool, str]:
    required = ("orderId", "orderUpdateId", "nodes", "edges")
    missing = [key for key in required if key not in order]

    if missing:
        return False, f"Missing required fields: {', '.join(missing)}"

    if order.get("manufacturer") != manufacturer:
        return False, "Manufacturer does not match fleet controller"

    if order.get("serialNumber") != serial_number:
        return False, "Serial number does not match fleet controller"

    nodes = order.get("nodes", [])
    edges = order.get("edges", [])

    if len(nodes) < 2:
        return False, "An order must contain at least two nodes"

    if len(nodes) != len(edges) + 1:
        return False, "Number of nodes must equal number of edges plus one"

    for index, node in enumerate(nodes):
        if "nodeId" not in node or "sequenceId" not in node:
            return False, "Every node needs nodeId and sequenceId"

        if index and node["sequenceId"] <= nodes[index - 1]["sequenceId"]:
            return False, "Node sequence IDs must be increasing"

    edge_ids = set()

    for index, edge in enumerate(edges):
        if "edgeId" not in edge or "sequenceId" not in edge:
            return False, "Every edge needs edgeId and sequenceId"

        if edge["edgeId"] in edge_ids:
            return False, f"Duplicate edgeId: {edge['edgeId']}"

        edge_ids.add(edge["edgeId"])

        if index and edge["sequenceId"] <= edges[index - 1]["sequenceId"]:
            return False, "Edge sequence IDs must be increasing"

    return True, "Order validation passed"
