"""Discover robot instances in the Webots world through Supervisor."""


def discover_world_robots(
    supervisor,
    active_manufacturer,
    active_robot_id=None,
):
    root = supervisor.getRoot()
    children = root.getField("children")

    if children is None:
        raise RuntimeError("Cannot inspect Webots world children for robots")

    discovered = []
    name_counts = {}
    self_node = supervisor.getSelf()
    self_node_id = self_node.getId()

    for index in range(children.getCount()):
        node = children.getMFNode(index)
        if node is None:
            continue

        try:
            type_name = node.getTypeName()
            base_type_name = node.getBaseTypeName()
        except Exception:
            continue

        if type_name != "Robot" and base_type_name != "Robot":
            continue

        name_field = node.getField("name")
        name = name_field.getSFString().strip() if name_field else ""
        definition = node.getDef()
        is_active = node.getId() == self_node_id
        base_id = (
            name
            or definition
            or (
                active_robot_id
                if is_active and active_robot_id
                else f"robot-{node.getId()}"
            )
        )
        name_counts[base_id] = name_counts.get(base_id, 0) + 1
        robot_id = (
            base_id
            if name_counts[base_id] == 1
            else f"{base_id}-{name_counts[base_id]}"
        )

        if type_name == "Robot":
            robot_type = name or "Webots Robot"
        else:
            robot_type = type_name

        position = node.getPosition()
        record = {
            "robotId": robot_id,
            "robotType": robot_type,
            "position": {
                "x": float(position[0]),
                "y": float(position[1]),
                "z": float(position[2]),
            },
            "present": True,
            "agentOnline": is_active,
        }
        record["manufacturer"] = active_manufacturer
        discovered.append((record, node.getId()))

    return discovered
