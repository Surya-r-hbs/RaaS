import re


# --------------------
# NODE MANAGER
# --------------------

class NodeManager:

    def __init__(self, supervisor):

        self.supervisor = supervisor

        self.nodes = {}

    # --------------------
    # DISCOVER NODES
    # --------------------

    def discover_nodes(self):

        self.nodes.clear()

        root = self.supervisor.getRoot()

        children_field = root.getField(
            "children"
        )

        if children_field is None:

            print("--------------------")
            print(
                "ERROR: Could not access "
                "Webots world children"
            )
            print("--------------------")

            return

        count = children_field.getCount()

        for index in range(count):

            node = children_field.getMFNode(index)

            if node is None:
                continue

            try:
                node_type = node.getTypeName()

            except Exception:
                continue

            if node_type != "Solid":
                continue

            try:
                node_def = node.getDef()

            except Exception:
                continue

            if not node_def:
                continue

            # Accept:
            #
            # N0
            # N1
            # N2
            # N10
            #

            if not re.fullmatch(
                r"N\d+",
                node_def
            ):
                continue

            try:

                position = node.getPosition()

                self.nodes[node_def] = {
                    "x": float(position[0]),
                    "y": float(position[1]),
                    "z": float(position[2])
                }

            except Exception as error:

                print(
                    f"Could not read position "
                    f"of {node_def}: {error}"
                )

    # --------------------
    # CHECK NODE
    # --------------------

    def exists(self, node_id):

        return node_id in self.nodes

    # --------------------
    # GET NODE
    # --------------------

    def get_node(self, node_id):

        return self.nodes.get(node_id)

    # --------------------
    # GET ALL NODES
    # --------------------

    def get_nodes(self):

        return self.nodes.copy()

    # --------------------
    # PRINT NODES
    # --------------------

    def print_nodes(self):

        print("--------------------")
        print("DISCOVERED WEBOTS NODES")

        if not self.nodes:

            print("No warehouse nodes discovered.")

        else:

            def node_number(name):
                return int(name[1:])

            for node_id in sorted(
                self.nodes.keys(),
                key=node_number
            ):

                position = self.nodes[node_id]

                print(
                    f"{node_id} -> "
                    f"x={position['x']:.2f}, "
                    f"y={position['y']:.2f}, "
                    f"z={position['z']:.2f}"
                )

        print("--------------------")