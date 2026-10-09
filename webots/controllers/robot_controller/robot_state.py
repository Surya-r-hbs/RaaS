# --------------------
# ROBOT STATE
# --------------------

class RobotState:

    def __init__(self):
        self.state = "IDLE"

        self.order_id = None

        self.current_node = None
        self.current_edge = None

        self.target_node = None
        self.target_edge = None

    # --------------------
    # START NAVIGATION
    # --------------------

    def start_navigation(self, order_id, node_id, edge_id):

        self.order_id = order_id

        self.target_node = node_id
        self.target_edge = edge_id

        self.state = "EXECUTING"

    # --------------------
    # NODE REACHED
    # --------------------

    def node_reached(self):

        self.current_node = self.target_node
        self.current_edge = self.target_edge

        self.target_node = None
        self.target_edge = None

        self.state = "NODE_REACHED"

    # --------------------
    # STOP ROBOT
    # --------------------

    def stop(self):

        # Keep the current node, edge and order information.
        # This is important so the ERP/Fleet Controller knows
        # exactly which mission was stopped.

        self.state = "STOPPED"

    # --------------------
    # ERROR
    # --------------------

    def set_error(self):

        self.state = "ERROR"

    # --------------------
    # CLEAR ORDER
    # --------------------

    def clear_order(self):

        self.order_id = None

        self.target_node = None
        self.target_edge = None

        self.state = "IDLE"

    # --------------------
    # PRINT STATE
    # --------------------

    def print_state(self):

        print("--------------------")
        print("ROBOT STATE")
        print(f"State        : {self.state}")
        print(f"Order ID     : {self.order_id}")
        print(f"Current node : {self.current_node}")
        print(f"Current edge : {self.current_edge}")
        print(f"Target node  : {self.target_node}")
        print(f"Target edge  : {self.target_edge}")
        print("--------------------")