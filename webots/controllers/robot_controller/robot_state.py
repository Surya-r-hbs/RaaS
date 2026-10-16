import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from raas_logging import get_logger, log_event

logger = get_logger("raas.robot_state")

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
        log_event(logger, "ROBOTS", "robot_state.snapshot", "Robot state snapshot", state=self.state, order_id=self.order_id, current_node=self.current_node, current_edge=self.current_edge, target_node=self.target_node, target_edge=self.target_edge)