# RaaS VDA-5050 ERP Lab

This package connects the ERP frontend to the MQTT fleet controller and the
Webots robot controller.

## Architecture

ERP browser
    |
    | HTTP
    v
erp_frontend/app.py
    |
    | MQTT VDA-5050 order
    v
vda5050/v3/DemoRobot/robot001/order
    |
    v
fleet_control/fleet_controller.py
    |
    | internal MQTT MOVE_TO_NODE
    v
robot/DemoRobot/robot001/command
    |
    v
Webots robot_controller.py
    |
    | EXECUTING / NODE_REACHED
    v
robot/DemoRobot/robot001/status
    |
    v
fleet_control/fleet_controller.py
    |
    | VDA-5050 state
    v
vda5050/v3/DemoRobot/robot001/state
    |
    v
erp_frontend/app.py
    |
    | HTTP polling
    v
ERP browser

## MQTT topics

Order:
vda5050/v3/DemoRobot/robot001/order

VDA-5050 state:
vda5050/v3/DemoRobot/robot001/state

Internal robot command:
robot/DemoRobot/robot001/command

Internal robot status:
robot/DemoRobot/robot001/status

Fleet robot registry:
fleet/robots

Robot discovery:
robot/discovery

## Requirements

- Python 3.10+
- Mosquitto on localhost:1883
- Webots
- paho-mqtt 2.x
- Flask 3.x

## Install

From the project root:

    python -m pip install -r requirements.txt

## Run

Terminal 1:

    mosquitto -v

Terminal 2:

    python fleet_control/fleet_controller.py

Terminal 3:

    webots webots/worlds/RaaS_warehouse.wbt

The world assigns the `robot_controller` controller to the E-puck, MiR100,
TIAGo, and YouBot. The controller program is
`webots/controllers/robot_controller/robot_controller.py`.

Terminal 4:

    python erp_frontend/app.py

Open:

    http://localhost:5000

## Test

1. Open the ERP dashboard.
2. Click Create Order.
3. Use order003.
4. Use route N0,N1,N2,N3.
5. Create the order.
6. Open Orders.
7. Click Dispatch.
8. The ERP publishes the VDA-5050 order.
9. Fleet controller validates it.
10. Fleet controller sends MOVE_TO_NODE to Webots.
11. Webots publishes EXECUTING.
12. Webots publishes NODE_REACHED at each node.
13. Fleet controller advances E1 -> E2 -> E3.
14. Fleet controller publishes COMPLETED at N3.
15. ERP receives MQTT state/status and updates automatically.

## Robot fleet registry

The Webots controller scans the world for robot instances and publishes
discovery information to `robot/discovery`. The fleet controller maintains the
authoritative registry and publishes retained snapshots on `fleet/robots`;
the ERP reads this snapshot to show robot model, live state, and availability.

The generic `robot_controller` delegates robot hardware access through an
adapter under `webots/controllers/robot_controller/adapters/`. The E-puck
adapter reuses the existing X/Y floor navigation implementation. MiR100 and
TIAGo adapters use their model-specific drive motors with the shared X/Y
waypoint controller. The YouBot adapter uses four-wheel holonomic kinematics
so it can drive toward a waypoint in any direction without first turning.
These adapters use Supervisor pose data; the E-puck continues to use its GPS
and compass.

The current `RaaS_warehouse.wbt` assigns `robot_controller` to the E-puck,
MiR100, TIAGo, and YouBot. Each agent uses its Webots robot name as its ID and
the configured `DemoRobot` manufacturer. All four robots advertise navigation
and can be selected for orders when their agents are online and idle.

The Fleet registry's `currentOrder` is the live robot assignment; saved ERP
orders are history and do not reserve a robot by themselves. Dispatch reserves
the robot under the ERP lock before MQTT publishing. A status heartbeat must be
fresh and report no current assignment before another order can be dispatched.
After an ERP restart, persisted `DISPATCHED`, `EXECUTING`, or `STOPPING` orders
are marked `RECOVERY_REQUIRED` rather than blindly reactivated. Fresh matching
Fleet telemetry restores an active assignment; a fresh terminal/idle report
reconciles the old order as terminal/interrupted without deleting its history.

If Fleet reports an uncertain assignment, the Robot Fleet page offers
**Recover Robot**. The operator must confirm the recovery is safe and provide a
reason. Recovery sends a robot-specific command that stops the motors; Fleet
and ERP clear the assignment only after that same robot acknowledges the
matching order ID. A disconnected or stale robot remains unavailable. Robot
controller session IDs and monotonic status sequence numbers reject delayed
or duplicate status messages; recurring `NODE_REACHED` heartbeats for the
completed order refresh liveness without restoring its assignment. Retained VDA
order messages are not replayed.

There is no Forklift robot in the current world. The Webots R2025a
`Forklift.proto` is a `Solid` model, not a `Robot`, and exposes no controller or
motor devices; it cannot run this robot agent as-is. Do not assign the
E-puck-specific control behavior to it or advertise lifting/transport support.

Webots device names are enumerated from each running robot at startup and
shown in the Fleet page. The current adapters use only model-verified motor
names: E-puck `left wheel motor` / `right wheel motor`, MiR100
`middle_left_wheel_joint` / `middle_right_wheel_joint`, and TIAGo
`wheel_left_joint` / `wheel_right_joint`, and YouBot `wheel1`–`wheel4`.

To test, run Mosquitto, then from the project root run:

    source ../.venv/bin/activate
    python fleet_control/fleet_controller.py

Start `webots/worlds/RaaS_warehouse.wbt` in Webots and then in another
terminal run:

    source ../.venv/bin/activate
    python erp_frontend/app.py

Confirm all four agents log `ROBOT ADAPTER READY`, their IDs/types and
available devices, then check the ERP Robot Fleet page. With each agent online,
all four should become available while `IDLE` and without an active order.
Verify navigation and STOP in a clear area. Webots simulation has to be run
locally to validate the model/controller combination and motion.

Run the MQTT-independent lifecycle regression tests with:

    source ../.venv/bin/activate
    python -m unittest discover -s tests -v

## Important

The ERP frontend is now the order sender.

Do not run the old standalone order-sender script while testing this package,
because it would create duplicate VDA-5050 orders.

The frontend does not fake mission completion. Order status is updated from
the actual MQTT robot/fleet events.

## Webots

The Webots world must contain DEF nodes:

N0
N1
N2
N3

The current robot controller assumes the route is physically laid out so that
driving forward reaches the next target node.

## Order planning

The ERP order form supports required route waypoints, a destination-only
request, or route waypoints followed by a destination. The robot controller
publishes its current position and nearest map node while idle; order creation
uses that telemetry as the start of the planned order.

Required route nodes are kept in the specified order, with the shortest
connected map path inserted between them. A destination is appended as a
direct navigation leg from the current/last required node; destination-only
orders therefore contain just the live start node and destination. This
prototype does not perform obstacle avoidance or collision-aware path planning.
Keep `MAP_NODE_POSITIONS` and `MAP_CONNECTIONS` in `fleet_control/config.py`
consistent with the Webots world for required-waypoint routing.
