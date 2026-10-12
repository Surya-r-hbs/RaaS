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
TIAGo, YouBot, and UR10e. The UR10e uses the Robotiq EPick vacuum gripper for
physics-based pick-and-place. The controller program is
`webots/controllers/robot_controller/robot_controller.py`.

### UR10e pick-and-place

Publish a `PICK_AND_PLACE` command to
`robot/DemoRobot/UR10e/command` (or the robot's configured manufacturer and ID).
`pickPosition` and `placePosition` are world-coordinate suction-cup contact
points (`x`, `y`, and `z` in meters); they are not object-center coordinates.
The controller approaches from above, enables suction at the pick point,
checks gripper presence, transfers the object above the higher point, releases
it at the place point, and reports progress in the robot status `manipulation`
field.

Example:

```json
{
  "command": "PICK_AND_PLACE",
  "taskId": "bottle-transfer-001",
  "pickPosition": {"x": 3.94, "y": -4.73, "z": 1.06},
  "placePosition": {"x": 4.0, "y": -4.3, "z": 0.74}
}
```

Run this command only when the UR10e is idle and the contact points are within
reach. The EPick gripper works best with flat, suction-compatible surfaces.
The example uses the water-bottle top and a clear spot on the table in the
current world; adjust the contact coordinates if the world has changed.

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

## Warehouse object inventory

The warehouse world includes a dedicated Webots Supervisor (`object_supervisor`)
that scans supported top-level scene objects once per second. It publishes raw
snapshots on `warehouse/objects/raw`; the Fleet controller validates the
snapshot, recalculates geometric relationships, and publishes a retained
snapshot on `warehouse/objects`. The ERP subscribes to that processed topic and
does not invent or persist scene objects when Webots is offline. The Supervisor
is excluded from robot fleet discovery.

The Objects page displays the live object inventory, geometry-derived
relationships, filters, and object details. Its refresh action publishes to
`warehouse/objects/refresh`; the next Supervisor scan is then published. The
HTTP API is:

- `GET /api/objects` — inventory, relationship summary, source timestamp, and
  simulation freshness.
- `GET /api/objects/<object_id>` — one object and its relationships.
- `GET /api/objects/relationships` — current object-to-object relationships.
- `POST /api/objects/refresh` — request an immediate scan (HTTP 202 when sent).

An inventory is considered connected only when its source timestamp and local
MQTT receive time are both no older than five seconds. The scan timestamp is
ISO-8601 with a timezone. Malformed or duplicate object records are rejected
by Fleet instead of silently creating a partial inventory.

Positions and bounds use this project's Webots convention: **X/Y are the floor
plane and Z is height**. Each object's position is the PROTO origin; its local
bounds, PROTO dimensions and rotation are kept separately. Objects currently
discovered are Table, PlasticCrate, WaterBottle, and WoodenBox. `ON_TOP_OF`
requires near-contact with a modeled support surface plus a 60% minimum
footprint overlap. `INSIDE` checks the transformed object bounds against a
known container interior. These are geometric checks, not sensor detections;
unmodeled occlusion, collisions, and unknown container interiors are not
inferred. Objects without a Webots DEF use a node ID that is stable during a
simulation run, but may change after reloading the world.

Run the regression tests, including object discovery, geometry, API freshness,
refresh handling, and the existing robot/fleet behavior, with:

    source ../.venv/bin/activate
    python -m unittest discover -s tests -v

## Object transport tasks (preloaded milestone)

The ERP **Transport Tasks** page creates internal `TRANSPORT_OBJECT` tasks and
uses the existing destination-only order planner and `/api/orders/<id>/dispatch`
path for navigation. Transport metadata stays in `transport_tasks.json`; it is
not added to VDA-5050 messages. Available APIs are:

- `GET /api/transport-options` — fresh inventory, configured navigation nodes,
  and fleet carrier metadata.
- `POST /api/transport-tasks` — create a `PRELOADED` task with an inventory
  `objectId` and configured `destinationNode`.
- `GET /api/transport-tasks` and `GET /api/transport-tasks/<task_id>` — task
  history and current backend state.
- `POST /api/transport-tasks/<task_id>/dispatch` — reserve the verified object
  and carrier, then dispatch through the existing VDA-5050 order code.

Current task states include `VALIDATING`, `LOAD_REQUIRED`, `CREATED`,
`READY_FOR_DISPATCH`, `DISPATCHING`, `DISPATCHED`, `EXECUTING`,
`VERIFYING_DELIVERY`, `COMPLETED`, `FAILED`, and `RECOVERY_REQUIRED`.
The task service never teleports or repositions an object. Its position and
spatial relationships remain sourced from fresh Supervisor inventory scans.
Successful delivery is recorded only after navigation reaches the selected
node and a fresh object pose plus load verification place the object's bounds
at that node. A verified load remains associated with the carrier after
delivery; ordinary empty-robot dispatch is blocked until a later verified
unload is reported.

The ERP Transport Tasks form is for preloaded objects only: users must first
place an object physically on a carrier's support surface in Webots. The page
does not pick, attach, or reposition objects. A `LOAD_REQUIRED` task means the
fresh Supervisor geometry check did not confirm that the object is supported by
the selected carrier. After physical staging, fresh inventory updates can
reconcile the task automatically, but dispatch still requires a verified
payload and stability profile.

The fleet derives `load_verification` only from a fresh object pose, a fresh
carrier pose/orientation, and support geometry reported by that robot adapter.
The Webots YouBot adapter exposes the central plate's measured support patch
from the Webots model for this geometric check, but does not claim a payload or
stability rating. The current world sets YouBot `numberOfArms 0`; its adapter
only advertises `NAVIGATION`, and its payload capacity and stability limits
remain unknown. The WoodenBox in the checked-in world is on the table, not on
the YouBot, and its footprint exceeds that conservative support patch.
Therefore it must remain `LOAD_REQUIRED`; even a geometrically verified load
cannot be dispatched until a `VERIFIED` payload/stability profile is configured.
Neither robot identity nor visual fit alone is accepted as proof of carrying
capability.

The inventory accepts a fleet-produced `load_verification` record only when
it has a fresh, timezone-aware timestamp (maximum age five seconds), matching
object ID, carrier robot/manufacturer, `verified: true`,
and `method: "SUPERVISOR_GEOMETRY"`. A carrier agent must separately publish a
`transportCapability` profile containing `status: "VERIFIED"`,
`canCarryPreloaded: true`, `stabilityVerified: true`, a positive
`maxPayloadKg`, and measured `platformDimensions` (`x`, `y`). The object must
also have measured mass and geometry dimensions. Missing/invalid evidence,
unknown capacity, or an unsupported footprint prevents dispatch.

Load associations are persisted with task history. On ERP restart, an
in-flight/uncertain task and its load move to `RECOVERY_REQUIRED` rather than
being cleared. A known MQTT publish rejection rolls back only the matching
pre-dispatch reservation; an uncertain publish outcome retains the reservation.
Verified `UNLOADED` evidence is required to release a prior carrier association.
