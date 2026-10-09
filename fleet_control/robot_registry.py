"""Fleet-side registry of robot agents and Webots world discoveries."""

from datetime import datetime, timezone
import threading
import time


class RobotRegistry:
    def __init__(self, heartbeat_timeout_seconds=8.0):
        self._robots = {}
        self._lock = threading.Lock()
        self._heartbeat_timeout_seconds = heartbeat_timeout_seconds

    @staticmethod
    def _key(manufacturer, robot_id):
        return str(manufacturer or "UNKNOWN"), str(robot_id)

    def _get_or_create(self, manufacturer, robot_id):
        key = self._key(manufacturer, robot_id)
        return self._robots.setdefault(
            key,
            {
                "robotId": key[1],
                "manufacturer": key[0],
                "robotType": "Unknown",
                "status": "NO_AGENT",
                "available": False,
                "availabilityReason": "No live robot agent",
                "present": True,
                "agentOnline": False,
                "currentNode": None,
                "position": None,
                "currentEdge": None,
                "currentOrder": None,
                "lastOrderId": None,
                "lastOrderStatus": None,
                "recoveryRequired": False,
                "recoveryPending": False,
                "recoveryReason": None,
                "agentSessionId": None,
                "capabilities": [],
                "availableDevices": [],
                "transportCapability": None,
                "lastUpdate": None,
                "_last_seen": 0.0,
                "_source": "webots",
                "_agent_session_id": None,
                "_status_sequence": -1,
            },
        )

    def discover_world(self, records):
        with self._lock:
            seen = set()
            for record in records:
                robot_id = str(record.get("robotId") or "").strip()
                if not robot_id:
                    continue
                manufacturer = record.get("manufacturer") or "UNKNOWN"
                key = self._key(manufacturer, robot_id)
                seen.add(key)
                entry = self._get_or_create(manufacturer, robot_id)
                if entry["_source"] != "agent":
                    entry["_source"] = "webots"
                entry["present"] = True
                if record.get("robotType"):
                    entry["robotType"] = record["robotType"]
                if (
                    isinstance(record.get("position"), dict)
                    and (
                        entry["_source"] != "agent"
                        or entry["position"] is None
                    )
                ):
                    entry["position"] = record["position"]

                if entry["_source"] == "webots":
                    entry["agentOnline"] = False
                    entry["status"] = "NO_AGENT"
                    entry["available"] = False
                    entry["availabilityReason"] = "No live robot agent"
            for key, entry in self._robots.items():
                if entry["_source"] == "webots" and key not in seen:
                    entry["present"] = False
                    entry["agentOnline"] = False
                    entry["available"] = False

    def update_agent(self, payload):
        robot_id = str(
            payload.get("robotId") or payload.get("serialNumber") or ""
        ).strip()
        if not robot_id:
            return False

        manufacturer = payload.get("manufacturer") or "UNKNOWN"
        with self._lock:
            entry = self._get_or_create(manufacturer, robot_id)

            session_id = payload.get("agentSessionId")
            if session_id:
                known_session = entry["_agent_session_id"]
                if known_session and session_id != known_session:
                    return False
                if known_session is None:
                    entry["_agent_session_id"] = session_id

            sequence = payload.get("statusSequence")
            if sequence is not None:
                try:
                    sequence = int(sequence)
                except (TypeError, ValueError):
                    return False
                if sequence <= entry["_status_sequence"]:
                    return False
                entry["_status_sequence"] = sequence

            state = payload.get("state") or payload.get("status")
            incoming_order = payload.get("orderId")
            recovered_order = payload.get("recoveredOrderId")
            if (
                recovered_order
                and recovered_order != entry["currentOrder"]
                and not (
                    entry["recoveryRequired"]
                    and recovered_order == entry["lastOrderId"]
                )
            ):
                return False
            if (
                entry["currentOrder"]
                and incoming_order != entry["currentOrder"]
                and not (
                    state == "IDLE"
                    and payload.get("recoveredOrderId")
                    == entry["currentOrder"]
                )
            ):
                if state != "IDLE" or incoming_order is not None:
                    return False
                if sequence is not None:
                    entry["_status_sequence"] = sequence
                entry["_source"] = "agent"
                entry["present"] = True
                entry["agentOnline"] = True
                entry["_last_seen"] = time.monotonic()
                entry["lastUpdate"] = self._timestamp()
                entry["status"] = "IDLE"
                entry["recoveryRequired"] = True
                entry["recoveryReason"] = (
                    "Robot reports IDLE but the fleet still holds an order assignment"
                )
                if "position" in payload:
                    entry["position"] = payload["position"]
                self._refresh_availability(entry)
                return True
            if (
                not entry["currentOrder"]
                and state == "NODE_REACHED"
                and incoming_order == entry["lastOrderId"]
                and entry["lastOrderStatus"] == "COMPLETED"
            ):
                entry["_source"] = "agent"
                entry["present"] = True
                entry["agentOnline"] = True
                entry["_last_seen"] = time.monotonic()
                entry["lastUpdate"] = self._timestamp()
                if "position" in payload:
                    entry["position"] = payload["position"]
                if payload.get("currentNode") or payload.get("nodeId"):
                    entry["currentNode"] = (
                        payload.get("currentNode") or payload["nodeId"]
                    )
                self._refresh_availability(entry)
                return True
            if (
                not entry["currentOrder"]
                and state in {"EXECUTING", "NODE_REACHED"}
                and incoming_order
                and incoming_order == entry["lastOrderId"]
            ):
                return False

            entry["_source"] = "agent"
            entry["present"] = True
            entry["agentOnline"] = True
            entry["_last_seen"] = time.monotonic()
            entry["lastUpdate"] = self._timestamp()
            for field in (
                "robotType",
                "currentNode",
                "position",
                "currentEdge",
                "capabilities",
                "availableDevices",
                "agentSessionId",
                "transportCapability",
            ):
                if field in payload:
                    entry[field] = payload[field]
            if state:
                entry["status"] = state

            if payload.get("recoveredOrderId"):
                recovered_order_id = payload["recoveredOrderId"]
                if (
                    recovered_order_id == entry["currentOrder"]
                    or (
                        entry["recoveryRequired"]
                        and recovered_order_id == entry["lastOrderId"]
                    )
                ):
                    entry["lastOrderId"] = recovered_order_id
                    entry["lastOrderStatus"] = "INTERRUPTED"
                    entry["currentOrder"] = None
                    entry["recoveryRequired"] = False
                    entry["recoveryPending"] = False
                    entry["recoveryReason"] = None
            elif state in {"STOPPED", "ERROR"}:
                if incoming_order:
                    entry["lastOrderId"] = incoming_order
                entry["lastOrderStatus"] = state
                entry["currentOrder"] = None
                entry["recoveryRequired"] = state == "ERROR"
                entry["recoveryReason"] = (
                    "Robot reported an error; operator recovery is required"
                    if state == "ERROR"
                    else None
                )
            elif state == "IDLE":
                if incoming_order is None or incoming_order == entry["currentOrder"]:
                    if entry["currentOrder"]:
                        entry["lastOrderId"] = entry["currentOrder"]
                        entry["lastOrderStatus"] = "INTERRUPTED"
                    entry["currentOrder"] = None
                    if not entry["recoveryRequired"]:
                        entry["recoveryReason"] = None
            elif incoming_order:
                entry["currentOrder"] = incoming_order

            self._refresh_availability(entry)
        return True

    def register_agent(self, payload):
        robot_id = str(payload.get("robotId") or "").strip()
        if not robot_id:
            return False

        manufacturer = payload.get("manufacturer") or "UNKNOWN"
        session_id = payload.get("agentSessionId")
        with self._lock:
            entry = self._get_or_create(manufacturer, robot_id)
            previous_session = entry["_agent_session_id"]
            if session_id and session_id != previous_session:
                if entry["currentOrder"]:
                    entry["recoveryRequired"] = True
                    entry["recoveryReason"] = (
                        "Robot controller restarted during an active assignment"
                    )
                entry["_agent_session_id"] = session_id
                entry["_status_sequence"] = -1
                entry["agentSessionId"] = session_id
            entry["_source"] = "agent"
            entry["present"] = True
            entry["agentOnline"] = True
            entry["_last_seen"] = time.monotonic()
            entry["lastUpdate"] = self._timestamp()
            for field in (
                "robotType",
                "capabilities",
                "availableDevices",
                "transportCapability",
            ):
                if field in payload:
                    entry[field] = payload[field]
            if not entry["currentOrder"] and not entry["recoveryRequired"]:
                entry["status"] = "IDLE"
            self._refresh_availability(entry)
        return True

    def mark_recovery_required(self, manufacturer, robot_id, reason):
        with self._lock:
            entry = self._robots.get(self._key(manufacturer, robot_id))
            if entry is None:
                return False
            entry["recoveryRequired"] = True
            entry["recoveryReason"] = reason
            self._refresh_availability(entry)
            return True

    def begin_recovery(self, manufacturer, robot_id, order_id):
        with self._lock:
            entry = self._robots.get(self._key(manufacturer, robot_id))
            if (
                entry is None
                or not entry["agentOnline"]
                or not entry["recoveryRequired"]
                or entry["currentOrder"] != order_id
            ):
                return False
            entry["recoveryPending"] = True
            entry["recoveryReason"] = "Waiting for robot recovery acknowledgement"
            self._refresh_availability(entry)
            return True

    def update_status(self, payload):
        return self.update_agent(payload)

    def set_state(
        self,
        manufacturer,
        robot_id,
        state,
        current_order=None,
        current_node=None,
        current_edge=None,
    ):
        with self._lock:
            entry = self._get_or_create(manufacturer, robot_id)
            entry["_source"] = "agent"
            entry["status"] = state
            entry["agentOnline"] = True
            entry["_last_seen"] = time.monotonic()
            entry["lastUpdate"] = self._timestamp()
            if state in {"IDLE", "STOPPED", "ERROR"}:
                if entry["currentOrder"]:
                    entry["lastOrderId"] = entry["currentOrder"]
                elif current_order:
                    entry["lastOrderId"] = current_order
                entry["lastOrderStatus"] = {
                    "IDLE": "COMPLETED",
                    "STOPPED": "STOPPED",
                    "ERROR": "FAILED",
                }[state]
                entry["currentOrder"] = None
                entry["recoveryPending"] = False
                entry["recoveryRequired"] = state == "ERROR"
                entry["recoveryReason"] = (
                    "Robot reported an error; operator recovery is required"
                    if state == "ERROR"
                    else None
                )
            else:
                entry["currentOrder"] = current_order
                entry["recoveryPending"] = False
                if current_order:
                    entry["lastOrderId"] = current_order
            if current_edge is not None or state in {"IDLE", "STOPPED", "ERROR"}:
                entry["currentEdge"] = current_edge
            if current_node is not None:
                entry["currentNode"] = current_node
            self._refresh_availability(entry)

    def get(self, manufacturer, robot_id):
        with self._lock:
            entry = self._robots.get(self._key(manufacturer, robot_id))
            if entry is None:
                return None
            self._expire_if_stale(entry)
            return self._public_entry(entry)

    def snapshot(self):
        with self._lock:
            for entry in self._robots.values():
                self._expire_if_stale(entry)
            return [
                self._public_entry(entry)
                for entry in sorted(
                    self._robots.values(),
                    key=lambda item: (item["manufacturer"], item["robotId"]),
                )
            ]

    def expire_stale_agents(self):
        changed = False
        with self._lock:
            for entry in self._robots.values():
                was_online = entry["agentOnline"]
                self._expire_if_stale(entry)
                changed = changed or was_online != entry["agentOnline"]
        return changed

    def _expire_if_stale(self, entry):
        if (
            entry["agentOnline"]
            and time.monotonic() - entry["_last_seen"]
            > self._heartbeat_timeout_seconds
        ):
            entry["agentOnline"] = False
            entry["status"] = "OFFLINE"
            entry["available"] = False
            entry["availabilityReason"] = "Robot heartbeat timed out"
            entry["recoveryRequired"] = bool(entry["currentOrder"])
            if entry["recoveryRequired"]:
                entry["recoveryReason"] = (
                    "Robot disconnected while assigned to an order"
                )

    @staticmethod
    def _timestamp():
        return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace(
            "+00:00", "Z"
        )

    @staticmethod
    def _refresh_availability(entry):
        if not entry["present"]:
            reason = "Robot is not present in the Webots world"
        elif not entry["agentOnline"]:
            reason = "No live robot agent"
        elif entry.get("recoveryRequired"):
            reason = entry.get("recoveryReason") or "Robot requires recovery"
        elif entry.get("recoveryPending"):
            reason = "Waiting for robot recovery acknowledgement"
        elif entry.get("currentOrder"):
            reason = f"Active mission {entry['currentOrder']}"
        elif entry["status"] not in {"IDLE", "STOPPED"}:
            reason = f"Robot state is {entry['status']}"
        elif "NAVIGATION" not in entry["capabilities"]:
            reason = "Navigation capability is not implemented"
        else:
            reason = None

        entry["availabilityReason"] = reason
        entry["available"] = (
            reason is None
            and entry["status"] in {"IDLE", "STOPPED"}
        )

    @staticmethod
    def _public_entry(entry):
        return {
            key: value
            for key, value in entry.items()
            if not key.startswith("_")
        }
