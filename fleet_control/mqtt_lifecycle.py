"""MQTT connection lifecycle callbacks for the Fleet controller."""


def handle_connect(
    client,
    userdata,
    flags,
    reason_code,
    properties,
    *,
    broker,
    port,
    objects_raw_topic,
    publish_robot_registry,
    log,
):
    log("FLEET CONTROLLER MQTT CONNECTED")
    log(f"Broker : {broker}")
    log(f"Port   : {port}")

    if reason_code != 0:
        log(f"MQTT connection failed: {reason_code}")
        return

    for topic, label in (
        ("robot/+/+/status", "ROBOT STATUS"),
        ("vda5050/v3/+/+/order", "VDA-5050 ORDER"),
        ("robot/discovery", "ROBOT DISCOVERY"),
        (objects_raw_topic, "WEBOTS OBJECT INVENTORY"),
    ):
        result, message_id = client.subscribe(topic, qos=0)
        log(f"SUBSCRIBED TO {label}")
        log(f"Topic      : {topic}")
        log(f"Result     : {result}")
        log(f"Message ID : {message_id}")

    publish_robot_registry()


def handle_disconnect(
    client,
    userdata,
    disconnect_flags,
    reason_code,
    properties,
    *,
    log,
):
    log("FLEET CONTROLLER MQTT DISCONNECTED")
    log(f"Reason code: {reason_code}")
