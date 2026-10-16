async function stopRobot(
  manufacturer,
  robotId
) {

  const confirmed = confirm(
    `Stop robot ${robotId}?`
  );

  if (!confirmed) {

    return;
  }

  try {

    const response = await fetch(
      `/api/robots/${encodeURIComponent(manufacturer)}/${encodeURIComponent(
        robotId
      )}/stop`,
      {
        method: "POST",
      }
    );

    const result =
      await response.json();

    if (!response.ok) {

      alert(
        result.error
        || "Robot stop failed"
      );

      return;
    }

    await refresh();

  } catch (error) {

    console.error(error);

    alert(
      "Failed to send stop command"
    );
  }
}

async function recoverRobot(manufacturer, robotId) {

  const robot = data?.robots.find(
    item => item.manufacturer === manufacturer && item.robotId === robotId
  );
  const orderId = robot?.currentOrder || robot?.lastOrderId;
  if (!orderId) {
    alert("No recoverable order is available for this robot.");
    return;
  }

  const confirmed = confirm(
    `Recover ${robotId} from order ${orderId}?\n\n`
    + "Confirm the robot's area is clear and it is safe to stop its motors. "
    + "The assignment will remain blocked until the robot acknowledges recovery."
  );
  if (!confirmed) {
    return;
  }

  const reason = prompt(
    "Enter the reason for this recovery:",
    "Verified robot is safe; reconciling stale mission state"
  );
  if (!reason || !reason.trim()) {
    return;
  }

  try {
    const response = await fetch(
      `/api/robots/${encodeURIComponent(manufacturer)}/${encodeURIComponent(robotId)}/recover`,
      {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({confirmSafe: true, reason: reason.trim()}),
      }
    );
    const result = await response.json();
    if (!response.ok && response.status !== 202) {
      alert(result.error || "Robot recovery failed");
      return;
    }
    alert(result.message || "Waiting for robot recovery acknowledgement.");
    await refresh();
  } catch (error) {
    console.error(error);
    alert("Failed to request robot recovery.");
  }
}
