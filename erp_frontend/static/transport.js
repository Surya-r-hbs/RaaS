let transportOptions = {
  objects: [],
  destinations: [],
  carriers: [],
};

function transportEscape(value) {
  return String(value ?? "").replace(/[&<>"']/g, character => ({
    "&": "&amp;",
    "<": "&lt;",
    ">": "&gt;",
    '"': "&quot;",
    "'": "&#39;",
  })[character]);
}

async function refreshTransportOptions() {
  const status = document.getElementById("transport-options-status");
  if (!status) {
    return;
  }
  try {
    const response = await fetch("/api/transport-options");
    const payload = await response.json();
    if (!response.ok) {
      status.textContent = payload.error || "Live transport options are unavailable.";
      status.classList.add("inventory-status-error");
      return;
    }
    transportOptions = payload;
    status.textContent =
      "Live Webots inventory and configured map nodes are available. "
      + "Carrier/load compatibility is checked again by the backend on dispatch.";
    status.classList.remove("inventory-status-error");

    const objectSelect = document.getElementById("transport-object");
    const previousObject = objectSelect.value;
    objectSelect.innerHTML = '<option value="">Select an object</option>'
      + (payload.objects || []).map(object => `
        <option value="${transportEscape(object.object_id)}">
          ${transportEscape(object.name)} — ${transportEscape(object.object_id)}
        </option>
      `).join("");
    if ((payload.objects || []).some(item => item.object_id === previousObject)) {
      objectSelect.value = previousObject;
    }

    const destinationSelect = document.getElementById("transport-destination");
    const previousDestination = destinationSelect.value;
    destinationSelect.innerHTML = '<option value="">Select a destination</option>'
      + (payload.destinations || []).map(node => `
        <option value="${transportEscape(node.nodeId)}">
          ${transportEscape(node.nodeId)}
          (X ${Number(node.position.x).toFixed(2)}, Y ${Number(node.position.y).toFixed(2)})
        </option>
      `).join("");
    if ((payload.destinations || []).some(item => item.nodeId === previousDestination)) {
      destinationSelect.value = previousDestination;
    }

    const carrierSelect = document.getElementById("transport-carrier");
    const previousCarrier = carrierSelect.value;
    carrierSelect.innerHTML =
      '<option value="">Use the object’s verified carrier</option>'
      + (payload.carriers || []).map(robot => {
        const value = `${encodeURIComponent(robot.manufacturer)}::${encodeURIComponent(robot.robotId)}`;
        const capability = robot.transportCapability;
        const capabilityLabel = capability?.status === "VERIFIED"
          ? "transport capability verified"
          : "transport capability unverified";
        return `<option value="${transportEscape(value)}">
          ${transportEscape(robot.robotId)} — ${transportEscape(robot.robotType || "Unknown")}
          (${transportEscape(capabilityLabel)})
        </option>`;
      }).join("");
    if ((payload.carriers || []).some(robot => (
      `${encodeURIComponent(robot.manufacturer)}::${encodeURIComponent(robot.robotId)}`
      === previousCarrier
    ))) {
      carrierSelect.value = previousCarrier;
    }
  } catch (error) {
    console.error(error);
    status.textContent = "Could not load transport options from the ERP API.";
    status.classList.add("inventory-status-error");
  }
}

async function createTransportTask() {
  const objectId = document.getElementById("transport-object").value;
  const destinationNode = document.getElementById("transport-destination").value;
  const carrierValue = document.getElementById("transport-carrier").value;
  if (!objectId || !destinationNode) {
    alert("Select an object from live inventory and a destination node.");
    return;
  }
  const [manufacturer, carrierRobotId] = carrierValue
    ? carrierValue.split("::").map(decodeURIComponent)
    : ["", ""];
  try {
    const response = await fetch("/api/transport-tasks", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({
        taskType: "TRANSPORT_OBJECT",
        objectId,
        destinationNode,
        handlingMode: "PRELOADED",
        carrierRobotId,
        manufacturer,
      }),
    });
    const result = await response.json();
    if (!response.ok) {
      alert(result.error || "Could not create the transport task.");
      return;
    }
    await refresh();
    renderTransportTasks(data?.transportTasks || []);
    if (result.status === "LOAD_REQUIRED") {
      alert(result.message || "The object is not verified as loaded on a carrier.");
    } else if (result.status === "FAILED") {
      alert(result.message || "Carrier compatibility could not be verified.");
    }
  } catch (error) {
    console.error(error);
    alert("Transport task creation failed.");
  }
}

function renderTransportTasks(tasks) {
  const container = document.getElementById("transport-task-list");
  if (!container) {
    return;
  }
  if (!tasks.length) {
    container.innerHTML = "<p>No transport tasks yet.</p>";
    return;
  }
  container.innerHTML = tasks.map(task => `
    <article class="transport-task">
      <div class="transport-task-heading">
        <strong>${transportEscape(task.taskId)}</strong>
        <span class="badge ${transportEscape(task.status)}">
          ${transportEscape(task.status)}
        </span>
      </div>
      <p><strong>Object:</strong> ${transportEscape(task.objectName || task.objectId)}
        <small>(${transportEscape(task.objectId)})</small></p>
      <p><strong>Destination:</strong> ${transportEscape(task.destinationNode)}</p>
      <p><strong>Carrier:</strong>
        ${transportEscape(task.carrierRobotId || "Not assigned")}
        ${task.manufacturer ? `(${transportEscape(task.manufacturer)})` : ""}
      </p>
      <p><strong>Load state:</strong> ${transportEscape(task.loadState || "UNKNOWN")}</p>
      <p>${transportEscape(task.message || "")}</p>
      ${task.orderId
        ? `<p><strong>Navigation order:</strong> ${transportEscape(task.orderId)}</p>`
        : ""}
      ${(task.history || []).length
        ? `<ol class="transport-task-history">
            ${task.history.map(event => `
              <li>
                <strong>${transportEscape(event.status)}</strong>
                — ${transportEscape(event.message || "")}
                <small>${transportEscape(event.timestamp)}</small>
              </li>
            `).join("")}
          </ol>`
        : ""}
      ${task.status === "READY_FOR_DISPATCH"
        ? `<button class="primary" onclick="dispatchTransportTask('${transportEscape(task.taskId)}')">
            Dispatch transport
          </button>`
        : ""}
    </article>
  `).join("");
}

async function dispatchTransportTask(taskId) {
  try {
    const response = await fetch(
      `/api/transport-tasks/${encodeURIComponent(taskId)}/dispatch`,
      {method: "POST"},
    );
    const result = await response.json();
    if (!response.ok && response.status !== 202) {
      alert(result.error || "Transport dispatch failed.");
    }
    await refresh();
  } catch (error) {
    console.error(error);
    alert("Transport dispatch request failed.");
  }
}

document.addEventListener("DOMContentLoaded", () => {
  if (document.getElementById("transport")) {
    refreshTransportOptions();
  }
});
