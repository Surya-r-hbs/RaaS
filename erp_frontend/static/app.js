let data = null;


function escapeHtml(value) {
  return String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#39;");
}


// --------------------
// REFRESH
// --------------------

async function refresh() {

  try {

    const response = await fetch(
      "/api/dashboard"
    );

    if (!response.ok) {

      throw new Error(
        "Dashboard request failed"
      );
    }

    data = await response.json();

    render();

  } catch (error) {

    console.error(error);
  }
}


// --------------------
// STATUS BADGE
// --------------------

function badge(status) {

  return (
    `<span class="badge ${status}">`
    + status
    + "</span>"
  );
}


// --------------------
// ROUTE
// --------------------

function route(order) {

  return (order.nodes || [])
    .map(node => node.nodeId)
    .join(" → ");
}


function populateRobotSelector() {

  const select = document.getElementById("robot-id");

  if (!select) {
    return;
  }

  const currentValue = select.value;
  const robots = data && Array.isArray(data.robots) ? data.robots : [];

  if (!robots.length) {
    select.innerHTML = '<option value="">Waiting for robot discovery...</option>';
    return;
  }

  const available = robots.filter(robot => robot && robot.robotId);

  select.innerHTML = available
    .map(robot => `
      <option
        value="${encodeURIComponent(robot.manufacturer)}::${encodeURIComponent(robot.robotId)}"
        ${robot.available ? "" : "disabled"}
      >
        ${robot.robotId} — ${robot.robotType || "Unknown"} —
        ${robot.manufacturer} — ${robot.available ? "Available" : robot.availabilityReason || robot.status}
      </option>
    `)
    .join("");

  select.onchange = updateLocationPreview;

  if (currentValue && available.some(robot => selectorValue(robot) === currentValue)) {
    select.value = currentValue;
    return;
  }

  const firstAvailable = available.find(robot => robot.available);
  select.value = firstAvailable ? selectorValue(firstAvailable) : "";
}

function selectorValue(robot) {
  return `${encodeURIComponent(robot.manufacturer)}::${encodeURIComponent(robot.robotId)}`;
}

function selectedRobot() {
  if (!data) {
    return null;
  }
  const value = document.getElementById("robot-id")?.value || "";
  return data.robots.find(robot => selectorValue(robot) === value) || null;
}


// --------------------
// ROBOT STOP BUTTON
// --------------------

function robotStopButton(robot) {

  const stoppable = [
    "EXECUTING",
    "DISPATCHED",
  ].includes(robot.status);

  if (!stoppable) {

    if (
      robot.status === "STOPPING"
    ) {

      return `
        <button disabled>
          Stopping...
        </button>
      `;
    }

    return "";
  }

  return `
    <button
      class="stop-button"
      onclick="stopRobot('${robot.manufacturer}', '${robot.robotId}')"
    >
      Stop Robot
    </button>
  `;
}

function robotRecoveryButton(robot) {

  if (!robot.recoveryRequired || !(robot.currentOrder || robot.lastOrderId)) {
    return "";
  }

  return `
    <button
      class="stop-button"
      onclick="recoverRobot('${robot.manufacturer}', '${robot.robotId}')"
    >
      ${robot.recoveryPending ? "Retry Robot Recovery" : "Recover Robot"}
    </button>
  `;
}


// --------------------
// RENDER
// --------------------

function bindLogFilters() {
  const search = document.getElementById("logs-search");
  const category = document.getElementById("logs-category");
  const severity = document.getElementById("logs-severity");

  if (!search || !category || !severity) {
    return;
  }

  search.oninput = renderLogs;
  category.onchange = renderLogs;
  severity.onchange = renderLogs;
}

function renderLogs() {
  const list = document.getElementById("logs-list");
  if (!list) {
    return;
  }

  const logs = Array.isArray(data?.logs) ? data.logs : [];
  const search = (document.getElementById("logs-search")?.value || "").trim().toLowerCase();
  const category = (document.getElementById("logs-category")?.value || "").trim().toUpperCase();
  const severity = (document.getElementById("logs-severity")?.value || "").trim().toUpperCase();

  const filteredLogs = logs.filter(log => {
    const entryCategory = (log.category || "ERP").toUpperCase();
    const entrySeverity = (log.severity || "INFO").toUpperCase();
    const haystack = [
      entryCategory,
      log.categoryLabel || "",
      log.source || "",
      log.message || "",
      log.type || "",
    ].join(" ").toLowerCase();

    if (category && entryCategory !== category) {
      return false;
    }
    if (severity && entrySeverity !== severity) {
      return false;
    }
    if (search && !haystack.includes(search)) {
      return false;
    }
    return true;
  });

  list.innerHTML = filteredLogs
    .map(log => `
      <div class="event">
        <strong>${escapeHtml(log.categoryLabel || log.category || "ERP")}</strong>
        <span>${escapeHtml(log.severity || "INFO")}</span>
        <span>${escapeHtml(log.timestamp || "-")}</span>
        <div>${escapeHtml(log.message || "No message")}</div>
        ${log.source ? `<div><small>Source: ${escapeHtml(log.source)}</small></div>` : ""}
      </div>
    `)
    .join("") || "<p>No logs available.</p>";
}

function render() {

  if (!data) {

    return;
  }

  bindLogFilters();
  renderLogs();
  populateRobotSelector();
  updateLocationPreview();

  // --------------------
  // SUMMARY
  // --------------------

  document.getElementById(
    "total"
  ).textContent =
    data.summary.totalOrders;

  document.getElementById(
    "active"
  ).textContent =
    data.summary.activeOrders;

  document.getElementById(
    "completed"
  ).textContent =
    data.summary.completedOrders;

  document.getElementById(
    "available"
  ).textContent =
    data.summary.availableRobots;


  // --------------------
  // DASHBOARD ROBOTS
  // --------------------

  document.getElementById(
    "robot-list"
  ).innerHTML =

    data.robots
      .map(robot => `

        <div class="robot-item">

          <div class="robot-row">

            <strong>
              ${robot.robotId}
            </strong>

            ${badge(robot.status)}

          </div>

          <div>
            Type:
            ${robot.robotType || "Unknown"}
          </div>

          <div>
            Availability:
            ${robot.available ? "Available" : robot.availabilityReason || "Unavailable"}
          </div>

          ${
            robot.lastError
              ? `<div>Error: ${escapeHtml(robot.lastError)}</div>`
              : ""
          }

          <div>
            Location:
            ${escapeHtml(robotLocationLabel(robot))}
          </div>

          <div>
            Edge:
            ${robot.currentEdge || "-"}
          </div>

          <div>
            Mission:
            ${robot.currentOrder || "None"}
          </div>

          <div>
            Last update:
            ${robot.lastUpdate || "-"}
          </div>

          <div style="margin-top: 10px;">
            ${robotStopButton(robot)}
            ${robotRecoveryButton(robot)}
          </div>

        </div>

      `)
      .join("");


  // --------------------
  // RECENT ORDERS
  // --------------------

  document.getElementById(
    "recent"
  ).innerHTML =

    data.orders
      .slice()
      .reverse()
      .slice(0, 5)
      .map(order => `

        <div class="robot-item">

          <div class="robot-row">

            <strong>
              ${order.orderId}
            </strong>

            ${badge(order.status)}

          </div>

          <div>
            Robot:
            ${order.serialNumber}
          </div>

          <div>
            Route:
            ${route(order)}
          </div>

        </div>

      `)
      .join("")

    || "<p>No orders.</p>";


  // --------------------
  // ORDERS TABLE
  // --------------------

  document.getElementById(
    "orders-table"
  ).innerHTML =

    data.orders
      .map(order => `

        <tr>

          <td>
            <strong>
              ${order.orderId}
            </strong>
          </td>

          <td>
            ${order.serialNumber}
          </td>

          <td>
            ${order.robotType || "Unknown"}
          </td>

          <td>
            ${order.erpPriority || "NORMAL"}
          </td>

          <td>
            ${route(order)}
          </td>

          <td>
            ${badge(order.status)}
          </td>

          <td>

            ${
              [
                "CREATED",
                "FAILED",
                "STOPPED",
              ].includes(order.status)

              ? `
                <button
                  onclick="dispatchOrder(
                    '${order.orderId}'
                  )"
                >
                  Dispatch
                </button>
              `

              : ""
            }

          </td>

        </tr>

      `)
      .join("");


  // --------------------
  // ROBOT GRID
  // --------------------

  document.getElementById(
    "robot-grid"
  ).innerHTML =

    data.robots
      .map(robot => `

        <div class="robot-card">

          <h3>
            ${robot.robotId}
          </h3>

          <p>
            Manufacturer:
            ${robot.manufacturer}
          </p>

          <p>
            Type:
            ${robot.robotType || "Unknown"}
          </p>

          <p>
            Status:
            ${badge(robot.status)}
          </p>

          <p>
            Availability:
            ${robot.available ? "Available" : robot.availabilityReason || "Unavailable"}
          </p>

          ${
            robot.recoveryRequired
              ? `<p>Recovery: ${escapeHtml(robot.recoveryReason || "Required")}</p>`
              : ""
          }

          ${
            robot.lastError
              ? `<p>Last error: ${escapeHtml(robot.lastError)}</p>`
              : ""
          }

          <p>
            Capabilities:
            ${(robot.capabilities || []).join(", ") || "None"}
          </p>

          <p>
            Devices:
            ${(robot.availableDevices || []).join(", ") || "None reported"}
          </p>

          <p>
            Current location:
            <b>
              ${escapeHtml(robotLocationLabel(robot))}
            </b>
          </p>

          <p>
            Current edge:
            <b>
              ${robot.currentEdge || "-"}
            </b>
          </p>

          <p>
            Mission:
            ${robot.currentOrder || "None"}
          </p>

          <div style="margin-top: 15px;">
            ${robotStopButton(robot)}
            ${robotRecoveryButton(robot)}
          </div>

        </div>

      `)
      .join("");


  // --------------------
  // ACTIVE MISSIONS
  // --------------------

  const assignedOrderIds = new Set(
    data.robots
      .map(robot => robot.currentOrder)
      .filter(Boolean)
  );
  const activeOrders = data.orders.filter(
    order => assignedOrderIds.has(order.orderId)
  );


  document.getElementById(
    "missions-list"
  ).innerHTML =

    activeOrders
      .map(order => `

        <div class="mission-item">

          <strong>
            ${order.orderId}
          </strong>

          <p>
            Robot:
            ${order.serialNumber}
          </p>

          <p>
            Route:
            ${route(order)}
          </p>

          <p>
            Status:
            ${badge(order.status)}
          </p>

        </div>

      `)
      .join("")

    || "<p>No active missions.</p>";


  // --------------------
  // EVENTS
  // --------------------

  document.getElementById(
    "events-list"
  ).innerHTML =

    data.events
      .map(event => `

        <div class="event">

          <strong>
            ${event.type}
          </strong>

          <span>
            ${event.timestamp}
          </span>

          <div>
            ${event.message}
          </div>

        </div>

      `)
      .join("")

    || "<p>No events.</p>";

  if (typeof renderTransportTasks === "function") {
    renderTransportTasks(data.transportTasks || []);
  }
}


// --------------------
// STOP ROBOT
// --------------------

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


// --------------------
// PAGE NAVIGATION
// --------------------

function showPage(page) {

  document
    .querySelectorAll(".page")
    .forEach(element =>
      element.classList.remove(
        "active"
      )
    );

  document
    .querySelectorAll(".nav")
    .forEach(element =>
      element.classList.remove(
        "active"
      )
    );

  document
    .getElementById(page)
    .classList.add("active");


  const pages = [
    "dashboard",
    "orders",
    "robots",
    "objects",
    "transport",
    "missions",
    "logs",
    "events",
  ];


  document
    .querySelectorAll(".nav")[
      pages.indexOf(page)
    ]
    .classList.add("active");


  document.getElementById(
    "title"
  ).textContent = {

    dashboard:
      "Operations Dashboard",

    orders:
      "Order Management",

    robots:
      "Robot Fleet",

    objects:
      "Object Inventory",

    transport:
      "Transport Tasks",

    missions:
      "Mission Control",

    logs:
      "Operational Logs",

    events:
      "Live System Events",

  }[page];


  render();
  if (page === "objects" && typeof refreshObjects === "function") {
    refreshObjects();
  }
  if (page === "transport" && typeof refreshTransportOptions === "function") {
    refreshTransportOptions();
  }
}


// --------------------
// ORDER MODAL
// --------------------

async function openModal() {

  await refresh();

  document
    .getElementById("modal")
    .classList.remove(
      "hidden"
    );
}

function updateLocationPreview() {

  const preview = document.getElementById(
    "start-location"
  );

  if (!preview || !data || !data.robots.length) {
    return;
  }

  const robot = selectedRobot();
  if (!robot || !robot.position) {
    preview.textContent =
      "Current location: waiting for live telemetry...";
    return;
  }

  preview.textContent =
    `Current location: ${robotLocationLabel(robot)}. `
    + `Updated ${robot.lastUpdate || "just now"}`;
}

function robotLocationLabel(robot) {
  const position = robot?.position;
  if (
    !position
    || !Number.isFinite(Number(position.x))
    || !Number.isFinite(Number(position.y))
  ) {
    return "Waiting for live telemetry";
  }

  const location = robot.currentNode || "Off graph";
  const nearestNode = !robot.currentNode && robot.nearestNode
    ? `; nearest route node ${robot.nearestNode}`
      + (Number.isFinite(Number(robot.nearestNodeDistance))
        ? ` (${Number(robot.nearestNodeDistance).toFixed(1)} m away)`
        : "")
    : "";
  return `${location}${nearestNode} `
    + `(X ${Number(position.x).toFixed(2)}, `
    + `Y ${Number(position.y).toFixed(2)})`;
}

function updateOrderFields() {

  const mode = document.getElementById(
    "planning-mode"
  ).value;

  document.getElementById(
    "route-field"
  ).classList.toggle(
    "hidden",
    mode === "destination"
  );

  document.getElementById(
    "destination-field"
  ).classList.toggle(
    "hidden",
    mode === "route"
  );
}


function closeModal() {

  document
    .getElementById("modal")
    .classList.add(
      "hidden"
    );
}


// --------------------
// CREATE ORDER
// --------------------

async function createOrder() {

  const orderId =
    document
      .getElementById("order-id")
      .value
      .trim();


  const robot = selectedRobot();

  if (!robot || !robot.available) {
    alert("Select an available robot.");
    return;
  }


  const priority =
    document.getElementById(
      "priority"
    ).value;

  const planningMode =
    document.getElementById(
      "planning-mode"
    ).value;

  const routeNodes =
    document
      .getElementById("route")
      .value
      .split(",")
      .map(
        value => value.trim()
      )
      .filter(Boolean);

  const destination =
    document
      .getElementById("destination")
      .value
      .trim();


  if (!orderId) {

    alert(
      "Enter an order ID."
    );

    return;
  }

  if (
    (planningMode === "route" && !routeNodes.length)
    || (planningMode === "destination" && !destination)
    || (planningMode === "both" && (!routeNodes.length || !destination))
  ) {
    alert("Fill in the fields required by the selected planning mode.");
    return;
  }

  try {
    const response = await fetch(
      "/api/orders",
      {
        method: "POST",

        headers: {
          "Content-Type":
            "application/json",
        },

        body: JSON.stringify({
          orderId,
          robotId: robot.robotId,
          manufacturer: robot.manufacturer,
          priority,
          planningMode,
          route: routeNodes,
          destination,
        }),
      }
    );


    const result =
      await response.json();


    if (!response.ok) {

      alert(
        result.error
        || "Order creation failed"
      );

      return;
    }


    closeModal();


    document.getElementById(
      "order-id"
    ).value = "";


    document.getElementById("route").value = "";
    document.getElementById("destination").value = "";

    await refresh();

    showPage("orders");
  } catch (error) {
    console.error(error);
    alert("Order creation request failed.");
  }
}


// --------------------
// DISPATCH ORDER
// --------------------

async function dispatchOrder(
  orderId
) {

  const response =
    await fetch(
      `/api/orders/${encodeURIComponent(
        orderId
      )}/dispatch`,
      {
        method: "POST",
      }
    );


  const result =
    await response.json();


  if (!response.ok) {

    alert(
      result.error
      || "Dispatch failed"
    );

    return;
  }


  await refresh();
}


// --------------------
// INITIAL LOAD
// --------------------

refresh();


// --------------------
// AUTO REFRESH
// --------------------

setInterval(
  refresh,
  2000
);