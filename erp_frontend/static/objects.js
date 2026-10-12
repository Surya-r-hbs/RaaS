let objectData = {
  objects: [],
  relationships: [],
  simulationConnected: false,
};
let selectedObjectId = null;
let objectRefreshInProgress = false;

function objectEscape(value) {
  return String(value ?? "").replace(/[&<>"']/g, character => ({
    "&": "&amp;",
    "<": "&lt;",
    ">": "&gt;",
    '"': "&quot;",
    "'": "&#39;",
  })[character]);
}

function objectPosition(object) {
  const position = object.position || {};
  return ["x", "y", "z"]
    .map(axis => Number.isFinite(Number(position[axis]))
      ? Number(position[axis]).toFixed(3)
      : "-")
    .join(", ");
}

function objectDimensions(object) {
  const dimensions = object.dimensions;
  if (!dimensions) {
    return "Unknown";
  }
  return ["x", "y", "z"]
    .map(axis => Number.isFinite(Number(dimensions[axis]))
      ? Number(dimensions[axis]).toFixed(3)
      : "-")
    .join(" × ");
}

function formatObjectTime(timestamp) {
  if (!timestamp) {
    return "-";
  }
  const date = new Date(timestamp);
  return Number.isNaN(date.getTime()) ? timestamp : date.toLocaleString();
}

function renderObjectInventory() {
  const objects = Array.isArray(objectData.objects) ? objectData.objects : [];
  const search = (document.getElementById("object-search").value || "")
    .trim()
    .toLowerCase();
  const typeFilter = document.getElementById("object-type-filter").value;
  const filtered = objects.filter(object => {
    const matchesSearch = !search
      || `${object.name} ${object.object_id} ${object.object_type}`
        .toLowerCase()
        .includes(search);
    return matchesSearch && (!typeFilter || object.object_type === typeFilter);
  });

  document.getElementById("object-count").textContent = objects.length;
  document.getElementById("relationship-count").textContent =
    (objectData.relationships || []).length;
  document.getElementById("object-refresh-time").textContent =
    formatObjectTime(objectData.lastRefresh);
  document.getElementById("object-refresh-time").title =
    objectData.lastRefresh || "";
  document.getElementById("object-simulation-status").textContent =
    objectData.simulationConnected ? "Connected" : "Disconnected";
  document.getElementById("object-simulation-status").classList.toggle(
    "inventory-online",
    Boolean(objectData.simulationConnected),
  );
  document.getElementById("object-simulation-status").classList.toggle(
    "inventory-offline",
    !objectData.simulationConnected,
  );
  document.getElementById("object-coordinate-system").textContent =
    `Coordinates: ${objectData.coordinateSystem || "X/Y floor plane; Z height"}`;

  const types = [...new Set(objects.map(object => object.object_type).filter(Boolean))]
    .sort();
  const typeSelect = document.getElementById("object-type-filter");
  const selectedType = typeSelect.value;
  typeSelect.innerHTML = '<option value="">All object types</option>'
    + types.map(type => `<option value="${objectEscape(type)}">${objectEscape(type)}</option>`)
      .join("");
  if (types.includes(selectedType)) {
    typeSelect.value = selectedType;
  }

  const table = document.getElementById("objects-table");
  table.innerHTML = filtered.map(object => `
    <tr data-object-id="${objectEscape(object.object_id)}"
        class="${object.object_id === selectedObjectId ? "selected-object" : ""}">
      <td>${objectEscape(object.name)}</td>
      <td>${objectEscape(object.object_type)}</td>
      <td>${objectEscape(object.object_id)}</td>
      <td>${objectEscape(object.position?.x?.toFixed?.(3) ?? "-")}</td>
      <td>${objectEscape(object.position?.y?.toFixed?.(3) ?? "-")}</td>
      <td>${objectEscape(object.position?.z?.toFixed?.(3) ?? "-")}</td>
      <td>${objectEscape(objectDimensions(object))}</td>
      <td>${objectEscape(object.status || "UNKNOWN")}</td>
      <td>${(object.relationships || []).length}</td>
      <td>${objectEscape(formatObjectTime(object.last_seen))}</td>
    </tr>
  `).join("") || `
    <tr><td colspan="10">No objects match the current filter.</td></tr>
  `;

  if (!objects.some(object => object.object_id === selectedObjectId)) {
    selectedObjectId = objects[0]?.object_id || null;
  }
  renderObjectDetails();
  renderObjectHierarchy();
}

function renderObjectDetails() {
  const details = document.getElementById("object-details");
  const objects = Array.isArray(objectData.objects) ? objectData.objects : [];
  const object = objects.find(item => item.object_id === selectedObjectId);
  if (!object) {
    details.innerHTML = "<p>Select an object to inspect its position and relationships.</p>";
    return;
  }

  const byId = new Map(objects.map(item => [item.object_id, item]));
  const relationships = [];
  for (const relation of object.relationships || []) {
    const target = byId.get(relation.target_object_id);
    relationships.push(
      `${relation.type} ${target ? target.name : relation.target_object_id}`,
    );
  }
  for (const source of objects) {
    for (const relation of source.relationships || []) {
      if (relation.target_object_id === object.object_id) {
        relationships.push(`${source.name} ${relation.type} this object`);
      }
    }
  }

  details.innerHTML = `
    <h3>${objectEscape(object.name)}</h3>
    <p><strong>Type:</strong> ${objectEscape(object.object_type)}</p>
    <p><strong>Object ID:</strong> ${objectEscape(object.object_id)}</p>
    <p><strong>Position:</strong> X ${objectEscape(object.position.x.toFixed(3))},
      Y ${objectEscape(object.position.y.toFixed(3))},
      Z ${objectEscape(object.position.z.toFixed(3))}</p>
    <p><strong>Reference:</strong> ${objectEscape(object.position_reference || "PROTO_ORIGIN")}</p>
    <p><strong>Dimensions:</strong> ${objectEscape(objectDimensions(object))}</p>
    <p><strong>Geometry:</strong> ${objectEscape(object.geometry_status || "UNKNOWN")}
      ${object.geometry?.source
        ? `(${objectEscape(object.geometry.source)})`
        : "(dimensions unavailable from Supervisor)"}</p>
    <p><strong>PROTO / node type:</strong>
      ${objectEscape(object.proto_type || object.node_type || "unknown")}</p>
    <p><strong>Category:</strong>
      ${objectEscape(object.semantic_category || "unknown")}</p>
    <p><strong>Status:</strong> ${objectEscape(object.status || "UNKNOWN")}</p>
    <p><strong>Source:</strong> ${objectEscape(object.source || "Unknown")}</p>
    <p><strong>Last seen:</strong> ${objectEscape(formatObjectTime(object.last_seen))}</p>
    <p><strong>Simulation session:</strong>
      ${objectEscape(object.simulation_session_id || "Unavailable")}</p>
    <p><strong>Relationships:</strong></p>
    ${relationships.length
      ? `<ul>${relationships.map(value => `<li>${objectEscape(value)}</li>`).join("")}</ul>`
      : "<p>No verified spatial relationships.</p>"}
  `;
}

function renderObjectHierarchy() {
  const container = document.getElementById("object-hierarchy");
  const objects = Array.isArray(objectData.objects) ? objectData.objects : [];
  const byId = new Map(objects.map(object => [object.object_id, object]));
  const children = new Map(objects.map(object => [object.object_id, []]));
  const externalParents = new Map();
  const hasParent = new Set();

  for (const object of objects) {
    for (const relation of object.relationships || []) {
      if (
        ["ON_TOP_OF", "INSIDE"].includes(relation.type)
        && byId.has(relation.target_object_id)
      ) {
        children.get(relation.target_object_id).push({
          objectId: object.object_id,
          type: relation.type,
        });
        hasParent.add(object.object_id);
      } else if (
        relation.type === "ON_TOP_OF"
        && relation.certainty === "SUPERVISOR_GEOMETRY"
        && relation.target_object_id
      ) {
        const branches = externalParents.get(relation.target_object_id) || [];
        branches.push({
          objectId: object.object_id,
          type: relation.type,
        });
        externalParents.set(relation.target_object_id, branches);
        hasParent.add(object.object_id);
      }
    }
  }

  const visited = new Set();
  function renderNode(objectId, path, relationType = null) {
    const object = byId.get(objectId);
    if (!object) {
      return "";
    }
    if (path.has(objectId)) {
      return `<li>${objectEscape(object.name)} <span class="hierarchy-note">(cycle)</span></li>`;
    }
    if (visited.has(objectId)) {
      return `<li>${objectEscape(object.name)} <span class="hierarchy-note">(also related)</span></li>`;
    }
    visited.add(objectId);
    const nextPath = new Set(path);
    nextPath.add(objectId);
    const branches = children.get(objectId) || [];
    return `<li>
      ${relationType ? `<span class="relationship-label">${objectEscape(relationType)}</span> ` : ""}
      <button class="hierarchy-object" data-object-id="${objectEscape(objectId)}">${objectEscape(object.name)}</button>
      ${branches.length
        ? `<ul>${branches.map(branch => renderNode(
          branch.objectId,
          nextPath,
          branch.type,
        )).join("")}</ul>`
        : ""}
    </li>`;
  }

  const roots = objects.filter(object => !hasParent.has(object.object_id));
  const tree = roots.map(object => renderNode(object.object_id, new Set())).join("")
    + [...externalParents.entries()].map(([carrierId, branches]) => `
      <li>
        <strong>${objectEscape(carrierId)}</strong>
        <ul>${branches.map(branch => renderNode(
          branch.objectId,
          new Set(),
          branch.type,
        )).join("")}</ul>
      </li>
    `).join("")
    + objects
      .filter(object => !visited.has(object.object_id))
      .map(object => renderNode(object.object_id, new Set()))
      .join("");
  container.innerHTML = tree
    ? `<ul class="object-tree">${tree}</ul>`
    : "<p>No objects are currently present.</p>";
}

async function refreshObjects() {
  const page = document.getElementById("objects");
  if (!page || objectRefreshInProgress) {
    return;
  }
  objectRefreshInProgress = true;
  try {
    const response = await fetch("/api/objects");
    const payload = await response.json();
    objectData = payload;
    const status = document.getElementById("object-inventory-status");
    status.textContent = response.ok
      ? `Live simulation inventory. ${payload.objects.length} objects reported.`
      : payload.error || "Simulation inventory is unavailable.";
    status.classList.toggle("inventory-status-error", !response.ok);
    renderObjectInventory();
  } catch (error) {
    console.error(error);
    objectData.simulationConnected = false;
    const status = document.getElementById("object-inventory-status");
    status.textContent = "Could not reach the ERP object inventory API.";
    status.classList.add("inventory-status-error");
    renderObjectInventory();
  } finally {
    objectRefreshInProgress = false;
  }
}

async function requestObjectRefresh() {
  const button = document.getElementById("refresh-objects");
  button.disabled = true;
  try {
    const response = await fetch("/api/objects/refresh", {method: "POST"});
    const payload = await response.json();
    if (!response.ok) {
      document.getElementById("object-inventory-status").textContent =
        payload.error || "Could not request a simulation refresh.";
      document.getElementById("object-inventory-status")
        .classList.add("inventory-status-error");
      return;
    }
    document.getElementById("object-inventory-status").textContent =
      payload.message || "Refresh requested.";
    await refreshObjects();
  } catch (error) {
    console.error(error);
    document.getElementById("object-inventory-status").textContent =
      "Could not request a simulation refresh.";
    document.getElementById("object-inventory-status")
      .classList.add("inventory-status-error");
  } finally {
    button.disabled = false;
  }
}

document.getElementById("object-search").addEventListener(
  "input",
  renderObjectInventory,
);
document.getElementById("object-type-filter").addEventListener(
  "change",
  renderObjectInventory,
);
document.getElementById("objects-table").addEventListener("click", event => {
  const row = event.target.closest("[data-object-id]");
  if (!row) {
    return;
  }
  selectedObjectId = row.dataset.objectId;
  renderObjectInventory();
});
document.getElementById("object-hierarchy").addEventListener("click", event => {
  const button = event.target.closest(".hierarchy-object");
  if (!button) {
    return;
  }
  selectedObjectId = button.dataset.objectId;
  renderObjectInventory();
});

setInterval(() => {
  if (document.getElementById("objects").classList.contains("active")) {
    refreshObjects();
  }
}, 2000);
