const jsonSignatures = new Map();

function jsonToHtml(data, container) {
  const fragment = document.createDocumentFragment();

  function buildTree(value, parent) {
    if (typeof value !== "object" || value === null) {
      parent.textContent = String(value);
      return;
    }

    const ul = document.createElement("ul");
    ul.className = "json-tree";
    const entries = Array.isArray(value) ? value.map((v, i) => [i, v]) : Object.entries(value);

    for (const [key, child] of entries) {
      const li = document.createElement("li");
      if (typeof child === "object" && child !== null) {
        const details = document.createElement("details");
        details.open = true;

        const summary = document.createElement("summary");
        summary.textContent = key;
        details.appendChild(summary);

        buildTree(child, details);
        li.appendChild(details);
      } else {
        const keySpan = document.createElement("span");
        keySpan.className = "json-leaf-key";
        keySpan.textContent = `${key}: `;

        const valueSpan = document.createElement("span");
        valueSpan.className = "json-leaf-value";
        valueSpan.textContent = String(child);

        li.append(keySpan, valueSpan);
      }
      ul.appendChild(li);
    }

    parent.appendChild(ul);
  }

  buildTree(data, fragment);
  container.replaceChildren(fragment);
}

async function fetchAndDisplayJsonWithTree(id, url) {
  const container = document.getElementById(id);
  if (!container) return;

  try {
    const response = await fetch(url, { cache: "no-store" });
    if (!response.ok) throw new Error(response.statusText);

    const data = await response.json();
    const signature = JSON.stringify(data);
    if (jsonSignatures.get(id) !== signature) {
      jsonToHtml(data, container);
      jsonSignatures.set(id, signature);
    }

    setStatus("log-status", true);
  } catch (error) {
    setStatus("log-status", false);
    console.warn(`json fetch failed (${url}): ${error.message}`);
  }
}
