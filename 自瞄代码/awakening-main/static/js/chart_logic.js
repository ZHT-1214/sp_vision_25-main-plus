const DATA_POLL_INTERVAL_MS = 100;
const CHART_RENDER_INTERVAL_MS = 33;
const DEFAULT_MAX_POINTS = 100;

const chartMap = {
  yaw: { label: "Yaw", color: "#2fc3a2" },
  pitch: { label: "Pitch", color: "#f4b44d" },
  target_yaw: { label: "Target Yaw", color: "#7aa2ff" },
  target_pitch: { label: "Target Pitch", color: "#ef6b73" },
  gimbal_yaw: { label: "Gimbal Yaw", color: "#b18cff" },
  gimbal_pitch: { label: "Gimbal Pitch", color: "#55c7f7" },
  control_v_yaw: { label: "Control V Yaw", color: "#7bd88f" },
  control_v_pitch: { label: "Control V Pitch", color: "#f7cf5d" },
  control_a_yaw: { label: "Control A Yaw", color: "#ff9f7a" },
  control_a_pitch: { label: "Control A Pitch", color: "#c9a7ff" },
  fly_time: { label: "Fly Time", color: "#8bd3dd" },
  target_v_yaw: { label: "Target V Yaw", color: "#f58ac2" },
  target_v_roll: { label: "Target V Roll", color: "#a7d46f" },
  rune_a_diff: { label: "Rune A Diff", color: "#ffcf8a" },
  rune_w_diff: { label: "Rune W Diff", color: "#a7d46f" },
  yaw_diff: { label: "Yaw Diff", color: "#a7d46f" },
  pitch_diff: { label: "Pitch Diff", color: "#ffcf8a" },
  enable_yaw_diff: { label: "Enable Yaw Diff", color: "#ffcf8a" },
  enable_pitch_diff: { label: "Enable Pitch Diff", color: "#a7d46f" }
};

let mainChart = null;
let dataTimer = null;
let dataFetchController = null;
let latestDataSignature = "";
let latestChartData = null;
let renderQueued = false;
let lastRenderAt = 0;
let selectedKeys = [];
let individualCharts = {};
let individualRanges = {};

function chartOptions() {
  return {
    animation: false,
    responsive: true,
    maintainAspectRatio: false,
    normalized: true,
    interaction: {
      mode: "nearest",
      axis: "x",
      intersect: false,
    },
    elements: {
      line: {
        borderWidth: 1.5,
        tension: 0,
        spanGaps: true,
      },
      point: {
        radius: 0,
        hoverRadius: 0,
      },
    },
    plugins: {
      decimation: {
        enabled: true,
        algorithm: "min-max",
      },
      legend: {
        labels: {
          color: "#d8e2ea",
          boxWidth: 10,
          boxHeight: 10,
          font: { size: 12 },
        },
      },
      tooltip: {
        enabled: true,
        intersect: false,
        callbacks: {
          title: () => "",
          label: (context) => `${context.dataset.label}: ${Number(context.parsed.y).toFixed(3)}`,
        },
      },
    },
    scales: {
      x: {
        display: false,
        grid: { display: false },
      },
      y: {
        ticks: {
          color: "#8fa0ad",
          maxTicksLimit: 6,
        },
        grid: { color: "rgba(143, 160, 173, 0.16)" },
      },
    },
  };
}

function setStatus(id, ok) {
  const el = document.getElementById(id);
  if (!el) return;
  el.classList.toggle("ok", ok);
  el.classList.toggle("error", !ok);
}

function getMaxPoints() {
  const input = document.getElementById("mainMaxPts");
  const parsed = Number.parseInt(input?.value, 10);
  return Number.isFinite(parsed) ? Math.min(Math.max(parsed, 10), 1000) : DEFAULT_MAX_POINTS;
}

function getSelectedKeys() {
  return Array.from(document.querySelectorAll("#chartSelectControls input[type='checkbox']:checked"))
    .map((input) => input.dataset.key)
    .filter(Boolean);
}

function setAutoScale(chart, values, range) {
  if (range?.enabled) {
    chart.options.scales.y.min = range.min;
    chart.options.scales.y.max = range.max;
    return;
  }

  const clean = values.filter((value) => Number.isFinite(value));
  if (clean.length === 0) {
    chart.options.scales.y.min = undefined;
    chart.options.scales.y.max = undefined;
    return;
  }

  let min = clean[0];
  let max = clean[0];
  for (const value of clean) {
    if (value < min) min = value;
    if (value > max) max = value;
  }

  const padding = (max - min) * 0.1 || 1;
  chart.options.scales.y.min = min - padding;
  chart.options.scales.y.max = max + padding;
}

function createDataset(key) {
  const meta = chartMap[key] || { label: key, color: "#d8e2ea" };
  return {
    label: meta.label,
    data: [],
    borderColor: meta.color,
    backgroundColor: meta.color,
    fill: false,
  };
}

function createChart(canvas, datasets) {
  return new Chart(canvas.getContext("2d"), {
    type: "line",
    data: {
      labels: [],
      datasets,
    },
    options: chartOptions(),
  });
}

function initCharts() {
  if (!window.Chart) {
    setStatus("data-status", false);
    console.warn("Chart.js is not available");
    return;
  }

  const mainCanvas = document.getElementById("mainChart");
  mainChart = createChart(mainCanvas, []);
  updateCharts();
}

function updateMainRange() {
  if (!mainChart) return;
  mainChart._maxPoints = getMaxPoints();
  latestDataSignature = "";
  fetchDataAndQueueRender();
}

function updateCharts() {
  if (!mainChart) return;

  selectedKeys = getSelectedKeys();
  const showMulti = document.getElementById("multiLineChart")?.checked ?? true;
  const container = document.getElementById("individualCharts");

  mainChart.data.datasets = showMulti ? selectedKeys.map(createDataset) : [];
  mainChart.update("none");

  for (const chart of Object.values(individualCharts)) {
    chart.destroy();
  }
  individualCharts = {};
  container.replaceChildren();

  for (const key of selectedKeys) {
    const meta = chartMap[key] || { label: key };
    const box = document.createElement("section");
    box.className = "chart-box";

    const title = document.createElement("h4");
    title.textContent = meta.label;
    box.appendChild(title);

    const controls = document.createElement("div");
    controls.className = "range-controls";
    controls.innerHTML = `
      <label><input type="checkbox" class="childEnable" /> 固定范围</label>
      <span>min</span><input type="number" class="childMin" value="0" step="0.1" />
      <span>max</span><input type="number" class="childMax" value="1" step="0.1" />
      <button type="button" class="applyRange">应用</button>
    `;
    box.appendChild(controls);

    const canvasWrap = document.createElement("div");
    canvasWrap.className = "chart-canvas-wrap";
    const canvas = document.createElement("canvas");
    canvasWrap.appendChild(canvas);
    box.appendChild(canvasWrap);
    container.appendChild(box);

    const chart = createChart(canvas, [createDataset(key)]);
    individualCharts[key] = chart;

    controls.querySelector(".applyRange").addEventListener("click", () => {
      const enabled = controls.querySelector(".childEnable").checked;
      const min = Number.parseFloat(controls.querySelector(".childMin").value);
      const max = Number.parseFloat(controls.querySelector(".childMax").value);
      individualRanges[key] = {
        enabled,
        min: Number.isFinite(min) ? min : undefined,
        max: Number.isFinite(max) ? max : undefined,
      };
      latestDataSignature = "";
      fetchDataAndQueueRender();
    });
  }

  latestDataSignature = "";
  fetchDataAndQueueRender();
}

function dataSignature(json, keys) {
  const time = Array.isArray(json.time) ? json.time : [];
  const lastTime = time.length ? time[time.length - 1] : "";
  const lengths = keys.map((key) => `${key}:${Array.isArray(json[key]) ? json[key].length : 0}`).join("|");
  return `${time.length}:${lastTime}:${lengths}`;
}

function updateChartData(json) {
  const labels = Array.isArray(json.time) ? json.time : [];
  if (labels.length === 0) return;

  if (document.getElementById("multiLineChart")?.checked) {
    mainChart.data.labels = labels;
    const allValues = [];
    mainChart.data.datasets.forEach((dataset, index) => {
      const key = selectedKeys[index];
      const values = Array.isArray(json[key]) ? json[key] : [];
      dataset.data = values;
      allValues.push(...values);
    });
    setAutoScale(mainChart, allValues);
    mainChart.update("none");
  }

  for (const [key, chart] of Object.entries(individualCharts)) {
    const values = Array.isArray(json[key]) ? json[key] : [];
    chart.data.labels = labels;
    chart.data.datasets[0].data = values;
    setAutoScale(chart, values, individualRanges[key]);
    chart.update("none");
  }
}

function queueChartRender() {
  if (renderQueued) return;
  renderQueued = true;
  requestAnimationFrame(renderChartFrame);
}

function renderChartFrame(timestamp) {
  if (!latestChartData) {
    renderQueued = false;
    return;
  }

  const elapsed = timestamp - lastRenderAt;
  if (elapsed < CHART_RENDER_INTERVAL_MS) {
    setTimeout(() => requestAnimationFrame(renderChartFrame), CHART_RENDER_INTERVAL_MS - elapsed);
    return;
  }

  lastRenderAt = timestamp;
  renderQueued = false;
  updateChartData(latestChartData);
}

async function fetchDataAndQueueRender() {
  if (!mainChart) return;
  if (dataFetchController) return;

  const controller = new AbortController();
  dataFetchController = controller;
  const maxPoints = getMaxPoints();
  try {
    const response = await fetch(`/data?max_points=${maxPoints}`, {
      cache: "no-store",
      signal: controller.signal,
    });
    if (!response.ok) throw new Error(response.statusText);

    const json = await response.json();
    const time = Array.isArray(json.time) ? json.time : [];
    if (time.length === 0) return;

    const signature = dataSignature(json, selectedKeys);
    if (signature === latestDataSignature) {
      setStatus("data-status", true);
      return;
    }
    latestDataSignature = signature;
    latestChartData = json;
    queueChartRender();

    setStatus("data-status", true);
  } catch (error) {
    if (error.name !== "AbortError") {
      setStatus("data-status", false);
      console.warn("data fetch failed:", error.message);
    }
  } finally {
    if (dataFetchController === controller) {
      dataFetchController = null;
    }
  }
}

function startDataLoop() {
  if (dataTimer) return;
  const tick = async () => {
    await fetchDataAndQueueRender();
    dataTimer = setTimeout(tick, DATA_POLL_INTERVAL_MS);
  };
  dataTimer = setTimeout(tick, 0);
}
