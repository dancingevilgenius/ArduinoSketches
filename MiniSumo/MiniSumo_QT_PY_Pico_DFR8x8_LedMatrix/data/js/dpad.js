// ---------------------- WebSocket ----------------------
let ws;

function initWS() {
  ws = new WebSocket(`ws://${location.host}/ws`);

  ws.onopen = () => console.log("WS connected");
  ws.onclose = () => console.log("WS disconnected");

  ws.onmessage = (event) => {
    const msg = event.data;

    if (msg.startsWith("SNACK:")) {
      handleSnackMessage(msg);
      return;
    }
  };
}

window.onload = () => {
  initWS();
  buildGrid();
};

// ---------------------- FPS ----------------------
const fpsSlider = document.getElementById("fpsSlider");
const fpsValue  = document.getElementById("fpsValue");

fpsSlider.oninput = () => {
  fpsValue.textContent = fpsSlider.value + " FPS";
  ws.send("FPS:" + fpsSlider.value);
};

// ---------------------- Animation Toggle ----------------------
const animToggle = document.getElementById("animToggle");
let animRunning = true;

animToggle.onclick = () => {
  animRunning = !animRunning;
  animToggle.textContent = animRunning ? "Running" : "Stopped";
  animToggle.classList.toggle("off", !animRunning);
  ws.send("RUN:" + (animRunning ? 1 : 0));
};

// ---------------------- Mode Selector ----------------------
document.getElementById("modeSelect").onchange = (e) => {
  ws.send("MODE:" + e.target.value);
};

// ---------------------- D-Pad ----------------------
function sendDpad(dir) {
  fetch("/controller", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ direction: dir })
  });
}

document.getElementById("btnUp").onclick    = () => sendDpad("up");
document.getElementById("btnDown").onclick  = () => sendDpad("down");
document.getElementById("btnLeft").onclick  = () => sendDpad("left");
document.getElementById("btnRight").onclick = () => sendDpad("right");

// ---------------------- 8×8 Grid ----------------------
function buildGrid() {
  const grid = document.getElementById("grid");
  for (let i = 0; i < 64; i++) {
    const cell = document.createElement("div");
    cell.className = "grid-cell";
    grid.appendChild(cell);
  }
}

// ---------------------- Snackbar Engine ----------------------
function showSnackbar(type, message) {
  const container = document.getElementById("snackbar-container");

  const bar = document.createElement("div");
  bar.className = "snackbar " + type;

  let icon = "ℹ️";
  if (type === "success") icon = "✔️";
  else if (type === "error") icon = "✖️";
  else if (type === "warn") icon = "⚠️";

  bar.innerHTML = `
    <span class="snackbar-icon">${icon}</span>
    <span>${message}</span>
  `;

  container.appendChild(bar);

  void bar.offsetWidth;
  bar.classList.add("show");

  const autoTimer = setTimeout(() => dismissSnackbar(bar), 3000);

  bar.addEventListener("click", () => {
    clearTimeout(autoTimer);
    dismissSnackbar(bar);
  });
}

function dismissSnackbar(bar) {
  bar.classList.remove("show");
  setTimeout(() => bar.remove(), 250);
}

function handleSnackMessage(msg) {
  const parts = msg.split(":");
  if (parts.length < 3) return;

  const type = parts[1];
  const text = parts.slice(2).join(":");

  showSnackbar(type, text);
}
