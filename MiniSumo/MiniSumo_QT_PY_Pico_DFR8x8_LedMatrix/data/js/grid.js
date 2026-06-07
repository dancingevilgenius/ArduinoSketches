/* ============================================================
   GRID.JS — WebSocket Grid Controller (v1.2)
   With PING IDs for accurate latency measurement
   ============================================================ */

let socket = null;
let reconnectTimer = null;

let frameCount = 0;
let lastFPSUpdate = performance.now();

let cells = [];

// Latency tracking
let pingId = 0;
let pingTimes = {};

/* ------------------------------------------------------------
   Snackbar
   ------------------------------------------------------------ */
function showSnackbar(message, type = "success") {
    const container = document.getElementById("snackbar-container");
    const bar = document.createElement("div");
    bar.className = "snackbar";

    if (type === "success") bar.classList.add("snackbar-success");
    if (type === "warning") bar.classList.add("snackbar-warning");
    if (type === "error")   bar.classList.add("snackbar-error");

    bar.textContent = message;

    bar.addEventListener("click", () => {
        bar.classList.remove("show");
        setTimeout(() => bar.remove(), 300);
    });

    container.appendChild(bar);
    void bar.offsetWidth;
    bar.classList.add("show");

    setTimeout(() => {
        bar.classList.remove("show");
        setTimeout(() => bar.remove(), 300);
    }, 6000);
}

/* ------------------------------------------------------------
   Formatting helpers
   ------------------------------------------------------------ */
function pad2(n) { return n.toString().padStart(2, ' '); }

function padLatency(n) {
    let parts = n.toFixed(1).split('.');
    return parts[0].padStart(4, ' ') + '.' + parts[1];
}

function pad3(n) { return n.toString().padStart(3, ' '); }

/* ------------------------------------------------------------
   DOM Ready
   ------------------------------------------------------------ */
document.addEventListener("DOMContentLoaded", () => {

    // Dropdown navigation
    const dropdown = document.getElementById("modeDropdown");
    dropdown.onchange = () => {
        if (dropdown.value === "dpad") {
            window.location = "/index_dpad.html";
        }
    };

    // Cache grid cells
    cells = Array.from(document.querySelectorAll(".grid-cell"));

    setupControls();
    connectWebSocket();
});

/* ------------------------------------------------------------
   WebSocket Connection
   ------------------------------------------------------------ */
function connectWebSocket() {
    const host = window.location.hostname;
    socket = new WebSocket(`ws://${host}/ws`);
    socket.binaryType = "arraybuffer";

    socket.onopen = () => {
        socket.send("UA:" + navigator.userAgent);

        // Send first PING immediately
        pingId++;
        pingTimes[pingId] = performance.now();
        socket.send("PING:" + pingId);

        if (reconnectTimer) clearTimeout(reconnectTimer);
    };

    socket.onclose = () => {
        reconnectTimer = setTimeout(connectWebSocket, 500);
    };

    socket.onerror = () => socket.close();

    socket.onmessage = (event) => {

        /* ------------------------------
           TEXT MESSAGES
           ------------------------------ */
        if (typeof event.data === "string") {

            // PONG:<id>
            if (event.data.startsWith("PONG:")) {
                const id = parseInt(event.data.split(":")[1]);
                if (pingTimes[id]) {
                    const latency = performance.now() - pingTimes[id];
                    document.getElementById("lat").textContent = padLatency(latency);
                    delete pingTimes[id];
                }
                return;
            }

            if (event.data.startsWith("SNACK:")) {
                const parts = event.data.split(":");
                const type = parts[1];
                const msg  = parts.slice(2).join(":");
                showSnackbar(msg, type);
                return;
            }

            return;
        }

        /* ------------------------------
           BINARY MESSAGES
           ------------------------------ */
        const buf = event.data;
        document.getElementById("pkt").textContent = pad3(buf.byteLength);

        frameCount++;
        updateFPS();

        // FULL RGB GRID (256 bytes)
        if (buf.byteLength === 256) {
            const view = new DataView(buf);
            for (let i = 0; i < 64; i++) {
                const rgb = view.getUint32(i * 4, true);
                updateCellColor(i, rgb);
            }
        }

        // BITMASK GRID (8 bytes)
        else if (buf.byteLength === 8) {
            const view = new DataView(buf);
            const bits = view.getBigUint64(0, true);

            for (let i = 0; i < 64; i++) {
                const on = ((bits >> BigInt(i)) & 1n) === 1n;
                updateCellColor(i, on ? 0x00FF00 : 0);
            }
        }
    };
}

/* ------------------------------------------------------------
   Send PING every 2 seconds
   ------------------------------------------------------------ */
setInterval(() => {
    if (socket && socket.readyState === WebSocket.OPEN) {
        pingId++;
        pingTimes[pingId] = performance.now();
        socket.send("PING:" + pingId);
    }
}, 2000);

/* ------------------------------------------------------------
   FPS Counter
   ------------------------------------------------------------ */
function updateFPS() {
    const now = performance.now();
    const elapsed = now - lastFPSUpdate;

    if (elapsed >= 1000) {
        const fps = Math.round((frameCount / elapsed) * 1000);
        document.getElementById("fps").textContent = pad2(fps);
        frameCount = 0;
        lastFPSUpdate = now;
    }
}

/* ------------------------------------------------------------
   Update a single grid cell
   ------------------------------------------------------------ */
function updateCellColor(index, rgb) {
    if (rgb === 0) {
        cells[index].style.background = "#222";
        return;
    }

    let r = (rgb >> 16) & 0xFF;
    let g = (rgb >> 8) & 0xFF;
    let b = rgb & 0xFF;

    cells[index].style.background = `rgb(${r},${g},${b})`;
}

/* ------------------------------------------------------------
   Send command to backend
   ------------------------------------------------------------ */
function sendCommand(cmd) {
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(cmd);
    }
}

/* ------------------------------------------------------------
   Controls (FPS, Mode, Animation)
   ------------------------------------------------------------ */
function setupControls() {
    const btnRun = document.getElementById("btnRun");
    const fpsSlider = document.getElementById("fpsSlider");
    const modeSelect = document.getElementById("modeSelect");

    let running = true;

    btnRun.onclick = () => {
        running = !running;
        btnRun.textContent = running ? "Pause" : "Resume";
        sendCommand("RUN:" + (running ? "1" : "0"));
    };

    fpsSlider.oninput = () => sendCommand("FPS:" + fpsSlider.value);

    modeSelect.onchange = () => sendCommand("MODE:" + modeSelect.value);
}
