function pad2(n) { return n.toString().padStart(2, ' '); }
function padLatency(n) {
    let parts = n.toFixed(1).split('.');
    return parts[0].padStart(4, ' ') + '.' + parts[1];
}
function pad3(n) { return n.toString().padStart(3, ' '); }

// Snackbar
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

// D-Pad HTTP
function sendDirection(dir) {
    fetch('/controller', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ direction: dir })
    })
    .then(r => r.json())
    .then(data => {
        document.getElementById('hStatus').innerText = data.horiz || '';
        document.getElementById('vStatus').innerText = data.vert || '';
        if (data.message) showSnackbar(data.message, "success");
    });
}

function sendAction(action) {
    fetch('/controller', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ action: action })
    })
    .then(r => r.json())
    .then(data => {
        document.getElementById('hStatus').innerText = data.horiz || '';
        document.getElementById('vStatus').innerText = data.vert || '';
        if (data.message) showSnackbar(data.message, "success");
    });
}

let socket = null;
let reconnectTimer = null;
let lastPing = 0;
let frameCount = 0;
let lastFPSUpdate = performance.now();
let cells = [];

document.addEventListener('DOMContentLoaded', () => {
    const dropdown = document.getElementById('dropdown');
    const dpadBlock = document.getElementById('dpadBlock');
    const grid8 = document.getElementById('grid8x8');
    const statusRow = document.querySelector('.status-row');
    const bottomButtons = document.querySelector('.bottom-buttons');
    const healthBtnContainer = document.getElementById('healthBtnContainer');

    dropdown.addEventListener('change', function() {
        if (this.value === '8x8') {
            dpadBlock.style.display = 'none';
            grid8.style.display = 'block';
            statusRow.style.display = 'none';
            bottomButtons.style.display = 'none';
            healthBtnContainer.style.display = 'none';
        } else {
            dpadBlock.style.display = 'block';
            grid8.style.display = 'none';
            statusRow.style.display = 'flex';
            bottomButtons.style.display = 'flex';
            healthBtnContainer.style.display = 'block';
        }
    });

    cells = Array.from(document.querySelectorAll('.grid-cell'));

    setupControls();
    connectWebSocket();
});

function connectWebSocket() {
    const host = window.location.hostname;
    socket = new WebSocket(`ws://${host}/ws`);
    socket.binaryType = "arraybuffer";

    socket.onopen = () => {
        socket.send("UA:" + navigator.userAgent);
        if (reconnectTimer) clearTimeout(reconnectTimer);
    };

    socket.onclose = () => {
        reconnectTimer = setTimeout(connectWebSocket, 500);
    };

    socket.onerror = () => socket.close();

    socket.onmessage = (event) => {
        if (typeof event.data === "string") {
            if (event.data === "PONG") {
                const latency = performance.now() - lastPing;
                document.getElementById("lat").textContent = padLatency(latency);
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

        const buf = event.data;
        document.getElementById("pkt").textContent = pad3(buf.byteLength);

        frameCount++;
        updateFPS();

        if (buf.byteLength === 256) {
            const view = new DataView(buf);
            for (let i = 0; i < 64; i++) {
                const rgb = view.getUint32(i * 4, true);
                updateCellColor(i, rgb);
            }
        }

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

setInterval(() => {
    if (socket && socket.readyState === WebSocket.OPEN) {
        lastPing = performance.now();
        socket.send("PING");
    }
}, 2000);

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

function sendCommand(cmd) {
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(cmd);
    }
}

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

function runHealthChecks() {
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send("HEALTHCHECK");
    }
}
