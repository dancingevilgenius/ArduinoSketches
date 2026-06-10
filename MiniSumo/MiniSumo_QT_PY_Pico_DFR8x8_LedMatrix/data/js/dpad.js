// Formatting helpers
function pad2(n) { return n.toString().padStart(2, ' '); }
function padLatency(ms) {
    return ms.toFixed(1).padStart(5, ' ');
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

// A2-compatible D-Pad HTTP
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
        if (data.message) showSnackbar(data.message, data.severity || "success");
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
        if (data.message) showSnackbar(data.message, data.severity || "success");
    });
}

// Optional: health check command (safe no-op if backend ignores it)
function runHealthChecks() {
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send("HEALTHCHECK");
    } else {
        showSnackbar("WebSocket not connected", "warning");
    }
}

// WebSocket (A2-style: PING/PONG + SNACK only)
let socket = null;
let reconnectTimer = null;
let lastPingTime = 0;

function connectWebSocket() {
    const host = window.location.hostname;
    socket = new WebSocket(`ws://${host}/ws`);

    socket.onopen = () => {
        socket.send("UA:" + navigator.userAgent);
        lastPingTime = performance.now();
        socket.send("PING");
        if (reconnectTimer) clearTimeout(reconnectTimer);
    };

    socket.onclose = () => {
        reconnectTimer = setTimeout(connectWebSocket, 500);
    };

    socket.onerror = () => socket.close();

    socket.onmessage = (event) => {
        if (typeof event.data === "string") {

            if (event.data === "PONG") {
                // Latency available if you want to display it later
                const latency = performance.now() - lastPingTime;
                return;
            }

            if (event.data.startsWith("SNACK:")) {
                const parts = event.data.split(":");
                const type = parts[1];
                const msg  = parts.slice(2).join(":");
                showSnackbar(msg, type);
                return;
            }
        }
    };
}

// Periodic PING (A2-style)
setInterval(() => {
    if (socket && socket.readyState === WebSocket.OPEN) {
        lastPingTime = performance.now();
        socket.send("PING");
    }
}, 2000);

// Mode dropdown -> navigate to grid page
document.addEventListener("DOMContentLoaded", () => {
    connectWebSocket();

    const dropdown = document.getElementById("modeDropdown");
    if (dropdown) {
        dropdown.onchange = () => {
            if (dropdown.value === "grid") {
                window.location = "/index_grid.html";
            }
        };
    }
});
