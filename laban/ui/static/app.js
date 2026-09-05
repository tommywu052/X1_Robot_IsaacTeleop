const grid = document.getElementById("grid");
const search = document.getElementById("search");
const speed = document.getElementById("speed");
const speedOut = document.getElementById("speedOut");
const head = document.getElementById("head");
const statusEl = document.getElementById("status");
const logEl = document.getElementById("log");
const counts = document.getElementById("counts");

let gestures = [];
let busyUntil = 0;

function setLog(msg, kind) {
  logEl.textContent = msg;
  logEl.className = "log" + (kind ? " " + kind : "");
}

async function refreshStatus() {
  try {
    const s = await (await fetch("/api/status")).json();
    const via = s.daemon ? "daemon" : "spawn";
    statusEl.textContent = `${s.dds} ， ${s.mapper} ， ${via} ， lead ${s.lead_s}s`;
    statusEl.className = "status " + (s.daemon ? "ok" : "warn");
    statusEl.title = s.daemon
      ? `Resident player on ${s.socket}`
      : `No daemon on ${s.socket}: each gesture spawns run_player.sh (~2.3s before the arms move)`;
  } catch (e) {
    statusEl.textContent = "server unreachable";
    statusEl.className = "status err";
  }
}

function render() {
  const q = search.value.trim().toLowerCase();
  const shown = gestures.filter((g) => !q || g.name.toLowerCase().includes(q));
  grid.replaceChildren(
    ...shown.map((g) => {
      const btn = document.createElement("button");
      btn.className = "card" + (g.tier === "caution" ? " caution" : "");
      btn.disabled = !g.playable;
      btn.title = g.playable ? g.reason : "Blocked: " + g.reason;
      btn.dataset.file = g.file;
      btn.textContent = g.name;
      const tag = document.createElement("span");
      tag.className = "tag";
      tag.textContent = g.sample ? "sample ， " + g.tier : g.tier;
      btn.append(tag);
      btn.addEventListener("click", () => play(g, btn));
      return btn;
    })
  );
  counts.textContent = `${shown.length} shown ， ${gestures.filter((g) => g.playable).length} playable of ${gestures.length}`;
}

async function play(g, btn) {
  if (Date.now() < busyUntil) return;
  document.querySelectorAll(".card.playing").forEach((c) => c.classList.remove("playing"));
  btn.classList.add("playing");
  setLog(`playing ${g.name}\u2026`);
  try {
    const res = await fetch("/api/play", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        file: g.file,
        speed: parseFloat(speed.value),
        head: head.checked,
      }),
    });
    const r = await res.json();
    if (!r.ok) {
      setLog(`${g.name}: ${r.error}`, "err");
      btn.classList.remove("playing");
      return;
    }
    // ends_in_s is only reported by the daemon; a spawn does not know yet.
    const ends = r.ends_in_s;
    if (ends) {
      busyUntil = Date.now() + ends * 1000;
      setLog(`${g.name} via ${r.via}: arms move in ${r.lead_s}s, done in ${ends}s`, "ok");
      setTimeout(() => btn.classList.remove("playing"), ends * 1000);
    } else {
      setLog(`${g.name} via ${r.via || "spawn"} (pid ${r.pid})`, "ok");
      setTimeout(() => btn.classList.remove("playing"), 4000);
    }
  } catch (e) {
    setLog(`${g.name}: ${e}`, "err");
    btn.classList.remove("playing");
  }
}

document.getElementById("stop").addEventListener("click", async () => {
  busyUntil = 0;
  document.querySelectorAll(".card.playing").forEach((c) => c.classList.remove("playing"));
  await fetch("/api/stop", { method: "POST" });
  setLog("stopped");
});

search.addEventListener("input", render);
speed.addEventListener("input", () => {
  speedOut.textContent = parseFloat(speed.value).toFixed(2) + "\u00d7";
});

(async () => {
  const data = await (await fetch("/api/gestures")).json();
  gestures = data.gestures;
  render();
  refreshStatus();
  setInterval(refreshStatus, 5000);
})();
