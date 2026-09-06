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

// gesture_policy.py is shared with the autonomous stack, so its wording is left
// alone and translated here instead. Every reason it produces comes from one of
// these patterns; anything unrecognised is shown as it arrived rather than
// mangled, which is also how a new pattern would make itself visible.
const TIERS = { ok: "安全", caution: "注意", blocked: "已封鎖", unknown: "未量測" };

function translateReason(text) {
  if (!text) return "";
  let m;
  if ((m = text.match(/^self-collision at full amplitude \(min gap ([\d.]+) m\)$/))) {
    return `全幅度下會自我碰撞（最小間隙 ${m[1]} 公尺）`;
  }
  if ((m = text.match(/^self-collision, min gap ([\d.]+) m(?: in (\d+) keyframes?)?$/))) {
    return `自我碰撞，最小間隙 ${m[1]} 公尺` + (m[2] ? `，共 ${m[2]} 個關鍵幀` : "");
  }
  if ((m = text.match(/^clearance ([\d.]+) m, inside the shoulders' tracking error$/))) {
    return `間隙 ${m[1]} 公尺，落在肩部追蹤誤差範圍內`;
  }
  if ((m = text.match(/^clearance ([\d.]+) m$/))) {
    return `間隙 ${m[1]} 公尺`;
  }
  if (text === "not in the measured allow-list") {
    return "尚未量測，不在允許清單內";
  }
  return text;
}

function setLog(msg, kind) {
  logEl.textContent = msg;
  logEl.className = "log" + (kind ? " " + kind : "");
}

async function refreshStatus() {
  try {
    const s = await (await fetch("/api/status")).json();
    const via = s.daemon ? "常駐" : "另開行程";
    statusEl.textContent = `${s.dds} \u00b7 ${s.mapper} \u00b7 ${via} \u00b7 前置 ${s.lead_s} 秒`;
    statusEl.className = "status " + (s.daemon ? "ok" : "warn");
    statusEl.title = s.daemon
      ? `常駐播放器：${s.socket}`
      : `${s.socket} 上沒有常駐播放器，每個手勢都會另開 run_player.sh（手臂約 2.3 秒後才動）`;
  } catch (e) {
    statusEl.textContent = "無法連線到伺服器";
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
      const why = translateReason(g.reason);
      btn.title = g.playable ? why : "已封鎖：" + why;
      btn.dataset.file = g.file;
      btn.textContent = g.name;
      const tag = document.createElement("span");
      tag.className = "tag";
      const tier = TIERS[g.tier] || g.tier;
      tag.textContent = g.sample ? "範例 \u00b7 " + tier : tier;
      btn.append(tag);
      btn.addEventListener("click", () => play(g, btn));
      return btn;
    })
  );
  const playable = gestures.filter((g) => g.playable).length;
  counts.textContent = `顯示 ${shown.length} 個 \u00b7 共 ${gestures.length} 個，其中 ${playable} 個可播放`;
}

async function play(g, btn) {
  if (Date.now() < busyUntil) return;
  document.querySelectorAll(".card.playing").forEach((c) => c.classList.remove("playing"));
  btn.classList.add("playing");
  setLog(`播放 ${g.name} 中\u2026`);
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
      setLog(`${g.name}：${translateReason(String(r.error).replace(/^blocked: /, "已封鎖："))}`, "err");
      btn.classList.remove("playing");
      return;
    }
    // ends_in_s is only reported by the daemon; a spawn does not know yet.
    const ends = r.ends_in_s;
    const via = r.via === "daemon" ? "常駐" : "另開行程";
    if (ends) {
      busyUntil = Date.now() + ends * 1000;
      setLog(`${g.name}（${via}）：手臂 ${r.lead_s} 秒後開始動，${ends} 秒完成`, "ok");
      setTimeout(() => btn.classList.remove("playing"), ends * 1000);
    } else {
      setLog(`${g.name}（${via}，pid ${r.pid}）`, "ok");
      setTimeout(() => btn.classList.remove("playing"), 4000);
    }
  } catch (e) {
    setLog(`${g.name}：${e}`, "err");
    btn.classList.remove("playing");
  }
}

document.getElementById("stop").addEventListener("click", async () => {
  busyUntil = 0;
  document.querySelectorAll(".card.playing").forEach((c) => c.classList.remove("playing"));
  await fetch("/api/stop", { method: "POST" });
  setLog("已停止");
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
