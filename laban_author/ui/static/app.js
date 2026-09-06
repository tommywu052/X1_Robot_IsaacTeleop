"use strict";

// Kept to plain ASCII on purpose: this file has been mangled by an encoding
// round trip before, and the symbols are easier to read as escapes than to
// debug as replacement characters.
var MIDDOT = "\u00b7";
var TIMES = "\u00d7";

var state = {
  jobs: {},
  activeId: null,
  // What each recording is called, once somebody has typed a name for it.
  // Keyed by job id so switching recordings cannot carry a name across.
  names: {},
  analysis: null,
  // null means "wherever the energy curve peaks"; an array means somebody has
  // been clicking the curve and their choice wins.
  keyframes: null,
  built: null,
  saved: null,
  poll: null
};

var el = {};
// Every id the page scripts touch. A missing entry used to fail as an
// undefined property deep inside a promise, which is worth catching here.
["status", "video", "videoName", "name", "maxFrames", "framesOut", "upload",
 "cost", "jobs", "progressStep", "bar", "stage", "joblog", "framingStep",
 "verdict", "overlay", "editStep", "energy", "window", "windowOut",
 "sigma", "sigmaOut",
 "deriveHead", "acromion", "baseFirst", "reset", "editLog", "score",
 "robotStep", "save", "play", "speed", "speedOut", "playHead", "stop",
 "robotLog", "libInfo"].forEach(function (id) {
  el[id] = document.getElementById(id);
  if (!el[id]) { throw new Error("the page has no #" + id); }
});

function api(path, options) {
  return fetch(path, options).then(function (response) {
    return response.json().then(function (body) {
      if (!response.ok && body && body.error) { throw new Error(body.error); }
      if (!response.ok) { throw new Error("HTTP " + response.status); }
      return body;
    });
  });
}

function say(node, message, kind) {
  node.textContent = message;
  node.className = "note" + (kind ? " " + kind : " dim");
}

// The stage names are the pipeline's own, sent up as identifiers rather than
// prose. Translated at the edge so the backend keeps one vocabulary for its
// logs, its CLI and its state file.
var STAGES = {
  "new": "準備中", probe: "讀取影片", trim: "裁切", bbox: "框選人體",
  estimate: "估計姿態", export: "匯出關鍵點", overlay: "產生疊圖",
  analyse: "分析", ready: "完成", failed: "失敗"
};

function stageName(stage) { return STAGES[stage] || stage; }

// ---------------------------------------------------------------- status

function refreshStatus() {
  api("/api/status").then(function (s) {
    var bits = [s.dds, "mapper " + s.mapper,
                s.daemon ? "常駐播放器運作中" : "常駐播放器未啟動"];
    el.status.textContent = bits.join(" " + MIDDOT + " ");
    el.status.className = "status " + (s.estimator_ready
      ? (s.daemon ? "ok" : "warn") : "err");
    if (!s.estimator_ready) {
      el.status.textContent = "估計器無法使用";
    }
    el.libInfo.textContent = "手勢庫 " + s.library + " 共 "
      + s.library_count + " 個手勢";
    if (s.per_frame_s) {
      timing = {
        startup_s: s.startup_s,
        per_frame_s: s.per_frame_s,
        batch_size: s.batch_size
      };
      quoteCost();
    }
    if (s.player_error) {
      say(el.robotLog, "無法播放：" + s.player_error, "err");
    }
  }).catch(function (error) {
    el.status.textContent = error.message;
    el.status.className = "status err";
  });
}

// ---------------------------------------------------------------- upload

el.video.addEventListener("change", function () {
  var file = el.video.files[0];
  el.videoName.textContent = file ? file.name : "選擇影片…";
  el.upload.disabled = !file;
  if (file && !el.name.value) {
    el.name.value = file.name.replace(/\.[^.]+$/, "");
  }
});

// Both measured, and both replaced by whatever /api/status reports, so the
// number on screen is the one the machine actually produces.
var timing = { startup_s: 18, per_frame_s: 0.042, batch_size: 8 };

function quoteCost() {
  el.framesOut.textContent = el.maxFrames.value;
  var seconds = Math.round(timing.startup_s
    + timing.per_frame_s * Number(el.maxFrames.value));
  el.cost.textContent = el.maxFrames.value + " 幀大約需要 "
    + (seconds >= 90 ? Math.round(seconds / 6) / 10 + " 分鐘"
                     : seconds + " 秒")
    + "：其中約 " + Math.round(timing.startup_s) + " 秒在載入模型，"
    + "之後每幀 " + Math.round(timing.per_frame_s * 1000)
    + " 毫秒（批次大小 " + timing.batch_size + "）。";
}

el.maxFrames.addEventListener("input", quoteCost);

el.upload.addEventListener("click", function () {
  var file = el.video.files[0];
  if (!file) { return; }
  var form = new FormData();
  form.append("video", file);
  form.append("name", el.name.value);
  form.append("max_frames", el.maxFrames.value);

  el.upload.disabled = true;
  el.progressStep.hidden = false;
  el.bar.style.width = "0%";
  say(el.stage, "上傳中 " + Math.round(file.size / 1e6) + " MB");

  api("/api/jobs", { method: "POST", body: form }).then(function (reply) {
    select(reply.job.id, reply.job);
    startPolling();
  }).catch(function (error) {
    say(el.stage, error.message, "err");
  }).then(function () {
    el.upload.disabled = false;
  });
});

// ------------------------------------------------------------------ jobs

function select(id, job) {
  state.activeId = id;
  if (job) { state.jobs[id] = job; }
  state.analysis = null;
  state.keyframes = null;
  state.built = null;
  state.saved = null;
  el.play.disabled = true;
  renderJobs();
  applyJob(state.jobs[id]);
}

function renderJobs() {
  var ids = Object.keys(state.jobs).sort(function (a, b) {
    return state.jobs[b].created - state.jobs[a].created;
  });
  el.jobs.innerHTML = "";
  ids.forEach(function (id) {
    var job = state.jobs[id];
    var button = document.createElement("button");
    button.className = "job"
      + (id === state.activeId ? " active" : "")
      + (job.stage === "failed" ? " failed" : "");
    var detail = job.stage === "ready"
      ? job.frames + " 幀 " + MIDDOT + " " + job.peaks.length + " 個波峰"
      : stageName(job.stage) + " " + Math.round(job.progress * 100) + "%";
    button.innerHTML = "";
    button.appendChild(document.createTextNode(job.name));
    var small = document.createElement("small");
    small.textContent = detail;
    button.appendChild(small);
    button.addEventListener("click", function () { select(id); });
    el.jobs.appendChild(button);
  });
}

function startPolling() {
  if (state.poll) { return; }
  state.poll = setInterval(function () {
    var running = Object.keys(state.jobs).filter(function (id) {
      var stage = state.jobs[id].stage;
      return stage !== "ready" && stage !== "failed";
    });
    if (!running.length) {
      clearInterval(state.poll);
      state.poll = null;
      return;
    }
    running.forEach(function (id) {
      api("/api/jobs/" + id).then(function (job) {
        state.jobs[id] = job;
        renderJobs();
        if (id === state.activeId) { applyJob(job); }
      }).catch(function () {});
    });
  }, 1200);
}

function applyJob(job) {
  if (!job) { return; }
  el.progressStep.hidden = false;
  el.bar.style.width = Math.round(job.progress * 100) + "%";
  el.joblog.textContent = (job.log || []).join("\n");
  el.joblog.scrollTop = el.joblog.scrollHeight;

  if (job.stage === "failed") {
    say(el.stage, job.error || "失敗", "err");
    el.framingStep.hidden = true;
    el.editStep.hidden = true;
    el.robotStep.hidden = true;
    return;
  }
  if (job.stage !== "ready") {
    say(el.stage, stageName(job.stage) + "，" + Math.round(job.progress * 100) + "%"
      + (job.elapsed_s ? "，已經過 " + Math.round(job.elapsed_s) + " 秒" : ""));
    return;
  }

  say(el.stage, "已估計 " + job.frames + " 幀"
    + (job.elapsed_s ? "，耗時 " + Math.round(job.elapsed_s) + " 秒" : ""), "ok");
  el.framingStep.hidden = false;
  el.overlay.src = "/api/jobs/" + job.id + "/overlay.png?t=" + job.created;
  // Per recording, not once. Filling this only when it was empty meant the name
  // stayed behind when the selection moved on, so a 10 s score was saved under
  // the name of the 20 s recording next to it and looked like the head
  // derivation had failed. Each recording now carries its own name, edits
  // included, and switching always shows the one that belongs to what is
  // selected.
  el.name.value = state.names[job.id] || job.name;
  renderVerdict(job.framing, job.trimmed);

  if (!state.analysis) {
    api("/api/jobs/" + job.id + "/analysis").then(function (analysis) {
      state.analysis = analysis;
      el.editStep.hidden = false;
      el.robotStep.hidden = false;
      rebuild();
    }).catch(function (error) {
      say(el.editLog, error.message, "err");
    });
  }
}

// The verdict is an identifier the page and the API both key off, so it stays
// English on the wire and is turned into prose here. The note is rebuilt from
// the numbers rather than translated, which keeps the percentages live.
var VERDICTS = {
  "head-cropped": "頭部被裁切", "partly-cropped": "上半身部分出框",
  "too-far": "距離太遠", "too-close": "距離太近"
};

function framingNote(framing) {
  var pct = Math.round(framing.body_height_fraction * 100);
  switch (framing.verdict) {
    case "ok":
      return "從頭到臀部都在畫面內，人體高度佔畫面 " + pct + "%。";
    case "head-cropped":
      return "頭部在多數畫面中都在框外，因此量測手臂方向所依據的肩線是外推出來的，"
        + "而不是真的看到的。";
    case "partly-cropped":
      return "約三分之一的上半身落在畫面外，被切到的那一側肢體，符號都只是猜測。";
    case "too-far":
      return "人體只佔畫面高度的 " + pct + "%，像素太少，手肘和手腕的位置放不準。";
    case "too-close":
      return "人體比畫面還大，正在被裁切。";
    default:
      return framing.note;
  }
}

function renderVerdict(framing, trimmed) {
  el.verdict.innerHTML = "";
  if (!framing) { el.verdict.className = "verdict"; return; }

  // Said before the framing, because a dropped tail is the more complete kind
  // of missing: bad framing degrades every frame, but a trim removes some
  // entirely, and nothing further down the page can hint that they were there.
  if (trimmed) {
    var warn = document.createElement("b");
    warn.className = "cut";
    warn.textContent = "只用了前 " + trimmed.used + " 幀（全片共 "
      + trimmed.available + " 幀），最後 " + trimmed.dropped_s + " 秒被捨棄。";
    el.verdict.appendChild(warn);
    var how = document.createElement("span");
    how.textContent = (Math.round(trimmed.used / 3) / 10)
      + " 秒之後做的動作都不在這份樂譜裡。把上方的「幀數上限」調高後重新上傳，"
      + "才會完整保留。";
    el.verdict.appendChild(how);
  }

  var ok = framing.verdict === "ok";
  el.verdict.className = "verdict " + (ok && !trimmed ? "ok" : "bad");
  var headline = document.createElement("b");
  headline.textContent = ok
    ? "取景沒問題。"
    : "取景有問題：" + (VERDICTS[framing.verdict] || framing.verdict) + "。";
  el.verdict.appendChild(headline);
  el.verdict.appendChild(document.createTextNode(" " + framingNote(framing)));
  var detail = document.createElement("span");
  detail.textContent = "頭部在 "
    + Math.round(framing.nose_visible * 100) + "% 的畫面中可見，上半身有 "
    + Math.round(framing.upper_body_visible * 100) + "% 落在畫面內，"
    + "人體高度佔畫面 " + Math.round(framing.body_height_fraction * 100) + "%"
    + (ok ? "" : " \u2014 請重錄，讓人從頭到臀部都入鏡、正面對鏡頭、"
        + "並佔滿畫面大部分");
  el.verdict.appendChild(detail);
}

// ----------------------------------------------------------------- score

var rebuildTimer = null;

function rebuild(immediate) {
  // Invalidated here rather than when the new score arrives: clicking Save
  // blurs the name field, which fires a change, which schedules a rebuild, and
  // whichever request answered last used to decide whether Play was available.
  // Doing it on the edit itself makes the order the user's, not the network's.
  state.saved = null;
  el.play.disabled = true;
  if (rebuildTimer) { clearTimeout(rebuildTimer); }
  rebuildTimer = setTimeout(doRebuild, immediate ? 0 : 200);
}

function doRebuild() {
  if (!state.activeId || !state.analysis) { return; }
  var body = {
    name: el.name.value || undefined,
    keyframes: state.keyframes,
    gauss_window: Number(el.window.value),
    gauss_sigma: Number(el.sigma.value),
    derive_head: el.deriveHead.checked,
    acromion: el.acromion.checked,
    base_rotation: el.baseFirst.checked ? "first" : "every"
  };
  api("/api/jobs/" + state.activeId + "/score", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body)
  }).then(function (built) {
    state.built = built;
    drawEnergy(built.energy, built.keyframes);
    renderScore(built);
    say(el.editLog, "從 " + built.keyframes.length + " 個選定的幀產生 "
      + built.count + " 個關鍵幀"
      + (state.keyframes ? "（手動挑選）" : "（取自曲線波峰）"));
  }).catch(function (error) {
    say(el.editLog, error.message, "err");
  });
}

function renderScore(built) {
  var body = built.score[built.name];
  var tbody = el.score.querySelector("tbody");
  tbody.innerHTML = "";
  Object.keys(body).forEach(function (key) {
    var frame = body[key];
    var row = document.createElement("tr");
    var cells = [
      [key, ""],
      [(Number(frame["start time"][0]) / 1000).toFixed(2) + "s", ""],
      [frame.head.join(" / "), "head"],
      [frame["right elbow"].join(" / "), "sym"],
      [frame["right wrist"].join(" / "), "sym"],
      [frame["left elbow"].join(" / "), "sym"],
      [frame["left wrist"].join(" / "), "sym"]
    ];
    cells.forEach(function (pair) {
      var cell = document.createElement("td");
      cell.textContent = pair[0];
      if (pair[1]) { cell.className = pair[1]; }
      row.appendChild(cell);
    });
    tbody.appendChild(row);
  });
}

// ---------------------------------------------------------- energy curve

var PAD = { left: 8, right: 8, top: 10, bottom: 20 };

function plotGeometry(energy) {
  var ratio = window.devicePixelRatio || 1;
  var width = el.energy.clientWidth;
  var height = 150;
  if (el.energy.width !== width * ratio) {
    el.energy.width = width * ratio;
    el.energy.height = height * ratio;
  }
  var lo = Math.min.apply(null, energy);
  var hi = Math.max.apply(null, energy);
  var span = (hi - lo) || 1;
  return {
    ratio: ratio, width: width, height: height,
    x: function (i) {
      return PAD.left + (width - PAD.left - PAD.right)
        * (energy.length < 2 ? 0.5 : i / (energy.length - 1));
    },
    y: function (v) {
      return PAD.top + (height - PAD.top - PAD.bottom) * (1 - (v - lo) / span);
    },
    frameAt: function (px) {
      var t = (px - PAD.left) / (width - PAD.left - PAD.right);
      return Math.max(0, Math.min(energy.length - 1,
        Math.round(t * (energy.length - 1))));
    }
  };
}

function drawEnergy(energy, keyframes) {
  if (!energy || !energy.length) { return; }
  var g = plotGeometry(energy);
  var ctx = el.energy.getContext("2d");
  ctx.setTransform(g.ratio, 0, 0, g.ratio, 0, 0);
  ctx.clearRect(0, 0, g.width, g.height);

  // keyframes first, so the curve sits on top of them
  ctx.strokeStyle = "#3a5210";
  ctx.lineWidth = 1;
  keyframes.forEach(function (frame) {
    ctx.beginPath();
    ctx.moveTo(g.x(frame), PAD.top - 4);
    ctx.lineTo(g.x(frame), g.height - PAD.bottom);
    ctx.stroke();
  });

  ctx.strokeStyle = "#8d959c";
  ctx.lineWidth = 1.4;
  ctx.beginPath();
  energy.forEach(function (value, i) {
    if (i === 0) { ctx.moveTo(g.x(i), g.y(value)); }
    else { ctx.lineTo(g.x(i), g.y(value)); }
  });
  ctx.stroke();

  ctx.fillStyle = "#76b900";
  keyframes.forEach(function (frame) {
    ctx.beginPath();
    ctx.arc(g.x(frame), g.y(energy[frame]), 4, 0, Math.PI * 2);
    ctx.fill();
  });

  // a few time ticks, since frame numbers mean nothing to a performer
  var times = state.analysis ? state.analysis.times_ms : null;
  if (times) {
    ctx.fillStyle = "#8d959c";
    ctx.font = "11px system-ui, sans-serif";
    ctx.textAlign = "center";
    for (var k = 0; k <= 4; k++) {
      var i = Math.round((energy.length - 1) * k / 4);
      ctx.fillText((times[i] / 1000).toFixed(1) + "s",
                   g.x(i), g.height - 6);
    }
  }
}

el.energy.addEventListener("click", function (event) {
  if (!state.built) { return; }
  var energy = state.built.energy;
  var g = plotGeometry(energy);
  var box = el.energy.getBoundingClientRect();
  var frame = g.frameAt(event.clientX - box.left);

  var current = (state.keyframes || state.built.keyframes).slice();
  // Nearest existing keyframe within a few frames counts as a hit, because
  // clicking exactly on one is not realistic.
  var tolerance = Math.max(2, Math.round(energy.length / 60));
  var nearest = -1;
  var best = Infinity;
  current.forEach(function (value, index) {
    var distance = Math.abs(value - frame);
    if (distance < best) { best = distance; nearest = index; }
  });

  if (nearest >= 0 && best <= tolerance) {
    if (current.length <= 1) {
      say(el.editLog, "一份樂譜至少要有一個關鍵幀", "warn");
      return;
    }
    current.splice(nearest, 1);
  } else {
    current.push(frame);
  }
  state.keyframes = current.sort(function (a, b) { return a - b; });
  rebuild(true);
});

el.reset.addEventListener("click", function () {
  state.keyframes = null;
  rebuild(true);
});

[el.window, el.sigma].forEach(function (input) {
  input.addEventListener("input", function () {
    el.windowOut.textContent = el.window.value;
    el.sigmaOut.textContent = Number(el.sigma.value).toFixed(1);
    // Smoothing changes where the peaks are, so a hand-picked set would be
    // stale rather than preserved.
    state.keyframes = null;
    rebuild();
  });
});

[el.deriveHead, el.acromion, el.baseFirst].forEach(function (input) {
  input.addEventListener("change", function () { rebuild(true); });
});

el.name.addEventListener("input", function () {
  if (state.activeId) { state.names[state.activeId] = el.name.value; }
});
el.name.addEventListener("change", function () { rebuild(true); });

window.addEventListener("resize", function () {
  if (state.built) { drawEnergy(state.built.energy, state.built.keyframes); }
});

// ----------------------------------------------------------------- robot

el.save.addEventListener("click", function () {
  if (!state.activeId) { return; }
  el.save.disabled = true;
  api("/api/jobs/" + state.activeId + "/save", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      name: el.name.value || undefined,
      keyframes: state.keyframes,
      gauss_window: Number(el.window.value),
      gauss_sigma: Number(el.sigma.value),
      derive_head: el.deriveHead.checked,
      acromion: el.acromion.checked,
      base_rotation: el.baseFirst.checked ? "first" : "every"
    })
  }).then(function (reply) {
    state.saved = reply;
    el.play.disabled = false;
    say(el.robotLog, "已存入 " + reply.relative + "，共 " + reply.keyframes
      + " 個關鍵幀。" + reply.note, "ok");
    refreshStatus();
  }).catch(function (error) {
    say(el.robotLog, error.message, "err");
  }).then(function () {
    el.save.disabled = false;
  });
});

el.speed.addEventListener("input", function () {
  el.speedOut.textContent = Number(el.speed.value).toFixed(2) + TIMES;
});

el.play.addEventListener("click", function () {
  if (!state.saved) { return; }
  el.play.disabled = true;
  api("/api/play", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      file: state.saved.file,
      speed: Number(el.speed.value),
      head: el.playHead.checked
    })
  }).then(function (reply) {
    var how = reply.daemon ? "常駐播放器" : "另開播放行程";
    say(el.robotLog, "正在以" + how + "播放 " + state.saved.name, "ok");
  }).catch(function (error) {
    say(el.robotLog, error.message, "err");
  }).then(function () {
    el.play.disabled = false;
  });
});

el.stop.addEventListener("click", function () {
  api("/api/stop", { method: "POST" }).then(function () {
    say(el.robotLog, "已停止");
  }).catch(function (error) {
    say(el.robotLog, error.message, "err");
  });
});

// Recordings outlive the page. Reloading should pick the session back up
// rather than hide work the service is still holding.
function restore() {
  api("/api/jobs").then(function (reply) {
    if (!reply.jobs.length) { return; }
    reply.jobs.forEach(function (job) { state.jobs[job.id] = job; });
    var newest = reply.jobs[0];
    var ready = reply.jobs.filter(function (j) { return j.stage === "ready"; });
    select((ready.length ? ready[0] : newest).id);
    startPolling();
  }).catch(function (error) {
    // Said out loud rather than swallowed: a silent catch here once hid a
    // broken page for a whole debugging session.
    el.status.textContent = "無法列出錄影：" + error.message;
    el.status.className = "status err";
  });
}

refreshStatus();
restore();
setInterval(refreshStatus, 10000);
