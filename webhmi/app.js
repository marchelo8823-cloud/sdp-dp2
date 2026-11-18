/* SDP-DP2 Web-HMI — фронтенд пульта (без зависимостей). */
"use strict";
const $ = id => document.getElementById(id);
const NS = "http://www.w3.org/2000/svg";
const D2R = Math.PI / 180;

function el(name, attrs, parent) {
  const e = document.createElementNS(NS, name);
  for (const k in attrs) e.setAttribute(k, attrs[k]);
  if (parent) parent.appendChild(e);
  return e;
}
function cmd(o) {
  fetch("/cmd", { method: "POST", body: JSON.stringify(o) }).catch(() => {});
}

/* ---------- статические части SVG ---------- */
// Обвод судна (нос вверх = север), длина 76 м, в метрах
const SHIP = "M38 0 L12 9 L-30 9 L-38 4 L-38 -4 L-30 -9 L12 -9 Z";

// --- position plot ---
const pp = $("posplot");
const ppGrid = el("g", {}, pp);
const ppScale = el("g", {}, pp);          // масштаб метры→экран
const ppTrail = el("polyline", { fill: "none", stroke: "#4d8dff",
  "stroke-width": 0.8, "stroke-opacity": 0.55 }, ppScale);
const ppWarn = el("circle", { cx: 0, cy: 0, fill: "none", stroke: "#ffc542",
  "stroke-width": 0.6, "stroke-dasharray": "2 2" }, ppScale);
const ppAlarm = el("circle", { cx: 0, cy: 0, fill: "none", stroke: "#ff5252",
  "stroke-width": 0.7, "stroke-dasharray": "3 2" }, ppScale);
el("circle", { cx: 0, cy: 0, r: 1.2, fill: "#ff5252" }, ppScale); // уставка
const ppShip = el("g", {}, ppScale);
el("path", { d: SHIP, fill: "#4d8dff", "fill-opacity": 0.9,
  stroke: "#0a0f1a", "stroke-width": 0.8 }, ppShip);
el("circle", { cx: 0, cy: 0, r: 1.5, fill: "#fff" }, ppShip);
const ppWindTxt = el("text", { x: -1.04, y: 1.0, fill: "#39c2d7",
  "font-size": 0.055 }, pp);
const ppWindArr = el("path", { d: "M0 0 L0.09 0 M0.09 0 L0.06 -0.02 M0.09 0 L0.06 0.02",
  stroke: "#39c2d7", "stroke-width": 0.012, fill: "none" }, pp);

function drawPPGrid(range) {
  ppGrid.innerHTML = "";
  el("circle", { cx: 0, cy: 0, r: 1, fill: "#0d1524", stroke: "#1e2c47",
    "stroke-width": 0.004 }, ppGrid);
  for (let f = 1; f <= 3; f++)
    el("circle", { cx: 0, cy: 0, r: f / 3, fill: "none", stroke: "#1e2c47",
      "stroke-width": 0.003, "stroke-dasharray": "0.01 0.01" }, ppGrid);
  for (let a = 0; a < 360; a += 10) {
    const big = a % 30 === 0, r1 = big ? 0.955 : 0.975;
    el("line", { x1: r1 * Math.sin(a * D2R), y1: -r1 * Math.cos(a * D2R),
      x2: Math.sin(a * D2R), y2: -Math.cos(a * D2R), stroke: "#5f7396",
      "stroke-width": 0.004 }, ppGrid);
    if (big) {
      const t = el("text", { x: 0.9 * Math.sin(a * D2R),
        y: -0.9 * Math.cos(a * D2R) + 0.014, fill: "#5f7396",
        "font-size": 0.038, "text-anchor": "middle" }, ppGrid);
      t.textContent = a;
    }
  }
  // подписи дистанций колец
  for (let f = 1; f <= 3; f++) {
    const t = el("text", { x: 0.012, y: -f / 3 + 0.012, fill: "#5f7396",
      "font-size": 0.034 }, ppGrid);
    t.textContent = Math.round(range * f / 3);
  }
  const n = el("text", { x: -0.04, y: -1.03, fill: "#c9d6ea",
    "font-size": 0.06, "font-weight": "bold" }, ppGrid);
  n.textContent = "N";
}

// --- compass ---
const cp = $("compass");
(function buildCompass() {
  el("circle", { cx: 0, cy: 0, r: 1, fill: "#0d1524", stroke: "#1e2c47",
    "stroke-width": 0.008 }, cp);
  for (let a = 0; a < 360; a += 5) {
    const big = a % 30 === 0, r1 = big ? 0.86 : 0.93;
    el("line", { x1: r1 * Math.sin(a * D2R), y1: -r1 * Math.cos(a * D2R),
      x2: 0.99 * Math.sin(a * D2R), y2: -0.99 * Math.cos(a * D2R),
      stroke: big ? "#c9d6ea" : "#5f7396",
      "stroke-width": big ? 0.012 : 0.006 }, cp);
    if (a % 90 === 0) {
      const t = el("text", { x: 0.72 * Math.sin(a * D2R),
        y: -0.72 * Math.cos(a * D2R) + 0.05, fill: "#c9d6ea",
        "font-size": 0.13, "text-anchor": "middle", "font-weight": "bold" }, cp);
      t.textContent = "NESW"[a / 90];
    }
  }
})();
const cpBug = el("path", { d: "M0 -1.02 L0.05 -0.9 L-0.05 -0.9 Z",
  fill: "#2fd07c" }, cp);                      // жучок уставки
const cpShip = el("path", { d: "M0 -0.62 L0.16 0.42 L0 0.25 L-0.16 0.42 Z",
  fill: "#4d8dff", stroke: "#0a0f1a", "stroke-width": 0.02 }, cp);
const cpTxt = el("text", { x: 0, y: 0.06, fill: "#c9d6ea", "font-size": 0.3,
  "text-anchor": "middle", "font-weight": "bold",
  "font-variant-numeric": "tabular-nums" }, cp);

// --- wind rose ---
const wr = $("windrose");
(function buildWind() {
  el("circle", { cx: 0, cy: 0, r: 1, fill: "#0d1524", stroke: "#1e2c47",
    "stroke-width": 0.008 }, wr);
  for (let a = 0; a < 360; a += 30) {
    el("line", { x1: 0.9 * Math.sin(a * D2R), y1: -0.9 * Math.cos(a * D2R),
      x2: Math.sin(a * D2R), y2: -Math.cos(a * D2R), stroke: "#5f7396",
      "stroke-width": 0.008 }, wr);
    const t = el("text", { x: 0.74 * Math.sin(a * D2R),
      y: -0.74 * Math.cos(a * D2R) + 0.035, fill: "#5f7396",
      "font-size": 0.09, "text-anchor": "middle" }, wr);
    t.textContent = a;
  }
  el("path", { d: "M0 -0.5 L0.12 0.34 L0 0.2 L-0.12 0.34 Z",
    fill: "none", stroke: "#5f7396", "stroke-width": 0.02 }, wr);
})();
const wrArr = el("path", { d: "M0 0.75 L0.07 0.15 L0 0.22 L-0.07 0.15 Z M0 0.75 L0 0.15",
  fill: "#39c2d7", stroke: "#39c2d7", "stroke-width": 0.02 }, wr);

// --- thruster mimic ---
const mm = $("mimic");
(function buildMimic() {
  // корпус: x = east(ly), y = -north(lx); нос сверху
  el("path", { d: "M0 -1.0 L0.28 -0.72 L0.28 0.72 L0.14 1.0 L-0.14 1.0 L-0.28 0.72 L-0.28 -0.72 Z",
    fill: "#0d1524", stroke: "#5f7396", "stroke-width": 0.02 }, mm);
})();
const thrEls = [];
const THR_POS = [[30, 0], [18, 0], [-30, 6], [-30, -6]];  // (lx, ly), м
for (let i = 0; i < 4; i++) {
  const [lx, ly] = THR_POS[i];
  const g = el("g", { transform: `translate(${ly / 32}, ${-lx / 34})` }, mm);
  const dial = el("circle", { cx: 0, cy: 0, r: 0.14, fill: "#111a2c",
    stroke: "#5f7396", "stroke-width": 0.015 }, g);
  const needle = el("line", { x1: 0, y1: 0, x2: 0, y2: -0.14,
    stroke: "#4d8dff", "stroke-width": 0.035, "stroke-linecap": "round" }, g);
  const bar = el("rect", { x: 0.18, y: -0.1, width: 0.05, height: 0,
    fill: "#4d8dff" }, g);
  const txt = el("text", { x: 0, y: 0.24, fill: "#c9d6ea", "font-size": 0.09,
    "text-anchor": "middle", "font-variant-numeric": "tabular-nums" }, g);
  thrEls.push({ g, dial, needle, bar, txt });
}

/* ---------- цикл телеметрии ---------- */
let trail = [], trend = [], lastTrailT = -1, range = 100;

async function tick() {
  let s;
  try { s = await (await fetch("/state")).json(); }
  catch (e) { return; }

  // часы модельного времени
  const mm2 = String(Math.floor(s.t / 60)).padStart(2, "0"),
        ss = String(Math.floor(s.t % 60)).padStart(2, "0");
  $("simclock").innerHTML = `<b>T+${mm2}:${ss}${s.paused ? " ⏸" : ""} · ${s.speed}×</b>`;

  // статус
  const st = $("dpstatus");
  if (s.alarm_worst >= 2 || s.conseq.level >= 2) {
    st.className = "lamp alarm"; st.textContent = "DP STATUS: ALARM";
  } else if (s.alarm_worst === 1 || s.conseq.level === 1 ||
             s.pos_err > s.warn_pos) {
    st.className = "lamp warn"; st.textContent = "DP STATUS: WARNING";
  } else {
    st.className = "lamp ok"; st.textContent = "DP STATUS: NORMAL";
  }
  const cq = $("conseq");
  cq.textContent = s.conseq.text;
  cq.className = ["ok", "warn", "alarm"][s.conseq.level];
  const bg = $("albadge");
  bg.textContent = s.unacked;
  bg.className = "badge" + (s.unacked ? "" : " zero");

  // --- position plot ---
  const newRange = Math.min(500, Math.max(30, 1.5 * s.pos_err,
                                          1.7 * s.alarm_pos));
  if (Math.abs(newRange - range) / range > 0.15) {
    range = newRange;
    drawPPGrid(range);
    $("range_lbl").textContent = `range ${Math.round(range)} м`;
  }
  ppScale.setAttribute("transform", `scale(${1 / range})`);
  ppWarn.setAttribute("r", s.warn_pos);
  ppWarn.setAttribute("stroke-width", 0.6 * range / 100);
  ppAlarm.setAttribute("r", s.alarm_pos);
  ppAlarm.setAttribute("stroke-width", 0.7 * range / 100);
  ppTrail.setAttribute("stroke-width", 0.8 * range / 100);
  const ex = s.err_e, ey = -s.err_n;
  ppShip.setAttribute("transform", `translate(${ex}, ${ey}) rotate(${s.hdg_deg})`);
  if (s.t - lastTrailT > 1.0) {
    trail.push([ex, ey]);
    if (trail.length > 600) trail.shift();
    ppTrail.setAttribute("points", trail.map(p => p.join(",")).join(" "));
    lastTrailT = s.t;
  }
  ppWindArr.setAttribute("transform",
    `translate(-0.98, 0.88) rotate(${s.wind_meas.dir_deg + 180})`);
  ppWindTxt.textContent = `Ветер ${s.wind_meas.dir_deg.toFixed(0)}°  ${s.wind_meas.speed_kn.toFixed(1)} kn`;

  // --- compass ---
  cpShip.setAttribute("transform", `rotate(${s.hdg_deg})`);
  cpBug.setAttribute("transform", `rotate(${s.hdg_d_deg})`);
  cpTxt.textContent = s.hdg_deg.toFixed(1) + "°";
  $("rot").textContent = s.rot_dpm.toFixed(1) + " °/min";
  $("hdev").textContent = s.hdg_err_deg.toFixed(1) + "°";
  $("sog").textContent = s.sog_kn.toFixed(2) + " kn";
  $("gyros").innerHTML = s.sensors.gyro.map((f, i) =>
    `<span class="led ${f ? "off" : ""}"></span>G${i + 1}`).join(" ");

  // --- thrusters ---
  s.thrusters.forEach((th, i) => {
    const e = thrEls[i];
    e.needle.setAttribute("transform", `rotate(${th.alpha_deg})`);
    e.needle.setAttribute("stroke", th.failed ? "#ff5252" : "#4d8dff");
    e.dial.setAttribute("stroke", th.failed ? "#ff5252" : "#5f7396");
    const h = Math.min(Math.abs(th.u_pct) / 100, 1) * 0.2;
    e.bar.setAttribute("height", h.toFixed(3));
    e.bar.setAttribute("y", th.u_pct >= 0 ? (-0.1 - h + 0.2) : -0.1);
    e.bar.setAttribute("fill", th.failed ? "#ff5252" : "#4d8dff");
    e.txt.textContent = th.failed ? "ОТКАЗ"
      : `${th.alpha_deg.toFixed(0)}° ${Math.abs(th.u_pct).toFixed(0)}%`;
  });
  $("resf").textContent = s.resultant.force_kn.toFixed(0) + " kN";
  $("resd").textContent = s.resultant.dir_deg.toFixed(1) + "°";
  $("resm").textContent = s.resultant.moment_knm.toFixed(0) + " kNm";

  // --- wind / sensors / numeric ---
  wrArr.setAttribute("transform", `rotate(${s.wind_true.dir_deg + 180})`);
  $("wind_m").textContent = `${s.wind_meas.speed_kn.toFixed(1)} kn  ${s.wind_meas.dir_deg.toFixed(0)}°`;
  $("wind_t").textContent = `${s.wind_true.speed_kn.toFixed(1)} kn  ${s.wind_true.dir_deg.toFixed(0)}°`;
  $("wave").textContent = `${s.hs.toFixed(1)} м / ${s.tp.toFixed(0)} с`;
  $("sensors").innerHTML =
    s.sensors.prs.map((f, i) =>
      `<div class="kv"><span><span class="led ${f ? "off" : ""}"></span>PRS ${i + 1}</span><b>${f ? "ОТКАЗ" : "OK"}</b></div>`).join("") +
    s.sensors.gyro.map((f, i) =>
      `<div class="kv"><span><span class="led ${f ? "off" : ""}"></span>ГИРО ${i + 1}</span><b>${f ? "ОТКАЗ" : "OK"}</b></div>`).join("");
  $("perr_ne").textContent = `${s.err_n.toFixed(2)} / ${s.err_e.toFixed(2)} м`;
  const pe = $("perr");
  pe.textContent = s.pos_err.toFixed(2) + " м";
  pe.style.color = s.pos_err > s.alarm_pos ? "var(--red)"
    : s.pos_err > s.warn_pos ? "var(--yellow)" : "var(--green)";
  $("bias").textContent = s.bias_kn.toFixed(1) + " kN";

  // --- alarms ---
  $("alarms").innerHTML = s.alarms.map(a =>
    `<div class="a l${a.level}${a.acked ? " acked" : ""}"><span class="t">${a.t.toFixed(0)}с</span>${a.text}</div>`).join("");

  // --- trends ---
  trend.push({ t: s.t, e: s.pos_err, w: s.wind_meas.speed_kn,
               th: Math.abs(s.resultant.force_kn) });
  if (trend.length > 2400) trend.shift();
  drawTrends();
}

function drawTrends() {
  const c = $("trends"), ctx = c.getContext("2d");
  const W = c.width, H = c.height;
  ctx.clearRect(0, 0, W, H);
  const n = trend.length;
  if (n < 2) return;
  const series = [
    { k: "e", max: 8, col: "#ff9f43" },
    { k: "w", max: 45, col: "#39c2d7" },
    { k: "th", max: 400, col: "#2fd07c" },
  ];
  ctx.strokeStyle = "#1e2c47";
  for (let i = 1; i < 4; i++) {
    ctx.beginPath(); ctx.moveTo(0, H * i / 4); ctx.lineTo(W, H * i / 4); ctx.stroke();
  }
  for (const sr of series) {
    ctx.strokeStyle = sr.col; ctx.lineWidth = 1.6; ctx.beginPath();
    trend.forEach((p, i) => {
      const x = i / (n - 1) * W, y = H - Math.min(p[sr.k] / sr.max, 1) * H * 0.94 - 3;
      i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
    });
    ctx.stroke();
  }
}

/* ---------- органы управления ---------- */
$("ack").onclick = () => cmd({ action: "ack" });
$("albadge").onclick = () => cmd({ action: "ack" });
$("pause").onclick = () => cmd({ action: "pause" });
$("reset").onclick = () => { if (confirm("Сбросить симуляцию?")) cmd({ action: "reset" }); };
$("hdgm").onclick = () => cmd({ action: "heading_bump", d_deg: -10 });
$("hdgp").onclick = () => cmd({ action: "heading_bump", d_deg: 10 });
$("theme").onclick = () => document.body.classList.toggle("day");
$("wv").oninput = e => { $("wv_lbl").textContent = e.target.value + " м/с";
  cmd({ action: "wind", speed_ms: +e.target.value, dir_deg: +$("wd").value }); };
$("wd").oninput = e => { $("wd_lbl").textContent = e.target.value + "°";
  cmd({ action: "wind", speed_ms: +$("wv").value, dir_deg: +e.target.value }); };
$("hs").oninput = e => { $("hs_lbl").textContent = (+e.target.value).toFixed(1) + " м";
  cmd({ action: "hs", hs: +e.target.value }); };
$("spd").oninput = e => { $("sp_lbl").textContent = e.target.value + "×";
  cmd({ action: "speed", value: +e.target.value }); };
$("wcset").onclick = () => cmd({ action: "limits",
  warn: +$("wcw").value, alarm: +$("wca").value });
document.querySelectorAll("[data-f]").forEach(b => b.onclick = () => {
  const [kind, idx] = b.dataset.f.split(":");
  cmd({ action: "fail", kind, idx });
});
// уставка кликом по position plot
pp.addEventListener("click", ev => {
  const r = pp.getBoundingClientRect();
  const px = (ev.clientX - r.left) / r.width * 2.16 - 1.08;
  const py = (ev.clientY - r.top) / r.height * 2.16 - 1.08;
  fetch("/state").then(r2 => r2.json()).then(s => {
    cmd({ action: "setpoint", north: s.eta_d[0] - py * range,
                              east: s.eta_d[1] + px * range });
  });
});

drawPPGrid(range);
tick();
setInterval(tick, 250);
