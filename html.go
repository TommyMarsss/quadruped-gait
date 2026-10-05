package main

import (
	"encoding/json"
	"fmt"
	"os"
	"strings"
)

// PageData 是内嵌进回放 HTML 的全部数据。
type PageData struct {
	Frames   []Frame       `json:"frames"`
	Switches []SwitchEvent `json:"switches"`
	Params   SimParams     `json:"params"`
	Notes    []string      `json:"notes"`
}

// WriteHTML 生成单一静态 HTML 回放文件（数据与逻辑全部内嵌，
// 不依赖任何第三方库/框架，不需要启动服务）。
func WriteHTML(path string, data PageData) error {
	raw, err := json.Marshal(data) // encoding/json 默认转义 < > &，可安全内嵌 <script>
	if err != nil {
		return err
	}
	page := strings.Replace(replayTemplate, "__DATA__", string(raw), 1)
	if strings.Contains(page, "__DATA__") {
		return fmt.Errorf("模板数据占位符替换失败")
	}
	return os.WriteFile(path, []byte(page), 0o644)
}

const replayTemplate = `<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>四足机器人步态回放</title>
<style>
  :root { --ok:#2da44e; --bad:#cf222e; --ink:#1f2328; --muted:#656d76; --line:#d0d7de; }
  * { box-sizing: border-box; }
  body { font-family: -apple-system, "PingFang SC", "Helvetica Neue", Arial, sans-serif;
         margin: 0; padding: 16px 20px 40px; color: var(--ink); background:#f6f8fa; }
  h1 { font-size: 20px; margin: 4px 0; }
  h2 { font-size: 14px; margin: 0 0 6px; color: var(--muted); font-weight: 600; }
  .sub { color: var(--muted); font-size: 13px; margin-bottom: 12px; }
  #controls { display:flex; flex-wrap:wrap; gap:10px; align-items:center; margin:10px 0;
              padding:10px 12px; background:#fff; border:1px solid var(--line); border-radius:8px; }
  button { font-size:13px; padding:5px 14px; border:1px solid var(--line); border-radius:6px;
           background:#fff; cursor:pointer; }
  button:hover { background:#f0f2f5; }
  input[type=range] { flex:1; min-width:160px; }
  select { font-size:13px; padding:4px 6px; }
  .stat { font-size:13px; font-variant-numeric: tabular-nums; }
  .stat b { color: var(--muted); font-weight:600; margin-right:2px; }
  #warn { display:none; margin:10px 0; padding:8px 12px; border-radius:8px; font-size:13px;
          background:#ffebe9; border:1px solid var(--bad); color:var(--bad); font-weight:600; }
  body.unstable #warn { display:block; }
  main { display:flex; flex-wrap:wrap; gap:16px; margin-top:6px; }
  section.panel { background:#fff; border:1px solid var(--line); border-radius:8px; padding:10px; }
  body.unstable section.panel { border-color: var(--bad); box-shadow:0 0 0 2px rgba(207,34,46,.25); }
  canvas.view { display:block; }
  #timeline { display:block; width:100%; height:40px; margin-top:14px; background:#fff;
              border:1px solid var(--line); border-radius:8px; cursor:pointer; }
  #legend { display:flex; flex-wrap:wrap; gap:14px; font-size:12px; color:var(--muted); margin-top:10px; }
  .dot { display:inline-block; width:10px; height:10px; border-radius:50%; margin-right:4px; vertical-align:-1px; }
  #switches { font-size:12px; color:var(--muted); margin-top:8px; }
</style>
</head>
<body>
<h1>四足机器人步态回放（俯视图 + 侧视图）</h1>
<div class="sub">摆线摆动轨迹 · 支撑多边形静态稳定性判据 · 红色帧 = 稳定性裕度不足（支撑足&lt;3 或重心投影越出支撑多边形/裕度低于阈值）</div>
<div id="controls">
  <button id="play">▶ 播放</button>
  <input type="range" id="slider" min="0" value="0" step="1">
  <select id="speed">
    <option value="0.25">0.25×</option>
    <option value="0.5">0.5×</option>
    <option value="1" selected>1×</option>
    <option value="2">2×</option>
    <option value="4">4×</option>
  </select>
  <span class="stat"><b>t</b><span id="st-t">–</span></span>
  <span class="stat"><b>步态</b><span id="st-gait">–</span></span>
  <span class="stat"><b>支撑足</b><span id="st-stance">–</span></span>
  <span class="stat"><b>稳定裕度</b><span id="st-margin">–</span></span>
  <span class="stat"><b>判定</b><span id="st-stable">–</span></span>
</div>
<div id="warn">⚠ 当前帧静态稳定性裕度不足：重心投影不在支撑多边形内（或支撑足不足 3 只），几何上存在倾覆风险。</div>
<main>
  <section class="panel"><h2>俯视图（x 前向 →，y 左向 ↑；阴影 = 支撑多边形，✚ = 重心投影）</h2>
    <canvas id="top" class="view" width="600" height="440"></canvas></section>
  <section class="panel"><h2>侧视图（x 前向 →，z 竖直 ↑）</h2>
    <canvas id="side" class="view" width="600" height="440"></canvas></section>
</main>
<canvas id="timeline"></canvas>
<div id="legend">
  <span><span class="dot" style="background:#d62728"></span>FL 左前</span>
  <span><span class="dot" style="background:#1f77b4"></span>FR 右前</span>
  <span><span class="dot" style="background:#2ca02c"></span>RL 左后</span>
  <span><span class="dot" style="background:#ff7f0e"></span>RR 右后</span>
  <span>实心 = 支撑相，空心 = 摆动相（摆动足旁数字为离地高度 cm）</span>
  <span><span class="dot" style="background:#2da44e"></span>时间轴绿 = 稳定</span>
  <span><span class="dot" style="background:#cf222e"></span>红 = 裕度不足</span>
  <span><span class="dot" style="background:#bf8700"></span>黄线 = 步态切换点</span>
</div>
<div id="switches"></div>
<script>
"use strict";
const DATA = __DATA__;
const frames = DATA.frames, P = DATA.params, switches = DATA.switches;
const LEG_COLORS = ["#d62728", "#1f77b4", "#2ca02c", "#ff7f0e"];
const LEG_NAMES = ["FL", "FR", "RL", "RR"];
// 髋部相对机体中心的偏移，与 Go 端 nominal() 一致
const HIP = [[P.hipX, P.hipY], [P.hipX, -P.hipY], [-P.hipX, P.hipY], [-P.hipX, -P.hipY]];

const topC = document.getElementById("top"), tg = topC.getContext("2d");
const sideC = document.getElementById("side"), sg = sideC.getContext("2d");
const tlC = document.getElementById("timeline"), tl = tlC.getContext("2d");
const slider = document.getElementById("slider");
const playBtn = document.getElementById("play");
const speedSel = document.getElementById("speed");
slider.max = frames.length - 1;

let idx = 0, playing = false, acc = 0, lastTs = 0;
const frameDt = frames.length > 1 ? frames[1].t - frames[0].t : 0.01;

// Andrew 单调链凸包（与 Go 端一致），点格式 [x, y]
function hull2(pts) {
  const p = pts.slice().sort((a, b) => (a[0] - b[0]) || (a[1] - b[1]));
  if (p.length < 3) return p;
  const cross = (o, a, b) => (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0]);
  const lower = [];
  for (const q of p) { while (lower.length >= 2 && cross(lower[lower.length-2], lower[lower.length-1], q) <= 0) lower.pop(); lower.push(q); }
  const upper = [];
  for (let i = p.length - 1; i >= 0; i--) { const q = p[i]; while (upper.length >= 2 && cross(upper[upper.length-2], upper[upper.length-1], q) <= 0) upper.pop(); upper.push(q); }
  lower.pop(); upper.pop();
  return lower.concat(upper);
}

function drawTop(f) {
  const W = topC.width, H = topC.height, S = 190;
  const cx = W / 2, cy = H / 2;
  const X = wx => cx + (wx - f.body.x) * S;
  const Y = wy => cy - (wy - f.body.y) * S;
  tg.clearRect(0, 0, W, H);
  // 网格（0.25 m）
  tg.strokeStyle = "#eaeef2"; tg.lineWidth = 1; tg.beginPath();
  const x0 = Math.floor((f.body.x - cx / S) / 0.25) * 0.25;
  for (let gx = x0; gx < f.body.x + cx / S + 0.25; gx += 0.25) { tg.moveTo(X(gx), 0); tg.lineTo(X(gx), H); }
  const y0 = Math.floor((f.body.y - cy / S) / 0.25) * 0.25;
  for (let gy = y0; gy < f.body.y + cy / S + 0.25; gy += 0.25) { tg.moveTo(0, Y(gy)); tg.lineTo(W, Y(gy)); }
  tg.stroke();
  // 足端历史轨迹（最近 50 帧，渐隐）
  for (let k = Math.max(0, idx - 50); k < idx; k++) {
    const a = (k - (idx - 50)) / 50 * 0.35;
    for (let i = 0; i < 4; i++) {
      tg.fillStyle = LEG_COLORS[i];
      tg.globalAlpha = a;
      tg.beginPath(); tg.arc(X(frames[k].feet[i].x), Y(frames[k].feet[i].y), 2, 0, 6.2832); tg.fill();
    }
  }
  tg.globalAlpha = 1;
  // 支撑多边形
  const sp = [];
  for (let i = 0; i < 4; i++) if (f.stance[i]) sp.push([f.feet[i].x, f.feet[i].y]);
  const col = f.stable ? "#2da44e" : "#cf222e";
  if (sp.length >= 3) {
    const h = hull2(sp);
    tg.beginPath();
    tg.moveTo(X(h[0][0]), Y(h[0][1]));
    for (let k = 1; k < h.length; k++) tg.lineTo(X(h[k][0]), Y(h[k][1]));
    tg.closePath();
    tg.fillStyle = f.stable ? "rgba(45,164,78,0.18)" : "rgba(207,34,46,0.22)";
    tg.fill();
    tg.strokeStyle = col; tg.lineWidth = 2; tg.stroke();
  } else if (sp.length === 2) {
    tg.beginPath();
    tg.moveTo(X(sp[0][0]), Y(sp[0][1])); tg.lineTo(X(sp[1][0]), Y(sp[1][1]));
    tg.strokeStyle = col; tg.lineWidth = 3; tg.setLineDash([6, 4]); tg.stroke(); tg.setLineDash([]);
  }
  // 机体
  tg.strokeStyle = "#1f2328"; tg.lineWidth = 2;
  tg.strokeRect(X(f.body.x - P.hipX - 0.04), Y(f.body.y + P.hipY + 0.03),
                (2 * P.hipX + 0.08) * S, (2 * P.hipY + 0.06) * S);
  // 腿（髋 → 足）
  for (let i = 0; i < 4; i++) {
    tg.beginPath();
    tg.moveTo(X(f.body.x + HIP[i][0]), Y(f.body.y + HIP[i][1]));
    tg.lineTo(X(f.feet[i].x), Y(f.feet[i].y));
    tg.strokeStyle = LEG_COLORS[i]; tg.lineWidth = 1.5; tg.stroke();
  }
  // 足端
  tg.font = "10px sans-serif";
  for (let i = 0; i < 4; i++) {
    const fx = X(f.feet[i].x), fy = Y(f.feet[i].y);
    tg.beginPath(); tg.arc(fx, fy, 6, 0, 6.2832);
    if (f.stance[i]) { tg.fillStyle = LEG_COLORS[i]; tg.fill(); }
    else { tg.strokeStyle = LEG_COLORS[i]; tg.lineWidth = 2; tg.stroke();
           tg.fillStyle = LEG_COLORS[i]; tg.fillText(Math.round(f.feet[i].z * 100) + "", fx + 8, fy - 6); }
  }
  // 重心投影
  tg.strokeStyle = "#1f2328"; tg.lineWidth = 2; tg.beginPath();
  tg.moveTo(X(f.body.x) - 8, Y(f.body.y)); tg.lineTo(X(f.body.x) + 8, Y(f.body.y));
  tg.moveTo(X(f.body.x), Y(f.body.y) - 8); tg.lineTo(X(f.body.x), Y(f.body.y) + 8);
  tg.stroke();
}

function drawSide(f) {
  const W = sideC.width, H = sideC.height, S = 190;
  const cx = W / 2, ground = H - 50;
  const X = wx => cx + (wx - f.body.x) * S;
  const Z = wz => ground - wz * S;
  sg.clearRect(0, 0, W, H);
  // 地面
  sg.strokeStyle = "#57606a"; sg.lineWidth = 2; sg.beginPath();
  sg.moveTo(0, ground); sg.lineTo(W, ground); sg.stroke();
  sg.strokeStyle = "#d0d7de"; sg.lineWidth = 1; sg.beginPath();
  const x0 = Math.floor((f.body.x - cx / S) / 0.25) * 0.25;
  for (let gx = x0; gx < f.body.x + cx / S + 0.25; gx += 0.25) { sg.moveTo(X(gx), ground); sg.lineTo(X(gx), ground + 6); }
  sg.stroke();
  // 重心高度参考线
  sg.setLineDash([4, 4]); sg.strokeStyle = "#bf8700"; sg.beginPath();
  sg.moveTo(0, Z(P.bodyH)); sg.lineTo(W, Z(P.bodyH)); sg.stroke(); sg.setLineDash([]);
  // 机身
  sg.strokeStyle = "#1f2328"; sg.lineWidth = 4; sg.beginPath();
  sg.moveTo(X(f.body.x - P.hipX - 0.05), Z(f.body.z));
  sg.lineTo(X(f.body.x + P.hipX + 0.05), Z(f.body.z)); sg.stroke();
  // 腿（侧视只画左侧 FL/RL 与右侧 FR/RR 各两条，投影到 x-z 平面）
  for (let i = 0; i < 4; i++) {
    sg.beginPath();
    sg.moveTo(X(f.body.x + HIP[i][0]), Z(f.body.z));
    sg.lineTo(X(f.feet[i].x), Z(f.feet[i].z));
    sg.strokeStyle = LEG_COLORS[i]; sg.lineWidth = 2; sg.stroke();
    sg.beginPath(); sg.arc(X(f.feet[i].x), Z(f.feet[i].z), 5, 0, 6.2832);
    if (f.stance[i]) { sg.fillStyle = LEG_COLORS[i]; sg.fill(); }
    else { sg.strokeStyle = LEG_COLORS[i]; sg.lineWidth = 2; sg.stroke(); }
  }
  // 重心
  sg.fillStyle = "#1f2328";
  sg.beginPath(); sg.arc(X(f.body.x), Z(f.body.z), 5, 0, 6.2832); sg.fill();
}

function drawTimeline() {
  const dpr = window.devicePixelRatio || 1;
  const w = tlC.clientWidth, h = tlC.clientHeight;
  if (tlC.width !== w * dpr) { tlC.width = w * dpr; tlC.height = h * dpr; }
  tl.setTransform(dpr, 0, 0, dpr, 0, 0);
  tl.clearRect(0, 0, w, h);
  const bw = Math.max(1, w / frames.length);
  for (let i = 0; i < frames.length; i++) {
    tl.fillStyle = frames[i].stable ? "#2da44e" : "#cf222e";
    tl.fillRect(i / frames.length * w, 6, bw, h - 12);
  }
  tl.fillStyle = "#bf8700";
  for (const sw of switches) {
    let k = 0; while (k < frames.length - 1 && frames[k].t < sw.t) k++;
    tl.fillRect(k / frames.length * w - 1, 0, 2, h);
  }
  tl.fillStyle = "#1f2328";
  tl.fillRect(idx / frames.length * w - 1, 0, 2, h);
}

function render() {
  const f = frames[idx];
  drawTop(f); drawSide(f); drawTimeline();
  slider.value = idx;
  document.getElementById("st-t").textContent = f.t.toFixed(2) + " s";
  document.getElementById("st-gait").textContent = f.gait;
  let ns = 0; for (let i = 0; i < 4; i++) if (f.stance[i]) ns++;
  document.getElementById("st-stance").textContent = ns + " / 4";
  document.getElementById("st-margin").textContent = f.margin <= -1 ? "—（支撑足<3）" : f.margin.toFixed(3) + " m";
  const st = document.getElementById("st-stable");
  st.textContent = f.stable ? "静态稳定" : "裕度不足";
  st.style.color = f.stable ? "#2da44e" : "#cf222e";
  document.body.classList.toggle("unstable", !f.stable);
}

function tick(ts) {
  if (playing) {
    if (!lastTs) lastTs = ts;
    acc += (ts - lastTs) / 1000 * parseFloat(speedSel.value);
    while (acc >= frameDt && idx < frames.length - 1) { idx++; acc -= frameDt; }
    if (idx >= frames.length - 1) { playing = false; playBtn.textContent = "▶ 播放"; }
    render();
  }
  lastTs = ts;
  requestAnimationFrame(tick);
}

playBtn.onclick = () => {
  if (idx >= frames.length - 1) idx = 0;
  playing = !playing;
  playBtn.textContent = playing ? "⏸ 暂停" : "▶ 播放";
};
slider.oninput = () => { idx = parseInt(slider.value, 10); render(); };
tlC.onclick = e => {
  const r = tlC.getBoundingClientRect();
  idx = Math.min(frames.length - 1, Math.max(0, Math.floor((e.clientX - r.left) / r.width * frames.length)));
  render();
};
window.addEventListener("keydown", e => { if (e.code === "Space") { e.preventDefault(); playBtn.onclick(); } });
window.addEventListener("resize", drawTimeline);

document.getElementById("switches").textContent =
  switches.length ? "步态切换事件：" + switches.map(s => s.from + " → " + s.to + " @ t=" + s.t.toFixed(2) + "s").join("；")
                  : "本次回放无步态切换。";
render();
requestAnimationFrame(tick);
</script>
</body>
</html>
`
