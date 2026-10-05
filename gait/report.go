package gait

import (
	"bytes"
	"encoding/json"
)

// reportTemplate 是单文件 HTML 报告模板。运行数据以 JSON 形式内嵌在
// <script type="application/json"> 中，页面逻辑全部为原生 JS，
// 不引用任何外部资源、框架或图形库。
const reportTemplate = `<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>四足步态回放与静态稳定性检测</title>
<style>
  :root { --bg:#0f1419; --panel:#171e26; --ink:#d8e0e8; --dim:#7d8a99;
          --ok:#3ecf8e; --bad:#ff5d5d; --warn:#ffb454; --line:#2a3542;
          --fl:#5aa9ff; --fr:#8f7bff; --bl:#34d4c5; --br:#ff8fa3; }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--ink);
         font:13px/1.5 -apple-system,"PingFang SC","Microsoft YaHei",sans-serif; }
  header { padding:10px 16px; border-bottom:1px solid var(--line);
           display:flex; align-items:baseline; gap:14px; flex-wrap:wrap; }
  header h1 { font-size:15px; margin:0; }
  .stat { color:var(--dim); }
  .stat b { color:var(--ink); font-weight:600; }
  #badges { margin-left:auto; display:flex; gap:8px; }
  .badge { padding:2px 10px; border-radius:10px; font-size:12px;
           border:1px solid var(--line); color:var(--dim); background:var(--panel); }
  .badge.ok  { color:var(--ok);  border-color:var(--ok); }
  .badge.bad { color:var(--bad); border-color:var(--bad); }
  main { padding:12px 16px; }
  .views { display:grid; grid-template-columns:1fr 1fr; gap:12px; }
  @media (max-width:980px){ .views{ grid-template-columns:1fr; } }
  .panel { background:var(--panel); border:1px solid var(--line);
           border-radius:8px; padding:8px; }
  .panel h2 { margin:0 0 6px 4px; font-size:12px; color:var(--dim);
              font-weight:600; letter-spacing:.5px; }
  canvas { width:100%; display:block; background:#0c1116; border-radius:6px; }
  .controls { margin-top:12px; background:var(--panel); border:1px solid var(--line);
              border-radius:8px; padding:10px 12px; }
  .ctrl-row { display:flex; align-items:center; gap:10px; flex-wrap:wrap; }
  button { background:#222c38; color:var(--ink); border:1px solid var(--line);
           border-radius:5px; padding:4px 12px; cursor:pointer; font-size:13px; }
  button:hover { border-color:var(--fl); }
  input[type=range]{ flex:1; min-width:200px; accent-color:var(--fl); }
  #scrub { position:relative; height:22px; margin:8px 2px 2px; cursor:pointer; }
  #scrubTrack { position:absolute; left:0; right:0; top:9px; height:4px;
                border-radius:2px; background:#26313d; }
  #scrubDanger { position:absolute; top:9px; height:4px; background:
                 repeating-linear-gradient(45deg,var(--bad),var(--bad) 4px,
                 #b03a3a 4px,#b03a3a 8px); border-radius:2px; }
  #scrubKnob { position:absolute; top:4px; width:6px; height:14px;
               margin-left:-3px; background:var(--ink); border-radius:2px; }
  #scrubSwitches { position:absolute; top:1px; bottom:1px; width:2px;
                   background:var(--warn); }
  .readout { display:grid; grid-template-columns:repeat(auto-fit,minmax(150px,1fr));
             gap:6px 18px; margin-top:8px; }
  .readout div span { color:var(--dim); }
  #reason { margin-top:6px; font-size:12px; min-height:18px; }
  #reason.bad { color:var(--bad); }
  #reason.ok  { color:var(--ok); }
  .legend { display:flex; gap:14px; flex-wrap:wrap; margin:6px 2px 0;
            color:var(--dim); font-size:12px; }
  .legend i { display:inline-block; width:10px; height:10px; border-radius:50%;
              margin-right:5px; vertical-align:-1px; }
  .footnote { color:var(--dim); font-size:12px; margin:14px 2px 0; }
  .footnote code { color:var(--warn); }
</style>
</head>
<body>
<header>
  <h1>四足步态回放 · 静态稳定性检测</h1>
  <span class="stat">序列：<b id="mName"></b></span>
  <span class="stat">时长 <b id="mDur"></b> s · <b id="mFrames"></b> 帧</span>
  <span class="stat">机身尺寸 a×b=<b id="mGeo"></b> m</span>
  <div id="badges">
    <span class="badge" id="bStable">--</span>
    <span class="badge" id="bGait">--</span>
  </div>
</header>
<main>
  <div class="views">
    <div class="panel">
      <h2>俯视 X–Y（相机随机身）· 多边形=支撑凸包，★=重心投影</h2>
      <canvas id="top" height="380"></canvas>
    </div>
    <div class="panel">
      <h2>侧视 X–Z（摆动抬脚弧清晰可见）· 竖直虚线=重心投影</h2>
      <canvas id="side" height="380"></canvas>
    </div>
  </div>
  <div class="legend">
    <span><i style="background:var(--fl)"></i>FL 左前</span>
    <span><i style="background:var(--fr)"></i>FR 右前</span>
    <span><i style="background:var(--bl)"></i>BL 左后</span>
    <span><i style="background:var(--br)"></i>BR 右后</span>
    <span><i style="background:var(--ok)"></i>支撑多边形/静态稳定</span>
    <span><i style="background:var(--bad)"></i>裕度不足帧</span>
    <span><i style="background:var(--warn)"></i>步态切换点</span>
    <span>空心圆=摆动相，实心圆=支撑相（半径∝抬脚高度）</span>
  </div>
  <div class="controls">
    <div class="ctrl-row">
      <button id="btnPlay">⏸ 暂停</button>
      <button id="btnPrev">‹ 不稳定</button>
      <button id="btnNext">不稳定 ›</button>
      <span class="stat">速度</span><input id="speed" type="range" min="0.1" max="3" step="0.1" value="1">
      <span class="stat" id="spdLab">1.0×</span>
    </div>
    <div id="scrub">
      <div id="scrubTrack"></div>
      <div id="scrubDanger"></div>
      <div id="scrubSwitches"></div>
      <div id="scrubKnob"></div>
    </div>
    <div class="readout">
      <div><span>时间 t：</span><b id="rT"></b> s</div>
      <div><span>步态/模式：</span><b id="rGait"></b></div>
      <div><span>周期相位：</span><b id="rPhase"></b></div>
      <div><span>机身速度：</span><b id="rV"></b> m/s</div>
      <div><span>支撑脚数：</span><b id="rN"></b></div>
      <div><span>静态裕度：</span><b id="rMargin"></b> m（阈值 <span id="rThr"></span>）</div>
    </div>
    <div id="reason" class="ok"></div>
  </div>
  <p class="footnote">
    摆线轨迹 <code>s(u)=u−sin(2πu)/(2π)</code>、抬脚高度 <code>h(u)=(1−cos2πu)/2</code>，
    两端位置/速度/加速度连续，落脚垂直速度解析收敛到 0；稳定性判据为 McGhee &amp; Frank (1968)
    静态稳定裕度（重心投影到支撑凸包各边的最小有符号距离）。数据与逻辑全部内嵌，离线可用。
  </p>
</main>

<script type="application/json" id="payload">__PAYLOAD__</script>
<script>
"use strict";
const D = JSON.parse(document.getElementById("payload").textContent);
const LEGCOLOR = ["#5aa9ff","#8f7bff","#34d4c5","#ff8fa3"];
const LEGNAME  = ["FL","FR","BL","BR"];
const frames = D.frames;
const geo = D.geo;

// ---------- 画布适配 ----------
function setup(cv){
  const dpr = window.devicePixelRatio || 1;
  const cssW = cv.clientWidth, cssH = cv.clientHeight;
  cv.width = cssW*dpr; cv.height = cssH*dpr;
  const ctx = cv.getContext("2d");
  ctx.setTransform(dpr,0,0,dpr,0,0);
  return [ctx, cssW, cssH];
}
const topCv = document.getElementById("top"), sideCv = document.getElementById("side");
let topCtx, topW, topH, sideCtx, sideW, sideH;
function resizeAll(){
  [topCtx,topW,topH] = setup(topCv);
  [sideCtx,sideW,sideH] = setup(sideCv);
}
window.addEventListener("resize", ()=>{ resizeAll(); draw(idx); });

function grid(ctx,w,h,cx,cy,scale,axisY){
  ctx.save();
  ctx.strokeStyle="#1b2433"; ctx.lineWidth=1;
  ctx.fillStyle="#3a4757"; ctx.font="10px monospace";
  const step = 0.1*scale, ox = cx % step, oy = cy % step;
  ctx.beginPath();
  for(let x=ox;x<w;x+=step){ ctx.moveTo(x,0); ctx.lineTo(x,h); }
  for(let y=oy;y<h;y+=step){ ctx.moveTo(0,y); ctx.lineTo(w,y); }
  ctx.stroke();
  ctx.restore();
}

// ---------- 状态 ----------
let idx = 0, playing = true, speed = 1, lastT = performance.now();
// 支持 ?t=<秒> 或 ?frame=<序号> 直达某一帧（便于复核/分享不稳定帧）。
(function seekFromURL(){
  const q = new URLSearchParams(location.search);
  if (q.has("t")) {
    const t = parseFloat(q.get("t"));
    let k = 0, best = 1e9;
    frames.forEach((f,i)=>{ const dd=Math.abs(f.t-t); if(dd<best){best=dd;k=i;} });
    idx = k; playing = false;
  } else if (q.has("frame")) {
    idx = Math.max(0, Math.min(frames.length-1, parseInt(q.get("frame"),10)||0));
    playing = false;
  }
})();
const dangerIdx = frames.map((f,i)=>!f.stable ? i : -1).filter(i=>i>=0);

function draw(i){
  const f = frames[i], f0 = frames[0];
  // ----- 俯视 -----
  let ctx=topCtx,w=topW,h=topH;
  ctx.clearRect(0,0,w,h);
  const cx = w/2, cy = h/2, scT = Math.min(w,h)/1.05; // 米 -> 像素
  const TX = wx => cx + (wx - f.bx)*scT;
  const TY = wy => cy - wy*scT;
  grid(ctx,w,h,cx - f.bx*scT, cy, scT,true);
  // 历史轨迹
  for(let L=0;L<4;L++){
    ctx.strokeStyle = LEGCOLOR[L]; ctx.globalAlpha=.35; ctx.lineWidth=1.5;
    ctx.beginPath();
    for(let k=0;k<=i;k++){
      const p=frames[k].legs[L];
      const x=TX(p.x), y=TY(p.y);
      k===0?ctx.moveTo(x,y):ctx.lineTo(x,y);
    }
    ctx.stroke(); ctx.globalAlpha=1;
  }
  // 支撑多边形
  if(f.poly && f.poly.length>=3){
    ctx.beginPath();
    f.poly.forEach((p,k)=>{ const x=TX(p[0]),y=TY(p[1]); k?ctx.lineTo(x,y):ctx.moveTo(x,y); });
    ctx.closePath();
    ctx.fillStyle = f.stable ? "rgba(62,207,142,.12)" : "rgba(255,93,93,.12)";
    ctx.fill();
    ctx.strokeStyle = f.stable ? "#3ecf8e" : "#ff5d5d";
    ctx.setLineDash([5,4]); ctx.lineWidth=1.5; ctx.stroke(); ctx.setLineDash([]);
  }
  // 机身矩形 + 髋关节
  ctx.strokeStyle="#9fb0c3"; ctx.lineWidth=2;
  ctx.strokeRect(TX(f.bx-geo.a), TY(geo.b), 2*geo.a*scT, 2*geo.b*scT);
  // 重心投影
  ctx.fillStyle = f.stable ? "#3ecf8e" : "#ff5d5d";
  ctx.font="14px sans-serif"; ctx.textAlign="center";
  ctx.fillText("★", TX(f.cx), TY(f.cy)+5);
  // 四足
  for(let L=0;L<4;L++){
    const p=f.legs[L];
    const x=TX(p.x), y=TY(p.y);
    ctx.beginPath(); ctx.arc(x,y, 4+p.z*120, 0, 7);
    if(p.swing){ ctx.strokeStyle=LEGCOLOR[L]; ctx.lineWidth=2; ctx.stroke(); }
    else { ctx.fillStyle=LEGCOLOR[L]; ctx.fill(); }
    ctx.fillStyle=LEGCOLOR[L]; ctx.font="10px monospace";
    ctx.fillText(LEGNAME[L], x, y-9);
  }

  // ----- 侧视 -----
  ctx=sideCtx; w=sideW; h=sideH;
  ctx.clearRect(0,0,w,h);
  const scS = Math.min(w/1.1,h)/0.55;
  const groundY = h*0.82;
  const SX = wx => cx + (wx - f.bx)*scS;
  const SZ = z => groundY - z*scS;
  grid(ctx,w,h,cx-f.bx*scS,groundY,scS,false);
  // 地面
  ctx.strokeStyle="#3a4757"; ctx.beginPath();
  ctx.moveTo(0,groundY); ctx.lineTo(w,groundY); ctx.stroke();
  // 重心竖线
  ctx.strokeStyle = f.stable ? "rgba(62,207,142,.5)" : "rgba(255,93,93,.6)";
  ctx.setLineDash([4,4]);
  ctx.beginPath(); ctx.moveTo(SX(f.cx),groundY); ctx.lineTo(SX(f.cx),groundY-geo.bodyH*scS-24); ctx.stroke();
  ctx.setLineDash([]);
  // 机身
  ctx.strokeStyle="#9fb0c3"; ctx.lineWidth=2;
  ctx.beginPath();
  ctx.moveTo(SX(f.bx-geo.a), SZ(geo.bodyH)); ctx.lineTo(SX(f.bx+geo.a), SZ(geo.bodyH)); ctx.stroke();
  ctx.fillStyle="#9fb0c3"; ctx.font="10px monospace";
  ctx.fillText("body", SX(f.bx-geo.a), SZ(geo.bodyH)-6);
  // 轨迹 + 当前足（侧视图中左右腿按颜色区分；支撑足落在地面）
  for(let L=0;L<4;L++){
    ctx.strokeStyle=LEGCOLOR[L]; ctx.globalAlpha=.30; ctx.lineWidth=1.5;
    ctx.beginPath();
    for(let k=0;k<=i;k++){
      const p=frames[k].legs[L], x=SX(p.x), y=SZ(p.z);
      k===0?ctx.moveTo(x,y):ctx.lineTo(x,y);
    }
    ctx.stroke(); ctx.globalAlpha=1;
  }
  for(let L=0;L<4;L++){
    const p=f.legs[L];
    // 腿段：髋关节到足端（侧视按前/后两组髋部 x 连线）
    const hipXF = (L===0||L===1) ? f.bx+geo.a : f.bx-geo.a;
    ctx.strokeStyle=LEGCOLOR[L]; ctx.globalAlpha=.5; ctx.lineWidth=1;
    ctx.beginPath(); ctx.moveTo(SX(hipXF),SZ(geo.bodyH)); ctx.lineTo(SX(p.x),SZ(p.z)); ctx.stroke();
    ctx.globalAlpha=1;
    ctx.beginPath(); ctx.arc(SX(p.x),SZ(p.z), p.swing?4.5:3.5,0,7);
    if(p.swing){ ctx.strokeStyle=LEGCOLOR[L]; ctx.lineWidth=2; ctx.stroke(); }
    else { ctx.fillStyle=LEGCOLOR[L]; ctx.fill(); }
  }
  updateReadout(f,i);
}

function fmt(x,n=3){ return (x===null||x===undefined)? "—" : Number(x).toFixed(n); }
function updateReadout(f,i){
  document.getElementById("rT").textContent=fmt(f.t,2);
  document.getElementById("rGait").textContent=f.gait+(f.mode==="dwell"?"（驻留过渡）":"");
  document.getElementById("rPhase").textContent=fmt(f.phase,2);
  document.getElementById("rV").textContent=fmt(f.v,2);
  document.getElementById("rN").textContent=f.nSup;
  document.getElementById("rMargin").textContent = f.margin===null ? "不适用（<3脚支撑）" : fmt(f.margin);
  const r=document.getElementById("reason");
  if(f.stable){ r.className="ok"; r.textContent="静态稳定：重心投影位于支撑多边形内且裕度达标。"; }
  else { r.className="bad"; r.textContent="⚠ "+(f.reason||"静态不稳定"); }
  const bs=document.getElementById("bStable");
  bs.textContent = f.stable ? "当前帧稳定" : "当前帧不稳定";
  bs.className = "badge "+(f.stable?"ok":"bad");
  const bg=document.getElementById("bGait");
  bg.textContent = f.gait.toUpperCase()+" · "+(f.mode==="dwell"?"驻留":"运行");
  bg.className = "badge "+(f.switch?"bad":"ok");
  // 进度条
  const pct = i/(frames.length-1)*100;
  document.getElementById("scrubKnob").style.left = pct+"%";
}

// ---------- 不稳定区段条 ----------
(function buildDanger(){
  const el=document.getElementById("scrubDanger");
  // 把连续不稳定帧合并为区段
  let segs=[], start=null;
  frames.forEach((f,i)=>{
    if(!f.stable && start===null) start=i;
    if((f.stable||i===frames.length-1)&&start!==null){
      const e = f.stable? i-1 : i;
      segs.push([start,e]); start=null;
    }
  });
  el.style.background = "transparent";
  segs.forEach(([a,b])=>{
    const bar=document.createElement("div");
    bar.style.cssText="position:absolute;top:9px;height:4px;background:var(--bad);border-radius:2px;opacity:.85";
    bar.style.left=(a/(frames.length-1)*100)+"%";
    bar.style.width=((b-a+1)/(frames.length-1)*100)+"%";
    el.appendChild(bar);
  });
  frames.forEach((f,i)=>{
    if(f.switch){
      const m=document.createElement("div");
      m.style.cssText="position:absolute;top:2px;bottom:2px;width:2px;background:var(--warn)";
      m.style.left=(i/(frames.length-1)*100)+"%";
      m.title="步态切换 @ "+f.t.toFixed(2)+"s";
      document.getElementById("scrub").appendChild(m);
    }
  });
})();

// ---------- 控件 ----------
document.getElementById("scrub").addEventListener("click", e=>{
  const rect=e.currentTarget.getBoundingClientRect();
  idx = Math.round((e.clientX-rect.left)/rect.width*(frames.length-1));
  draw(idx);
});
const btnPlay=document.getElementById("btnPlay");
btnPlay.onclick=()=>{ playing=!playing; btnPlay.textContent=playing?"⏸ 暂停":"▶ 播放"; lastT=performance.now(); };
document.getElementById("speed").oninput=e=>{ speed=+e.target.value;
  document.getElementById("spdLab").textContent=speed.toFixed(1)+"×"; };
function jump(dir){
  let res=-1;
  if(dir>0){ for(const k of dangerIdx){ if(k>idx){res=k;break;} } if(res<0&&dangerIdx.length)res=dangerIdx[0]; }
  else { for(let j=dangerIdx.length-1;j>=0;j--){ if(dangerIdx[j]<idx){res=dangerIdx[j];break;} }
         if(res<0&&dangerIdx.length)res=dangerIdx[dangerIdx.length-1]; }
  if(res>=0){ idx=res; draw(idx); }
}
document.getElementById("btnNext").onclick=()=>jump(1);
document.getElementById("btnPrev").onclick=()=>jump(-1);
document.addEventListener("keydown",e=>{
  if(e.code==="Space"){ e.preventDefault(); btnPlay.click(); }
  if(e.code==="ArrowRight"){ idx=Math.min(frames.length-1,idx+1); draw(idx); }
  if(e.code==="ArrowLeft"){ idx=Math.max(0,idx-1); draw(idx); }
});

// ---------- 主循环 ----------
function tick(now){
  const dtReal=(now-lastT)/1000; lastT=now;
  if(playing){
    const dtSim = dtReal*speed;
    let t = frames[idx].t + dtSim;
    let k = idx;
    while(k<frames.length-1 && frames[k+1].t < t) k++;
    if(k>=frames.length-1){ playing=false; btnPlay.textContent="▶ 播放"; idx=frames.length-1; }
    else idx=k;
    draw(idx);
  }
  requestAnimationFrame(tick);
}

// ---------- 元信息 ----------
document.getElementById("mName").textContent=D.meta.name;
document.getElementById("mDur").textContent=fmt(D.meta.duration,1);
document.getElementById("mFrames").textContent=frames.length;
document.getElementById("mGeo").textContent=geo.a+" × "+geo.b;
document.getElementById("rThr").textContent=D.meta.marginThreshold+" m";

resizeAll();
if (!playing) document.getElementById("btnPlay").textContent = "▶ 播放";
draw(idx);
requestAnimationFrame(tick);
</script>
</body>
</html>
`

// ReportMeta 是报告头部展示的描述信息。
type ReportMeta struct {
	Name            string  `json:"name"`
	Duration        float64 `json:"duration"`
	MarginThreshold float64 `json:"marginThreshold"`
}

type reportPayload struct {
	Geo struct {
		A     float64 `json:"a"`
		B     float64 `json:"b"`
		BodyH float64 `json:"bodyH"`
		LiftH float64 `json:"liftH"`
	} `json:"geo"`
	Frames    []Frame          `json:"frames"`
	Touchdown []TouchdownEvent `json:"touchdowns"`
	Switches  []SwitchEvent    `json:"switches"`
	Meta      ReportMeta       `json:"meta"`
}

// RenderReport 把仿真结果渲染为单一离线 HTML 文件的字节内容。
func RenderPayload(geo Geometry, liftH float64, frames []Frame,
	touchdowns []TouchdownEvent, switches []SwitchEvent, meta ReportMeta) ([]byte, error) {
	var p reportPayload
	p.Geo.A = geo.HalfLength
	p.Geo.B = geo.HalfWidth
	p.Geo.BodyH = geo.BodyHeight
	p.Geo.LiftH = liftH
	p.Frames = frames
	p.Touchdown = touchdowns
	p.Switches = switches
	p.Meta = meta
	raw, err := json.Marshal(p)
	if err != nil {
		return nil, err
	}
	// 防止任意字段中出现 "</script>" 截断脚本块。
	safe := bytes.ReplaceAll(raw, []byte("</"), []byte(`<\/`))
	out := bytes.ReplaceAll([]byte(reportTemplate), []byte("__PAYLOAD__"), safe)
	return out, nil
}
