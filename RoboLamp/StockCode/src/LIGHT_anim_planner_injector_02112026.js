(() => {
  // =========================
  // RoboLamp Animation Planner (FULL) + Flat/Dome toggle
  // Paste into DevTools Console.
  // =========================

  // --- locate target accordion panel ---
  const matrixItem = document.querySelector('.accordion__item--matrix-face');
  if (!matrixItem) return console.warn("Couldn't find .accordion__item--matrix-face");
  const headingBtn = matrixItem.querySelector('[data-accordion-component="AccordionItemButton"]');
  const panel = matrixItem.querySelector('[data-accordion-component="AccordionItemPanel"]');
  if (!headingBtn || !panel) return console.warn("Missing accordion button/panel");

  headingBtn.textContent = "ANIMATION PLANNER";
  panel.style.overflow = "hidden";

  // --- styles ---
  const STYLE_ID = "robolamp-widget-style";
  if (!document.getElementById(STYLE_ID)) {
    const style = document.createElement("style");
    style.id = STYLE_ID;
    style.textContent = `
      :root{ --rlwLav:#b9a6ff; }
      .rlw{ font-family: ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Arial; color:#fff; overflow:hidden; }
      .rlw *{ box-sizing:border-box; }

      .rlw .section-title{
        font-weight: 900;
        font-size: 12px;
        letter-spacing: .10em;
        text-transform: uppercase;
        color: rgba(255,255,255,.88);
        margin: 0 0 10px 0;
      }

      .rlw details{
        border: 1px solid rgba(255,255,255,.12);
        border-radius: 10px;
        background: rgba(0,0,0,.18);
        margin: 10px 0;
        overflow: hidden;
      }
      .rlw summary{
        cursor:pointer;
        padding: 10px 12px;
        font-weight: 900;
        font-size: 12px;
        letter-spacing: .08em;
        text-transform: uppercase;
        list-style: none;
        display:flex;
        align-items:center;
        justify-content: space-between;
      }
      .rlw summary::-webkit-details-marker{ display:none; }
      .rlw .rlw-body{ padding: 10px 12px 12px; border-top: 1px solid rgba(255,255,255,.10); }

      .rlw .row{ display:flex; gap:10px; align-items:center; flex-wrap:wrap; }
      .rlw .mono{ font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", monospace; }
      .rlw .tiny{ font-size: 11px; color: rgba(255,255,255,.65); }

      .rlw button{
        border: 1px solid rgba(255,255,255,.14);
        background: rgba(0,0,0,.22);
        color:#fff;
        border-radius: 10px;
        padding: 8px 10px;
        font-weight: 900;
        font-size: 12px;
        letter-spacing: .02em;
        cursor:pointer;
        white-space: nowrap;
        flex: 0 0 auto;
      }
      .rlw button:hover{ background: rgba(255,255,255,.06); }

      .rlw input, .rlw select, .rlw textarea{
        border: 1px solid rgba(255,255,255,.14);
        background: rgba(0,0,0,.22);
        color:#fff;
        border-radius: 10px;
        padding: 8px 10px;
        font-size: 12px;
        outline:none;
      }
      .rlw textarea{ width:100%; min-height: 160px; resize: vertical; }

      .rlw input[type="checkbox"],
      .rlw input[type="range"]{ accent-color: var(--rlwLav); }

      /* Grid canvas: force square & responsive */
      #rlwGrid{
        width: 100%;
        aspect-ratio: 1 / 1;
        height: auto;
        border-radius: 12px;
        border: 1px solid rgba(255,255,255,.12);
        background: #0a0a0b;
        display:block;
      }

      .rlw .box{
        border: 1px solid rgba(255,255,255,.10);
        border-radius:10px;
        padding:10px;
        background: rgba(0,0,0,.10);
      }

      .rlw .kv{ display:grid; grid-template-columns: 110px 1fr; gap:8px; align-items:center; }

      .rlw .toast{
        margin-left:auto;
        font-size: 11px;
        color: rgba(191,231,191,.95);
      }

      /* scrubber row: PLAY STOP [scrubber expands] */
      .rlw .topControls{
        display:flex;
        gap:10px;
        align-items:center;
        width:100%;
        flex-wrap:nowrap;
        min-width: 0;
      }
      .rlw .scrubWrap{
        position: relative;
        flex: 1 1 auto;
        min-width: 260px;
        height: 32px;
        display:flex;
        align-items:center;
      }
      .rlw .scrubTicks{
        position:absolute;
        inset: 0;
        pointer-events: auto;
        cursor: pointer;
        border-radius: 10px;
      }
      .rlw .scrubSlider{
        position: relative;
        width: 100%;
      }

      .rlw .lbl{ display:flex; align-items:center; gap:8px; }

      /* height controls inline */
      .rlw .heightInline{
        display:flex;
        align-items:center;
        gap:10px;
        flex-wrap:wrap;
      }
      .rlw .heightInline input[type="number"]{ width: 120px; }

      /* mode pill */
      .rlw .modePill{
        border: 1px solid rgba(255,255,255,.14);
        background: rgba(0,0,0,.22);
        color:#fff;
        border-radius: 999px;
        padding: 6px 10px;
        font-weight: 900;
        font-size: 11px;
        letter-spacing: .02em;
        cursor:pointer;
        user-select:none;
      }
      .rlw .modePill[data-active="true"]{
        border-color: rgba(185,166,255,.65);
        box-shadow: 0 0 0 2px rgba(185,166,255,.12) inset;
      }
    `;
    document.head.appendChild(style);
  }

  // --- HTML ---
  panel.innerHTML = `
    <div class="rlw" id="robolamp-widget">
      <div class="row" style="justify-content:space-between; align-items:end; gap:12px;">
        <div class="section-title" style="margin:0;">VISUALIZER</div>
        <div class="row" style="gap:8px; flex-wrap:nowrap;">
          <span class="tiny" style="opacity:.8;">View</span>
          <div class="modePill" id="rlwModeFlat" data-active="true">FLAT</div>
          <div class="modePill" id="rlwModeDome" data-active="false">DOME</div>
        </div>
      </div>

      <canvas id="rlwGrid" width="560" height="560"></canvas>

      <div class="box" style="margin-top:10px;">
        <div class="row" style="justify-content:space-between; width:100%; align-items:center;">
          <div class="topControls">
            <button id="rlwPlay">PLAY</button>
            <button id="rlwStop">STOP</button>

            <div class="scrubWrap">
              <canvas class="scrubTicks" id="rlwTicks"></canvas>
              <input class="scrubSlider" id="rlwScrub" type="range" min="0" max="0" step="1" value="0" />
            </div>
          </div>

          <div class="tiny mono" id="rlwStatus">Ready</div>
        </div>

        <div class="row" style="margin-top:12px; justify-content:space-between; width:100%;">
          <div class="row">
            <button id="rlwRandom">RANDOM ANIMATION</button>
            <label class="tiny lbl"><input id="rlwRandHeight" type="checkbox" /> include height</label>
          </div>

          <div class="row">
            <label class="tiny lbl">
              Keyframes
              <input id="rlwCount" type="number" min="2" max="64" step="1" value="6" style="width:84px;" />
            </label>
            <label class="tiny lbl">
              Array name
              <input id="rlwName" type="text" value="animation1" style="width:150px;" />
            </label>
          </div>
        </div>

        <div class="row" style="margin-top:12px; justify-content:space-between;">
          <div class="row">
            <button id="rlwSave1">SAVE ANIMATION 1</button>
            <button id="rlwSave2">SAVE ANIMATION 2</button>
          </div>
          <div class="toast" id="rlwToast"></div>
        </div>

        <div class="tiny" style="margin-top:8px;">
          Double-click a keyframe dot to toggle Linear ↔ Bezier. Drag hollow handle dots when Bezier is enabled. Click tick marks to select a keyframe.
        </div>
      </div>

      <details style="margin-top:12px;">
        <summary>
          <span>KEYFRAME CONTROLS</span>
          <span class="tiny">Easing + Duration + Height + Export</span>
        </summary>
        <div class="rlw-body">
          <div class="box">
            <div class="row" style="justify-content:space-between; margin-bottom:10px;">
              <div class="tiny">Selected Keyframe</div>
              <div class="tiny mono" id="rlwSelInfo"></div>
            </div>

            <div class="kv">
              <div class="tiny">Duration</div>
              <div class="row" style="gap:10px;">
                <input id="rlwDurSlider" type="range" min="0" max="2000" step="10" value="300" style="flex:1;" />
                <input id="rlwDur" type="number" min="0" max="60000" step="10" style="width:110px;" />
              </div>

              <div class="tiny">Easing</div>
              <select id="rlwEase">
                <option value="0">Linear</option>
                <option value="1">Ease In</option>
                <option value="2">Ease Out</option>
                <option value="3">Ease In-Out</option>
              </select>

              <div class="tiny">Height</div>
              <div class="heightInline">
                <label class="tiny lbl" style="margin:0;">
                  <input id="rlwChangeHeight" type="checkbox" />
                  Change height?
                </label>
                <input id="rlwHeight" type="number" min="-32768" max="32767" step="1" value="500" />
                <span class="tiny">min = 130 | max = 770</span>
              </div>
            </div>
          </div>

          <div class="row" style="margin-top:12px;">
            <button id="rlwGen">GENERATE C++</button>
            <button id="rlwCopy">COPY</button>
            <span class="tiny mono" id="rlwLenVar"></span>
          </div>

          <textarea id="rlwOut" class="mono" spellcheck="false" style="margin-top:10px;" placeholder="Generated C++ will appear here..."></textarea>
        </div>
      </details>
    </div>
  `;

  // --- helpers ---
  const $ = (id) => document.getElementById(id);
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const lerp = (a,b,t) => a + (b-a)*t;
  const deg2rad = (d) => (d * Math.PI) / 180;

  // --- refs ---
  const grid = $("rlwGrid");
  const g = grid.getContext("2d");

  const ticksCanvas = $("rlwTicks");
  const tg = ticksCanvas.getContext("2d");

  const status = $("rlwStatus");
  const scrub = $("rlwScrub");
  const countEl = $("rlwCount");
  const nameEl = $("rlwName");

  const durEl = $("rlwDur");
  const durSliderEl = $("rlwDurSlider");
  const easeEl = $("rlwEase");
  const changeHeightEl = $("rlwChangeHeight");
  const heightEl = $("rlwHeight");

  const selInfoEl = $("rlwSelInfo");
  const outEl = $("rlwOut");
  const lenVarEl = $("rlwLenVar");

  const modeFlatEl = $("rlwModeFlat");
  const modeDomeEl = $("rlwModeDome");

  // --- mapping for FLAT mode ---
  const pad = 46;
  const degToPxX_flat = (yaw) => pad + (clamp(yaw,0,180)/180) * (grid.width - pad*2);
  const degToPxY_flat = (pitch) => pad + (clamp(pitch,0,180)/180) * (grid.height - pad*2);
  const pxToDegX_flat = (px) => clamp(((px - pad) / (grid.width - pad*2)) * 180, 0, 180);
  const pxToDegY_flat = (py) => clamp(((py - pad) / (grid.height - pad*2)) * 180, 0, 180);

  // --- hemisphere projection for DOME mode ---
  function domeParams() {
    const pad = 18;
    const R = Math.min(grid.width, grid.height) / 2 - pad;
    const cx = grid.width / 2;
    const cy = grid.height / 2;
    return { R, cx, cy };
  }

  function projectHemisphere(yawDeg, pitchDeg) {
    const az = deg2rad(yawDeg - 90);
    const el = deg2rad(pitchDeg - 90);

    const x = Math.cos(el) * Math.sin(az);
    const y = Math.sin(el);
    const z = Math.cos(el) * Math.cos(az);

    const { R, cx, cy } = domeParams();
    return { sx: cx + R * x, sy: cy - R * y, z, R, cx, cy };
  }

  function unprojectHemisphere(px, py) {
    const { R, cx, cy } = domeParams();
    const x0 = (px - cx) / R;
    const y0 = (cy - py) / R;

    let x = clamp(x0, -1, 1);
    let y = clamp(y0, -1, 1);

    const rr = x*x + y*y;
    if (rr > 1) {
      const s = 1 / Math.sqrt(rr);
      x *= s; y *= s;
    }

    const z = Math.sqrt(Math.max(0, 1 - (x*x + y*y)));
    const az = Math.atan2(x, z);
    const el = Math.asin(y);

    const yaw = clamp((az * 180 / Math.PI) + 90, 0, 180);
    const pitch = clamp((el * 180 / Math.PI) + 90, 0, 180);
    return { yaw, pitch };
  }

  // --- easing / bezier ---
  const applyEasing = (t, mode) => {
    t = clamp(t, 0, 1);
    switch (mode|0) {
      case 1: return t*t;
      case 2: { const u=1-t; return 1-u*u; }
      case 3: return (t<0.5) ? (2*t*t) : (1 - 2*(1-t)*(1-t));
      default: return t;
    }
  };

  const quad = (p0,p1,p2,t) => {
    const u = 1-t;
    return {
      x: u*u*p0.x + 2*u*t*p1.x + t*t*p2.x,
      y: u*u*p0.y + 2*u*t*p1.y + t*t*p2.y,
    };
  };

  // --- state ---
  let keyframes = [];
  let selected = 0;

  let playing = false;
  let playStartPerf = 0;
  let playStartT = 0;
  let currentT = 0;

  let dragging = null; // {kind:"kf"|"handle", index}
  let viewMode = "flat"; // "flat" | "dome"

  const totalDuration = () => keyframes.slice(1).reduce((s,k)=>s + (k.duration|0), 0);

  function sanitizeName(raw){
    const n = (raw||"").trim().replace(/[^A-Za-z0-9_]/g, "_");
    return n || "animation1";
  }
  function lenVar(name){ return `${name}Length`; }

  function seedHandle(i){
    if (i <= 0) return;
    const k = keyframes[i];
    const prev = keyframes[i-1];
    k.controlYaw = clamp(Math.round((prev.yaw + k.yaw)/2), 0, 180);
    k.controlPitch = clamp(Math.round((prev.pitch + k.pitch)/2) + 20, 0, 180);
  }

  function resetLayout(n=6){
    const N = clamp(n|0, 2, 64);
    keyframes = [];
    for (let i=0;i<N;i++){
      const t=i/(N-1);
      const yaw=Math.round(lerp(30,150,t));
      const pitch=Math.round(lerp(40,140,t));
      keyframes.push({
        yaw,pitch,
        duration: i===0 ? 0 : 300,
        easing: 0,
        height: -1,
        controlYaw: yaw,
        controlPitch: pitch,
        useBezier: false
      });
    }
    selected = 0;
    currentT = 0;

    changeHeightEl.checked = false;
    heightEl.disabled = true;

    syncUIFromSelected();
    syncScrub();
    draw();
  }

  function evalAt(tMs){
    const total = totalDuration();
    const t = clamp(tMs, 0, total);
    let acc = 0;
    for (let i=1;i<keyframes.length;i++){
      const a = keyframes[i-1];
      const b = keyframes[i];
      const dur = Math.max(0, b.duration|0);
      const segEnd = acc + dur;
      if (t <= segEnd || i === keyframes.length-1){
        const raw = dur===0 ? 1 : clamp((t-acc)/dur, 0, 1);
        const tt = applyEasing(raw, b.easing);
        const p0={x:a.yaw,y:a.pitch}, p2={x:b.yaw,y:b.pitch};
        if (!b.useBezier) return {x:lerp(p0.x,p2.x,tt), y:lerp(p0.y,p2.y,tt), seg:i};
        const p1={x:b.controlYaw,y:b.controlPitch};
        const p = quad(p0,p1,p2,tt);
        return {x:p.x,y:p.y,seg:i};
      }
      acc = segEnd;
    }
    const last = keyframes[keyframes.length-1];
    return {x:last.yaw,y:last.pitch,seg:keyframes.length-1};
  }

  // --- boundaries & ticks ---
  function getBoundaries(){
    const total = totalDuration();
    const boundaries = [0];
    let acc = 0;
    for (let i=1;i<keyframes.length;i++){
      acc += Math.max(0, keyframes[i].duration|0);
      boundaries.push(acc);
    }
    return { total, boundaries };
  }

  function resizeTicksCanvas(){
    const r = scrub.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    const w = Math.max(1, Math.floor(r.width * dpr));
    const h = Math.max(1, Math.floor(32 * dpr));
    if (ticksCanvas.width !== w) ticksCanvas.width = w;
    if (ticksCanvas.height !== h) ticksCanvas.height = h;
  }

  function drawTicks(){
    resizeTicksCanvas();
    const w = ticksCanvas.width, h = ticksCanvas.height;

    tg.clearRect(0,0,w,h);

    const { total, boundaries } = getBoundaries();
    if (total <= 0) return;

    tg.fillStyle = "rgba(0,0,0,0.15)";
    tg.fillRect(0,0,w,h);

    const top = Math.floor(h * 0.18);
    const bot = Math.floor(h * 0.92);
    const dpr = window.devicePixelRatio || 1;

    for (let i=0;i<boundaries.length;i++){
      const t = boundaries[i];
      const x = Math.round((t / total) * (w - 1));
      const isSel = (i === selected);

      tg.strokeStyle = isSel ? "rgba(185,166,255,1)" : "rgba(255,255,255,0.78)";
      tg.lineWidth = isSel ? Math.max(3, 2*dpr) : Math.max(2, 1.4*dpr);

      tg.beginPath();
      tg.moveTo(x, top);
      tg.lineTo(x, bot);
      tg.stroke();
    }
  }

  function selectKeyframe(i){
    const { boundaries } = getBoundaries();
    selected = clamp(i|0, 0, keyframes.length-1);
    currentT = boundaries[selected] ?? 0;
    syncUIFromSelected();
    syncScrub();
    draw();
  }

  ticksCanvas.addEventListener("click", (e) => {
    const { total, boundaries } = getBoundaries();
    if (total <= 0) return;

    const r = ticksCanvas.getBoundingClientRect();
    const xCss = clamp(e.clientX - r.left, 0, r.width);
    const tGuess = (xCss / r.width) * total;

    let bestI = 0, bestD = Infinity;
    for (let i=0;i<boundaries.length;i++){
      const d = Math.abs(boundaries[i] - tGuess);
      if (d < bestD){ bestD = d; bestI = i; }
    }
    selectKeyframe(bestI);
  });

  // --- view toggle ---
  function setViewMode(mode) {
    viewMode = mode;
    modeFlatEl.dataset.active = (mode === "flat") ? "true" : "false";
    modeDomeEl.dataset.active = (mode === "dome") ? "true" : "false";
    draw();
  }
  modeFlatEl.addEventListener("click", () => setViewMode("flat"));
  modeDomeEl.addEventListener("click", () => setViewMode("dome"));

  // --- drawing helpers (mode-dependent projection) ---
  function toScreen(yaw, pitch) {
    if (viewMode === "dome") {
      const p = projectHemisphere(yaw, pitch);
      return { x: p.sx, y: p.sy, z: p.z };
    }
    return { x: degToPxX_flat(yaw), y: degToPxY_flat(pitch), z: 1 };
  }

  function fromScreen(px, py) {
    if (viewMode === "dome") {
      return unprojectHemisphere(px, py);
    }
    return { yaw: pxToDegX_flat(px), pitch: pxToDegY_flat(py) };
  }

  // --- FLAT background ---
  function drawDotGridFlat(){
    g.clearRect(0,0,grid.width,grid.height);
    g.fillStyle="#0a0a0b";
    g.fillRect(0,0,grid.width,grid.height);

    const dots = 19;
    const w = grid.width - pad*2;
    const h = grid.height - pad*2;
    g.fillStyle="rgba(232,232,232,0.24)";
    for (let iy=0;iy<dots;iy++){
      for (let ix=0;ix<dots;ix++){
        const x = pad + (ix/(dots-1))*w;
        const y = pad + (iy/(dots-1))*h;
        g.beginPath(); g.arc(x,y,1.25,0,Math.PI*2); g.fill();
      }
    }
    g.strokeStyle="rgba(232,232,232,0.45)";
    g.lineWidth=1;
    g.strokeRect(pad,pad,w,h);
  }

  // --- DOME background ---
  function drawHemisphereGrid(){
    g.clearRect(0,0,grid.width,grid.height);
    g.fillStyle="#0a0a0b";
    g.fillRect(0,0,grid.width,grid.height);

    const { R, cx, cy } = domeParams();

    const grad = g.createRadialGradient(cx - R*0.25, cy - R*0.25, R*0.15, cx, cy, R);
    grad.addColorStop(0, "rgba(255,255,255,0.06)");
    grad.addColorStop(1, "rgba(255,255,255,0)");
    g.fillStyle = grad;
    g.beginPath();
    g.arc(cx, cy, R, 0, Math.PI*2);
    g.fill();

    g.strokeStyle = "rgba(232,232,232,0.55)";
    g.lineWidth = 1.2;
    g.beginPath();
    g.arc(cx, cy, R, 0, Math.PI*2);
    g.stroke();

    const latCount = 6;
    for (let i = 1; i < latCount; i++) {
      const t = i / latCount;
      const yy = lerp(-R, R, t);
      const rr = Math.sqrt(Math.max(0, R*R - yy*yy));
      g.strokeStyle = "rgba(232,232,232,0.18)";
      g.lineWidth = 1;
      g.beginPath();
      g.ellipse(cx, cy + yy, rr, rr * 0.35, 0, 0, Math.PI*2);
      g.stroke();
    }

    const lonCount = 8;
    for (let i = 0; i < lonCount; i++) {
      const a = (i / lonCount) * Math.PI;
      const rx = Math.sin(a) * R;
      const ry = R;
      g.strokeStyle = "rgba(232,232,232,0.16)";
      g.lineWidth = 1;
      g.beginPath();
      g.ellipse(cx, cy, Math.max(0.01, rx), ry, 0, 0, Math.PI*2);
      g.stroke();
    }

    g.strokeStyle = "rgba(185,166,255,0.25)";
    g.lineWidth = 1.6;
    g.beginPath();
    g.ellipse(cx, cy, R, R * 0.35, 0, 0, Math.PI*2);
    g.stroke();
  }

  // --- draw paths/points (works for both modes) ---
  function drawPaths(){
    for (let i=1;i<keyframes.length;i++){
      const a=keyframes[i-1], b=keyframes[i];

      g.strokeStyle = (i===selected) ? "rgba(242,242,242,0.88)" : "rgba(242,242,242,0.33)";
      g.lineWidth = (i===selected) ? 1.6 : 1.0;

      g.beginPath();

      const samples = 28;
      for (let s=0;s<=samples;s++){
        const t=s/samples;

        let yaw, pitch;
        if (!b.useBezier){
          yaw = lerp(a.yaw, b.yaw, t);
          pitch = lerp(a.pitch, b.pitch, t);
        } else {
          const u = 1 - t;
          yaw = u*u*a.yaw + 2*u*t*b.controlYaw + t*t*b.yaw;
          pitch = u*u*a.pitch + 2*u*t*b.controlPitch + t*t*b.pitch;
        }

        const p = toScreen(yaw, pitch);
        if (s===0) g.moveTo(p.x, p.y);
        else g.lineTo(p.x, p.y);
      }
      g.stroke();

      if (b.useBezier){
        const end = toScreen(b.yaw, b.pitch);
        const h = toScreen(b.controlYaw, b.controlPitch);

        g.strokeStyle = (i===selected) ? "rgba(242,242,242,0.50)" : "rgba(242,242,242,0.18)";
        g.lineWidth = 1;
        g.beginPath();
        g.moveTo(end.x, end.y);
        g.lineTo(h.x, h.y);
        g.stroke();

        g.strokeStyle = (i===selected) ? "rgba(185,166,255,0.95)" : "rgba(242,242,242,0.75)";
        g.lineWidth = 2.0;
        g.beginPath();
        g.arc(h.x, h.y, 7.2, 0, Math.PI*2);
        g.stroke();
      }
    }
  }

  function drawPoints(){
    for (let i=0;i<keyframes.length;i++){
      const k=keyframes[i];
      const p = toScreen(k.yaw, k.pitch);
      const isSel = (i===selected);

      const alpha = (viewMode === "dome" && p.z < 0) ? 0.28 : 1.0;

      const r=(isSel)?9.5:7.5;
      g.fillStyle = isSel ? `rgba(242,242,242,${1.0*alpha})` : `rgba(242,242,242,${0.75*alpha})`;
      g.beginPath(); g.arc(p.x,p.y,r,0,Math.PI*2); g.fill();

      g.strokeStyle = isSel ? `rgba(185,166,255,${0.95*alpha})` : `rgba(242,242,242,${0.30*alpha})`;
      g.lineWidth = isSel ? 2 : 1;
      g.stroke();

      g.fillStyle=`rgba(10,10,11,${0.9*alpha})`;
      g.font="12px ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, Liberation Mono, monospace";
      g.textAlign="center"; g.textBaseline="middle";
      g.fillText(String(i), p.x, p.y);

      g.textAlign="start"; g.textBaseline="alphabetic";
    }
  }

  function drawMarker(){
    const ev=evalAt(currentT);
    const p = toScreen(ev.x, ev.y);
    const alpha = (viewMode === "dome" && p.z < 0) ? 0.28 : 1.0;

    g.fillStyle=`rgba(185,166,255,${0.95*alpha})`;
    g.beginPath(); g.arc(p.x,p.y,5.2,0,Math.PI*2); g.fill();
  }

  function draw(){
    if (viewMode === "dome") drawHemisphereGrid();
    else drawDotGridFlat();

    drawPaths();
    drawPoints();
    drawMarker();

    const k = keyframes[selected];
    const mode = (selected===0) ? "START" : (k.useBezier ? "BEZIER" : "LINEAR");
    selInfoEl.textContent = `#${selected} • ${mode}`;

    drawTicks();
  }

  // --- UI sync ---
  function syncScrub(){
    const total = totalDuration();
    scrub.max = String(total);
    scrub.value = String(clamp(currentT,0,total));
    scrub.step = "1";
    drawTicks();
  }

  function syncUIFromSelected(){
    const k = keyframes[selected];

    durEl.value = (selected===0) ? 0 : (k.duration|0);
    durSliderEl.value = String(clamp((k.duration|0),0,2000));
    easeEl.value = String(k.easing|0);

    const disabled = selected===0;
    durEl.disabled = disabled;
    durSliderEl.disabled = disabled;
    easeEl.disabled = disabled;

    if (disabled){
      changeHeightEl.disabled = true;
      changeHeightEl.checked = false;
      heightEl.disabled = true;
      k.height = -1;
    } else {
      changeHeightEl.disabled = false;
      changeHeightEl.checked = (k.height|0) !== -1;
      heightEl.disabled = !changeHeightEl.checked;
      if (changeHeightEl.checked) heightEl.value = String(k.height|0);
    }
  }

  function syncSelectedFromUI(){
    const k = keyframes[selected];
    if (selected===0){
      k.duration = 0;
      k.useBezier = false;
      k.controlYaw = k.yaw;
      k.controlPitch = k.pitch;
      k.height = -1;
      syncUIFromSelected();
      syncScrub();
      draw();
      return;
    }

    k.duration = clamp(parseInt(durEl.value||"0",10)||0, 0, 60000);
    k.easing = clamp(parseInt(easeEl.value||"0",10)||0, 0, 255);

    if (!changeHeightEl.checked) k.height = -1;
    else k.height = clamp(parseInt(heightEl.value||"0",10)||0, -32768, 32767);

    durSliderEl.value = String(clamp(k.duration|0, 0, 2000));
    heightEl.disabled = !changeHeightEl.checked;

    syncScrub();
    draw();
  }

  // --- hit testing (mode-aware) ---
  function hitTest(px, py){
    for (let i=keyframes.length-1;i>=0;i--){
      const k=keyframes[i];
      const p = toScreen(k.yaw, k.pitch);
      const d=(px-p.x)*(px-p.x)+(py-p.y)*(py-p.y);
      if (d <= 12*12) return {kind:"kf", index:i};
    }
    for (let i=keyframes.length-1;i>=1;i--){
      const k=keyframes[i];
      if (!k.useBezier) continue;
      const p = toScreen(k.controlYaw, k.controlPitch);
      const d=(px-p.x)*(px-p.x)+(py-p.y)*(py-p.y);
      if (d <= 12*12) return {kind:"handle", index:i};
    }
    return null;
  }

  // --- pointer interactions ---
  grid.addEventListener("pointerdown", (e) => {
    const r = grid.getBoundingClientRect();
    const px = (e.clientX - r.left) * (grid.width / r.width);
    const py = (e.clientY - r.top) * (grid.height / r.height);
    const hit = hitTest(px, py);
    if (hit){
      dragging = hit;
      selectKeyframe(hit.index);
      status.textContent = hit.kind === "kf" ? `Dragging keyframe #${selected}` : `Dragging handle for #${selected}`;
    }
  });

  window.addEventListener("pointermove", (e) => {
    if (!dragging) return;
    const r = grid.getBoundingClientRect();
    const px = (e.clientX - r.left) * (grid.width / r.width);
    const py = (e.clientY - r.top) * (grid.height / r.height);

    const { yaw, pitch } = fromScreen(px, py);

    if (dragging.kind === "kf"){
      const k = keyframes[dragging.index];
      k.yaw = yaw; k.pitch = pitch;
      if (!k.useBezier){ k.controlYaw = yaw; k.controlPitch = pitch; }
    } else {
      const k = keyframes[dragging.index];
      k.controlYaw = yaw; k.controlPitch = pitch;
    }

    syncUIFromSelected();
    draw();
  });

  window.addEventListener("pointerup", () => {
    if (dragging) status.textContent = "Ready";
    dragging = null;
  });

  grid.addEventListener("dblclick", (e) => {
    const r = grid.getBoundingClientRect();
    const px = (e.clientX - r.left) * (grid.width / r.width);
    const py = (e.clientY - r.top) * (grid.height / r.height);
    const hit = hitTest(px, py);
    if (!hit || hit.kind !== "kf") return;
    const i = hit.index;
    if (i === 0) return;

    const k = keyframes[i];
    k.useBezier = !k.useBezier;
    if (k.useBezier){
      if (Math.round(k.controlYaw) === Math.round(k.yaw) && Math.round(k.controlPitch) === Math.round(k.pitch)) seedHandle(i);
    } else {
      k.controlYaw = k.yaw; k.controlPitch = k.pitch;
    }
    selectKeyframe(i);
  });

  // --- UI bindings ---
  [durEl, easeEl, heightEl].forEach(el => el.addEventListener("input", syncSelectedFromUI));
  durSliderEl.addEventListener("input", () => {
    if (selected===0) return;
    durEl.value = durSliderEl.value;
    syncSelectedFromUI();
  });
  changeHeightEl.addEventListener("change", () => {
    if (selected===0) return;
    syncSelectedFromUI();
  });

  countEl.addEventListener("change", () => resetLayout(parseInt(countEl.value||"6",10)||6));

  // --- playback ---
  $("rlwPlay").addEventListener("click", () => {
    const total = totalDuration();
    if (total<=0) return;
    playing = !playing;
    $("rlwPlay").textContent = playing ? "PAUSE" : "PLAY";
    if (playing){
      playStartPerf = performance.now();
      playStartT = currentT;
    }
  });

  $("rlwStop").addEventListener("click", () => {
    playing=false;
    $("rlwPlay").textContent="PLAY";
    currentT=0;
    syncScrub();
    draw();
  });

  scrub.addEventListener("input", () => {
    playing=false;
    $("rlwPlay").textContent="PLAY";
    currentT = clamp(parseInt(scrub.value||"0",10)||0, 0, totalDuration());
    draw();
  });

  function loop(){
    if (playing){
      const total = totalDuration();
      const dt = performance.now() - playStartPerf;
      currentT = clamp(playStartT + dt, 0, total);
      scrub.value = String(Math.round(currentT));
      if (currentT >= total){
        playing=false;
        $("rlwPlay").textContent="PLAY";
      }
      draw();
    }
    requestAnimationFrame(loop);
  }

  // --- save slots (ONLY 1 + 2) ---
  function toast(msg){
    const t = $("rlwToast");
    t.textContent = msg;
    setTimeout(()=>{ if (t.textContent===msg) t.textContent=""; }, 1200);
  }
  function saveSlot(n){
    const key = `robolamp_anim_slot_${n}`;
    localStorage.setItem(key, JSON.stringify({ keyframes }));
    toast(`Saved ${n} ✓`);
  }
  $("rlwSave1").addEventListener("click", ()=>saveSlot(1));
  $("rlwSave2").addEventListener("click", ()=>saveSlot(2));

  // --- random (with confirmation) ---
  $("rlwRandom").addEventListener("click", () => {
    const ok = confirm("Generate a random animation? This will overwrite the current keyframes.");
    if (!ok) return;

    const N = clamp(parseInt(countEl.value||"6",10)||6, 2, 64);
    const includeH = $("rlwRandHeight").checked;

    keyframes = [{
      yaw:90, pitch:90, duration:0, easing:0, height:-1,
      controlYaw:90, controlPitch:90, useBezier:false
    }];

    const rint=(a,b)=>Math.floor(a+Math.random()*(b-a+1));
    const chance=(p)=>Math.random()<p;

    for (let i=1;i<N;i++){
      const yaw=rint(15,165);
      const pitch=rint(15,165);
      const duration=rint(120,1200);
      const easing=rint(0,3);
      const useBezier=chance(0.55);
      const prev=keyframes[i-1];
      let cy=yaw, cp=pitch;
      if (useBezier){
        cy = clamp(Math.round((prev.yaw+yaw)/2 + rint(-50,50)),0,180);
        cp = clamp(Math.round((prev.pitch+pitch)/2 + rint(-50,50)),0,180);
      }
      const height = includeH ? rint(200,900) : -1;
      keyframes.push({yaw,pitch,duration,easing,height,controlYaw:cy,controlPitch:cp,useBezier});
    }

    selected=0; currentT=0;
    changeHeightEl.checked = false;
    heightEl.disabled = true;

    syncUIFromSelected();
    syncScrub();
    draw();
    toast("Randomized ✓");
  });

  // --- export ---
  function formatKf(k, isFirst){
    const yaw = clamp(Math.round(k.yaw),0,180);
    const pitch = clamp(Math.round(k.pitch),0,180);
    const duration = isFirst ? 0 : clamp(k.duration|0,0,60000);
    const easing = clamp(k.easing|0,0,255);
    const height = clamp(k.height|0, -32768, 32767);

    const useBezier = (!isFirst) && !!k.useBezier;
    const cy = useBezier ? clamp(Math.round(k.controlYaw),0,180) : yaw;
    const cp = useBezier ? clamp(Math.round(k.controlPitch),0,180) : pitch;

    return `  { ${yaw}, ${pitch}, ${duration}, ${easing}, ${height}, ${cy}, ${cp}, ${useBezier ? "true":"false"} }`;
  }

  function generateCpp(){
    const name = sanitizeName(nameEl.value);
    const len = lenVar(name);
    lenVarEl.textContent = `(+ ${len})`;

    const lines = [];
    lines.push(`// Generated by RoboLamp Animation Planner (prototype)`);
    lines.push(`// Struct order: yaw, pitch, duration, easing, height, controlYaw, controlPitch, useBezier`);
    lines.push(`const Keyframe ${name}[] PROGMEM = {`);
    for (let i=0;i<keyframes.length;i++){
      lines.push(formatKf(keyframes[i], i===0) + (i===keyframes.length-1 ? "" : ","));
    }
    lines.push(`};`);
    lines.push(`const uint8_t ${len} = sizeof(${name}) / sizeof(${name}[0]);`);
    return lines.join("\n");
  }

  $("rlwGen").addEventListener("click", () => { outEl.value = generateCpp(); toast("Generated ✓"); });
  $("rlwCopy").addEventListener("click", async () => {
    try { await navigator.clipboard.writeText(outEl.value||""); toast("Copied ✓"); }
    catch { toast("Copy failed"); }
  });

  nameEl.addEventListener("input", () => {
    lenVarEl.textContent = `(+ ${lenVar(sanitizeName(nameEl.value))})`;
  });

  // keep ticks aligned on resize
  const ro = new ResizeObserver(() => {
    drawTicks();
    draw();
  });
  ro.observe(panel);

  // init
  resetLayout(parseInt(countEl.value||"6",10)||6);
  lenVarEl.textContent = `(+ ${lenVar(sanitizeName(nameEl.value))})`;
  loop();

  console.log("Animation Planner FULL + DOME view (2 save slots)");
})();