const char htmlPageUI[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover" />
  <title>Nixie Clock Control Panel</title>
  <style>
    :root {
      --bg:#140a26;
      --fg:#f4eaff;
      --accent:#e9d5ff;
      --btn:#5b21b6;
      --btnHover:#6d28d9;
      --btnText:#f3e8ff;
      --activeGradient:linear-gradient(135deg,#c084fc,#a855f7);
      --activeBorder:#f5d0fe;
      --muted:#c4b5fd;
      --radius:14px;
      --gap:12px;
      --shadow:0 6px 16px rgba(0,0,0,.35);
    }
    * {
      box-sizing:border-box;
    }
    body {
      margin:0;
      background: var(--bg);
      color: var(--fg);
      font-family: system-ui, -apple-system, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;
    }
    .container {
      max-width: 840px;
      margin: 0 auto;
      padding: clamp(16px, 4vw, 32px);
    }
    header {
      position: sticky;
      top: 0;
      background: rgba(20,10,38,.92);
      backdrop-filter: blur(6px);
      padding: 14px 8px;
      border-bottom: 1px solid rgba(255,255,255,.1);
      z-index: 10;
    }
    h1 {
      margin:0;
      font-size: clamp(20px, 3.8vw, 28px);
      color: var(--accent);
      text-align:center;
    }
    .btn {
      appearance:none;
      border:none;
      border-radius: var(--radius);
      padding: 14px 16px;
      width: 100%;
      font-size: clamp(15px, 3.6vw, 16px);
      font-weight:600;
      background: var(--btn);
      color: var(--btnText);
      box-shadow: var(--shadow);
      cursor:pointer;
      transition: all .2s ease;
    }
    .btn:hover { background: var(--btnHover); }
    .btn.active {
      background: var(--activeGradient);
      border: 2px solid var(--activeBorder);
      color: #fff;
      transform: scale(1.02);
      box-shadow: 0 0 12px rgba(192,132,252,0.6), var(--shadow);
    }
    .card {
      background: rgba(255,255,255,.05);
      border: 1px solid rgba(255,255,255,.1);
      border-radius: calc(var(--radius) + 4px);
      padding: clamp(14px, 3.2vw, 22px);
      margin-top: clamp(12px, 2.4vw, 20px);
      box-shadow: var(--shadow);
    }
    .section-title {
      margin:0 0 8px 0;
      color: var(--accent);
      font-size: clamp(16px, 3.6vw, 18px);
      text-align:center;
    }
    .grid {
      display:grid;
      gap: var(--gap);
      grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
    }

    /* Sliders */
    .slider-label {
      display:flex;
      align-items:center;
      justify-content:space-between;
      gap:12px;
      font-size:clamp(14px,3.2vw,16px);
    }
    .slider-wrap { margin-top:10px; }
    input[type="range"] {
      width:100%;
      height:36px;
      -webkit-appearance:none;
      background:transparent;
    }
    input[type="range"]::-webkit-slider-runnable-track {
      height:6px;
      background: var(--muted);
      border-radius:3px;
    }
    input[type="range"]::-webkit-slider-thumb {
      -webkit-appearance:none;
      width:30px;
      height:30px;
      border-radius:50%;
      background: var(--activeGradient);
      border:2px solid var(--activeBorder);
      margin-top:-12px;
      box-shadow: var(--shadow);
      cursor:pointer;
    }
    input[type="range"]::-moz-range-track {
      height:6px;
      background: var(--muted);
      border-radius:3px;
    }
    input[type="range"]::-moz-range-thumb {
      width:30px;
      height:30px;
      border-radius:50%;
      background: var(--activeGradient);
      border:2px solid var(--activeBorder);
      box-shadow: var(--shadow);
      cursor:pointer;
    }

    #colorIcon,#ugColorIcon {
      width:48px;
      height:48px;
      border-radius:50%;
      border:2px solid var(--muted);
      flex:0 0 48px;
      background:#ffffff;
      box-shadow: inset 0 0 0 1px rgba(0,0,0,.2);
    }

    /* Time / Alarm / Timer inputs */
    .time-row {
      display:flex;
      flex-wrap:wrap;
      align-items:center;
      gap:8px;
      margin-top:8px;
    }
    .time-input,
    .time-select {
      padding:6px 10px;
      border-radius:var(--radius);
      border:1px solid var(--muted);
      background:rgba(0,0,0,.25);
      color:var(--fg);
      font-size:15px;
    }
    .time-input {
      width:8.2ch;               /* slightly wider so text isn't hidden by arrows */
      text-align:center;
    }
    .time-select {
      min-width:72px;
    }
    .inline-btn {
      flex:1;
      min-width:120px;
    }

    /* Timezone select */
    #tzSelect {
      flex:2;
      min-width:220px;
    }

    @media (max-width:480px){
      .time-row {
        justify-content:flex-start;
      }
    }
  </style>

<script>
const modes = ["RAINBOW","SOLID","GRADIENT","FLOW","WIPE","PULSE","BOUNCE"];
let sliderTimer;

function debounce(fn,delay){
  return (...args)=>{
    clearTimeout(sliderTimer);
    sliderTimer = setTimeout(()=>fn(...args), delay);
  };
}

function handleToggle(endpoint,el){
  el.disabled = true;
  fetch(endpoint).finally(()=>setTimeout(()=>{el.disabled=false;},300));
}

/* ---------- MAIN COLOR POS ---------- */
const updateSlider = debounce(val=>{
  document.getElementById("sliderValue").textContent = val;
  fetch("/setColorPos?val="+val).then(fetchColor);
},120);

function fetchColor(){
  fetch("/getColor")
    .then(r=>r.json())
    .then(c=>{
      document.getElementById("colorIcon").style.backgroundColor = `rgb(${c.r},${c.g},${c.b})`;
    })
    .catch(()=>{});
}

/* ---------- COLOR MODE ---------- */
function setColorState(mode){
  fetch("/setColorState?mode="+mode)
    .then(()=>highlightMode(mode))
    .catch(()=>{});
}
function highlightMode(active){
  modes.forEach(m=>{
    const btn = document.getElementById("btn_"+m);
    if(btn) btn.classList.toggle("active", m===active);
  });
}
function fetchColorState(){
  fetch("/getColorState")
    .then(r=>r.json())
    .then(data=>{
      if(data && data.mode){
        highlightMode(data.mode);
      }
    })
    .catch(()=>{});
}

/* ---------- PANEL BRIGHTNESS ---------- */
function updateBrightness(val){
  let pct = parseInt(val,10);
  if(isNaN(pct)) pct = 0;
  document.getElementById("brightnessValue").textContent = pct;
  fetch("/setBrightness?val="+pct).catch(()=>{});
}
function fetchBrightness(){
  fetch("/getBrightness")
    .then(r=>r.json())
    .then(data=>{
      const pct = (data && typeof data.pct==="number") ? data.pct : 100;
      document.getElementById("brightnessSlider").value = pct;
      document.getElementById("brightnessValue").textContent = pct;
    })
    .catch(()=>{});
}

/* ---------- UNDERGLOW COLOR POS ---------- */
const updateUGSlider = debounce(val=>{
  document.getElementById("ugSliderValue").textContent = val;
  fetch("/setUGColorPos?val="+val).then(fetchUGColor);
},120);

function fetchUGColor(){
  fetch("/getUGColor")
    .then(r=>r.json())
    .then(c=>{
      document.getElementById("ugColorIcon").style.backgroundColor = `rgb(${c.r},${c.g},${c.b})`;
    })
    .catch(()=>{});
}

/* ---------- UNDERGLOW BRIGHTNESS ---------- */
function updateUGBrightness(val){
  let pct = parseInt(val,10);
  if(isNaN(pct)) pct = 0;
  document.getElementById("ugBrightnessValue").textContent = pct;
  fetch("/setUGBrightness?val="+pct).catch(()=>{});
}
function fetchUGBrightness(){
  fetch("/getUGBrightness")
    .then(r=>r.json())
    .then(data=>{
      const pct = (data && typeof data.pct==="number") ? data.pct : 100;
      document.getElementById("ugBrightnessSlider").value = pct;
      document.getElementById("ugBrightnessValue").textContent = pct;
    })
    .catch(()=>{});
}

/* ---------- ALARM ---------- */
function submitAlarm(){
  const hEl = document.getElementById("alarmHour");
  const mEl = document.getElementById("alarmMin");
  const apEl= document.getElementById("alarmPeriod");

  let h = hEl.value ? parseInt(hEl.value,10) : parseInt(hEl.placeholder,10);
  let m = mEl.value ? parseInt(mEl.value,10) : parseInt(mEl.placeholder,10);
  const ap = apEl.value || "AM";

  if(isNaN(h)) h = 7;
  if(isNaN(m)) m = 0;

  if(h<1 || h>12 || m<0 || m>59){
    alert("Enter 1–12 for hour, 0–59 for minutes.");
    return;
  }

  fetch(`/setAlarm?hour=${h}&min=${m}&ampm=${encodeURIComponent(ap)}`)
    .catch(()=>{});
}
function clearAlarm(){
  fetch("/clearAlarm").catch(()=>{});
}

/* ---------- TIMER ---------- */
function submitTimer(){
  const hEl = document.getElementById("timerHours");
  const mEl = document.getElementById("timerMinutes");
  const sEl = document.getElementById("timerSeconds");

  let h = hEl.value ? parseInt(hEl.value,10) : 0;
  let m = mEl.value ? parseInt(mEl.value,10) : 5;
  let s = sEl.value ? parseInt(sEl.value,10) : 0;

  if(isNaN(h)) h = 0;
  if(isNaN(m)) m = 5;
  if(isNaN(s)) s = 0;

  if(h===0 && m===0 && s===0){
    alert("Timer must be at least 1 second.");
    return;
  }

  fetch(`/setTimer?h=${h}&m=${m}&s=${s}`).catch(()=>{});
}
function clearTimer(){
  fetch("/stopTimer").catch(()=>{});
}

/* ---------- TIMEZONE (auto-apply on change) ---------- */
function updateTimeZone(){
  const zone = document.getElementById("tzSelect").value;
  fetch("/setTZ?zone="+encodeURIComponent(zone))
    .then(r=>r.text())
    .then(txt=>console.log("TZ:",txt))
    .catch(()=>{});
}

/* ---------- ON LOAD ---------- */
window.addEventListener("load",()=>{
  fetchColor();
  fetchUGColor();
  fetchBrightness();
  fetchUGBrightness();
  fetchColorState();  // sync button highlight with actual mode (col_state / bootMode)
});
</script>
</head>

<body>
<header><h1>Nixie Clock Control Panel</h1></header>
<main class="container">

<!-- QUICK SETTINGS -->
<section class="card">
  <h2 class="section-title">Quick Settings</h2>
  <div class="grid">
    <button class="btn" onclick="handleToggle('/toggle24hr',this)">Toggle 24-Hour Mode</button>
    <button class="btn" onclick="handleToggle('/toggleLeadingZero',this)">Toggle Leading Zero</button>
  </div>
</section>

<!-- PANEL BRIGHTNESS -->
<section class="card">
  <h2 class="section-title">Panel Brightness</h2>
  <div class="slider-label">
    <span>Level: <strong id="brightnessValue">100</strong>%</span>
  </div>
  <div class="slider-wrap">
    <input id="brightnessSlider" type="range" min="0" max="100" value="100"
           oninput="updateBrightness(this.value)">
  </div>
</section>

<!-- PANEL COLOR -->
<section class="card">
  <h2 class="section-title">Panel Color</h2>
  <div class="slider-label">
    <span>Value: <strong id="sliderValue">200</strong></span>
    <div id="colorIcon"></div>
  </div>
  <div class="slider-wrap">
    <input type="range" min="0" max="255" value="200"
           oninput="updateSlider(this.value)">
  </div>
</section>

<!-- UNDERGLOW COLOR -->
<section class="card">
  <h2 class="section-title">UnderGlow Color</h2>
  <div class="slider-label">
    <span>Value: <strong id="ugSliderValue">200</strong></span>
    <div id="ugColorIcon"></div>
  </div>
  <div class="slider-wrap">
    <input type="range" min="0" max="255" value="200"
           oninput="updateUGSlider(this.value)">
  </div>
</section>

<!-- UNDERGLOW BRIGHTNESS -->
<section class="card">
  <h2 class="section-title">UnderGlow Brightness</h2>
  <div class="slider-label">
    <span>Level: <strong id="ugBrightnessValue">100</strong>%</span>
  </div>
  <div class="slider-wrap">
    <input id="ugBrightnessSlider" type="range" min="0" max="100" value="100"
           oninput="updateUGBrightness(this.value)">
  </div>
</section>

<!-- COLOR MODES -->
<section class="card">
  <h2 class="section-title">Select Color Mode</h2>
  <div class="grid">
    <button id="btn_RAINBOW"  class="btn" onclick="setColorState('RAINBOW')">Rainbow</button>
    <button id="btn_SOLID"    class="btn" onclick="setColorState('SOLID')">Solid</button>
    <button id="btn_GRADIENT" class="btn" onclick="setColorState('GRADIENT')">Gradient</button>
    <button id="btn_FLOW"     class="btn" onclick="setColorState('FLOW')">Flow</button>
    <button id="btn_WIPE"     class="btn" onclick="setColorState('WIPE')">Wipe</button>
    <button id="btn_PULSE"    class="btn" onclick="setColorState('PULSE')">Pulse</button>
    <button id="btn_BOUNCE"   class="btn" onclick="setColorState('BOUNCE')">Bounce</button>
  </div>
</section>

<!-- ALARM -->
<section class="card">
  <h2 class="section-title">Alarm</h2>
  <p style="margin:0 0 6px 0; text-align:center; font-size:14px; opacity:.85;">
    Set an alarm.
  </p>
  <div class="time-row">
    <input id="alarmHour" class="time-input" type="number" min="1" max="12" placeholder="07">
    <span>:</span>
    <input id="alarmMin" class="time-input" type="number" min="0" max="59" placeholder="00">
    <select id="alarmPeriod" class="time-select">
      <option value="AM">AM</option>
      <option value="PM">PM</option>
    </select>
    <button class="btn inline-btn" onclick="submitAlarm()">Set Alarm</button>
    <button class="btn inline-btn" onclick="clearAlarm()">Clear</button>
  </div>
</section>

<!-- TIMER -->
<section class="card">
  <h2 class="section-title">Timer</h2>
  <p style="margin:0 0 6px 0; text-align:center; font-size:14px; opacity:.85;">
    Countdown timer (hours : minutes : seconds).
  </p>
  <div class="time-row">
    <input id="timerHours" class="time-input" type="number" min="0" max="99" placeholder="00">
    <span>:</span>
    <input id="timerMinutes" class="time-input" type="number" min="0" max="59" placeholder="05">
    <span>:</span>
    <input id="timerSeconds" class="time-input" type="number" min="0" max="59" placeholder="00">
    <button class="btn inline-btn" onclick="submitTimer()">Start</button>
    <button class="btn inline-btn" onclick="clearTimer()">Stop</button>
  </div>
</section>

<!-- TIME ZONE -->
<section class="card">
  <h2 class="section-title">Time Zone</h2>
  <p style="margin:0 0 6px 0; text-align:center; font-size:14px; opacity:.85;">
    Select your local time zone (DST handled where applicable).
  </p>
  <div class="time-row">
    <select id="tzSelect" class="time-select" onchange="updateTimeZone()">
      <!-- North America -->
      <option value="US_PACIFIC">US / Canada – Pacific (PST/PDT)</option>
      <option value="US_MOUNTAIN">US / Canada – Mountain (MST/MDT)</option>
      <option value="US_CENTRAL">US / Canada – Central (CST/CDT)</option>
      <option value="US_EASTERN">US / Canada – Eastern (EST/EDT)</option>
      <option value="US_ALASKA">US – Alaska</option>
      <option value="US_HAWAII">US – Hawaii</option>

      <!-- Latin America -->
      <option value="MEXICO_CITY">Mexico – Mexico City</option>
      <option value="BOGOTA">Colombia – Bogotá</option>
      <option value="LIMA">Peru – Lima</option>
      <option value="SAO_PAULO">Brazil – São Paulo</option>
      <option value="BUENOS_AIRES">Argentina – Buenos Aires</option>
      <option value="SANTIAGO">Chile – Santiago</option>

      <!-- Europe -->
      <option value="UK">UK / Ireland (GMT/BST)</option>
      <option value="EU_WEST">Western Europe (WET/WEST)</option>
      <option value="EU_CENTRAL">Central Europe (CET/CEST)</option>
      <option value="EU_EAST">Eastern Europe (EET/EEST)</option>
      <option value="MOSCOW">Russia – Moscow</option>

      <!-- Africa -->
      <option value="SOUTH_AFRICA">South Africa – Johannesburg</option>
      <option value="EGYPT">Egypt – Cairo</option>
      <option value="EAST_AFRICA">East Africa (Kenya, Tanzania)</option>
      <option value="WEST_AFRICA">West Africa (Nigeria, etc.)</option>

      <!-- Middle East -->
      <option value="TURKEY">Turkey – Istanbul</option>
      <option value="ISRAEL">Israel</option>
      <option value="SAUDI_ARABIA">Saudi Arabia – Riyadh</option>
      <option value="UAE">UAE – Dubai / Abu Dhabi</option>

      <!-- Asia -->
      <option value="INDIA">India – IST</option>
      <option value="CHINA">China – Beijing</option>
      <option value="HONG_KONG">Hong Kong</option>
      <option value="TAIWAN">Taiwan</option>
      <option value="SINGAPORE">Singapore</option>
      <option value="JAPAN">Japan – JST</option>
      <option value="KOREA">Korea – KST</option>
      <option value="BANGKOK">Thailand – Bangkok</option>
      <option value="JAKARTA">Indonesia – Jakarta</option>

      <!-- Australia / NZ / Pacific -->
      <option value="NZ">New Zealand</option>
      <option value="AUS_EAST">Australia – East (Sydney/Melbourne)</option>
      <option value="AUS_QLD">Australia – Queensland</option>
      <option value="AUS_CENTRAL">Australia – Central (Adelaide)</option>
      <option value="AUS_NT">Australia – Northern Territory</option>
      <option value="AUS_WEST">Australia – West (Perth)</option>
      <option value="FIJI">Fiji</option>

      <!-- Raw UTC -->
      <option value="UTC">UTC / GMT</option>
    </select>
  </div>
</section>

</main>
</body>
</html>
)rawliteral";

