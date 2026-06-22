const char htmlPageTime[] PROGMEM = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
    <title>Send Time to ESP32</title>
  </head>
  <body>
    <h2>Time Sync</h2>
    <p>Getting time from phone...</p>
    <script>
      const now = new Date();
      fetch(`/time?hour=${now.getHours()}&min=${now.getMinutes()}&sec=${now.getSeconds()}&ms=${now.getMilliseconds()}&day=${now.getDate()}&month=${now.getMonth()+1}&year=${now.getFullYear()}`)
        .then(() => {
          // Wait 1 second for safety then redirect
          setTimeout(() => {
            window.location.href = "/ui";
          }, 1000);
        });
    </script>
  </body>
  </html>
)rawliteral";
  
const char htmlPageUI[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover" />
  <title>Nixie Clock Control Panel</title>
  <style>
    :root {
      --bg:#140a26;           /* very dark purple background */
      --fg:#f4eaff;           /* light lavender text */
      --accent:#e9d5ff;       /* pale lavender highlight */
      --btn:#5b21b6;          /* dark purple for buttons */
      --btnHover:#6d28d9;     /* lighter purple hover */
      --btnText:#f3e8ff;      /* button text color */
      --activeGradient:linear-gradient(135deg,#c084fc,#a855f7); /* glowing lavender gradient */
      --activeBorder:#f5d0fe;
      --muted:#c4b5fd;        /* muted lavender for borders */
      --radius:14px;
      --gap:12px;
      --shadow:0 6px 16px rgba(0,0,0,.35);
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
    .btn:hover {
      background: var(--btnHover);
    }
    .btn.active {
      background: var(--activeGradient);
      border: 2px solid var(--activeBorder);
      color: #fff;
      box-shadow: 0 0 12px rgba(192,132,252,0.6), var(--shadow);
      transform: scale(1.02);
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

    .grid { display:grid; gap: var(--gap); grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); }

    .slider-label {
      font-size: clamp(15px, 3.4vw, 17px);
      display:flex;
      align-items:center;
      justify-content:space-between;
      gap:12px;
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
      width:30px; height:30px;
      border-radius:50%;
      background: var(--activeGradient);
      border:2px solid var(--activeBorder);
      margin-top:-12px;
      cursor:pointer;
      box-shadow: var(--shadow);
    }
    input[type="range"]::-moz-range-track {
      height:6px;
      background: var(--muted);
      border-radius:3px;
    }
    input[type="range"]::-moz-range-thumb {
      width:30px; height:30px;
      border-radius:50%;
      background: var(--activeGradient);
      border:2px solid var(--activeBorder);
      cursor:pointer;
      box-shadow: var(--shadow);
    }

    #colorIcon,
    #ugColorIcon {
      width: 48px;
      height: 48px;
      border-radius: 50%;
      border: 2px solid var(--muted);
      margin-left:auto;
      flex:0 0 48px;
      background:#ffffff;
      box-shadow: inset 0 0 0 1px rgba(0,0,0,.2);
    }

    .mode-buttons {
      display:grid;
      gap: var(--gap);
      grid-template-columns: repeat(auto-fit, minmax(120px, 1fr));
      margin-top: 10px;
    }

    .toggle-grid {
      display:grid;
      gap: var(--gap);
      grid-template-columns: 1fr;
    }
    @media (min-width:560px){
      .toggle-grid{ grid-template-columns: 1fr 1fr; }
    }

    /* Time / alarm / timer inputs */
    .time-row{
      display:flex;
      flex-wrap:wrap;
      align-items:center;
      gap:8px;
      margin-top:8px;
    }
    .time-input{
      width:3.2ch;
      padding:6px 8px;
      border-radius:var(--radius);
      border:1px solid var(--muted);
      background:rgba(0,0,0,.25);
      color:var(--fg);
      text-align:center;
      font-size:15px;
    }
    .time-select{
      padding:6px 10px;
      border-radius:var(--radius);
      border:1px solid var(--muted);
      background:rgba(0,0,0,.25);
      color:var(--fg);
      font-size:15px;
    }
    .inline-btn{
      flex:1;
      min-width:120px;
    }
  </style>
  <script>
    const modes=["RAINBOW","SOLID","GRADIENT","FLOW","WIPE","PULSE","BOUNCE"];
    let sliderTimer;

    function debounce(fn,delay){
      return function(...args){
        clearTimeout(sliderTimer);
        sliderTimer=setTimeout(()=>fn.apply(this,args),delay);
      };
    }

    function handleToggle(endpoint,el){
      el.disabled=true;
      fetch(endpoint).finally(()=>{
        setTimeout(()=>{el.disabled=false;},300);
      });
    }

    // ---- MAIN COLOR POSITION SLIDER (0..255) ----
    const updateSlider=debounce((val)=>{
      document.getElementById("sliderValue").textContent=val;
      fetch("/setColorPos?val="+val).then(fetchColor);
    },120);

    function fetchColor(){
      fetch("/getColor")
        .then(r=>r.json())
        .then(data=>{
          const rgb=`rgb(${data.r},${data.g},${data.b})`;
          document.getElementById("colorIcon").style.backgroundColor=rgb;
        })
        .catch(()=>{});
    }

    function setColorState(mode){
      fetch("/setColorState?mode="+mode).then(()=>highlightMode(mode));
    }

    function highlightMode(active){
      modes.forEach(m=>{
        const btn=document.getElementById("btn_"+m);
        if(btn) btn.classList.toggle("active",m===active);
      });
    }

    // ---- PANEL BRIGHTNESS SLIDER (0..100 mapped to 0..255) ----
    function updateBrightness(val){
      const v=parseInt(val,10);
      document.getElementById("brightnessValue").textContent = isNaN(v)?0:v;
      fetch("/setBrightness?val="+v);
    }

    function fetchBrightness(){
      fetch("/getBrightness")
        .then(r=>r.json())
        .then(data=>{
          const pct = (data && typeof data.pct==="number") ? data.pct : 100;
          const slider = document.getElementById("brightnessSlider");
          const label  = document.getElementById("brightnessValue");
          slider.value = pct;
          label.textContent = pct;
        })
        .catch(()=>{});
    }

    // ---- UNDERGLOW BRIGHTNESS SLIDER (0..100) ----
    function updateUGBrightness(val){
      const v = parseInt(val,10);
      document.getElementById("ugBrightnessValue").textContent = isNaN(v)?0:v;
      fetch("/setUGBrightness?val="+v);
    }

    function fetchUGBrightness(){
      fetch("/getUGBrightness")
        .then(r=>r.json())
        .then(data=>{
          const pct = (data && typeof data.pct==="number") ? data.pct : 100;
          const slider = document.getElementById("ugBrightnessSlider");
          const label  = document.getElementById("ugBrightnessValue");
          slider.value = pct;
          label.textContent = pct;
        })
        .catch(()=>{});
    }

    // ---- UNDERGLOW COLOR POSITION SLIDER (0..255) ----
    const updateUGSlider = debounce((val)=>{
      document.getElementById("ugSliderValue").textContent = val;
      fetch("/setUGColorPos?val=" + val).then(fetchUGColor);
    }, 120);

    function fetchUGColor(){
      fetch("/getUGColor")
        .then(r => r.json())
        .then(data => {
          const rgb = `rgb(${data.r},${data.g},${data.b})`;
          document.getElementById("ugColorIcon").style.backgroundColor = rgb;
        })
        .catch(()=>{});
    }

    // ---- ALARM UI (with placeholder defaults) ----
    function submitAlarm(){
      const hEl  = document.getElementById("alarmHour");
      const mEl  = document.getElementById("alarmMin");
      const apEl = document.getElementById("alarmPeriod");

      let h = hEl.value ? parseInt(hEl.value,10) : parseInt(hEl.placeholder,10);
      let m = mEl.value ? parseInt(mEl.value,10) : parseInt(mEl.placeholder,10);
      const ap = apEl.value || "AM";

      if(h<1 || h>12 || m<0 || m>59){
        alert("Enter 1–12 for hour, 0–59 for minutes.");
        return;
      }

      fetch(`/setAlarm?hour=${h}&min=${m}&ampm=${encodeURIComponent(ap)}`)
        .then(r=>r.text())
        .then(txt=>console.log("Alarm:",txt))
        .catch(()=>{});
    }

    function clearAlarm(){
      fetch("/clearAlarm")
        .then(r => r.text())
        .then(txt => {
          console.log("Alarm:", txt);
          document.getElementById("alarmHour").value = "";
          document.getElementById("alarmMin").value  = "";
          document.getElementById("alarmPeriod").value = "AM";
        })
        .catch(()=>{});
    }

    // ---- TIMER UI (with placeholder defaults) ----
    function submitTimer(){
      const hEl=document.getElementById("timerHours");
      const mEl=document.getElementById("timerMinutes");
      const sEl=document.getElementById("timerSeconds");

      let h = hEl.value ? parseInt(hEl.value,10) : parseInt(hEl.placeholder,10);
      let m = mEl.value ? parseInt(mEl.value,10) : parseInt(mEl.placeholder,10);
      let s = sEl.value ? parseInt(sEl.value,10) : parseInt(sEl.placeholder,10);

      if(h<0) h=0; if(h>99) h=99;
      if(m<0) m=0; if(m>59) m=59;
      if(s<0) s=0; if(s>59) s=59;

      if(h===0 && m===0 && s===0){
        alert("Timer must be at least 1 second.");
        return;
      }

      fetch(`/setTimer?h=${h}&m=${m}&s=${s}`)
        .then(r=>r.text())
        .then(txt=>console.log("Timer:",txt))
        .catch(()=>{});
    }

    function clearTimer(){
      fetch("/stopTimer")
        .then(r => r.text())
        .then(txt => {
          console.log("Timer:", txt);
          document.getElementById("timerHours").value   = "";
          document.getElementById("timerMinutes").value = "";
          document.getElementById("timerSeconds").value = "";
        })
        .catch(()=>{});
    }

    window.addEventListener("load",()=>{
      document.getElementById("sliderValue").textContent = 200;
      document.getElementById("ugSliderValue").textContent = 200;

      fetchColor();
      fetchUGColor();
      highlightMode("RAINBOW");
      fetchBrightness();
      fetchUGBrightness();
    });
  </script>
</head>
<body>
  <header><h1>Nixie Clock Control Panel</h1></header>
  <main class="container">
    <!-- Quick toggles -->
    <section class="card">
      <h2 class="section-title">Quick Settings</h2>
      <div class="toggle-grid">
        <button class="btn" onclick="handleToggle('/toggle24hr', this)">Toggle 24-Hour Mode</button>
        <button class="btn" onclick="handleToggle('/toggleLeadingZero', this)">Toggle Leading Zero</button>
      </div>
    </section>

    <!-- Panel Brightness -->
    <section class="card">
      <h2 class="section-title">Panel Brightness</h2>
      <div class="slider-label">
        <span>Level: <strong id="brightnessValue">100</strong>%</span>
      </div>
      <div class="slider-wrap">
        <input id="brightnessSlider" type="range" min="0" max="100" value="100" oninput="updateBrightness(this.value)">
      </div>
    </section>

    <!-- UnderGlow Brightness -->
    <section class="card">
      <h2 class="section-title">UnderGlow Brightness</h2>
      <div class="slider-label">
        <span>Level: <strong id="ugBrightnessValue">100</strong>%</span>
      </div>
      <div class="slider-wrap">
        <input id="ugBrightnessSlider" type="range" min="0" max="100" value="100" oninput="updateUGBrightness(this.value)">
      </div>
    </section>

    <!-- Panel Color Wheel Position -->
    <section class="card">
      <h2 class="section-title">Panel Color</h2>
      <div class="slider-label">
        <span>Value: <strong id="sliderValue">200</strong></span>
        <div id="colorIcon"></div>
      </div>
      <div class="slider-wrap">
        <input type="range" min="0" max="255" value="200" oninput="updateSlider(this.value)">
      </div>
    </section>

    <!-- UnderGlow Color Position -->
    <section class="card">
      <h2 class="section-title">UnderGlow Color</h2>
      <div class="slider-label">
        <span>Value: <strong id="ugSliderValue">200</strong></span>
        <div id="ugColorIcon"></div>
      </div>
      <div class="slider-wrap">
        <input type="range" min="0" max="255" value="200" oninput="updateUGSlider(this.value)">
      </div>
    </section>

    <!-- Color Mode -->
    <section class="card">
      <h2 class="section-title">Select Color Mode</h2>
      <div class="mode-buttons">
        <button id="btn_RAINBOW" class="btn" onclick="setColorState('RAINBOW')">Rainbow</button>
        <button id="btn_SOLID" class="btn" onclick="setColorState('SOLID')">Solid</button>
        <button id="btn_GRADIENT" class="btn" onclick="setColorState('GRADIENT')">Gradient</button>
        <button id="btn_FLOW" class="btn" onclick="setColorState('FLOW')">Flow</button>
        <button id="btn_WIPE" class="btn" onclick="setColorState('WIPE')">Wipe</button>
        <button id="btn_PULSE" class="btn" onclick="setColorState('PULSE')">Pulse</button>
        <button id="btn_BOUNCE" class="btn" onclick="setColorState('BOUNCE')">Bounce</button>
      </div>
    </section>

    <!-- Alarm -->
    <section class="card">
      <h2 class="section-title">Alarm</h2>
      <p style="margin:0 0 6px 0; text-align:center; font-size:14px; opacity:.85;">
        Set an alarm time in 12-hour format.
      </p>
      <div class="time-row">
        <input id="alarmHour" class="time-input" type="number" min="1" max="12" placeholder="07" />
        <span>:</span>
        <input id="alarmMin" class="time-input" type="number" min="0" max="59" placeholder="00" />
        <select id="alarmPeriod" class="time-select">
          <option value="AM">AM</option>
          <option value="PM">PM</option>
        </select>
        <button class="btn inline-btn" onclick="submitAlarm()">Set Alarm</button>
        <button class="btn inline-btn" onclick="clearAlarm()">Clear Alarm</button>
      </div>
    </section>

    <!-- Timer -->
    <section class="card">
      <h2 class="section-title">Timer</h2>
      <p style="margin:0 0 6px 0; text-align:center; font-size:14px; opacity:.85;">
        Countdown timer (hours : minutes : seconds).
      </p>
      <div class="time-row">
        <input id="timerHours" class="time-input" type="number" min="0" max="99" placeholder="00" />
        <span>:</span>
        <input id="timerMinutes" class="time-input" type="number" min="0" max="59" placeholder="05" />
        <span>:</span>
        <input id="timerSeconds" class="time-input" type="number" min="0" max="59" placeholder="00" />
        <button class="btn inline-btn" onclick="submitTimer()">Start Timer</button>
        <button class="btn inline-btn" onclick="clearTimer()">Stop Timer</button>
      </div>
    </section>
  </main>
</body>
</html>
)rawliteral";