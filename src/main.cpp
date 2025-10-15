#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

/* ======================== User Config ======================== */

// Wi-Fi AP
static constexpr char AP_SSID[]     = "HoverCtrl";
static constexpr char AP_PASSWORD[] = "hover123";
static constexpr int  AP_CHANNEL    = 6;

// Hoverboard link (UART0 on TX/RX pins)
static constexpr int      TX_HOVER   = TX;
static constexpr int      RX_HOVER   = RX;
static constexpr uint32_t HOVER_BAUD = 115200;

// Protocol / timing
static constexpr uint16_t START_FRAME        = 0xABCD;
static constexpr uint16_t SEND_PERIOD_MS     = 20;     // 50 Hz commands
static constexpr uint32_t ARM_DELAY_MS       = 1000;   // send zero for a bit
static constexpr uint32_t COMMAND_TIMEOUT_MS = 500;    // decay to zero if no UI updates
static constexpr uint32_t FB_STALE_MS        = 1200;   // consider feedback stale after this

// Scaling & limits (tune to taste, depends on your firmware build)
static constexpr int16_t VOLT_FULL_UNITS         = 1000;  // firmware “100%”
static constexpr float   THROTTLE_CAP_PCT        = 50.0f; // UI soft cap
static constexpr float   THROTTLE_SLEW_PCT_PER_S = 80.0f; // rate limit

// Mixer gains (compensate firmware scaling so levers feel independent)
static constexpr float   GAIN_COMMON = 1.00f; // scales (L+R)/2
static constexpr float   GAIN_DIFF   = 1.40f; // scales (R-L)/2
static constexpr int     STEER_SIGN  = +1;    // flip to -1 if steering inverted

/* ======================== Hoverboard Protocol ======================== */

#pragma pack(push,1)
struct SerialCommand {
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  uint16_t checksum;
};

struct SerialFeedback {
  uint16_t start;
  int16_t  cmd1;
  int16_t  cmd2;
  int16_t  speedR_meas;
  int16_t  speedL_meas;
  int16_t  batVoltage;  // x10 (e.g., 420 -> 42.0 V)
  int16_t  boardTemp;   // °C
  uint16_t cmdLed;
  uint16_t checksum;
};
#pragma pack(pop)

namespace HB {
  HardwareSerial& port(){ static HardwareSerial h(0); return h; }
  void begin(){ port().begin(HOVER_BAUD, SERIAL_8N1, RX_HOVER, TX_HOVER); }

  inline uint16_t cs(const SerialCommand& c){ return (uint16_t)(c.start ^ c.steer ^ c.speed); }

  inline int16_t pctToUnits(float pct){
    pct = fminf(fmaxf(pct, -100.0f), 100.0f);
    float u = pct * (VOLT_FULL_UNITS / 100.0f);
    if (u >  (float)VOLT_FULL_UNITS) u =  (float)VOLT_FULL_UNITS;
    if (u < -(float)VOLT_FULL_UNITS) u = -(float)VOLT_FULL_UNITS;
    return (int16_t)lroundf(u);
  }

  void sendSpeedSteerPct(float speedPct, float steerPct){
    SerialCommand c;
    c.start = START_FRAME;
    c.speed = pctToUnits(speedPct);
    c.steer = pctToUnits(steerPct);
    c.checksum = cs(c);
    port().write(reinterpret_cast<const uint8_t*>(&c), sizeof(c));
  }

  // Tank inputs: Lp, Rp are signed percents (−100..+100)
  void sendTankPct(float Lp, float Rp){
    float speed = GAIN_COMMON * 0.5f * (Lp + Rp);
    float steer = GAIN_DIFF   * 0.5f * (Rp - Lp) * STEER_SIGN;
    sendSpeedSteerPct(speed, steer);
  }
}

/* ======================== Web UI (HTML) ======================== */

WebServer server(80);

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>HoverCtrl Tank</title>
<style>
  :root { font-family: system-ui, -apple-system, Segoe UI, Roboto, sans-serif; color-scheme: light dark; }
  body { margin:16px }
  details { margin-bottom: 16px; }
  details > summary { cursor:pointer; font-weight:600; padding:8px 0; }
  .telemetry{ padding:14px; border-radius:12px; box-shadow:0 2px 10px rgba(0,0,0,.12); }
  .tl-grid{ display:grid; grid-template-columns: repeat(auto-fit, minmax(140px, 1fr)); gap:12px }
  .tl { padding:10px 12px; border-radius:10px; background:rgba(127,127,127,.08); }
  .tl .v{ font-weight:700; font-variant-numeric: tabular-nums }
  .grid { display:grid; gap:48px; grid-template-columns: repeat(auto-fit, minmax(280px,1fr)); }
  .card { padding:16px; border-radius:12px; box-shadow:0 2px 10px rgba(0,0,0,.12); }
  h3 { margin:0 0 12px 0; font-size:18px }
  input[type=range]{ width:100% }
  .row{ display:flex; align-items:center; gap:10px }
  .val{ width:56px; text-align:right; font-variant-numeric: tabular-nums }
  .btn{ padding:10px 14px; border-radius:10px; border:0; cursor:pointer }
  .spacer { height:80px; } /* large vertical spacing on narrow layouts */
  .cfg{ display:flex; gap:14px; align-items:center; flex-wrap:wrap; margin: 10px 0 20px 0; }
  label{ user-select:none }
</style>

<!-- Collapsible telemetry -->
<details>
  <summary>Telemetry</summary>
  <div class="telemetry">
    <div class="tl-grid">
      <div class="tl">Battery<br><span class="v" id="vb">--.-</span> V</div>
      <div class="tl">Temp<br><span class="v" id="tp">--</span> °C</div>
      <div class="tl">Speed L<br><span class="v" id="sl">--</span></div>
      <div class="tl">Speed R<br><span class="v" id="sr">--</span></div>
      <div class="tl">Status<br><span class="v" id="st">No data</span></div>
    </div>
  </div>
</details>

<!-- Runtime input mapping controls -->
<div class="cfg">
  <label><input type="checkbox" id="swapLR"> Swap L/R</label>
  <label><input type="checkbox" id="invL"> Invert Left</label>
  <label><input type="checkbox" id="invR"> Invert Right</label>
  <button class="btn" id="stop">STOP</button>
</div>

<div class="grid">
  <div class="card">
    <h3>Left</h3>
    <div class="row">
      <input id="left" type="range" min="-100" max="100" step="1" value="0">
      <span class="val"><span id="lv">0</span>%</span>
    </div>
  </div>

  <div class="spacer"></div>

  <div class="card">
    <h3>Right</h3>
    <div class="row">
      <input id="right" type="range" min="-100" max="100" step="1" value="0">
      <span class="val"><span id="rv">0</span>%</span>
    </div>
  </div>
</div>

<p style="margin-top:8px">Keys. I/K = Left ±, O/L = Right ±. Space = stop. Shift = faster.</p>

<script>
  // UI elements
  const left  = document.getElementById('left');
  const right = document.getElementById('right');
  const lv = document.getElementById('lv');
  const rv = document.getElementById('rv');
  const stopBtn = document.getElementById('stop');

  const vb = document.getElementById('vb');
  const tp = document.getElementById('tp');
  const sl = document.getElementById('sl');
  const sr = document.getElementById('sr');
  const st = document.getElementById('st');

  const swapLR = document.getElementById('swapLR');
  const invL   = document.getElementById('invL');
  const invR   = document.getElementById('invR');

  // Persist mapping options
  function loadCfg(){
    swapLR.checked = localStorage.getItem('swapLR') === '1';
    invL.checked   = localStorage.getItem('invL') === '1';
    invR.checked   = localStorage.getItem('invR') === '1';
  }
  function saveCfg(){
    localStorage.setItem('swapLR', swapLR.checked ? '1' : '0');
    localStorage.setItem('invL',   invL.checked   ? '1' : '0');
    localStorage.setItem('invR',   invR.checked   ? '1' : '0');
  }
  [swapLR, invL, invR].forEach(el => el.addEventListener('change', saveCfg));
  loadCfg();

  function updateLabels(){ lv.textContent = left.value; rv.textContent = right.value; }
  let lastSentL=9999, lastSentR=9999, dirty=true;

  left.addEventListener('input', ()=>{ dirty=true; updateLabels(); });
  right.addEventListener('input', ()=>{ dirty=true; updateLabels(); });

  stopBtn.addEventListener('click', ()=>{
    left.value = right.value = 0; dirty = true; updateLabels(); navigator.vibrate?.(30);
  });

  // Keyboard
  addEventListener('keydown', (e)=>{
    const s = e.shiftKey ? 8 : 4;
    if (e.key==='i') left.value  = Math.min( 100, +left.value  + s);
    if (e.key==='k') left.value  = Math.max(-100, +left.value  - s);
    if (e.key==='o') right.value = Math.min( 100, +right.value + s);
    if (e.key==='l') right.value = Math.max(-100, +right.value - s);
    if (e.key===' ') { left.value = right.value = 0; }
    dirty = true; updateLabels();
  });

  // Apply mapping: swap + per-side inversion
  function mapLR(L, R){
    // swap first (so "invert left" always refers to the left slider on screen)
    if (swapLR.checked){ [L, R] = [R, L]; }
    if (invL.checked) L = -L;
    if (invR.checked) R = -R;
    return [L, R];
  }

  async function sendLoop(){
    try{
      let L = +left.value, R = +right.value;
      [L, R] = mapLR(L, R);
      if (dirty && (L!==lastSentL || R!==lastSentR)) {
        lastSentL=L; lastSentR=R; dirty=false;
        fetch(`/set?l=${L}&r=${R}`, {cache:'no-store'});
      }
    }catch(e){}
    setTimeout(sendLoop, 50);
  }

  async function pollStatus(){
    try{
      const r = await fetch('/status', {cache:'no-store'});
      if (r.ok){
        const j = await r.json();
        vb.textContent = j.volts.toFixed(1);
        tp.textContent = j.tempC;
        sl.textContent = j.speedL;
        sr.textContent = j.speedR;
        st.textContent = (j.stale ? 'Stale' : 'OK');
      }
    }catch(e){
      st.textContent = 'No link';
    }
    setTimeout(pollStatus, 300);
  }

  updateLabels();
  sendLoop();
  pollStatus();
</script>
)HTML";

/* ======================== Globals ======================== */

uint32_t tBoot=0, tLastSend=0, tLastCmd=0;

// Commands
float cmdL=0, cmdR=0, outL=0, outR=0;

// Feedback state (updated by parser)
volatile int16_t fb_speedL = 0;
volatile int16_t fb_speedR = 0;
volatile int16_t fb_tempC  = 0;
volatile float   fb_volts  = NAN;
volatile uint32_t tLastFb  = 0;

/* ======================== Utils ======================== */

static inline float clampf(float x,float a,float b){ return fminf(fmaxf(x,a), b); }
static inline float slew(float x,float target,float dt){
  const float md = THROTTLE_SLEW_PCT_PER_S * dt;
  float d = target - x;
  if (d >  md) d =  md;
  if (d < -md) d = -md;
  return x + d;
}

/* ======================== Feedback Parser ======================== */

// Resync-on-start-frame receiver
static void pollFeedback(){
  static uint8_t  idx = 0;
  static uint16_t startWin = 0;
  static uint8_t* p = nullptr;
  static SerialFeedback fbNew;

  auto& S = HB::port();
  while (S.available()){
    uint8_t b = S.read();
    startWin = (uint16_t)((startWin << 8) | b);

    if (idx == 0 && startWin == START_FRAME){
      p = (uint8_t*)&fbNew;
      // write start (little-endian)
      *p++ = (uint8_t)(START_FRAME & 0xFF);
      *p++ = (uint8_t)((START_FRAME >> 8) & 0xFF);
      idx = 2;
      continue;
    }

    if (idx >= 2 && idx < sizeof(SerialFeedback)){
      *p++ = b;
      idx++;
      if (idx == sizeof(SerialFeedback)){
        uint16_t cs = fbNew.start ^ fbNew.cmd1 ^ fbNew.cmd2 ^ fbNew.speedR_meas ^
                      fbNew.speedL_meas ^ fbNew.batVoltage ^ fbNew.boardTemp ^ fbNew.cmdLed;
        if (fbNew.start == START_FRAME && cs == fbNew.checksum){
          fb_speedL = fbNew.speedL_meas;
          fb_speedR = fbNew.speedR_meas;
          fb_tempC  = fbNew.boardTemp;
          fb_volts  = fbNew.batVoltage * 0.1f; // x10 → volts
          tLastFb   = millis();
        }
        idx = 0; // reset for next frame
        return;
      }
    }
  }
}

/* ======================== HTTP Handlers ======================== */

void handleRoot(){ server.send_P(200, "text/html; charset=utf-8", INDEX_HTML); }

void handleSet(){
  if (!server.hasArg("l") || !server.hasArg("r")) { server.send(400, "text/plain", "Missing l/r"); return; }
  float l = clampf(server.arg("l").toFloat(), -100.0f, 100.0f);
  float r = clampf(server.arg("r").toFloat(), -100.0f, 100.0f);
  l = clampf(l, -THROTTLE_CAP_PCT, THROTTLE_CAP_PCT);
  r = clampf(r, -THROTTLE_CAP_PCT, THROTTLE_CAP_PCT);
  cmdL = l; cmdR = r;
  tLastCmd = millis();
  server.send(200, "text/plain", "OK");
}

void handleStatus(){
  // snapshot (avoid tearing)
  float volts = isnan(fb_volts) ? NAN : fb_volts;
  int   tempC = fb_tempC;
  int   sL    = fb_speedL;
  int   sR    = fb_speedR;
  bool  stale = (millis() - tLastFb) > FB_STALE_MS;

  char buf[160];
  if (isnan(volts)) volts = 0.0f;
  snprintf(buf, sizeof(buf),
           "{\"volts\":%.1f,\"tempC\":%d,\"speedL\":%d,\"speedR\":%d,\"stale\":%s}",
           volts, tempC, sL, sR, stale ? "true" : "false");
  server.send(200, "application/json", buf);
}

void handleNotFound(){ server.send(404, "text/plain", "Not found"); }

/* ======================== Setup & Loop ======================== */

void setup(){
  HB::begin();

  WiFi.mode(WIFI_AP);
  (void)WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, false, 4);

  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/status", handleStatus);
  server.onNotFound(handleNotFound);
  server.begin();

  tBoot = millis();
  tLastSend = tBoot;
  tLastCmd  = tBoot;
}

void loop(){
  server.handleClient();

  // always poll feedback
  pollFeedback();

  const uint32_t now = millis();

  // arming: send zero for a bit after boot
  if (now - tBoot < ARM_DELAY_MS){
    if (now - tLastSend >= SEND_PERIOD_MS){
      HB::sendTankPct(0,0);
      tLastSend = now;
    }
    return;
  }

  // timeout → decay to zero
  if (now - tLastCmd > COMMAND_TIMEOUT_MS){ cmdL = 0; cmdR = 0; }

  // slew-limit toward commanded
  const float dt = (now - tLastSend) * 0.001f;
  if (dt > 0){
    outL = slew(outL, cmdL, dt);
    outR = slew(outR, cmdR, dt);
  }

  if (now - tLastSend >= SEND_PERIOD_MS){
    HB::sendTankPct(outL, outR);
    tLastSend = now;
  }
}