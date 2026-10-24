/*
 * ESP8266 (NodeMCU) + A4988 stepper control over WiFi  —  v2 UI
 * --------------------------------------------------------------
 * The ESP creates its own WiFi hotspot. Connect your phone to it,
 * open the page, and drive the motor: Forward / Reverse / Stop,
 * a speed slider with live RPM, smooth acceleration ramping, and
 * a "jog N steps" control. The page shows a live status that
 * reflects the motor even if controlled from another phone.
 *
 * WIRING (unchanged):
 *   A4988 STEP -> D5 (GPIO5)
 *   A4988 DIR  -> D2 (GPIO4)
 *   A4988 VDD  -> 3V3        A4988 GND (logic) -> G
 *   A4988 VMOT -> +12V       A4988 GND (motor) -> 12V GND
 *   RSTimpl jumpered to SLP. 100uF across VMOT/GND. Common ground tied.
 *
 * LIBRARY: install "AccelStepper" by Mike McCauley
 *   (Arduino IDE -> Tools -> Manage Libraries -> search AccelStepper)
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <AccelStepper.h>

// ---- Pins ----
#define STEP_PIN  14   // D1
#define DIR_PIN   4   // D2

// ---- WiFi hotspot credentials (change if you like) ----
const char* AP_SSID = "Stepper-ESP";
const char* AP_PASS = "12345678";   // min 8 chars

// ---- Motor config ----
const int   STEPS_PER_REV = 200;    // full steps/rev (1.8deg). Multiply if microstepping.
const int   MIN_SPEED     = 50;
const int   MAX_SPEED     = 2000;
const float RAMP_RATE     = 1200.0; // steps/sec^2 used to smooth speed changes

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
ESP8266WebServer server(80);

int   targetSpeed  = 400;   // steps/sec requested via slider
float liveSpeed    = 0;     // current ramped speed magnitude
int   direction    = 0;     // -1 rev, 0 stop, +1 fwd
long  jogRemaining = 0;     // steps left to move in jog mode (signed handled via dir)
int   jogDir       = 1;
unsigned long lastRampMs = 0;

// ---- Web page ----
String page() {
  String s = F(
"<!DOCTYPE html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no'>"
"<title>Stepper Control</title><style>"
":root{--bg:#0e1116;--card:#171c24;--mut:#8b95a5;--line:#262d38;--fwd:#22c55e;--rev:#3b82f6;--stop:#ef4444}"
"*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}"
"body{font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;background:var(--bg);color:#e7ecf3;margin:0;padding:18px;max-width:460px;margin:0 auto}"
"h1{font-size:20px;font-weight:600;text-align:center;margin:4px 0 16px}"
".card{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:18px;margin-bottom:14px}"
".dial{position:relative;width:170px;height:170px;margin:4px auto 10px}"
".ring{width:100%;height:100%;border-radius:50%;border:6px solid var(--line);position:relative;display:flex;align-items:center;justify-content:center}"
".spin{position:absolute;inset:-6px;border-radius:50%;border:6px solid transparent;border-top-color:var(--mut)}"
".dial.fwd .spin{border-top-color:var(--fwd)}.dial.rev .spin{border-top-color:var(--rev)}"
".dial.run .spin{animation:rot var(--dur,1s) linear infinite}"
".dial.rev.run .spin{animation-direction:reverse}"
"@keyframes rot{to{transform:rotate(360deg)}}"
".dial .rpm{font-size:30px;font-weight:700;line-height:1}.dial .u{font-size:12px;color:var(--mut);margin-top:4px}"
".state{text-align:center;font-size:13px;color:var(--mut);margin:0 0 6px}"
".state b{color:#e7ecf3}"
".lab{display:flex;justify-content:space-between;font-size:13px;color:var(--mut);margin-bottom:8px}"
".lab b{color:#e7ecf3;font-size:15px}"
"input[type=range]{-webkit-appearance:none;width:100%;height:8px;border-radius:8px;background:var(--line);outline:none}"
"input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:26px;height:26px;border-radius:50%;background:#e7ecf3;border:3px solid var(--card);box-shadow:0 0 0 1px var(--line)}"
".row{display:flex;gap:10px}"
"button{flex:1;font-size:16px;font-weight:600;padding:16px 8px;border:0;border-radius:12px;color:#fff;background:#2a313c;transition:transform .05s,filter .15s}"
"button:active{transform:scale(.97)}"
".fwd{background:var(--fwd)}.rev{background:var(--rev)}.stop{background:var(--stop);width:100%;flex:none;margin-top:10px}"
"button.off{filter:grayscale(.6) brightness(.6)}"
".jog{display:flex;gap:10px;align-items:center;margin-top:4px}"
".jog input{flex:1;background:var(--bg);border:1px solid var(--line);color:#e7ecf3;border-radius:10px;padding:12px;font-size:15px;text-align:center}"
".jog button{flex:none;width:46px;padding:12px 0;font-size:18px;background:#2a313c}"
".sub{font-size:12px;color:var(--mut);text-align:center;margin-top:2px}"
"</style></head><body>"
"<h1>Stepper Control</h1>"

"<div class='card'>"
"<div id='dial' class='dial'><div class='spin'></div>"
"<div class='ring'><div><div class='rpm' id='rpm'>0</div><div class='u'>RPM</div></div></div></div>"
"<p class='state'>Status: <b id='st'>Stopped</b> &middot; <span id='sp'>0</span> steps/s</p>"
"</div>"

"<div class='card'>"
"<div class='lab'><span>Speed</span><b><span id='sv'>");
  s += String(targetSpeed);
  s += F("</span> steps/s</b></div>"
"<input id='rng' type='range' min='");
  s += String(MIN_SPEED); s += F("' max='"); s += String(MAX_SPEED);
  s += F("' value='"); s += String(targetSpeed);
  s += F("' oninput='onSlide(this.value)'>"
"<p class='sub'>50 – 2000 steps/s</p>"
"</div>"

"<div class='card'>"
"<div class='row'>"
"<button id='bf' class='fwd' onclick=\"go('/fwd')\">&#9650; Forward</button>"
"<button id='br' class='rev' onclick=\"go('/rev')\">&#9660; Reverse</button>"
"</div>"
"<button class='stop' onclick=\"go('/stop')\">&#9632; Stop</button>"
"</div>"

"<div class='card'>"
"<div class='lab'><span>Jog (move fixed steps)</span></div>"
"<div class='jog'>"
"<button onclick=\"jog(-1)\">&minus;</button>"
"<input id='jn' type='number' value='200' min='1'>"
"<button onclick=\"jog(1)\">+</button>"
"</div>"
"<p class='sub'>One revolution = "); s += String(STEPS_PER_REV); s += F(" steps</p>"
"</div>"

"<script>");
  s += "var SPR=" + String(STEPS_PER_REV) + ";";
  s += F(
"function onSlide(v){document.getElementById('sv').innerText=v;fetch('/speed?v='+v)}"
"function go(u){fetch(u).then(poll)}"
"function jog(d){var n=parseInt(document.getElementById('jn').value)||0;if(n<=0)return;fetch('/jog?n='+n+'&d='+d).then(poll)}"
"function render(d){"
" var dial=document.getElementById('dial');"
" document.getElementById('rpm').innerText=Math.round(d.rpm);"
" document.getElementById('sp').innerText=Math.round(d.speed);"
" var t=d.dir>0?'Forward':d.dir<0?'Reverse':(d.jog?'Jogging':'Stopped');"
" document.getElementById('st').innerText=t;"
" dial.className='dial'+(d.speed>1?' run':'')+(d.dir>0||(d.jog&&d.jogdir>0)?' fwd':'')+(d.dir<0||(d.jog&&d.jogdir<0)?' rev':'');"
" var dur=d.speed>1?(SPR/d.speed):1; if(dur<0.15)dur=0.15;"
" dial.style.setProperty('--dur',dur.toFixed(3)+'s');"
" document.getElementById('bf').classList.toggle('off',d.dir<=0);"
" document.getElementById('br').classList.toggle('off',d.dir>=0);"
"}"
"function poll(){fetch('/status').then(r=>r.json()).then(render).catch(()=>{})}"
"setInterval(poll,500);poll();"
"</script></body></html>");
  return s;
}

void setDir(int d) {           // start continuous motion, cancels any jog
  jogRemaining = 0;
  direction = d;
}

void startJog(long n, int d) { // move n steps in direction d, then stop
  direction = 0;
  jogDir = (d >= 0) ? 1 : -1;
  jogRemaining = n;
}

void sendStatus() {
  float rpm = (liveSpeed / (float)STEPS_PER_REV) * 60.0;
  String j = "{";
  j += "\"dir\":" + String(direction);
  j += ",\"speed\":" + String(liveSpeed, 0);
  j += ",\"rpm\":" + String(rpm, 1);
  j += ",\"target\":" + String(targetSpeed);
  j += ",\"jog\":" + String(jogRemaining > 0 ? 1 : 0);
  j += ",\"jogdir\":" + String(jogDir);
  j += "}";
  server.send(200, "application/json", j);
}

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setSpeed(0);

  WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/",      []() { server.send(200, "text/html", page()); });
  server.on("/fwd",   []() { setDir(1);  server.send(200, "text/plain", "fwd"); });
  server.on("/rev",   []() { setDir(-1); server.send(200, "text/plain", "rev"); });
  server.on("/stop",  []() { setDir(0);  server.send(200, "text/plain", "stop"); });
  server.on("/speed", []() {
    if (server.hasArg("v")) {
      targetSpeed = constrain(server.arg("v").toInt(), MIN_SPEED, MAX_SPEED);
    }
    server.send(200, "text/plain", "ok");
  });
  server.on("/jog", []() {
    long n = server.hasArg("n") ? server.arg("n").toInt() : 0;
    int  d = server.hasArg("d") ? server.arg("d").toInt() : 1;
    if (n > 0) startJog(n, d);
    server.send(200, "text/plain", "jog");
  });
  server.on("/status", sendStatus);
  server.begin();

  lastRampMs = millis();
}

void loop() {
  server.handleClient();

  // ---- smooth speed ramp toward the active target ----
  unsigned long now = millis();
  float dt = (now - lastRampMs) / 1000.0;
  lastRampMs = now;

  bool moving = (direction != 0) || (jogRemaining > 0);
  float want  = moving ? (float)targetSpeed : 0.0;
  float maxStep = RAMP_RATE * dt;
  if (liveSpeed < want)      liveSpeed = min(want, liveSpeed + maxStep);
  else if (liveSpeed > want) liveSpeed = max(want, liveSpeed - maxStep);

  // ---- drive ----
  if (jogRemaining > 0) {
    stepper.setSpeed(jogDir * liveSpeed);
    long before = stepper.currentPosition();
    if (stepper.runSpeed()) {
      jogRemaining -= abs(stepper.currentPosition() - before);
      if (jogRemaining <= 0) { jogRemaining = 0; }
    }
  } else if (direction != 0 || liveSpeed > 0) {
    stepper.setSpeed(direction * liveSpeed);
    if (liveSpeed > 0) stepper.runSpeed();   // keep coasting down while ramping to 0
  }
}
