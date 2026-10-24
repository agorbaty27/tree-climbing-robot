/*
 * ============================================================================
 *  DUAL STEPPER WIFI CONTROL — one sketch for TWO NodeMCU ESP8266 boards
 * ============================================================================
 *  Each board drives ONE stepper through ONE A4988 / BT7221A driver
 *  (STEP/DIR interface — the two drivers are pin-compatible, treated the same).
 *
 *  MASTER board : hosts the web UI, drives Motor A, relays commands to slave.
 *  SLAVE  board : drives Motor B, exposes /m/* endpoints the master calls.
 *  Both join your home WiFi (station mode). Same .ino for both boards —
 *  comment/uncomment ROLE_MASTER below and flash each board.
 *
 *  WIRING (identical on both boards):
 *    driver STEP  -> D5 (GPIO14)
 *    driver DIR   -> D2 (GPIO4)
 *    driver EN    -> not connected (left open = always enabled).
 *                    Enable/Disable in the UI is a SOFTWARE lockout only; with
 *                    no EN wire it cannot de-energize the coils (motor keeps
 *                    holding torque). Wire EN to a GPIO and set HAVE_EN_PIN 1
 *                    below if you want true coil-off disable.
 *    KILL switch  -> D7 (GPIO13)   normally-CLOSED to GND = "run"
 *                                  (open contact OR broken wire = KILL, fail-safe)
 *    driver VDD   -> 3V3           driver logic GND -> G
 *    driver VMOT  -> +12V          driver motor GND -> 12V GND
 *    RST <-> SLP jumpered on each driver (mandatory — asleep otherwise!)
 *    100uF electrolytic across VMOT/GND on EACH driver.
 *    COMMON GROUND: tie both ESP grounds and both 12V supply grounds together.
 *
 *  LIBRARIES (Arduino IDE, board "NodeMCU 1.0 (ESP-12E)"):
 *    - AccelStepper by Mike McCauley  (Library Manager)
 *    - Everything else ships with the ESP8266 core:
 *      ESP8266WiFi, ESP8266WebServer, ESP8266HTTPClient, ESP8266mDNS,
 *      LittleFS, espnow.
 *
 *  TIMING NOTE (important): the ESP8266 is single-core and also runs WiFi.
 *  Step pulses here come from AccelStepper serviced in loop(), so very high
 *  simultaneous speeds (roughly >1500-2000 steps/s while the web server is
 *  busy) may jitter and can stall the rotor. Microstepping (MS pins -> 3V3)
 *  smooths the mid-band resonance a lot. Enabling USE_ESPNOW removes the
 *  blocking HTTP relay from the master's loop, which is the single biggest
 *  jitter source. An ESP32 (dual-core, RMT pulses) would relieve all of this.
 * ============================================================================
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266mDNS.h>
#include <AccelStepper.h>
#include <LittleFS.h>

/* ============================================================================
 *  CONFIGURATION — edit this block, nothing below needs touching
 * ==========================================================================*/

// ---- ROLE: leave defined on the MASTER board, comment out on the SLAVE ----
//#define ROLE_MASTER

// ---- OPTIONAL: ESP-NOW low-latency link for near-simultaneous moves -------
// 1) Flash both boards once with ESPNOW off, open Serial Monitor @115200,
//    write down each board's MAC address (printed at boot).
// 2) Set PEER_MAC below: on the MASTER build put the SLAVE's MAC,
//    on the SLAVE build put the MASTER's MAC.
// 3) Uncomment USE_ESPNOW and re-flash both.
// ESP-NOW frames ride on the current WiFi channel — both boards are on the
// same router so channels match automatically. If your router band-steers
// them apart, pin the router to one 2.4GHz channel.
// #define USE_ESPNOW
uint8_t PEER_MAC[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// ---- WiFi (your home network — both boards use the same) ------------------
// Credentials live in secrets.h (gitignored). Copy secrets.example.h to
// secrets.h and fill in your network.
#include "secrets.h"
const char* WIFI_SSID = SECRET_WIFI_SSID;
const char* WIFI_PASS = SECRET_WIFI_PASS;

// ---- Names / addressing ----------------------------------------------------
#define MDNS_NAME_MASTER "motorA"      // master reachable at http://motorA.local
#define MDNS_NAME_SLAVE  "motorB"      // slave  reachable at http://motorB.local
#define SLAVE_HOST       "motorB"      // hostname master resolves (no ".local")
const char* SLAVE_STATIC_IP = "";      // optional: "192.168.1.50" skips mDNS

// ---- Pins (both boards identical) ------------------------------------------
#define STEP_PIN  14   // D5
#define DIR_PIN    4   // D2
#define HAVE_EN_PIN 0  // 0 = no ENABLE wire (enable/disable is software-only).
                       // Set to 1 and wire EN to a spare GPIO for coil-off disable.
#define EN_PIN     5   // D1  (only used when HAVE_EN_PIN 1; active-low, LOW = enabled)
#define KILL_PIN  13   // D7  (INPUT_PULLUP; NC-to-GND = run, open = KILL)
#define LED_PIN   LED_BUILTIN   // heartbeat (NodeMCU LED is active-LOW)

// ---- Motion defaults --------------------------------------------------------
#define DEFAULT_PPR_DIV10  30      // pulses-per-rev / 10  =>  PPR = 300
                                   // editable live from the web UI, persisted
#define DEFAULT_SPEED     400.0f   // steps/sec
#define MIN_SPEED          10.0f
#define MAX_SPEED        2000.0f   // steps/sec (UI slider + % presets top out here)
#define DEFAULT_ACCEL     800.0f   // steps/sec^2
#define MIN_ACCEL          50.0f
#define MAX_ACCEL        5000.0f

// ---- Robustness timings -----------------------------------------------------
#define SLAVE_POLL_MS       3000   // master pings slave this often (reachability + dead-man)
#define SLAVE_HTTP_TIMEOUT   300   // ms; keep short — this blocks the master's loop
#define DEADMAN_MS          8000   // slave stops its motor if no master/UI contact this long
#define WIFI_STOP_MS        8000   // stop local motor if WiFi has been down this long
#define KILL_DEBOUNCE_MS      30

/* ============================================================================
 *  END OF CONFIGURATION
 * ==========================================================================*/

#ifdef USE_ESPNOW
#include <espnow.h>
#endif

AccelStepper     motor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
ESP8266WebServer server(80);

// ---- Motor state (identical logic on both boards) ---------------------------
enum Mode : uint8_t { M_IDLE = 0, M_VEL = 1, M_POS = 2 };
Mode   mode      = M_IDLE;
int    velDir    = 0;                 // -1 / 0 / +1 while in M_VEL
float  speedSet  = DEFAULT_SPEED;     // magnitude, steps/sec
float  accelSet  = DEFAULT_ACCEL;     // steps/sec^2
bool   enabled   = true;              // user intent for the EN pin
bool   killed    = false;             // kill switch latched state
int    pprDiv10  = DEFAULT_PPR_DIV10; // PPR = pprDiv10 * 10

const long VEL_HORIZON = 100000000L;  // "infinite" target used for velocity mode

// ---- Kill switch debounce ----
bool          killRaw       = false;
unsigned long killRawSince  = 0;

// ---- Misc bookkeeping ----
unsigned long wifiDownSince = 0;
unsigned long lastContactMs = 0;      // slave: last time master/UI touched us
bool          cfgDirty      = false;
unsigned long cfgDirtySince = 0;

#ifdef ROLE_MASTER
// ---- Master-only state ----
bool   mirrorMode = false;   // B mirrors A's every command, opposite direction
bool   linkSpeeds = false;   // speed set on A also applies to B (x gear)
float  gearRatio  = 1.0f;    // B runs at gearRatio x A's speed

struct Preset { char name[16]; float speed; int dir; bool used; };
Preset presets[4];

IPAddress     slaveIP;
bool          slaveOk        = false;
String        slaveCache     = "";    // last /m/status JSON from slave
unsigned long slaveCacheMs   = 0;
unsigned long lastSlavePoll  = 0;
unsigned long lastResolveTry = 0;
#endif

// ============================================================================
//  Unit conversions — PPR (= pprDiv10 * 10) is used for ALL deg/rev math
// ============================================================================
long  pprSteps()             { return (long)pprDiv10 * 10; }
long  degToSteps(float d)    { return lroundf(d * pprSteps() / 360.0f); }
long  revsToSteps(float r)   { return lroundf(r * pprSteps()); }
float stepsToDeg(long s)     { return s * 360.0f / pprSteps(); }

// ============================================================================
//  Motor primitives (shared by both roles — this is Motor A on the master,
//  Motor B on the slave)
// ============================================================================
void applyEnPin() {  // EN is active-low; only energize if enabled AND not killed
#if HAVE_EN_PIN
  digitalWrite(EN_PIN, (enabled && !killed) ? LOW : HIGH);
#endif
  // With no EN wire this is a no-op: `enabled` still gates motion in software
  // (setVel/moveRel/etc. refuse to move when disabled), but the coils stay
  // energized. `killed` still hard-stops regardless.
}

void hardStop() {    // instant halt: zero speed, target = here (position kept)
  motor.setCurrentPosition(motor.currentPosition());
  mode = M_IDLE; velDir = 0;
}

void softStop() {    // decelerate to a stop using current accel setting
  if (mode == M_IDLE) return;
  motor.stop();      // AccelStepper computes the decel target
  mode = M_POS; velDir = 0;
}

void setVel(int dir) {   // continuous rotation, ramps up with accelSet
  if (killed || !enabled) return;
  velDir = dir; mode = M_VEL;
  motor.setMaxSpeed(speedSet);
  motor.setAcceleration(accelSet);
  motor.moveTo(motor.currentPosition() + (long)dir * VEL_HORIZON);
}

void moveRel(long steps) {  // jog / run-N-revs: relative move, auto-stops
  if (killed || !enabled || steps == 0) return;
  velDir = 0; mode = M_POS;
  motor.setMaxSpeed(speedSet);
  motor.setAcceleration(accelSet);
  motor.move(steps);
}

void moveAbs(long steps) {  // go-to absolute position, auto-stops
  if (killed || !enabled) return;
  velDir = 0; mode = M_POS;
  motor.setMaxSpeed(speedSet);
  motor.setAcceleration(accelSet);
  motor.moveTo(steps);
}

void setSpeedVal(float v) {
  speedSet = constrain(v, MIN_SPEED, MAX_SPEED);
  motor.setMaxSpeed(speedSet);
}

void setAccelVal(float v) {
  accelSet = constrain(v, MIN_ACCEL, MAX_ACCEL);
  motor.setAcceleration(accelSet);
}

void zeroHere() {    // stops (setCurrentPosition kills speed) and zeroes counter
  hardStop();
  motor.setCurrentPosition(0);
}

void setEnabled(bool on) {
  enabled = on;
  if (!on) hardStop();  // de-energizing while moving would lose steps anyway
  applyEnPin();
}

// Called every loop() iteration — never blocks.
void serviceMotor() {
  if (killed) return;
  motor.run();
  if (mode == M_VEL && labs(motor.distanceToGo()) < VEL_HORIZON / 2)
    motor.moveTo(motor.currentPosition() + (long)velDir * VEL_HORIZON);  // extend horizon
  if (mode == M_POS && motor.distanceToGo() == 0)
    mode = M_IDLE;
}

// JSON snapshot of the LOCAL motor (A on master, B on slave)
String motorJson() {
  long  p  = motor.currentPosition();
  float sp = motor.speed();
  int   d  = (sp > 0.5f) ? 1 : (sp < -0.5f) ? -1 : 0;
  String j = "{";
  j += "\"pos\":"   + String(p);
  j += ",\"deg\":"  + String(stepsToDeg(p), 1);
  j += ",\"spd\":"  + String(fabsf(sp), 0);
  j += ",\"set\":"  + String(speedSet, 0);
  j += ",\"acc\":"  + String(accelSet, 0);
  j += ",\"mode\":" + String((int)mode);
  j += ",\"dir\":"  + String(d);
  j += ",\"en\":"   + String(enabled ? 1 : 0);
  j += ",\"killed\":" + String(killed ? 1 : 0);
  j += ",\"ppr\":"  + String(pprSteps());
  j += "}";
  return j;
}

// ============================================================================
//  Persistence (LittleFS) — pprDiv10 on both boards; gear/mirror/link/presets
//  additionally on the master. Saves are debounced to spare flash wear.
// ============================================================================
void saveCfg() {
  File f = LittleFS.open("/cfg", "w");
  if (!f) return;
  f.printf("ppr=%d\n", pprDiv10);
#ifdef ROLE_MASTER
  f.printf("gear=%.4f\nmirror=%d\nlinked=%d\n", gearRatio, mirrorMode ? 1 : 0, linkSpeeds ? 1 : 0);
  for (int i = 0; i < 4; i++)
    if (presets[i].used)
      f.printf("p%d=%s,%.0f,%d\n", i, presets[i].name, presets[i].speed, presets[i].dir);
#endif
  f.close();
  cfgDirty = false;
}

void markCfgDirty() { cfgDirty = true; cfgDirtySince = millis(); }

void loadCfg() {
  File f = LittleFS.open("/cfg", "r");
  if (!f) return;
  while (f.available()) {
    String line = f.readStringUntil('\n'); line.trim();
    int eq = line.indexOf('=');
    if (eq < 1) continue;
    String k = line.substring(0, eq), v = line.substring(eq + 1);
    if (k == "ppr")    pprDiv10 = constrain(v.toInt(), 1, 3200);
#ifdef ROLE_MASTER
    if (k == "gear")   gearRatio = v.toFloat();
    if (k == "mirror") mirrorMode = v.toInt() != 0;
    if (k == "linked") linkSpeeds = v.toInt() != 0;
    if (k.length() == 2 && k[0] == 'p') {
      int i = k[1] - '0';
      if (i >= 0 && i < 4) {
        int c1 = v.indexOf(','), c2 = v.lastIndexOf(',');
        if (c1 > 0 && c2 > c1) {
          v.substring(0, c1).toCharArray(presets[i].name, sizeof(presets[i].name));
          presets[i].speed = v.substring(c1 + 1, c2).toFloat();
          presets[i].dir   = v.substring(c2 + 1).toInt();
          presets[i].used  = true;
        }
      }
    }
#endif
  }
  f.close();
}

// ============================================================================
//  ESP-NOW (optional, compile-time) — tiny fixed command frames, ~1-2 ms
//  latency, no TCP/HTTP overhead. Used master->slave for motor-B commands so
//  same/opposite-direction moves start as close to simultaneously as possible.
// ============================================================================
enum NowCmd : uint8_t {
  N_FWD = 1, N_REV, N_STOP, N_ESTOP, N_SPEED, N_ACCEL,
  N_JOG, N_GOTO, N_ZERO, N_ENABLE, N_PING
};
struct NowMsg { uint8_t cmd; float v; };

#ifdef USE_ESPNOW
volatile bool   nowPending = false;
NowMsg          nowMsgBuf;

#ifndef ROLE_MASTER
// Received in WiFi-task context: just queue it, execute from loop().
void nowRecvCb(u8* mac, u8* data, u8 len) {
  (void)mac;
  if (len == sizeof(NowMsg)) { memcpy((void*)&nowMsgBuf, data, len); nowPending = true; }
}

void nowExecute() {
  if (!nowPending) return;
  NowMsg m; memcpy(&m, (const void*)&nowMsgBuf, sizeof(m)); nowPending = false;
  lastContactMs = millis();
  switch (m.cmd) {
    case N_FWD:    setVel(+1); break;
    case N_REV:    setVel(-1); break;
    case N_STOP:   softStop(); break;
    case N_ESTOP:  hardStop(); break;
    case N_SPEED:  setSpeedVal(m.v); break;
    case N_ACCEL:  setAccelVal(m.v); break;
    case N_JOG:    moveRel((long)m.v); break;
    case N_GOTO:   moveAbs((long)m.v); break;
    case N_ZERO:   zeroHere(); break;
    case N_ENABLE: setEnabled(m.v > 0.5f); break;
    case N_PING:   break;
  }
}
#endif

void nowInit() {
  if (esp_now_init() != 0) { Serial.println(F("ESP-NOW init failed")); return; }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_add_peer(PEER_MAC, ESP_NOW_ROLE_COMBO, 0 /*current channel*/, NULL, 0);
#ifndef ROLE_MASTER
  esp_now_register_recv_cb(nowRecvCb);
#endif
  Serial.println(F("ESP-NOW ready"));
}
#endif // USE_ESPNOW

#ifdef ROLE_MASTER
#ifdef USE_ESPNOW
bool nowSend(uint8_t cmd, float v = 0) {
  NowMsg m = { cmd, v };
  return esp_now_send(PEER_MAC, (uint8_t*)&m, sizeof(m)) == 0;
}
#endif

// ============================================================================
//  MASTER: talking to the slave
// ============================================================================
void resolveSlave() {
  if (strlen(SLAVE_STATIC_IP)) { slaveIP.fromString(SLAVE_STATIC_IP); return; }
  if (slaveIP.isSet() && slaveOk) return;
  if (millis() - lastResolveTry < 10000) return;     // don't hammer mDNS
  lastResolveTry = millis();
  // ESP8266 LEAmDNS has no queryHost(); browse HTTP services and pick the one
  // whose hostname matches SLAVE_HOST. Blocks <=500ms, only while unresolved.
  String want = String(SLAVE_HOST); want.toLowerCase();
  uint32_t n = MDNS.queryService("http", "tcp", 500);
  for (uint32_t i = 0; i < n; i++) {
    String hn = MDNS.hostname(i); hn.toLowerCase();
    if (hn.startsWith(want)) {
      slaveIP = MDNS.answerIP(i);
      Serial.print(F("Slave resolved: ")); Serial.println(slaveIP);
      break;
    }
  }
}

// Blocking HTTP GET to the slave. Kept short (SLAVE_HTTP_TIMEOUT) because it
// stalls step generation on Motor A — this is why USE_ESPNOW is recommended.
String slaveGet(const String& pathAndQuery) {
  if (!slaveIP.isSet()) { slaveOk = false; return ""; }
  WiFiClient c; HTTPClient h;
  h.setTimeout(SLAVE_HTTP_TIMEOUT);
  if (!h.begin(c, "http://" + slaveIP.toString() + pathAndQuery)) { slaveOk = false; return ""; }
  int code = h.GET();
  String body = (code == 200) ? h.getString() : "";
  h.end();
  slaveOk = (code == 200);
  return body;
}

// High-level "send simple command to Motor B" — ESP-NOW when compiled in
// (falls back to HTTP if the send fails), plain HTTP otherwise.
void bCmd(uint8_t nowCmd, float v, const String& httpPathQuery) {
#ifdef USE_ESPNOW
  if (nowSend(nowCmd, v)) return;
#else
  (void)nowCmd; (void)v;
#endif
  slaveGet(httpPathQuery);
}

void bFwd()            { bCmd(N_FWD,   0, "/m/fwd"); }
void bRev()            { bCmd(N_REV,   0, "/m/rev"); }
void bStop()           { bCmd(N_STOP,  0, "/m/stop"); }
void bEstop()          { bCmd(N_ESTOP, 0, "/m/estop"); }
void bZero()           { bCmd(N_ZERO,  0, "/m/zero"); }
void bSpeed(float v)   { bCmd(N_SPEED, v, "/m/speed?v="  + String(v, 0)); }
void bAccel(float v)   { bCmd(N_ACCEL, v, "/m/accel?v="  + String(v, 0)); }
void bJog(long s)      { bCmd(N_JOG,   s, "/m/jog?steps=" + String(s)); }
void bGoto(long s)     { bCmd(N_GOTO,  s, "/m/goto?steps=" + String(s)); }
void bEnable(bool on)  { bCmd(N_ENABLE, on ? 1 : 0, String("/m/enable?on=") + (on ? "1" : "0")); }

float gearedSpeed(float aSpeed) {
  return constrain(fabsf(aSpeed * gearRatio), MIN_SPEED, MAX_SPEED);
}

void stopAll() { bEstop(); hardStop(); }   // slave first — minimizes skew
#endif // ROLE_MASTER

// ============================================================================
//  HTTP endpoint plumbing
// ============================================================================
void sendPlain(const String& s) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", s);
}
void sendJson(const String& s) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", s);
}

// Reads a move amount from steps= / deg= / revs= (deg & revs use PPR math).
bool argSteps(long& out) {
  if      (server.hasArg("steps")) out = server.arg("steps").toInt();
  else if (server.hasArg("deg"))   out = degToSteps(server.arg("deg").toFloat());
  else if (server.hasArg("revs"))  out = revsToSteps(server.arg("revs").toFloat());
  else return false;
  if (server.hasArg("d") && server.arg("d").toInt() < 0) out = -out;
  return true;
}

// Executes a command on the LOCAL motor. Returns the HTTP response body,
// or "" for unknown commands. Shared verbatim by master (/a/*) and slave (/m/*).
String execLocal(const String& cmd) {
  long n;
  if (cmd == "fwd")    { setVel(+1);  return "fwd"; }
  if (cmd == "rev")    { setVel(-1);  return "rev"; }
  if (cmd == "stop")   { softStop();  return "stop"; }
  if (cmd == "estop")  { hardStop();  return "estop"; }
  if (cmd == "zero")   { zeroHere();  return "zeroed"; }
  if (cmd == "speed")  { if (server.hasArg("v")) setSpeedVal(server.arg("v").toFloat()); return String(speedSet, 0); }
  if (cmd == "accel")  { if (server.hasArg("v")) setAccelVal(server.arg("v").toFloat()); return String(accelSet, 0); }
  if (cmd == "jog")    { if (argSteps(n)) moveRel(n);  return "jog"; }
  if (cmd == "goto")   { if (argSteps(n)) moveAbs(n);  return "goto"; }
  if (cmd == "runrevs"){
    float r = server.hasArg("n") ? server.arg("n").toFloat() : 0;
    int   d = (server.hasArg("d") && server.arg("d").toInt() < 0) ? -1 : 1;
    moveRel(revsToSteps(r) * d);
    return "runrevs";
  }
  if (cmd == "enable") { setEnabled(server.hasArg("on") ? server.arg("on").toInt() != 0 : true); return enabled ? "enabled" : "disabled"; }
  if (cmd == "status") { return motorJson(); }
  if (cmd == "ping")   { return "pong"; }
  return "";
}

#ifdef ROLE_MASTER
// Mirror mode: every command applied to A is repeated on B, inverted.
// (Speed/accel are copied — speed through the gear ratio.)
void mirrorToB(const String& cmd) {
  long n;
  if      (cmd == "fwd")    bRev();
  else if (cmd == "rev")    bFwd();
  else if (cmd == "stop")   bStop();
  else if (cmd == "estop")  bEstop();
  else if (cmd == "zero")   bZero();
  else if (cmd == "speed")  bSpeed(gearedSpeed(speedSet));
  else if (cmd == "accel")  bAccel(accelSet);
  else if (cmd == "jog")    { if (argSteps(n)) bJog(-n); }
  else if (cmd == "goto")   { if (argSteps(n)) bGoto(-n); }
  else if (cmd == "runrevs") {
    float r = server.hasArg("n") ? server.arg("n").toFloat() : 0;
    int   d = (server.hasArg("d") && server.arg("d").toInt() < 0) ? -1 : 1;
    bJog(-revsToSteps(r) * d);
  }
  else if (cmd == "enable") bEnable(server.hasArg("on") ? server.arg("on").toInt() != 0 : true);
}

// Rebuild "?k=v&k2=v2" from parsed args (for raw /b/* HTTP forwarding).
String rebuildQuery() {
  String q = "";
  for (int i = 0; i < server.args(); i++) {
    if (server.argName(i) == "plain") continue;
    q += (q.length() ? "&" : "?") + server.argName(i) + "=" + server.arg(i);
  }
  return q;
}

// Forward a /b/<cmd> request to the slave. Converts to ESP-NOW when possible.
String execRemoteB(const String& cmd) {
  long n;
#ifdef USE_ESPNOW
  if (cmd == "fwd")    { bFwd();   return "fwd"; }
  if (cmd == "rev")    { bRev();   return "rev"; }
  if (cmd == "stop")   { bStop();  return "stop"; }
  if (cmd == "estop")  { bEstop(); return "estop"; }
  if (cmd == "zero")   { bZero();  return "zeroed"; }
  if (cmd == "speed" && server.hasArg("v")) { bSpeed(server.arg("v").toFloat()); return "ok"; }
  if (cmd == "accel" && server.hasArg("v")) { bAccel(server.arg("v").toFloat()); return "ok"; }
  if (cmd == "jog"  && argSteps(n)) { bJog(n);  return "jog"; }
  if (cmd == "goto" && argSteps(n)) { bGoto(n); return "goto"; }
  if (cmd == "runrevs") {
    float r = server.hasArg("n") ? server.arg("n").toFloat() : 0;
    int   d = (server.hasArg("d") && server.arg("d").toInt() < 0) ? -1 : 1;
    bJog(revsToSteps(r) * d);  return "runrevs";
  }
  if (cmd == "enable") { bEnable(server.hasArg("on") ? server.arg("on").toInt() != 0 : true); return "ok"; }
#endif
  if (cmd == "status" && slaveCache.length() && millis() - slaveCacheMs < 1500)
    return slaveCache;                                   // fresh cached copy
  String r = slaveGet("/m/" + cmd + rebuildQuery());     // plain HTTP relay
  if (cmd == "status" && r.length()) { slaveCache = r; slaveCacheMs = millis(); }
  return r.length() ? r : "slave unreachable";
}
#endif // ROLE_MASTER

// Router for all prefixed motor endpoints (registered via onNotFound so the
// same handler serves /a/*, /b/* and /m/* without duplicated route tables).
void routeMotor() {
  String uri = server.uri();
  lastContactMs = millis();          // any HTTP contact feeds the dead-man timer
  String out = "";
#ifdef ROLE_MASTER
  if (uri.startsWith("/a/")) {
    String cmd = uri.substring(3);
    out = execLocal(cmd);
    if (out.length() && mirrorMode && cmd != "status" && cmd != "ping") mirrorToB(cmd);
    if (out.length() && linkSpeeds && !mirrorMode && cmd == "speed") bSpeed(gearedSpeed(speedSet));
  } else if (uri.startsWith("/b/")) {
    out = execRemoteB(uri.substring(3));
  }
#else
  if (uri.startsWith("/m/")) out = execLocal(uri.substring(3));
#endif
  if (!out.length()) { server.send(404, "text/plain", "unknown endpoint"); return; }
  if (out[0] == '{') sendJson(out); else sendPlain(out);
}

// ============================================================================
//  Web UI (master only) — self-contained page, no external libs.
//  Style matches the original sketch family: clean sans-serif, light page,
//  big rounded buttons (green fwd / blue rev / red stop), live speed sliders.
// ============================================================================
#ifdef ROLE_MASTER
static const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html><head><meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no'>
<title>Dual Stepper Control</title><style>
:root{--fwd:#2e7d32;--rev:#1565c0;--stop:#b71c1c;--line:#dfe3e8;--mut:#667085}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{font-family:system-ui,-apple-system,'Segoe UI',Roboto,sans-serif;background:#f5f6f8;color:#1d2530;margin:0 auto;padding:12px;max-width:540px}
h1{font-size:20px;font-weight:650;text-align:center;margin:6px 0 12px}
h2{font-size:15px;font-weight:650;margin:0 0 10px}
.card{background:#fff;border:1px solid var(--line);border-radius:16px;padding:16px;margin-bottom:12px;box-shadow:0 1px 3px rgba(16,24,40,.06)}
button{font-size:15px;font-weight:600;padding:14px 8px;border:0;border-radius:12px;color:#fff;background:#546e7a;transition:transform .05s,filter .15s;cursor:pointer}
button:active{transform:scale(.97)}
.fwd{background:var(--fwd)}.rev{background:var(--rev)}.stop{background:var(--stop)}
.sm{padding:11px 14px;font-size:14px}
.row{display:flex;gap:8px;margin-bottom:8px}
.row>button{flex:1}.row>input,.row>select{flex:1;min-width:0}
.estop{position:sticky;top:8px;z-index:50;width:100%;font-size:17px;padding:16px;background:var(--stop);box-shadow:0 3px 10px rgba(183,28,28,.4);margin-bottom:12px}
.banner{border-radius:12px;padding:10px 14px;font-size:14px;font-weight:600;margin-bottom:10px;display:none}
.bkill{background:var(--stop);color:#fff}.bwarn{background:#fff3cd;color:#7a5c00;border:1px solid #ffe08a}
.lab{display:flex;justify-content:space-between;font-size:13px;color:var(--mut);margin:10px 0 6px}
.lab b{color:#1d2530;font-size:14px}
input[type=range]{-webkit-appearance:none;width:100%;height:8px;border-radius:8px;background:var(--line);outline:none}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:26px;height:26px;border-radius:50%;background:#1d2530;border:3px solid #fff;box-shadow:0 1px 4px rgba(0,0,0,.3)}
input[type=number],input[type=text],select{background:#f5f6f8;border:1px solid var(--line);color:#1d2530;border-radius:10px;padding:11px;font-size:15px;text-align:center}
.chip{display:inline-block;font-size:11.5px;font-weight:700;border-radius:99px;padding:3px 10px;background:#eef1f4;color:var(--mut);vertical-align:middle}
.chip.on{background:#e3f2e6;color:var(--fwd)}.chip.run{background:#e3ecf9;color:var(--rev)}.chip.bad{background:#fbe4e4;color:var(--stop)}
.tele{font-size:13px;color:var(--mut);margin:4px 0 10px}.tele b{color:#1d2530}
.tog{display:flex;align-items:center;gap:8px;font-size:14px;font-weight:600;flex:1}
.tog input{width:20px;height:20px}
.phead{display:flex;align-items:center;gap:8px;margin-bottom:2px}.phead b{font-size:16px}
.sub{font-size:12px;color:var(--mut);text-align:center;margin-top:4px}
</style></head><body>
<h1>Dual Stepper Control</h1>
<button class='estop' onclick="api('/stopall')">&#9632; EMERGENCY STOP — BOTH MOTORS</button>
<div id='bkill' class='banner bkill'>&#9888; KILLED — kill switch open</div>
<div id='bslave' class='banner bwarn'>&#9888; Motor B (slave) unreachable — check its power / WiFi</div>

<!-- ============ ZONE 1: global / combined ============ -->
<div class='card'><h2>Both Motors</h2>
<div class='row'>
<button class='fwd' onclick="api('/both/fwd')">&#9650; Both Fwd</button>
<button class='rev' onclick="api('/both/rev')">&#9660; Both Rev</button>
</div>
<div class='row'>
<button onclick="api('/both/opposite?swap='+($('swap').checked?1:0))">&#8646; Spin Opposite</button>
<label class='tog' style='flex:0 0 auto'><input id='swap' type='checkbox'>swap</label>
</div>
<button class='stop' style='width:100%' onclick="api('/stopall')">&#9632; BOTH STOP</button>
<div class='lab'><span>Global speed (drives both)</span><b><span id='gsv'>400</span> st/s</b></div>
<input id='gspd' type='range' min='10' max='2000' value='400'
 oninput="$('gsv').innerText=this.value" onchange="api('/both/speed?v='+this.value)">
<div class='row' style='margin-top:12px'>
<label class='tog'><input id='linked' type='checkbox' onchange="api('/config?linked='+(this.checked?1:0))">Link speeds</label>
<label class='tog'><input id='mirror' type='checkbox' onchange="api('/config?mirror='+(this.checked?1:0))">Mirror mode</label>
</div>
<div class='lab'><span>Gear ratio (B = ratio &times; A speed)</span></div>
<div class='row'>
<input id='gear' type='number' step='0.05' value='1.0'>
<button class='sm' onclick="api('/config?gear='+$('gear').value)">Apply</button>
</div>
<div class='lab'><span>Sync move (fires A + B together)</span></div>
<div class='row'>
<select id='synca'><option value='fwd'>A: fwd</option><option value='rev'>A: rev</option><option value='stop'>A: stop</option></select>
<select id='syncb'><option value='fwd'>B: fwd</option><option value='rev'>B: rev</option><option value='stop'>B: stop</option></select>
<button class='sm' onclick="api('/both/sync?a='+$('synca').value+'&b='+$('syncb').value)">GO</button>
</div>
</div>

<!-- ============ ZONE 2: per-motor panels (built by JS) ============ -->
<div id='panels'></div>

<!-- ============ ZONE 3: presets & utilities ============ -->
<div class='card'><h2>Presets</h2>
<div id='presets'></div>
<div class='lab'><span>Quick speed (% of max, both motors)</span></div>
<div class='row'>
<button class='sm' onclick="pct(25)">25%</button><button class='sm' onclick="pct(50)">50%</button>
<button class='sm' onclick="pct(75)">75%</button><button class='sm' onclick="pct(100)">100%</button>
</div>
</div>

<div class='card'><h2>Setup &amp; Status</h2>
<div class='lab'><span>PPR &divide; 10 (pulses/rev = value &times; 10)</span><b><span id='pprv'>—</span> ppr</b></div>
<div class='row'>
<input id='ppr' type='number' value='30' min='1'>
<button class='sm' onclick="api('/config?pprdiv10='+$('ppr').value)">Apply</button>
</div>
<div class='tele' style='margin-top:10px'>
Master: <span class='chip' id='cmast'>—</span> &nbsp; Slave: <span class='chip' id='cslav'>—</span>
</div>
<p class='sub' id='ipline'></p>
</div>

<script>
const $=id=>document.getElementById(id);
let slaveip='',bDirect=false,bFail=9,MAXS=2000;
function api(p){fetch(p).catch(()=>{})}

function panel(id,name){return `
<div class='card'>
<div class='phead'><b>${name}</b> <span class='chip' id='${id}state'>—</span> <span class='chip' id='${id}enc'>—</span></div>
<div class='tele' id='${id}tele'>—</div>
<div class='row'>
<button class='fwd' onclick="api('/${id}/fwd')">&#9650; Fwd</button>
<button class='rev' onclick="api('/${id}/rev')">&#9660; Rev</button>
<button class='stop' onclick="api('/${id}/stop')">&#9632; Stop</button>
</div>
<div class='lab'><span>Speed</span><b><span id='${id}sv'>—</span> st/s</b></div>
<input type='range' id='${id}spd' min='10' max='2000' value='400'
 oninput="$('${id}sv').innerText=this.value" onchange="api('/${id}/speed?v='+this.value)">
<div class='lab'><span>Acceleration</span><b><span id='${id}av'>—</span> st/s&sup2;</b></div>
<input type='range' id='${id}acc' min='50' max='5000' value='800'
 oninput="$('${id}av').innerText=this.value" onchange="api('/${id}/accel?v='+this.value)">
<div class='lab'><span>Jog (&plusmn; amount)</span></div>
<div class='row'>
<button class='sm' onclick="jog('${id}',-1)">&minus;</button>
<input id='${id}jn' type='number' value='1' step='any'>
<select id='${id}ju'><option value='revs'>revs</option><option value='deg'>deg</option><option value='steps'>steps</option></select>
<button class='sm' onclick="jog('${id}',1)">+</button>
</div>
<div class='lab'><span>Go to absolute position</span></div>
<div class='row'>
<input id='${id}gn' type='number' value='0' step='any'>
<select id='${id}gu'><option value='steps'>steps</option><option value='deg'>deg</option><option value='revs'>revs</option></select>
<button class='sm' onclick="gopos('${id}')">GO</button>
<button class='sm' onclick="api('/${id}/zero')">ZERO</button>
</div>
<div class='lab'><span>Run N revolutions, then stop</span></div>
<div class='row'>
<input id='${id}rn' type='number' value='5' step='any'>
<select id='${id}rd'><option value='1'>fwd</option><option value='-1'>rev</option></select>
<button class='sm' onclick="api('/${id}/runrevs?n='+$('${id}rn').value+'&d='+$('${id}rd').value)">GO</button>
</div>
<button id='${id}enb' style='width:100%;margin-top:10px' onclick="togEn('${id}')">Enable / Disable</button>
</div>`}
$('panels').innerHTML=panel('a','Motor A (master)')+panel('b','Motor B (slave)');

let ph='';for(let i=0;i<4;i++)ph+=`<div class='row'>
<input id='pn${i}' type='text' placeholder='preset ${i+1}' style='flex:2;text-align:left'>
<button class='sm' onclick="api('/preset/save?slot=${i}&name='+encodeURIComponent($('pn${i}').value||'preset${i+1}'))">Save</button>
<button class='sm fwd' onclick="api('/preset/recall?slot=${i}')">Recall</button></div>`;
$('presets').innerHTML=ph;

function jog(id,s){const v=$(''+id+'jn').value*s;api('/'+id+'/jog?'+$(''+id+'ju').value+'='+v)}
function gopos(id){api('/'+id+'/goto?'+$(''+id+'gu').value+'='+$(''+id+'gn').value)}
function pct(p){const v=Math.round(MAXS*p/100);$('gspd').value=v;$('gsv').innerText=v;api('/both/speed?v='+v)}
const enState={a:1,b:1};
function togEn(id){api('/'+id+'/enable?on='+(enState[id]?0:1))}

function render(m,id){
 if(!m)return;
 enState[id]=m.en;
 const st=m.killed?'KILLED':m.mode==1?(m.dir>0?'FWD':'REV'):m.mode==2?'MOVING':'STOPPED';
 const c=$(''+id+'state');c.innerText=st;
 c.className='chip'+(m.killed?' bad':m.mode?' run':'');
 const e=$(''+id+'enc');e.innerText=m.en?'ENABLED':'DISABLED';e.className='chip'+(m.en?' on':' bad');
 $(''+id+'tele').innerHTML=`pos <b>${m.pos}</b> st (<b>${m.deg}&deg;</b>) &middot; spd <b>${m.spd}</b> st/s &middot; accel <b>${m.acc}</b>`;
 if(document.activeElement!==$(''+id+'spd')){$(''+id+'spd').value=m.set;$(''+id+'sv').innerText=m.set}
 if(document.activeElement!==$(''+id+'acc')){$(''+id+'acc').value=m.acc;$(''+id+'av').innerText=m.acc}
}

function poll(){fetch('/status').then(r=>r.json()).then(d=>{
 MAXS=d.max;slaveip=d.slaveip||'';
 $('bkill').style.display=(d.a&&d.a.killed)||(d.b&&d.b.killed)?'block':'none';
 $('bslave').style.display=(!d.slaveok&&!bDirect)?'block':'none';
 if(document.activeElement!==$('gear'))$('gear').value=d.gear;
 if(document.activeElement!==$('ppr'))$('ppr').value=d.pprdiv10;
 $('pprv').innerText=d.ppr;$('mirror').checked=!!d.mirror;$('linked').checked=!!d.linked;
 for(let i=0;i<4;i++)if(d.presets[i]&&document.activeElement!==$('pn'+i))$('pn'+i).value=d.presets[i];
 $('cmast').innerText='OK '+d.ip;$('cmast').className='chip on';
 $('cslav').innerText=(d.slaveok||bDirect)?('OK '+slaveip):'UNREACHABLE';
 $('cslav').className='chip '+((d.slaveok||bDirect)?'on':'bad');
 $('ipline').innerText='master '+d.ip+' (motorA.local) · RSSI '+d.rssi+' dBm';
 render(d.a,'a');
 if(!bDirect&&d.b)render(d.b,'b');
}).catch(()=>{$('cmast').innerText='NO LINK';$('cmast').className='chip bad'})}

/* The browser polls the slave DIRECTLY (CORS enabled) so Motor B telemetry
   stays live without the master having to relay status over HTTP. */
function pollB(){if(!slaveip)return;
 fetch('http://'+slaveip+'/m/status').then(r=>r.json()).then(d=>{bDirect=true;bFail=0;render(d,'b')})
 .catch(()=>{if(++bFail>3)bDirect=false})}

setInterval(poll,500);setInterval(pollB,500);poll();
</script></body></html>)HTML";

// ---- master /status: motor A + flags + cached motor B ----
String masterStatusJson() {
  String j = "{\"role\":\"master\"";
  j += ",\"ip\":\"" + WiFi.localIP().toString() + "\"";
  j += ",\"rssi\":" + String(WiFi.RSSI());
  j += ",\"mirror\":" + String(mirrorMode ? 1 : 0);
  j += ",\"linked\":" + String(linkSpeeds ? 1 : 0);
  j += ",\"gear\":" + String(gearRatio, 3);
  j += ",\"pprdiv10\":" + String(pprDiv10);
  j += ",\"ppr\":" + String(pprSteps());
  j += ",\"max\":" + String((int)MAX_SPEED);
  j += ",\"slaveok\":" + String(slaveOk ? 1 : 0);
  j += ",\"slaveip\":\"" + (slaveIP.isSet() ? slaveIP.toString() : "") + "\"";
  j += ",\"presets\":[";
  for (int i = 0; i < 4; i++) {
    if (i) j += ",";
    j += presets[i].used ? ("\"" + String(presets[i].name) + "\"") : "\"\"";
  }
  j += "]";
  j += ",\"a\":" + motorJson();
  j += ",\"b\":" + (slaveCache.length() ? slaveCache : "null");
  j += ",\"bage\":" + String(slaveCache.length() ? millis() - slaveCacheMs : 0);
  j += "}";
  return j;
}

void handleConfig() {
  if (server.hasArg("pprdiv10")) {
    pprDiv10 = constrain(server.arg("pprdiv10").toInt(), 1, 3200);
    slaveGet("/m/config?pprdiv10=" + String(pprDiv10));   // keep slave's math in sync
    markCfgDirty();
  }
  if (server.hasArg("gear"))   { gearRatio  = constrain(server.arg("gear").toFloat(), 0.01f, 100.0f); markCfgDirty(); }
  if (server.hasArg("mirror")) { mirrorMode = server.arg("mirror").toInt() != 0; markCfgDirty(); }
  if (server.hasArg("linked")) { linkSpeeds = server.arg("linked").toInt() != 0; markCfgDirty(); }
  sendJson(masterStatusJson());
}

void handleBoth(const String& what) {
  // Slave command goes FIRST (ESP-NOW ~1-2ms / HTTP ~20-50ms), then A locally —
  // this ordering keeps the two starts as close to simultaneous as possible.
  if (what == "fwd")  { bFwd();  setVel(+1); sendPlain("both fwd");  return; }
  if (what == "rev")  { bRev();  setVel(-1); sendPlain("both rev");  return; }
  if (what == "stop") { stopAll();           sendPlain("both stop"); return; }
  if (what == "opposite") {
    bool swap = server.hasArg("swap") && server.arg("swap").toInt() != 0;
    if (swap) { bFwd(); setVel(-1); } else { bRev(); setVel(+1); }
    sendPlain("opposite");
    return;
  }
  if (what == "speed") {
    if (server.hasArg("v")) {
      float v = server.arg("v").toFloat();
      bSpeed(gearedSpeed(v));
      setSpeedVal(v);
    }
    sendPlain("both speed");
    return;
  }
  if (what == "sync") {   // /both/sync?a=fwd&b=rev — fire together
    String ca = server.arg("a"), cb = server.arg("b");
    if      (cb == "fwd") bFwd(); else if (cb == "rev") bRev(); else if (cb == "stop") bStop();
    if      (ca == "fwd") setVel(+1); else if (ca == "rev") setVel(-1); else if (ca == "stop") softStop();
    sendPlain("sync");
    return;
  }
  server.send(404, "text/plain", "unknown /both command");
}

String sanitizeName(String s) {   // keep the hand-built JSON/cfg file safe
  s.replace("\"", ""); s.replace(",", ""); s.replace("=", ""); s.replace("\n", "");
  if (s.length() > 15) s = s.substring(0, 15);
  return s;
}
#endif // ROLE_MASTER

// ============================================================================
//  Kill switch + LED + WiFi supervision
// ============================================================================
void serviceKill() {
  bool raw = (digitalRead(KILL_PIN) == HIGH);   // pull-up: open contact reads HIGH = KILL
  if (raw != killRaw) { killRaw = raw; killRawSince = millis(); }
  if (millis() - killRawSince < KILL_DEBOUNCE_MS) return;

  if (killRaw && !killed) {          // switch just opened -> KILL
    killed = true;
    hardStop();
    applyEnPin();                    // de-energize coils
#ifdef ROLE_MASTER
    bEstop(); bEnable(false);        // take the slave down with us
#endif
    Serial.println(F("KILL engaged"));
  } else if (!killRaw && killed) {   // switch closed again -> ready (motor stays stopped)
    killed = false;
    applyEnPin();
#ifdef ROLE_MASTER
    bEnable(true);
#endif
    Serial.println(F("KILL released"));
  }
}

void serviceLed() {
  unsigned long now = millis();
  bool on;
  if      (killed)                            on = (now / 120) & 1;   // fast blink = killed
  else if (WiFi.status() != WL_CONNECTED)     on = (now / 400) & 1;   // medium blink = no WiFi
  else                                        on = (now % 1400) < 60; // short flash = alive
  digitalWrite(LED_PIN, on ? LOW : HIGH);     // NodeMCU LED is active-low
}

void serviceWifi() {
  if (WiFi.status() == WL_CONNECTED) { wifiDownSince = 0; return; }
  if (!wifiDownSince) wifiDownSince = millis();
  else if (millis() - wifiDownSince > WIFI_STOP_MS && mode != M_IDLE) {
    softStop();                      // don't keep spinning with nobody in control
    Serial.println(F("WiFi lost -> motor stopped"));
  }
  // WiFi.setAutoReconnect(true) (set in setup) handles the actual reconnect.
}

// ============================================================================
//  Setup
// ============================================================================
void setup() {
#if HAVE_EN_PIN
  pinMode(EN_PIN, OUTPUT);  digitalWrite(EN_PIN, HIGH);  // coils OFF until we're up
#endif
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(KILL_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(115200);
  Serial.println();
#ifdef ROLE_MASTER
  Serial.println(F("=== DUAL STEPPER — MASTER (Motor A) ==="));
#else
  Serial.println(F("=== DUAL STEPPER — SLAVE (Motor B) ==="));
#endif
  Serial.print(F("MAC address (for ESP-NOW PEER_MAC on the other board): "));
  Serial.println(WiFi.macAddress());

  LittleFS.begin();
  loadCfg();

  motor.setMaxSpeed(speedSet);
  motor.setAcceleration(accelSet);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
#ifdef ROLE_MASTER
  WiFi.hostname(MDNS_NAME_MASTER);
#else
  WiFi.hostname(MDNS_NAME_SLAVE);
#endif
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print(F("Joining WiFi"));
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    digitalWrite(LED_PIN, (millis() / 100) & 1);   // frantic blink while joining
    delay(50);                                     // setup() only — loop() never delays
    if ((millis() / 500) % 2 == 0) Serial.print('.');
  }
  Serial.println();
  Serial.print(F("IP: ")); Serial.println(WiFi.localIP());

#ifdef ROLE_MASTER
  if (MDNS.begin(MDNS_NAME_MASTER)) Serial.println(F("mDNS: http://" MDNS_NAME_MASTER ".local"));
#else
  if (MDNS.begin(MDNS_NAME_SLAVE))  Serial.println(F("mDNS: http://" MDNS_NAME_SLAVE ".local"));
#endif
  MDNS.addService("http", "tcp", 80);

#ifdef USE_ESPNOW
  nowInit();
#endif

  // ---- routes ----
#ifdef ROLE_MASTER
  server.on("/", []() {
    server.send_P(200, "text/html", PAGE);
  });
  server.on("/status",  []() { lastContactMs = millis(); sendJson(masterStatusJson()); });
  server.on("/config",  []() { handleConfig(); });
  server.on("/stopall", []() { stopAll(); sendPlain("stopped all"); });
  server.on("/both/fwd",      []() { handleBoth("fwd"); });
  server.on("/both/rev",      []() { handleBoth("rev"); });
  server.on("/both/stop",     []() { handleBoth("stop"); });
  server.on("/both/opposite", []() { handleBoth("opposite"); });
  server.on("/both/speed",    []() { handleBoth("speed"); });
  server.on("/both/sync",     []() { handleBoth("sync"); });
  server.on("/preset/save", []() {
    int i = server.arg("slot").toInt();
    if (i >= 0 && i < 4) {
      sanitizeName(server.arg("name")).toCharArray(presets[i].name, sizeof(presets[i].name));
      presets[i].speed = speedSet;                 // capture Motor A's current speed
      presets[i].dir   = (mode == M_VEL) ? velDir : 0;   // ...and direction
      presets[i].used  = true;
      markCfgDirty();
    }
    sendPlain("saved");
  });
  server.on("/preset/recall", []() {
    int i = server.arg("slot").toInt();
    if (i >= 0 && i < 4 && presets[i].used) {
      setSpeedVal(presets[i].speed);
      bSpeed(gearedSpeed(presets[i].speed));
      if      (presets[i].dir > 0) { bFwd();  setVel(+1); }
      else if (presets[i].dir < 0) { bRev();  setVel(-1); }
      else                         { bStop(); softStop(); }
    }
    sendPlain("recalled");
  });
#else
  server.on("/", []() {
    sendPlain("This is the SLAVE (Motor B). The control page lives on the master: http://" MDNS_NAME_MASTER ".local");
  });
  server.on("/status", []() { lastContactMs = millis(); sendJson(motorJson()); });
  server.on("/config", []() {
    lastContactMs = millis();
    if (server.hasArg("pprdiv10")) { pprDiv10 = constrain(server.arg("pprdiv10").toInt(), 1, 3200); markCfgDirty(); }
    sendJson(motorJson());
  });
#endif
  server.onNotFound(routeMotor);   // /a/* /b/* (master) or /m/* (slave)
  server.begin();

  lastContactMs = millis();
  if (WiFi.status() == WL_CONNECTED && !killRaw) { enabled = true; applyEnPin(); }
  serviceKill();   // pick up the switch state immediately
  Serial.println(F("Ready."));
}

// ============================================================================
//  Main loop — strictly non-blocking. Every iteration services: step engine,
//  web server, mDNS, kill switch, LED, WiFi watchdog, (master) slave poll,
//  (slave) dead-man timer, deferred config saves.
// ============================================================================
void loop() {
  serviceMotor();
  server.handleClient();
  MDNS.update();
  serviceKill();
  serviceLed();
  serviceWifi();

#ifdef USE_ESPNOW
#ifndef ROLE_MASTER
  nowExecute();          // run any queued ESP-NOW command in loop context
#endif
#endif

#ifdef ROLE_MASTER
  // Periodic slave poll: reachability + fresh Motor B telemetry + feeds the
  // slave's dead-man timer. NOTE: this HTTP call blocks up to
  // SLAVE_HTTP_TIMEOUT ms, which can hiccup Motor A's step train at high
  // speed — the UI's direct browser->slave polling keeps this rare, and
  // USE_ESPNOW replaces the command path entirely.
  if (millis() - lastSlavePoll > SLAVE_POLL_MS) {
    lastSlavePoll = millis();
    resolveSlave();
    if (slaveIP.isSet()) {
      String r = slaveGet("/m/status");
      if (r.length()) { slaveCache = r; slaveCacheMs = millis(); }
    }
  }
#else
  // Dead-man: if nothing (master ping, UI poll, ESP-NOW) has touched us for
  // DEADMAN_MS while the motor is moving, stop on our own.
  if (mode != M_IDLE && millis() - lastContactMs > DEADMAN_MS) {
    softStop();
    Serial.println(F("Dead-man timeout -> motor stopped"));
  }
#endif

  if (cfgDirty && millis() - cfgDirtySince > 2000) saveCfg();   // debounced flash write
}
