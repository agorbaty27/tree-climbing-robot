# Dual Stepper WiFi Control (2 × ESP8266 + A4988/BT7221A)

One sketch (`dual_stepper_wifi.ino`), two NodeMCU boards. The **MASTER** hosts the
mobile web UI, drives **Motor A**, and relays commands to the **SLAVE**, which drives
**Motor B**. Both boards join your home WiFi.

## Flashing each board

Arduino IDE → board **"NodeMCU 1.0 (ESP-12E)"** (esp8266 core installed via Boards
Manager, URL `https://arduino.esp8266.com/stable/package_esp8266com_index.json`).
Library needed: **AccelStepper** by Mike McCauley (Library Manager). Everything else
(ESP8266WiFi, ESP8266WebServer, ESP8266HTTPClient, ESP8266mDNS, LittleFS, espnow)
ships with the ESP8266 core.

Edit only the **CONFIGURATION** block at the top of the sketch:

| Line | Master board | Slave board |
|---|---|---|
| `#define ROLE_MASTER` | leave as-is | **comment out** |
| `SECRET_WIFI_SSID` / `SECRET_WIFI_PASS` in `secrets.h` (copy from `secrets.example.h`) | your WiFi | same |
| `SLAVE_STATIC_IP` | optional; only if mDNS is flaky | ignored |
| `PEER_MAC` (ESP-NOW only) | the **slave's** MAC | the **master's** MAC |

Flash master → flash slave → open **http://motorA.local** on your phone
(the master also prints its IP on Serial @115200). The slave answers at
**http://motorB.local** but only serves API endpoints, not the UI.

## Wiring (identical on both boards)

| Driver pin | Goes to | Notes |
|---|---|---|
| STEP | **D5** (GPIO14) | |
| DIR | **D2** (GPIO4) | |
| EN | *not connected* | left open = always enabled; Enable/Disable in the UI is a software lockout only (coils stay energized). To get true coil-off disable, wire EN to a spare GPIO and set `HAVE_EN_PIN 1`. |
| VDD | 3V3 | logic supply |
| GND (logic) | G | |
| VMOT | +12 V | **not** 5 V |
| GND (motor) | 12 V supply − | |
| RST ↔ SLP | jumper together | **mandatory** — driver sleeps otherwise |
| 1A/1B, 2A/2B | motor coils | find pairs with a multimeter |

- **Kill switch → D7 (GPIO13)**, `INPUT_PULLUP`, **normally-closed to GND = run**.
  Opening the contact (or a broken wire — fail-safe) = KILL: instant stop, coils
  de-energized, and the master also kills the slave. Closing it re-arms (motor stays
  stopped until you command it again).
- **100 µF electrolytic across VMOT/GND on each driver** (stripe → GND).
- **Common ground**: tie both ESP grounds and both 12 V supply grounds together.
- Onboard LED: short flash every 1.4 s = alive · medium blink = no WiFi · fast blink = KILLED.

## PPR / unit math

`PPR_DIV10` (default **30** → **300 pulses/rev**) is used for every deg/rev ↔ step
conversion. Edit it live in the UI's "Setup & Status" card — the master pushes the
change to the slave and both persist it to flash (LittleFS).

## Ways to drive the motors

**Per motor (A and B each):** continuous fwd/rev · independent speed slider ·
acceleration slider · jog ± by steps/degrees/revs · go-to absolute position
(steps/deg/revs) · zero/home the counter · run N revolutions then auto-stop ·
enable/disable (de-energize coils) · soft stop (decel) · live telemetry.

**Combined:** both forward · both reverse · spin opposite (+ swap which is which) ·
synchronized start (sync-move picker) · global speed slider · link-speeds toggle ·
mirror mode (B repeats A's every command, inverted) · gear ratio (B speed = ratio × A) ·
one-button BOTH STOP / sticky EMERGENCY STOP (hard halt).

**Presets:** 4 named speed+direction presets (saved to flash) · 25/50/75/100 % quick-speed buttons.

## HTTP endpoints

Motor commands, prefix `/a/` (master local) or `/b/` (master → relayed to slave).
The slave exposes the identical set under `/m/`.

| Endpoint | Action |
|---|---|
| `/a/fwd` · `/a/rev` | continuous rotation at set speed (ramped) |
| `/a/stop` | decelerate to stop |
| `/a/estop` | instant hard stop |
| `/a/speed?v=800` | set speed (steps/s, 10–2000) |
| `/a/accel?v=1200` | set acceleration (steps/s², 50–5000) |
| `/a/jog?steps=200` / `?deg=90` / `?revs=1.5` | relative move (negative or `&d=-1` reverses) |
| `/a/goto?steps=0` / `?deg=…` / `?revs=…` | absolute move |
| `/a/runrevs?n=5&d=-1` | run N revs then stop |
| `/a/zero` | stop + zero the position counter |
| `/a/enable?on=0` | software motion lockout (1=allow, 0=block). With no EN wire, coils stay energized; set `HAVE_EN_PIN 1` for true coil-off. |
| `/a/status` | JSON telemetry |

Master-only globals:

| Endpoint | Action |
|---|---|
| `/both/fwd` · `/both/rev` | both same direction (B fired first to minimize skew) |
| `/both/opposite?swap=0\|1` | A fwd + B rev (swap flips it) |
| `/both/speed?v=` | A = v, B = v × gear |
| `/both/sync?a=fwd&b=rev` | fire any fwd/rev/stop pair together |
| `/both/stop` · `/stopall` | hard-stop both |
| `/config` | GET returns full state JSON; set `?pprdiv10=` `?gear=` `?mirror=0\|1` `?linked=0\|1` |
| `/preset/save?slot=0..3&name=` · `/preset/recall?slot=` | presets |
| `/status` | aggregated JSON (A live, B cached, reachability, flags) |

## ESP-NOW option (recommended for tight sync)

HTTP relay adds ~20–50 ms between B's and A's start and briefly blocks the master's
loop. ESP-NOW sends a tiny frame in ~1–2 ms with no blocking. To enable: flash both
boards once with it off, note each board's **MAC** from Serial, fill `PEER_MAC`
(each board gets the *other's* MAC), uncomment `#define USE_ESPNOW`, re-flash both.
ESP-NOW rides the current WiFi channel — fine on one router; if yours band-steers the
boards to different channels, pin it to a single 2.4 GHz channel. Status polling
stays on HTTP either way.

## Safety / robustness behavior

- **Kill switch** (each board, fail-safe NC wiring) — see wiring section.
- **Dead-man on the slave:** if nothing contacts it (master ping every 3 s, UI polls,
  ESP-NOW) for 8 s while its motor is moving, it stops itself.
- **WiFi watchdog:** either board stops its motor if WiFi stays down > 8 s;
  auto-reconnect is on. Motors stay disabled until WiFi is up at boot.
- **Slave unreachable:** the UI shows a warning banner; master keeps retrying
  (mDNS re-resolve every 10 s).
- Main loop is fully non-blocking — no `delay()` outside `setup()`.

## Known timing limitation

The ESP8266 is single-core and shares CPU with the WiFi stack, and AccelStepper
generates step pulses from `loop()`. At high speeds (≳1500–2000 steps/s) with heavy
web traffic, pulse timing can jitter and stall the rotor — this project has hit that
before (see `../CLAUDE.md`). Mitigations: enable **USE_ESPNOW**, use microstepping
(MS1/MS2 → 3V3 and raise PPR_DIV10 accordingly), and keep speeds moderate. An ESP32
would remove the limitation entirely.
