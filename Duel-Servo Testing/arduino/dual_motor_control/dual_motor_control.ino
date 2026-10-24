// Dual stepper control for an ATmega2560 3D-printer board (RAMPS 1.4 pinout).
// Drives two steppers in the E0 and E1 driver slots (A4988) with raw step
// pulses and non-blocking timing — both motors can move simultaneously.
//
// Flash with Arduino IDE / arduino-cli, board "Arduino Mega 2560".
// NOTE: this erases the printer's original Marlin firmware.
//
// Serial protocol (115200 baud, newline-terminated):
//   A <n> [sps]   move motor A (E0) n steps, negative = reverse;
//                 optional speed in steps/sec for this and later moves
//   B <n> [sps]   move motor B (E1) likewise
//   S <sps>       set shared move speed, 1-20000 steps/sec (default 1000)
//   R <m> <sps>   run motor m (0=A, 1=B) continuously at signed steps/sec;
//                 0 stops it (driver stays energized = holding torque)
//   E <m> <0|1>   0 = release driver (motor freewheels), 1 = energize
//   Q             query -> "OK Q <a_left> <b_left>" (-1 = running continuously)
//   X             stop both motors immediately and release both drivers
//   P             ping -> "OK PONG"
// Every command is acknowledged with "OK ..." or "ERR ...".
//
// A4988 ENABLE is active LOW. At 1/16 microstepping (typical default),
// 3200 steps = one revolution of a 1.8-degree stepper.

const uint16_t MAX_SPS = 20000;

struct Motor {
  uint8_t stepPin, dirPin, enPin;
  long remaining;            // steps left in a finite move
  bool continuous;
  unsigned long intervalUs;  // time between step pulses
  unsigned long lastStepUs;
  bool energized;
};

// RAMPS 1.4 convention: E0 = 26/28/24, E1 = 36/34/30 (STEP/DIR/ENABLE)
Motor motors[2] = {
  {26, 28, 24, 0, false, 1000, 0, false},
  {36, 34, 30, 0, false, 1000, 0, false},
};

uint16_t moveSpeed = 1000;  // steps/sec for finite moves

void energize(Motor &m, bool on) {
  m.energized = on;
  digitalWrite(m.enPin, on ? LOW : HIGH);  // active LOW
  if (!on) { m.remaining = 0; m.continuous = false; }
}

void stopMotor(Motor &m) {
  m.remaining = 0;
  m.continuous = false;
}

void startMove(Motor &m, long steps, uint16_t sps) {
  m.continuous = false;
  if (steps == 0) { m.remaining = 0; return; }
  energize(m, true);
  digitalWrite(m.dirPin, steps > 0 ? HIGH : LOW);
  m.remaining = steps > 0 ? steps : -steps;
  m.intervalUs = 1000000UL / sps;
  m.lastStepUs = micros();
}

void startRun(Motor &m, int sps) {
  if (sps == 0) { stopMotor(m); return; }
  energize(m, true);
  digitalWrite(m.dirPin, sps > 0 ? HIGH : LOW);
  m.continuous = true;
  m.remaining = 0;
  m.intervalUs = 1000000UL / (unsigned long)(sps > 0 ? sps : -sps);
  m.lastStepUs = micros();
}

void service(Motor &m, unsigned long now) {
  if (!m.energized) return;
  if (!m.continuous && m.remaining == 0) return;
  if (now - m.lastStepUs < m.intervalUs) return;
  m.lastStepUs = now;
  digitalWrite(m.stepPin, HIGH);
  delayMicroseconds(3);  // A4988 needs >= 1 us
  digitalWrite(m.stepPin, LOW);
  if (!m.continuous) m.remaining--;
}

// ---- serial command handling ----

char lineBuf[40];
uint8_t lineLen = 0;

// Parse up to two whitespace-separated integers after the command letter.
// Returns how many were found.
uint8_t parseArgs(char *p, long &a, long &b) {
  uint8_t n = 0;
  for (uint8_t i = 0; i < 2; i++) {
    while (*p == ' ') p++;
    if (*p != '-' && (*p < '0' || *p > '9')) break;
    (i == 0 ? a : b) = atol(p);
    n++;
    while (*p && *p != ' ') p++;
  }
  return n;
}

void handleLine(char *line) {
  char cmd = line[0];
  long a = 0, b = 0;
  uint8_t nargs = parseArgs(line + 1, a, b);

  switch (cmd) {
    case 'P':
      Serial.println(F("OK PONG"));
      break;

    case 'A':
    case 'B': {
      if (nargs < 1) { Serial.println(F("ERR need step count")); return; }
      if (nargs == 2) {
        if (b < 1 || b > MAX_SPS) { Serial.println(F("ERR speed 1-20000")); return; }
        moveSpeed = (uint16_t)b;
      }
      startMove(motors[cmd == 'A' ? 0 : 1], a, moveSpeed);
      Serial.print(F("OK ")); Serial.print(cmd); Serial.print(' '); Serial.println(a);
      break;
    }

    case 'S':
      if (nargs < 1 || a < 1 || a > MAX_SPS) { Serial.println(F("ERR speed 1-20000")); return; }
      moveSpeed = (uint16_t)a;
      Serial.print(F("OK S ")); Serial.println(moveSpeed);
      break;

    case 'R':
      if (nargs < 2 || a < 0 || a > 1 || b < -(long)MAX_SPS || b > (long)MAX_SPS) {
        Serial.println(F("ERR R <0|1> <-20000..20000>")); return;
      }
      startRun(motors[a], (int)b);
      Serial.print(F("OK R ")); Serial.print(a); Serial.print(' '); Serial.println(b);
      break;

    case 'E':
      if (nargs < 2 || a < 0 || a > 1 || (b != 0 && b != 1)) {
        Serial.println(F("ERR E <0|1> <0|1>")); return;
      }
      if (b) energize(motors[a], true); else energize(motors[a], false);
      Serial.print(F("OK E ")); Serial.print(a); Serial.print(' '); Serial.println(b);
      break;

    case 'Q':
      Serial.print(F("OK Q "));
      Serial.print(motors[0].continuous ? -1 : motors[0].remaining);
      Serial.print(' ');
      Serial.println(motors[1].continuous ? -1 : motors[1].remaining);
      break;

    case 'X':
      for (uint8_t i = 0; i < 2; i++) { stopMotor(motors[i]); energize(motors[i], false); }
      Serial.println(F("OK X"));
      break;

    default:
      Serial.println(F("ERR unknown cmd"));
  }
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < 2; i++) {
    pinMode(motors[i].stepPin, OUTPUT);
    pinMode(motors[i].dirPin, OUTPUT);
    pinMode(motors[i].enPin, OUTPUT);
    digitalWrite(motors[i].stepPin, LOW);
    energize(motors[i], false);  // start released
  }
  Serial.println(F("READY dual_motor_control v1"));
}

void loop() {
  unsigned long now = micros();
  service(motors[0], now);
  service(motors[1], now);

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}
