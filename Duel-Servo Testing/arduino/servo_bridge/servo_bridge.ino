// Dual-servo bridge: Arduino Uno + PCA9685 (I2C @ 0x40)
// Drives the PCA9685 with raw Wire calls — no external libraries required.
//
// Serial protocol (115200 baud, newline-terminated):
//   S <ch> <us>   set servo pulse width in microseconds (e.g. "S 0 1500")
//   X <ch>        release channel (output fully off, servo goes limp)
//   F <hz>        set PWM frequency (default 50 Hz)
//   P             ping -> "OK PONG"
// Every command is acknowledged with "OK ..." or "ERR ...".

#include <Wire.h>

const uint8_t PCA_ADDR = 0x40;

// PCA9685 registers
const uint8_t REG_MODE1     = 0x00;
const uint8_t REG_LED0_ON_L = 0x06;
const uint8_t REG_PRESCALE  = 0xFE;

const uint8_t MODE1_SLEEP   = 0x10;
const uint8_t MODE1_AI      = 0x20;  // register auto-increment
const uint8_t MODE1_RESTART = 0x80;

uint16_t pwmFreq = 50;

void pcaWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t pcaRead(uint8_t reg) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(PCA_ADDR, (uint8_t)1);
  return Wire.read();
}

void pcaSetFreq(uint16_t hz) {
  pwmFreq = hz;
  // prescale = round(25MHz / (4096 * hz)) - 1
  uint8_t prescale = (uint8_t)((25000000.0 / (4096.0 * hz)) + 0.5) - 1;
  uint8_t oldMode = pcaRead(REG_MODE1);
  pcaWrite(REG_MODE1, (oldMode & ~MODE1_RESTART) | MODE1_SLEEP);
  pcaWrite(REG_PRESCALE, prescale);
  pcaWrite(REG_MODE1, oldMode);
  delay(5);
  pcaWrite(REG_MODE1, oldMode | MODE1_RESTART | MODE1_AI);
}

// Set raw on/off tick values (0-4095) for a channel
void pcaSetPWM(uint8_t ch, uint16_t on, uint16_t off) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(REG_LED0_ON_L + 4 * ch);
  Wire.write(on & 0xFF);
  Wire.write(on >> 8);
  Wire.write(off & 0xFF);
  Wire.write(off >> 8);
  Wire.endTransmission();
}

void pcaSetPulseUs(uint8_t ch, uint16_t us) {
  // ticks = us * 4096 * freq / 1e6
  uint32_t ticks = (uint32_t)us * 4096UL * pwmFreq / 1000000UL;
  if (ticks > 4095) ticks = 4095;
  pcaSetPWM(ch, 0, (uint16_t)ticks);
}

void pcaRelease(uint8_t ch) {
  // Set the full-off bit (bit 4 of LEDn_OFF_H)
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(REG_LED0_ON_L + 4 * ch);
  Wire.write((uint8_t)0); Wire.write((uint8_t)0);
  Wire.write((uint8_t)0); Wire.write((uint8_t)0x10);
  Wire.endTransmission();
}

char lineBuf[32];
uint8_t lineLen = 0;

void handleLine(char *line) {
  char cmd = line[0];
  if (cmd == 'P') {
    Serial.println(F("OK PONG"));
    return;
  }

  int ch = -1, val = -1;
  char *p = line + 1;
  while (*p == ' ') p++;
  ch = atoi(p);
  while (*p && *p != ' ') p++;   // skip first number
  while (*p == ' ') p++;
  if (*p) val = atoi(p);

  switch (cmd) {
    case 'S':
      if (ch < 0 || ch > 15 || val < 400 || val > 2800) {
        Serial.println(F("ERR S args (ch 0-15, us 400-2800)"));
        return;
      }
      pcaSetPulseUs((uint8_t)ch, (uint16_t)val);
      Serial.print(F("OK S ")); Serial.print(ch); Serial.print(' '); Serial.println(val);
      break;
    case 'X':
      if (ch < 0 || ch > 15) { Serial.println(F("ERR X args")); return; }
      pcaRelease((uint8_t)ch);
      Serial.print(F("OK X ")); Serial.println(ch);
      break;
    case 'F':
      if (ch < 24 || ch > 1000) { Serial.println(F("ERR F args (24-1000)")); return; }
      pcaSetFreq((uint16_t)ch);
      Serial.print(F("OK F ")); Serial.println(ch);
      break;
    default:
      Serial.println(F("ERR unknown cmd"));
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  pcaWrite(REG_MODE1, MODE1_AI);  // wake, auto-increment
  delay(5);
  pcaSetFreq(50);
  for (uint8_t ch = 0; ch < 16; ch++) pcaRelease(ch);

  Serial.println(F("READY servo_bridge v1"));
}

void loop() {
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
