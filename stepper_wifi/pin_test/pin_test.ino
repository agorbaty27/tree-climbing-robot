/*
 * GPIO PIN TEST  —  NodeMCU ESP8266
 * ---------------------------------------------------------
 * No stepper, no WiFi. Just toggles two pins slowly so you can
 * confirm each one works with a plain voltmeter or an LED.
 *
 * D2 and D5 toggle in OPPOSITE phases:
 *   - Probe D2 to GND: reads 3.3 V, then 0 V, alternating every 1.5 s
 *   - Probe D5 to GND: reads 0 V, then 3.3 V  (the opposite of D2)
 *   - Onboard LED blinks = the sketch is actually running
 *
 * Flash this, test, then re-flash your stepper sketch afterward.
 */

#define PIN_A 4     // D2  (GPIO4)
#define PIN_B 14    // D5  (GPIO14)

void setup() {
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // Phase 1 -> D2 HIGH, D5 LOW
  digitalWrite(PIN_A, HIGH);
  digitalWrite(PIN_B, LOW);
  digitalWrite(LED_BUILTIN, LOW);    // onboard LED ON (it's active-low)
  delay(1500);

  // Phase 2 -> D2 LOW, D5 HIGH
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);   // onboard LED OFF
  delay(1500);
}
