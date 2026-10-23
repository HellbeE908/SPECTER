const int EN_PIN   = D12;
const int STEP_PIN = D11;
const int DIR_PIN  = D10;

const int MICROSTEPS    = 8;
const int STEPS_PER_REV = 200 * MICROSTEPS;
const int PULSE_US      = 2000;

void setup() {
  Serial.begin(115200);
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);
  delay(200);
  digitalWrite(EN_PIN, LOW);
  Serial.println("enabled");
}

void spin(bool cw, int steps, int pulse_us) {
  digitalWrite(DIR_PIN, cw ? HIGH : LOW);
  delayMicroseconds(10);
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(pulse_us);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(pulse_us);
  }
}

void loop() {
  Serial.println("CW");  spin(true,  STEPS_PER_REV, PULSE_US); delay(800);
  Serial.println("CCW"); spin(false, STEPS_PER_REV, PULSE_US); delay(800);
}