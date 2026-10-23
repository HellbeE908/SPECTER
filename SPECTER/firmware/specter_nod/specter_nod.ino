// SPECTER nod-axis controller
// Seeed XIAO ESP32-S3 + TMC2209, NEMA 17 through a 3:1 GT2 belt
//
// Commands, one per line (Serial Monitor: line ending = Newline):
//   G<deg>   move to absolute axis angle    -> ARRIVED <deg>
//   Z        set current position as zero   -> OK ZERO
//   ?        report position                -> POS <deg>
//   E        enable motor (holds position)  -> OK ENABLED
//   D        disable motor (turns freely)   -> OK DISABLED
//
// Errors: ERR LIMIT, ERR DISABLED, ERR UNKNOWN

// ---------------------------------------------------------------- pins
const int EN_PIN   = D0;
const int STEP_PIN = D1;
const int DIR_PIN  = D2;

// ---------------------------------------------------------------- mechanics
const float GEAR_RATIO    = 3.0;    // 20T motor pulley : 60T axis pulley
const int   MICROSTEPS    = 8;      // TMC2209 with MS1/MS2 left floating
const float STEPS_PER_DEG = 200.0 * MICROSTEPS * GEAR_RATIO / 360.0;  // 13.333
                                    // one microstep = 0.075 deg of nod

// Soft limits - set these INSIDE your mechanism's safe travel
// so the head never strains the cable loop or hits anything.
const float MIN_DEG = -100.0;
const float MAX_DEG =  100.0;

// ---------------------------------------------------------------- motion
const int  START_US   = 1200;   // half-period at the start/end of a move (slow)
const int  CRUISE_US  = 300;    // half-period at full speed
const long RAMP_STEPS = 200;    // steps used to accelerate and to decelerate
const int  SETTLE_MS  = 150;    // pause after each move before ARRIVED
const bool DIR_INVERT = false;  // flip if positive angles move the wrong way

long position = 0;              // microsteps from zero
bool enabled  = false;

// ----------------------------------------------------------------
void setEnabled(bool on) {
  digitalWrite(EN_PIN, on ? LOW : HIGH);   // TMC2209: LOW = enabled
  enabled = on;
}

float positionDeg() {
  return position / STEPS_PER_DEG;
}

// Move a number of microsteps with a linear accelerate/decelerate ramp.
void moveSteps(long steps) {
  if (steps == 0) return;

  bool fwd = steps > 0;
  digitalWrite(DIR_PIN, (fwd != DIR_INVERT) ? HIGH : LOW);
  delayMicroseconds(20);                   // DIR setup time

  long n = labs(steps);
  long ramp = min(RAMP_STEPS, n / 2);

  for (long i = 0; i < n; i++) {
    long fromEdge = min(i, n - 1 - i);     // distance to nearest end of move
    long us = CRUISE_US;
    if (fromEdge < ramp) {
      us = START_US - (START_US - CRUISE_US) * fromEdge / ramp;
    }
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(us);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(us);
    position += fwd ? 1 : -1;
  }
}

// Move to an absolute angle. The step count is always derived from the
// target angle, so rounding error never accumulates.
void gotoDeg(float deg) {
  if (deg < MIN_DEG || deg > MAX_DEG) {
    Serial.println("ERR LIMIT");
    return;
  }
  if (!enabled) {
    Serial.println("ERR DISABLED");
    return;
  }
  long target = lround(deg * STEPS_PER_DEG);
  moveSteps(target - position);
  delay(SETTLE_MS);                        // let vibration die down
  Serial.print("ARRIVED ");
  Serial.println(positionDeg(), 3);
}

// ----------------------------------------------------------------
void setup() {
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);              // stay disabled while booting
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);

  Serial.begin(115200);
  Serial.setTimeout(50);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) { }

  setEnabled(true);
  Serial.println("READY");
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char c = toupper(line.charAt(0));
  switch (c) {
    case 'G':
      gotoDeg(line.substring(1).toFloat());
      break;
    case 'Z':
      position = 0;
      Serial.println("OK ZERO");
      break;
    case '?':
      Serial.print("POS ");
      Serial.println(positionDeg(), 3);
      break;
    case 'E':
      setEnabled(true);
      Serial.println("OK ENABLED");
      break;
    case 'D':
      setEnabled(false);
      Serial.println("OK DISABLED");
      break;
    default:
      Serial.println("ERR UNKNOWN");
  }
}
