/****************************************************************************
  RP2040 + 1 × Adafruit 8‑Channel PWM FeatherWing
  + 2 × MG996R high‑torque digital servos

  servo1  → left / right (pan)
  servo2  → front / back (tilt)

  Randomly chooses a new position for each axis, triggers a sound,
  holds the position for 5–15 s, then pauses 5–10 s.
*********************************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/* ---------- FeatherWing (PCA9685) ---------- */
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);  // default address 0x40

/* ---------- Audio‑FX board trigger pins ---------- */
const uint8_t TRIG1_PIN = 2;   // RP2040 pin → Audio‑FX Trigger‑1 (T01.WAV)
const uint8_t TRIG2_PIN = 3;   // RP2040 pin → Audio‑FX Trigger‑2 (T02.WAV)

/* ---------- Servo channels ---------- */
const uint16_t SERVO1_CHAN = 0;   // Servo1 – pan (left/right)
const uint16_t SERVO2_CHAN = 1;   // Servo2 – tilt (front/back)

/* ---------- MG996R pulse‑width limits (PCA9685 steps) ---------- */
/* 0.5 ms = 102 steps (≈ 0°)
   2.5 ms = 512 steps (≈ 180°) */
const uint16_t MIN_PULSE = 102;   // ~0°
const uint16_t MAX_PULSE = 512;   // ~180° (tweak if you need a tighter range)

/* ---------- Target positions (angles) ---------- */
const uint16_t LEFT_MIN_ANGLE   = 0;     // fully left
const uint16_t LEFT_MAX_ANGLE   = 180;   // fully right

const uint16_t FRONT_MIN_ANGLE  = 0;     // fully forward
const uint16_t FRONT_MAX_ANGLE  = 180;   // fully back

/* ---------- Timing ranges (ms) ---------- */
const unsigned long MOVE_MIN_MS = 5000;
const unsigned long MOVE_MAX_MS = 15000;
const unsigned long PAUSE_MIN_MS = 5000;
const unsigned long PAUSE_MAX_MS = 10000;

/* ---------- Utility to map degrees → PCA9685 steps ---------- */
uint16_t pulseForAngle(uint16_t angle)
{
  // linear mapping from 0–180° to MIN_PULSE–MAX_PULSE
  return map(angle, 0, 180, MIN_PULSE, MAX_PULSE);
}

/* ---------- Random‑position helper ---------- */
void setRandomPosition()
{
  /* ----- Servo 1 (pan) ----- */
  uint16_t panAngle = random(LEFT_MIN_ANGLE, LEFT_MAX_ANGLE + 1);
  uint16_t panPulse = pulseForAngle(panAngle);

  /* ----- Servo 2 (tilt) ----- */
  uint16_t tiltAngle = random(FRONT_MIN_ANGLE, FRONT_MAX_ANGLE + 1);
  uint16_t tiltPulse = pulseForAngle(tiltAngle);

  // write to the wing
  pwm.setPWM(SERVO1_CHAN, 0, panPulse);
  pwm.setPWM(SERVO2_CHAN, 0, tiltPulse);

  Serial.print("Pan = ");
  Serial.print(panAngle);
  Serial.print("°   Tilt = ");
  Serial.print(tiltAngle);
  Serial.println("°");
}

/* ---------- Sound trigger helper ---------- */
void triggerSound(uint8_t pin)
{
  digitalWrite(pin, LOW);
  delayMicroseconds(10);        // 10 µs low pulse – enough for the Audio‑FX board
  digitalWrite(pin, HIGH);
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) ;              // wait for serial monitor

  /* I²C */
  Wire.begin();
  Wire.setClock(400000);         // 400 kHz

  /* FeatherWing – 50 Hz (20 ms period) */
  pwm.begin();
  pwm.setPWMFreq(50);

  /* Audio‑FX trigger pins – idle high */
  pinMode(TRIG1_PIN, OUTPUT);
  pinMode(TRIG2_PIN, OUTPUT);
  digitalWrite(TRIG1_PIN, HIGH);
  digitalWrite(TRIG2_PIN, HIGH);

  /* Random seed */
  randomSeed(analogRead(A0));    // any floating pin will do
}

void loop()
{
  /* ---- 1️⃣ Choose how long the new position will be held ----- */
  unsigned long moveDuration  = random(MOVE_MIN_MS, MOVE_MAX_MS + 1);
  unsigned long pauseDuration = random(PAUSE_MIN_MS, PAUSE_MAX_MS + 1);

  /* ---- 2️⃣ Pick a random position for each axis (pan & tilt) ---- */
  setRandomPosition();

  /* ---- 3️⃣ Trigger the two sounds ---- */
  triggerSound(TRIG1_PIN);
  triggerSound(TRIG2_PIN);

  /* ---- 4️⃣ Hold for the random duration ---- */
  Serial.print("Holding position for ");
  Serial.print(moveDuration / 1000);
  Serial.println(" s");
  delay(moveDuration);

  /* ---- 5️⃣ Pause before next iteration ---- */
  Serial.print("Pausing for ");
  Serial.print(pauseDuration / 1000);
  Serial.println(" s");
  delay(pauseDuration);
}
