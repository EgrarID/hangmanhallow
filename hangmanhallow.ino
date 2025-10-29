/*********************************************************************
  Animatronic “Spine” – Random X/Y motion with smooth transitions
  Parts used
    • Adafruit RP2040 Scorpio Feather V2
    • Adafruit 8‑Channel PWM Feather wing (PCA9685)
    • 2× MG996R 55 g Digital RC Servo (metal‑gear, 50 Hz)
  Author:  EgrarID
  Date:    10-29-25
*********************************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/* -------------------------------------------------------------
   1.  Create a PWM/Servo driver object
   ------------------------------------------------------------- */
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();   // default address 0x40

/* -------------------------------------------------------------
   2.  PCA9685 channel assignments
   ------------------------------------------------------------- */
const uint8_t SERVO_X = 0;   // channel 0  – X‑axis (forward/backward)
const uint8_t SERVO_Y = 1;   // channel 1  – Y‑axis (side‑to‑side)

/* -------------------------------------------------------------
   3.  Servo pulse‑width limits for MG996R
   ------------------------------------------------------------- */
/* 50 Hz → 20 ms period, 4096 ticks → 4.88 µs per tick
   1 ms pulse  → 205 ticks  (≈ 1 ms)
   1.5 ms pulse → 308 ticks  (≈ 1.5 ms)
   2 ms pulse  → 410 ticks  (≈ 2 ms)
*/
const uint16_t MIN_PULSE = 205;   // 1 ms  – 0°
const uint16_t MAX_PULSE = 410;   // 2 ms  – 180°

/* -------------------------------------------------------------
   4.  Current target angles (start at neutral 90°)
   ------------------------------------------------------------- */
int currentAngleX = 90;
int currentAngleY = 90;

/* -------------------------------------------------------------
   5.  Helper – convert angle (0…180) to pulse width (ticks)
   ------------------------------------------------------------- */
uint16_t angleToPulse(int angle) {
  /* linear mapping: 0° → MIN_PULSE, 180° → MAX_PULSE */
  return map(angle, 0, 180, MIN_PULSE, MAX_PULSE);
}

/* -------------------------------------------------------------
   6.  Smoothly move a single servo from its current angle
       to a new target angle.  Returns the new angle.
   ------------------------------------------------------------- */
void smoothMove(uint8_t pin, int targetAngle, unsigned int stepDelay = 15) {
  int step = (targetAngle > currentAngleX || targetAngle > currentAngleY) ? 1 : -1;
  for (int a = currentAngleX; a != targetAngle; a += step) {
    uint16_t pulse = angleToPulse(a);
    pwm.setPin(pin, pulse);          // Adafruit‑PWM‑Servo‑Driver‑Library syntax
    delay(stepDelay);                // ~15 ms gives ~2–3 Hz slew
  }
  /* Update the global current angle after the loop */
  if (pin == SERVO_X) currentAngleX = targetAngle;
  if (pin == SERVO_Y) currentAngleY = targetAngle;
}

/* -------------------------------------------------------------
   7.  Move both servos simultaneously toward new angles.
       This function steps both servos together for smooth
       transitions.  It does not block for the entire motion
       – it only steps until both reach their targets.
   ------------------------------------------------------------- */
void moveBothSimultaneously(int targetX, int targetY, unsigned int stepDelay = 15) {
  int angleX = currentAngleX;
  int angleY = currentAngleY;
  while (angleX != targetX || angleY != targetY) {
    if (angleX < targetX) angleX++;
    else if (angleX > targetX) angleX--;

    if (angleY < targetY) angleY++;
    else if (angleY > targetY) angleY--;

    uint16_t pulseX = angleToPulse(angleX);
    uint16_t pulseY = angleToPulse(angleY);

    pwm.setPin(SERVO_X, pulseX);
    pwm.setPin(SERVO_Y, pulseY);

    delay(stepDelay);
  }
  currentAngleX = targetX;
  currentAngleY = targetY;
}

/* -------------------------------------------------------------
   8.  Setup – initialise I²C, PWM driver, and servo frequency
   ------------------------------------------------------------- */
void setup() {
  Serial.begin(115200);
  while (!Serial) ;           // wait for serial (useful on some boards)

  pwm.begin();                 // initialise PCA9685
  pwm.setPWMFreq(50);           // 50 Hz for standard RC servos
  Serial.println(F("PWM‑Servo‑Driver initialised – 50 Hz"));
}

/* -------------------------------------------------------------
   9.  Main loop – random spine motion for 20 s, return to 90°,
       pause 5 s, repeat
   ------------------------------------------------------------- */
void loop() {
  unsigned long startTime = millis();
  while (millis() - startTime < 20000) {   // 20 s of random motion
    /* Random target angles in the safe 0–180° range */
    int targetX = random(0, 181);
    int targetY = random(0, 181);

    /* Move both servos at the same time to the new targets */
    moveBothSimultaneously(targetX, targetY, 15);

    /* Small pause before the next random move (optional) */
    delay(200);
  }

  /* Return to neutral (90°) smoothly */
  moveBothSimultaneously(90, 90, 20);

  /* 5 second pause before next cycle */
  delay(5000);
}
