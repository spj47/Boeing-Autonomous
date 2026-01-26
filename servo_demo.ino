// arduino nano code for 550 kg-cm servo
#include <Servo.h>


const byte SERVO_PIN = 9;   // Servo signal pin

// servo limits, most likely changed with testing
const int STEER_LEFT   = 1320;
const int STEER_CENTER = 1500;
const int STEER_RIGHT  = 1680;

const int STEP_SIZE  = 3;   // 3ms step
const int STEP_DELAY = 4;   // 4 ms between steps

// centers
Servo steer;
int currentPos = STEER_CENTER;

void setup() {
  steer.attach(SERVO_PIN);
  steer.writeMicroseconds(STEER_CENTER);
}

// moves servo to target pos 
void moveSteering(int target) {
  target = constrain(target, STEER_LEFT, STEER_RIGHT);

  while (currentPos != target) {
    currentPos += (target > currentPos) ? STEP_SIZE : -STEP_SIZE;
    steer.writeMicroseconds(currentPos);
    delay(STEP_DELAY);
  }
}

void loop() {
  // ===== DEMO STEERING SEQUENCE =====
  moveSteering(STEER_LEFT);
  delay(1000);

  moveSteering(STEER_RIGHT);
  delay(1000);

  moveSteering(STEER_CENTER);
  delay(2000);
}
