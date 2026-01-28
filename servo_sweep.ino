// arduino nano code for 550 kg-cm servo
#include <Servo.h>

Servo servo;

int SERVO_PIN = 9;

// servo limits
const int MOVE_LEFT   = 1000;
const int CENTER = 1500;
const int MOVE_RIGHT  = 2000;

// 3 ms per step
const int STEP_SIZE_US = 5;
const int STEP_DELAY_MS = 3;

// initialize current position to center
int currentPos = CENTER;

// 
void setup() {
  servo.attach(SERVO_PIN, MOVE_LEFT, MOVE_RIGHT);
  servo.writeMicroseconds(currentPos);
}

// moves servo to target pos 
void moveServo(int target) {
  target = constrain(target, MOVE_LEFT, MOVE_RIGHT);

  while (currentPos != target) {
    if (target > currentPos) {
      currentPos += STEP_SIZE_US;
      if (currentPos > target) currentPos = target;
    } else {
      currentPos -= STEP_SIZE_US;
      if (currentPos < target) currentPos = target;
    }

    servo.writeMicroseconds(currentPos);
    delay(STEP_DELAY_MS);
  }
}

void loop() {
  // move dat hoe -90 to 90 degrees
  moveServo(MOVE_LEFT);
  delay(1000);

  moveServo(MOVE_RIGHT);
  delay(1000);

  moveServo(CENTER);
  delay(1000);
}
