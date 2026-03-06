#include <Servo.h>

Servo servo;

const int SERVO_PIN = 9;

const int MANUAL_DRIVER_FORWARD_PIN = 1;
const int MANUAL_DRIVER_BACKWARD_PIN = 2;
const int MANUAL_ENABLE_BUTTON = 3;
const int MANUAL_DISABLE_BUTTON = 4;

// Servo pulse limits (µs)
const int LOWER_BOUND  = 1000;
const int HIGHER_BOUND = 2000;
const int BOUND_DIFF   = HIGHER_BOUND - LOWER_BOUND;

// Motion handling (Both of these combined determine how quickly the servo moves)
const int STEP_SIZE_US  = 5;
const int STEP_DELAY_MS = 3;

// Servo State Control
int currentPos             = LOWER_BOUND; // initialize to lower bound
bool servoMoving           = false;       // True when the servo is moving to a targetPos 
int targetPos              = currentPos;  // Defines where we want the servo to be
unsigned long lastStepTime = 0;           // Defines the time since the last step of the servo motor

// Manual State Control
bool inManualMode          = false;       // True when in manual mode (This stops the driver funciton from getting run when true!!)
bool actuatingManual       = false;       // True when linear actuator is in motion to stop other inputs
bool deactuatingManual     = false;       // True when linear actuator is in motion to stop other inputs

void setup() {
  servo.attach(SERVO_PIN, LOWER_BOUND, HIGHER_BOUND);
  servo.writeMicroseconds(currentPos);

  pinMode(MANUAL_DRIVER_FORWARD_PIN, OUTPUT);
  pinMode(MANUAL_DRIVER_BACKWARD_PIN, OUTPUT);
  pinMode(MANUAL_ENABLE_BUTTON, INPUT);
  pinMode(MANUAL_DISABLE_BUTTON, INPUT);

  Serial.begin(9600);
}

void loop() {
  if (actuatingManual)
  {
    actuatingManual = checkManual(MANUAL_ENABLE_BUTTON);
    return;
  } else if (deactuatingManual)
  {
    deactuatingManual = checkManual(MANUAL_DISABLE_BUTTON);
    return;
  }

  if (!actuatingManual && !deactuatingManual)
  {
    digitalWrite(MANUAL_DRIVER_FORWARD_PIN, LOW);
    digitalWrite(MANUAL_DRIVER_BACKWARD_PIN, LOW);
  }

  handleSerial();
  updateServo();
}

bool checkManual(int pin)
{
    return digitalRead(pin) > 3;
}

// ===== Servo Control =====

// Use to request the servo to move 
void moveServo(int target) {
  targetPos = constrain(target, LOWER_BOUND, HIGHER_BOUND);
  servoMoving = true;
}

// Steps toward the current target
void updateServo() {
  if (!servoMoving) return;

  unsigned long now = millis();
  if (now - lastStepTime < STEP_DELAY_MS) return;

  lastStepTime = now;

  if (currentPos == targetPos) {
    servoMoving = false;
    return;
  }

  if (targetPos > currentPos) {
    currentPos += STEP_SIZE_US;
    if (currentPos > targetPos) currentPos = targetPos;
  } else {
    currentPos -= STEP_SIZE_US;
    if (currentPos < targetPos) currentPos = targetPos;
  }

  servo.writeMicroseconds(currentPos);
}

// ===== PUBLIC API =====

void driveServo(int percent) {
  /*
    This function is used to drive the servo motor from an external master by converting the input percent to a target for the servo motor
    This funciton instantly returns in manual mode to avoid any issues

    Args:
      percent (int): defines the position of the motor as a percentage from the LOWER_BOUND to the UPPER_BOUND
  */

  if (inManualMode) return;

  percent = constrain(percent, 0, 100);
  int target = LOWER_BOUND + (BOUND_DIFF * percent) / 100;
  moveServo(target);
}

void toggleManualMode() {
  inManualMode = !inManualMode;
  setManualMode (isManual);
}

void setManualMode(bool isManual) {
  inManualMode = isManual;
  if (inManualMode) enterManualMode();
  else exitManualMode();
}

void enterManualMode() {
  actuatingManual = true;
  digitalWrite(MANUAL_DRIVER_FORWARD_PIN, HIGH);
}

void exitManualMode() {
  deactuatingManual = true;
  digitalWrite(MANUAL_DRIVER_BACKWARD_PIN, HIGH);
}


void handleSerial() {
  /*
    This function is how the rasberry PI is going to communicate with the nano to avoid using a USB port 
    Intead this looks for data coming in on the RX0 pin

    Usage:
      "S:" - This sets the servo position
        Example command sent from the PI: "S:0\n"
                                          "S:50\n"
                                          "S:100\n"
      "T" -  This toggles manual mode 
        Example command sent from the PI: "T\n"
      "M:" -  This sets manual mode 
        Example command sent from the PI: "M:1\n" (enter manaul mode)
                                          "M:0\n" (exit manual mode)
  */

  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("S:")) {
    int percent = cmd.substring(2).toInt();
    driveServo(percent);
  }
  else if (cmd == "T") {
    toggleManualMode();
  }
  else if (cmd.startsWith("M:")) {
    bool setManual = (cmd.substring(2).toInt() == 1);
    setManualMode(setManual);
  }
}
