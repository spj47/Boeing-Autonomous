#include <Servo.h>

// Encoder Pins
const int ENCODER_STEERING_WHEEL_PIN  = A0;

// Servo data
Servo servo;
const int SERVO_PIN = 9;

// Motion handling (Both of these combined determine how quickly the servo moves)
const int STEP_SIZE_US  = 5;
const int STEP_DELAY_MS = 3;

// Servo pulse limits (µs)
const int SERVO_LEFT_US   = 1000;
const int SERVO_CENTER_US = 2035;
const int SERVO_RIGHT_US  = 2500;

const int LOWER_BOUND = SERVO_LEFT_US;
const int HIGHER_BOUND = SERVO_RIGHT_US;

// Servo State Control
int currentPos             = SERVO_CENTER_US; // initialize to lower bound
bool servoMoving           = false;       // True when the servo is moving to a targetPos
int targetPos              = SERVO_CENTER_US;  // Defines where we want the servo to be
unsigned long lastStepTime = 0;           // Defines the time since the last step of the servo motor

// Manual State Control
bool inManualMode = true;                // True when in manual mode
int manualOffset = 0;
int lastVal;

// Misc
const byte ALL_ADDRESS   = 0x00;
const byte LOCAL_ADDRESS = 0x01;
const int BAUD_RATE      = 9600;

// Debug
const bool IS_DEBUG = false;
float debugTimer = 0;
float debugTime  = 1000;

void setup() {
  servo.attach(SERVO_PIN, LOWER_BOUND, HIGHER_BOUND);
  servo.writeMicroseconds(currentPos);

  Serial.begin(BAUD_RATE);
}

void loop() {
  if (inManualMode)
  {
    int angle = getEncoderAngle(ENCODER_STEERING_WHEEL_PIN);
    int servoPulse = angleToServo(angle);
    if (!isLoop(angle))
    {
      lastVal = angle;
      moveServo(servoPulse);
    }
  }

  handleSerial();
  updateServo();
}

int angleToServo(int angle)
{
  return map(angle, 0, 360, LOWER_BOUND, HIGHER_BOUND);
}

bool isLoop(int value)
{
  int diff = value - lastVal;
  if (diff < 0) diff *= -1;

  return diff > 30;
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

// Convert a 0 to 100 steering percent into a calibrated pulse width
// 0 = full left
// 50 = straight
// 100 = full right
int percentToPulseUs(int percent) {
  percent = constrain(percent, 0, 100);

  if (percent <= 50) {
    return map(percent, 0, 50, SERVO_LEFT_US, SERVO_CENTER_US);
  }

  return map(percent, 50, 100, SERVO_CENTER_US, SERVO_RIGHT_US);
}

// ===== PUBLIC API =====
void driveServo(int percent) {
  /*
    This function is used to drive the servo motor from an external master by converting
    the input percent to a target for the servo motor.
    This function instantly returns in manual mode to avoid any issues.

    Args:
      percent (int): defines the position of the motor as a percentage
                     from the LOWER_BOUND to the UPPER_BOUND
  */

  if (inManualMode) return;

  int target = percentToPulseUs(percent);
  moveServo(target);
}

void toggleManualMode() {
  inManualMode = !inManualMode;
}

void setManualMode(bool isManual) {
  inManualMode = isManual;
}

void enterManualMode()
{
  inManualMode = true;
  manualOffset = getEncoderAngle(ENCODER_STEERING_WHEEL_PIN);  
}

int getEncoderAngle(int pin)
{
  int rawValue = analogRead(pin);
  return map(rawValue, 0, 1023, 0, 360);
}

void centerServo() {
  moveServo(SERVO_CENTER_US);
}

void handleSerial() {
  /*
    This function is how the rasberry PI is going to communicate with the nano
    to avoid using a USB port.
    Instead this looks for data coming in on the RX0 pin.

    Usage:
      "S:" - This sets the servo position
        Example command sent from the PI: "01:S:0\n"
                                          "01:S:50\n"
                                          "01:S:100\n"
      "T" -  This toggles manual mode
        Example command sent from the PI: "01:T\n"
      "M:" -  This sets manual mode
        Example command sent from the PI: "01:M:1\n" (enter manual mode)
                                          "01:M:0\n" (exit manual mode)
      "E" - This sets steering into emergency mode
        Example command sent from the PI: "01:E\n"
  */

  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  // Check to make sure the command was meant for this Arduino
  if (!(cmd.startsWith(String(ALL_ADDRESS)) || cmd.startsWith(String(LOCAL_ADDRESS)))) {
    return;
  }

  // Remove the address + separator
  int sepIndex = cmd.indexOf(':');
  if (sepIndex != -1) {
    cmd = cmd.substring(sepIndex + 1);
  }

  // Remove case sensitivity
  cmd.toUpperCase();

  // Process the command
  if (cmd.startsWith("S:")) {
    // Separate the numbers from the percent
    int ValueSepIndex = cmd.indexOf(':');
    if (ValueSepIndex != -1) {
      cmd = cmd.substring(ValueSepIndex + 1);
    }

    int percent = cmd.toInt();

    // toInt() returns 0 for non-numeric, so verify input is "0"
    if (percent == 0 && cmd != "0") {
      Serial.println("1:ERR:UNKNOWN");
    } else {
      driveServo(percent);

      // Confirm command received
      Serial.print("1:ACK:S:");
      Serial.println(percent);
      Serial.print(":US:");
      Serial.println(percentToPulseUs(percent));
    }
  }
  else if (cmd == "T") {
    toggleManualMode();

    Serial.println("1:ACK:T");
    Serial.println(inManualMode ? 1 : 0);
  }
  else if (cmd.startsWith("M:")) {
    bool setManual = (cmd.substring(2).toInt() == 1);
    setManualMode(setManual);

    Serial.print("1:ACK:M:");
    Serial.println(setManual ? 1 : 0);
  }
  else if (cmd == "E") {
    setManualMode(true);
    centerServo();

    Serial.println("1:ACK:E");
  }

  else if (cmd == "C") {
    centerServo();
    Serial.print("1:ACK:C:US:");
    Serial.println(SERVO_CENTER_US);
  }
  else {
    Serial.println("1:ERR:UNKNOWN");
  }
}