#include <Servo.h>

// Encoder Pins
const int ENCODER_STEERING_WHEEL_PIN  = A0;
const int ENCODER_STEERING_COLUMN_PIN  = A1;

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

// Error Handling
const byte NO_ERROR = 00;
const byte COLUMN_DEVIATION = 01; // column deviated beyond what motor is reporting

byte currentErrorCode = NO_ERROR;

// Debug
const bool IS_DEBUG = false;
float debugTimer = 0;
float debugTime  = 1000;

// Auto-Calibration
int calibrationState = -1; // -1 Uncalibrated | 0 Calibrating Lower Bound | 1 Calibrating Upper Bound | 2 Calibrated

int upperAngleBound;
int lowerAngleBound;
int centerBound;
int angleBoundDiff;

long currentCalibrationSteps;
const long CALIBRATION_STEPS = 20000;
const int CALIBRATION_BUFFER = 5;
const long CALIBRATION_SETTLE_TIME = 5000;

const int ERROR_BUFFER = 20;
const long ERROR_COUNT = 20000;
long errorCounter = 0;

// Calibration Meta Data
int lastBoundValue;

void setup() {
  servo.attach(SERVO_PIN, LOWER_BOUND, HIGHER_BOUND);

  pinMode(SERVO_PIN, OUTPUT);
  pinMode(ENCODER_STEERING_WHEEL_PIN, INPUT);
  pinMode(ENCODER_STEERING_COLUMN_PIN, INPUT);

  InitCalibration();
  if (inManualMode)
  {
    enterManualMode();
  }

  Serial.begin(BAUD_RATE);
}

void InitCalibration()
{
  calibrationState = -1;
}

void loop() {
  if (calibrationState != 2)
  {
    CalibrateSystem();
    return;
  }

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

  if (checkInvalidAngle())
  {
    currentErrorCode = COLUMN_DEVIATION;
  }

  if (currentErrorCode != NO_ERROR)
  {
    return;
  }

  updateServo();
}

bool checkInvalidAngle()
{
  // Get column pulse
  int columnAngle = getEncoderAngle(ENCODER_STEERING_COLUMN_PIN);
  int rawColumnPercent = columnAngle - lowerAngleBound / (upperAngleBound - lowerAngleBound);
  int curvedPulsePercent = percentToPulseUs(rawColumnPercent);

  // Get difference between target
  int errDeviation = curvedPulsePercent - targetPos;
  if (errDeviation < 0) errDeviation *= -1;

  bool isError = errDeviation > ERROR_BUFFER;
  if (isError)
  {
    errorCounter++;
  }
  else
  {
    errorCounter = 0;
  }

  return (errorCounter > ERROR_COUNT);
}

int angleToServo(int angle)
{
  return map(angle, 0, 360, LOWER_BOUND, HIGHER_BOUND);
}

void CalibrateSystem()
{
  // Get the new Value
  int c1 = getEncoderAngle(ENCODER_STEERING_COLUMN_PIN);
  switch (calibrationState)
  {
    case (-1): 
      servo.writeMicroseconds(LOWER_BOUND);
      currentCalibrationSteps = 0;
      lastBoundValue = c1;
      calibrationState++;
      break;
    case (0): 
      // Check if it is within buffer
      if (abs(lastBoundValue - c1) < CALIBRATION_BUFFER)
      {
        currentCalibrationSteps++;
      }
      else 
      {
        lastBoundValue = c1;
        currentCalibrationSteps = 0;
      }

      // Set calibrated Values
      if (currentCalibrationSteps > CALIBRATION_STEPS)
      {
        lowerAngleBound = c1;
        servo.writeMicroseconds(HIGHER_BOUND);
        currentCalibrationSteps = 0;
        calibrationState++;
      }
      break;
    case (1): 
      // Check if it is within buffer
      if (abs(lastBoundValue - c1) < CALIBRATION_BUFFER)
      {
        currentCalibrationSteps++;
      }
      else 
      {
        lastBoundValue = c1;
        currentCalibrationSteps = 0;
      }
      // Set calibrated Values
      if (currentCalibrationSteps > CALIBRATION_STEPS)
      {
        upperAngleBound = c1;
        angleBoundDiff = upperAngleBound - lowerAngleBound;
        calibrationState++;
        driveServo(50); // Go to middle when done
      }
      break;
  }
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