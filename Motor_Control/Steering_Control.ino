#include <Servo.h>

// Encoder Pins
const int ENCODER_STEERING_WHEEL_PIN  = A0;
const int ENCODER_STEERING_COLUMN_PIN = A1;

// Servo data
Servo servo;
const int SERVO_PIN = 9;

// Motion handling (Both of these combined determine how quickly the servo moves)
const int STEP_SIZE_US  = 5;
const int STEP_DELAY_MS = 3;

// Servo pulse limits (µs)
const float MIDPOINTPERCENT = 0.69;
const int LOWER_BOUND  = 1000;
const int UPPER_BOUND = 2500;

// Servo State Control
int currentPos             = 1250;        // initialize to middle of the bounds
int targetAngle            = 50;          // Defines where we want the servo to be
unsigned long lastStepTime = 0;           // Defines the time since the last step of the servo motor

// Manual State Control
bool inManualMode          = true;
int manualOffset           = 0;
float manualRatio          = 1;

// Misc
const byte ALL_ADDRESS   = 0x00;
const byte LOCAL_ADDRESS = 0x01;
const int BAUD_RATE      = 9600;

// Debug
const bool IS_DEBUG      = true;
const bool IS_DEBUG_CALIBRATION = false;
const bool IS_DEBUG_MANUAL = true;
float debugTimer         = 0;
float debugTime          = 1000;

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

// Calibration Meta Data
int lastBoundValue;

void setup() {
  servo.attach(SERVO_PIN, LOWER_BOUND, UPPER_BOUND);

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

void loop() {
  if (IS_DEBUG)
  {
    if (millis() - debugTimer > debugTime)
    {
      int steeringWheelAngle = getEncoderAngle(ENCODER_STEERING_WHEEL_PIN);
      int steeringColumnAngle = getEncoderAngle(ENCODER_STEERING_COLUMN_PIN);

      Serial.print("Calibration Status - ");
      Serial.print(calibrationState);
      Serial.print(" | Steering Wheel Angle - ");
      Serial.print(steeringWheelAngle);
      Serial.print(" | Steering Column Angle - ");
      Serial.print(steeringColumnAngle);
      Serial.print(" | In Manual Mode - ");
      Serial.println(inManualMode);

      if (IS_DEBUG_CALIBRATION)
      {
        Serial.print("Calibration Steps - ");
        Serial.print(currentCalibrationSteps);
        Serial.print(" | Last Value - ");
        Serial.print(lastBoundValue);
        Serial.print(" | Lower Bound -  ");
        Serial.print(lowerAngleBound);
        Serial.print(" | Upper Bound - ");
        Serial.println(upperAngleBound);
      }

      if (IS_DEBUG_MANUAL)
      {
        Serial.print("Steering Angle - ");
        Serial.print(steeringWheelAngle);
        Serial.print(" | Steering ColumnAngle ");
        Serial.print(steeringColumnAngle);
        Serial.print(" | Steering Wheel Offset -  ");
        Serial.println(steeringWheelAngle - manualOffset);
      }
      
      debugTimer = millis();
    }
  }

  if (calibrationState != 2)
  {
    CalibrateSystem();
    return;
  }

  if (inManualMode)
  {
    int steeringAngle = getEncoderAngle(ENCODER_STEERING_WHEEL_PIN);
    int coloumnAngle = getEncoderAngle(ENCODER_STEERING_COLUMN_PIN);

    if (abs(steeringAngle - manualOffset) > CALIBRATION_BUFFER)
    {
      if (steeringAngle > manualOffset)
      {
        targetAngle--;
      }
      else 
      {
        targetAngle++;        
      }
      moveServo(targetAngle);
    }
  }

  updateServo();
  handleSerial();
}

void InitCalibration()
{
  calibrationState = -1;
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
        servo.writeMicroseconds(UPPER_BOUND);
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
        centerBound = angleBoundDiff * MIDPOINTPERCENT + lowerAngleBound;
        driveServo(50); // Go to middle when done
      }
      break;
  }
}

// ===== Servo Control =====

// Use to request the servo to move 
void moveServo(int target) {
  targetAngle = curveAngle(target);
}

int curveAngle(int rawAngle)
{
  rawAngle = constrain(rawAngle, lowerAngleBound, upperAngleBound);
  int rawMidPoint = lowerAngleBound + (angleBoundDiff / 2);
  if (rawAngle < MIDPOINTPERCENT)
  {
    return map(rawAngle, lowerAngleBound, rawMidPoint, lowerAngleBound, centerBound);
  }
  else
  {
    return map(rawAngle, rawMidPoint, upperAngleBound, centerBound, upperAngleBound);
  }
}

// Steps toward the current target
void updateServo() {
  unsigned long now = millis();

  // Step only after a STEP_DELAY_MS amount of time has pssed
  if (now - lastStepTime < STEP_DELAY_MS) return;
  lastStepTime = now;

  // Get the current real angle of the steering coloun
  int currentAngle = getEncoderAngle(ENCODER_STEERING_COLUMN_PIN);

  // Check to see if the angle is within the buffer of the target
  if (abs(currentAngle - targetAngle) < CALIBRATION_BUFFER) {
    return;
  }

  // Inch the motor in the direction of the target
  if (targetAngle > currentAngle) {
    currentPos += STEP_SIZE_US;
  } else {
    currentPos -= STEP_SIZE_US;
  }

  // Write the new PWM to the servo
  servo.writeMicroseconds(currentPos);
}

int getEncoderAngle(int pin)
{
  int rawValue = analogRead(pin);
  return map(rawValue, 0, 1023, 0, 360);
}

void driveServo(int percent) 
{
  if (inManualMode) return;

  percent = constrain(percent, 0, 100);

  // Convert the percent to angles
  int target = lowerAngleBound + (angleBoundDiff * percent) / 100;
  moveServo(target);
}

void toggleManualMode() {
  inManualMode = !inManualMode;
  setManualMode(inManualMode);
}

void setManualMode(bool isManual) {
  inManualMode = isManual;
  if (inManualMode) enterManualMode();
  else exitManualMode();
}

void enterManualMode() {
  inManualMode = true;
  manualOffset = getEncoderAngle(ENCODER_STEERING_WHEEL_PIN);
}

void exitManualMode() {
  inManualMode = false;
}


void handleSerial() {
  /*
    This function is how the rasberry PI is going to communicate with the nano to avoid using a USB port 
    Intead this looks for data coming in on the RX0 pin

    Usage:
      "S:" - This sets the servo position
        Example command sent from the PI: "01:S:0\n"
                                          "01:S:50\n"
                                          "01:S:100\n"
      "T" -  This toggles manual mode 
        Example command sent from the PI: "01:T\n"
      "M:" -  This sets manual mode 
        Example command sent from the PI: "01:M:1\n" (enter manaul mode)
                                          "01:M:0\n" (exit manual mode)
      "E" - This sets steering into emergency mode
        Example Command sent from the PI: "01:E\n"
  */

  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n'); // read the command until its end
  cmd.trim();

  // Check to make sure the command was meant for this adruino
  if (!(cmd.startsWith(String(ALL_ADDRESS)) || cmd.startsWith(String(LOCAL_ADDRESS))))
  {
    return;
  }

  // Remove the address + separator
  int sepIndex = cmd.indexOf(':');
  if (sepIndex != -1) 
  {
    cmd = cmd.substring(sepIndex + 1);
  }

  // Remove case senstivity
  cmd.toUpperCase();

  // Proccess the command
  if (cmd.startsWith("S:")) 
  {
    // Seperate the numbers from the percent
    int sepIndex = cmd.indexOf(':');
    if (sepIndex != -1) 
    {
        cmd = cmd.substring(sepIndex + 1);
    }

    // throttle commands only work in AUTO mode
    int percent = cmd.toInt();

    // toInt() returns 0 for non-numeric, so verify input is "0"
    if (percent == 0 && cmd != "0")
    {
        Serial.println("ERR:UNKNOWN");
    }
    else 
    {
      driveServo(percent);
    }
  }
  else if (cmd == "T") {
    toggleManualMode();
  }
  else if (cmd.startsWith("M:")) {
    bool setManual = (cmd.substring(2).toInt() == 1);
    setManualMode(setManual);
  }
  else if (cmd == "E") {
    setManualMode(true);
  }
}
