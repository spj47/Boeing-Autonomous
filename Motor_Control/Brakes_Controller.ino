#include <Servo.h>

Servo servo;

const int SERVO_PIN = 9;
const int E_LIMITSWITCH = 2;

// Servo pulse limits (µs)
const int LOWER_BOUND  = 1000;
const int HIGHER_BOUND = 2000;
const int BOUND_DIFF   = HIGHER_BOUND - LOWER_BOUND;

// Motion handling
const int STEP_SIZE_US  = 5;
const int STEP_DELAY_MS = 3;

// State control
int currentPos             = LOWER_BOUND; // initialize to lower bound
bool inManualMode          = false;       // True when in manual mode (This stops the driver funciton from getting run when true!!)
bool servoMoving           = false;       // True when the servo is moving to a targetPos 
int targetPos              = currentPos;  // Defines where we want the servo to be
unsigned long lastStepTime = 0;           // Defines the time since the last step of the servo motor
bool isInEmergencyMode     = false;
const byte NULL_ERRORCODE  = 00;
byte currentErrorCode      = NULL_ERRORCODE; // 00 no error | 01 Serial recived while in emergency mode | 02 invalid serial input

// Misc
const byte ALL_ADDRESS   = 0x00;
const byte LOCAL_ADDRESS = 0x03;
const int BAUD_RATE      = 9600;

void setup() {
  servo.attach(SERVO_PIN, LOWER_BOUND, HIGHER_BOUND);
  servo.writeMicroseconds(currentPos);

  pinMode(E_LIMITSWITCH, INPUT_PULLUP);

  Serial.begin(BAUD_RATE);
}

void loop() 
{
  Serial.println(digitalRead(E_LIMITSWITCH));
  return;

  checkEStops();  // Checks to see if the pedal has been pressed (1st Priority since it handles emergency mode)
  handleSerial(); // Check for any inputed serial (2nd priorty since it overrides how the update servo is going to act)
  updateServo();  // Handle the servo's position (last priority since nothing else is left :) )
}

void checkEStops()
{
  if (!isInEmergencyMode && currentErrorCode != NULL_ERRORCODE)
  {
    enterEmergencyMode();
    return;
  }

  int val = digitalRead(E_LIMITSWITCH);
  if (val)
  {
    enterEmergencyMode();
  }
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
  if (inManualMode) enterManualMode();
}

void setManualMode(bool isManual) {
  inManualMode = isManual;
  if (inManualMode) enterManualMode();
}

void enterManualMode() {
  // Set to Lower Bound to completely disengage the calibers from the pad
  moveServo(LOWER_BOUND);
}

void handleSerial() {
  /*
    This function is how the rasberry PI is going to communicate with the nano to avoid using a USB port 
    Intead this looks for data coming in on the RX0 pin

    Usage:
      "S:" - This sets the servo position
        Example command sent from the PI: "03:S:0\n"
                                          "03:S:50\n"
                                          "03:S:100\n"
      "T" -  This toggles manual mode 
        Example command sent from the PI: "03:T\n"
      "M:" -  This sets manual mode 
        Example command sent from the PI: "03:M:1\n" (enter manaul mode)
                                          "03:M:0\n" (exit manual mode)
      "E" - This sets steering into emergency mode
        Example Command sent from the PI: "03:E\n"
  */

  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n'); // read the command until its end
  cmd.trim();

  // Check to make sure the command was meant for this adruino
  if (!(cmd.startsWith(String(ALL_ADDRESS)) || cmd.startsWith(String(LOCAL_ADDRESS))) || isInEmergencyMode)
  {
    if (isInEmergencyMode)
    {
      currentErrorCode = 01;
    }
    return;
  }

  // Remove the address + separator
  int sepIndex = cmd.indexOf(':');
  if (sepIndex != -1) 
  {
    cmd = cmd.substring(sepIndex + 1);
  }
  else
  {
    currentErrorCode = 02;
    return;
  }

  // Remove case senstivity
  cmd.toUpperCase();

  // Proccess the command
  if (cmd.startsWith("S:")) {
    // Seperate the numbers from the percent
    int sepIndex = cmd.indexOf(':');
    if (sepIndex != -1) 
    {
      cmd = cmd.substring(sepIndex + 1);
    }
    else 
    {
      currentErrorCode = 02;
      return;
    }

    // throttle commands only work in AUTO mode
    int percent = cmd.toInt();
    
    // toInt() returns 0 for non-numeric, so verify input is "0"
    if (percent == 0 && cmd != "0")
    {
      currentErrorCode = 02;
      return;
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
    enterEmergencyMode();
  }
}

void enterEmergencyMode()
{
  isInEmergencyMode = true;
  setManualMode(true);
}