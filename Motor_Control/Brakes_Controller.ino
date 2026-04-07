#include <Servo.h>

Servo servo;

const int SERVO_PIN = 9;
const int E_LIMITSWITCH_PIN = 2;
const int CURRENT_SENSE_PIN = A0;

// Servo pulse limits (µs)
const int LOWER_BOUND  = 500;
const int HIGHER_BOUND = 2500;

// Current Target handling
const int LOWER_CURRENT_BOUND = 170; // 5A/30A * 1023
const int UPPER_CURRENT_BOUND = 682; // 20A/30A * 1023
const int CURRENT_BOUND_DIFF = UPPER_CURRENT_BOUND - LOWER_CURRENT_BOUND;
const int CURRENT_TARGET_BUFFER = 10;

// Motion handling
const int STEP_SIZE_US  = 5;
const int STEP_DELAY_MS = 3;

// State control
int currentStepPos             = LOWER_BOUND; // initialize to lower bound
bool inManualMode          = false;       // True when in manual mode (This stops the driver funciton from getting run when true!!)
int targetCurrent              = 50;  // Defines where we want the servo to be
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
  servo.writeMicroseconds(currentStepPos);

  pinMode(E_LIMITSWITCH_PIN, INPUT_PULLUP);
  pinMode(CURRENT_SENSE_PIN, INPUT);

  Serial.begin(BAUD_RATE);
}

void loop() 
{
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

  int val = digitalRead(E_LIMITSWITCH_PIN);
  if (val)
  {
    enterEmergencyMode();
  }
}

// ===== Servo Control =====

// Use to request the servo to move 
void moveServo(int target) {
  targetCurrent = constrain(target, LOWER_CURRENT_BOUND, UPPER_CURRENT_BOUND);
}

// Steps toward the current target
void updateServo() {
  if (!servoMoving) return;

  unsigned long now = millis();
  if (now - lastStepTime < STEP_DELAY_MS) return;

  lastStepTime = now;
  int cCurrent = analogRead(CURRENT_SENSE_PIN);

  if (abs(cCurrent - targetCurrent) < CURRENT_TARGET_BUFFER) {
    // Close enough to target; Do nothing
  }
  else // Not at target yet
  {
    if (targetCurrent > cCurrent) 
    {
      currentStepPos += STEP_SIZE_US;
    } 
    else 
    {
      currentStepPos -= STEP_SIZE_US;
    }
  }

  servo.writeMicroseconds(currentStepPos);
}

// ===== PUBLIC API =====

void driveServo(int percent) {
  if (inManualMode) return;

  percent = constrain(percent, 0, 100);
  int target = LOWER_CURRENT_BOUND + (CURRENT_BOUND_DIFF * percent) / 100;
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