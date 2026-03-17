#include <Servo.h>

// Servo data
Servo servo;
const int SERVO_PIN = 9;

// Motion handling (Both of these combined determine how quickly the servo moves)
const int STEP_SIZE_US  = 5;
const int STEP_DELAY_MS = 3;

// Servo pulse limits (µs)
const int LOWER_BOUND  = 1000;
const int HIGHER_BOUND = 2000;
const int BOUND_DIFF   = HIGHER_BOUND - LOWER_BOUND;

// Servo State Control
int currentPos             = LOWER_BOUND; // initialize to lower bound
bool servoMoving           = false;       // True when the servo is moving to a targetPos 
int targetPos              = currentPos;  // Defines where we want the servo to be
unsigned long lastStepTime = 0;           // Defines the time since the last step of the servo motor

// Manual pins
const int MANUAL_DRIVER_FORWARD_PIN  = 2;
const int MANUAL_DRIVER_BACKWARD_PIN = 3;
const int MANUAL_ENABLE_BUTTON       = 4;
const int MANUAL_DISABLE_BUTTON      = 5;

// Manual State Control
bool inManualMode          = false;       // True when in manual mode (This stops the driver funciton from getting run when true!!)
bool actuatingManual       = false;       // True when linear actuator is in motion to stop other inputs
bool deactuatingManual     = false;       // True when linear actuator is in motion to stop other inputs

// Misc
const byte ALL_ADDRESS   = 0x00;
const byte LOCAL_ADDRESS = 0x01;
const int BAUD_RATE      = 9600;

void setup() {
  servo.attach(SERVO_PIN, LOWER_BOUND, HIGHER_BOUND);
  servo.writeMicroseconds(currentPos);

  pinMode(MANUAL_DRIVER_FORWARD_PIN, OUTPUT);
  pinMode(MANUAL_DRIVER_BACKWARD_PIN, OUTPUT);
  pinMode(MANUAL_ENABLE_BUTTON, INPUT);
  pinMode(MANUAL_DISABLE_BUTTON, INPUT);

  Serial.begin(BAUD_RATE);
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
    return digitalRead(pin);
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
  if (!(cmd.startsWith(ALL_ADDRESS) || cmd.startsWith(LOCAL_ADDRESS)))
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
