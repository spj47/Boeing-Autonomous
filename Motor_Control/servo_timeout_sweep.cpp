#include <Servo.h>

Servo servo;

const int SERVO_PIN = 9;

// Servo pulse limits (µs)
const int LOWER_BOUND  = 1300;
const int HIGHER_BOUND = 1700;
const int BOUND_DIFF   = HIGHER_BOUND - LOWER_BOUND;

// Motion handling
const int STEP_SIZE_US  = 8; // reduce second
const int STEP_DELAY_MS = 10; // increase first

// If no valid serial command arrives for this long, the servo will automatically
// sweep between LOW and HIGH every SWEEP_PERIOD_MS.
const unsigned long CMD_TIMEOUT_MS  = 2000;  // 2 seconds
const unsigned long SWEEP_PERIOD_MS = 3000;  // 3 seconds between direction flips
bool enableTimeoutSweep = true;

// State control
int currentPos             = LOWER_BOUND; // initialize to lower bound
bool inManualMode          = false;       // True when in manual mode (This stops the driver funciton from getting run when true!!)
bool servoMoving           = false;       // True when the servo is moving to a targetPos
int targetPos              = LOWER_BOUND; // Defines where we want the servo to be
unsigned long lastStepTime = 0;           // Defines the time since the last step of the servo motor

// Tracks last time we received a valid command
unsigned long lastCmdTime  = 0;

// Timeout sweep state
bool sweepToHigh                 = false;
unsigned long lastSweepToggleTime = 0;

// ===== CSV LOGGING =====
const unsigned long LOG_PERIOD_MS = 20;  // log at 50 Hz (adjust as needed)
unsigned long lastLogTime = 0;

void logCsvRow() {
  unsigned long now = millis();
  if (now - lastLogTime < LOG_PERIOD_MS) return;
  lastLogTime = now;

  // CSV row: ms,current_us,target_us,moving,manual,timeout_sweep
  Serial.print(now);
  Serial.print(",");
  Serial.print(currentPos);
  Serial.print(",");
  Serial.print(targetPos);
  Serial.print(",");
  Serial.print(servoMoving ? 1 : 0);
  Serial.print(",");
  Serial.print(inManualMode ? 1 : 0);
  Serial.print(",");
  Serial.println(enableTimeoutSweep ? 1 : 0);
}

void setup() {
  servo.attach(SERVO_PIN, LOWER_BOUND, HIGHER_BOUND);
  servo.writeMicroseconds(currentPos);

  Serial.begin(9600);

  lastCmdTime = millis();

  Serial.println("ms,current_us,target_us,moving,manual,timeout_sweep");
}

void loop() {
  handleSerial();
  handleTimeoutSweep();
  updateServo();
  //Serial.print('a');
  logCsvRow();
}

// ===== Servo Control =====

// Use to request the servo to move
void moveServo(int targetUs) {
  targetPos = constrain(targetUs, LOWER_BOUND, HIGHER_BOUND);
  servoMoving = true;
}

// Steps toward the current target
void updateServo() {
  if (!servoMoving) return;

  unsigned long now = millis();
  if (now - lastStepTime < (unsigned long)STEP_DELAY_MS) return;

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
  // Both subsystems using this motor have mechanical manual modes
  // So setting it to 0% means theres no electrical interference
  moveServo(LOWER_BOUND);
}

void handleTimeoutSweep() {
  if (!enableTimeoutSweep) return;
  if (inManualMode) return;

  unsigned long now = millis();
  if (now - lastCmdTime < CMD_TIMEOUT_MS) return;

  if (now - lastSweepToggleTime >= SWEEP_PERIOD_MS) {
    lastSweepToggleTime = now;
    sweepToHigh = !sweepToHigh;
    moveServo(sweepToHigh ? HIGHER_BOUND : LOWER_BOUND);
  }
}

void markCommandReceived() {
  lastCmdTime = millis();
  lastSweepToggleTime = lastCmdTime;
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
  if (cmd.length() == 0) return;

  if (cmd.startsWith("S:")) {
    int percent = cmd.substring(2).toInt();
    driveServo(percent);
    markCommandReceived();
    return;
  }

  else if (cmd == "T") {
    toggleManualMode();
    markCommandReceived();
    return;
  }

  else if (cmd.startsWith("M:")) {
    bool setManual = (cmd.substring(2).toInt() == 1);
    setManualMode(setManual);
    markCommandReceived();
    return;
  }

  if (cmd.startsWith("SWEEP:")) {
    int val = cmd.substring(6).toInt();
    enableTimeoutSweep = (val == 1);
    if (!enableTimeoutSweep) {
      moveServo(currentPos);
    }
    markCommandReceived();
    return;
  }
}