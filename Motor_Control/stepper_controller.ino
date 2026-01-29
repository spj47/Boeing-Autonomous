/*
 * Arduino Nano controller for throttle actuation via stepper motor.
 * Receives throttle commands (0-100%) from Raspberry Pi over hardware serial.
 * 
 * WIRING:
 *   Arduino Pin 3  -> DM556 PUL+ (step signal)
 *   Arduino Pin 4  -> DM556 DIR+ (direction signal)
 *   Arduino Pin 5  -> DM556 ENA+ (enable signal)
 *   Arduino Pin 2  -> Limit switch (to GND when triggered)
 *   Arduino RX0    -> Raspberry Pi TX (for commands)
 *   Arduino TX0    -> Raspberry Pi RX (for responses)
 *   Arduino GND    -> DM556 PUL-, DIR-, ENA-, Pi GND
 * 
 * COMMANDS (via Serial from Pi):
 *   <0-100>    - Set throttle position (just send the number, e.g., "50")
 *   CAL        - Calibrate (set current position as 0/idle)
 *   STOP       - Emergency stop - immediately halt and disable
 *   STATUS     - Report current system state
 */

#include <AccelStepper.h>

// pin assignments
const int PIN_STEP   = 3;    // DM556 PUL+ (pulse/step signal)
const int PIN_DIR    = 4;    // DM556 DIR+ (direction signal)  
const int PIN_ENABLE = 5;    // DM556 ENA+ (enable signal)
const int PIN_LIMIT  = 2;    // Limit switch (detects idle position)

// motor & driver settings
// NEMA23 23HS32-4004S: 200 steps/rev (1.8° per step)
// DM556 microstepping: Set via DIP switches on driver
const int MICROSTEPS = 8;                    // Match DM556 DIP switch setting
const int STEPS_PER_REV = 200 * MICROSTEPS;  // 1600 steps/rev at 8x microstepping

// throttle range
const long THROTTLE_MIN = 0;     // Steps at idle (0%)
const long THROTTLE_MAX = 800;   // Steps at full throttle (100%)

// motion profile
// Tune these for smooth, responsive throttle actuation
const float SPEED_MAX = 2000.0;        // Max speed (steps/second)
const float ACCELERATION = 4000.0;     // Acceleration (steps/second²)

// serial communication
const long BAUD_RATE = 9600;   // Must match Pi's serial config

AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

bool isCalibrated = false;    // Has the system been calibrated?
bool isEnabled = false;       // Is the motor currently enabled?
int currentThrottle = 0;      // Current throttle percentage (0-100)

void setup() {
    // initialize serial for Pi communication
    Serial.begin(BAUD_RATE);
    
    // configure stepper motor
    stepper.setMaxSpeed(SPEED_MAX);
    stepper.setAcceleration(ACCELERATION);
    stepper.setCurrentPosition(0);
    
    // configure enable pin - DM556: LOW = enabled, HIGH = disabled
    pinMode(PIN_ENABLE, OUTPUT);
    disableMotor();  // Start disabled for safety
    
    // configure limit switch with internal pull-up
    // switch connects pin to GND when triggered (idle position)
    pinMode(PIN_LIMIT, INPUT_PULLUP);
    
    Serial.println("READY");
}

// main loop
void loop() {
    // process any incoming serial commands
    processSerial();
    
    // run stepper (non-blocking - must be called frequently)
    if (isEnabled) {
        stepper.run();
    }
}

// serial command processing
void processSerial() {
    if (!Serial.available()) return;
    
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    // check for text commands first
    String cmdUpper = cmd;
    cmdUpper.toUpperCase();
    
    if (cmdUpper == "CAL") {
        calibrate();
    }
    else if (cmdUpper == "STOP") {
        emergencyStop();
    }
    else if (cmdUpper == "STATUS") {
        sendStatus();
    }
    // otherwise, treat as throttle value (0-100)
    else {
        int percent = cmd.toInt();
        // Validate: toInt() returns 0 for non-numeric, so check if input is actually "0"
        if (percent == 0 && cmd != "0") {
            Serial.println("ERR:UNKNOWN");
        } else {
            setThrottle(percent);
        }
    }
}

// motor control functions
// enable stepper motor driver
void enableMotor() {
    digitalWrite(PIN_ENABLE, LOW);  // DM556: LOW = enabled
    isEnabled = true;
}

// disable stepper motor driver
void disableMotor() {
    digitalWrite(PIN_ENABLE, HIGH);  // DM556: HIGH = disabled
    isEnabled = false;
}

// set throttle position (0-100%)
// non-blocking
void setThrottle(int percent) {
    // must be calibrated first
    if (!isCalibrated) {
        Serial.println("ERR:NOT_CAL");
        return;
    }
    
    // clamp to valid range
    percent = constrain(percent, 0, 100);
    currentThrottle = percent;
    
    // convert percentage to step position
    long targetSteps = map(percent, 0, 100, THROTTLE_MIN, THROTTLE_MAX);
    
    // enable motor and set target (non-blocking)
    enableMotor();
    stepper.moveTo(targetSteps);
    
    Serial.print("OK:");
    Serial.println(percent);
}

// calibrate the system
// sets current position as idle (0%)
// note: manually position throttle to idle before calling
void calibrate() {
    disableMotor();
    stepper.setCurrentPosition(0);
    currentThrottle = 0;
    isCalibrated = true;
    Serial.println("OK:CAL");
}

// emergency stop
// immediately halts motor and disables driver
void emergencyStop() {
    stepper.stop();                                    // decelerate to stop
    stepper.setCurrentPosition(stepper.currentPosition());  // clear target
    disableMotor();
    currentThrottle = getActualThrottle();
    Serial.println("STOPPED");
}


// check if limit switch is triggered (throttle at idle)
bool isAtIdle() {
    return digitalRead(PIN_LIMIT) == LOW;  // LOW when switch triggered (pulled to GND)
}

// get actual throttle percentage based on current step position
int getActualThrottle() {
    return map(stepper.currentPosition(), THROTTLE_MIN, THROTTLE_MAX, 0, 100);
}

// send current system status
void sendStatus() {
    Serial.print("S:");
    Serial.print(currentThrottle);           // target throttle %
    Serial.print(",");
    Serial.print(getActualThrottle());       // actual throttle %
    Serial.print(",");
    Serial.print(isEnabled ? "1" : "0");     // motor enabled
    Serial.print(",");
    Serial.print(isCalibrated ? "1" : "0");  // calibrated
    Serial.print(",");
    Serial.println(isAtIdle() ? "1" : "0");  // at idle position
}