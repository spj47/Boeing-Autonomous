/*
 * Arduino Nano controller for throttle actuation via stepper motor.
 * Supports MANUAL mode (potentiometer/lever) and AUTO mode (Pi commands).
 * 
 * MODES:
 *   MANUAL (default) - Potentiometer lever controls throttle directly
 *   AUTO             - Raspberry Pi sends throttle commands via serial
 * 
 * WIRING:
 *   Arduino Pin 3   -> DM556 PUL+ (step signal)
 *   Arduino Pin 4   -> DM556 DIR+ (direction signal)
 *   Arduino Pin 5   -> DM556 ENA+ (enable signal)
 *   Arduino Pin 2   -> Limit switch (to GND when triggered) [optional]
 *   Arduino Pin A0  -> Potentiometer wiper (10k pot between 5V and GND)
 *   Arduino RX0     -> Raspberry Pi TX (for commands)
 *   Arduino TX0     -> Raspberry Pi RX (for responses)
 *   Arduino GND     -> DM556 PUL-, DIR-, ENA-, Pi GND
 * 
 * COMMANDS (via Serial):
 *   <0-100>    - Set throttle position (just send the number, e.g., "50")
 *   CAL        - Calibrate (set current position as 0/idle)
 *   STOP       - Emergency stop - immediately halt and disable
 *   STATUS     - Report current system state
 *   AUTO       - Switch to autonomous mode (Pi control)
 *   MANUAL     - Switch to manual mode (pot control)
 */

#include <AccelStepper.h>

// pin assignments
const int PIN_STEP   = 3;    // DM556 PUL+ (pulse/step signal)
const int PIN_DIR    = 4;    // DM556 DIR+ (direction signal)  
const int PIN_ENABLE = 5;    // DM556 ENA+ (enable signal)
const int PIN_LIMIT  = 2;    // Limit switch (detects idle position) [optional]
const int PIN_POT    = A0;   // Potentiometer for manual throttle control

// motor & driver settings
// NEMA23 23HS32-4004S: 200 steps/rev (1.8° per step)
// DM556 microstepping: Set via DIP switches on driver
// Your setting: 64x microstepping = 12800 pulses/rev
const int MICROSTEPS = 64;                   // Matches your DM556 DIP switch setting
const int STEPS_PER_REV = 200 * MICROSTEPS;  // 12800 steps/rev at 64x microstepping

// throttle range
// 6400 steps = 180° rotation (half revolution)
// Adjust THROTTLE_MAX based on how much travel your throttle needs
const long THROTTLE_MIN = 0;     // Steps at idle (0%)
const long THROTTLE_MAX = 6400;  // Steps at full throttle (100%) = 180° rotation

// potentiometer settings
const int POT_DEADZONE = 3;      // Ignore changes smaller than this (0-100 scale)
const int POT_READ_INTERVAL = 50; // Read pot every 50ms to reduce noise

// motion profile
// Tune these for smooth, responsive throttle actuation
// Higher values needed for 64x microstepping
const float SPEED_MAX = 8000.0;        // Max speed (steps/second) ~0.6 rev/sec
const float ACCELERATION = 16000.0;    // Acceleration (steps/second²)

// serial communication
const long BAUD_RATE = 9600;   // Must match Pi's serial config

// debug mode - set to false for normal operation
const bool DEBUG = false;

AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

// operating modes
enum Mode { MANUAL, AUTO };
Mode currentMode = MANUAL;    // Start in manual mode for testing

bool isCalibrated = false;    // Has the system been calibrated?
bool isEnabled = false;       // Is the motor currently enabled?
int currentThrottle = 0;      // Current throttle percentage (0-100)
int lastPotThrottle = 0;      // Last pot reading (for deadzone check)
unsigned long lastPotRead = 0; // Time of last pot reading

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
    
    Serial.print("READY:");
    Serial.println(currentMode == MANUAL ? "MANUAL" : "AUTO");
}

// main loop
void loop() {
    // process any incoming serial commands
    processSerial();
    
    // in manual mode, read potentiometer
    if (currentMode == MANUAL) {
        processManualControl();
    }
    
    // run stepper (non-blocking - must be called frequently)
    if (isEnabled) {
        stepper.run();
    }
}

// read potentiometer and set throttle in manual mode
void processManualControl() {
    // rate limit pot reads
    if (millis() - lastPotRead < POT_READ_INTERVAL) return;
    lastPotRead = millis();
    
    // read pot and convert to 0-100%
    int rawPot = analogRead(PIN_POT);
    int potThrottle = map(rawPot, 0, 1023, 0, 100);
    
    // debug output
    if (DEBUG && (abs(potThrottle - lastPotThrottle) >= POT_DEADZONE)) {
        Serial.print("POT:");
        Serial.print(rawPot);
        Serial.print(" -> ");
        Serial.print(potThrottle);
        Serial.println("%");
    }
    
    // apply deadzone to prevent jitter
    if (abs(potThrottle - lastPotThrottle) < POT_DEADZONE) return;
    lastPotThrottle = potThrottle;
    
    // set throttle (bypasses calibration check in manual mode for testing)
    setThrottleManual(potThrottle);
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
    else if (cmdUpper == "TEST") {
        testMotor();
    }
    else if (cmdUpper == "AUTO") {
        currentMode = AUTO;
        Serial.println("MODE:AUTO");
    }
    else if (cmdUpper == "MANUAL") {
        currentMode = MANUAL;
        Serial.println("MODE:MANUAL");
    }
    // throttle commands only work in AUTO mode
    else if (currentMode == AUTO) {
        int percent = cmd.toInt();
        // Validate: toInt() returns 0 for non-numeric, so check if input is actually "0"
        if (percent == 0 && cmd != "0") {
            Serial.println("ERR:UNKNOWN");
        } else {
            setThrottle(percent);
        }
    }
    else {
        // in manual mode, ignore numeric commands (pot has control)
        Serial.println("ERR:MANUAL_MODE");
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

// set throttle position (0-100%) - AUTO mode
// non-blocking, requires calibration
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

// set throttle position (0-100%) - MANUAL mode
// bypasses calibration for testing, auto-calibrates on first use
void setThrottleManual(int percent) {
    // auto-calibrate if needed (assumes pot at 0 = idle position)
    if (!isCalibrated && percent < 5) {
        stepper.setCurrentPosition(0);
        isCalibrated = true;
        Serial.println("AUTO_CAL");
    }
    
    // if still not calibrated, wait for user to move lever to idle
    if (!isCalibrated) {
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
    
    if (DEBUG) {
        Serial.print("MOVE:");
        Serial.print(stepper.currentPosition());
        Serial.print(" -> ");
        Serial.print(targetSteps);
        Serial.print(" (enabled=");
        Serial.print(isEnabled);
        Serial.println(")");
    }
}

// test motor function - sends pulses directly for diagnostics
void testMotor() {
    Serial.println("TEST:Sending 12800 pulses (1 full rev at 64x microstepping)...");
    
    // Enable the driver
    digitalWrite(PIN_ENABLE, LOW);
    delay(100);
    
    // Set direction
    digitalWrite(PIN_DIR, HIGH);
    
    // Send 12800 pulses (1 full revolution)
    for (long i = 0; i < 12800; i++) {
        digitalWrite(PIN_STEP, HIGH);
        delayMicroseconds(50);
        digitalWrite(PIN_STEP, LOW);
        delayMicroseconds(50);
        
        if (i % 3200 == 0 && i > 0) {
            Serial.print("TEST:");
            Serial.print((i * 100) / 12800);
            Serial.println("%");
        }
    }
    
    Serial.println("TEST:Complete - motor should have made 1 full revolution");
    Serial.println("TEST:Type STOP to disable motor");
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
    Serial.print(isAtIdle() ? "1" : "0");    // at idle position
    Serial.print(",");
    Serial.println(currentMode == MANUAL ? "M" : "A");  // mode
}