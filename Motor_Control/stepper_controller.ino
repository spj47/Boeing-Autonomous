/*
 * Arduino Nano controller for throttle actuation via stepper motor.
 * Supports MANUAL mode (potentiometer) and AUTO mode (Pi commands).
 * 
 * WIRING:
 *   Arduino Pin 3   -> DM556 PUL+ (step signal)
 *   Arduino Pin 4   -> DM556 DIR+ (direction signal)
 *   Arduino Pin 5   -> DM556 ENA+ (enable signal)
 *   Arduino Pin 2   -> Limit switch (to GND when triggered) [optional]
 *   Arduino Pin A1  -> Potentiometer wiper (10k pot between 5V and GND)
 *   Arduino RX0     -> Raspberry Pi TX (for commands)
 *   Arduino TX0     -> Raspberry Pi RX (for responses)
 *   Arduino GND     -> DM556 PUL-, DIR-, ENA-, Pi GND
 * 
 * DM556 CONNECTIONS:
 *   PUL+  -> Arduino Pin 3
 *   PUL-  -> Arduino GND
 *   DIR+  -> Arduino Pin 4  
 *   DIR-  -> Arduino GND
 *   ENA+  -> Arduino Pin 5
 *   ENA-  -> Arduino GND
 * 
 * POTENTIOMETER:
 *   Pin 1 -> GND
 *   Pin 2 (wiper) -> Arduino A1
 *   Pin 3 -> 5V
 * 
 * COMMANDS (via Serial):
 *   <0-100>    - Set throttle position (AUTO mode only)
 *   CAL        - Calibrate (set current position as 0/idle)
 *   STOP       - Emergency stop
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
// Negative value = counter-clockwise, Positive = clockwise
const long THROTTLE_MIN = 0;      // Steps at idle (0%)
const long THROTTLE_MAX = -6400;  // Steps at full throttle (100%) = 180° CCW

// potentiometer settings
const int POT_DEADZONE = 5;        // Ignore changes smaller than this (0-100 scale)
const int POT_READ_INTERVAL = 50;  // Read pot every 50ms
const int POT_SAMPLES = 5;         // Number of samples for median filter

// motion profile
// Tune these for smooth, responsive throttle actuation
// Higher values needed for 64x microstepping
const float SPEED_MAX = 12000.0;       // Max speed (steps/second)
const float ACCELERATION = 50000.0;    // High acceleration for snappy response

// serial communication
const long BAUD_RATE = 9600;   // Must match Pi's serial config

// Set to false to completely disable pot/manual mode (use AUTO only)
const bool ENABLE_MANUAL_MODE = true;

AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

// operating modes
enum Mode { MANUAL, AUTO };
Mode currentMode = MANUAL;    // Start in manual mode

bool isCalibrated = false;    // Has the system been calibrated?
bool isEnabled = false;       // Is the motor currently enabled?
int currentThrottle = 0;      // Current throttle percentage (0-100)
int lastPotThrottle = 0;      // Last pot reading (for deadzone check)
int stablePotValue = 0;       // Filtered stable pot value
unsigned long lastPotRead = 0; // Time of last pot reading

void setup() {
    // initialize serial for Pi communication
    Serial.begin(BAUD_RATE);
    
    // Use default internal reference (don't touch AREF pin)
    analogReference(DEFAULT);
    
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
    
    // Initialize pot reading to current value (prevents startup drift)
    if (ENABLE_MANUAL_MODE) {
        stablePotValue = readPotSmoothed();
        lastPotThrottle = map(stablePotValue, 0, 1023, 0, 100);
    }
    
    // Auto-calibrate on startup - current position is home (0%)
    isCalibrated = true;
    
    Serial.print("READY:");
    Serial.println(currentMode == MANUAL ? "MANUAL" : "AUTO");
}

// main loop
void loop() {
    // process any incoming serial commands
    processSerial();
    
    // in manual mode, read potentiometer (if enabled)
    if (ENABLE_MANUAL_MODE && currentMode == MANUAL) {
        processManualControl();
    }
    
    // run stepper (non-blocking - must be called frequently)
    if (isEnabled) {
        stepper.run();
    }
}

// read potentiometer with median filter to reject noise spikes
int readPotSmoothed() {
    int samples[POT_SAMPLES];
    
    // Collect samples
    for (int i = 0; i < POT_SAMPLES; i++) {
        samples[i] = analogRead(PIN_POT);
        delayMicroseconds(500);  // Small delay between samples
    }
    
    // Sort samples (simple bubble sort - fine for small arrays)
    for (int i = 0; i < POT_SAMPLES - 1; i++) {
        for (int j = 0; j < POT_SAMPLES - i - 1; j++) {
            if (samples[j] > samples[j + 1]) {
                int temp = samples[j];
                samples[j] = samples[j + 1];
                samples[j + 1] = temp;
            }
        }
    }
    
    // Return median (middle value) - rejects outliers/spikes
    return samples[POT_SAMPLES / 2];
}

// read potentiometer and set throttle in manual mode
void processManualControl() {
    // rate limit pot reads
    if (millis() - lastPotRead < POT_READ_INTERVAL) return;
    lastPotRead = millis();
    
    // read pot with median filter (no additional smoothing)
    int rawPot = readPotSmoothed();
    
    int potThrottle = map(rawPot, 0, 1023, 0, 100);
    
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
    else if (cmdUpper == "AUTO") {
        currentMode = AUTO;
        Serial.println("MODE:AUTO");
    }
    else if (cmdUpper == "MANUAL") {
        if (ENABLE_MANUAL_MODE) {
            currentMode = MANUAL;
            Serial.println("MODE:MANUAL");
        } else {
            Serial.println("ERR:MANUAL_DISABLED");
        }
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
// no calibration required - treats current position as relative reference
void setThrottleManual(int percent) {
    // auto-calibrate on first call (wherever the pot is becomes the starting point)
    if (!isCalibrated) {
        stepper.setCurrentPosition(0);
        isCalibrated = true;
        Serial.println("AUTO_CAL");
    }
    
    // clamp to valid range
    percent = constrain(percent, 0, 100);
    currentThrottle = percent;
    
    // convert percentage to step position
    long targetSteps = map(percent, 0, 100, THROTTLE_MIN, THROTTLE_MAX);
    
    // enable motor and set target (non-blocking)
    enableMotor();
    stepper.moveTo(targetSteps);
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