// Arduino Nano code for throttle stepper control with DM556 driver

#include <AccelStepper.h>

// DM556 driver pins
const int STEP_PIN = 3;      // Connect to PUL+ on DM556
const int DIR_PIN = 4;       // Connect to DIR+ on DM556
const int ENABLE_PIN = 5;    // Connect to ENA+ on DM556

// Throttle limit switch (detects idle position)
const int THROTTLE_SWITCH_PIN = 2;  // HIGH = throttle at idle, LOW = throttle engaged
const int CALIBRATE_BTN_PIN = 7;    // Manual calibration button

// Stepper 23HS32-4004S
const int STEPS_PER_REV = 200;       // 1.8° per step
const int MICROSTEPPING = 8;          // Match DM556 DIP switches (SW5=ON, SW6=OFF, SW7=OFF, SW8=ON for 800 pulses/rev)
const int STEPS_PER_MICROSTEP_REV = STEPS_PER_REV * MICROSTEPPING;  // 1600 steps/rev

// Throttle range (in microsteps from home position)
const long THROTTLE_MIN_STEPS = 0;
const long THROTTLE_MAX_STEPS = 800;  // ~0.5 rev with 8x microstepping

// Speed and acceleration (tune for smooth throttle response)
const float MAX_SPEED = 2000.0;       // steps per second
const float ACCELERATION = 4000.0;   // steps per second^2

// Safety: Communication watchdog timeout (milliseconds)
const unsigned long WATCHDOG_TIMEOUT = 2000;  // Auto-disable if no command for 2 seconds

// Global variables
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

int targetThrottlePercent = 0;
bool systemEnabled = false;
bool isCalibrated = false;
unsigned long lastCommandTime = 0;
bool watchdogEnabled = false;  // Disabled by default - enable with WATCHDOG:ON for production

// Setup
void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }  // Wait for serial connection
    
    // Configure stepper
    stepper.setMaxSpeed(MAX_SPEED);
    stepper.setAcceleration(ACCELERATION);
    stepper.setCurrentPosition(0);
    
    // Configure enable pin (DM556: LOW = enabled, HIGH = disabled)
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);  // Start disabled for safety
    
    // Configure switches
    pinMode(THROTTLE_SWITCH_PIN, INPUT_PULLUP);  // Limit switch at idle position
    pinMode(CALIBRATE_BTN_PIN, INPUT_PULLUP);    // Manual calibration button
    
    lastCommandTime = millis();
    
    Serial.println("THROTTLE_CONTROLLER_READY");
    Serial.println("Commands: T:0-100, ENABLE, DISABLE, CALIBRATE, HOME, STATUS, STOP, WATCHDOG:ON/OFF");
}

// Main loop
void loop() {
    handleSerial();
    
    // auto-disable if no commands received
    if (watchdogEnabled && systemEnabled && (millis() - lastCommandTime > WATCHDOG_TIMEOUT)) {
        Serial.println("WATCHDOG:TIMEOUT");
        emergencyStop();
    }
    
    if (systemEnabled) {
        stepper.run();
    }
    
    // Check calibration button (with debounce)
    if (digitalRead(CALIBRATE_BTN_PIN) == LOW) {
        delay(50);
        if (digitalRead(CALIBRATE_BTN_PIN) == LOW) {
            calibrate();
            while (digitalRead(CALIBRATE_BTN_PIN) == LOW);
        }
    }
}

// Check if throttle is at idle (switch closed)
bool isThrottleAtIdle() {
    return digitalRead(THROTTLE_SWITCH_PIN) == HIGH;
}

// Serial command handler
void handleSerial() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toUpperCase();  // Make commands case-insensitive
        
        lastCommandTime = millis();  // Reset watchdog timer
        
        if (command.startsWith("T:")) {
            // Throttle command: T:0 to T:100
            int percent = command.substring(2).toInt();
            setThrottlePercent(percent);
        }
        else if (command == "ENABLE") {
            enableMotor();
        }
        else if (command == "DISABLE") {
            disableMotor();
        }
        else if (command == "CALIBRATE") {
            calibrate();
        }
        else if (command == "HOME") {
            homeThrottle();
        }
        else if (command == "STATUS") {
            sendStatus();
        }
        else if (command == "STOP") {
            emergencyStop();
        }
        else if (command == "PING") {
            // Heartbeat command - keeps watchdog alive
            Serial.println("PONG");
        }
        else if (command == "WATCHDOG:ON") {
            watchdogEnabled = true;
            Serial.println("WATCHDOG:ENABLED");
        }
        else if (command == "WATCHDOG:OFF") {
            watchdogEnabled = false;
            Serial.println("WATCHDOG:DISABLED");
        }
        else {
            Serial.print("ERR:UNKNOWN_CMD:");
            Serial.println(command);
        }
    }
}

// Motor control functions
void enableMotor() {
    if (!isCalibrated) {
        Serial.println("ERR:CALIBRATE_FIRST");
        return;
    }
    digitalWrite(ENABLE_PIN, LOW);  // DM556: LOW = enabled
    systemEnabled = true;
    lastCommandTime = millis();
    Serial.println("OK:ENABLED");
}

void disableMotor() {
    digitalWrite(ENABLE_PIN, HIGH);  // DM556: HIGH = disabled
    systemEnabled = false;
    Serial.println("OK:DISABLED");
}

void setThrottlePercent(int percent) {
    if (!isCalibrated) {
        Serial.println("ERR:NOT_CALIBRATED");
        return;
    }
    if (!systemEnabled) {
        Serial.println("ERR:NOT_ENABLED");
        return;
    }
    
    percent = constrain(percent, 0, 100);
    targetThrottlePercent = percent;
    
    long targetSteps = map(percent, 0, 100, THROTTLE_MIN_STEPS, THROTTLE_MAX_STEPS);
    stepper.moveTo(targetSteps);
    
    Serial.print("OK:T:");
    Serial.println(percent);
}

void calibrate() {
    Serial.println("CALIBRATING...");
    
    // Disable motor during calibration for manual positioning
    digitalWrite(ENABLE_PIN, HIGH);
    
    // Reset positions - assumes throttle is at idle (0%) position
    stepper.setCurrentPosition(0);
    targetThrottlePercent = 0;
    isCalibrated = true;
    
    Serial.println("OK:CALIBRATED");
    Serial.println("NOTE:Position throttle at IDLE before calibrating");
}

void homeThrottle() {
    if (!systemEnabled) {
        Serial.println("ERR:NOT_ENABLED");
        return;
    }
    
    Serial.println("HOMING...");
    stepper.moveTo(0);
    
    // Block until home reached (with timeout)
    unsigned long homeStart = millis();
    while (stepper.distanceToGo() != 0) {
        stepper.run();
        if (millis() - homeStart > 5000) {  // 5 second timeout
            Serial.println("ERR:HOME_TIMEOUT");
            emergencyStop();
            return;
        }
    }
    
    targetThrottlePercent = 0;
    Serial.println("OK:HOME");
}

void emergencyStop() {
    stepper.stop();
    stepper.setCurrentPosition(stepper.currentPosition());
    targetThrottlePercent = getCurrentThrottlePercent();
    disableMotor();
    Serial.println("EMERGENCY_STOP");
}

int getCurrentThrottlePercent() {
    return map(stepper.currentPosition(), THROTTLE_MIN_STEPS, THROTTLE_MAX_STEPS, 0, 100);
}

void sendStatus() {
    Serial.print("STATUS:");
    Serial.print("pos=");
    Serial.print(stepper.currentPosition());
    Serial.print(",target=");
    Serial.print(stepper.targetPosition());
    Serial.print(",throttle=");
    Serial.print(targetThrottlePercent);
    Serial.print(",actual_throttle=");
    Serial.print(getCurrentThrottlePercent());
    Serial.print(",at_idle=");
    Serial.print(isThrottleAtIdle() ? "1" : "0");
    Serial.print(",enabled=");
    Serial.print(systemEnabled ? "1" : "0");
    Serial.print(",calibrated=");
    Serial.print(isCalibrated ? "1" : "0");
    Serial.print(",watchdog=");
    Serial.println(watchdogEnabled ? "1" : "0");
}