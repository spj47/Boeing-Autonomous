// arduino nano code for throttle stepper control with DM556 driver

#include <AccelStepper.h>

// DM556 driver pins
const int STEP_PIN = 3;
const int DIR_PIN = 4;
const int ENABLE_PIN = 5;   // connect to ENA+ if used

// KY040 rotary encoder pins
const int ENCODER_CLK = 2;  // Must be interrupt-capable
const int ENCODER_DT = 6;

// adjust these based on your mechanical setup
const int STEPS_PER_REV = 200;  // NEMA 23 standard (1.8° per step)
const int MICROSTEPPING = 8;    // Set on DM556 dip switches
const int STEPS_PER_MICROSTEP_REV = STEPS_PER_REV * MICROSTEPPING;

// throttle range (in steps from home position)
const long THROTTLE_MIN_STEPS = 0;
const long THROTTLE_MAX_STEPS = 800;    // adjust based on your mechanism

// speed and acceleration
const float MAX_SPEED = 2000.0; // steps per second
const float ACCELERATION = 4000.0;  // steps per second^2

// global variables
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

volatile long encoderPosition = 0;
volatile int lastEncoderCLK = HIGH;

int targetThrottlePercent = 0;
bool systemEnabled = false;
bool isCalibrated = false;

// encoder ISR
void encoderISR() {
    int clkState = digitalRead(ENCODER_CLK);
    if (clkState != lastEncoderCLK && clkState == LOW) {
        if (digitalRead(ENCODER_DT) != clkState) {
            encoderPosition++;
        } else {
            encoderPosition--;
        }
    }
    lastEncoderCLK = clkState;
}

// setup
void setup() {
    Serial.begin(115200);
    
    // configure stepper
    stepper.setMaxSpeed(MAX_SPEED);
    stepper.setAcceleration(ACCELERATION);
    stepper.setCurrentPosition(0);
    
    // configure enable pin
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);  // HIGH = disabled on most drivers
    
    // configure encoder
    pinMode(ENCODER_CLK, INPUT_PULLUP);
    pinMode(ENCODER_DT, INPUT_PULLUP);
    pinMode(ENCODER_SW, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), encoderISR, CHANGE);
    
    Serial.println("THROTTLE_READY");
}

// main loop
void loop() {
    handleSerial();
    
    if (systemEnabled) {
        stepper.run();
    }
    
    // Check calibration button
    if (digitalRead(ENCODER_SW) == LOW) {
        delay(50);  // Debounce
        if (digitalRead(ENCODER_SW) == LOW) {
            calibrate();
            while (digitalRead(ENCODER_SW) == LOW);  // Wait for release
        }
    }
}

// serial command handler
void handleSerial() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
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
    }
}

// motor control functions
void enableMotor() {
    digitalWrite(ENABLE_PIN, LOW);  // LOW = enabled on most drivers
    systemEnabled = true;
    Serial.println("OK:ENABLED");
}

void disableMotor() {
    digitalWrite(ENABLE_PIN, HIGH);
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
    
    // reset positions
    stepper.setCurrentPosition(0);
    encoderPosition = 0;
    isCalibrated = true;
    
    Serial.println("CALIBRATED");
}

void homeThrottle() {
    if (!systemEnabled) {
        Serial.println("ERR:NOT_ENABLED");
        return;
    }
    
    Serial.println("HOMING...");
    stepper.moveTo(0);
    
    // block until home reached
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
    
    targetThrottlePercent = 0;
    Serial.println("OK:HOMED");
}

void emergencyStop() {
    stepper.stop();
    stepper.setCurrentPosition(stepper.currentPosition());
    disableMotor();
    Serial.println("OK:STOPPED");
}

void sendStatus() {
    Serial.print("STATUS:");
    Serial.print("pos=");
    Serial.print(stepper.currentPosition());
    Serial.print(",target=");
    Serial.print(stepper.targetPosition());
    Serial.print(",throttle=");
    Serial.print(targetThrottlePercent);
    Serial.print(",encoder=");
    Serial.print(encoderPosition);
    Serial.print(",enabled=");
    Serial.print(systemEnabled ? "1" : "0");
    Serial.print(",calibrated=");
    Serial.println(isCalibrated ? "1" : "0");
}