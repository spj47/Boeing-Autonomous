/*
 * arduino nano controller for throttle actuation via stepper motor.
 * NEMA23 with DM556 driver, 64x microstepping.
 *
 * commands (via serial):
 *   02:S:<0-100>   - set throttle position (AUTO mode only)
 *   02:CAL         - calibrate (set current position as 0/idle)
 *   02:E           - emergency stop
 *   02:T           - Toggles manual mode
 *   02:M:<0-1>     - Sets manual mode
 *
 *   [DEPRECIATED] STATUS     - report current system state
 *   [DEPRECIATED] PEDAL        - toggle pedal reading stream
 *   [DEPRECIATED] AUTO       - switch to autonomous mode (Pi control)
 *   [DEPRECIATED] MANUAL     - switch to manual mode (PEDAL control)
 * 
 * Error Codes:
 *   0x00 - No Error 
 *   0x01 - (ERR:MANUAL_DISABLED) | System attempted to enter manual mode, but manual mode is disabled
 *   0x02 - (ERR:UNKNOWN)         | A serial command to set the throttle was recieved, but the input is invalid
 *   0x03 - (ERR:NOT_CAL)         | A serial command to set the throttle was recieved, but the system is not yet calibrated
 *   0x04 - (ERR:MANUAL_MODE)     | A serial command to set the throttle was recieved, but the system is in manual mode
 *   0x05 - (ERR:EMERGENCY_MODE)  | A serial command to set the throttle was recieved, but the system is in emergency mode and must be recalibrated
 */

#include <AccelStepper.h>

// pin assignments
const int PIN_STEP   = 3;    // DM556 PUL+
const int PIN_DIR    = 4;    // DM556 DIR+
const int PIN_ENABLE = 5;    // DM556 ENA+
const int PIN_PEDAL    = A0;   // Pelda for manual control
const int PIN_HALL_LEVER = A1;

// Pelda settings
const int PEDAL_MIN = 150;
const int PEDAL_MAX = 900;
const int PEDAL_DEADZONE = 5;
const int PEDAL_READ_INTERVAL = 50;
const int PEDAL_SAMPLES = 5;
const int PEDAL_SAMPLE_DELAY_US = 500;
const int PEDAL_STREAM_INTERVAL = 50;

// throttle curve settings
// 0.0 = linear, positive = more gradual at low end
const float THROTTLE_CURVE = 0.4;
const int THROTTLE_LIMIT = 100;
const float PERCENT_SCALE = 100.0;

// motion profile (tuned for 64x microstepping)
const float SPEED_MAX = 12000.0;
const float ACCELERATION = 50000.0;

// serial communication
const int BAUD_RATE = 9600;
const byte ALL_ADDRESS   = 0x00;
const byte LOCAL_ADDRESS = 0x02;

// set false to completely disable PEDAL/manual mode
const bool ENABLE_MANUAL_MODE = true;

// Hall Effect Values
float throttleMin;
float throttleMax;
const float hallBuffer = 20;

// Stepper Meta Data
long currentStep;
float currentPercentTarget;
const long STEP_SIZE = 10;
bool isEmergencyStopped;
byte currentErrorCode = 0x00;

// ----> MUST BE <FALSE> IN ALL CASES THAT ARE NOT ISOLATED TESTS<-------
const bool IS_DEBUG = false;
// ----> MUST BE <FALSE> IN ALL CASES THAT ARE NOT ISOLATED TESTS<-------

AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

// operating modes
enum Mode { MANUAL, AUTO };

// note: arduino requires global state for setup()/loop() architecture
Mode currentMode = MANUAL;
bool isCalibrated = false;
bool isEnabled = false;
int currentThrottle = 0;
int lastPEDALThrottle = 0;
unsigned long lastPEDALRead = 0;
bool PEDALStream = false;

void setup()
{
    Serial.begin(BAUD_RATE);
    analogReference(DEFAULT);

    // configure stepper motor
    stepper.setMaxSpeed(SPEED_MAX);
    stepper.setAcceleration(ACCELERATION);
    stepper.setCurrentPosition(0);

    // DM556: LOW = enabled, HIGH = disabled
    pinMode(PIN_ENABLE, OUTPUT);
    disableMotor();

    // initialize PEDAL reading to prevent startup drift
    if (ENABLE_MANUAL_MODE)
    {
        int smoothedPEDAL = readPEDALSmoothed();
        int clampedPEDAL = constrain(smoothedPEDAL, PEDAL_MIN, PEDAL_MAX);
        lastPEDALThrottle = map(clampedPEDAL, PEDAL_MIN, PEDAL_MAX, 0, 100);
    }

    // current position is home (0%)
    pinMode(PIN_HALL_LEVER, INPUT);
    
    calibrate();

    // report ready state
    if (IS_DEBUG)
    {
        Serial.print("READY:");
        if (currentMode == MANUAL)
        {
            Serial.println("MANUAL");
        }
        else
        {
            Serial.println("AUTO");
        }
    }
}

// main loop
void loop()
{
    processSerial();

    // stream PEDAL readings when enabled
    if (PEDALStream)
    {
        int PEDALReading = analogRead(PIN_PEDAL);
        if (IS_DEBUG)
        {
            Serial.println(PEDALReading);
        }
        delay(PEDAL_STREAM_INTERVAL);
    }

    // manual mode PEDAL control
    if (ENABLE_MANUAL_MODE && currentMode == MANUAL)
    {
        processManualControl();
    }

    // run stepper (non-blocking, must be called frequently)
    if (isEnabled)
    {
        moveThrottle();
        stepper.run();
    }
}

// apply throttle curve for finer control at low throttle
// input: 0-100, returns: 0-100 curved output
int applyThrottleCurve(int input)
{
    if (THROTTLE_CURVE <= 0.0)
    {
        return input;
    }

    // normalize to 0.0-1.0 range (mathematical formula)
    float x = input / PERCENT_SCALE;

    // x^(1 + curve) gives more resolution at low end
    float curved = pow(x, 1.0 + THROTTLE_CURVE);

    // blend between linear and curved response
    float result = (1.0 - THROTTLE_CURVE) * x + THROTTLE_CURVE * curved;

    // scale back to percentage and apply limit
    int output = (int)(result * THROTTLE_LIMIT);
    return constrain(output, 0, THROTTLE_LIMIT);
}

// read Pelda with median filter to reject noise spikes
int readPEDALSmoothed()
{
    int samples[PEDAL_SAMPLES];

    // collect samples with small delay between reads
    for (int i = 0; i < PEDAL_SAMPLES; i++)
    {
        samples[i] = analogRead(PIN_PEDAL);
        delayMicroseconds(PEDAL_SAMPLE_DELAY_US);
    }

    // bubble sort for median extraction (fine for small arrays)
    for (int i = 0; i < PEDAL_SAMPLES - 1; i++)
    {
        for (int j = 0; j < PEDAL_SAMPLES - i - 1; j++)
        {
            if (samples[j] > samples[j + 1])
            {
                int temp = samples[j];
                samples[j] = samples[j + 1];
                samples[j + 1] = temp;
            }
        }
    }

    // return median value
    return samples[PEDAL_SAMPLES / 2];
}

// read Pelda and update throttle in manual mode
void processManualControl()
{
    // rate limit PEDAL reads
    unsigned long currentTime = millis();
    if (currentTime - lastPEDALRead < PEDAL_READ_INTERVAL)
    {
        return;
    }
    lastPEDALRead = currentTime;

    int rawPEDAL = readPEDALSmoothed();

    // constrain to valid range and map to percentage
    rawPEDAL = constrain(rawPEDAL, PEDAL_MIN, PEDAL_MAX);
    int PEDALThrottle = map(rawPEDAL, PEDAL_MIN, PEDAL_MAX, 0, 100);

    // apply deadzone to prevent jitter
    int difference = abs(PEDALThrottle - lastPEDALThrottle);
    if (difference < PEDAL_DEADZONE)
    {
        return;
    }
    lastPEDALThrottle = PEDALThrottle;

    int curvedThrottle = applyThrottleCurve(PEDALThrottle);

    // auto-calibrate on first call if needed
    if (!isCalibrated)
    {
        calibrate();
        if (IS_DEBUG)
        {
            Serial.println("AUTO_CAL");
        }
    }

    moveToThrottle(curvedThrottle);
}

// serial command processing (handles all serial I/O)
void processSerial()
{
    if (!Serial.available())
    {
        return;
    }

    String cmd = Serial.readStringUntil('\n');
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

    String cmdUpper = cmd;
    cmdUpper.toUpperCase();

    if (cmdUpper == "CAL")
    {
        calibrate();
        if (IS_DEBUG)
        {
            Serial.println("OK:CAL");
        }
    }
    else if (isEmergencyStopped)
    {
        if (IS_DEBUG)
        {
            Serial.println("ERR:EMERGENCY_MODE");
        }
        currentErrorCode = 0x05;
    }
    else if (cmdUpper == "E")
    {
        emergencyStop();
        if (IS_DEBUG)
        {
            Serial.println("STOPPED");
        }
    }
    else if (cmdUpper == "STATUS")
    {
        sendStatus();
    }
    else if (cmdUpper == "PEDAL")
    {
        PEDALStream = !PEDALStream;
        if (!PEDALStream && IS_DEBUG)
        {
            Serial.println("PEDAL:OFF");
        }
    }
    else if (cmdUpper == "AUTO")
    {
        currentMode = AUTO;
        if (IS_DEBUG)
        {
            Serial.println("MODE:AUTO");
        }
    }
    else if (cmdUpper == "MANUAL")
    {
        if (ENABLE_MANUAL_MODE)
        {
            currentMode = MANUAL;
            if (IS_DEBUG)
            {
                Serial.println("MODE:MANUAL");
            }
        }
        else
        {
            if (IS_DEBUG)
            {
                Serial.println("ERR:MANUAL_DISABLED");
            }
            currentErrorCode = 0x01;
        }
    }
    else if (currentMode == AUTO && cmd.startsWith("S:"))
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
            if (IS_DEBUG)
            {
                Serial.println("ERR:UNKNOWN");
            }
            currentErrorCode = 0x02;
        }
        else if (!isCalibrated)
        {
            if (IS_DEBUG)
            {
                Serial.println("ERR:NOT_CAL");
            }
            currentErrorCode = 0x03;
        }
        else
        {
            int curvedPercent = applyThrottleCurve(percent);
            moveToThrottle(curvedPercent);

            if (IS_DEBUG)
            {
                Serial.print("OK:");
                Serial.println(curvedPercent);
            }
        }
    }
    else if (cmd == "T") 
    {
        setManualMode(currentMode == AUTO);
    }
    else if (cmd.startsWith("M:")) {
        bool setManual = (cmd.substring(2).toInt() == 1);
        setManualMode(setManual);
    }
    else
    {
        if (IS_DEBUG)
        {
            Serial.println("ERR:MANUAL_MODE");
        }
        currentErrorCode = 0x04;
    }
}

// Sets the system in and out of manual mode
void setManualMode(bool isSetManual)
{
    if (isSetManual)
    {
        if (ENABLE_MANUAL_MODE)
        {
            currentMode = MANUAL;
            if (IS_DEBUG)
            {
                Serial.println("MODE:MANUAL");
            }
        }
        else
        {
            if (IS_DEBUG)
            {
                Serial.println("ERR:MANUAL_DISABLED");
            }
            currentErrorCode = 0x01;
        }
    }
    else 
    {   
        currentMode = AUTO;
        if (IS_DEBUG)
        {
            Serial.println("MODE:AUTO");
        }
    }
}

// enable stepper motor driver
void enableMotor()
{
    digitalWrite(PIN_ENABLE, LOW);
    isEnabled = true;
}

// disable stepper motor driver
void disableMotor()
{
    digitalWrite(PIN_ENABLE, HIGH);
    isEnabled = false;
}

// move stepper to the given throttle percentage (0-100)
void moveToThrottle(int percent)
{
    percent = constrain(percent, 0, 100);
    currentThrottle = percent;

    currentPercentTarget = map(percent, 0, 100, throttleMin, throttleMax);
}

void moveThrottle()
{
    float currentHallValue = analogRead(PIN_HALL_LEVER);

    if (currentPercentTarget - currentHallValue > hallBuffer)
    {
        currentStep -= STEP_SIZE;
    }
    else if (currentPercentTarget - currentHallValue < -hallBuffer)
    {
        currentStep += STEP_SIZE;
    }
    else 
    {
        return;
    }

    enableMotor();
    stepper.moveTo(currentStep);
}

// calibrate: set current position as idle (0%)
// manually position throttle to idle before calling
void calibrate()
{
    disableMotor();
    stepper.setCurrentPosition(0);
    currentThrottle = 0;
    throttleMin = analogRead(PIN_HALL_LEVER);
    throttleMax = throttleMin + 100;
    currentStep = 0;
    currentPercentTarget = 0;
    isEmergencyStopped = false;
    isCalibrated = true;
}

// emergency stop: halt motor and disable driver
void emergencyStop()
{
    // Set Stepper to go to 0
    moveToThrottle(0);
    isEmergencyStopped = true;
}

// get actual throttle percentage based on step position
int getActualThrottle()
{
    long currentPos = analogRead(PIN_HALL_LEVER);
    return map(currentPos, throttleMin, throttleMax, 0, 100);
}

// send current system status over serial
void sendStatus()
{
    int actualThrottle = getActualThrottle();

    if (IS_DEBUG)
    { Serial.print("Requested Throttle:"); }
    else 
    { Serial.print("S:"); }
    Serial.print(currentThrottle);

    if (IS_DEBUG)
    { Serial.print(" | Actual Throttle: "); }
    else 
    { Serial.print(":"); }
    Serial.print(actualThrottle);

    if (IS_DEBUG)
    { Serial.print(" | In Emergency Mode: "); }
    else 
    { Serial.print(":"); }
    Serial.print(isEmergencyStopped);

    if (IS_DEBUG) 
    { Serial.println(""); }

    if (IS_DEBUG)
    { Serial.print("Is Enabled: "); }
    else 
    { Serial.print(":"); }
    Serial.print(isEnabled);

    if (IS_DEBUG)
    { Serial.print(" | Is Calibrated: "); }
    else 
    { Serial.print(":"); }
    Serial.print(isCalibrated);

    if (IS_DEBUG)
    { Serial.print(" | Is Manual Mode: "); }
    else 
    { Serial.print(":"); }
    Serial.print(currentMode == MANUAL);
}