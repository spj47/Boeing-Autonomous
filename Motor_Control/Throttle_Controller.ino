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
 *   [DEPRECIATED] POT        - toggle potentiometer reading stream
 *   [DEPRECIATED] AUTO       - switch to autonomous mode (Pi control)
 *   [DEPRECIATED] MANUAL     - switch to manual mode (pot control)
 */

#include <AccelStepper.h>

// pin assignments
const int PIN_STEP   = 3;    // DM556 PUL+
const int PIN_DIR    = 4;    // DM556 DIR+
const int PIN_ENABLE = 5;    // DM556 ENA+
const int PIN_LIMIT  = 2;    // limit switch (idle position)
const int PIN_POT    = A0;   // potentiometer for manual control

// throttle range in steps
// 6400 steps = 180 degree rotation (half revolution)
// negative = counter-clockwise, positive = clockwise
const long THROTTLE_MIN = 0;
const long THROTTLE_MAX = -6400;

// potentiometer settings
const int POT_MIN = 150;
const int POT_MAX = 900;
const int POT_DEADZONE = 5;
const int POT_READ_INTERVAL = 50;
const int POT_SAMPLES = 5;
const int POT_SAMPLE_DELAY_US = 500;
const int POT_STREAM_INTERVAL = 50;

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

// set false to completely disable pot/manual mode
const bool ENABLE_MANUAL_MODE = true;

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
int lastPotThrottle = 0;
unsigned long lastPotRead = 0;
bool potStream = false;

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

    // limit switch with internal pull-up (connects to GND when triggered)
    pinMode(PIN_LIMIT, INPUT_PULLUP);

    // initialize pot reading to prevent startup drift
    if (ENABLE_MANUAL_MODE)
    {
        int smoothedPot = readPotSmoothed();
        int clampedPot = constrain(smoothedPot, POT_MIN, POT_MAX);
        lastPotThrottle = map(clampedPot, POT_MIN, POT_MAX, 0, 100);
    }

    // current position is home (0%)
    isCalibrated = true;

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

    // stream pot readings when enabled
    if (potStream)
    {
        int potReading = analogRead(PIN_POT);
        if (IS_DEBUG)
        {
            Serial.println(potReading);
        }
        delay(POT_STREAM_INTERVAL);
    }

    // manual mode pot control
    if (ENABLE_MANUAL_MODE && currentMode == MANUAL)
    {
        processManualControl();
    }

    // run stepper (non-blocking, must be called frequently)
    if (isEnabled)
    {
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

// read potentiometer with median filter to reject noise spikes
int readPotSmoothed()
{
    int samples[POT_SAMPLES];

    // collect samples with small delay between reads
    for (int i = 0; i < POT_SAMPLES; i++)
    {
        samples[i] = analogRead(PIN_POT);
        delayMicroseconds(POT_SAMPLE_DELAY_US);
    }

    // bubble sort for median extraction (fine for small arrays)
    for (int i = 0; i < POT_SAMPLES - 1; i++)
    {
        for (int j = 0; j < POT_SAMPLES - i - 1; j++)
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
    return samples[POT_SAMPLES / 2];
}

// read potentiometer and update throttle in manual mode
void processManualControl()
{
    // rate limit pot reads
    unsigned long currentTime = millis();
    if (currentTime - lastPotRead < POT_READ_INTERVAL)
    {
        return;
    }
    lastPotRead = currentTime;

    int rawPot = readPotSmoothed();

    // constrain to valid range and map to percentage
    rawPot = constrain(rawPot, POT_MIN, POT_MAX);
    int potThrottle = map(rawPot, POT_MIN, POT_MAX, 0, 100);

    // apply deadzone to prevent jitter
    int difference = abs(potThrottle - lastPotThrottle);
    if (difference < POT_DEADZONE)
    {
        return;
    }
    lastPotThrottle = potThrottle;

    int curvedThrottle = applyThrottleCurve(potThrottle);

    // auto-calibrate on first call if needed
    if (!isCalibrated)
    {
        stepper.setCurrentPosition(0);
        isCalibrated = true;
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
    else if (cmdUpper == "POT")
    {
        potStream = !potStream;
        if (!potStream && IS_DEBUG)
        {
            Serial.println("POT:OFF");
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
        else if (IS_DEBUG)
        {
            Serial.println("ERR:MANUAL_DISABLED");
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
        }
        else if (!isCalibrated)
        {
            if (IS_DEBUG)
            {
                Serial.println("ERR:NOT_CAL");
            }
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

    long targetSteps = map(percent, 0, 100, THROTTLE_MIN, THROTTLE_MAX);

    enableMotor();
    stepper.moveTo(targetSteps);
}

// calibrate: set current position as idle (0%)
// manually position throttle to idle before calling
void calibrate()
{
    disableMotor();
    stepper.setCurrentPosition(0);
    currentThrottle = 0;
    isCalibrated = true;
}

// emergency stop: halt motor and disable driver
void emergencyStop()
{
    stepper.stop();

    // clear movement target by locking current position
    long currentPos = stepper.currentPosition();
    stepper.setCurrentPosition(currentPos);

    disableMotor();
    currentThrottle = map(currentPos, THROTTLE_MIN, THROTTLE_MAX, 0, 100);
}

// check if limit switch is triggered (throttle at idle)
bool isAtIdle()
{
    return digitalRead(PIN_LIMIT) == LOW;
}

// get actual throttle percentage based on step position
int getActualThrottle()
{
    long currentPos = stepper.currentPosition();
    return map(currentPos, THROTTLE_MIN, THROTTLE_MAX, 0, 100);
}

// send current system status over serial
void sendStatus()
{
    int actualThrottle = getActualThrottle();
    bool atIdle = isAtIdle();

    Serial.print("S:");
    Serial.print(currentThrottle);
    Serial.print(",");
    Serial.print(actualThrottle);
    Serial.print(",");

    if (isEnabled)
    {
        Serial.print("1");
    }
    else
    {
        Serial.print("0");
    }

    Serial.print(",");

    if (isCalibrated)
    {
        Serial.print("1");
    }
    else
    {
        Serial.print("0");
    }

    Serial.print(",");

    if (atIdle)
    {
        Serial.print("1");
    }
    else
    {
        Serial.print("0");
    }

    Serial.print(",");

    if (currentMode == MANUAL)
    {
        Serial.println("M");
    }
    else
    {
        Serial.println("A");
    }
}