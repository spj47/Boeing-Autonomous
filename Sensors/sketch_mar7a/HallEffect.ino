const int HALL_PIN = A0;

// Callibration
const int CALREADINGCOUNT = 50;
int calibrationCount = CALREADINGCOUNT;
int calibrationSum = 0;
float backgroundMag;
bool shownCalibrationDebug = false;
bool isCalibrated = false;

unsigned long timeLastSawMag = 0.0;
unsigned long timeSinceLastSawMag = 0.0;
const int NEARMAGVAL = 20;

void setup() {
  // Init Pins
  pinMode(HALL_PIN, INPUT);

  // Init Serial
  Serial.begin(9600);
}

void loop() {
  // Get Hall Value
  float halVal = analogRead(HALL_PIN);

  if (!isCalibrated)
  {
    isCalibrated = CalibrateSensor(halVal);
    return;
  }

  int calibratedReading = halVal - backgroundMag; // Centers the reading
  bool magNearby = abs(calibratedReading) > NEARMAGVAL; // checks if the magnet is nearby

  if (!magNearby) // Adds time to the counter if a magnet was not seen
  {
    timeSinceLastSawMag = (millis() - timeLastSawMag) / 1000.0;
  }
  else // Derives the velocity and resets timer
  {
    timeLastSawMag = millis();
    timeSinceLastSawMag = 0.0;
  }
  Serial.print("calibratedReading = ");
  Serial.print(calibratedReading);
  Serial.print(" | magNearby = ");
  Serial.print(magNearby);
  Serial.print(" | time = ");
  Serial.println(timeSinceLastSawMag);
}

bool CalibrateSensor(int halVal)
{
  if (!shownCalibrationDebug)
  {
    shownCalibrationDebug = true;
    Serial.println("Calibrating HallEffect Sensor...");
  }

  if (calibrationCount > 0)
  {
    calibrationSum += halVal;
    calibrationCount--;
    delay(20);
    return false;
  }
  else if (calibrationCount == 0)
  {
    backgroundMag = calibrationSum / CALREADINGCOUNT;
    calibrationCount--;
  }

  return true;
}
