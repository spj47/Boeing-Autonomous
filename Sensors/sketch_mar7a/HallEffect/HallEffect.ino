const int HALL_PIN1 = A0;
const int HALL_PIN2 = A1;
const int HALL_PIN3 = A2;
const int HALL_PINS[] = {HALL_PIN1, HALL_PIN2, HALL_PIN3};
const int HALL_PIN_COUNT = 3;

// Callibration
float backgroundMag = 513;

// Velocity
const float WHEELCIRCUMFRANCE = 1.28805298797; // 0.41m diameter * 3.1459
const float ZERO_VEL_TIME = 1000; // ms
float velocity;

unsigned long timeLastSawMag = 0.0;
unsigned long timeSinceLastSawMag = 0.0;
const int NEARMAGVAL = 20;
bool lastFrameSawMagnet = false;

void setup() 
{
  // Init Pins
  pinMode(HALL_PIN1, INPUT);
  pinMode(HALL_PIN2, INPUT);
  pinMode(HALL_PIN3, INPUT);

  // Init Serial
  Serial.begin(9600);
}

void loop() 
{
  // Check if any of the sensors see the magnet
  bool magNearby = IsMagnetNearby();

  if (!magNearby || lastFrameSawMagnet) // Adds time to the counter if a magnet was not seen
  {
    if (!magNearby) // This makes sure that there really isn't a magnet for the flag (Fix redundancy later)
      lastFrameSawMagnet = false;

    timeSinceLastSawMag = (millis() - timeLastSawMag);

    if (timeSinceLastSawMag > ZERO_VEL_TIME) // if the hall effect is sitting in (or hasn't seen) the magnets in ZERO_VEL_TIME then the velocity is set to 0
      velocity = 0;
  }
  else // Derives the velocity and resets timer
  {
    velocity = WHEELCIRCUMFRANCE / (timeSinceLastSawMag / 1000);

    // Reset the timers
    timeLastSawMag = millis();
    timeSinceLastSawMag = 0.0;
  }
}

bool IsMagnetNearby()
{
  for (int i = 0; i < HALL_PIN_COUNT; i++)
  {
    if (HallSeesMagnet(i))
    {
        return true;
    }
  }

  return false;
}

bool HallSeesMagnet(int hallSensor)
{
  float halVal = analogRead(HALL_PINS[hallSensor]);
  int calibratedReading = halVal - backgroundMag; // Centers the reading
  return abs(calibratedReading) > NEARMAGVAL; // checks if the magnet is nearby
}