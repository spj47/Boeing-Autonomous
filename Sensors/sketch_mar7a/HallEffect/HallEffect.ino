const int HALL_PIN = 2; // THIS HAS TO BE EITHER D2 OR D3 | -------------------------->NO EXCPETION <-------------------------------------------------------------------------
const float WHEEL_CIRCUMFRANCE = 1.28805298797; // 0.41m diameter * 3.1459
const int NUMBER_MAGNETS = 6;

// Velocity
const int VELOCITY_SAMPLES = 5;

float velocitySamples[VELOCITY_SAMPLES];
int velocityIndex = 0;

float velocity = 0;
const unsigned long ZERO_VEL_TIME = 1000000; // us  

// Timers
volatile unsigned long lastMagTime = 0;
volatile unsigned long magPeriod = 0;

const unsigned long MAG_DEBOUNCE_TIME = 50000; // us

// Serial COM
const char* HALL_ADR = "Ax7";
const char* GET_DATA_COMMAND = "D";

void setup() 
{
  // Init Pins
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), MagnetDetected, RISING);

  // Init Serial
  Serial.begin(9600);
}

void loop() 
{
  UpdateVelocity();
  CheckSerial();
}

void CheckSerial()
{
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith(HALL_ADR))
  {
    if (cmd.endsWith(GET_DATA_COMMAND))
    {
      Serial.println(velocity);
    }
  }
}

void UpdateVelocity()
{
  // Get the time since last magnet detection
  unsigned long now = micros();
  unsigned long period;
  unsigned long lastMag;

  noInterrupts();
  period = magPeriod;
  lastMag = lastMagTime; // This gets changed in the ISR, so cached to avoid bad data
  interrupts();

  // Set the new velocity
  if (period > 0)
  {
    float rawVelocity = (WHEEL_CIRCUMFRANCE * 1000000.0) / (period * NUMBER_MAGNETS);
    velocity = GetAverageVelocity(rawVelocity);
  }

  // Set velocity to zero if enough time has passed
  if (now - lastMag > ZERO_VEL_TIME)
  {
    velocity = 0;
  }
}

float GetAverageVelocity(float newVel)
{
  velocitySamples[velocityIndex] = newVel;

  velocityIndex++;
  if (velocityIndex >= VELOCITY_SAMPLES)
    velocityIndex = 0;

  float sum = 0;

  for (int i = 0; i < VELOCITY_SAMPLES; i++)
  {
    sum += velocitySamples[i];
  }

  return sum / VELOCITY_SAMPLES;
}


// Gets called when the hall effect detects a magnet
void MagnetDetected()
{
  unsigned long now = micros();

  // Debounce to avoid seeing the magnet mutlitple times in one rotaiton
  if (now - lastMagTime < MAG_DEBOUNCE_TIME)
    return;

  magPeriod = now - lastMagTime;
  lastMagTime = now;
}