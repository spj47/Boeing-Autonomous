#include <Wire.h>
#include <SoftwareSerial.h>

// Pins
const int HALL_PIN = 2; // THIS HAS TO BE EITHER D2 OR D3 | -------------------------->NO EXCPETION <-------------------------------------------------------------------------
// A4 SDA OF IMU
// A5 SCL OF IMU

// Hall Effect Constants
const float WHEEL_CIRCUMFRANCE = 1.28805298797; // 0.41m diameter * 3.1459
const int NUMBER_MAGNETS = 2;

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

// IMU Meta data
int16_t ax1, ay1, az1, gx1, gy1, gz1;

// Serial COM
SoftwareSerial PISerial(10, 11);
const byte ARD_IMU_ADR       = 0x06;
const byte ARD_HALL_ADR      = 0x07;
const byte IMU_ADR           = 0x69; // 69 for high one
const char* GET_DATA_COMMAND = "D";

const bool ISDEBUG           = false;

void setup() 
{
  // Init Pins
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), MagnetDetected, RISING);

  initMPU(IMU_ADR);

  // Init Serial
  Serial.begin(115200); // DO NOT FORGET TO CHANGE TO THIS BAUD RATE IN IDE 
  PISerial.begin(9600);
}

void initMPU(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x6B); 
  Wire.write(0);  
  Wire.endTransmission(true);
}

void loop() 
{
  if (ISDEBUG)
  {
    Serial.print("Velocity: ");
    Serial.println(velocity);
    Serial.print("A: ");
    Serial.print(ax1); Serial.print(", ");
    Serial.print(ay1); Serial.print(", ");
    Serial.print(az1); Serial.print(" | G: ");
    Serial.print(gx1); Serial.print(", ");
    Serial.print(gy1); Serial.print(", ");
    Serial.println(gz1);
  }
  
  UpdateVelocity();
  CheckSerial();
}

void CheckSerial()
{
  if (!PISerial.available()) return;

  String cmd = "";
  while (PISerial.available()) {
    char c = PISerial.read();
    if (c == '\n') break;
    cmd += c;
  }
  cmd.trim();

  if (cmd.startsWith(String(ARD_HALL_ADR)))
  {
    if (cmd.endsWith(GET_DATA_COMMAND))
    {
      PISerial.println(velocity);
    }
  } 
  else if (cmd.startsWith(String(ARD_IMU_ADR)))
  {
    if (cmd.endsWith(GET_DATA_COMMAND))
    {
      readIMU(IMU_ADR, ax1, ay1, az1, gx1, gy1, gz1);
      PISerial.print(ax1); PISerial.print(",");
      PISerial.print(ay1); PISerial.print(",");
      PISerial.print(az1); PISerial.print(",");
      PISerial.print(gx1); PISerial.print(",");
      PISerial.print(gy1); PISerial.print(",");
      PISerial.println(gz1);
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

void readIMU(uint8_t address,
             int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {

  Wire.beginTransmission(address);
  Wire.write(0x3B); // Starting register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 14, true);

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();

  Wire.read(); Wire.read(); // Skip temperature

  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
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
