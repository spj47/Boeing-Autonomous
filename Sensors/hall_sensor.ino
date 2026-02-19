// Hall effect sensor to find velocity

const bool IS_DEBUG = true;

const float WHEEL_DIAMETER = 0.41; // In meters
const float WHEEL_CIRCUMFERENCE = 3.14159 * WHEEL_DIAMETER; // In meters

const int HALL_PIN = A0;
const int NUMBER_OF_MAGNETS = 1;
const int HALL_CENTER = 503; // Background reading of the hall effect sensor
const int HALL_THRESHOLD = 520; // Reading when magnet is near
bool magnet_detected = false;

unsigned long time_1 = 0; // init time variable to find delta t

float rpm = 0;
float velocity = 0;

void setup() {
  Serial.begin(9600);
  pinMode(HALL_PIN, INPUT);
  Serial.println("Initializing Hall Effect Sensor...");
  time_1 = millis();
}

void loop() {
  int hall_reading = analogRead(HALL_PIN);

  // check if magnet detected
  if (hall_reading > HALL_THRESHOLD && !magnet_detected){
    // find delta t
    unsigned long time_2 = millis();
    unsigned long delta_t = time_2 - time_1;

    // check if delta t is 0 to ensure no 0 division
    if (delta_t > 0){
      // find RPM
      rpm = 60000.0/(delta_t * NUMBER_OF_MAGNETS);

      // find velocity from rpm
      velocity = (rpm * WHEEL_CIRCUMFERENCE)/60;
    }
    // update time & magnet detection
    time_2 = time_1;
    magnet_detected = true;

    printStats(hall_reading);
  }
  
  else if (hall_reading < HALL_CENTER){
    magnet_detected = false;
  }

  // set rpm and velocity to 0 if wheel stopped after 2s
  if (millis() - time_1 > 2000){
    rpm = 0;
    velocity = 0;
  }

}

// prints data
void printStats(int hall_reading) {
  Serial.print("Sensor Output: ");
  Serial.print(hall_reading);
  Serial.print(" | RPM: ");
  Serial.print(rpm);
  Serial.print(" | Velocity: ");
  Serial.print(velocity);
  Serial.println(" m/s");
}