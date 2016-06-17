#include <Wire.h>

// Debug options
#define DEBUG_INIT
#define DEBUG_CALIBRATE
#define DEBUG_UPDATE

// Unit: µs
unsigned long lastTime = 0;
// Unit: µs
unsigned long currentTime;
// Unit: µs
double timeDelta;

// Unit: Hz
const float REFRESH_RATE = 200;
// Unit: s
const float REFRESH_INTERVAL = 1 / REFRESH_RATE;


void setup() {
  Serial.begin(9600);
  Wire.begin();

#ifdef DEBUG_INIT
  Serial.println();
  Serial.println("Initialization starts");
#endif
  initGyro(250);
#ifdef DEBUG_INIT
  Serial.println("Initialization finished");
#endif

  delay(1500);

#ifdef DEBUG_CALIBRATE
  Serial.println();
  Serial.println("Calibration starts");
#endif
  calibrateGyro();
#ifdef DEBUG_CALIBRATE
  Serial.println("Calibration finished");
#endif

#ifdef DEBUG_UPDATE
  Serial.println();
  Serial.println("Update starts");
#endif
}

void loop() {
  if (lastTime == 0)
    lastTime = micros();
  currentTime = micros();
  timeDelta = (currentTime - lastTime) / 1000;
  lastTime = currentTime;

  updateGyro();

#ifdef DEBUG_UPDATE
  Serial.println();
#endif

  delay(REFRESH_INTERVAL * 1000);
}
