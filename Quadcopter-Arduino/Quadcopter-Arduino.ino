#include <Wire.h>

// Unit: µs
unsigned long lastTime = 0;
// Unit: µs
unsigned long currentTime;
// Unit: µs
double timeDelta;

// Unit: Hz
const float REFRESH_RATE = 10;
// Unit: s
const float REFRESH_INTERVAL = 1 / REFRESH_RATE;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  initGyro(250);
  delay(1500);
  Serial.println("Calibrating gyro");
  calibrateGyro();
  Serial.println("Calibration finished");
}

void loop() {
  if (lastTime == 0)
    lastTime = micros();
  currentTime = micros();
  timeDelta = (currentTime - lastTime) / 1000;
  lastTime = currentTime;

  readGyro();
  delay(REFRESH_INTERVAL * 1000);
}
