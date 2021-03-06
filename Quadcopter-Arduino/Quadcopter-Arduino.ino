#include <EEPROM.h>
#include <Wire.h>
#include "Kalman.h"

// Debug options
#define DEBUG_INIT
#define DEBUG_CALIBRATE
#define DEBUG_UPDATE

const double PI_OVER_2 = PI / 2;

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

const byte PIN_BUTTON_CALIBRATION = 2;
const byte PIN_LED_CALIBRATION = 3;
const byte PIN_LED_CALIBRATION_FINISHED = 4;

// Initialization and calibration
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialization of all sensors
#ifdef DEBUG_INIT
  Serial.println();
  Serial.println("Initialization starts");
#endif

  initGyro(250);
  initAcc(4);
  initKalman();

#ifdef DEBUG_INIT
  Serial.println("Initialization finished");
#endif

  delay(1500);

  // Calibration of all sensors
#ifdef DEBUG_CALIBRATE
  Serial.println();
  Serial.println("Calibration starts");
#endif
  pinMode(PIN_BUTTON_CALIBRATION, INPUT);
  pinMode(PIN_LED_CALIBRATION, OUTPUT);
  pinMode(PIN_LED_CALIBRATION_FINISHED, OUTPUT);
  digitalWrite(PIN_LED_CALIBRATION, HIGH);

  //calibrateAcc();
  //calibrateGyro();

  digitalWrite(PIN_LED_CALIBRATION, LOW);
  digitalWrite(PIN_LED_CALIBRATION_FINISHED, HIGH);
#ifdef DEBUG_CALIBRATE
  Serial.println("Calibration finished");
#endif

#ifdef DEBUG_UPDATE
  Serial.println();
  Serial.println("Update starts");
#endif
}

// Updating
void loop() {
  if (lastTime == 0)
    lastTime = micros();
  currentTime = micros();
  timeDelta = (currentTime - lastTime) / 1000;
  lastTime = currentTime;

  if (digitalRead(PIN_BUTTON_CALIBRATION))
    calibrateAcc();

  updateAcc();
  updateGyro();
  updateKalman();

#ifdef DEBUG_UPDATE
  Serial.println();
#endif

  delay(REFRESH_INTERVAL * 1000);
}

inline float mapf(float value, float fromLow, float fromHigh, float toLow, float toHigh)
{
  return (value - fromLow) / (fromHigh - fromLow) * (toHigh - toLow) + toLow;
}

