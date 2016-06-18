const byte GYRO_SERIAL_ADDRESS = 105;
const byte GYRO_CTRL_REG1 = 0x20;
const byte GYRO_CTRL_REG2 = 0x21;
const byte GYRO_CTRL_REG3 = 0x22;
const byte GYRO_CTRL_REG4 = 0x23;
const byte GYRO_CTRL_REG5 = 0x24;
// Unit: °/s
float deltaPitch;
// Unit: °/s
float deltaRoll;
// Unit: °/s
float deltaYaw;
float gyroDpsPerDigit;

float gyroZeroRoll;
float gyroZeroPitch;
float gyroZeroYaw;

const int GYRO_CALIBRATION_READINGS = 500;

//High Pass Filter (on the IMU)
#define GYRO_HPF_IMU

//High Pass Filter (on the Arduino)
#define GYRO_HPF_ARD
#ifdef GYRO_HPF_ARD
const byte GYRO_BUFFER_LENGTH = 16;
int gyroBuffer[GYRO_BUFFER_LENGTH][3];
int gyroBufferSum[3];
byte gyroBufferPos = 0;
#endif

void initGyro(int scale)
{
  // Initializes L3G4200D gyroscope
#ifdef DEBUG_INIT
  Serial.print("Initializing gyroscope");
#endif

  // Enable x, y, z and turn off power down
#ifdef GYRO_HPF_IMU
  // Output Data Rate: 01 (200Hz)
  // Bandwidth: 11, Cut-Off: 70
  writeRegister(GYRO_SERIAL_ADDRESS, GYRO_CTRL_REG1, 0b01111111);
#else
  writeRegister(GYRO_CTRL_REG1, GYRO_CTRL_REG1, 0b00001111);
#endif
#ifdef DEBUG_INIT
  Serial.println("GYRO_CTRL_REG5");
#endif

  // High pass filter
#ifdef GYRO_HPF_IMU
  writeRegister(GYRO_SERIAL_ADDRESS, GYRO_CTRL_REG2, 0b00000000);
#ifdef DEBUG_INIT
  Serial.println("GYRO_CTRL_REG2");
#endif
#endif

  // GYRO_CTRL_REG4: range
  if (scale == 250) {
    writeRegister(GYRO_SERIAL_ADDRESS, GYRO_CTRL_REG4, 0b00000000);
    gyroDpsPerDigit = 0.00875;
  }
  else if (scale == 500) {
    writeRegister(GYRO_SERIAL_ADDRESS, GYRO_CTRL_REG4, 0b00010000);
    gyroDpsPerDigit = 0.0175;
  }
  else {
    writeRegister(GYRO_SERIAL_ADDRESS, GYRO_CTRL_REG4, 0b00100000);
    gyroDpsPerDigit = 0.07;
  }
#ifdef DEBUG_INIT
  Serial.println("GYRO_CTRL_REG4");
#endif

  // CTRL_REG5: enable High Pass Filter
#ifdef GYRO_HPF_IMU
  writeRegister(GYRO_SERIAL_ADDRESS, GYRO_CTRL_REG5, 0b00010000);
#else
  writeRegister(GYRO_SERIAL_ADDRESS, GYRO_CTRL_REG5, 0b00000000);
#endif
#ifdef DEBUG_INIT
  Serial.println("GYRO_CTRL_REG5");
  Serial.println("Finished initializing gyroscope");
#endif
}

void calibrateGyro()
{
#ifdef DEBUG_CALIBRATE
  Serial.println("Calibrating gyro");
#endif
  byte* data;
  long sumRoll;
  long sumPitch;
  long sumYaw;
  for (int i = 0; i < GYRO_CALIBRATION_READINGS; i++)
  {
    data = readRegister(GYRO_SERIAL_ADDRESS, 0x28, 6);
    sumRoll += ((data[1] << 8) | data[0]);
    sumPitch += ((data[3] << 8) | data[2]);
    sumYaw += ((data[5] << 8) | data[4]);
  }
  gyroZeroRoll = sumRoll / 1000;
  gyroZeroPitch = sumRoll / 1000;
  gyroZeroYaw = sumRoll / 1000;
  digitalWrite(PIN_LED_CALIBRATION_FINISHED, HIGH);
  delay(100);
  digitalWrite(PIN_LED_CALIBRATION_FINISHED, LOW);
#ifdef DEBUG_CALIBRATE
  Serial.print("Calibration finished, averages (°/s): ");
  Serial.print(gyroZeroRoll * gyroDpsPerDigit * REFRESH_INTERVAL, 2);
  Serial.print(";\t");
  Serial.print(gyroZeroPitch * gyroDpsPerDigit * REFRESH_INTERVAL, 2);
  Serial.print(";\t");
  Serial.print(gyroZeroYaw * gyroDpsPerDigit * REFRESH_INTERVAL, 2);
  Serial.println();
#endif
}

void updateGyro()
{
  byte* data = readRegister(GYRO_SERIAL_ADDRESS, 0x28, 6);

  deltaRoll = ((data[1] << 8) | data[0]) - gyroZeroRoll;
  deltaPitch = ((data[3] << 8) | data[2]) - gyroZeroPitch;
  deltaYaw = ((data[5] << 8) | data[4]) - gyroZeroYaw;

#ifdef GYRO_HPF_ARD
  gyroBufferSum[0] -= gyroBuffer[gyroBufferPos][0];
  gyroBufferSum[1] -= gyroBuffer[gyroBufferPos][1];
  gyroBufferSum[2] -= gyroBuffer[gyroBufferPos][2];
  gyroBuffer[gyroBufferPos][0] = deltaRoll;
  gyroBuffer[gyroBufferPos][1] = deltaPitch;
  gyroBuffer[gyroBufferPos][2] = deltaYaw;
  gyroBufferSum[0] += deltaRoll;
  gyroBufferSum[1] += deltaPitch;
  gyroBufferSum[2] += deltaYaw;
  gyroBufferPos++;
  gyroBufferPos %= GYRO_BUFFER_LENGTH;
  deltaRoll = gyroBufferSum[0] / GYRO_BUFFER_LENGTH;
  deltaPitch = gyroBufferSum[1] / GYRO_BUFFER_LENGTH;
  deltaYaw = gyroBufferSum[2] / GYRO_BUFFER_LENGTH;
#endif

  deltaRoll *= gyroDpsPerDigit * REFRESH_INTERVAL;
  deltaPitch *= gyroDpsPerDigit * REFRESH_INTERVAL;
  deltaYaw *= gyroDpsPerDigit * REFRESH_INTERVAL;

#ifdef DEBUG_UPDATE
  Serial.print(deltaRoll, 2);
  Serial.print(";\t");
  Serial.print(deltaPitch, 2);
  Serial.print(";\t");
  Serial.print(deltaYaw, 2);
  Serial.print(";\t");
#endif
}
