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

float roll;

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
  Serial.print("Initializing gyroscope");

  // Enable x, y, z and turn off power down
#ifdef GYRO_HPF_IMU
  // Output Data Rate: 01 (200Hz)
  // Bandwidth: 11, Cut-Off: 70
  writeRegister(GYRO_SERIAL_ADDRESS, GYRO_CTRL_REG1, 0b01111111);
#else
  writeRegister(GYRO_SERIAL_ADDRESS, GYRO_CTRL_REG1, 0b00001111);
#endif
  Serial.print(".");

  // High pass filter
#ifdef GYRO_HPF_IMU
  writeRegister(GYRO_SERIAL_ADDRESS, GYRO_CTRL_REG2, 0b00000000);
  Serial.print(".");
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
  Serial.print(".");

  // CTRL_REG5: enable High Pass Filter
#ifdef GYRO_HPF_IMU
  writeRegister(GYRO_SERIAL_ADDRESS, GYRO_CTRL_REG5, 0b00010000);
#else
  writeRegister(GYRO_SERIAL_ADDRESS, GYRO_CTRL_REG5, 0b00000000);
#endif
  Serial.println(".");
}

void calibrateGyro()
{
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
}

void readGyro()
{
  byte* data = readRegister(GYRO_SERIAL_ADDRESS, 0x28, 6);

  deltaRoll = ((data[1] << 8) | data[0]) - gyroZeroRoll;
  deltaPitch = ((data[3] << 8) | data[2]) - gyroZeroPitch;
  deltaYaw = ((data[5] << 8) | data[4]) - gyroZeroYaw;
  //  deltaRoll = ((data[1] << 8) | data[0]);
  //  deltaPitch = ((data[3] << 8) | data[2]);
  //  deltaYaw = ((data[5] << 8) | data[4]);

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

  /*
    Serial.print(deltaRoll);
    Serial.print(";");
    Serial.print(gyroBufferSum[0] / GYRO_BUFFER_LENGTH / 1000);
    Serial.print(";");
    Serial.println(roll);//*/

  //  Serial.print(((data[1] << 8) | data[0]) * gyroDpsPerDigit);
  //  Serial.print(";");
  //  Serial.print(((data[3] << 8) | data[2]) * gyroDpsPerDigit);
  //  Serial.print(";");
  //  Serial.println(((data[5] << 8) | data[4]) * gyroDpsPerDigit);
  Serial.print(deltaRoll);
  Serial.print(";");
  Serial.print(deltaPitch);
  Serial.print(";");
  Serial.println(deltaYaw);

  /*
    // High-pass filter
    if (!gyroHPFEnable) return;

    gyroZero[0] = GYRO_ALPHA * gyroZero[0] + (1 - GYRO_ALPHA) * gyro[0];
    gyroZero[1] = GYRO_ALPHA * gyroZero[1] + (1 - GYRO_ALPHA) * gyro[1];
    gyroZero[2] = GYRO_ALPHA * gyroZero[2] + (1 - GYRO_ALPHA) * gyro[2];

    gyro[0] -= gyroZero[0];
    gyro[1] -= gyroZero[1];
    gyro[2] -= gyroZero[2];//*/
}
