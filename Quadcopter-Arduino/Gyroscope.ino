const byte GYRO_SERIAL_ADDRESS = 105;
const byte GYRO_CTRL_REG1 = 0x20;
const byte GYRO_CTRL_REG2 = 0x21;
const byte GYRO_CTRL_REG3 = 0x22;
const byte GYRO_CTRL_REG4 = 0x23;
const byte GYRO_CTRL_REG5 = 0x24;

// Unit: rad/s
float deltaPitch;
// Unit: rad/s
float deltaRoll;
// Unit: rad/s
float deltaYaw;

float gyroRadPerSecondPerDigit;
byte gyroData[6];

float gyroZeroRoll;
float gyroZeroPitch;
float gyroZeroYaw;

const int GYRO_CALIBRATION_READINGS = 500;

const int GYRO_EEPROM_ID = 1;
const int GYRO_EEPROM_OFFSET = 512 + GYRO_EEPROM_ID * 64;

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

// Initializes L3G4200D gyroscope
void initGyro(int scale)
{
#ifdef DEBUG_INIT
  Serial.print("Initializing gyroscope");
#endif

  // CTRL_REG_1: Enable x, y, z, turn off power down
  // Output Data Rate: 01 (200Hz)
  // Bandwidth: 11, Cut-Off: 70
  writeRegister(GYRO_SERIAL_ADDRESS, GYRO_CTRL_REG1, 0b01111111);
#ifdef DEBUG_INIT
  Serial.println("GYRO_CTRL_REG1");
#endif

  // GYRO_CTRL_REG2: High pass filter
#ifdef GYRO_HPF_IMU
  writeRegister(GYRO_SERIAL_ADDRESS, GYRO_CTRL_REG2, 0b00000000);
#ifdef DEBUG_INIT
  Serial.println("GYRO_CTRL_REG2");
#endif
#endif

  // GYRO_CTRL_REG4: Range
  if (scale == 250) {
    writeRegister(GYRO_SERIAL_ADDRESS, GYRO_CTRL_REG4, 0b00000000);
    gyroRadPerSecondPerDigit = 0.00875 * DEG_TO_RAD;
  }
  else if (scale == 500) {
    writeRegister(GYRO_SERIAL_ADDRESS, GYRO_CTRL_REG4, 0b00010000);
    gyroRadPerSecondPerDigit = 0.0175 * DEG_TO_RAD;
  }
  else {
    writeRegister(GYRO_SERIAL_ADDRESS, GYRO_CTRL_REG4, 0b00100000);
    gyroRadPerSecondPerDigit = 0.07 * DEG_TO_RAD;
  }
#ifdef DEBUG_INIT
  Serial.println("GYRO_CTRL_REG4");
#endif

  // CTRL_REG5: Enable High Pass Filter
#ifdef GYRO_HPF_IMU
  writeRegister(GYRO_SERIAL_ADDRESS, GYRO_CTRL_REG5, 0b00010000);
#else
  writeRegister(GYRO_SERIAL_ADDRESS, GYRO_CTRL_REG5, 0b00000000);
#endif
#ifdef DEBUG_INIT
  Serial.println("GYRO_CTRL_REG5");
#endif
  EEPROM.get(GYRO_EEPROM_OFFSET, gyroZeroRoll);
  EEPROM.get(GYRO_EEPROM_OFFSET + 1 * sizeof(float), gyroZeroPitch);
  EEPROM.get(GYRO_EEPROM_OFFSET + 2 * sizeof(float), gyroZeroYaw);

#ifdef DEBUG_INIT
  Serial.println("Finished initializing gyroscope");
#endif
}

void calibrateGyro()
{
#ifdef DEBUG_CALIBRATE
  Serial.println("Calibrating gyro");
#endif

  long sumRoll = 0;
  long sumPitch = 0;
  long sumYaw = 0;
  for (int i = 0; i < GYRO_CALIBRATION_READINGS; i++)
  {
    readRegisters(GYRO_SERIAL_ADDRESS, 0x28, 6, gyroData);
    sumRoll += ((gyroData[1] << 8) | gyroData[0]);
    sumPitch += ((gyroData[3] << 8) | gyroData[2]);
    sumYaw += ((gyroData[5] << 8) | gyroData[4]);
    delay(10);
  }
  gyroZeroRoll = sumRoll / GYRO_CALIBRATION_READINGS * gyroRadPerSecondPerDigit;
  gyroZeroPitch = sumPitch / GYRO_CALIBRATION_READINGS * gyroRadPerSecondPerDigit;
  gyroZeroYaw = sumYaw / GYRO_CALIBRATION_READINGS * gyroRadPerSecondPerDigit;
  EEPROM.put(GYRO_EEPROM_OFFSET, gyroZeroRoll);
  EEPROM.put(GYRO_EEPROM_OFFSET + 1 * sizeof(float), gyroZeroPitch);
  EEPROM.put(GYRO_EEPROM_OFFSET + 2 * sizeof(float), gyroZeroYaw);

  digitalWrite(PIN_LED_CALIBRATION_FINISHED, HIGH);
  delay(100);
  digitalWrite(PIN_LED_CALIBRATION_FINISHED, LOW);

#ifdef DEBUG_CALIBRATE
  Serial.print("Calibration finished, averages (°/s): ");
  Serial.print(gyroZeroRoll * RAD_TO_DEG);
  Serial.print(";\t");
  Serial.print(gyroZeroPitch * RAD_TO_DEG);
  Serial.print(";\t");
  Serial.print(gyroZeroYaw * RAD_TO_DEG);
  Serial.println();
#endif
}

void updateGyro()
{
  readRegisters(GYRO_SERIAL_ADDRESS, 0x28, 6, gyroData);

  deltaRoll = (gyroData[1] << 8) | gyroData[0];
  deltaPitch = (gyroData[3] << 8) | gyroData[2];
  deltaYaw = (gyroData[5] << 8) | gyroData[4];

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

  deltaRoll = (deltaRoll * gyroRadPerSecondPerDigit - gyroZeroRoll) * REFRESH_INTERVAL;
  deltaPitch = (deltaPitch * gyroRadPerSecondPerDigit - gyroZeroPitch) * REFRESH_INTERVAL;
  deltaYaw = (deltaYaw * gyroRadPerSecondPerDigit - gyroZeroYaw) * REFRESH_INTERVAL;

#ifdef DEBUG_UPDATE
  Serial.print("Gyro: ");
  Serial.print(deltaRoll * RAD_TO_DEG);
  Serial.print(";\t");
  Serial.print(deltaPitch * RAD_TO_DEG);
  Serial.print(";\t");
  Serial.print(deltaYaw * RAD_TO_DEG);
  Serial.print(";\t");
#endif
}

