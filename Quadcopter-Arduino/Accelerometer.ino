const byte ACC_SERIAL_ADDRESS = 0x53;
const byte ACC_DEVID = 0x00;
const byte ACC_BW_RATE = 0x2C;
const byte ACC_POWER_CTL = 0x2D;
const byte ACC_DATA_FORMAT = 0x31;
const byte ACC_FIFO_CTL = 0x38;
const byte ACC_DATA = 0x32;

// Unit: g
float accX;
// Unit: g
float accY;
// Unit: g
float accZ;
// Unit: rad
float accRoll;
// Unit: rad
float accPitch;

float accGPerLSB;
byte accData[6];

// Unit: g
float accMinX;
// Unit: g
float accMaxX;
// Unit: g
float accMinY;
// Unit: g
float accMaxY;
// Unit: g
float accMinZ;
// Unit: g
float accMaxZ;

const int ACC_CALIBRATION_READINGS = 500;

const int ACC_EEPROM_ID = 0;
const int ACC_EEPROM_OFFSET = 512 + ACC_EEPROM_ID * 64;

#define ACC_LPF
#ifdef ACC_LPF
const byte ACC_BUFFER_LENGTH = 8;
int accBuffer[ACC_BUFFER_LENGTH][3];
int accBufferSum[3];
byte accBufferPos = 0;
#endif

//Initializes ADXL345 accelerometer
void initAcc(int scale)
{
#ifdef DEBUG_INIT
  Serial.println("Initializing Accelerometer");
#endif

  // BW_RATE
  writeRegister(ACC_SERIAL_ADDRESS, ACC_BW_RATE, 0b00001011);
#ifdef DEBUG_INIT
  Serial.println("ACC_BW_RATE");
#endif

  // POWER_CTL
  writeRegister(ACC_SERIAL_ADDRESS, ACC_POWER_CTL, 0b00101001);
#ifdef DEBUG_INIT
  Serial.println("ACC_POWER_CTL");
#endif

  // DATA_FORMAT (6+7: Range)
  if (scale == 2)
  {
    writeRegister(ACC_SERIAL_ADDRESS, ACC_DATA_FORMAT, 0b00000000);
    accGPerLSB = 0.0039;
  }
  else if (scale == 4)
  {
    writeRegister(ACC_SERIAL_ADDRESS, ACC_DATA_FORMAT, 0b00000001);
    accGPerLSB = 0.0078;
  }
  else if (scale == 8)
  {
    writeRegister(ACC_SERIAL_ADDRESS, ACC_DATA_FORMAT, 0b00000010);
    accGPerLSB = 0.0156;
  }
  else
  {
    writeRegister(ACC_SERIAL_ADDRESS, ACC_DATA_FORMAT, 0b00000011);
    accGPerLSB = 0.0312;
  }
#ifdef DEBUG_INIT
  Serial.println("ACC_DATA_FORMAT");
#endif

  // FIFO_CTL
  writeRegister(ACC_SERIAL_ADDRESS, ACC_FIFO_CTL, 0b10000000);
#ifdef DEBUG_INIT
  Serial.println("ACC_FIFO_CTL");
#endif

  EEPROM.get(ACC_EEPROM_OFFSET, accMinX);
  EEPROM.get(ACC_EEPROM_OFFSET + 1 * sizeof(float), accMaxX);
  EEPROM.get(ACC_EEPROM_OFFSET + 2 * sizeof(float), accMinY);
  EEPROM.get(ACC_EEPROM_OFFSET + 3 * sizeof(float), accMaxY);
  EEPROM.get(ACC_EEPROM_OFFSET + 4 * sizeof(float), accMinZ);
  EEPROM.get(ACC_EEPROM_OFFSET + 5 * sizeof(float), accMaxZ);

#ifdef DEBUG_INIT
  Serial.println("Finished initializing accelerometer");
#endif
}

void calibrateAcc()
{
#ifdef DEBUG_CALIBRATE
  Serial.println("Calibrating accelerometer");
#endif

  calibrateValue(ACC_DATA + 4, &accMaxZ, "horizontal");
  calibrateValue(ACC_DATA + 4, &accMinZ, "upside down");
  calibrateValue(ACC_DATA + 2, &accMaxY, "right side down");
  calibrateValue(ACC_DATA + 2, &accMinY, "left side down");
  calibrateValue(ACC_DATA, &accMinX, "front side down");
  calibrateValue(ACC_DATA, &accMaxX, "back side down");

  EEPROM.put(ACC_EEPROM_OFFSET, accMinX);
  EEPROM.put(ACC_EEPROM_OFFSET + 1 * sizeof(float), accMaxX);
  EEPROM.put(ACC_EEPROM_OFFSET + 2 * sizeof(float), accMinY);
  EEPROM.put(ACC_EEPROM_OFFSET + 3 * sizeof(float), accMaxY);
  EEPROM.put(ACC_EEPROM_OFFSET + 4 * sizeof(float), accMinZ);
  EEPROM.put(ACC_EEPROM_OFFSET + 5 * sizeof(float), accMaxZ);
  delay(100);
  digitalWrite(PIN_LED_CALIBRATION_FINISHED, LOW);

#ifdef DEBUG_CALIBRATE
  Serial.print("Calibration finished, mins and maxs (g):");
  Serial.print(accMinX, 2);
  Serial.print(";\t");
  Serial.print(accMaxX, 2);
  Serial.print(";\t");
  Serial.print(accMinY, 2);
  Serial.print(";\t");
  Serial.print(accMaxY, 2);
  Serial.print(";\t");
  Serial.print(accMinZ, 2);
  Serial.print(";\t");
  Serial.print(accMaxZ, 2);
  Serial.println();
#endif
}

void calibrateValue(int offset, float* value, String message)
{
#ifdef DEBUG_CALIBRATE
  Serial.println("Hold " + message);
#endif
  while (!digitalRead(PIN_BUTTON_CALIBRATION));
  digitalWrite(PIN_LED_CALIBRATION_FINISHED, LOW);
  long sum = 0;
  for (int i = 0; i < ACC_CALIBRATION_READINGS; i++)
  {
    readRegistersFast(ACC_SERIAL_ADDRESS, offset, 2, accData);
    sum += (accData[1] << 8) | accData[0];
    delay(10);
  }
  *value = sum / ACC_CALIBRATION_READINGS * accGPerLSB;
  digitalWrite(PIN_LED_CALIBRATION_FINISHED, HIGH);
}

void updateAcc()
{
  readRegistersFast(ACC_SERIAL_ADDRESS, ACC_DATA, 6, accData);
  accX = (accData[1] << 8) | accData[0];
  accY = (accData[3] << 8) | accData[2];
  accZ = (accData[5] << 8) | accData[4];

#ifdef ACC_LPF
  accBufferSum[0] -= accBuffer[accBufferPos][0];
  accBufferSum[1] -= accBuffer[accBufferPos][1];
  accBufferSum[2] -= accBuffer[accBufferPos][2];
  accBuffer[accBufferPos][0] = accX;
  accBuffer[accBufferPos][1] = accY;
  accBuffer[accBufferPos][2] = accZ;
  accBufferSum[0] += accX;
  accBufferSum[1] += accY;
  accBufferSum[2] += accZ;
  accBufferPos++;
  accBufferPos %= ACC_BUFFER_LENGTH;
  accX = accBufferSum[0] / ACC_BUFFER_LENGTH * accGPerLSB;
  accY = accBufferSum[1] / ACC_BUFFER_LENGTH * accGPerLSB;
  accZ = accBufferSum[2] / ACC_BUFFER_LENGTH * accGPerLSB;
#endif

  accX = mapf(accX, accMinX, accMaxX, -1, 1);
  accY = -mapf(accY, accMinY, accMaxY, -1, 1);
  accZ = -mapf(accZ, accMinZ, accMaxZ, -1, 1);

  accRoll = atan2(sqrt(accX * accX + accZ * accZ), accY) - PI_OVER_2;
  accPitch = -(atan2(sqrt(accY * accY + accZ * accZ), accX) - PI_OVER_2);

#ifdef DEBUG_UPDATE
  Serial.print("Acc: ");
  Serial.print(accX);
  Serial.print(";\t");
  Serial.print(accY);
  Serial.print(";\t");
  Serial.print(accZ);
  Serial.print(";\t");
  Serial.print(accRoll * RAD_TO_DEG);
  Serial.print(";\t");
  Serial.print(accPitch * RAD_TO_DEG);
  Serial.print(";\t");
#endif
}

