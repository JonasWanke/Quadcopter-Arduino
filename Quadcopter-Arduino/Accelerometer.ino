const byte ACC_SERIAL_ADDRESS = 0x53;
const byte ACC_DEVID = 0x00;
const byte ACC_BW_RATE = 0x2C;
const byte ACC_POWER_CTL = 0x2D;
const byte ACC_DATA_FORMAT = 0x31;
const byte ACC_FIFO_CTL = 38;
float acc[3];
float accGPerLSB;

float accRoll;
float accPitch;

float accMinX;
float accMinY;
float accMinZ;
float accMaxX;
float accMaxY;
float accMaxZ;

const int ACC_CALIBRATION_READINGS = 500;

const int ACC_EEPROM_OFFSET = 0;

#define ACC_LPF
#ifdef ACC_LPF
const byte ACC_BUFFER_LENGTH = 5;
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

  // POWER_CTL
  writeRegister(ACC_SERIAL_ADDRESS, ACC_POWER_CTL, 0b00101001);
#ifdef DEBUG_INIT
  Serial.println("ACC_POWER_CTL");
#endif

  // FIFO_CTL
  writeRegister(ACC_SERIAL_ADDRESS, ACC_FIFO_CTL, 0b10000000);
#ifdef DEBUG_INIT
  Serial.println("ACC_FIFO_CTL");
#endif

  // BW_RATE
  writeRegister(ACC_SERIAL_ADDRESS, ACC_BW_RATE, 0b00001011);
#ifdef DEBUG_INIT
  Serial.println("ACC_BW_RATE");
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
  EEPROM.get(ACC_EEPROM_OFFSET, accMinX);
  EEPROM.get(ACC_EEPROM_OFFSET + 1 * sizeof(float), accMaxX);
  EEPROM.get(ACC_EEPROM_OFFSET + 2 * sizeof(float), accMinY);
  EEPROM.get(ACC_EEPROM_OFFSET + 3 * sizeof(float), accMaxY);
  EEPROM.get(ACC_EEPROM_OFFSET + 4 * sizeof(float), accMinZ);
  EEPROM.get(ACC_EEPROM_OFFSET + 5 * sizeof(float), accMaxZ);
  #ifdef DEBUG_INIT
  Serial.println("Finished initializing gyroscope");
#endif
}

void calibrateAcc()
{
#ifdef DEBUG_CALIBRATE
  Serial.println("Calibrating accelerometer");
#endif
  byte* data;
  long sumX;
  long sumY;
  long sumZ;

  delay(1000);
  // Hold horizontal
#ifdef DEBUG_CALIBRATE
  Serial.println("Hold horizontal");
#endif
  while (!digitalRead(PIN_BUTTON_CALIBRATION));
  digitalWrite(PIN_LED_CALIBRATION_FINISHED, LOW);
  sumY = 0;
  sumZ = 0;
  for (int i = 0; i < ACC_CALIBRATION_READINGS; i++)
  {
    data = readRegister(ACC_SERIAL_ADDRESS, 0x36, 2);
    sumZ += (data[1] << 8) | data[0];
    Serial.println(i);
    delay(10);
  }
  accMaxZ = sumZ / ACC_CALIBRATION_READINGS * accGPerLSB;
  digitalWrite(PIN_LED_CALIBRATION_FINISHED, HIGH);

  // Hold upside down
#ifdef DEBUG_CALIBRATE
  Serial.println("Hold upside down");
#endif
  while (!digitalRead(PIN_BUTTON_CALIBRATION));
  digitalWrite(PIN_LED_CALIBRATION_FINISHED, LOW);
  sumZ = 0;
  for (int i = 0; i < ACC_CALIBRATION_READINGS; i++)
  {
    data = readRegister(ACC_SERIAL_ADDRESS, 0x36, 2);
    sumZ += (data[1] << 8) | data[0];
    Serial.println(i);
    delay(10);
  }
  accMinZ = sumZ / ACC_CALIBRATION_READINGS * accGPerLSB;
  digitalWrite(PIN_LED_CALIBRATION_FINISHED, HIGH);

  // Hold right side down
#ifdef DEBUG_CALIBRATE
  Serial.println("Hold right side down");
#endif
  while (!digitalRead(PIN_BUTTON_CALIBRATION));
  digitalWrite(PIN_LED_CALIBRATION_FINISHED, LOW);
  Serial.println("Right down");
  sumY = 0;
  for (int i = 0; i < ACC_CALIBRATION_READINGS; i++)
  {
    data = readRegister(ACC_SERIAL_ADDRESS, 0x34, 2);
    sumY += (data[1] << 8) | data[0];
    Serial.println(i);
    delay(10);
  }
  accMaxY = sumY / ACC_CALIBRATION_READINGS * accGPerLSB;
  digitalWrite(PIN_LED_CALIBRATION_FINISHED, HIGH);

  // Hold left side down
#ifdef DEBUG_CALIBRATE
  Serial.println("Hold left side down");
#endif
  while (!digitalRead(PIN_BUTTON_CALIBRATION));
  digitalWrite(PIN_LED_CALIBRATION_FINISHED, LOW);
  sumY = 0;
  for (int i = 0; i < ACC_CALIBRATION_READINGS; i++)
  {
    data = readRegister(ACC_SERIAL_ADDRESS, 0x34, 2);
    sumY += (data[1] << 8) | data[0];
    Serial.println(i);
    delay(10);
  }
  accMinY = sumY / ACC_CALIBRATION_READINGS * accGPerLSB;
  digitalWrite(PIN_LED_CALIBRATION_FINISHED, HIGH);

  // Hold front side down
#ifdef DEBUG_CALIBRATE
  Serial.println("Hold front side down");
#endif
  while (!digitalRead(PIN_BUTTON_CALIBRATION));
  digitalWrite(PIN_LED_CALIBRATION_FINISHED, LOW);
  sumX = 0;
  for (int i = 0; i < ACC_CALIBRATION_READINGS; i++)
  {
    data = readRegister(ACC_SERIAL_ADDRESS, 0x32, 2);
    sumX += (data[1] << 8) | data[0];
    Serial.println(i);
    delay(10);
  }
  accMinX = sumX / ACC_CALIBRATION_READINGS * accGPerLSB;
  digitalWrite(PIN_LED_CALIBRATION_FINISHED, HIGH);

  // Hold back side down
#ifdef DEBUG_CALIBRATE
  Serial.println("Hold back side down");
#endif
  while (!digitalRead(PIN_BUTTON_CALIBRATION));
  digitalWrite(PIN_LED_CALIBRATION_FINISHED, LOW);
  sumX = 0;
  for (int i = 0; i < ACC_CALIBRATION_READINGS; i++)
  {
    data = readRegister(ACC_SERIAL_ADDRESS, 0x32, 2);
    sumX += (data[1] << 8) | data[0];
    Serial.println(i);
    delay(10);
  }
  accMaxX = sumX / ACC_CALIBRATION_READINGS * accGPerLSB;
  digitalWrite(PIN_LED_CALIBRATION_FINISHED, HIGH);

  EEPROM.put(ACC_EEPROM_OFFSET, accMinX);
  EEPROM.put(ACC_EEPROM_OFFSET + 1 * sizeof(float), accMaxX);
  EEPROM.put(ACC_EEPROM_OFFSET + 2 * sizeof(float), accMinY);
  EEPROM.put(ACC_EEPROM_OFFSET + 3 * sizeof(float), accMaxY);
  EEPROM.put(ACC_EEPROM_OFFSET + 4 * sizeof(float), accMinZ);
  EEPROM.put(ACC_EEPROM_OFFSET + 5 * sizeof(float), accMaxZ);
  delay(100);
  digitalWrite(PIN_LED_CALIBRATION_FINISHED, LOW);
#ifdef DEBUG_CALIBRATE
  Serial.print("Calibration finished, mins, avgs and maxs (g): ");
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

void readAcc()
{
  byte* data = readRegister(ACC_SERIAL_ADDRESS, 0x32, 6);

  int x = (data[1] << 8) | data[0];
  int y = (data[3] << 8) | data[2];
  int z = (data[5] << 8) | data[4];

#ifdef ACC_LPF
  accBufferSum[0] -= accBuffer[accBufferPos][0];
  accBufferSum[1] -= accBuffer[accBufferPos][1];
  accBufferSum[2] -= accBuffer[accBufferPos][2];
  accBuffer[accBufferPos][0] = x;
  accBuffer[accBufferPos][1] = y;
  accBuffer[accBufferPos][2] = z;
  accBufferSum[0] += x;
  accBufferSum[1] += y;
  accBufferSum[2] += z;
  accBufferPos++;
  if (accBufferPos == ACC_BUFFER_LENGTH) accBufferPos = 0;

  acc[0] = accBufferSum[0] / ACC_BUFFER_LENGTH;
  acc[1] = accBufferSum[1] / ACC_BUFFER_LENGTH;
  acc[2] = accBufferSum[2] / ACC_BUFFER_LENGTH;
#endif

  acc[0] = map(acc[0] * accGPerLSB, accMinX, accMaxX, -1, 1);
  acc[1] = map(acc[1] * accGPerLSB, accMinY, accMaxY, -1, 1);
  acc[2] = map(acc[2] * accGPerLSB, accMinZ, accMaxZ, -1, 1);

  accRoll = atan2(acc[2], acc[1]);
  accPitch = atan2(acc[2], acc[0]);

#ifdef DEBUG_UPDATE
  Serial.print("Acc: ");
  Serial.print(accRoll * RAD_TO_DEG, 2);
  Serial.print(";\t");
  Serial.print(accPitch * RAD_TO_DEG, 2);
  Serial.print(";\t");
#endif
}
