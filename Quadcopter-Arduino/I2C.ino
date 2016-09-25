void writeRegister(byte device, byte address, byte val)
{
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(val);
  Wire.endTransmission();
}

byte readRegister(byte device, byte address)
{
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(device, (byte) 1);

  return Wire.read();
}
void readRegister(byte device, byte address, byte count, byte* result)
{
  for (byte i = 0; i < count; i++)
    result[i] = readRegister(device, address + i);
}
