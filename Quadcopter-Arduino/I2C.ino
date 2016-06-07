void writeRegister(byte device, byte address, byte val)
{
	Wire.beginTransmission(device);
	Wire.write(address);
	Wire.write(val);
	Wire.endTransmission();
}

int readRegister(byte device, byte address)
{
	Wire.beginTransmission(device);
	Wire.write(address);
	Wire.endTransmission();

	Wire.requestFrom(device, (byte) 1);

	return Wire.read();
}
byte* readRegister(byte device, byte address, byte count)
{
	Wire.beginTransmission(device);
	Wire.write(address);
	Wire.endTransmission();

	Wire.requestFrom(device, count);

	byte *results = new byte[count];
	for (byte i = 0; i < count; i++)
		results[i] = Wire.read();
	return results;
}
