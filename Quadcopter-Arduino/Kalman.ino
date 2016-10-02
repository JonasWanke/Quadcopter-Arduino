// Unit: rad
float pitch;
// Unit: rad
float roll;
// Unit: rad
float yaw;

Kalman kalmanRoll;
Kalman kalmanPitch;

void initKalman()
{
  kalmanRoll.setAngle(0);
  kalmanPitch.setAngle(0);
}

void updateKalman()
{
  // Roll
  roll = kalmanRoll.getAngle(accRoll, deltaRoll, REFRESH_INTERVAL);

  // Pitch
  pitch = kalmanPitch.getAngle(accPitch, deltaPitch, REFRESH_INTERVAL);

  // Yaw
  yaw += deltaYaw * REFRESH_INTERVAL;

#ifdef DEBUG_UPDATE
  Serial.print("Kalman: ");
  Serial.print(roll * RAD_TO_DEG);
  Serial.print(";\t");
  Serial.print(pitch * RAD_TO_DEG);
  Serial.print(";\t");
  Serial.print(yaw * RAD_TO_DEG);
  Serial.print(";\t");
#endif
}

