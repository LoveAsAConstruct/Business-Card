#include <Wire.h>  // Include the Wire library for I2C communication

#define BMA400_ADDRESS 0x14      // I2C address of the BMA400
#define BMA400_ACC_X_LSB 0x04    // Register for X-axis LSB
#define BMA400_ACC_Y_LSB 0x06    // Register for Y-axis LSB
#define BMA400_ACC_Z_LSB 0x08    // Register for Z-axis LSB

void setup() {
  Wire.begin(11, 12);            // Initialize I2C with SDA on pin 11 and SCL on pin 12
  Serial.begin(9600);            // Start serial communication for debugging

  initializeBMA400();
}

void loop() {
  int16_t accelZ = readRegister16(BMA400_ACC_Z_LSB);

  int16_t zAngle = calculateZAngle(accelZ);   // Calculate Z-axis angle (yaw)

  Serial.print("Z-Axis (Yaw) Angle: ");
  Serial.println(zAngle);

  delay(500);  // Poll every 500 ms
}

void initializeBMA400() {
  Wire.beginTransmission(BMA400_ADDRESS);
  Wire.write(0x19);      // Send register address to set power mode
  Wire.write(0x01);      // Set to normal mode
  Wire.endTransmission();
  delay(10);
}

int16_t readRegister16(uint8_t reg) {
  Wire.beginTransmission(BMA400_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BMA400_ADDRESS, 2);

  int16_t value = 0;
  if (Wire.available() >= 2) {
    value = (Wire.read() | (Wire.read() << 8));
  }
  return value;
}

int16_t calculateZAngle(int16_t accelZ) {
  // Simplified calculation, returns Z-axis tilt directly as an integer
  return (accelZ / 256);  // Scale down by a factor for approximate angle
}
