#include <Wire.h>  // Include the Wire library for I2C communication

#define BMA400_ADDRESS 0x14  // I2C address of the BMA400 (check datasheet for your specific setup)
#define BMA400_ACC_X_LSB 0x04 // Register for X-axis LSB
#define BMA400_ACC_Y_LSB 0x06 // Register for Y-axis LSB
#define BMA400_ACC_Z_LSB 0x08 // Register for Z-axis LSB

void setup() {
  Wire.begin(11, 12);  // Initialize I2C with SDA on pin 11 and SCL on pin 12
  Serial.begin(9600);  // Start serial communication for debugging

  Serial.println("Initializing BMA400...");
  initializeBMA400();
}

void loop() {
  // Read and print acceleration values
  int16_t accelX = readRegister16(BMA400_ACC_X_LSB);
  int16_t accelY = readRegister16(BMA400_ACC_Y_LSB);
  int16_t accelZ = readRegister16(BMA400_ACC_Z_LSB);

  Serial.print("X: ");
  Serial.print(accelX);
  Serial.print(" Y: ");
  Serial.print(accelY);
  Serial.print(" Z: ");
  Serial.println(accelZ);

  delay(500);  // Poll every 500 ms
}

void initializeBMA400() {
  Wire.beginTransmission(BMA400_ADDRESS);
  Wire.write(0x19);      // Send register address to set power mode
  Wire.write(0x01);      // Set to normal mode (change as needed)
  Wire.endTransmission();
  delay(10);  // Allow time for the sensor to initialize
}

int16_t readRegister16(uint8_t reg) {
  Wire.beginTransmission(BMA400_ADDRESS);
  Wire.write(reg);        // Register to read
  Wire.endTransmission();
  Wire.requestFrom(BMA400_ADDRESS, 2); // Request 2 bytes from the register

  if (Wire.available() >= 2) {
    uint8_t lsb = Wire.read();
    uint8_t msb = Wire.read();
    return (int16_t)((msb << 8) | lsb);  // Combine MSB and LSB
  } else {
    return 0; // Return 0 if there's a communication issue
  }
}
