#include <Wire.h>  // Include the Wire library for I2C communication

void setup() {
  Serial.begin(9600);   // Initialize serial communication at 9600 baud
  Wire.begin();         // Start the I2C bus as a master
  Serial.println("I2C Scanner initialized. Scanning for devices...");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  // Scan all possible I2C addresses (1 to 126)
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);   // Begin transmission to a specific address
    error = Wire.endTransmission();    // End transmission and check for response

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  // Display result after scanning
  if (nDevices == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.print("Found ");
    Serial.print(nDevices);
    Serial.println(" I2C devices.");
  }

  delay(5000);  // Delay 5 seconds before the next scan
}
