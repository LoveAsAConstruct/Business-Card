#include <Wire.h>

#define BMA400_ADDRESS 0x14    // I2C address of the BMA400
#define BMA400_ACC_X_LSB 0x04  // Register for X-axis LSB
#define BMA400_ACC_Y_LSB 0x06  // Register for Y-axis LSB
#define BMA400_ACC_Z_LSB 0x08  // Register for Z-axis LSB

#define MATRIX_SIZE 8          // 8x8 matrix
#define SHAKE_THRESHOLD 2000   // Adjust this threshold for sensitivity

// Define positive and ground pins for LED matrix
int rowPins[8] = {1, 2, 3, 4, 5, 6, 8, 10};
int colPins[8] = {13, 14, 15, 16, 17, 18, 19, 20};

// Matrix to represent "sand" positions
bool sandMatrix[MATRIX_SIZE][MATRIX_SIZE] = {false};
unsigned long lastUpdate = 0;
const int updateInterval = 100;  // Update sand position every 100 ms

void setup() {
  Wire.begin(11, 12);        // Initialize I2C with SDA on pin 11 and SCL on pin 12
  Serial.begin(9600);

  initializeMatrix();
  initializeSand();
  initializeBMA400();
}

void loop() {
  unsigned long currentMillis = millis();

  // Read accelerometer values to check for shake or tilt
  int16_t accelX = readRegister16(BMA400_ACC_X_LSB);
  int16_t accelY = readRegister16(BMA400_ACC_Y_LSB);
  int16_t accelZ = readRegister16(BMA400_ACC_Z_LSB);

  if (detectShake(accelX, accelY, accelZ)) {
    initializeSand();  // Reset the sand positions
    Serial.println("Shake detected! Sand reset.");
  } else if (currentMillis - lastUpdate >= updateInterval) {
    // Update sand positions based on tilt every `updateInterval` milliseconds
    int tiltAngle = calculateZAngle(accelX, accelY, accelZ);
    updateSand(tiltAngle);
    lastUpdate = currentMillis;  // Reset the timer for the next update
  }

  // Continuously display the sand particles
  displaySand();
}

void initializeBMA400() {
  Wire.beginTransmission(BMA400_ADDRESS);
  Wire.write(0x19);       // Set power mode register
  Wire.write(0x01);       // Set to normal mode
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

int calculateZAngle(int16_t accelX, int16_t accelY, int16_t accelZ) {
  return accelZ / 256;   // Adjust scale if necessary
}

void initializeMatrix() {
  for (int i = 0; i < 8; i++) {
    pinMode(rowPins[i], OUTPUT);
    pinMode(colPins[i], OUTPUT);
  }
}

void initializeSand() {
  // Clear the matrix
  for (int i = 0; i < MATRIX_SIZE; i++) {
    for (int j = 0; j < MATRIX_SIZE; j++) {
      sandMatrix[i][j] = false;
    }
  }
  
  // Fill a quarter of the matrix with "sand" particles
  for (int i = 0; i < MATRIX_SIZE / 2; i++) {
    for (int j = 0; j < MATRIX_SIZE / 2; j++) {
      sandMatrix[i][j] = true;   // Set initial sand particles
    }
  }
}

bool detectShake(int16_t accelX, int16_t accelY, int16_t accelZ) {
  int magnitude = accelX * accelX + accelY * accelY + accelZ * accelZ;
  return magnitude > SHAKE_THRESHOLD * SHAKE_THRESHOLD;
}

void updateSand(int tiltAngle) {
  // Move sand particles based on tilt direction
  for (int row = MATRIX_SIZE - 1; row >= 0; row--) {
    for (int col = 0; col < MATRIX_SIZE; col++) {
      if (sandMatrix[row][col]) {
        // Check if sand can fall down or sideways
        if (tiltAngle > 10 && col < MATRIX_SIZE - 1 && !sandMatrix[row][col + 1]) {
          sandMatrix[row][col] = false;
          sandMatrix[row][col + 1] = true;
        } else if (tiltAngle < -10 && col > 0 && !sandMatrix[row][col - 1]) {
          sandMatrix[row][col] = false;
          sandMatrix[row][col - 1] = true;
        } else if (row < MATRIX_SIZE - 1 && !sandMatrix[row + 1][col]) {
          sandMatrix[row][col] = false;
          sandMatrix[row + 1][col] = true;
        }
      }
    }
  }
}

void displaySand() {
  // Turn off all LEDs
  clearMatrix();

  // Display sand particles by lighting up each "on" LED in the sandMatrix
  for (int row = 0; row < MATRIX_SIZE; row++) {
    for (int col = 0; col < MATRIX_SIZE; col++) {
      if (sandMatrix[row][col]) {
        digitalWrite(rowPins[row], HIGH);    // Turn on row pin
        digitalWrite(colPins[col], LOW);     // Turn on column pin
        delay(1);                            // Small delay to stabilize
        digitalWrite(rowPins[row], LOW);     // Turn off row pin
        digitalWrite(colPins[col], HIGH);    // Turn off column pin
      }
    }
  }
}

void clearMatrix() {
  // Reset all row and column pins to turn off LEDs
  for (int i = 0; i < 8; i++) {
    digitalWrite(rowPins[i], LOW);
    digitalWrite(colPins[i], HIGH);
  }
}
