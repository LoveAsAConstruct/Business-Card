#define DELAY_TIME 100  // Adjust delay for testing speed

// Define positive pins for the rows (connected to pins 1-6, 8, and 10)
int rowPins[8] = {1, 2, 3, 4, 5, 6, 8, 10};

// Define ground pins for the columns (connected to pins 13 to 20)
int colPins[8] = {13, 14, 15, 16, 17, 18, 19, 20};

void setup() {
  // Initialize row and column pins
  for (int i = 0; i < 8; i++) {
    pinMode(rowPins[i], OUTPUT);
    pinMode(colPins[i], OUTPUT);
  }

  Serial.begin(9600);
  Serial.println("Starting LED matrix test...");
}

void loop() {
  // Test each LED individually
  Serial.println("Testing each LED...");
  for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 8; col++) {
      lightUpLED(row, col);
      delay(DELAY_TIME);
      turnOffLED(row, col);
    }
  }

  delay(500); // Pause between tests

  // Test each row
  Serial.println("Testing each row...");
  for (int row = 0; row < 8; row++) {
    lightUpRow(row);
    delay(DELAY_TIME * 8);
    turnOffRow(row);
  }

  delay(500); // Pause between tests

  // Test each column
  Serial.println("Testing each column...");
  for (int col = 0; col < 8; col++) {
    lightUpColumn(col);
    delay(DELAY_TIME * 8);
    turnOffColumn(col);
  }

  delay(1000); // Delay before restarting the loop
}

// Function to light up a specific LED at (row, col)
void lightUpLED(int row, int col) {
  digitalWrite(rowPins[row], HIGH);    // Set row to HIGH (positive)
  digitalWrite(colPins[col], LOW);     // Set column to LOW (ground)
}

// Function to turn off a specific LED at (row, col)
void turnOffLED(int row, int col) {
  digitalWrite(rowPins[row], LOW);     // Set row to LOW
  digitalWrite(colPins[col], HIGH);    // Set column to HIGH
}

// Function to light up an entire row
void lightUpRow(int row) {
  digitalWrite(rowPins[row], HIGH);
  for (int i = 0; i < 8; i++) {
    digitalWrite(colPins[i], LOW);
  }
}

// Function to turn off an entire row
void turnOffRow(int row) {
  digitalWrite(rowPins[row], LOW);
  for (int i = 0; i < 8; i++) {
    digitalWrite(colPins[i], HIGH);
  }
}

// Function to light up an entire column
void lightUpColumn(int col) {
  digitalWrite(colPins[col], LOW);
  for (int i = 0; i < 8; i++) {
    digitalWrite(rowPins[i], HIGH);
  }
}

// Function to turn off an entire column
void turnOffColumn(int col) {
  digitalWrite(colPins[col], HIGH);
  for (int i = 0; i < 8; i++) {
    digitalWrite(rowPins[i], LOW);
  }
}
