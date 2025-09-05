#define DIR_PIN 2
#define STEP_PIN 3

// Change these to control movement
int direction = 1;       // 1 = one direction, 0 = other direction
int steps_to_move = 2000; // Number of steps to move

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  // Set direction
  digitalWrite(DIR_PIN, direction);

  // Move motor
  for (int i = 0; i < steps_to_move; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(500); // Pulse HIGH duration
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(500); // Time between steps
  }
}

void loop() {
  // Do nothing after movement
}

