#define STEP_PIN1 8   // Step pin for Motor 1
#define DIR_PIN1 9    // Direction pin for Motor 1
#define STEP_PIN2 10  // Step pin for Motor 2
#define DIR_PIN2 11   // Direction pin for Motor 2
#define STEP_PIN3 12  // Step pin for Motor 3
#define DIR_PIN3 13   // Direction pin for Motor 3
#define STEP_PIN4 4   // Step pin for Motor 4
#define DIR_PIN4 5    // Direction pin for Motor 4
#define STEP_PIN5 2   // Step pin for Motor 5
#define DIR_PIN5 3    // Direction pin for Motor 5
#define RELAY1_PIN 6  // Relay 1 pin
#define RELAY2_PIN 7  // Relay 2 pin

struct Motor {
  int stepPin;
  int dirPin;
  int speed;      // steps per second
  int position;   // degrees
};

Motor motors[5] = {
  {STEP_PIN1, DIR_PIN1, 100, 0},
  {STEP_PIN2, DIR_PIN2, 100, 0},
  {STEP_PIN3, DIR_PIN3, 100, 0},
  {STEP_PIN4, DIR_PIN4, 100, 0},
  {STEP_PIN5, DIR_PIN5, 100, 0}
};
bool relays[2] = {false, false};

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 5; i++) {
    pinMode(motors[i].stepPin, OUTPUT);
    pinMode(motors[i].dirPin, OUTPUT);
    digitalWrite(motors[i].stepPin, LOW);
    digitalWrite(motors[i].dirPin, HIGH);
  }
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
}

void rotateMotor(int motorIdx, int degrees) {
  // Assume 1 degree = 1 step for simplicity; adjust for your stepper
  int steps = abs(degrees); // Change this if your motor requires more steps per degree
  int dir = degrees > 0 ? HIGH : LOW;
  digitalWrite(motors[motorIdx].dirPin, dir);
  for (int i = 0; i < steps; i++) {
    digitalWrite(motors[motorIdx].stepPin, HIGH);
    delayMicroseconds(1000000 / motors[motorIdx].speed);
    digitalWrite(motors[motorIdx].stepPin, LOW);
    delayMicroseconds(1000000 / motors[motorIdx].speed);
  }
  motors[motorIdx].position += degrees;
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();

    // Relay control
    if (data.startsWith("R1:ON")) {
      digitalWrite(RELAY1_PIN, HIGH);
      relays[0] = true;
    } else if (data.startsWith("R1:OFF")) {
      digitalWrite(RELAY1_PIN, LOW);
      relays[0] = false;
    } else if (data.startsWith("R2:ON")) {
      digitalWrite(RELAY2_PIN, HIGH);
      relays[1] = true;
    } else if (data.startsWith("R2:OFF")) {
      digitalWrite(RELAY2_PIN, LOW);
      relays[1] = false;
    }

    // Motor control
    else if (data.startsWith("M1:+10")) {
      rotateMotor(0, 10);
    } else if (data.startsWith("M1:-10")) {
      rotateMotor(0, -10);
    } else if (data.startsWith("M2:+10")) {
      rotateMotor(1, 10);
    } else if (data.startsWith("M2:-10")) {
      rotateMotor(1, -10);
    } else if (data.startsWith("M3:+10")) {
      rotateMotor(2, 10);
    } else if (data.startsWith("M3:-10")) {
      rotateMotor(2, -10);
    } else if (data.startsWith("M4:+10")) {
      rotateMotor(3, 10);
    } else if (data.startsWith("M4:-10")) {
      rotateMotor(3, -10);
    } else if (data.startsWith("M5:+10")) {
      rotateMotor(4, 10);
    } else if (data.startsWith("M5:-10")) {
      rotateMotor(4, -10);
    }

    // Speed control
    else if (data.startsWith("SPEED:+10")) {
      for (int i = 0; i < 5; i++) {
        motors[i].speed = min(1000, motors[i].speed + motors[i].speed / 10); // Increase by 10%
      }
    } else if (data.startsWith("SPEED:-10")) {
      for (int i = 0; i < 5; i++) {
        motors[i].speed = max(10, motors[i].speed - motors[i].speed / 10); // Decrease by 10%
      }
    }
    // You can add more parsing as needed
  }
}
