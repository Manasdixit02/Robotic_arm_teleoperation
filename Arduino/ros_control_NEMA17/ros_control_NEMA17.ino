#define DIR_PIN 2
#define STEP_PIN 3

String inputString = "";
bool stringComplete = false;

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  Serial.begin(115200); // Match your sender's baud rate
  inputString.reserve(50); // Avoid memory fragmentation
}

void loop() {
  if (stringComplete) {
    int direction = 0;
    int steps = 0;

    // Parse input like "D:1,S:200"
    if (parseCommand(inputString, direction, steps)) {
      digitalWrite(DIR_PIN, direction);
      for (int i = 0; i < steps; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(200); // pulse width
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(200); // delay between steps
      }
    }

    inputString = "";
    stringComplete = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

bool parseCommand(String cmd, int &direction, int &steps) {
  int dIndex = cmd.indexOf("D:");
  int sIndex = cmd.indexOf("S:");

  if (dIndex == -1 || sIndex == -1) return false;

  direction = cmd.substring(dIndex + 2, cmd.indexOf(',', dIndex)).toInt();
  steps = cmd.substring(sIndex + 2).toInt();

  return true;
}
