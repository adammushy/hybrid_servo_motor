const int stepPin = 5;
const int dirPin = 3;
const int enaPin = 6;
const int potPin = A1;
const int enableButtonPin = 8; // Button to enable/disable motor //PB0 atmega I08 proteus 
const int dirButtonPin = 9;    // Button to change direction //PB1 atmega   I09 proteus blue button

// Define constants
const int stepsPerRevolution = 1600;

// Variables for button states
bool motorEnabled = true; // Start with the motor enabled
bool clockwise = false;   // Start with the motor rotating counterclockwise

void setup() {
    // Set up serial communication
    Serial.begin(9600);
    // Set pin modes
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enaPin, OUTPUT);
    pinMode(enableButtonPin, INPUT_PULLUP); // Enable internal pull-up resistor
    pinMode(dirButtonPin, INPUT_PULLUP);    // Enable internal pull-up resistor
    // Initial motor enable state
    digitalWrite(enaPin, motorEnabled ? LOW : HIGH); // Enable is active LOW
}

void loop() {
    // Read potentiometer value for speed control
    int potValue = analogRead(potPin);
  Serial.println("Potentiometer value: " + String(potValue)); // Debug statement

  int stepDelay = map(potValue, 0, 1023, 10000, 4000);
  Serial.println("delay time speed :: " + String(stepDelay)); // Debug statement

    // int stepDelay = map(potValue, 0, 1023, 10000, 4000);

    // Read enable button state
    if (digitalRead(enableButtonPin) == LOW) {
        delay(50); // Debounce delay
        motorEnabled = !motorEnabled;
        digitalWrite(enaPin, motorEnabled ? LOW : HIGH); // Enable is active LOW
        while (digitalRead(enableButtonPin) == LOW); // Wait for button release
    }

    // Read direction button state
    if (digitalRead(dirButtonPin) == LOW) {
        delay(50); // Debounce delay
        clockwise = !clockwise;
        while (digitalRead(dirButtonPin) == LOW); // Wait for button release
    
    }

    // Rotate the motor if enabled
    if (motorEnabled) {
        // Rotate for a specified duration
        rotate(stepsPerRevolution, stepDelay, clockwise);
        delay(500); // Wait for 5 seconds
    }
    Serial.println("delay time speed :: " + String(stepDelay));

    Serial.println("motor direction :: " + String(clockwise));
    delay(500);

}

void rotate(int steps, unsigned long stepDelay, bool clockwise) {
    digitalWrite(dirPin, clockwise ? HIGH : LOW); // Set rotation direction
    for (int i = 0; i < steps; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepDelay);
    }
}
