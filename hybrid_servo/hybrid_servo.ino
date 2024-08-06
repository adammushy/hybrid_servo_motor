



// NOTE:to controll moter direction using pin set DIP switch(SW2) to OFF
// to control dir using SW2 disable/comment codes related with dirPin

// Define pin numbers
const int pulsePin = 5;
const int dirPin = 3;
const int enablePin = 6;
const int potPin = A1;
const int dirButtonPin = 9;
const int enableButtonPin = 8;

// Variables for button states
bool direction = LOW;
bool motorEnabled = true;

void setup() {
    // Initialize serial communication
    Serial.begin(9600);
    // Set pin modes
    pinMode(pulsePin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    pinMode(dirButtonPin, INPUT_PULLUP);
    pinMode(enableButtonPin, INPUT_PULLUP);

    // Initial state
    digitalWrite(dirPin, direction);
    // digitalWrite(enablePin, motorEnabled ? HIGH : LOW);  // Enable is active LOW
    digitalWrite(enablePin,LOW);  // Enable is active LOW

}

void loop() {
    // Read potentiometer value
    int potValue = analogRead(potPin);
    int delayTime = map(potValue, 0, 1023, 40000, 10000);  // Map to appropriate delay for speed control

    // Read direction button state
    // if (digitalRead(dirButtonPin) == LOW) {
    //     delay(50);  // Debounce delay
    //     direction = !direction;
    //     digitalWrite(dirPin, direction);
    //     while (digitalRead(dirButtonPin) == LOW);  // Wait for button release
    // }

    // Read enable button state
    if (digitalRead(enableButtonPin) == LOW) {
        delay(50);  // Debounce delay
        motorEnabled = !motorEnabled;
        digitalWrite(enablePin, motorEnabled ? LOW : HIGH);  // Enable is active LOW
        while (digitalRead(enableButtonPin) == LOW);  // Wait for button release
    }

// // If motor is enabled, generate pulses

    if (motorEnabled) {
        digitalWrite(pulsePin, HIGH); //sets the pulse pin high.
        delayMicroseconds(delayTime);  //waits for the specified time
        digitalWrite(pulsePin, LOW); //sets the pulse pin low.
        delayMicroseconds(delayTime); // waits again.
        Serial.println("motor is enable the state is  :: " + String(digitalRead(enablePin)));

        // Adjust pulseDelay to control the speed. Lower values increase speed, and higher values decrease speed.
        //the value is controlled with the POT

    }

    Serial.println("delay time speed :: " + String(delayTime));
    Serial.println("motor state :: " + String(motorEnabled));
    // Serial.println("direction of motor :: " + String(direction));
    delay(1000);
}





// #include <AccelStepper.h>

// // Define pin numbers
// const int pulsePin = 2; // STEP
// const int dirPin = 3; // DIR - Note: Not used for direction control in this setup
// const int enablePin = 4; // ENABLE
// const int potPin = A0; // Potentiometer for speed control
// const int enableButtonPin = A2; // Button to enable/disable motor

// // Create an instance of the AccelStepper class
// AccelStepper stepper(AccelStepper::DRIVER, pulsePin, dirPin);

// // Variables for button states
// bool motorEnabled = true;

// void setup() {
//     // Initialize serial communication
//     Serial.begin(9600);

//     // Set pin modes
//     pinMode(enablePin, OUTPUT);
//     pinMode(enableButtonPin, INPUT_PULLUP);

//     // Initial state
//     digitalWrite(enablePin, motorEnabled ? LOW : HIGH);  // Enable is active LOW

//     // Set maximum speed and acceleration
//     stepper.setMaxSpeed(1000);  // Adjust to your needs
//     stepper.setAcceleration(100);  // Adjust to your needs
// }

// void loop() {
//     // Read potentiometer value
//     int potValue = analogRead(potPin);
//     int motorSpeed = map(potValue, 0, 1023, 0, 1000);  // Map to appropriate speed range

//     // Read enable button state
//     if (digitalRead(enableButtonPin) == LOW) {
//         delay(50);  // Debounce delay
//         motorEnabled = !motorEnabled;
//         digitalWrite(enablePin, motorEnabled ? LOW : HIGH);  // Enable is active LOW
//         while (digitalRead(enableButtonPin) == LOW);  // Wait for button release
//     }

//     // If motor is enabled, set speed and move
//     if (motorEnabled) {
//         stepper.setSpeed(motorSpeed);
//         stepper.runSpeed();
//     }

//     Serial.println("Motor speed :: " + String(motorSpeed));
//     Serial.println("Motor enabled :: " + String(motorEnabled));
//     delay(500);
// }

