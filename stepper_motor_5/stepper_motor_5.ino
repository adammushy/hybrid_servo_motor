#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

SoftwareSerial espSerial(2, 4); //Rx - tx 0f esp  IO2/pd2 of arduino -red,TX - rx esp - brown IO4 
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows A4=SDA-IO18=purple, A5=SCL-IO19=blue


const int stepPin = 5;
const int dirPin = 3;
const int enaPin = 6;
const int potPin = A1;
const int enableButtonPin = 8; // Button to enable/disable motor
const int dirButtonPin = 9;    // Button to change direction

// Define constants
const int stepsPerRevolution = 2000;
String str;
// Variables for button states
bool motorEnabled = true; // Start with the motor enabled
bool clockwise = false;   // Start with the motor rotating counterclockwise
// if 1==true== clockwise,,,,, if 0 == false== counterclockwise



void setup() {
    // Set up serial communication
    Serial.begin(9600);
    // Serial.begin(115200);
    // espSerial.begin(115200);
    espSerial.begin(9600);

    //set lcd
    lcd.init();
    lcd.backlight();

    lcd.setCursor(0, 0);
    lcd.print("   WELCOME... ");
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
    // lcd.clear();
    // Read potentiometer value for speed control
    int potValue = analogRead(potPin);
    Serial.println("Potentiometer value: " + String(potValue)); // Debug statement

    // Calculate step delay based on potentiometer value
    int stepDelay = map(potValue, 0, 1023, 10000, 4000);
    Serial.println("Step delay: " + String(stepDelay)); // Debug statement

    // Calculate motor speed in RPM
    float stepsPerSecond = 1000000.0 / (stepDelay * 2); // Convert microseconds to seconds
    float rpm = (stepsPerSecond * 60) / stepsPerRevolution;
    Serial.println("Motor speed (RPM): " + String(rpm)); // Display motor speed in RPM

    // Read enable button state
    if (digitalRead(enableButtonPin) == LOW) {
        delay(50); // Debounce delay
        motorEnabled = !motorEnabled;
        digitalWrite(enaPin, motorEnabled ? LOW : HIGH); // Enable is active LOW
        while (digitalRead(enableButtonPin) == LOW);  // Wait for button release
    }

    // Read direction button state
    if (digitalRead(dirButtonPin) == LOW) {
        delay(50); // Debounce delay
        clockwise = !clockwise;
        while (digitalRead(dirButtonPin) == LOW); // Wait for button release
    }
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Pwr:");
    lcd.print(motorEnabled ? "On " : "Off");
    lcd.print(" Dir:");
    lcd.print(clockwise ? "CW" : "CCW");
    lcd.setCursor(0, 1);
    lcd.print("Speed: ");
    lcd.print(stepDelay);
    lcd.print("");
    // Rotate the motor if enabled
    if (motorEnabled) {
        // Rotate for a specified duration
        
        rotate(stepsPerRevolution, stepDelay, clockwise);
        // delay(500); // Wait for 0.5 seconds
    }


    Serial.println("direction: "+ String(clockwise));
    Serial.println("powerstatus: "+ String(motorEnabled));
    espSerial.print(clockwise);
    espSerial.print(",");
    espSerial.print(motorEnabled);
    espSerial.print(",");
    espSerial.println(stepDelay);

    delay(2000);
}

void rotate(int steps, unsigned long stepDelay, bool clockwise) {
    digitalWrite(dirPin, clockwise ? HIGH : LOW); // Set rotation direction
    for (int i = 0; i < steps; i++) {
    int potValue = analogRead(potPin);

        int stepDelay = map(potValue, 0, 1023, 10000, 4000);
        // float stepsPerSecond = 1000000.0 / (stepDelay * 2); // Convert microseconds to seconds
        // float rpm = (stepsPerSecond * 60) / stepsPerRevolution;
        // lcd.setCursor(0, 1);
        // lcd.print("Speed: ");
        // lcd.print(stepDelay);
        // lcd.print(" RPM");
    
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepDelay);
    }
}
