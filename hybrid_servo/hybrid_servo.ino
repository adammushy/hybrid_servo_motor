#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
#define Bt_F 8
#define Bt_S A1
#define Bt_B 9
#define dirPin 3
#define stepPin 5
#define stepsPerRevolution 1600
int Mode=0;
void setup() {
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(Bt_F, INPUT_PULLUP);
  pinMode(Bt_S, INPUT_PULLUP);
  pinMode(Bt_B, INPUT_PULLUP);
 
  lcd.init();
  lcd.backlight();
 
}
void loop() {
  if(digitalRead (Bt_F) == 0) {Mode = 1;}
  if(digitalRead (Bt_S) == 0) {Mode = 0;}
  if(digitalRead (Bt_B) == 0) {Mode = 2;}
  lcd.setCursor(0,1);
  if(Mode == 1) {lcd.print(" Forward ");}
  if(Mode == 0) {lcd.print(" Stop ");}
  if(Mode == 2) {lcd.print(" Revers ");}

 if(Mode == 1) {
  // Set the spinning direction clockwise:
  digitalWrite(dirPin, HIGH);
  // Spin the stepper motor 1 revolution slowly:
  for (int i = 0; i < stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(50);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(50);
  }
 }


 if(Mode == 2) {
  // Set the spinning direction clockwise:
  digitalWrite(dirPin, LOW);
  // Spin the stepper motor 1 revolution slowly:
  for (int i = 0; i < stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(50);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(50);
  }
 }
}










// #include <LiquidCrystal_I2C.h>
// #include <SoftwareSerial.h>

// // Pin definitions
// const int reverseSwitch = 8;
// const int driverPul = 5;
// const int driverDir = 3;
// const int spd = A1;
// const int enableButtonPin = 9; // Button to enable/disable motor

// // LCD and Serial Communication
// SoftwareSerial espSerial(2, 4); // Rx - tx of ESP, TX - rx of ESP
// LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 columns and 2 rows

// // Variables
// int pd = 2000; // Pulse delay
// boolean setdir = LOW; // Set direction
// bool motorEnabled = true; // Motor starts enabled

// // Debounce variables
// unsigned long lastDebounceTime = 0;
// unsigned long debounceDelay = 50; // Debounce delay in milliseconds

// // Reverse motor direction function
// // void revmotor() {
// //   // Check if debounce time has passed
// //   if ((millis() - lastDebounceTime) > debounceDelay) {
// //     setdir = !setdir; // Toggle direction
// //     lastDebounceTime = millis(); // Update last debounce time
// //   }
// // }

// void setup() {
//   // Initialize pins
//   pinMode(driverPul, OUTPUT);
//   pinMode(driverDir, OUTPUT);
//   pinMode(reverseSwitch, INPUT_PULLUP);
//   pinMode(enableButtonPin, INPUT_PULLUP); // Enable internal pull-up resistor

//   // Attach interrupt to reverse switch
//   // attachInterrupt(digitalPinToInterrupt(reverseSwitch), revmotor, FALLING);

//   // Initialize Serial communication
//   Serial.begin(9600);
//   espSerial.begin(9600);

//   // Initialize LCD
//   lcd.init();
//   lcd.backlight();
//   lcd.setCursor(0, 0);
//   lcd.print("   WELCOME... ");
//   delay(2000); // Display welcome message for 2 seconds
// }

// void loop() {
//   // Read potentiometer value for speed control
//   int potValue = analogRead(spd);
//   pd = map(potValue, 0, 1023, 15000, 2000);

//   // Calculate motor speed in RPM
//   float stepsPerSecond = 1000000.0 / (pd * 2); // Convert microseconds to seconds
//   float rpm = (stepsPerSecond * 60) / 1600; // Steps per revolution is 1600

//   // // Read enable button state
//   // if (digitalRead(enableButtonPin) == LOW) {
//   //   delay(1); // Debounce delay
//   //   motorEnabled = !motorEnabled;
//   //   while (digitalRead(enableButtonPin) == LOW); // Wait for button release
//   // }
//   digitalRead(enableButtonPin);
//   // motorEnabled = !motorEnabled

//   // Set motor direction
//   // digitalWrite(driverDir, setdir);

//   // Rotate the motor if enabled

//     digitalWrite(driverPul, HIGH);
//     delayMicroseconds(pd);
//     digitalWrite(driverPul, LOW);
//     delayMicroseconds(pd);
  

//   // Update LCD display
//   lcd.clear();
//   lcd.setCursor(0, 0);
//   lcd.print("Pwr:");
//   lcd.print(motorEnabled ? "On " : "Off");
//   lcd.print(" Dir:");
//   lcd.print(setdir ? "CW" : "CCW");

//   lcd.setCursor(0, 1);
//   lcd.print("Speed: ");
//   lcd.print(rpm);
//   lcd.print(" RPM");

//   // Send data to ESP
//   espSerial.print(setdir);
//   espSerial.print(",");
//   espSerial.print(motorEnabled);
//   espSerial.print(",");
//   espSerial.println(rpm);

//   // Debugging output
//   Serial.print("Direction: ");
//   Serial.println(setdir ? "Clockwise" : "Counterclockwise");
//   Serial.print("Power status: ");
//   Serial.println(motorEnabled ? "Enabled" : "Disabled");
//   Serial.print("Motor speed (RPM): ");
//   Serial.println(rpm);

//   // delay(100); // Small delay for readability in the serial monitor
// }




// // //define pin

// int reverseSwitch =8;
// int driverPul = 5;
// int driverDir =3;
// int spd=A1;

// // variable
// int pd =2000; //pulse delat
// boolean setdir = LOW; //set direction

// unsigned long lastDebounceTime = 0;
// unsigned long debounceDelay = 50; // Debounce delay in milliseconds

// // Reverse motor direction function
// // void revmotor() {
// //   // Check if debounce time has passed
// //   if ((millis() - lastDebounceTime) > debounceDelay) {
// //     setdir = !setdir; // Toggle direction
// //     lastDebounceTime = millis(); // Update last debounce time
// //   }
// // }
// void setup(){
//   pinMode(driverPul, OUTPUT);
//   pinMode(driverDir, OUTPUT);
//   pinMode(reverseSwitch, INPUT_PULLUP); 
//   // attachInterrupt(digitalPinToInterrupt(reverseSwitch), revmotor,FALLING);


// }
// void loop(){
//   pd = map((analogRead(spd)), 0, 1023, 15000, 2000);
//   digitalWrite(reverseSwitch, setdir ? HIGH : LOW);
//   digitalWrite(driverDir,setdir);
//   digitalWrite(driverPul,HIGH);
//   delayMicroseconds(pd);
//   digitalWrite(driverPul,LOW);
//   delayMicroseconds(pd);
// }




// // NOTE:to controll moter direction using pin set DIP switch(SW2) to OFF
// // to control dir using SW2 disable/comment codes related with dirPin

// // Define pin numbers
// const int pulsePin = 5;
// const int dirPin = 3;
// const int enablePin = 6;
// const int potPin = A1;
// const int dirButtonPin = 9;
// const int enableButtonPin = 8;

// // Variables for button states
// bool direction = LOW;
// bool motorEnabled = true;

// void setup() {
//     // Initialize serial communication
//     Serial.begin(9600);
//     // Set pin modes
//     pinMode(pulsePin, OUTPUT);
//     pinMode(dirPin, OUTPUT);
//     pinMode(enablePin, OUTPUT);
//     pinMode(dirButtonPin, INPUT_PULLUP);
//     pinMode(enableButtonPin, INPUT_PULLUP);

//     // Initial state
//     digitalWrite(dirPin, direction);
//     // digitalWrite(enablePin, motorEnabled ? HIGH : LOW);  // Enable is active LOW
//     digitalWrite(enablePin,LOW);  // Enable is active LOW

// }

// void loop() {
//     // Read potentiometer value
//     int potValue = analogRead(potPin);
//     int delayTime = map(potValue, 0, 1023, 40000, 10000);  // Map to appropriate delay for speed control

//     // Read direction button state
//     // if (digitalRead(dirButtonPin) == LOW) {
//     //     delay(50);  // Debounce delay
//     //     direction = !direction;
//     //     digitalWrite(dirPin, direction);
//     //     while (digitalRead(dirButtonPin) == LOW);  // Wait for button release
//     // }

//     // Read enable button state
//     if (digitalRead(enableButtonPin) == LOW) {
//         delay(50);  // Debounce delay
//         motorEnabled = !motorEnabled;
//         digitalWrite(enablePin, motorEnabled ? LOW : HIGH);  // Enable is active LOW
//         while (digitalRead(enableButtonPin) == LOW);  // Wait for button release
//     }

// // // If motor is enabled, generate pulses

//     if (motorEnabled) {
//         digitalWrite(pulsePin, HIGH); //sets the pulse pin high.
//         delayMicroseconds(delayTime);  //waits for the specified time
//         digitalWrite(pulsePin, LOW); //sets the pulse pin low.
//         delayMicroseconds(delayTime); // waits again.
//         Serial.println("motor is enable the state is  :: " + String(digitalRead(enablePin)));

//         // Adjust pulseDelay to control the speed. Lower values increase speed, and higher values decrease speed.
//         //the value is controlled with the POT

//     }

//     Serial.println("delay time speed :: " + String(delayTime));
//     Serial.println("motor state :: " + String(motorEnabled));
//     // Serial.println("direction of motor :: " + String(direction));
//     delay(1000);
// }





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

