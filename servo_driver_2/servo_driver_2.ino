
// Include necessary libraries
#include <PWM.h>

/*
  Stepper Motor Control with Arduino Uno
  Uses HSS86 Stepper Driver
*/
/*
  Stepper Motor Control with Arduino Uno
  Uses HSS86 Stepper Driver
*/

// Define pins
int driverPUL = 9;    // PUL- pin for PWM
int driverDIR = 6;    // DIR- pin for direction
int RUNSTOP = 8;      // RUN/STOP control
int spd = A0;         // Potentiometer for speed control

// Variables
unsigned long pd;       // Pulse Delay period
int value;

void setup() {
  // Initialize pins
  pinMode(driverPUL, OUTPUT);
  pinMode(driverDIR, OUTPUT);
  pinMode(RUNSTOP, INPUT_PULLUP); // Use internal pull-up resistor
  pinMode(spd, INPUT);

  // Set initial states
  digitalWrite(driverDIR, LOW); // Set initial direction
  
  Serial.begin(9600); // Start serial communication for debugging
}

void loop() {
  // Read speed from potentiometer
  value = analogRead(spd);
  pd = map(value, 0, 1023, 1000, 10000); // Map speed to pulse delay

  // Check RUN/STOP pin
  int runstopState = digitalRead(RUNSTOP);
  if (runstopState == LOW) {
    Serial.println("Motor running");
    // Motor running
    pulseMotor(pd);
  } else {
    Serial.println("Motor stopped");
    // Motor stopped
    digitalWrite(driverPUL, LOW);
  }

  delay(100); // Add delay for stability
}

// Function to generate pulses for the motor
void pulseMotor(unsigned long pulseDelay) {
  digitalWrite(driverPUL, HIGH);
  delayMicroseconds(pulseDelay / 2);
  digitalWrite(driverPUL, LOW);
  delayMicroseconds(pulseDelay / 2);
}















// Define pins
// int driverPUL = 9;    // PUL- pin for PWM
// int driverDIR = 6;    // DIR- pin for direction
// int driverPEND = 5;   // PEND- pin (Position End)
// int driverALRM = 4;   // ALRM- pin (Alarm)
// int driverATU = 7;    // ATU- pin (Alarm Trigger)
// int RUNSTOP = 8;      // RUN/STOP control
// int spd = A0;         // Potentiometer for speed control

// // Variables
// unsigned long pd;       // Pulse Delay period
// boolean setdir = LOW;   // Set Direction
// int value;
// int filtered;

// void setup() {
//   // Initialize pins
//   pinMode(driverPUL, OUTPUT);
//   pinMode(driverDIR, OUTPUT);
//   pinMode(driverPEND, INPUT);
//   pinMode(driverALRM, INPUT);
//   pinMode(driverATU, INPUT);
//   pinMode(RUNSTOP, OUTPUT);
//   pinMode(spd, INPUT);

//   // Set initial states
//   digitalWrite(RUNSTOP, HIGH); // STOP
// }

// void loop() {
//   // Read speed from potentiometer
//   value = analogRead(spd);
//   pd = map(value, 0, 1023, 1000, 10000); // Map speed to pulse delay

//   // Check direction and set it
//   if (digitalRead(driverATU) == HIGH) {
//     setdir = !setdir;
//     digitalWrite(driverDIR, setdir);
//   }

//   // Check for RUN/STOP condition
//   if (digitalRead(driverATU) == LOW) {
//     digitalWrite(RUNSTOP, HIGH); // STOP
//     digitalWrite(driverPUL, LOW);
//   } else {
//     digitalWrite(RUNSTOP, LOW);  // RUN
//     pulseMotor(pd);
//   }
// }

// // Function to generate pulses for the motor
// void pulseMotor(unsigned long pulseDelay) {
//   digitalWrite(driverPUL, HIGH);
//   delayMicroseconds(pulseDelay);
//   digitalWrite(driverPUL, LOW);
//   delayMicroseconds(pulseDelay);
// }
