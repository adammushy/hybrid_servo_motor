#include <SoftwareSerial.h>

SoftwareSerial espSerial(2, 3); // RX, TX

int powerStatus = 1;  // Example: 1 for on, 0 for off
int direction = 1;    // Example: 1 for forward, 0 for reverse
int speed = 100;      // Example: speed value (0-255)

void setup() {
  Serial.begin(9600);
  espSerial.begin(9600);
}

void loop() {
  // Send the data as a comma-separated string
  espSerial.print(powerStatus);
  espSerial.print(",");
  espSerial.print(direction);
  espSerial.print(",");
  espSerial.println(speed);

  delay(1000); // Send data every second
}
