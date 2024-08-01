const int stepPin=5;
const int dirPin=3;
const int enaPin =6;
const int potPin = A1;

//define consts
const int stepPerRevolution = 1600;
const unsigned long stepDelay =5000;

void setup() {
  // seting ppins
  Serial.begin(9600);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enaPin,OUTPUT);
  digitalWrite(enaPin,LOW); // enable the motor driver


}

void loop() {
  // int potValue = analogRead(potPin);
  int potValue = analogRead(potPin);
  Serial.println("Potentiometer value: " + String(potValue)); // Debug statement

  int stepDelay = map(potValue, 0, 1023, 10000, 4000);
  Serial.println("delay time speed :: " + String(stepDelay));
  // rotate 10 seconds to the right
  rotate(stepPerRevolution,stepDelay,false);

  // rest for 5secs
  delay(500);

  // // rotates 5 seeconds to the left
  // rotate(stepPerRevolution/2,stepDelay,true);

  // // rest fro 5 secs
  // delay(500);

}

void rotate(int steps, unsigned long stepDelay, bool clockwise){
  digitalWrite(dirPin,clockwise ? HIGH : LOW);// SET rotation direction
  for (int i = 0; i<steps; i++){
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(stepDelay);
  }
}
