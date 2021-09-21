#include <Servo.h>

Servo myservo;

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
}

void loop() {
  for (pos = 0; pos <= 180; pos += 1) {
    // in steps of 1 degree
    myservo.write(pos);
    digitalWrite(13, HIGH);
    digitalWrite(12, HIGH);
    delay(50);
    digitalWrite(13, LOW); 
    digitalWrite(12, LOW);
    delay(50);
  }
  for (pos = 180; pos >= 0; pos -= 1) {
    myservo.write(pos);
    digitalWrite(13, HIGH);
    digitalWrite(12, HIGH);
    delay(50);
    digitalWrite(13, LOW); 
    digitalWrite(12, LOW);
    delay(50);
  }
}
