#include <Servo.h>

Servo leftServo;
Servo rightServo;

void setup() {
  leftServo.attach(13);
  rightServo.attach(12);
  
  leftServo.writeMicroseconds(1500);
  rightServo.writeMicroseconds(1500);
}

void loop() {

}
