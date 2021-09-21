#include <Servo.h>

Servo leftServo;
Servo rightServo;

//Wheel diameter = 2.5in
//Circumference of wheel = 2*pi*r = 7.86in
int full = 6980; //100% = 43.75rpm @1400/1600 us
int half = 10000; //50% = 28.25rpm @1450/1550 us
int quarter = 21000; //25% = 11.75rpm @1475/1525 us

void setup() {
  tone(4, 3000, 1000);
  delay(1000);
  
  leftServo.attach(13);
  rightServo.attach(12);

  //leftServo.writeMicroseconds(1385);
  //rightServo.writeMicroseconds(1600);
  //delay(full);

  //leftServo.writeMicroseconds(1448);
  //rightServo.writeMicroseconds(1550);
  //delay(half);

  leftServo.writeMicroseconds(1472);
  rightServo.writeMicroseconds(1525);
  delay(quarter);

  leftServo.detach();
  rightServo.detach();
}

void loop(){
  
}
