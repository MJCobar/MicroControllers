#include <Servo.h>

Servo leftServo;
Servo rightServo;

void setup() {
  tone(4, 3000, 1000);
  delay(1000);
  
  leftServo.attach(13);
  rightServo.attach(12);
  int meter_time = 6980; //100% = 43.75rpm @1400/1600 us
  int half_meter = 3490;

  //Forwards
  leftServo.writeMicroseconds(1385);
  rightServo.writeMicroseconds(1600);
  delay(meter_time);

  //Right Turn
  leftServo.writeMicroseconds(1600);
  rightServo.writeMicroseconds(1600);
  delay(580);

  //Forwards
  leftServo.writeMicroseconds(1385);
  rightServo.writeMicroseconds(1600);
  delay(half_meter);

  //Left Turn
  leftServo.writeMicroseconds(1400);
  rightServo.writeMicroseconds(1400);
  delay(580);

  //Backwards
  leftServo.writeMicroseconds(1600);
  rightServo.writeMicroseconds(1400);
  delay(half_meter);

  //180 Turn
  leftServo.writeMicroseconds(1400);
  rightServo.writeMicroseconds(1400);
  delay(1180);

  leftServo.detach();
  rightServo.detach();
}

void loop() {
  // put your main code here, to run repeatedly:

}
