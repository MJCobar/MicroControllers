#include <Servo.h>

Servo leftServo;
Servo rightServo;

void setup() {
  tone(4, 3000, 1000);
  delay(1000);

  Serial.begin(9600);
  leftServo.attach(13);
  //rightServo.attach(12);
  
  //leftServo.writeMicroseconds(1500);
  //rightServo.writeMicroseconds(1500);
}

void loop() {
  int pulseWidth = 1700;
  Serial.print("pulseWidth = ");
  Serial.println(pulseWidth);
  Serial.println("Press a key and click");
  Serial.println("Send to start servo...");

  while(Serial.available() == 0);
  Serial.read();

  Serial.println("Running...");
  leftServo.writeMicroseconds(pulseWidth);
  delay(6000);
  leftServo.writeMicroseconds(1500);
  Serial.read();
}
