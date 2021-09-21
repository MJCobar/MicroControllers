// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 and 12 as an output.
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(13, HIGH);
  digitalWrite(12, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(500);                       // wait
  digitalWrite(13, LOW); 
  digitalWrite(12, LOW); // turn the LED off by making the voltage LOW
  delay(500);                       // wait
}
