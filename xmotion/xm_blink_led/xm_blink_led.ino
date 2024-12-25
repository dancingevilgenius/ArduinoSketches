int UserLed1 = 9; //Led Pin is D9
void setup() {
pinMode(UserLed1, OUTPUT); // Led as output element
}
void loop() {
digitalWrite(UserLed1, HIGH); // turn on the User Led 1, High Statement
delay(500); // wait for half second (500 ms)
digitalWrite(UserLed1, LOW); // turn off the User Led 1, Low Statement
delay(500); // wait for half second (500 ms)
}
