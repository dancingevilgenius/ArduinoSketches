int left_sensor=A1; //Micro Line Sensor Connected to Analog 1
int right_sensor=A2; //Micro Line Sensor Connected to Analog 1
int line_white_left, line_white_right; //We declared some variable for storing ML1 output

void setup() {
   pinMode(right_sensor, INPUT); //ML1 sensor input added as digital input
   pinMode(left_sensor, INPUT); //ML1 sensor input added as digital input
   Serial.begin(9600); // Serial communication started with 9600 bits per second.
}

void loop() {
  line_white_left=!digitalRead(left_sensor); // It will be read between 0 to 1023 by Analog Reading
  line_white_right=!digitalRead(right_sensor); // It will be read between 0 to 1023 by Analog Reading
  Serial.print("Left: "); 
  Serial.print(line_white_left); 
  Serial.print(" Right: ");
  Serial.println(line_white_right);
  delay(100);  // we are adding 100 ms delay for delay at readings. 
  }

