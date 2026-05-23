/* 
 * Maker-ESP32 DC Motor Test Sketch
 * Specifically for the Null Labs Maker-ESP32 integrated driver.
 */

// Define motor pins based on Maker-ESP32 hardware layout
// Port M1
// #define M1_A 12 
// #define M1_B 13
// // Port M2
// #define M2_A 25
// #define M2_B 26
// // Port M3
// #define M3_A 27
// #define M3_B 14
// // Port M4
// #define M4_A 32
// #define M4_B 33



#define M1_A 27
#define M1_B 13
// Port M2
#define M2_A 4
#define M2_B 2
// Port M3
#define M3_A 17
#define M3_B 12
// Port M4
#define M4_A 15
#define M4_B 14


void setup() {
  Serial.begin(115200);
  
  // Initialize all motor pins as outputs
  pinMode(M1_A, OUTPUT); pinMode(M1_B, OUTPUT);
  pinMode(M2_A, OUTPUT); pinMode(M2_B, OUTPUT);
  pinMode(M3_A, OUTPUT); pinMode(M3_B, OUTPUT);
  pinMode(M4_A, OUTPUT); pinMode(M4_B, OUTPUT);

  Serial.println("Maker-ESP32 Motor Test Started...");
}

void driveMotor(int pinA, int pinB, String direction) {
  if (direction == "forward") {
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
  } else if (direction == "backward") {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
  } else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
  }
}

void loop() {
  // Test Motor 1
  Serial.println("Testing M1 Forward");
  driveMotor(M1_A, M1_B, "forward");
  delay(2000);
  
  Serial.println("Testing M1 Backward");
  driveMotor(M1_A, M1_B, "backward");
  delay(2000);
  
  driveMotor(M1_A, M1_B, "stop");
  delay(1000);

  // Repeat or expand for M2, M3, M4 as needed
}
