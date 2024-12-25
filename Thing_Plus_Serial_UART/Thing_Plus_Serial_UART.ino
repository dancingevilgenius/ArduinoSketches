// --------------------------------------
// UART example using Serial1
//
//
// This sketch prints "Hello World!" every second
// using the secondary UART pins D0 and D1.
//



HardwareSerial Serial1(D0, D1); //Attach Serial1 to D0 and D1. RX:43/17 TX:42/16

void setup() {
  Serial1.begin(115200);
  while (!Serial1) {
    ; // wait for serial port to connect. Needed for Native USB only
  }
  Serial1.println("Goodnight moon!");

}

void loop() {
  Serial1.println("Hello World!");
  delay(1000);
}
