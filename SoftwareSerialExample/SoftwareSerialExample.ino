/*
  Software serial multple serial test

 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.

 The circuit:
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)

 created back in the mists of time
 modified 25 May 2012
 by Tom Igoe
 based on Mikal Hart's example

 This example code is in the public domain.

 */
#include <SoftwareSerial.h>
#include <Arduino.h>

SoftwareSerial mySerial(43, 42); // RX, TX

void setup() {
  // Open serial communications and wait for port to open:
  mySerial.begin(9600);
  while (!mySerial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  mySerial.println("Goodnight moon!");

  // set the data rate for the SoftwareSerial port
  mySerial.begin(4800); // 4800
  mySerial.println("Hello, world?");
}

void loop() { // run over and over
  if (mySerial.available()) {
    mySerial.write(mySerial.read());
  }
  if (mySerial.available()) {
    mySerial.write(mySerial.read());
  }
}

