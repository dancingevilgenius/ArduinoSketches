/*
  Custom_Robot_Control.h - Library for communicating with the Custom Robot Control app over Bluetooth.
  Created by L. Franklin, July 15, 2025.
  Updated by L. Franklin, July 15, 2025.
*/

#ifndef Custom_Robot_Control_h
#define Custom_Robot_Control_h

#include "Arduino.h"
#include "ArduinoBLE.h"


class Custom_Robot_Control
{
  public:
    Custom_Robot_Control();
    bool begin();
    bool loop();
    bool isConnected();
    bool write(long index, long value);
    long read(long index);
    bool isUpdated(long index);
  private:
      bool readBuffer();
      bool writeBuffer();
      void intsToBytes();
      void bytesToInts();
      String _name;
      long longArray[10];
      long longArrayWritten[10];




};

#endif

