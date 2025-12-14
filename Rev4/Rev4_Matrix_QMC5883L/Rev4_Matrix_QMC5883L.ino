#include "Arduino_LED_Matrix.h"
#include <Wire.h>
#include "SparkFun_Qwiic_Joystick_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_joystick
#include <QMC5883LCompass.h>

QMC5883LCompass compass;
JOYSTICK joystick; 
ArduinoLEDMatrix matrix;




uint8_t blank[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

uint8_t error[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0 },
  { 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0 },
  { 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0 },
  { 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0 },
  { 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0 },
  { 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};



uint8_t center_dot[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};



uint8_t north[8][12] = {
  { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

uint8_t south[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 }
};


uint8_t east[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};


uint8_t west[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};



uint8_t north_east[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};


uint8_t south_east[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1 }
};


uint8_t north_west[8][12] = {
  { 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};


uint8_t south_west[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};


void setup() {
  
  Serial.begin(9600);

  Wire1.begin();

  initMatrix();

  initJoystick();

  initCompass();
}

void initCompass(){
  compass.init();



  // compass.setCalibrationOffsets(123.00, 68.00, -575.00);
  // compass.setCalibrationScales(1.05, 0.98, 0.97);  

  compass.setCalibrationOffsets(67.00, 161.00, 386.00);
  compass.setCalibrationScales(1.06, 0.99, 0.96);


}


void initJoystick(){
  if(joystick.begin(Wire1) == false)
  {
    Serial.println("Joystick does not appear to be connected. Please check wiring. Freezing...");
    matrix.renderBitmap(error, 8, 12);    
    while(1);
  } else {
    Serial.println("Joystick initialized.");
  }
}

void initMatrix(){
  matrix.begin();
}






void loop(){

  //clockwiseArrows();
  //readJoystick();
  readCompass();

}


void readCompass(){

  int x, y, z;
    char myArray[3];

  // Clear the screen
  matrix.renderBitmap(blank, 8, 12);


  compass.read();

  byte a = compass.getAzimuth();
  // Output here will be a value from 0 - 15 based on the direction of the bearing / azimuth.
  byte b = compass.getBearing(a);


  
  Serial.print("B: ");
  Serial.print(b);
  Serial.println();

/*
0   N
1   NE
2   NE
3   E
4   SE
5   SE
6   S
7   SW
8   SW
9   W
10  NW
11  NW
*/

  switch (b) {
    case 0:
      matrix.renderBitmap(north, 8, 12);    
      break;
    case 1:
      matrix.renderBitmap(north_east, 8, 12);    
      break;
    case 2:
      matrix.renderBitmap(north_east, 8, 12);    
      break;
    case 3:
      matrix.renderBitmap(east, 8, 12);    
      break;
    case 4:
      matrix.renderBitmap(south_east, 8, 12);    
      break;
    case 5:
      matrix.renderBitmap(south_east, 8, 12);    
      break;
    case 6:
      matrix.renderBitmap(south, 8, 12);    
      break;
    case 7:
      matrix.renderBitmap(south_west, 8, 12);    
      break;
    case 8:
      matrix.renderBitmap(south_west, 8, 12);    
      break;
    case 9:
      matrix.renderBitmap(west, 8, 12);    
      break;
    case 10:
      matrix.renderBitmap(north_west, 8, 12);    
      break;
    case 11:
      matrix.renderBitmap(north_west, 8, 12);    
      break;
    case 12:
      matrix.renderBitmap(west, 8, 12);    
      break;
    case 13:
      matrix.renderBitmap(west, 8, 12);    
      break;
    case 14:
      matrix.renderBitmap(north_west, 8, 12);    
      break;
    case 15:
      matrix.renderBitmap(north, 8, 12);    
      break;
    default:
      matrix.renderBitmap(error, 8, 12);    
      break;
  }



  Serial.print(" Direction: ");
  Serial.print(myArray[0]);
  Serial.print(myArray[1]);
  Serial.print(myArray[2]);




  delay(250);

}



void readJoystick(){

  // Clear the screen
  matrix.renderBitmap(blank, 8, 12);

    int X = joystick.getHorizontal();
    int Y = joystick.getVertical();
    int B = joystick.getButton();

    String east_west = "";
    String north_south = "";

    if  (X > 700)
    {
        Serial.println("W");
        east_west = "W";
        matrix.renderBitmap(west, 8, 12);    
    }
    else if (X < 300)
    {      
        Serial.println("E");
        east_west = "E";
        matrix.renderBitmap(east, 8, 12);    
    } 

    if  (Y > 700)
    {
        Serial.println("N");
        north_south="N";
        matrix.renderBitmap(north, 8, 12);    
    }
    else if (Y < 300)
    {
        Serial.println("S");
        north_south = "S";
        matrix.renderBitmap(south, 8, 12);    
    }


    if(east_west == "E"){

      if(north_south == "N"){
        matrix.renderBitmap(north_east, 8, 12);    
      } else if (north_south == "S"){
        matrix.renderBitmap(south_east, 8, 12);    
      } else {
        matrix.renderBitmap(east, 8, 12);    
      }

    } else if(east_west == "W"){
      if(north_south == "N"){
        matrix.renderBitmap(north_west, 8, 12);    
      } else if (north_south == "S"){
        matrix.renderBitmap(south_west, 8, 12);    
      } else {
        matrix.renderBitmap(west, 8, 12);    
      }
    } else if (north_south == "N"){
      matrix.renderBitmap(north, 8, 12);    
    } else if (north_south == "S") {
      matrix.renderBitmap(south, 8, 12);    
    }



    if (B == 0)
    {
        Serial.println("Button");
        matrix.renderBitmap(center_dot, 8, 12);    
    }



  delay(200);

}



void clockwiseArrows(){
// Blank
delay(1000);
matrix.renderBitmap(blank, 8, 12);




delay(1000);
matrix.renderBitmap(north, 8, 12);


delay(1000);
matrix.renderBitmap(north_east, 8, 12);


delay(1000);
matrix.renderBitmap(east, 8, 12);


delay(1000);
matrix.renderBitmap(south_east, 8, 12);

delay(1000);
matrix.renderBitmap(south, 8, 12);


delay(1000);
matrix.renderBitmap(south_west, 8, 12);


delay(1000);
matrix.renderBitmap(west, 8, 12);


delay(1000);
matrix.renderBitmap(north_west, 8, 12);


}
