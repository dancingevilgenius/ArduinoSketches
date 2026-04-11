/***************************************************************************************************************************** /
 * 
 * This library is designed to accompany the Custom Robot Control app.
 * Download the app: https://play.google.com/store/apps/details?id=com.lf.customrobotcontrol&listing=arduinolibrary
 * Documentation: https://musician-app-pro.web.app/manual.pdf#page=3
 * Tutorials: https://musician-app-pro.web.app/manual.pdf#page=4
 * 
/ *****************************************************************************************************************************/


#include "Custom_Robot_Control.h"

Custom_Robot_Control controller;

void setup() {
  Serial.begin(9600);
  
  // Start the Bluetooth® Low Energy controller
  controller.begin();
}

void loop() {

  // Must be called as often as possible; does essential background work
  controller.loop();

  // Only updates values if the app is connected
  if(controller.isConnected()){
    
    // Checks if index 0 of the Bluetooth® array is updated; prints the updated value
    if(controller.isUpdated(0))
      Serial.println("New Value: " + String(controller.read(0)));
    
    // Writes the number of uptime seconds to index 9 (the highest index) of the Bluetooth® array
    controller.write(9, (long)(millis() / 1000UL));
  
  }

}
