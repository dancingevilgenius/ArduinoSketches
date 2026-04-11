#include "Custom_Robot_Control.h"

Custom_Robot_Control controller; // See the complete example for more information.

void setup() {
  controller.begin(); // This line starts the Bluetooth® Low Energy controller.
}

void loop() {
  controller.loop(); // Must be called as often as possible; does essential background work.
}
