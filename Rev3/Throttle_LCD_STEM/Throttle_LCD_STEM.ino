#include <LiquidCrystal_I2C_STEM.h>

LiquidCrystal_I2C_STEM lcd(0x27, 20, 4);  // set the LCD address to 0x27 for a 20 chars and 4 line display

void setup() {

initLCD20X4();  
}

void initLCD20X4(){
    lcd.init();       // initialize the lcd
    lcd.backlight();  // Set backlight on

    // Print a message to the LCD.
    lcd.setCursor(0, 0);
    lcd.print("Initializing");
    lcd.setCursor(0, 1);
    lcd.print("Throttle: 0");
    lcd.setCursor(0, 2);
    lcd.print("AMPS: 0");

}

void loop() {
}
