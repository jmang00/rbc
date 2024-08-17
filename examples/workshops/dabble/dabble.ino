/*
   Gamepad module provides three different mode namely Digital, JoyStick and Accerleometer.

   You can reduce the size of library compiled by enabling only those modules that you want to
   use. For this first define CUSTOM_SETTINGS followed by defining INCLUDE_modulename.

   Explore more on: https://thestempedia.com/docs/dabble/game-pad-module/
*/
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <Dabble.h>

#define RX 3
#define TX 2

#define SERIAL_BAUD 115200
#define BLUETOOTH_BAUD 9600

void setup() {
  // put your setup code here, to run once:
  Serial.begin(SERIAL_BAUD);      // make sure your Serial Monitor is also set at this baud rate.
  Dabble.begin(BLUETOOTH_BAUD, RX, TX);      //Enter baudrate of your bluetooth.Connect bluetooth on Bluetooth port present on evive.
}

void loop() {
  Dabble.processInput();             //this function is used to refresh data obtained from smartphone.Hence calling this function is mandatory in order to get data properly from your mobile.

  // Serial.print("KeyPressed: ");
  if (GamePad.isRightPressed()){
    Serial.print("R");
  }
  else if (GamePad.isLeftPressed()){
    Serial.print("L");
  }
}