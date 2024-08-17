/**
 * The main sketch.
 * 
 */

// Arduino Libraries
#include <math.h>
#include <Dabble.h>

// Local Imports
#include "src/pins.h"
#include "src/classes/motor.h"
#include "src/constants.h"
#include "src/util.h"

// Dabble definitions
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE

// ------------------------------ GLOBAL VARIABLES ------------------------------
// everything declared here can be used in loop()
int frame = 0;  // keeps track of the number of frames that have passed
int frameStart, frameEnd;   // record frame times
int debug_level = 0; // 0=nothing, 1=serial controls + basics, 2=everythings

char serialCharacter; // serial input character

// Controller inputs
float joystickRadius, // an integer between 0 and 7
      joystickAngle;  // an angle in RADIANS

// Motors`
Motor leftMotor(ENA, IN1, IN2);
Motor rightMotor(ENB, IN3, IN4);

// ------------------------------ THE MAIN BIT ------------------------------

/**
 * @brief Runs once when you press reset or power the board
 * 
 */
void setup() {
  // Initialise comms
  Serial.begin(SERIAL_BAUDRATE);
  Dabble.begin(BLUETOOTH_BAUDRATE, RX, TX);

  // Initialise motors
  leftMotor.init();
  rightMotor.init();
}

/**
 * @brief The main loop
 * 
 */
void loop() {
  startFrame();

  if (debug_level > 0) {
    processSerialInput();
  }

  processControllerInput();

  updateMotorsFromJoyStick();

  endFrame();
}


// -------------------------- Functions called in the main loop ----------------

/**
 * @brief Run at the start of each frame
 * 
 */
void startFrame() {
    frame += 1;
    frameStart = millis();
}

/**
 * @brief Process the input from the serial
 * 
 */
void processSerialInput() {
  // Handle Serial input
  if (Serial.available()) {
    serialCharacter = Serial.read();

    // Quit on 'q'
    if (serialCharacter=='q') {
      Serial.end();
      return;
    }
    // Echo any other character
    else {
      Serial.write(serialCharacter);
    }
  }
}

/**
 * @brief Process input from the controller.
 * 
 */
void processControllerInput() {
  Dabble.processInput(); 

  joystickRadius = GamePad.getRadius();
  joystickAngle = GamePad.getAngle() * DEG_TO_RAD;  // convert to radians
}

/**
 * @brief Updates the motors bases on the joystick position
 * 
 */
void updateMotorsFromJoyStick() {
  if (joystickRadius >= CONTROLLER_DEADZONE) {
    // Have a deadzone in the middle
    leftMotor.update(
      joystickRadius * calcL(joystickAngle)
    );

    rightMotor.update(
      joystickRadius * calcR(joystickAngle)
    );
  }
  else {
    leftMotor.update(0);
    rightMotor.update(0);
  }
}

/**
 * @brief Run at the end of each frame
 * 
 */
void endFrame() {
  frameEnd = millis();

  if (debug_level > 0) {
    Serial.print("FPS: ");
    Serial.println(1000/(frameEnd-frameStart));
    Serial.println();

    Serial.println();
  }
}
