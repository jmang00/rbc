/**
 * The main sketch.
 * 
 */

// Arduino Libraries
#include <math.h>
#include <Dabble.h>
#include <Servo.h>

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
int debug_level = 3; // 0=nothing, 1=serial controls + basics, 2=everything, 3=everything + delay

char serialCharacter; // serial input character

// Controller inputs
float joystickRadius, // an integer between 0 and 7
      joystickAngle;  // an angle in RADIANS

int leftMotorSpeed, rightMotorSpeed;
int leftServoAngle, rightServoAngle;

// bool updateMotorsFlag = false;
// bool updateServosFlag = false;

// Motors`
Motor leftMotor(ENA, IN1, IN2, 1);
Motor rightMotor(ENB, IN3, IN4, 1);

// Servos
Servo leftServo;
Servo rightServo;

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

  // Initialise servos
  leftServo.attach(LEFT_SERVO);
  rightServo.attach(RIGHT_SERVO);
  leftServo.write(LEFT_SERVO_OPEN);
  rightServo.write(RIGHT_SERVO_OPEN);
}

/**
 * @brief The main loop
 * 
 */
void loop() {
  // // Motor test
  // // Comment out 
  // leftMotor.update(7);
  // rightMotor.update(7);
  // delay(5000);
  startFrame();

  // if (debug_level >= 1) {
  //   processSerialInput();
  // }

  processControllerInput();

  // for (int i = 0; i < 4; i++) {
  //   Serial.print("Angle: ");
  //   Serial.println(90 * i);
  //   joystickAngle = 90 * i * DEG_TO_RAD;
  //   joystickRadius = 7;
  //   updateMotorsFromJoyStick();
  //   delay(5000);
  // }

  // joystickAngle = 90 * DEG_TO_RAD;
  // joystickRadius = 7;
  updateMotorsFromJoyStick();

  // currently updated in processSerialInput()
  // updateServos();

  
  if (debug_level >= 3) {
    delay(5000);
  }

  endFrame();
}


// -------------------------- Functions called in the main loop ----------------

/**
 * @brief Run at the start of each frame
 * 
 */
void startFrame() {
    if (debug_level >= 2) {
      Serial.print("Frame: ");
      Serial.println(frame);
    }

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

  // // Flip the controls if needed
  // if (flippedControls) {
  //   // joystickAngle = 2*PI-joystickAngle;

  //   joystickAngle += PI;
  //   if (joystickAngle > TWO_PI) {
  //     joystickAngle -= TWO_PI;
  //   }
  // }

  if (debug_level >= 2) {
    Serial.print("Joystick Radius: ");
    Serial.println(joystickRadius);
    Serial.print("Joystick Angle: ");
    Serial.println(joystickAngle);
  }

  // X = close servos
  if (GamePad.isCrossPressed()) {
    Serial.println("Pressed CROSS.");
    leftServo.write(LEFT_SERVO_CLOSED);
    rightServo.write(RIGHT_SERVO_CLOSED);

    // TODO: only update servos when necessary
    // switch (PINCER_CLOSE_MODE)
    // {
    // case 't'/:
    //   leftServoAngle = LEFT_SERVO_CLOSED;
    //   rightServoAngle = RIGHT_SERVO_CLOSED;
    //   break;
    
    // case 'h'/:
    //   leftServoAngle += LEFT_SERVO_SPEED;
    //   rightServoAngle += RIGHT_SERVO_SPEED;
    //   break;
    // default:
    //   break;
    // }
    
  }
  else if (GamePad.isCirclePressed()) {
    Serial.println("Pressed CIRCLE.");
    leftServo.write(LEFT_SERVO_OPEN);
    rightServo.write(RIGHT_SERVO_OPEN);
    delay(2000);
  }
  
  else if (GamePad.isSquarePressed()) {
    Serial.println("Pressed SQUARE.");
    flippedControls = !flippedControls;
    delay(2000);
  }

  // else if (GamePad.isTrianglePressed()) {
  //   Serial.println("Pressed TRIANGLE.");

  // }

}

/**
 * @brief Updates the motors bases on the joystick position
 * 
 */
void updateMotorsFromJoyStick() {
  if (debug_level >= 2) {
    Serial.print("calcL result:");
    Serial.println(calcL(joystickAngle));
    Serial.print("calcR result:");
    Serial.println(calcL(joystickAngle));
  }

  // Have a deadzone in the middle
  if (joystickRadius > CONTROLLER_DEADZONE) {
    leftMotorSpeed = joystickRadius * calcL(joystickAngle);
    rightMotorSpeed = joystickRadius * calcR(joystickAngle);   
  }
  else {
    leftMotorSpeed = 0;
    rightMotorSpeed = 0;
  }

  leftMotor.update(leftMotorSpeed);
  rightMotor.update(rightMotorSpeed);

  if (debug_level >= 2) {
    Serial.print("Left motor speed set to ");
    Serial.println(leftMotorSpeed);
    Serial.print("Right motor speed set to ");
    Serial.println(rightMotorSpeed);
  }
}

// TODO: actually use this
// /**
//  * @brief Update the servos based on the current angle
//  * 
//  */
// void updateServos() {
//   leftServo.write(leftServoAngle);
//   rightServo.write(rightServoAngle);

//     if (debug_level >= 2) {
//       Serial.print("Left Servo angle set to: ");
//       Serial.println(leftMotorSpeed);
//       Serial.print("Right Speed: ");
//       Serial.println(rightMotorSpeed);
//     }
// }

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
