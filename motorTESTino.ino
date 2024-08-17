/**
 * The main file.
 * 
 */

// // Include Dabble
// #define CUSTOM_SETTINGS
// #define INCLUDE_GAMEPAD_MODULE
// #include <Dabble.h>

#include "src/pins.h"
#include "src/classes/motor.h"

Motor leftMotor(ENA, IN1, IN2, 1);
Motor rightMotor(ENB, IN3, IN4, 1);


// the setup function runs once when you press reset or power the board
void setup() {

  Serial.begin(115200);
  // leftMotor.init();
  // rightMotor.init();
  
  // old "manual" code
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  delayMicroseconds(2);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  delayMicroseconds(2);

  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  analogWrite(ENB, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(1000); 
}

void loop() {
  // leftMotor.forward(2);
  // rightMotor.forward(2);


  // old
  for (int i = 0; i <= 7; i++) {
  int speedCalc = round(200+55 * i / 7);
  int speed = 255; //

  Serial.print(speedCalc);
  Serial.println();

  analogWrite(ENA, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  analogWrite(ENB, speed);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  delay(5000);
  }

}

