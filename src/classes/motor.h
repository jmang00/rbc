/**
 * A class for a motor.
 * 
 */

#pragma once

class Motor {
private:
  int _ena, _in1, _in2, _scaling;

public:
  Motor(int ena, int in1, int in2, int scaling = 1) {
    // Scaling factor should be something like 0.98 for the slower motor.
    // Store pin values
    _ena = ena;
    _in1 = in1;
    _in2 = in2;

    _scaling = scaling;
  };

  void init() {
    // Set pins to OUTput
    pinMode(_ena, OUTPUT);
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    delayMicroseconds(2);

    // Clear the pins
    digitalWrite(_ena, LOW);
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
    delayMicroseconds(2);

    Serial.print('Initialised motor!');
  }

  int getSpeed(int speed) {
    // Take in a speed as an integer
    // Range is 180-255
    switch (speed)
    {
    case 0:
      return 160;
    
    case 1:
      return 170;
    
    case 2:
      return 255;
  
    default:
      break;
    }

  }

  void stop() {
    digitalWrite(_ena, HIGH);
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
  };

  void forward(int speedCode) {
    // Speed is an int from 0-255
    analogWrite(_ena, round(getSpeed(speedCode)*_scaling));
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, LOW);
  };

  void backward(int speedCode) {
    digitalWrite(_ena, round(getSpeed(speedCode)*_scaling));
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, HIGH);
  };
};