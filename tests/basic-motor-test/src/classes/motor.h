/**
 * A class for a motor.
 * 
 * Define 8 speeds for the motor.
 */

#pragma once

class Motor {
private:
  /**
   * @brief Pin values
   * 
   */
  int _ena, _in1, _in2, _scaling;

  /**
   * @brief The speed (an integer from 0 to 7)
   * 
   */
  int _speed = 0;

  /**
   * @brief The direction (1 = forward, -1 = backward)
   * 
   */
  int _direction = 0;
  // TODO: could be useful to manually define how motor scales
  // int[] _speeds = {0, 0, 180, 190, 200, 210, 220, 230, 240, 250, 255};

public:
  /**
   * @brief Construct a new Motor object
   * 
   * @param ena the enable pin
   * @param in1 the input 1 pin
   * @param in2 the input 2 pin
   * @param scaling the scaling factor
   */
   Motor(int ena, int in1, int in2, int scaling = 1) {
        // Store pin values
        _ena = ena;
        _in1 = in1;
        _in2 = in2;

        _scaling = scaling;
    };

  /**
   * @brief Initialise the motor
   * 
   */
  void init() {
    // Set pins to OUTput
    pinMode(_ena, OUTPUT);
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    delayMicroseconds(2);

    // // Clear the pins
    // digitalWrite(_ena, LOW);
    // digitalWrite(_in1, LOW);
    // digitalWrite(_in2, LOW);
    // delayMicroseconds(2);
    analogWrite(_ena, 0);
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
    delayMicroseconds(2);

    Serial.print('Initialised motor!');
  }

  /**
   * @brief Get speed
   * 
   */
  int getSpeed() {
    return _speed;
  }

  /**
   * @brief Get direction
   * 
   */
  int getDirection() {
    return _direction;
  }

  /**
   * @brief Update the motor's speed
   * 
   * @param s an integer from 0 to 7 indicating the speed
   */
  float updateSpeed(int s) {
    if (_speed == s) {
      return;
    }

    int PWM = round((float)s/7*255);
    // return PWM;
    analogWrite(_ena, PWM);
    _speed = s;
  }

  /**
   * @brief Update the motor's direction.
   * 
   * @param d an integer -1 or 1
   */
  void updateDirection(int d) {
    if (_direction == d) {
      return;
    }

    if (d == 1) {
      float a = 0/0;
      digitalWrite(_in1, HIGH);
      digitalWrite(_in2, LOW);
    } else {
      digitalWrite(_in1, LOW);
      digitalWrite(_in2, HIGH);
    }
    _direction = d;
  } 

  /**
   * @brief Update speed and direction based on one number
   * 
   * @param n an integer from -7 to 7
   */
  void update(int n) {

    // Minimal version
    // updateSpeed(abs(n));

    // if (n >= 0) {
    //   updateDirection(1);
    // } else {
    //   updateDirection(-1);
    // }

    // Bare minimum version
    analogWrite(_ena, round((float)abs(n)/7*255));

    if (n > 1) {
      digitalWrite(_in1, HIGH);
      digitalWrite(_in2, LOW);
    } else {  
      digitalWrite(_in1, LOW);
      digitalWrite(_in2, HIGH);
    }
  }
};