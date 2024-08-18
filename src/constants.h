/**
 * Define constants here
 * 
 */
#pragma once


/**
 * @brief Control options
 * 
 */
#define CONTROLLER_DEADZONE 2
#define PINCER_OPEN_MODE 't'        // 't' for toggle, 'h' for hold
#define PINCER_CLOSE_MODE 't'
#define PINCER_SPEED 10             // 

/**
 * @brief Motors
 * 
 */
// TODO: add motor scaling constants here
// TODO: add 180 spin sensitivity
#define SPIN_TIME 1000  // in milliseconds

/**
 * @brief Servos
 * 
 */
#define LEFT_SERVO_OPEN 90
#define LEFT_SERVO_CLOSED 0
#define RIGHT_SERVO_OPEN 0
#define RIGHT_SERVO_CLOSED 120

/**
 * @brief Comms
 * 
 */
#define SERIAL_BAUDRATE 9600
#define BLUETOOTH_BAUDRATE 9600

/** 
 * @brief Maths
 */
#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
