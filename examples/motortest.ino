/**
 * @brief Just grabbed bits from main code.. could make this into a proper "test" case
 * 
 */
 
  // testing motor
int motor = 0; // 0 = left, 1 = right
  
  // // COMMAND TO TEST MOTOR
  // String motor = Serial.readString();
  // int speed = Serial.parseInt();

  // //eg. L 255
  // if (motor == "L") {
  //   leftMotor.update(speed);
  // }
  // else if (motor == "R") {
  //   rightMotor.update(speed);
  // }
  // /////

    ///// CYCLE MOTOR SPEEDS ////
  for (int i=0; i<=7; i++) {
    //print motor speeed
    Serial.print("Motor speed: ");
    Serial.println(i);

    // leftMotor.update(i);
    // rightMotor.update(i);
    delay(5000);
  }
  /////////////////////////////