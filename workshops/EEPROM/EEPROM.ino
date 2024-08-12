#include <EEPROM.h>

int address = 0;
byte value;
  
void setup() {

//  Sets onboard LED to pin 13
  pinMode(13,OUTPUT);
  
  Serial.begin(9600);

  
  

//  Loops through each byte of the EEPROM from 0 up to size of EEPROM and sets it to 0
//  for (int i = 0 ; i < EEPROM.length() ; i++) {
//    EEPROM.write(i, 0);
//  }

}

void loop(){
  
  //  Sets value to the EEPROM value stored in address location i
  Serial.print("CUM");
    for (int i = 0 ; i < EEPROM.length() ; i++) {
      value=EEPROM.read(i);
      Serial.print(i);
      Serial.print("\t");
      Serial.print(value,DEC);
      Serial.println();
      delay(100);
    }
}