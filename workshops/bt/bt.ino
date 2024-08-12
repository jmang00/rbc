
// Serial communication with Bluetooth HM-10 Slave
// Uses serial monitor for communication with Bluetooth HM-10
//
//  Arduino to HM-10 connections
//  Arduino pin 3 (TX) to voltage divider then to HM-10 RX
//  Arduino pin 2 to HM-10 TX
//  Connect GND from the Arduiono to GND on the HM-10
//
// When a command is entered in to the serial monitor on the computer
// the Arduino will relay the command to the HM-10


// Library to make a Software UART
#include <SoftwareSerial.h>

#define RX 3
#define TX 2

// Define the LED pin
#define LED1 7
#define LED2 8

#define BAUDRATE 9600              // speed of communication in bps
SoftwareSerial BTSerialM(RX, TX);  //Set RX and TX pins


void setup() {

  // Start Serial Monitor for feedback
  Serial.begin(BAUDRATE);

  // TODO: set up pin modes for LEDs
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  // Bluetooth serial connection in specific baud rate that we defined earlier
  BTSerialM.begin(BAUDRATE);
}

void loop() {
  String slave_data = "";

  // Communication between Slave HM-10 and Serial Monitor
  if (BTSerialM.available()) {
    while (BTSerialM.available()) {
      char c = BTSerialM.read();
      if (isPrintable(c)) {
        slave_data += c;
        Serial.write(c);

        switch (c) {
          case '1':
            digitalWrite(LED1, HIGH);  // turn the LED on (HIGH is the voltage level)
            delay(1000);                      // wait for a second
            digitalWrite(LED1, LOW);   // turn the LED off by making the voltage LOW
            delay(1000);                      // wait for a second
            break;

          case '2':
            digitalWrite(LED2, HIGH);  // turn the LED on (HIGH is the voltage level)
            delay(1000);                      // wait for a second
            digitalWrite(LED2, LOW);   // turn the LED off by making the voltage LOW
            delay(1000);                      // wait for a second
        }
      }
    }
  }

  // Check if there is data from the Serial Monitor
  if (Serial.available()) {
    char c = Serial.read();
    // Send data to both Master and Slave HM-10 Modules
    BTSerialM.write(c);
  }
}
