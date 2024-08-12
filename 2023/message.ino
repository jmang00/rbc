// --------------------------------------- IMPORTS -----------------------------------------------------
#include <NewPing.h>
#include <SoftwareSerial.h>
#include <cppQueue.h>
#define	IMPLEMENTATION LIFO

// --------------------------------------- CONSTANTS -----------------------------------------------------

#define BAUDRATE 9600

// Maze size
#define GRID_SIZE 4
#define NO_CELLS 16
#define CELL_SIZE 40 // in cm

// Starting conditions
#define START_DIRECTION 1 // define the start direction as north
#define START_INSTRUCTION 0
#define START_STATE 0 // start going forward

#define START_CELL 0 // start at cell 0 (bottom left)
#define START_CELL_COLOR 3

#define DESTINATION_CELL 4
#define DESTINATION_CELL_COLOR 3


// Car geometry
#define colorSensorDist 0.5

#define leftSonarDist 5
#define leftSonarAngle 0.523599 // 30deg in radians

#define centerSonarDist 2

#define rightSonarDist 5
#define rightSonarAngle 0.523599 // 30deg in radians

#define carWidth 13
#define carHeight 15

// Colors:
// 0 = black
// 1 = white
// 2 = blue
// 3 = green
// 4 = purple
// 5 = yellow
// 6 = red

// Cardinal Directions:
// 0 = north/up
// 1 = east/right
// 2 = south/down
// 3 = west/left

// Relative Directions (for instructions)
// 0 = forward
// 1 = right
// 2 = backward (spin around)
// 3 = left

// States:
// 0 = forward
// 1 = turning left
// 2 = turning around
// 3 = turning right
// 4 = stop


// --------------------------------------- CLASSES -----------------------------------------------------

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
  };

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

class ColorSensor {
private:
  int _S0, _S1, _S2, _S3, _OUT;

  int _red_min, _red_max;
  int _green_min, _green_max;
  int _blue_min, _blue_max;

  int no_colors = 7;
  int colorsRGB[7][3] = {
    {0,0,0}, // black 0
    {255, 255, 255}, // white 1
    {40, 75, 155}, // blue 2
    {115, 190, 80}, // green 3
    {110, 50, 110}, // purple 4
    {255, 180, 70}, // yellow 5
    {220, 40, 50}, // red 6
  };

public:
  ColorSensor(int s0, int s1, int s2, int s3, int out) {
    _S0 = s0;
    _S1 = s1;
    _S2 = s2;
    _S3 = s3;
    _OUT = out;

    // Initialise pins
    pinMode(_S0, OUTPUT);
    pinMode(_S1, OUTPUT);
    pinMode(_S2, OUTPUT);
    pinMode(_S3, OUTPUT);
    pinMode(_OUT, INPUT);

    // Set frequency-scaling to 20%
    setScaling(20);
  };

  int setCalibration(int red_min, int green_min, int blue_min, int red_max,  int green_max, int blue_max) {
    _red_min = red_min;
    _green_min = green_min;
    _blue_min = blue_min;
    _red_max = red_max;
    _green_max = green_max;
    _blue_max = blue_max;
  }

  int setScaling(int scaling) {
    switch (scaling) {
      case 0:
        digitalWrite(_S0, LOW); // Set scaling to 0% (scaling is turned OFF)
        digitalWrite(_S1, LOW);
        break;

      case 2:
        digitalWrite(_S0, LOW); // Set scaling to 2%
        digitalWrite(_S1, HIGH);
        break;

      case 20: // Set scaling to 20%
        digitalWrite(_S0, HIGH);
        digitalWrite(_S1, LOW);
        // Serial.print("Set to 20%!");
        break;

      case 100: // Set scaling to 100%
        digitalWrite(_S0, HIGH);
        digitalWrite(_S1, HIGH);
        break;

      default: // Set default scaling (default scaling is 20%)
        digitalWrite(_S0, HIGH);
        digitalWrite(_S1, LOW);
        break;
    }
  };

  int readDiode(char color) {
    // Reads red, green, blue, or all diodes
    switch (color) {
      case 'r': // Setting red filtered photodiodes to be read
        digitalWrite(_S2, LOW);
        digitalWrite(_S3, LOW);
        
        break;

      case 'b': // Setting blue filtered photodiodes to be read
        digitalWrite(_S2, LOW);
        digitalWrite(_S3, HIGH);
        break;

      case 'c': // Setting clear photodiodes(no filters on diodes) to be read
        digitalWrite(_S2, HIGH);
        digitalWrite(_S3, LOW);
        break;

      case 'g': // Setting green filtered photodiodes to be read
        digitalWrite(_S2, HIGH);
        digitalWrite(_S3, HIGH);
        break;

      default:
        digitalWrite(_S2, HIGH);
        digitalWrite(_S3, LOW);
        break;
    }

    int duration = pulseIn(_OUT, LOW);

    if (duration != 0) {
      return 1000 / duration; // Reads and returns the frequency of selected color
    }
    else {
      return 0;
    }
  };

  int readClosestColor(bool print = false) {
    // Reads the colour sensor and returns the matching colour code
    // print = false -> don't print anything
    // print = true -> print RGB colours to serial

    int c, r, g, b;

    // Take measurements
    r = readDiode('r');
    g = readDiode('g');
    b = readDiode('b');
    // c = readDiode('c');

    // Serial.print("FREQs: ");
    // Serial.print("   ");
    // Serial.print(r);
    // Serial.print(", ");
    // Serial.print(g);
    // Serial.print(", ");
    // Serial.print(b);
    // Serial.print("   ");
    

    // Convert to RGB
    r = map(r, _red_min, _red_max, 0, 255);
    g = map(g, _green_min, _green_max, 0, 255);
    b = map(b, _blue_min, _blue_max, 0, 255);

    // Serial.print("      RGB: ");
    if (print) {
      Serial.print(r);
      Serial.print(", ");
      Serial.print(g);
      Serial.print(", ");
      Serial.print(b);
      Serial.print("   ");

      Serial.println("");
    }

    // Match to one of the colours
    int index = -1; // return -1 if no declared color matches color sensor is reading
    int biggestDifference = 765;

    for (int i = 0; i < no_colors; i++)
    {
      int difference = sqrt(pow(r - colorsRGB[i][0], 2) + pow(g - colorsRGB[i][1], 2) + pow(b - colorsRGB[i][2], 2));
      if (difference < biggestDifference)
      {
        index = i;
        biggestDifference = difference;
      }
    }

    // Return the color code
    return index;
  };

  void calibration(int n) {
    //
  };
};

// class SonarGroup {
// private:
//   int _trigger, _echo1, _echo2, _echo3;
//   int _max_distance;
// public:
//   SonarGroup(int trigger, int echo1, int echo2, int echo3, int max_distance) {
//     _trigger = trigger;
//     _echo1 = echo1;
//     _echo2 = echo2;
//     _echo3 = echo3;
//     _max_distance = max_distance;

//     pinMode(_trigger, OUTPUT);
//     pinMode(_echo1, INPUT);
//     pinMode(_echo2, INPUT);
//     pinMode(_echo3, INPUT);
//   };

//   int ping_cm() {
//     // Send a ping to the first sensor
//     // left -> center -> right

//     // Stores results L F R
  
//     digitalWrite(_trigger, LOW);
//     delayMicroseconds(2);
//     digitalWrite(_trigger, HIGH);
//     delayMicroseconds(10);
//     digitalWrite(_trigger, LOW);

//     int duration1 = pulseIn(_echo1, HIGH);
//     int duration2 = pulseIn(_echo2, HIGH);
//     int duration3 = pulseIn(_echo3, HIGH);
//     int L = duration1 *0.034/2;
//     int F = duration2 *0.034/2;
//     int R = duration3 *0.034/2;

//     if (L > _max_distance) {
//       L = 0;
//     }
//     if (F > _max_distance) {
//       F = 0;
//     }
//     if (R > _max_distance) {
//       R = 0;
//     }
//   };
// };

class LED {
private:
  int _pin;
public:
  LED(int pin) {
    _pin = pin;
    pinMode(_pin, OUTPUT);
  };

  void blink(int n, int t) {
    // Blink n times, for t ms each time
    for (int i=0; i<n-1; i++) {
      digitalWrite(_pin, HIGH);
      delay(t);
      digitalWrite(_pin, LOW);
      delay(t);
    };
    digitalWrite(_pin, LOW);
  };
};


// --------------------------------------- DEFINE PINS & INITIALISE  -------------------------------------------------
// 

// Color sensor
#define S0 11 // only use in setup
#define S1 12 // only use in setup
#define S2 2
#define S3 3
#define OUT 10 // PWM ~
ColorSensor colorSensor(S0, S1, S2, S3, OUT);

// Motors
#define ENA 5 // PWM ~
#define IN1 A1
#define IN2 A0
#define IN3 A2
#define IN4 A3
#define ENB 6 // PWM ~

#define SCALING_LEFT 1
#define SCALING_RIGHT 1
Motor leftMotor(ENA, IN1, IN2, SCALING_LEFT);
Motor rightMotor(ENB, IN3, IN4, SCALING_RIGHT);

#define LED_PIN A4
LED led(LED_PIN);

// // Bluetooth Serial
// // Only have 1
// #define RX 0
// #define TX 1 // remember the wiring is *FLIPPED*
// #define receiver_MAC "4074e03f39c7" // "00d49e00c02a"
// SoftwareSerial BTSerial(RX, TX);

// Ultrasonic Sensors
// Use the NewPing library
// https://www.arduino.cc/reference/en/libraries/newping/
#define MAX_DISTANCE 50 //in cm
#define SONAR_TOLERANCE 2
#define sonarAngleRange 0.261799 //15deg in radians

#define TRIGGER_LEFT 13
#define TRIGGER_CENTER 7
#define TRIGGER_RIGHT 8
#define ECHO_LEFT A5
#define ECHO_CENTER 9
#define ECHO_RIGHT 4

NewPing leftSonar(TRIGGER_LEFT, ECHO_LEFT, MAX_DISTANCE);
NewPing centerSonar(TRIGGER_CENTER, ECHO_CENTER, MAX_DISTANCE);
NewPing rightSonar(TRIGGER_RIGHT, ECHO_RIGHT, MAX_DISTANCE);

// #define TRIGGER 4
// #define ECHO_LEFT 7
// #define ECHO_CENTER 8
// #define ECHO_RIGHT 9
// SonarGroup sonars(TRIGGER, ECHO_LEFT, ECHO_CENTER, ECHO_RIGHT, MAX_DISTANCE);


// --------------------------------------- VARIABLES -------------------------------------------------
// The maze is a 6x6 grid of cells
// Reference each cell by a specific id
// Bottom left cell = 0
int currentCell = START_CELL;
int targetCell = DESTINATION_CELL;
int currentDirection = START_DIRECTION;
int currentInstruction = START_INSTRUCTION;
int state = START_STATE;

// A 6x6 matrix of cell colors
int cellColor[NO_CELLS];

// A matrix with the adjacent cells for each cell
// where adjacentCell[cell][direction] = adjacentCell or -1 if it doesn't exist
// so we don't actually need to store the walls seperately in matrices, we can
// just represent a wall as a -1 adjacent link. This also works for the outside walls,
// because trying to access an adjacent cell outside the grid will return -1.
int adjacentCell[NO_CELLS][4];

// The current set of instruction it is following
int path[NO_CELLS];
int pathPos = 0;
int pathLength = 0;
int instructions[NO_CELLS];
int instructionsPos = 0;
int instructionsLength = 0;

int straightLength = 0;
int straightPos = 0;

int angleTurned = 0;
int turnAngle = 0;

// Gate info
// gates[cell1] = cell2     and
// gates[cell2] = cell1     if there is a gate between cell1 and cell2
int gateDestinations[NO_CELLS];

// The current sensor readings
int colorCode, colorCode2;
float L, F, R;

void initialiseCells() {
  // Initialise cell colours, gate info, and adjacent cells
  for (int i=0; i<NO_CELLS; i++) {
    cellColor[i] = -1;
    gateDestinations[i] = -1;
    for (int j=0; j<4; j++) {
      adjacentCell[i][j] = getAdjacentCell(i,j);
    };
  };
  cellColor[START_CELL] = START_CELL_COLOR;
  cellColor[DESTINATION_CELL] = DESTINATION_CELL_COLOR;
}

// prevColorCodes[10];
// int prevColorCodesPos = 0;

// prevSonarData[10];
// int prevSonarDataPos = 0;

// --------------------------------------- HELPER FUNCTIONS -------------------------------------------------


bool withinTolerance(float a, float b, float tolerance) {
  // Check if a and b are within tolerance of each other
  return (abs(a-b) < tolerance);
};

int getAdjacentCell(int cell, int direction) {
  // Returns the cell id of the adjacent cell in a certain direction
  // If it doesn't exist, return -1

  switch (direction) {
    case 0: // up
      if (cell >= GRID_SIZE*(GRID_SIZE-1)) {
        return -1;
      };
      return cell + GRID_SIZE;

    case 2: // down
      if (cell < GRID_SIZE) {
        return -1;
      };
      return cell - GRID_SIZE;

    case 1: // right
      if (cell%GRID_SIZE == GRID_SIZE-1) {
        return -1;
      };
      return cell + 1;

    case 3: // left
      if (cell%GRID_SIZE == 0) {
        return -1;
      };
      return cell - 1;
    
    default:
      return -1;
  };
};

void calculatePath() {
  // Calculate the shortest path from the currentCell to the targetCell
  // Use Dijkstra's algorithm
  // https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm

  // Serial.print("Calculating the path from cell ");
  // Serial.print(currentCell);
  // Serial.print(" to cell ");
  // Serial.println(targetCell);

  // Initialise distances as inf and prev as -1
  int dist[NO_CELLS];
  int prev[NO_CELLS];

  for (int i=0; i<NO_CELLS;  i++) {
    dist[i] = 9999;
    prev[i] = -1;
  };

  dist[targetCell] = 0;

  // Initialise queue
  cppQueue	q(sizeof(int), NO_CELLS, IMPLEMENTATION);
  q.push(&targetCell); // push the *memory address* of the start csell

  // Print stuff
  Serial.println("Initial Distances: ");
  Serial.println(dist[0]);
  Serial.println(dist[1]);
  Serial.println(dist[30]);
  

  // Loop through all cells
  while (!q.isEmpty()) {
    // Pop from the queue
    int cell;
    q.pop(&cell);
    // Serial.print("Popped cell ");
    // Serial.println(cell);

    // Check if we've reached the start
    if (cell == currentCell) {
      break;
    };
    
    // Loop through all adjacent cells
    for (int i=0; i<4; i++) {
      int adjCell = adjacentCell[cell][i];
      if (adjCell != -1) {
        // Check if we've already visited this cell
        if (dist[adjCell] == 9999) {
          // We haven't visited this cell yet
          // Add it to the queue
          q.push(&adjCell);
        };

        // Calculate the distance to this cell
        int newDist = dist[cell] + 1; // TODO: change this to the actual distance

        // Check if this is a shorter path
        if (newDist < dist[adjCell]) {
          // Update the distance
          dist[adjCell] = newDist;

          // Update the previous cell
          prev[adjCell] = cell;
        };
      };
    };
  };

  // Print stuff
  Serial.println("Final Distances: ");
  Serial.println(dist[0]);
  Serial.println(dist[1]);
  Serial.println(dist[30]);

  // Now we have the shortest path
  // We can use the prev array to find the path
  pathLength = 0;
  int cell = currentCell;
  while (cell != targetCell) {
    path[pathLength] = cell;
    pathLength++;
    cell = prev[cell];
  };
  path[pathLength] = targetCell;
  pathLength++;
  pathPos = 0;

  Serial.print("New path: ");
  for (int i=0; i<pathLength-1; i++) {
    Serial.print(path[i]);
    Serial.print(", ");
  };
  Serial.print(path[pathLength-1]);
  Serial.println("");

  // And then convert the path to a list of instructions
  instructionsLength = 0;
  cell = currentCell;
  int direction = currentDirection;
  while (cell != targetCell) {
    if (prev[cell] == adjacentCell[cell][direction]) {
      // Keep going forwards;
      instructions[instructionsLength] = 0;
    }
    else if (prev[cell] == adjacentCell[cell][(direction+1)%4]) {
      // Turn right
      instructions[instructionsLength] = 1;
      direction = (direction+1)%4;
    }
    else if (prev[cell] == adjacentCell[cell][(direction+2)%4]) {
      // Turn around
      instructions[instructionsLength] = 2;
      direction = (direction+2)%4;
    }
    else if (prev[cell] == adjacentCell[cell][(direction+3)%4]) {
      // Turn left
      instructions[instructionsLength] = 3;
      direction = (direction+3)%4;
    }
    else {
      Serial.print("huh");
    };

    instructionsLength++;
    cell = prev[cell];
  };
  instructions[instructionsLength] = 4;
  instructionsLength++;
  instructionsPos = 0;

  Serial.print("New instructions: ");
  for (int i=0; i<instructionsLength-1; i++) {
    Serial.print(instructions[i]);
    Serial.print(", ");
  };
  Serial.print(instructions[instructionsLength-1]);
  Serial.println("");
};

void addFrontLeftRightWalls(int cell) {
  if (withinTolerance(F,CELL_SIZE-colorSensorDist-centerSonarDist,SONAR_TOLERANCE) ) {
    // There is a wall in front
    // Store that there's a wall
    adjacentCell[cell][currentDirection] = -1;
    delay(100000);
    Serial.print("Wall in front");
  };

  if (L != 0 || L < CELL_SIZE) {
    // There is a wall to the left
    // Store that there's a wall
    adjacentCell[cell][(currentDirection-1)%4] = -1;
    delay(100);
    Serial.print("Wall to the left");
  };

  if (R != 0 || R < CELL_SIZE) {
    // There is a wall to the right
    // Store that there's a wall
    adjacentCell[cell][(currentDirection+1)%4] = -1;
    delay(100);
    Serial.print("Wall to the right");
  };

}


// --------------------------------------- SOLVING MAZE -------------------------------------------------

void startTurningLeft() {
  angleTurned = 0;
  state = 3;
  leftMotor.forward(0);
  rightMotor.stop();
}

void startTurningAround() {
  angleTurned = 0;
  state = 2;
  leftMotor.forward(0);
  rightMotor.backward(0);

}

void startTurningRight() {
  angleTurned = 0;
  state = 1;
  leftMotor.forward(0);
  rightMotor.stop();
}

void atCellBoundary() {
  // Code to run every time we cross a cell boundary
  int newCell = getAdjacentCell(currentCell, currentDirection);

  Serial.print("Crossed a boundary, now in cell ");
  Serial.println(newCell);

  if (newCell == -1) {
    Serial.print('FUCK!');
  };
  

  // New cell colour
  if (cellColor[newCell] == -1) {
    // We don't know what colour the cell should be, so store in the matrix
    cellColor[newCell] = colorCode;

    // Check if we just discovered a gate
    for (int i=4; i++; i<4) {
      int adjCell = adjacentCell[newCell][i];
      if (cellColor[adjCell] == colorCode) {
        // Add the gate info
        gateDestinations[newCell] = adjCell;
        gateDestinations[adjCell] = newCell;
      };
    };
  }

  // Reached destination cell
  if (colorCode == DESTINATION_CELL_COLOR) {
    led.blink(1,0.3);
    // We've reached the destination
    // Stop
    state = 0;
    leftMotor.stop();
    rightMotor.stop();
  }
  else if (cellColor[newCell] != colorCode) {
    // We're reading a different color to what we should
    // Maybe overwrite??
  };

  // Look at ultrasonic readings while we're at the edge
  addFrontLeftRightWalls(newCell);
  
  // Open a gate if we need to
  if (path[pathPos+1] == gateDestinations[currentCell]) {
    // Stop
    state = 4;

    // Send a bluetooth signal with the color code.
    // BTSerial.print(colorCode); 
    // remember to convert to char

    // Just blink for 3 secs
    led.blink(1,3);

  };

  // Increment the position in the path
  pathPos++;

  // Reached the end of the current path, meaning we are at the targetCell
  if (pathPos == pathLength-1) {
    Serial.print("Reached the current target...");
  }

  // Read an instruction
  int instruction = instructions[instructionsPos];
  switch (instruction) {
    case 0: // forward
      state = 0;
      leftMotor.forward(0);
      rightMotor.forward(0);
      break;

    case 1: // turn right
      startTurningRight();
      turnAngle = PI/2;
      break;

    case 2: // turn around 
      startTurningAround();
      turnAngle = PI;
      break;

    case 3: // left
      startTurningLeft();
      turnAngle = PI/2;
      break;

    case 4: // stop
      state = 0;
      leftMotor.stop();
      rightMotor.stop();
      break;
  }

  instructionsPos++;

  // Update the current cell
  currentCell = newCell;
};

void mazeSetup() {
  Serial.println("Starting maze setup...");

  Serial.println("Initialising cells...");
  initialiseCells();

  Serial.println("Adding walls...");
  addFrontLeftRightWalls(currentCell);

  // print adjacent nodes
  for (int i=0; i<NO_CELLS; i++) {
    Serial.print("Cell ");
    Serial.print(i);
    Serial.print(": ");
    for (int j=0; j<4; j++) {
      Serial.print(adjacentCell[i][j]);
      Serial.print(", ");
    };
    Serial.println("");
  };

  // Calculate the initial path
  // Serial.println("Calculating path...");
  // calculatePath();

  int ilist[13] = {0,0,0,3,0,0,3,0,0,3,0,1,0};
  int instructionsPos = 0;
  int instructionsLength = 13;

  for (int i=0; i<13; i++) {
    instructions[i] = ilist[i];
  }
  int currentInstruction = instructions[0];

}

void mazeLoop() {
  // The main code to run
  Serial.print("Cell: ");
  Serial.println(currentCell);

  Serial.print("Direction: ");
  Serial.println(currentDirection);

  Serial.print("Instruction: ");
  Serial.println(currentInstruction);

  Serial.print("Current state: ");
  Serial.println(state);
  delay(50);

  // Read color sensor
  colorCode = colorSensor.readClosestColor();
  Serial.print("Color:  ");
  Serial.println(colorCode);
  delay(50);

  // Read ultrasonic sensors
  // sonars.ping_cm();
  L = leftSonar.ping_cm();
  delay(50);
  F = centerSonar.ping_cm();
  delay(50);
  R = rightSonar.ping_cm();

  Serial.print(L);
  Serial.print("   ");
  Serial.print(F);
  Serial.print("   ");
  Serial.println(R);

  // Emergengy Stop
  if (F!=0 || F<8) {
    state = 4;
  };

  // Handle changes in the color (moving into a new cell)
  if (colorCode != cellColor[currentCell]) {
    // Take another color reading to verify the change
    // Hopefully will help reduce outliers/false positives?
    colorCode2 = colorSensor.readClosestColor();
    if (colorCode == colorCode2) {
      atCellBoundary();
    };
  };

  // Correct for small errors in the direction using the ultrasonic sensors
  int xL = L*sin(leftSonarAngle - sonarAngleRange/2);
  int xR = R*sin(rightSonarAngle - sonarAngleRange/2);

  Serial.print("xL: ");
  Serial.print(xL);
  Serial.print("   xR: ");
  Serial.println(xR);

  // Send motor directions
  switch (state) {
    case 0: // Going forward
      leftMotor.forward(0);
      rightMotor.forward(0);

      // Correcting
      if (L - R > 2) {
        // Turn slightly left
        Serial.print("Correcting left");
        leftMotor.forward(0);
        rightMotor.forward(1);
      }
      else if (R - L > 2) {
        // Turn slightly right
        Serial.print("Correcting right");
        leftMotor.forward(1);
        rightMotor.forward(0);
      }
      break;
  
    case 1: // turning right
      leftMotor.stop();
      rightMotor.forward(0);
      delay(100);
      break;

    case 2: // turning around
      leftMotor.forward(2);
      rightMotor.forward(2);
      delay(400);
      break;
    
    case 3: // turning left
      leftMotor.forward(0);
      rightMotor.stop();
      delay(100);
      break;
    
    case 4: // Braking
      leftMotor.stop();
      rightMotor.stop();
      break;
  }; 

  Serial.print("");
}


// --------------------------------------- CALIBRATION/TESTING -------------------------------------------------

void calibrateColorSensor() {
  // Calibrate the color sensor by printing RGB values to show with the python script
  // Then fiddle with the values above
  colorSensor.readClosestColor(true);
}

void calibrateUltrasonicSensors() {
  // Point the car directly facing a straight wall
  // Assuming the car is perpendicular to the wall, calculate 
  // the angle between the ultrasonic sensors.

  // Then fiddle with the values above
  // Read ultrasonic sensors
  L = leftSonar.ping_cm();
  delay(10);
  F = centerSonar.ping_cm();
  delay(10);
  R = rightSonar.ping_cm();
  delay(10);
  
  Serial.print(L);
  Serial.print("   ");
  Serial.print(F);
  Serial.print("   ");
  Serial.println(R);

  if (L != 0) {
    float tL = asin((F+centerSonarDist)/L);
    Serial.print("Left Angle:   ");
    Serial.println(tL*180/PI);
  };

  if (R != 0) {
    float tR = asin((F+centerSonarDist)/R);
    Serial.print("Right Angle: ");
    Serial.println(tR*180/PI);
  }
  Serial.println("");
  
  int xL = L*sin(leftSonarAngle - sonarAngleRange/2);
  int xR = R*sin(rightSonarAngle - sonarAngleRange/2);

  Serial.print("xL: ");
  Serial.print(xL);
  Serial.print("   xR: ");
  Serial.println(xR);

  delay(1000);

}

void testCellBoundaries() {
  // Drive the car in a straight line and stop before hitting a wall
  // Blink an LED every time it crosses a cell boundary
  int colorCode = colorSensor.readClosestColor(true);
  
  leftMotor.forward(0);
  rightMotor.forward(0);

  F = centerSonar.ping_cm();
  // sonars.ping_cm();


  if (F != 0) {
    // Stop
    leftMotor.stop();
    rightMotor.stop();
  }

  delay(100);

  // Handle changes in the color (moving into a new cell)
  if (colorCode != cellColor[currentCell]) {
    // Take another color reading to verify the change
    // Hopefully will help reduce outliers/false positives?
    colorCode2 = colorSensor.readClosestColor();
    if (colorCode == colorCode2) {
      atCellBoundary();
      led.blink(1,1);
    };
  };

}

void testing() {
  leftMotor.forward(180);
  rightMotor.forward(180);
  delay(100);
}

// --------------------------------------- RUN -------------------------------------------------

int mode = 1;

void loop() {
  switch (mode) {
    case 0:
      mazeLoop();
      break;
    
    case 1:
      calibrateColorSensor();
      break;
    
    case 2:
      calibrateUltrasonicSensors();
      break;
    
    case 3:
      testCellBoundaries();
      break;

    case 4:
      testing();
      break;
    
    default:
      break;
  }
}

void setup() {
  // Setup serial
  Serial.begin(BAUDRATE);
  
  // Setup bluetooth serial
  // BTSerial.begin(BAUDRATE);
  // BTSerial.print("AT+ROLE1"); // Set HM-10 as master
  // BTSerial.print("AT+CON4074E03F39C7"); // Connect to the receiver
  
  // Set color sensor calibration
  // colorSensor.setCalibration(2,2,2,45,33,33); // my own one
  // colorSensor.setCalibration(2,2,2,48,43,53); // uni
  // Actual one
  colorSensor.setCalibration(2,2,2,48,43,53);

  switch (mode) {
    case 0: // 
      delay(2000);
      mazeSetup();
      break;

    default:
      break;
  }
}