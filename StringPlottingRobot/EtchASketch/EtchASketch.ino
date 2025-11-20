#pragma region README
/* 
This etch-a-sketch hack requires a repin on the motor cable to move pin 3 (gray wire) to pin 6 due to timer conflicts with the IRremote library. You will also need to add the IR reciver from turret whose output goes into pin 7.

The arrow keys will move the robot, and the "OK" button toggles the pen position.
*/
#pragma endregion README
#pragma region LICENSE

/*
  ************************************************************************************
  * MIT License
  *
  * Copyright (c) 2024 Crunchlabs LLC (DEALR Code)

  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to deal
  * in the Software without restriction, including without limitation the rights
  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  * copies of the Software, and to permit persons to whom the Software is furnished
  * to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in all
  * copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
  * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
  * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
  * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  *
  ************************************************************************************
*/

#pragma endregion LICENSE

//////////////////////////////////////////////////
//  LIBRARIES AND CONFIGURATION //
//////////////////////////////////////////////////

#pragma region LIBRARIES
#include "config.h"
#include <AS5600.h>             // encoder
#include <Adafruit_Sensor.h>    // I2C Multiplexer
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <ezButton.h>
#include <IRremote.h>
#pragma endregion LIBRARIES

#pragma region CONFIGURATION
#define move_left 0x8
#define move_right 0x5A
#define move_up 0x18
#define move_down 0x52
#define move_ok 0x1C

#define TCAADDR 0x70
#define SERVOPIN 9
#define SD_CS_PIN 14            // Chip select pin for the SD card module
#define PAUSEPIN 15
#define CURSORPIN 16
#define L_M_DIR 2
#define L_M_PWM 6
#define R_M_DIR 4
#define R_M_PWM 5
#define IR_PIN 7

// Image Size presets ---------------------------------------------------------
#define MED_WIDTH 750
#define MED_INIT 500
#define MED_Y_OFFSET 350

#pragma endregion CONFIGURATION

//////////////////////////////////////////////////
//  CLASS OBJECTS //
//////////////////////////////////////////////////
#pragma region OBJECTS
ezButton redButton(CURSORPIN, 20);
ezButton greenButton(PAUSEPIN, 20);

Servo myPen;               // pen Servo

AS5600 encL;               //  encoder left
AS5600 encR;               //  right
#pragma endregion OBJECTS

//////////////////////////////////////////////////
// STRUCT VARIABLES //
//////////////////////////////////////////////////
#pragma region STRUCTS
//artboard & image params-------------------------------------
struct image{
  //Canvass height is effectively unlimited
  uint16_t canvassWidth;     // Distance suction cups are apart

  float centerXOffset;       // Calculated offset to center bmp in canvass
  float centerYOffset;

  float gameXOffset;       
  float gameYOffset;       // Positive Y moves image down. Units in mm
};

//movement params-------------------------------------------
struct motor{
  float maxSpeed;           // 255 is 100% duty cycle
  float drawSpeed;

  float setSpeeds[2];       
};

//hardware specs--------------------------------------------
struct encoder{
  float rollerDiam;         // diameter of encoder 12.5mm spool + 0.5mm string

  float encLInit;
  float encRInit;           // inital Positions of encoders

  bool posLReached;         // left. Is string at its target length?
  bool posRReached;         // right

  long lastUpdateTime;      // tracks encoder polling
};

//Pen Placement Variables-----------------------------------
struct pen{
  uint8_t penUp;              // servo position presets
  uint8_t penDown;
  uint8_t penStart;

  float penPos;             // current servo position
  float penWindow;          // How often pen behavior is updated
  float penMoveFlag;
  
  long nextPenUpdate;       // which millis pen behavior will be updated again
};

//UI variables-----------------------------------------------
struct ui{
  bool selectMode;         // Conditional for UI while loop, no file chosen
  uint8_t drawMode;         // Conditional for pause while loop

  bool redWasPressed;        // Button debounce variables for Scroll button... 
  bool greenWasPressed;        // ... and Pause Button

  uint8_t xCoord;
  uint8_t yCoord;
};

//Robot global variables-----------------------------------
struct bot{
  float currentCoords[2];           // current XY pos. {X, Y}
  float currentLengths[2];          // current String Lengths (mm) {Left, Right}

  float nextCoords[2];              // next target coords {X, Y}
  float nextLengths[2];             // next target string length {Left, Right}

  float savedCoords[2];             // {X, Y} any coords needed in future
  float initStringLengths[2];       // {Left, Right} robot's startup position

  bool posUpdated;
  bool penDown;

  long penTimeout;
};

////////////////////////////// INIT STRUCTS /////////////////////////////////
//canvass width, image center height, init size setting, bmp width, bmp height, fileDataRowSize, fileDataOffset, Image Cenetring Offset X, Image Cenetring Offset Y.
image Image = {MED_WIDTH * 0.8, 0, 0, 0, 0};
//motor max duty cycle, min, right speed, left speed
motor Motor = {motorTravelSpeed, drawSpeed, {0}};
//encoder diameter, zero1, zero0, reachedTarget1, reachedTarget2, lastUpdateTime
encoder Encoder = {12.5, 0.0, 0.0, false, false, 0};
//pen Up servo pos, pen down pos, init pos, current pos, pen update period, pen move time, next scheduled update time
pen Pen = {penUpPos, penDownPos, penStartPos, 0, 0};
//menu flag (starts true), is paused (starts false), green button pressed, red button pressed, selected file number (list index)
ui UI = {true, 0, false, false, 0, 0};
//current XY, current string lengths, next XY, next string lengths, any saved  coordinates, inital string lengths.
bot Plotter = {{0}, {0}, {0}, {0}, {0}, {0}, false, false, 0};
#pragma endregion STRUCTS

//////////////////////////////////////////////////
// SETUP FUNCTIONS //
//////////////////////////////////////////////////
#pragma region SETUP 
//////////////////I2C Multiplexer Port Selector//////////////////////
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

//////////////////////////////SETUP/////////////////////////////////
void setup() { 
  pinMode(L_M_DIR, OUTPUT);
  pinMode(L_M_PWM, OUTPUT);
  pinMode(R_M_DIR, OUTPUT);
  pinMode(R_M_PWM, OUTPUT);
  pinMode(SERVOPIN, OUTPUT);
  pinMode(PAUSEPIN, INPUT_PULLUP);
  pinMode(CURSORPIN, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.println("PLTTR Hack Etch A Sketch ..... 7/10/2025");

  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);

  //pixels.begin();

  redButton.setDebounceTime(25);
  greenButton.setDebounceTime(25);

  myPen.attach(SERVOPIN);                   // pen Servo on pin 9
  myPen.write(Pen.penStart);

  delay(800);
  
  // Choose starting position
  Image.centerXOffset = (MED_WIDTH - boardSize) / 2;
  Image.centerYOffset = 100;

  Plotter.initStringLengths[0] = MED_INIT;
  Plotter.initStringLengths[1] = MED_INIT;
  // StartUp sequence, file select from UI ---------------------------------
  myPen.attach(SERVOPIN);
  myPen.write(Pen.penUp);

  // Init Left Encoder ---------------------------------------------------------
  tcaselect(0);
  encL.begin();
  delay(50);
  encL.setDirection(AS5600_CLOCK_WISE);
  Encoder.encLInit = encL.resetCumulativePosition();

  // Init Right Encoder --------------------------------------------------------
  tcaselect(1);
  encR.begin();  
  delay(50);                          
  encR.setDirection(AS5600_CLOCK_WISE);            // default, just be explicit.
  Encoder.encRInit = encR.resetCumulativePosition(); // zero encoder at start
  
  // Generate first coordiate to move to
  float nextXY[2] = {MED_WIDTH / 2.0, MED_Y_OFFSET + 150}; 
  float nextL[2];

  UI.drawMode = 1;                      // draw game board

  Plotter.nextCoords[0] = nextXY[0];    // X
  Plotter.nextCoords[1] = nextXY[1];    // Y

  XYToLengths(Plotter.nextCoords[0], Plotter.nextCoords[1], nextL); 
  Plotter.nextLengths[0] = nextL[0];    // left
  Plotter.nextLengths[1] = nextL[1];    // right
}
#pragma endregion SETUP 

//////////////////////////////////////////////////
// MAIN LOOP //
//////////////////////////////////////////////////
#pragma region LOOP
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> LOOP <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void loop(){ 
  // Check for user inputs
  if(!Plotter.posUpdated){
    receiveIR();
  } else {
    moveBot();
  }
}
#pragma endregion LOOP 


//////////////////////////////////////////////////
// RECIEVE IR COMMANDS //
//////////////////////////////////////////////////
#pragma region GET_COMMANDS
void receiveIR(){
  if (IrReceiver.decode()) {
    //Print a short summary of received data
    IrReceiver.printIRResultShort(&Serial);
    IrReceiver.printIRSendUsage(&Serial);
    if (IrReceiver.decodedIRData.protocol == UNKNOWN) { //command garbled or not recognized
        Serial.println(F("Received noise or an unknown (or not yet enabled) protocol - if you wish to add this command, define it at the top of the file with the hex code printed below (ex: 0x8)"));
        // We have an unknown protocol here, print more info
        IrReceiver.printIRResultRawFormatted(&Serial, true);
    }
    Serial.println();
    IrReceiver.resume(); // Enable receiving of the next value

    float nextL[2];   // Create variable to store lengths

    switch(IrReceiver.decodedIRData.command){
      case move_left:
        Plotter.nextCoords[0] -= 15;
        Plotter.nextCoords[0] = max(Plotter.nextCoords[0], 0);  // constrain

        XYToLengths(Plotter.nextCoords[0], Plotter.nextCoords[1], nextL);
        Plotter.nextLengths[0] = nextL[0];
        Plotter.nextLengths[1] = nextL[1];

        Serial.println(Plotter.nextCoords[0]);
        Serial.println(Plotter.nextCoords[1]);
        Plotter.posUpdated = true;         // flag for bot movement
        IrReceiver.stop();                 // stop recieving until target is hit
        myPen.attach(SERVOPIN);
        break;
      case move_right:
        Plotter.nextCoords[0] += 15;
        Plotter.nextCoords[0] = min(Plotter.nextCoords[0], Image.canvassWidth);

        XYToLengths(Plotter.nextCoords[0], Plotter.nextCoords[1], nextL);
        Plotter.nextLengths[0] = nextL[0];
        Plotter.nextLengths[1] = nextL[1];

        Serial.println(Plotter.nextCoords[0]);
        Serial.println(Plotter.nextCoords[1]);
        Plotter.posUpdated = true;
        IrReceiver.stop();
        myPen.attach(SERVOPIN);
        break;
      case move_up:
        Plotter.nextCoords[1] -= 15;
        Plotter.nextCoords[1] = max(Plotter.nextCoords[1], 100);

        XYToLengths(Plotter.nextCoords[0], Plotter.nextCoords[1], nextL);
        Plotter.nextLengths[0] = nextL[0];
        Plotter.nextLengths[1] = nextL[1];

        Serial.println(Plotter.nextCoords[0]);
        Serial.println(Plotter.nextCoords[1]);
        
        Plotter.posUpdated = true;
        IrReceiver.stop();
        myPen.attach(SERVOPIN);
        break;
      case move_down:
        Plotter.nextCoords[1] += 15;
        Plotter.nextCoords[1] = min(Plotter.nextCoords[1], 600);

        XYToLengths(Plotter.nextCoords[0], Plotter.nextCoords[1], nextL);
        Plotter.nextLengths[0] = nextL[0];
        Plotter.nextLengths[1] = nextL[1];

        Serial.println(Plotter.nextCoords[0]);
        Serial.println(Plotter.nextCoords[1]);
        Plotter.posUpdated = true;
        IrReceiver.stop();
        myPen.attach(SERVOPIN);
        break;
      case move_ok:
        if(millis() > Plotter.penTimeout + 1000){
          Plotter.penDown = !Plotter.penDown;
          Plotter.penTimeout = millis();
        }
        myPen.attach(SERVOPIN);
        Plotter.posUpdated = true;
        IrReceiver.stop();
        break;
      default:
        break;
    }
  }
}
#pragma endregion GET_COMMANDS

//////////////////////////////////////////////////
// MOTOR RELATED FUNCTIONS //
//////////////////////////////////////////////////
#pragma region MOVE_BOT
//////////////////////////// Plotter Run Functions ////////////////////////////

// Moves motor towards next coordinate, returns true if either motor moved. myPen.wrCode continually tries to move towards the exact target position every iteration, boolean "target reached" flags will update if error is less than epsilon set by user. 
bool moveBot(){
  int nextSpeeds[2];                              // define motor speeds {L, R}
  float currentXY[2];                             // current position, local var

  bool penFree = false;                           // enable pen movement

  updateStringLength();                           // updates currentLengths[]

  lengthsToXY(Plotter.currentLengths[0], Plotter.currentLengths[1], currentXY);
  Plotter.currentCoords[0] = currentXY[0];        // updates currentCoords
  Plotter.currentCoords[1] = currentXY[1];

  //Helpful print statements --------------------------------------------------
  //Serial.print("actual x pos: ");
  //Serial.println(Plotter.currentCoords[0]);
  //Serial.print("actual y pos: ");
  //Serial.println(Plotter.currentCoords[1]);

  //Serial.print("left length to-go: ");
  //Serial.println(Plotter.nextLengths[0] - Plotter.currentLengths[0]);
  //Serial.print("right length to-go: ");
  //Serial.println(Plotter.nextLengths[1] - Plotter.currentLengths[1]);
  
  // move Pen based on IR
  movePen(Plotter.penDown);

  // Get motor speeds
  getSpeeds(Plotter.nextLengths[0], Plotter.nextLengths[1], nextSpeeds);
  
  // Drive left motor until within epsilon, then set "position reached" flag to true. 
  if (abs(Plotter.currentLengths[0] - Plotter.nextLengths[0]) < epsilon){
    driveMotors(0, true);   // left
    Encoder.posLReached = true;
  } else {
    driveMotors(nextSpeeds[0], true);   // left
    Encoder.posLReached = false;
  }

  // Drive right motor.
  if (abs(Plotter.currentLengths[1] - Plotter.nextLengths[1]) < epsilon){
    driveMotors(0, false);   // right
    Encoder.posRReached = true;
  } else {
    driveMotors(nextSpeeds[1], false);   // right
    Encoder.posRReached = false;
  }

  // Returns true if either motor needs to move. Returns false when position is reached within the defined error.
  if (Encoder.posLReached && Encoder.posRReached){
    Plotter.posUpdated = false;
    IrReceiver.begin(IR_PIN);
    myPen.detach();
    return false;
  } else {
    return true;
  }
}

 // Call function with true to move left motor, false for right motor.
void driveMotors(int speed, bool left){  
  if(left){
    if(speed < 0){
      digitalWrite(L_M_DIR, HIGH);
    } else {
      digitalWrite(L_M_DIR, LOW);
    }
    analogWrite(L_M_PWM, abs(speed));
    Motor.setSpeeds[0] = speed;
  } else {
    if(speed < 0){
      digitalWrite(R_M_DIR, HIGH);
    } else {
      digitalWrite(R_M_DIR, LOW);
    }
    analogWrite(R_M_PWM, abs(speed));
    Motor.setSpeeds[1] = speed;
  }
}


void movePen(bool down){
  if(down ){
    myPen.write(Pen.penDown);
  } else{
    myPen.write(Pen.penUp);
  }
}

// Gives speeds to move each motor towards it's target position. Once within  Linearly scales speed relative to distance-to-go with a lower bound at the minimum duty cycle required to lift the robot.
void getSpeeds(float toLengthLeft, float toLengthRight, int* nextSpeeds){
  // First, store distance remaining in these variables for future calculations
  float speedL = (toLengthLeft - Plotter.currentLengths[0]);   // left
  float speedR = (toLengthRight - Plotter.currentLengths[1]);  // right
  
  speedL = constrain(speedL, -255, 255);
  speedR = constrain(speedR, -255, 255);

  if(speedL < 0){
      speedL = -255 * max(pow(-speedL / 255, 1.0 / 3.0), mScale);
  } else {
      speedL = 255 * max(pow(speedL / 255, 1.0 / 3.0), mScale);
  }
  if(speedR < 0){
    speedR = -255 * max(pow(-speedR / 255, 1.0 / 3.0), mScale);
  } else{
    speedR = 255 * max(pow(speedR / 255, 1.0 / 3.0), mScale); 
  }

  // flipping motors
  if(flipLeftMotor){
    speedL = - speedL;
  }
  if(flipRightMotor){
    speedR = - speedR;
  }

  nextSpeeds[0] = speedL;
  nextSpeeds[1] = speedR;
}
#pragma endregion MOVE_BOT

#pragma region GET_DATA

//////////////////////////// Encoder Functions ///////////////////////
// Updates currentLengths global variable based on encoder positions.
void updateStringLength(){  
  //  update every 10 ms
  if (millis() - Encoder.lastUpdateTime >= sampleTime){  
    Encoder.lastUpdateTime = millis();

    //Left Encoder
    tcaselect(0);
    int32_t thetaL = encL.getCumulativePosition();
    Plotter.currentLengths[0] = lengthFromCounts(thetaL, 0);

    //Right Encoder
    tcaselect(1);
    int32_t thetaR = encR.getCumulativePosition();
    Plotter.currentLengths[1] = lengthFromCounts(thetaR, 1);
  }
}

// returns lengths (in mm) from encoder movments and inital zero.               
// encNum = 0 - left; 1 - right
float lengthFromCounts(int32_t pos, uint8_t encNum){
  // Left
  if (encNum == 0) {
    if(flipLeftEncoder){
      return - pos / 4096.0 * Encoder.rollerDiam * PI 
            + Plotter.initStringLengths[0];
    } else{
      return pos / 4096.0 * Encoder.rollerDiam * PI 
            + Plotter.initStringLengths[0];
    }
  }
  // Right
  else if (encNum == 1){
    if(flipRightEncoder){
      return - pos / 4096.0 * Encoder.rollerDiam * PI 
            + Plotter.initStringLengths[1];
    } else {
      return  pos / 4096.0 * Encoder.rollerDiam * PI 
            + Plotter.initStringLengths[1];
    }
  }
  else {
    return -1;  // error
  }
}
#pragma endregion GET_DATA

//////////////////////////////////////////////////
// INVERSE KINEMATICS //
//////////////////////////////////////////////////
#pragma region COORDINATE_MAPPING
// The heart of the program lies in these few lines of code. The math below converts string lengths to cartesian coordinates, allowing us to position the robot where we want. 
void lengthsToXY(float lengthLeft, float lengthRight, float* XY){
  float s = (lengthLeft + lengthRight + MED_WIDTH) / 2.0;
  float area = sqrt(s * (s - lengthLeft) * (s - lengthRight)
               * (s - MED_WIDTH));

  XY[1] = 2.0 * area / MED_WIDTH;                             // Y - coord
  XY[0] = sqrt(pow(lengthLeft, 2.0) - pow(XY[1], 2.0));       // X - coord
}

// converts cartesian coords to string lengths.
void XYToLengths(float x, float y, float* L){
  L[0] = sqrt(pow(x, 2.0) + pow(y, 2.0));                    //left
  L[1] = sqrt(pow((MED_WIDTH - x), 2.0) + pow(y, 2.0));      //right
}
#pragma endregion COORDINATE_MAPPING
