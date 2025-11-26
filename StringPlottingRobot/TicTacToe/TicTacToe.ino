#pragma region README
/* 
Tic Tac Toe Played on the plotter robot. Press green button to start the game and draw a game board. Red Numbers represent X coordinates, blue numbers are Y coordinates. Press the green button to confrim your selection. 

game will automatically switch between drawing "X" and "O".

Updated 7/29/2025
*/
#pragma endregion README
#pragma region LICENSE
/*
  ************************************************************************************
  * MIT License
  *
  * Copyright (c) 2025 Crunchlabs LLC (Balance Bot Code)

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
#pragma region LIBRARIES_AND_CONFIGURATION
#include "config.h"
#include <AS5600.h>             // encoder
#include <Adafruit_Sensor.h>    // I2C Multiplexer
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <ezButton.h>

#define TCAADDR 0x70
#define SERVOPIN 9
#define LEDPIN 10
#define SD_CS_PIN 14            // Chip select pin for the SD card module
#define PAUSEPIN 15
#define CURSORPIN 16
#define L_M_DIR 2
#define L_M_PWM 3
#define R_M_DIR 4
#define R_M_PWM 5

// Image Size presets ---------------------------------------------------------
#define SMALL_WIDTH 500
#define SMALL_INIT 260
#define SMALL_Y_OFFSET 300

#define MED_WIDTH 750
#define MED_INIT 500
#define MED_Y_OFFSET 350

#define LARGE_WIDTH 1250
#define LARGE_INIT 720
#define LARGE_Y_OFFSET 425

#pragma endregion LIBRARIES_AND_CONFIGURATION


//////////////////////////////////////////////////
//  CLASS OBJECTS //
//////////////////////////////////////////////////
#pragma region CLASS_OBJECTS
ezButton redButton(CURSORPIN, 20);
ezButton greenButton(PAUSEPIN, 20);

Adafruit_NeoPixel pixels (10, LEDPIN, NEO_GRB + NEO_KHZ800);

Servo myPen;               // pen Servo

AS5600 encL;               //  left
AS5600 encR;               //  right
#pragma region CLASS_OBJECTS

//////////////////////////////////////////////////
// STRUCTS //
//////////////////////////////////////////////////
#pragma region STRUCTS
struct game{
  bool isXTurn;
  bool isRunUI;
  
  int8_t gameBoard[3][3];
};

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

  long lastCallTime;        // last millis pen behavior was updated
  long penDownTime;         // until which millis pen should be drawing
  long nextPenUpdate;       // which millis pen behavior will be updated again
};


//parametric "time" variables for stepping thru space filling patterns. paramState = overall "time", maxParamState stops the pattern, and paramStep is the timestep width
struct parametric{
  double paramState;        // paramteric space filling variables...        
  double maxParamState;  
  double paramStep;

  float dx;                 // current tangenet vector...
  float dy;       
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

  bool newBlankPix;
};

////////////////////////////// INIT STRUCTS ////////////////////////////////////
game Game = {true, false, {{0}}};
//canvass width, image center height, init size setting, bmp width, bmp height, fileDataRowSize, fileDataOffset, Image Cenetring Offset X, Image Cenetring Offset Y.
image Image = {MED_WIDTH, 75, 100, 0, 0};
//motor max duty cycle, min, right speed, left speed
motor Motor = {motorTravelSpeed, drawSpeed, {0}};
//encoder diameter, zero1, zero0, reachedTarget1, reachedTarget2, lastUpdateTime
encoder Encoder = {12.5, 0.0, 0.0, false, false, 0};
//parametric start time, end time, timestep size, current step dx, dy.
parametric Pos = {0, 0, timeStep, 0, 0};
//pen Up servo pos, pen down pos, init pos, current pos, pen update period, pen move time, next scheduled update time
pen Pen = {penUpPos, penDownPos, penStartPos, 0, penRes, dotTime, 0};
//menu flag (starts true), is paused (starts false), green button pressed, red button pressed, selected file number (list index)
ui UI = {true, 0, false, false, 0, 0};
//current XY, current string lengths, next XY, next string lengths, any saved  coordinates, inital string lengths.
bot Plotter = {{0}, {0}, {0}, {0}, {0}, {0}, false};
#pragma endregion STRUCTS


//////////////////////////////////////////////////
//  SETUP FUNCTIONS //
//////////////////////////////////////////////////
#pragma region SETUP
// ////////////////////// I2C Multiplexer Port Selector ///////////////////////
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

// /////////////////////////////////// UI //////////////////////////////////////
// Enters this loop first. Prompts users to the use buttons to select a file / edit settings before running the plotter.
void runUI(){
  myPen.detach();
  while(UI.selectMode){
    greenButton.loop();

    // TODO: Add lights
    
    // check for choose/pause (green) button press and release
    if(greenButton.isPressed()){
      UI.greenWasPressed = true;
    }
    if(greenButton.isReleased() && UI.greenWasPressed){
      UI.selectMode = false;
      Serial.println("tapped");
    }
    delay(50);
  }
}


//////////////////////////////////////////////////
//  SETUP FUNCTIONS //
//////////////////////////////////////////////////
// ///////////////////////////////// SETUP /////////////////////////////////////
void setup() { 
  pinMode(L_M_DIR, OUTPUT);
  pinMode(L_M_PWM, OUTPUT);
  pinMode(R_M_DIR, OUTPUT);
  pinMode(R_M_PWM, OUTPUT);
  pinMode(SERVOPIN, OUTPUT);
  pinMode(PAUSEPIN, INPUT_PULLUP);
  pinMode(CURSORPIN, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.println("PLTTR Hack TicTacToe ..... 7/10/2025");

  pixels.begin();

  redButton.setDebounceTime(25);
  greenButton.setDebounceTime(25);

  myPen.attach(SERVOPIN);                   // pen Servo on pin 9
  myPen.write(Pen.penStart);

  delay(800);
  
  pixels.setBrightness(255);

  Image.centerXOffset = (MED_WIDTH - boardSize) / 2;
  Image.centerYOffset = 200;

  Plotter.initStringLengths[0] = MED_INIT;
  Plotter.initStringLengths[1] = MED_INIT;
  // StartUp sequence, file select from UI ---------------------------------
  runUI();
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
  float nextXY[2] = {Image.centerXOffset, Image.centerYOffset}; 
  float nextL[2];

  UI.drawMode = 1;                      // draw game board
  Pos.paramState = 0;                   // setup parametric draw
  Pos.maxParamState = boardSize * 4;    // setup param state

  Plotter.nextCoords[0] = nextXY[0];    // X
  Plotter.nextCoords[1] = nextXY[1];    // Y

  XYToLengths(Plotter.nextCoords[0], Plotter.nextCoords[1], nextL); 
  Plotter.nextLengths[0] = nextL[0];    // left
  Plotter.nextLengths[1] = nextL[1];    // right
}
#pragma endregion SETUP


//////////////////////////////////////////////////
//  MAIN LOOP //
//////////////////////////////////////////////////
#pragma region LOOP
// /////////////////////////////////> LOOP /////////////////////////////////<
void loop(){ 
  // Check for user inputs
  pollButtons();
  drawBoard(UI.drawMode);
}
#pragma endregion LOOP


//////////////////////////////////////////////////
//  RUN GAME UI //
//////////////////////////////////////////////////
#pragma region UI
//////////////////////////////// UI Functions ////////////////////////////////
// looks for user inputs for pause or restart
void pollButtons(){
  bool selectXCoord = true;

  while(UI.drawMode == 0){
    driveMotors(0, false);
    driveMotors(0, true);

    redButton.loop();                           // MUST call the loop() first
    greenButton.loop();

    if(redButton.isPressed()){
      UI.redWasPressed = true;
    }
    if(redButton.isReleased() && UI.redWasPressed){
      if(selectXCoord){
        UI.xCoord = (UI.xCoord + 1) % 3;
      } else {
        UI.yCoord = (UI.yCoord + 1) % 3;
      }
    }

    if(greenButton.isPressed()){
      UI.greenWasPressed = true;
    }
    if(greenButton.isReleased() && UI.greenWasPressed){
      if(selectXCoord){
        selectXCoord = false;
      } else {
        if(Game.gameBoard[UI.xCoord][UI.yCoord] != 0){
          pixels.clear();
          pixels.fill(pixels.Color(255, 0, 0), 2, pixels.numPixels());
          pixels.show();
          delay(200);
          pixels.clear();
          pixels.show();

          return;
        }

        if(Game.isXTurn){
          UI.drawMode = 2;
          Pos.maxParamState = 160;
          Pos.paramState = 0;

          Game.gameBoard[UI.xCoord][UI.yCoord] = 1;

          pixels.fill(pixels.Color(0, 255, 0), 2, pixels.numPixels());
          pixels.show();
          delay(200);
          pixels.clear();
          pixels.show();
        }
        else{
          UI.drawMode = 3;
          Pos.maxParamState = 160;
          Pos.paramState = 0;

          Game.gameBoard[UI.xCoord][UI.yCoord] = 2;

          pixels.fill(pixels.Color(0, 255, 0), 2, pixels.numPixels());
          pixels.show();
          delay(200);
          pixels.clear();
          pixels.show();
        }
        Game.isXTurn = !Game.isXTurn; 
      }
    }
    pixels.clear();
    if(selectXCoord){
      pixels.setPixelColor(UI.xCoord + 2, pixels.Color(255, 0, 0));
    } else {
      pixels.setPixelColor(UI.yCoord + 2, pixels.Color(0, 0, 255));
    }
    pixels.show();
    delay(50);
  }
}
#pragma endregion UI


//////////////////////////////////////////////////
//  ROBOT MOVEMENT FUNCTIONS //
//////////////////////////////////////////////////
#pragma region MOVE_BOT
// //////////////////////// Plotter Run Functions ////////////////////////////
// Handles pathing by advancing the parametric state. Calls run motors when colored pixel is found, but skips over blank space (white pixels).
void drawBoard(int drawType){
  if(Pos.paramState < Pos.maxParamState){
    if (!moveBot()){      
      float nextXY[2];          // init for new coordinates {X, Y}
      float nextL[2];           // init for next target lengths {L, R}

      Pos.paramState += Pos.paramStep;            // advance in time.

      switch(drawType){
        case 0:
          return;
          break;
        case 1:                                      // draw tic tac toe 
          drawGameBoard(Pos.paramState, nextXY);     // get new coord
          break;
        case 2:                                      // draw X
          drawX(Pos.paramState, nextXY, UI.xCoord, UI.yCoord);
          break;
        case 3:                                      // draw O
          drawO(Pos.paramState, nextXY, UI.xCoord, UI.yCoord);
          break;
        default:
          break;
      }

      Plotter.nextCoords[0] = nextXY[0];        // set coord as move target
      Plotter.nextCoords[1] = nextXY[1];

      XYToLengths(Plotter.nextCoords[0], Plotter.nextCoords[1], nextL);
      Plotter.nextLengths[0] = nextL[0];        // determine target lengths
      Plotter.nextLengths[1] = nextL[1];
    } 
  } else {
    //TO DO: Move bot out of the way.
    return;
  }
}

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

  Serial.print("left length to-go: ");
  Serial.println(Plotter.nextLengths[0] - Plotter.currentLengths[0]);
  Serial.print("right length to-go: ");
  Serial.println(Plotter.nextLengths[1] - Plotter.currentLengths[1]);
  

  //Disables pen when moing to start point, and when drawing is finished
  if(Pos.paramState < Pos.paramStep || Pos.paramState >= Pos.maxParamState - Pos.paramStep){
    penFree = false;
  }
  // Disables pen when moving to next Target 
  else if (abs(Plotter.currentLengths[0] - Plotter.nextLengths[0])
            > epsilon ||
           abs(Plotter.currentLengths[1] - Plotter.nextLengths[1]) 
            > epsilon) {
    penFree = false;
  } else {
    penFree = true;
  }

  // Get motor speeds
  getSpeeds(Plotter.nextLengths[0], Plotter.nextLengths[1], nextSpeeds);

  if(penFree){
    movePen(true);
  } else {
    movePen(false);
  }
  
  // Drive left motor until within epsilon, then set "position reached" flag to true. 
  if (abs(Plotter.currentLengths[0] - Plotter.nextLengths[0]) < epsilon){
    driveMotors(nextSpeeds[0], true);   // left
    Encoder.posLReached = true;
  } else {
    driveMotors(nextSpeeds[0], true);   // left
    Encoder.posLReached = false;
  }

  // Drive right motor.
  if (abs(Plotter.currentLengths[1] - Plotter.nextLengths[1]) < epsilon){
    driveMotors(nextSpeeds[1], false);   // right
    Encoder.posRReached = true;
  } else {
    driveMotors(nextSpeeds[1], false);   // right
    Encoder.posRReached = false;
  }

  // Returns true if either motor needs to move. Returns false when position is reached within the defined error.
  if (Encoder.posLReached && Encoder.posRReached){
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
  if(down || millis() < Pen.lastCallTime + 100){
    myPen.write(Pen.penDown);
    if(down){
      Pen.lastCallTime = millis();
    }
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


//////////////////////////////////////////////////
//  ENCODER FUNCTIONS //
//////////////////////////////////////////////////
#pragma region READ_ENCODERS
// ////////////////////////////// Encoder Functions ////////////////////////////
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
    return -1;                                                     // error
  }
}
#pragma endregion READ_ENCODERS


//////////////////////////////////////////////////
//  INVERSE KINEMATICS //
//////////////////////////////////////////////////
#pragma region COORDINATE_MAPPING
// /////////////////////// Inverse Kinetmatic Functions ////////////////////////
// The heart of the program lies in these few lines of code. The math below converts string lengths to cartesian coordinates, allowing us to position the robot where we want. 

void lengthsToXY(float lengthLeft, float lengthRight, float* XY){
  float s = (lengthLeft + lengthRight + Image.canvassWidth) / 2.0;
  float area = sqrt(s * (s - lengthLeft) * (s - lengthRight)
               * (s - Image.canvassWidth));

  XY[1] = 2.0 * area / Image.canvassWidth;                    // Y - coord
  XY[0] = sqrt(pow(lengthLeft, 2.0) - pow(XY[1], 2.0));       // X - coord
}

// converts cartesian coords to string lengths.
void XYToLengths(float x, float y, float* L){
  L[0] = sqrt(pow(x, 2.0) + pow(y, 2.0));                             //left
  L[1] = sqrt(pow((Image.canvassWidth - x), 2.0) + pow(y, 2.0));      //right
}
#pragma endregion COORDINATE_MAPPING


//////////////////////////////////////////////////
//  PARAMETRIC DRAW FUNCTIONSz//
//////////////////////////////////////////////////
#pragma region PARAMETRIC
// parametric function to draw the game board, X's, and O's.
void drawGameBoard(double time, float* XY){
  float x = 0;
  float y = 0;

  float lineCoord = boardSize / 3;

  if(time < boardSize){
    x = lineCoord;
    y = time;
  }
  else if(time < 2 * boardSize){
    x = 2 * lineCoord;
    y = (time - boardSize);
  }
  else if(time < 3 * boardSize){
    x = (time - 2 * boardSize);
    y = lineCoord;
  }
  else{
    x = (time - 3 * boardSize);
    y = 2 * lineCoord;
  }

  if(time >= Pos.maxParamState){
    UI.drawMode = 0;
    Pos.maxParamState = Pos.paramStep;;
    Pos.paramState = 0;
  }

  XY[0] = x + Image.centerXOffset;
  XY[1] = y + Image.centerYOffset;
}

void drawX(double time, float* XY, int gameX, int gameY){
  float x = 0;
  float y = 0;

  if(time < Pos.maxParamState / 2){
    x = time;
    y = time;
  } else {
    x = time - Pos.maxParamState / 2;
    y = Pos.maxParamState / 2 - (time - Pos.maxParamState / 2);
  }
  
  switch(gameX){
    case 0:
      Image.gameXOffset = boardSize / 6;
      break;
    case 1:
      Image.gameXOffset = boardSize / 2;
      break;
    case 2:
      Image.gameXOffset = boardSize / 2 + boardSize / 3;
      break;
    default:
      break;
  }

  switch(gameY){
    case 0:
      Image.gameYOffset = boardSize / 6;
      break;
    case 1:
      Image.gameYOffset = boardSize / 2;
      break;
    case 2:
      Image.gameYOffset = boardSize / 2 + boardSize / 3;
      break;
    default:
      break;
  }

  if(time >= Pos.maxParamState){
    UI.drawMode = 0;
    Pos.maxParamState = Pos.paramStep;
    Pos.paramState = 0;
  }
  
  XY[0] = x + Image.gameXOffset + Image.centerXOffset - Pos.maxParamState / 4;
  XY[1] = y + Image.gameYOffset + Image.centerYOffset - Pos.maxParamState / 4;  
}

void drawO(double time, float* XY, int gameX, int gameY){
  float theta = (time / Pos.maxParamState) * TWO_PI;

  // Parametric equations for circle
  float x = Pos.maxParamState / 4 * cos(theta);
  float y = Pos.maxParamState / 4 * sin(theta);

  switch(gameX){
    case 0:
      Image.gameXOffset = boardSize / 6;
      break;
    case 1:
      Image.gameXOffset = boardSize / 2;
      break;
    case 2:
      Image.gameXOffset = boardSize / 2 + boardSize / 3;
      break;
    default:
      break;
  }

  switch(gameY){
    case 0:
      Image.gameYOffset = boardSize / 6;
      break;
    case 1:
      Image.gameYOffset = boardSize / 2;
      break;
    case 2:
      Image.gameYOffset = boardSize / 2 + boardSize / 3;
      break;
    default:
      break;
  }
  
  if(time >= Pos.maxParamState){
    UI.drawMode = 0;
    Pos.maxParamState = Pos.paramStep;
    Pos.paramState = 0;
  }

  XY[0] = x + Image.gameXOffset + Image.centerXOffset;
  XY[1] = y + Image.gameYOffset + Image.centerYOffset;  
}
#pragma endregion PARAMETRIC
