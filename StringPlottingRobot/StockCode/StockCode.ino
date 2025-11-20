#pragma region README
/* Dear HackPack User,

this is code for a wall drawing robot that hangs from two anchor points by
adjusting the lengths of each string. Recommended image size is up to 250 x 250
pixels on the default "small" preset, 375 x 375 pixels on "medium", and 500 x
500 on large. Image print times vary from 30 mins - 4hrs based on size and pixel
density. Sizes for presets are as follows:

small - 500mm width, 260mm inital string lengths, 300mm vertical image center
medium - 750 mm width, 500mm inital string lengths, 350mm vertical image center
large - 1000 mm width, 720mm inital string lengths, 425mm vertical image center

To add your own images, be sure to add the corresponding file name to the
files[] list below, images should be formatted as a 24-bit color BitMaps ending
with ".bmp". If you're using your own SD card, be sure to set the storage type
to the FAT32 format so that the SdFat library can read the contents
(https://www.lifewire.com/format-sd-card-to-fat32-6666166).

On Boot up, the Plotter Robot will wait for a user selection. The Red button is
"next" and the green button is "select". Numbers 1 - 6 correspond to the 6 files
that come pre-loaded.

  The rectangle icon allows selecting between image sizes.
  The pen icon selects between different fill styles.
  E1, error 1- SD Card couldn't be read.
  E2, error 2- File type incorrect, needs to be ".bmp"

Visit our website to convert your images into bitmap! [Coming Soon...]

Lastly, the config.h file contains key variables for custom sizing, image
placement, parametric fill style, motor behavior, etc. Editing these values will
allow you to customize your robot's calibration.

Updated 7/22/2025
*/
#pragma endregion README
#pragma region LICENSE
/*
  ************************************************************************************
  * MIT License
  *
  * Copyright (c) 2025 CrunchLabs LLC (Balance Bot Code)

  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to
  deal
  * in the Software without restriction, including without limitation the rights
  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  * copies of the Software, and to permit persons to whom the Software is
  furnished
  * to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in
  all
  * copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED,
  * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR
  A
  * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
  COPYRIGHT
  * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
  ACTION OF
  * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE
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
#include <AS5600.h> // encoder
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h> // I2C Multiplexer
#include <SPI.h>             // SD
#include <Servo.h>
#include <ezButton.h>
#include <SdFat.h>           // SD card reading

// The following options are added to SdFat src/SdFatConfig.h to reduce code size:
// #define USE_FAT_FILE_FLAG_CONTIGUOUS 0
// #define ENABLE_DEDICATED_SPI 0
// #define USE_LONG_FILE_NAMES 0
// #define SDFAT_FILE_TYPE 1
// #define CHECK_FLASH_PROGRAMMING 0

#define TCAADDR 0x70
#define SERVOPIN 9
#define LEDPIN 10
#define SD_CS_PIN 14 // Chip select pin for the SD card module
#define PAUSEPIN 15
#define CURSORPIN 16
#define L_M_DIR 2
#define L_M_PWM 3
#define R_M_DIR 4
#define R_M_PWM 5
#define FILE_NUM 6

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

// file list--------------------------------------------------
String files[FILE_NUM] = {"1", "2", "3", "4", "5", "6"};
/*
1 - Mark Rober Logo
2 - CrunchLabs Logo
3 - Hack Pack Logo
4 - Penguin Highway
5 - Rick Roll
6 - Program Test Image
*/
#pragma endregion LIBRARIES_AND_CONFIGURATION

//////////////////////////////////////////////////
//  CLASS OBJECTS //
//////////////////////////////////////////////////
#pragma region CLASS_OBJECTS
// hardware definitions-----------------------------------

ezButton redButton(CURSORPIN, 20);
ezButton greenButton(PAUSEPIN, 20);

Adafruit_NeoPixel pixels(10, LEDPIN, NEO_GRB + NEO_KHZ800);

Servo myPen; // pen Servo

AS5600 encL; //  left
AS5600 encR; //  right

SdFat SD;
File bmpFile;
#pragma endregion CLASS_OBJECTS

//////////////////////////////////////////////////
//  STRUCTS //
//////////////////////////////////////////////////
#pragma region STRUCTS
// artboard & image params-------------------------------------
struct image {
  // Canvass height is effectively unlimited
  uint16_t canvassWidth; // Distance suction cups are apart
  uint16_t centerHeight; // Only limited by string length
  uint8_t setSize;

  uint16_t bmpWidth; // These are filled in by the program...
  uint16_t bmpHeight;
  uint16_t rowSize;
  uint16_t imgDatOffset;

  float centerXOffset; // Calculated offset to center bmp in canvass
  float centerYOffset;
};

// movement params-------------------------------------------
struct motor {
  float maxSpeed; // 255 is 100% duty cycle
  float drawSpeed;

  float setSpeeds[2];
};

// hardware specs--------------------------------------------
struct encoder {
  float rollerDiam; // diameter of encoder 12.5mm spool + 0.5mm string

  float encLInit;
  float encRInit; // inital Positions of encoders

  bool posLReached; // left. Is string at its target length?
  bool posRReached; // right

  long lastUpdateTime; // tracks encoder polling
};

// Pen Placement Variables-----------------------------------
struct pen {
  uint8_t penUp; // servo position presets
  uint8_t penDown;
  uint8_t penStart;

  float penPos;    // current servo position
  float penWindow; // How often pen behavior is updated
  float penMoveFlag;

  long lastCallTime;  // last millis pen behavior was updated
  long penDownTime;   // until which millis pen should be drawing
  long nextPenUpdate; // which millis pen behavior will be updated again
};

// parametric "time" variables ------------------
struct parametric {
  double paramState;    // parametric space filling variables...
  double maxParamState; // max param value
  double paramStep;     // step size
};

// UI variables-----------------------------------------------
struct ui {
  bool selectMode; // Conditional for UI while loop, no file chosen
  bool paused;     // Conditional for pause while loop

  bool sWasPressed; // Button debounce variables for Scroll button...
  bool pWasPressed; // ... and Pause Button

  uint8_t cursorPos; // cursor for choosing files or settings
};

// Robot global variables-----------------------------------
struct bot {
  float currentCoords[2];  // current XY pos. {X, Y}
  float currentLengths[2]; // current String Lengths (mm) {Left, Right}

  float nextCoords[2];  // next target coords {X, Y}
  float nextLengths[2]; // next target string length {Left, Right}

  float savedCoords[2];       // {X, Y} any coords needed in future
  float initStringLengths[2]; // {Left, Right} robot's startup position

  bool newBlankPix;
};

///////////////////////////// INIT STRUCTS /////////////////////////////////////

// canvass width, image center height, init size setting, bmp width, bmp height,
// fileDataRowSize, fileDataOffset, Image Cenetring Offset X, Image Cenetring
// Offset Y.
image Image = {0, 0, bootSize, 0, 0, 0, 0, 0, 0};
// motor max duty cycle, min, right speed, left speed
motor Motor = {motorTravelSpeed, drawSpeed, {0}};
// encoder diameter, zero1, zero0, reachedTarget1, reachedTarget2,
// lastUpdateTime
encoder Encoder = {12.5, 0.0, 0.0, false, false, 0};
// parametric start time, end time, timestep size, current step dx, dy.
parametric Pos = {0, 0, timeStep};
// pen Up servo pos, pen down pos, init pos, current pos, pen update period, pen
// move time, next scheduled update time
pen Pen = {penUpPos, penDownPos, penStartPos, 0, penRes, dotTime, 0};
// menu flag (starts true), is paused (starts false), green button pressed, red
// button pressed, selected file number (list index)
ui UI = {true, false, false, false, initCursorPos};
// current XY, current string lengths, next XY, next string lengths, any saved
// coordinates, inital string lengths.
bot Plotter = {{0}, {0}, {0}, {0}, {0}, {0}, false};
#pragma endregion STRUCTS

//////////////////////////////////////////////////
//  SETUP //
//////////////////////////////////////////////////
#pragma region SETUP_FUNCTIONS
/////////////////////// I2C Multiplexer Port Selector //////////////////////////
void tcaselect(uint8_t i) {
  if (i > 7)
    return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

////////////////////////////////// UI /////////////////////////////////////
// Enters this loop first. Prompts users to the use buttons to select a file, or
// edit settings before running the plotter.
void runUI() {
  myPen.detach();
  while (UI.selectMode) {
    redButton.loop(); // MUST call the loop() each iteration to update button
    greenButton.loop();

    // check for scroll/cancel (red) button press and release
    if (redButton.isPressed()) {
      UI.sWasPressed = true;
    }
    if (redButton.isReleased() && UI.sWasPressed) {
      UI.sWasPressed = false;
      UI.cursorPos += 1;            // cycle thru menu
      UI.cursorPos %= FILE_NUM + 2; // wrap back after the draw settings
    }

    // check for choose/pause (green) button press and release
    if (greenButton.isPressed()) {
      UI.pWasPressed = true;
    }

    if (greenButton.isReleased() && UI.pWasPressed) {
      // if hit choose on size icon
      if (UI.cursorPos == FILE_NUM) { // if size preset selection
        pixels.clear();

        Image.setSize += 1; // change size
        Image.setSize %= 3;
        switch (Image.setSize) {
        case 0:
          pixels.setPixelColor(8, pixels.Color(0, 255, 0)); // green, small
          break;
        case 1:
          pixels.setPixelColor(8, pixels.Color(255, 180, 0)); // yellow, med
          break;
        case 2:
          pixels.setPixelColor(8, pixels.Color(255, 0, 0)); // red, large
          break;
        default:
          break;
        }
        pixels.show();
      }

      // if hit choose on pen icon
      else if (UI.cursorPos == FILE_NUM + 1) { // pen movement style selection
        pixels.clear();

        penStyle += 1;
        penStyle %= 4;
        pixels.show();
      }

      // otherwise, we've hit choose on a file number
      else {
        UI.pWasPressed = false; // ensure not paused
        UI.selectMode = false;  // Exit UI loop with this flag

        for (uint8_t i = 0; i < 3; i++) { // flash selected pixel green
          pixels.clear();
          pixels.show();
          pixels.setPixelColor(UI.cursorPos + 2, pixels.Color(0, 255, 0));
          pixels.show();
          delay(300);
          pixels.setPixelColor(UI.cursorPos + 2, pixels.Color(0, 0, 0));
          pixels.show();
          delay(300);
        }
        if (Plotter.initStringLengths[0] == 0 &&
            Plotter.initStringLengths[1] == 0) {
          switch (Image.setSize) {
          case 0:
            Image.canvassWidth = SMALL_WIDTH - 80;
            Image.centerHeight = SMALL_Y_OFFSET;
            Plotter.initStringLengths[0] = SMALL_INIT;
            Plotter.initStringLengths[1] = SMALL_INIT;
            break;
          case 1:
            Image.canvassWidth = MED_WIDTH - 80;
            Image.centerHeight = MED_Y_OFFSET;
            Plotter.initStringLengths[0] = MED_INIT;
            Plotter.initStringLengths[1] = MED_INIT;
            break;
          case 2:
            Image.canvassWidth = LARGE_WIDTH - 80;
            Image.centerHeight = LARGE_Y_OFFSET;
            Plotter.initStringLengths[0] = LARGE_INIT;
            Plotter.initStringLengths[1] = LARGE_INIT;
            break;
          default:
            break;
          }
        }
      }
    }

    // Blink cursor LED purple if over file
    if (UI.cursorPos < 6) {
      int col = 255 * (0.5 + 0.5 * sin(millis() / 300.0));
      pixels.clear();
      pixels.setPixelColor(UI.cursorPos + 2, pixels.Color(col, 0, col));
      pixels.show();
    }
    // change cursor LED's color to select sizes
    else if (UI.cursorPos == 6) {
      int col = 255;
      pixels.clear();
      switch (Image.setSize) {
      case 0:
        pixels.setPixelColor(8, pixels.Color(0, col, 0));
        break;
      case 1:
        pixels.setPixelColor(8, pixels.Color(col, 180, 0));
        break;
      case 2:
        pixels.setPixelColor(8, pixels.Color(col, 0, 0));
        break;
      default:
        break;
      }
      pixels.show();
    }
    // change cursor LED's color to select pen style
    else {
      int col = 255;
      pixels.clear();
      switch (penStyle) {
      case 0:
        pixels.setPixelColor(9, pixels.Color(col, 0, 0));
        break;
      case 1:
        pixels.setPixelColor(9, pixels.Color(0, col, 0));
        break;
      case 2:
        pixels.setPixelColor(9, pixels.Color(0, 0, col));
        break;
      case 3:
        pixels.setPixelColor(9, pixels.Color(col, col, col));
        break;
      default:
        break;
      }
      pixels.show();
    }
  }
}

///////////////////////////// SETUP ///////////////////////////////
void setup() {
  pinMode(L_M_DIR, OUTPUT);
  pinMode(L_M_PWM, OUTPUT);
  pinMode(R_M_DIR, OUTPUT);
  pinMode(R_M_PWM, OUTPUT);
  pinMode(SERVOPIN, OUTPUT);
  pinMode(PAUSEPIN, INPUT_PULLUP);
  pinMode(CURSORPIN, INPUT_PULLUP);

  Wire.begin();
  pixels.begin();

  redButton.setDebounceTime(25);
  greenButton.setDebounceTime(25);

  myPen.attach(SERVOPIN); // pen Servo on pin 9
  myPen.write(Pen.penStart);

  delay(800);

  pixels.setBrightness(255);
  // StartUp sequence, file select from UI ---------------------------------
  runUI();
  myPen.attach(SERVOPIN);
  myPen.write(Pen.penUp);

  // SD Card ----------------------------------------------------------
  if (!SD.begin(SD_CS_PIN)) {
    // SD card initialization failed!
    while (true) {
      int sinwave = 255 * (0.5 + 0.5 * sin(millis() / 500.0));
      pixels.setPixelColor(0, pixels.Color(sinwave, 0, 0));
      pixels.show(); // flash error 1
    }
  }

  bmpFile = SD.open(files[UI.cursorPos] + ".bmp");
  if (!bmpFile) {
    // Could not open the BMP file.
    while (true) {
      int sinwave = 255 * (0.5 + 0.5 * sin(millis() / 500.0));
      pixels.setPixelColor(1, pixels.Color(sinwave, 0, 0));
      pixels.show(); // flash error 2
    }
  }

  // Bitmap Setup-------------------------------------------------------
  bmpFile.seek(10); // Seek to start position of img data
  Image.imgDatOffset = bmpFile.read() + (bmpFile.read() << 8) +
                       (bmpFile.read() << 16) + (bmpFile.read() << 24);
  bmpFile.seek(18); // Seek to width data in the BMP header
  Image.bmpWidth = bmpFile.read() + (bmpFile.read() << 8);
  bmpFile.seek(22); // Seek to height data
  Image.bmpHeight = bmpFile.read() + (bmpFile.read() << 8);

  // BMP format specifics, Each row is padded to a multiple of 4 bytes
  Image.rowSize = (Image.bmpWidth * 3 + 3) & ~3;

  // Offset needed to center image in canvass
  Image.centerXOffset = Image.canvassWidth / 2 - Image.bmpWidth / 2;
  Image.centerYOffset = Image.centerHeight - Image.bmpHeight / 2;

  // Parametric Setup From Image Parameters ----------------------------
  Pos.maxParamState = double(Image.bmpHeight) * double(Image.bmpWidth);

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
  encR.setDirection(AS5600_CLOCK_WISE); // default, just be explicit.
  Encoder.encRInit = encR.resetCumulativePosition(); // zero encoder at start

  // Generate first coordinate to move to
  float nextXY[2];
  float nextL[2];

  nextParametricCoord(Pos.paramState, nextXY);
  Plotter.nextCoords[0] = nextXY[0]; // X
  Plotter.nextCoords[1] = nextXY[1]; // Y

  XYToLengths(Plotter.nextCoords[0], Plotter.nextCoords[1], nextL);
  Plotter.nextLengths[0] = nextL[0]; // left
  Plotter.nextLengths[1] = nextL[1]; // right

  for (uint8_t i = 2; i < 2 + UI.cursorPos; i++) {
    pixels.setPixelColor(i, pixels.Color(5, 1, 5));
  }
  pixels.setPixelColor(2 + UI.cursorPos, pixels.Color(100, 0, 100));
  pixels.show();
}
#pragma endregion SETUP_FUNCTIONS

//////////////////////////////////////////////////
//  LOOP //
//////////////////////////////////////////////////
#pragma region LOOP
void loop() {
  // Check for user inputs
  pollButtons();
  // Check if paused
  if (UI.paused) {
    pause();
    return; // end loop here
  }
  // Run bot
  runPlotter();
}
#pragma endregion LOOP

//////////////////////////////////////////////////
// BUTTON PRESS ACTIONS//
//////////////////////////////////////////////////
#pragma region RUN_BUTTONS
///////////////////////////// UI Functions ////////////////////////////////
// looks for user inputs for pause or restart
void pollButtons() {
  redButton.loop(); // MUST call the loop() first
  greenButton.loop();

  if (redButton.isPressed()) {
    UI.sWasPressed = true;
  }
  if (redButton.isReleased() && UI.sWasPressed) {
    UI.sWasPressed = false;
    restart(true); // restart on release
    return;
  }

  if (greenButton.isPressed()) {
    UI.pWasPressed = true;
  }
  if (greenButton.isReleased() && UI.pWasPressed) {
    UI.pWasPressed = false;
    UI.paused = !UI.paused; // toggle pause on release
    if (UI.paused) {
      myPen.detach();
    } else {
      myPen.attach(SERVOPIN); // if resume, light selected file
      pixels.clear();
      pixels.setPixelColor(UI.cursorPos + 2, pixels.Color(100, 0, 100));
      pixels.show();
    }
  }
}

// Freeze motors, Lift Pen, and run light animations for paused state.
void pause() {
  driveMotors(0, true);  // left -> true
  driveMotors(0, false); // right -> false
  myPen.write(Pen.penUp);

  for (uint8_t i = 0; i < pixels.numPixels(); i++) { // "paused" LED animation
    uint32_t color =
        pixels.Color(0, 255 * (0.5 + 0.5 * sin(millis() / 1000.0)), 0);

    pixels.setPixelColor(i, color);
  }
  pixels.show();
}

// homes the plotter bot, then goes back to file selection
void restart(bool move) {
  bmpFile.close();

  float homeXY[2];
  float homeLen[2];

  uint32_t nextUpdate = 0; // for light animations
  uint8_t lightPos = pixels.numPixels();
  uint8_t fillPos = 0;

  Pos.paramState = 0; // reset values to inital state

  UI.selectMode = true;
  UI.paused = false;

  nextParametricCoord(Pos.paramState, homeXY); // set target coords to home
  Plotter.nextCoords[0] = homeXY[0];           // x
  Plotter.nextCoords[1] = homeXY[1];           // y

  XYToLengths(homeXY[0], homeXY[1], homeLen); // calculate home string lengths
  Plotter.nextLengths[0] = homeLen[0];        // left
  Plotter.nextLengths[1] = homeLen[1];        // right

  // Bot should move to "home" when canceling a drawing (move == true), but
  // should not move when the drawing is finished (move == false), or it erases
  // over itself.
  if (move) {
    while (moveBot()) {            // runs until home is reached
      if (millis() > nextUpdate) { // animate homing light effect
        nextUpdate = millis() + 100;
        pixels.clear();
        pixels.setPixelColor(lightPos, pixels.Color(0, 255, 0));
        lightPos--;

        if (lightPos < fillPos + 1) {
          lightPos = pixels.numPixels();
          fillPos++;
          fillPos %= pixels.numPixels();
        }

        for (uint8_t i = 0; i < fillPos; i++) {
          pixels.setPixelColor(i, pixels.Color(0, 255, 0));
        }
        pixels.show();
      }
    }
  }

  // Stop both motors after homing
  driveMotors(0, true);  // left
  driveMotors(0, false); // right

  Plotter.initStringLengths[0] = Plotter.currentLengths[0];
  Plotter.initStringLengths[1] = Plotter.currentLengths[1];

  delay(100);
  setup();
}
#pragma region RUN_BUTTONS

//////////////////////////////////////////////////
//  MOVE ROBOT //
//////////////////////////////////////////////////
#pragma region RUN_BOT
/////////////////////////// Plotter Run Functions //////////////////////////////
// Handles pathing by advancing the parametric state. Calls run motors when any
// colored pixel is found, but skips over blank space (white pixels).
void runPlotter() {
  if (Pos.paramState < Pos.maxParamState) {
    if (!moveBot()) {
      // If target position is reached, loop to find next colored pixel.
      while (Pos.paramState < Pos.maxParamState) {

        float nextXY[2]; // init for new coordinates {X, Y}
        float nextL[2];  // init for next target lengths {L, R}

        uint8_t pixVal;

        Pos.paramState += Pos.paramStep;             // advance in time.
        nextParametricCoord(Pos.paramState, nextXY); // get new coord

        pixVal = readBmpPixel((nextXY[0] - Image.centerXOffset - manualXOffset),
                              (nextXY[1] - Image.centerYOffset -
                               manualYOffset)); // read pixel color at coord

        if (pixVal != 0) {                   // if colored pix,
          Plotter.nextCoords[0] = nextXY[0]; // set coord as move target
          Plotter.nextCoords[1] = nextXY[1];

          XYToLengths(Plotter.nextCoords[0], Plotter.nextCoords[1], nextL);
          Plotter.nextLengths[0] = nextL[0]; // determine target lengths
          Plotter.nextLengths[1] = nextL[1];

          Plotter.newBlankPix = true;
          break;                 // exit loop, Target chosen
        } else {                 // Otherwise, blank pixel.
          driveMotors(0, true);  // pause left motor
          driveMotors(0, false); // right

          Motor.setSpeeds[0] = 0;
          Motor.setSpeeds[1] = 0;

          myPen.write(Pen.penUp); // raise servo
          if (Plotter.newBlankPix) {
            delay(50); // if new white pix, pause for servo.
          }
          Plotter.newBlankPix = false;
          Pos.paramState += (1.0 - Pos.paramStep); // Advance by 1 pix.
        }
      }
    } else {
      return;
    }
  } else {          // max time, reset bot.
    restart(false); // false -> reset without homing
  }
}

// Moves motor towards next coordinate, returns true if either motor moved. Code
// continually tries to move towards the exact target position every iteration,
// boolean "target reached" flags will update if error is less than epsilon set
// by user.
bool moveBot() {
  int nextSpeeds[2];  // define motor speeds {L, R}
  float currentXY[2]; // current position, local var
  float dir[2];       // direction of next movement

  bool penFree = false; // enable pen movement

  updateStringLength(); // updates currentLengths[]

  lengthsToXY(Plotter.currentLengths[0], Plotter.currentLengths[1], currentXY);
  Plotter.currentCoords[0] = currentXY[0]; // updates currentCoords
  Plotter.currentCoords[1] = currentXY[1];

  // Disables pen when moving to start point, and when drawing is finished
  if (Pos.paramState < Pos.paramStep ||
      Pos.paramState >= Pos.maxParamState - Pos.paramStep) {
    penFree = false;
  }
  // Disables pen when moving to next Target
  else if (abs(Plotter.currentLengths[0] - Plotter.nextLengths[0]) > epsilon ||
           abs(Plotter.currentLengths[1] - Plotter.nextLengths[1]) > epsilon) {
    penFree = false;
  } else {
    penFree = true;
  }

  // Get motor speeds
  getSpeeds(Plotter.nextLengths[0], Plotter.nextLengths[1], nextSpeeds);

  if (penFree) {
    switch (penStyle) { // run pen movements
    case 0:
      movePen();
      break;
    case 1:
      movePen2();
      break;
    case 2:
      movePen3();
      break;
    case 3:
      movePen4();
      break;
    default:
      break;
    }
  }

  // Drive left motor until within epsilon, then set "position reached" flag to
  // true.
  if (abs(Plotter.currentLengths[0] - Plotter.nextLengths[0]) < epsilon) {
    driveMotors(nextSpeeds[0], true); // left
    Encoder.posLReached = true;
  } else {
    driveMotors(nextSpeeds[0], true); // left
    Encoder.posLReached = false;
  }

  // Drive right motor.
  if (abs(Plotter.currentLengths[1] - Plotter.nextLengths[1]) < epsilon) {
    driveMotors(nextSpeeds[1], false); // right
    Encoder.posRReached = true;
  } else {
    driveMotors(nextSpeeds[1], false); // right
    Encoder.posRReached = false;
  }

  // Returns true if either motor needs to move. Returns false when position is
  // reached within the defined error.
  if (Encoder.posLReached && Encoder.posRReached) {
    return false;
  } else {
    return true;
  }
}

// Call function with true to move left motor, false for right motor.
void driveMotors(int speed, bool left) {
  if (left) {
    if (speed < 0) {
      digitalWrite(L_M_DIR, HIGH);
    } else {
      digitalWrite(L_M_DIR, LOW);
    }
    analogWrite(L_M_PWM, abs(speed));
    Motor.setSpeeds[0] = speed;
  } else {
    if (speed < 0) {
      digitalWrite(R_M_DIR, HIGH);
    } else {
      digitalWrite(R_M_DIR, LOW);
    }
    analogWrite(R_M_PWM, abs(speed));
    Motor.setSpeeds[1] = speed;
  }
}

// Style 1, every penUnit of distance, the pen is held for darkness / 255
// fraction of the distance.
void movePen() {

  // calculate distance move since last save position
  float dist = sqrt(pow(Plotter.currentCoords[0] - Plotter.savedCoords[0], 2) +
                    pow(Plotter.currentCoords[1] - Plotter.savedCoords[1], 2));

  // if distance > pen dotting unit...
  if (dist > penUnit) {
    //... create new save position
    Plotter.savedCoords[0] = Plotter.currentCoords[0];
    Plotter.savedCoords[1] = Plotter.currentCoords[1];

    float pixVal = readBmpPixel(
        (Plotter.currentCoords[0] - Image.centerXOffset - manualXOffset),
        (Plotter.currentCoords[1] - Image.centerYOffset - manualYOffset));
    // Modify pixel darkness to create better effect
    if (pixVal < 128) {
      pixVal = pow(pixVal / 255.0, 0.6) * 255.0;
    }

    // Calculate when to place pen in next unit (based on distance and darkness)
    Pen.penMoveFlag = penUnit - penUnit * (pixVal / 255.0);
  } else {
    float currentDist =
        sqrt(pow(Plotter.currentCoords[0] - Plotter.savedCoords[0], 2) +
             pow(Plotter.currentCoords[1] - Plotter.savedCoords[1], 2));
    if (currentDist > Pen.penMoveFlag * 0.75) {
      Pen.penPos =
          Pen.penPos * penSmoothFactor + Pen.penDown * (1.0 - penSmoothFactor);
      myPen.write(Pen.penPos);
    } else {
      // lift pen
      Pen.penPos = Pen.penUp;
      myPen.write(Pen.penPos);
    }
  }
}

// Style 2, similar to above, but timing based. Every penWindow milliseconds,
// the pen is down for blackval/255 fraction of the time.
void movePen2() {
  if (millis() > Pen.nextPenUpdate) { // every 500 ms window
    float pixVal;
    pixVal = readBmpPixel(
        (Plotter.currentCoords[0] - Image.centerXOffset - manualXOffset),
        (Plotter.currentCoords[1] - Image.centerYOffset - manualYOffset));

    // Quadratically modify the pixel value to slightly darken lighter colors
    if (pixVal <= 128) {
      pixVal = pow(pixVal / 255.0, 0.75) * 255.0;
    }

    // calculate for how long each penWindow to place pen down
    int window = Pen.penWindow * pixVal / 255.0;

    Pen.penDownTime = millis() + (Pen.penWindow - window - Pen.penMoveFlag);
    Pen.nextPenUpdate = millis() + Pen.penWindow;
  } else {
    // if past pendown window
    if (millis() > Pen.penDownTime) {
      // lower pen
      Pen.penPos =
          Pen.penPos * penSmoothFactor + Pen.penDown * (1.0 - penSmoothFactor);
      myPen.write(Pen.penPos);
    } else {
      // lift pen
      Pen.penPos = Pen.penUp;
      myPen.write(Pen.penPos);
    }
  }
}

// Style 3, Pen movement follows a cosine wave whose period is scaled by pixel
// darkness. Set "dps" to edit maximum dots placed per second.
void movePen3() {
  long phaseShift;

  // We've started a new dotting sequence if 800ms has elapsed between calls to
  // this function. If we have a new call, then we need to calculate a new phase
  // shift so that we can align the millis() sinewave with the current pen
  // position. This helps significantly with the pitter-patter jerkiness
  // behavior.
  if (millis() > Pen.lastCallTime + 800) {
    phaseShift = millis();
  } else {
    phaseShift = 0;
  }

  // Update the "bew call" trigger flag
  Pen.lastCallTime = millis();

  float pixVal = readBmpPixel(
      (Plotter.currentCoords[0] - Image.centerXOffset - manualXOffset),
      (Plotter.currentCoords[1] - Image.centerYOffset - manualYOffset));

  // quadratic relation to lighten pixels. Seems to help in practice from
  // multiple test runs
  if (pixVal < 128) {
    pixVal = pow((pixVal / 255.0), 2.0) * 255.0;
  }

  // create a cos wave that oscillates "dps" times per second. Then scale the
  // period by pixel color.
  float wave = cos((2.0 * PI * (millis() - phaseShift) / (1000.0 / dps)) *
                   (pixVal / 255.0));
  // multiply amplitude to the wave.
  Pen.penPos = wave * (Pen.penUp - Pen.penDown + 2.0 * penHold) / 2.0 +
               (Pen.penUp + Pen.penDown) / 2.0; // centering shift for pos
  // constrain to pen position to bounds
  Pen.penPos = constrain(Pen.penPos, Pen.penDown, Pen.penUp);

  myPen.write(Pen.penPos);
}

// percent darkness is used as percent chance that a particular dot is placed
// every 50ms
void movePen4() {
  if (millis() > Pen.nextPenUpdate) { // every 50 ms window
    float pixVal = readBmpPixel(
        (Plotter.currentCoords[0] - Image.centerXOffset - manualXOffset),
        (Plotter.currentCoords[1] - Image.centerYOffset - manualYOffset));
    int rand = random(0, 255);

    if (rand <= int(pixVal)) {
      Pen.penMoveFlag = 1;
    } else {
      Pen.penMoveFlag = 0;
    }

    if (Pen.penMoveFlag != 0) {
      Pen.penPos = max(Pen.penDown, Pen.penPos - 30);
      myPen.write(Pen.penPos);
      if (Pen.penPos == Pen.penDown) {
        Pen.penMoveFlag = 0;
      }
    } else {
      Pen.penPos = Pen.penUp;
      myPen.write(Pen.penPos);
    }

    Pen.nextPenUpdate = millis() + 50;
  }
}

/////////////////////// Movement Control Functions //////////////////////////
// Gives speeds to move each motor towards it's target position. Once within
// Linearly scales speed relative to distance-to-go with a lower bound at the
// minimum duty cycle required to lift the robot.
void getSpeeds(float toLengthLeft, float toLengthRight, int *nextSpeeds) {
  float leftMScale = 1;
  float rightMScale = 1;

  // First, store distance remaining in these variables for future calculations
  float speedL = (toLengthLeft - Plotter.currentLengths[0]);  // left
  float speedR = (toLengthRight - Plotter.currentLengths[1]); // right

  // If target position is hit, slow the motor linearly near exact position
  if (abs(speedL) < epsilon) {
    leftMScale = max(abs(speedL) / epsilon, epsilonScaling);
  }
  //
  else if (abs(speedL) < motorCoastDist) {
    if (speedL >= 0) {
      speedL = Motor.drawSpeed;
    } else {
      speedL = -Motor.drawSpeed;
    }
  } else {
    if (speedL >= 0) {
      speedL = Motor.maxSpeed;
    } else {
      speedL = -Motor.maxSpeed;
    }
  }

  if (abs(speedR) < epsilon) {
    rightMScale = max(epsilonScaling * abs(speedR) / epsilon, epsilonScaling);
  } else if (abs(speedR) < motorCoastDist) {
    if (speedR >= 0) {
      speedR = Motor.drawSpeed;
    } else {
      speedR = -Motor.drawSpeed;
    }
  } else {
    if (speedR >= 0) {
      speedR = Motor.maxSpeed;
    } else {
      speedR = -Motor.maxSpeed;
    }
  }

  // set left motor speed to speedL variable
  if (speedL >= 0) {
    speedL = Motor.drawSpeed * leftMScale;
  } else {
    speedL = -Motor.drawSpeed * leftMScale;
  }

  // set right motor speed to speedR variable
  if (speedR >= 0) {
    speedR = Motor.drawSpeed * rightMScale;
  } else {
    speedR = -Motor.drawSpeed * rightMScale;
  }

  // flipping motors
  if (flipLeftMotor) {
    speedL = -speedL;
  }
  if (flipRightMotor) {
    speedR = -speedR;
  }

  nextSpeeds[0] = speedL;
  nextSpeeds[1] = speedR;
}
#pragma endregion RUN_BOT

//////////////////////////////////////////////////
//  READ ENCODERS FOR STRING LENGTHS //
//////////////////////////////////////////////////
#pragma region ENCODERS
//////////////////////// Encoder Functions ////////////////////////////
// Updates currentLengths global variable based on encoder positions.
void updateStringLength() {
  //  update every 10 ms
  if (millis() - Encoder.lastUpdateTime >= sampleTime) {
    Encoder.lastUpdateTime = millis();

    // Left Encoder
    tcaselect(0);
    int32_t thetaL = encL.getCumulativePosition();
    Plotter.currentLengths[0] = lengthFromCounts(thetaL, 0);

    // Right Encoder
    tcaselect(1);
    int32_t thetaR = encR.getCumulativePosition();
    Plotter.currentLengths[1] = lengthFromCounts(thetaR, 1);
  }
}

// returns lengths (in mm) from encoder moments and inital zero.
// encNum = 0 - left; 1 - right
float lengthFromCounts(int32_t pos, uint8_t encNum) {
  // Left
  if (encNum == 0) {
    if (flipLeftEncoder) {
      return -pos / 4096.0 * Encoder.rollerDiam * PI +
             Plotter.initStringLengths[0];
    } else {
      return pos / 4096.0 * Encoder.rollerDiam * PI +
             Plotter.initStringLengths[0];
    }
  }
  // Right
  else if (encNum == 1) {
    if (flipRightEncoder) {
      return -pos / 4096.0 * Encoder.rollerDiam * PI +
             Plotter.initStringLengths[1];
    } else {
      return pos / 4096.0 * Encoder.rollerDiam * PI +
             Plotter.initStringLengths[1];
    }
  } else {
    return -1; // error
  }
}
#pragma endregion ENCODERS

//////////////////////////////////////////////////
//  INVERSE KINEMATICS //
//////////////////////////////////////////////////
#pragma region COORDINATE_MAPPING
////////////////////// Inverse Kinematic Functions ////////////////////////
// The heart of the program lies in these few lines of code. The math below
// converts string lengths to cartesian coordinates, allowing us to position the
// robot where we want.
void lengthsToXY(float lengthLeft, float lengthRight, float *XY) {
  float s = (lengthLeft + lengthRight + Image.canvassWidth) / 2.0;
  float area =
      sqrt(s * (s - lengthLeft) * (s - lengthRight) * (s - Image.canvassWidth));

  XY[1] = 2.0 * area / Image.canvassWidth;              // Y - coord
  XY[0] = sqrt(pow(lengthLeft, 2.0) - pow(XY[1], 2.0)); // X - coord
}

// converts cartesian coords to string lengths. Call with next = 0 to update
// current lengths, or next = 1 to set next lengths.
void XYToLengths(float x, float y, float *L) {
  L[0] = sqrt(pow(x, 2.0) + pow(y, 2.0));                        // left
  L[1] = sqrt(pow((Image.canvassWidth - x), 2.0) + pow(y, 2.0)); // right
}

#pragma endregion COORDINATE_MAPPING

//////////////////////////////////////////////////
//  PARAMETRIC PATHING //
//////////////////////////////////////////////////
#pragma region PARAMETRIC
/////////////////////// Parametric Draw Functions //////////////////////////////

// parametric function zigzag to fill from top down. Units are in mm. Travels
// over every pixel in the image when called from 0 < time < maxParamState.
void nextParametricCoord(double time, float *XY) {
  // Computes integer division of parametric variable to get row number.
  long long y = (long long)time / (long long)Image.bmpWidth;
  // Remainder is used to get column value
  double x = time - y * Image.bmpWidth;

  // repair rounding error effects at the boundary
  if (x >= Image.bmpWidth) {
    x = 0;
    y += 1; // Move to the next line
  }

  // flip x-coord of every-other row to make a continuous path
  if (int(y) % 2 == 1) {
    if (Image.bmpWidth - x - 1 > 0) {
      x = Image.bmpWidth - x - 1; // Subtract 1 to stay within range
    } else {
      x = Image.bmpWidth - x;
    }
  }

  XY[0] = x + Image.centerXOffset + manualXOffset;
  XY[1] = y + Image.centerYOffset + manualYOffset;
}

void nextCoordMyFunc(float time, float *XY) {
  // Add your own parametric fill functions here!!
}
#pragma endregion PARAMETRIC

//////////////////////////////////////////////////
//  READ FILES //
//////////////////////////////////////////////////
#pragma region FILE_PROCESSING
////////////////// BMP Image Read Functions ///////////////////////////
uint8_t readBmpPixel(int x, int y) {
  if (x >= Image.bmpWidth || y >= Image.bmpHeight || x < 0 || y < 0){
    return -1;     // return white pixel if coord out of image range
  }

  // x-coordinate is flipped due to way we calculated string lengths
  uint32_t flippedX = Image.bmpWidth - 1 - x;
  // BMP rows are stored bottom-to-top, so we need to flip the y-coordinate
  uint32_t flippedY = Image.bmpHeight - 1 - y;
  // Calculate the byte offset to reach the desired pixel
  uint32_t pixelStart = Image.imgDatOffset + (flippedY * Image.rowSize) 
                                     + (x * 3);

  // Seek to the pixel's position
  bmpFile.seek(pixelStart);

  // Read the pixel value (assuming 24-bit BMP format)
  uint8_t blue = bmpFile.read();
  uint8_t green = bmpFile.read();
  uint8_t red = bmpFile.read();

  // Take avergage RGB to get grayscale brightness
  uint16_t white = (blue + green + red) / 3;

  // return 8-bit black intensity
  uint8_t black = 255 - white;
  return black;
}

#pragma endregion FILE_PROCESSING
