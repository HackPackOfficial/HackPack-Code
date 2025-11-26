//////////////////////////////////////////////////
              //  LICENSE  //
//////////////////////////////////////////////////
#pragma region LICENSE
/*
  ************************************************************************************
  * MIT License
  *
  * Copyright (c) 2025 Crunchlabs LLC (LabelMaker Code)

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
               //  LIBRARIES  //
//////////////////////////////////////////////////
#pragma region LIBRARIES
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Stepper.h>
#include <ezButton.h>
#include <Servo.h>

#pragma endregion LIBRARIES

//////////////////////////////////////////////////
          //  PINS AND PARAMETERS  //
//////////////////////////////////////////////////
#pragma region PINS AND PARAMS

LiquidCrystal_I2C lcd(0x27, 16, 2);  // Set the LCD address to 0x27 for a 16x2 display

ezButton button1(14); //joystick button handler
#define INIT_MSG "Initializing..." // Text to display on startup
#define MODE_NAME "  LIGHT DRIVER  " //these are variables for the text which is displayed in different menus. 
#define SCORE "  SCORE:        " //try changing these, or making new ones and adding conditions for when they are used
//NOTE: These strings need to be 16 characters long to fit on the LCD screen


// Joystick setup
const int joystickXPin = A2;  // Connect the joystick X-axis to this analog pin
const int joystickYPin = A1;  // Connect the joystick Y-axis to this analog pin
const int joystickButtonThreshold = 200;  // Adjust this threshold value based on your joystick

// Menu parameters
int currentCharacter = 0; //keep track of which character is currently displayed under the cursor
int cursorPosition = 0; //keeps track of the cursor position (left to right) on the screen
int currentPage = 0; //keeps track of the current page for menus
const int charactersPerPage = 16; //number of characters that can fit on one row of the screen

// Stepper motor parameters
const int stepCount = 200;
const int stepsPerRevolution = 2048;

// initialize the stepper library for both steppers:
Stepper xStepper(stepsPerRevolution, 6, 8, 7, 9);
Stepper yStepper(stepsPerRevolution, 2, 4, 3, 5); 

int xPins[4] = {6, 8, 7, 9};  // pins for x-motor coils
int yPins[4] = {2, 4, 3, 5};    // pins for y-motor coils

//Servo
const int SERVO_PIN  = 13;
Servo servo;
int angle = 30; // the current angle of servo motor


// Creates states to store what the current menu and joystick states are
// Decoupling the state from other functions is good because it means the sensor / screen aren't hardcoded into every single action and can be handled at a higher level
enum State { MainMenu, LightDriver, GameOver };
State currentState = MainMenu;
State prevState = GameOver;

enum jState {LEFT, RIGHT, UP, DOWN, MIDDLE, UPRIGHT, UPLEFT, DOWNRIGHT, DOWNLEFT};
jState joyState = MIDDLE;
jState prevJoyState = MIDDLE;

boolean pPenOnPaper = false; // pen on paper in previous cycle
int lineCount = 0;
int points = 0;
int xpos = 0;
int ypos = 0;
int ymax = 3000;
bool joyUp;
bool joyDown;
bool joyLeft;
bool joyRight;
int button1State;
int joystickX;
int joystickY;
#pragma endregion PINS AND PARAMS

//////////////////////////////////////////////////
               //  S E T U P  //
//////////////////////////////////////////////////
#pragma region SETUP
void setup() {
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print(INIT_MSG);  // print start up message

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);

  button1.setDebounceTime(50);  //debounce prevents the joystick button from triggering twice when clicked

  servo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
  servo.write(angle);

  plot(false);  //servo to tape surface so pen can be inserted

  // set the speed of the motors 
  yStepper.setSpeed(12);  // set first stepper speed (these should stay the same)
  xStepper.setSpeed(10);  // set second stepper speed (^ weird stuff happens when you push it too fast)

  penUp();      //ensure that the servo is lifting the pen carriage away from the tape
  homeYAxis();  //lower the Y axis all the way to the bottom

  ypos = 0;
  xpos = 0;

  releaseMotors();
  lcd.clear();
}
#pragma endregion SETUP

//////////////////////////////////////////////////
               //  L O O P  //
//////////////////////////////////////////////////
#pragma region LOOP
void loop() {

  button1.loop();
  button1State = button1.getState();

  joystickX = analogRead(joystickXPin);
  joystickY = analogRead(joystickYPin);
  joyUp = joystickY < (512 - joystickButtonThreshold);
  joyDown = joystickY > (512 + joystickButtonThreshold);
  joyLeft = joystickX < (512 - joystickButtonThreshold);
  joyRight = joystickX > (512 + joystickButtonThreshold);

  switch (currentState) { //state machine that determines what to do with the input controls based on what mode the device is in
  
    case LightDriver:
    {
      if (prevState == MainMenu){
         prevState = LightDriver;
         lcd.clear();
         lcd.setCursor(0,0);
         lcd.print(MODE_NAME);
         lcd.setCursor(0,1);
         lcd.print("  SCORE:        ");
         cursorPosition = 8;
         Serial.println("Starting Game");
      }
      int speed_scaler = 3; // number of times to run the movement, alters speed, keep low-ish so you don't get locked into this for loop forever

      for(int i = 0; i < 2; i++){

        if (joyUp && ypos <= ymax-speed_scaler) { // ensure that our height is within the tape range
            yStepper.step(speed_scaler); //move up by a certain amount if the joystick is pushed up.
            ypos++;
            if(points > 0) xStepper.step(2);
        }

        if (joyDown && ypos > 0) {
            yStepper.step(-speed_scaler);
            ypos--;
            if(points > 0) xStepper.step(2);
        }
      }


      if(analogRead(A7) < 500){
        xStepper.step(2);
        xpos++;
        points = xpos/4;
      } else if(points != 0 || points >= 9999){
        points = 0;
        xpos = 0;
        lcd.setCursor(0,1);
        lcd.print(SCORE);
        cursorPosition = 8;
      }

      lcd.setCursor(cursorPosition, 1);
      lcd.print(points);

    }

    break;
    
    case MainMenu:
      {
      if (prevState != MainMenu){
         lcd.clear();
         lcd.setCursor(0,0);
         lcd.print(MODE_NAME);
         lcd.setCursor(0,1);
         lcd.print("      START     ");
         cursorPosition = 5;
         prevState = MainMenu;
        Serial.println("Main Menu");
      }

      lcd.setCursor(cursorPosition, 1);

      if (millis() % 600 < 400) {  // Blink the cursor
       lcd.print(">");
      } else {
       lcd.print(" ");
      }

      if (button1.isPressed()) { //handles clicking options in text size setting
        lcd.clear();
        currentState = LightDriver;
        prevState = MainMenu;
      }
    }
      break;

    //hack prompt: it would be pretty cool to add a high score screen, maybe high score could even get saved to EEPROM so it's recalled after shutting off?
    //hack prompt: create a case for a Game Over screen when you lose
  }
}
#pragma endregion LOOP

//////////////////////////////////////////////////
              // FUNCTIONS  //
//////////////////////////////////////////////////
#pragma region FUNCTIONS
void plot(boolean penOnPaper) { //used to handle lifting or lowering the pen on to the tape
  if(penOnPaper){//if the pen is already up, put it down
    angle = 80;
  } else {  //if down, then lift up.
    angle = 25;
    }
  servo.write(angle);                        //actuate the servo to either position.
  if (penOnPaper != pPenOnPaper) delay(50);  //gives the servo time to move before jumping into the next action
  pPenOnPaper = penOnPaper;                  //store the previous state.
}

void penUp() {  //singular command to lift the pen up
  servo.write(25);
}

void penDown() {  //singular command to put the pen down
  servo.write(80);
}

void releaseMotors() {
  for (int i = 0; i < 4; i++) {  //deactivates all the motor coils
    digitalWrite(xPins[i], 0);   //just picks each motor pin and send 0 voltage
    digitalWrite(yPins[i], 0);
  }
  plot(false);
}

void homeYAxis() {
  yStepper.step(-3000);  //lowers the pen holder to it's lowest position.
}

void resetScreen() {
  lcd.clear();          // clear LCD
  lcd.setCursor(0, 0);  // set cursor to row 0 column 0
  lcd.print(": ");
  lcd.setCursor(1, 0);  //move cursor down to row 1 column 0
  cursorPosition = 1; 
}
#pragma endregion FUNCTIONS

//////////////////////////////////////////////////
               //  END CODE  //
//////////////////////////////////////////////////