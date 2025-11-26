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
#define MODE_NAME "    MARK LOGO   " //these are variables for the text which is displayed in different menus. 
#define PLOTTING "   PLOTTING...  " //try changing these, or making new ones and adding conditions for when they are used


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
enum State { MainMenu, Editing, PrintConfirmation, Printing, plotMarkLogo };
State currentState = MainMenu;
State prevState = Printing;

enum jState {LEFT, RIGHT, UP, DOWN, MIDDLE, UPRIGHT, UPLEFT, DOWNRIGHT, DOWNLEFT};
jState joyState = MIDDLE;
jState prevJoyState = MIDDLE;

boolean pPenOnPaper = false; // pen on paper in previous cycle
int lineCount = 0;

int xpos = 0;
int ypos = 0;
const int posS = 2;
const int posM = 7;
const int posL = 12;
bool joyUp;
bool joyDown;
bool joyLeft;
bool joyRight;
int button1State;
int joystickX;
int joystickY;
#pragma endregion PINS AND PARAMS

//////////////////////////////////////////////////
           //  LOGO VECTORS  //
//////////////////////////////////////////////////
#pragma region LOGO VECTORS
//the first number is X coordinate, second is Y coordinate, and third is pen up / down (0 = up)

const uint8_t MRvector[22][3] = { 
  //try calling the plotMR() function instead of printing on button press 
  {57, 6, 0},
  {33, 9, 1},
  {60, 15, 0},
  {30, 18, 1},
  {60, 24, 0},
  {30, 27, 1},
  {30, 36, 1},
  {21, 42, 1},
  {21, 75, 1},
  {45, 84, 1},
  {71, 75, 1},
  {71, 42, 1},
  {60, 36, 1},
  {60, 30, 1},
  {60, 48, 0},
  {60, 69, 1},
  {45, 63, 1},
  {30, 69, 1},
  {30, 48, 1},
  {45, 63, 0},
  {45, 42, 1},
  {0,0,0}
};
#pragma endregion LOGO VECTORS

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
  yStepper.setSpeed(10);    // set first stepper speed
  xStepper.setSpeed(8);   // set second stepper speed

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
  
    case plotMarkLogo:
      {
        if (prevState != plotMarkLogo){
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print(MODE_NAME); //display the mode name
          lcd.setCursor(0,1);
          lcd.print(PLOTTING);
          cursorPosition = 5;
          prevState = plotMarkLogo;
      }

      lcd.setCursor(cursorPosition, 1);

      plotMR();
      homeYAxis();
      releaseMotors();
      lcd.clear();
      currentState = MainMenu;
      prevState = plotMarkLogo;

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
      }

      lcd.setCursor(cursorPosition, 1);

      if (millis() % 600 < 400) {  // Blink every 500 ms
        lcd.print(">");
      } else {
        lcd.print(" ");
      }

      if (button1.isPressed()) { //handles clicking options in text size setting
        lcd.clear();
        currentState = plotMarkLogo;
        prevState = MainMenu;
      }
    }
      break;
  }
}
#pragma endregion LOOP

//////////////////////////////////////////////////
              // FUNCTIONS  //
//////////////////////////////////////////////////
#pragma region FUNCTIONS
void line(int newx,int newy, bool drawing) { 
//this function is an implementation of bresenhams line algorithm
//this algorithm basically finds the slope between any two points, allowing us to figure out how many steps each motor should do to move smoothly to the target
//in order to do this, we give this function our next X (newx) and Y (newy) coordinates, and whether the pen should be up or down (drawing)

  if (drawing < 2) { //checks if we should be drawing and puts the pen up or down based on that.
      plot(drawing); // dashed: 0= don't draw / 1=draw / 2... = draw dashed with variable dash width
  } else {
      plot((stepCount / drawing) % 2);//can do dashed lines, but for now this isn't doing anything since we're only sending 0 or 1.
  }
  
  int i;
  long over= 0;
  
  long dx  = newx-xpos; //calculate the difference between where we are (xpos) and where we want to be (newx)
  long dy  = newy-ypos; //difference for Y
  int dirx = dx>0?-1:1; //this is called a ternary operator, it's basically saying: if dx is greater than 0, then dirx = -1, if dx is less than or equal to 0, dirx = 1.
  int diry = dy>0?1:-1; //this is called a ternary operator, it's basically saying: if dy is greater than 0, then diry = 1, if dy is less than or equal to 0, diry = -1.
  //the reason one of these ^ is inverted logic (1/-1) is due to the direction these motors rotate in the system.

  dx = abs(dx);  //normalize the dx/dy values so that they are positive.
  dy = abs(dy);  //abs() is taking the "absolute value" - basically it removes the negative sign from negative numbers

    //the following nested If statements check which change is greater, and use that to determine which coordinate (x or y) get's treated as the rise or the run in the slope calculation
    //we have to do this because technically bresenhams only works for the positive quandrant of the cartesian coordinate grid,
    // so we are just flipping the values around to get the line moving in the correct direction relative to it's current position (instead of just up an to the right)
  if (dx > dy) {
    over = dx / 2;
    for (i = 0; i < dx; i++) {  //for however much our current position differs from the target,
      xStepper.step(dirx);      //do a step in that direction (remember, dirx is always going to be either 1 or -1 from the ternary operator above)

      // Serial.print("Xsteps: ");
      // Serial.print(dirx);
      // Serial.print("  ");

      over += dy;
      if(over>=dx) {
        over -= dx;
        
        // Serial.print("Ysteps: ");
        // Serial.println(diry);

        yStepper.step(diry); 
      }
      //delay(1);
    }
  } else {
    over = dy/2;
    for(i=0; i<dy; i++) {
      yStepper.step(diry);
      // Serial.print("Ysteps: ");
      // Serial.print(diry);
      // Serial.print("  ");
      over += dx;
      if(over >= dy) {
        over -= dy;
        // Serial.print("Xsteps: ");
        // Serial.println(dirx);
        xStepper.step(dirx);
      }
      //delay(1);
    }
  }

  xpos = newx; //store positions
  ypos = newy; //store positions
}

void plot(boolean penOnPaper) {  //used to handle lifting or lowering the pen on to the tape
  if (penOnPaper) {              //if the pen is already up, put it down
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

void plotMR(){ //plots a simplified version of the mark rober logo, stored as coordinates in memory in the MRvector array at the top
  int MRscale = 15; //multiplied scale 
  Serial.println("MARK LOGO TIME");
  for(int i = 0; i < 22; i++){//for each point in the shape we want to draw (in this case 22 points) execute the following 
    //(we step from one point to the next by increasing i by 1 each for loop, so we access the coordinates stored at row i)

    int x_end = (MRvector[i][0])*MRscale; //get the X for the point we want to hit
    int y_end = (MRvector[i][1])*MRscale*2.75;// get it's Y
    int p_up = MRvector[i][2];//get whether the pen is up or down

    Serial.print("X_goal: ");
    Serial.print(x_end);
    Serial.print(" Y_goal: ");
    Serial.print(y_end);
    Serial.print(" Draw: ");
    Serial.println(MRvector[i][2]);
    line(x_end, y_end, p_up); //use our line function to head to that X and Y position, the third value is the pen up/down.
  }
  releaseMotors();
}
#pragma endregion FUNCTIONS

//////////////////////////////////////////////////
               //  END CODE  //
//////////////////////////////////////////////////