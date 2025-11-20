/* Hack Pack 008: Tank Plant. 
HACK: Snake Game.
Play the classic video game Snake on the LED matrix! Use the left and right bumpers as controls to make
the snake turn left or right. Steer the snake to eat the food on the playing field, making it grow longer.
See how big you can make the snake grow before crashing into itself! Once the game is over, long press
either one of the bumpers to start a new game.

NOTE: This is the first hack I wrote, and it was based on a really early version of the code for Tank Plant. 
You might notice a lot of things look different compared to the stock code. This is also a pretty quick and
dirty hack: I roughly tore into the state machine and added a few new states. This could be much more elegant,
but it works and is fun! 

There are 8 game modes you can play, made from the combination of 3 different settings. The settings are
world wrap mode, diagonal mode, and hard mode. In world wrap mode, when the snake hits the edge of the
screen, it wraps around to the other side. If that mode is off, the edge of the screen are walls that end
the game. With diagonal mode, the snake turns 45 degrees each time you hit a bumper instead of 90. 
And with hard mode turned on, the snake speeds up a little bit every time it eats food. A cool feature
of this hack is that it stores your high score for each of the 8 game modes in EEPROM so that you can try
to beat them later! Your high scores will persist across resets, and will even stay in EEPROM if you 
upload a different program to the microcontroller for a while. As long as other programs don't overwrite
the section of EEPROM the scores are stored in (which they probably won't), you'll be able to upload Snake
to this microcontroller again and pick up from right where you left off.

The default game mode is world wrap on, hard mode on, and diagonal mode off. To toggle hard mode, you have
to recompile the code. But to toggle world warp or hard mode, you can use jumper wires. Take the light sensors
out of their connectors on the LED matrix PCB. If you look at the connector on the LED matrix for the left
and right light sensors, you will see silkscreen labels for the pins. Connect pin L1 from the left light
sensor to ground with a jumper wire to turn off world wrap mode - now you have to avoid the walls! Connect
pin L2 from the right light sensor to ground to turn on diagonal mode. After making these changes, you have
to reset the microcontroller for them to take effect. 

For the curious, here's a bit more detail about how high scores are stored. The three switchable features 
can be combined to make 8 different game modes. The system stores the high score for each of these combinations 
in EEPROM. It does this by representing the game mode as a single unsigned 8 bit number. The first three bits 
in the uint8_t track whether or not world wrap, hard mode, and diagonal mode are engaged. The LSB represents 
world wrap, the second bit represents hard mode, and the third bit represents diagonal mode. For example:
   uint8_t currentGameMode = 0b011;   ---->  represents diagonal off, hard mode on, world wrap on
           currentGameMode = 0b111;   ---->  diagonal on, hard mode on, world wrap on
           currentGameMode = 0b110;   ---->  diagonal on, hard mode on, world wrap off
This currentGameMode number then serves as the index for accessing the correct byte of memory in EEPROM,
so it can read and write to the correct location based on the current game mode without needing to use
a switch statement or a bunch of if statements. If you come up with new game modes to add, there are 5 bits
left over for you to use in currentGameMode to keep track of them!

Possible additions to make to this hack:
- Victory dances: make the tank platform move and dance in response to how good of a game you played.
- Optimize: this was a quick and dirty hack. Lots of old original code from Tank Plant is left over, and
     I just used the tankState state machine system because it was already in place. You can tell that this
     hack is something I did early on too, because it's called tankState and not robotState. This code could
     be highly pruned back and optimized. I made this in a few hours and stopped after it was fun to play,
     so I'm sure there are many improvements to be made.
- EEPROM improvements: right now, the game always stores the high scores in the same location in EEPROM.
     It's apparently best practice to actually move them around to a new location each time you write them
     to EEPROM, because EEPROM has a limited number of times that each byte of memory can be written to
     before it's too damaged and breaks down. Moving it around spreads out this wear and tear evenly. Each
     byte can be written about 100,000 times, so I wasn't too worried about leaving it in the same location,
     since high scores won't be broken super frequently, and the bytes only get written to if the high score
     is broken. But this could still be good practice.
- Damage mode: Make it so that when the snake collides with itself, it takes damage and gets shorter, rather
     than just ending the game.
- Jump: Make it so that the player has a limited number of jumps they can use to cross over a portion of the 
     snake without colliding. Maybe you get more jumps as the snake eats more food, but they get used up, so
     you have to be strategic about when you use them. Maybe you could use a simultaneous press of both bumpers
     as the jump command. 
*/



/* GAME MODE DIRECTIVES:
// define WORLD_WRAP to make it so that the screen wraps around, top connected to bottom and left connected to right. No walls to run into.
// define HARD_MODE to make it so that the snake speeds up every time it eats food.
// define DIAGONAL_MODE to make it so that snake can move diagonally as well as horizontally and vertically. The left and right bumpers
//        still make the snake turn left and right, but now it turns in 45 degree increments instead of 90. 
*/

#define WORLD_WRAP  
#define HARD_MODE
// #define DIAGONAL_MODE


// HARDWARE GAME MODE SETTING:
// To make it so that you can set the game mode with jumper wires, uncomment the JUMPER_SET line below.

// #define JUMPER_SET

#include <Arduino.h>        // has to be included before declaring variables

// EXTRA PARAMETERS FOR SNAKE HACK
constexpr uint8_t SNAKE_STARTING_LEN = 2;   // how big the snake is when the game starts
constexpr uint8_t MAX_SNAKE_LEN = 144;      // maximum length of the snake that triggers the player win condition
constexpr uint16_t DEFAULT_MOVE_INTERVAL = 250;       // 250ms delay between snake moves is the standard time. smaller delay increases snake move speed.
constexpr uint16_t MIN_MOVE_INTERVAL = 50;            // minimum allowed delay between snake moves.
uint16_t snakeMoveIntSubtractor = 5;                  // subtract 5ms from snakeMoveInterval each time it eats food if hard mode is enabled


// Included libraries
#include "MotionControl.h"
#include <OneButton.h>
#include <elapsedMillis.h>
#include <Servo.h>
#include <MultiMap.h>
#include <Adafruit_IS31FL3731.h>
#include <MatrixFace.h>
#include <EEPROM.h>                 // used for storing high scores of all 8 game modes between power cycles

/////////////////////////////////////////////////////////////////////////////////////
// Motor controller definitions and similar things. I left a lot of the motion control
// systems in place here in case I want to add victory dances for the robot when the player
// plays a good game of snake. As yet, that isn't implemented, and all motion control stuff
// could be removed to make room in memory for other things.
/////////////////////////////////////////////////////////////////////////////////////

// motor control pins
constexpr int LEFT_SPEED_PIN = 6;
constexpr int LEFT_DIR_PIN = 7;
constexpr int RIGHT_SPEED_PIN = 5;
constexpr int RIGHT_DIR_PIN = 4;
constexpr int SERVO_PIN = 9;

// sensor pin definitions
// I recently learned that modern C++ style is to do const int instead of #define for pin values because it
// is better for type safety and debugging, and can allow more optimizations on the part of the compiler.
constexpr int LEFT_LIGHT_SENSOR_PIN = A3;
constexpr int MOISTURE_SENSOR_PIN = A2;
constexpr int RIGHT_LIGHT_SENSOR_PIN = A1;

// switch pins
constexpr int L_SWITCH = 3;  // left object detection switch
constexpr int R_SWITCH = 2;  // right side object detection switch


// Create an instance of the MotionControl class that will be used to drive the tank
MotionControl tank(LEFT_SPEED_PIN, LEFT_DIR_PIN, RIGHT_SPEED_PIN, RIGHT_DIR_PIN);

// Create an instance of Servo that controls the head/pot servo motor
Servo headServo;

// create instances of the bumper switch controllers
OneButton leftBumper;
OneButton rightBumper;

// state machine flags for indicating whether the bumper is activated. false == no object detected
bool leftBumperActived = false, rightBumperActived = false;  // single clicks for object detection
bool leftBumperLongPress = false, rightBumperLongPress = false;  // for holding state of long presses


// set up the LED matrix
Adafruit_IS31FL3731 matrix = Adafruit_IS31FL3731();
// create an instance of the class that manages displaying things on the LED matrix
Face face(matrix);
// constants for matrix width and height
constexpr uint8_t MATRIX_WIDTH = 16;
constexpr uint8_t MATRIX_HEIGHT = 9;

// setting up some enums and structs to track the state of the tank platform
// first, this enum will be used as a way to indicate how we want the tank to be moving.
// This is building up toward non-blocking (no delay() calls) control of the motion platform.
enum class TankMoveTypes {
  // the last four of these are used in the TankState struct for the driveType enum
  STRAIGHT,  // drive straight flag
  STOP,      // stop motors flag
  ROTATE,    // used to indicate rotate mode of turning (motors moving opposite directions)
  CURVE,      // used to indicate curve mode of turning (motors moving same direction different speeds)
  LEFT,
  RIGHT
};

// used for staging the main behavioral modes of the robot in the main loop.
// if you want to add a new behavior mode, name it here so that you can test for it in the main loop state machine
enum class BehaviorModes {
  PARK,                                 
  RESET,                                
  PLAYER_LOSES,                           
  SNAKE
};


// simplified version of this struct. already initialized upon creation.
struct TankStateContainer {
  BehaviorModes behaviorState;
  unsigned long retreatInterval;
  unsigned long retreatRotateInterval;
  elapsedMillis retreatTimer;
  elapsedMillis retreatRotateTimer;
  bool retreatInitiated;
  bool retreatRotateInitiated;

  // Constructor to initialize members. when new instance is created, it initializes to these values
  TankStateContainer() 
    : behaviorState(BehaviorModes::PARK), // the behavior mode the robot will be in when it powers on
      retreatInterval(1000),              // how long to retreat
      retreatRotateInterval(1000),        // if doing STRAIGHT retreat, how long to rotate away from object
      retreatTimer(0),                    // for tracking retreat moves- might remove
      retreatRotateTimer(0),              // for tracking retreat moves - might remove 
      retreatInitiated(false),            // for tracking retreat moves
      retreatRotateInitiated(false)       // for tracking retreat moves
  {}
};

// Create an instance of the struct
TankStateContainer tankState;             // initialized to the above values




/*
// Variables and things related to the snake game
*/

uint16_t snakeMoveInterval = DEFAULT_MOVE_INTERVAL;   // 250ms delay between snake moves by default. 

// struct for storing pixel coordinate values
struct Coordinate {
  int8_t x;        // x coordinate of a pixel on the matrix
  int8_t y;        // y coordinate of a pixel on the matrix
};

// struct for storing snake head motion vectors. This could just be another instance of Coordinate,
// but I like the clarity.
struct SnakeHeadDir {
  int8_t xDir;      // -1, 0, or 1 represents horizontal motion as left, none, right respectively
  int8_t yDir;      // -1, 0, or 1 represents vertical motion as up, none, down respectively
};
// instance of the snake direction vectors
SnakeHeadDir snakeVectors;

uint8_t snakeLength = SNAKE_STARTING_LEN;      // initialize the snake to 2 pixels long

// create an array that stores the coordinates of active snake body pixels
Coordinate snakeBody[MAX_SNAKE_LEN];

// create a variable for storing the coordinate of snake food
Coordinate snakeFood;


bool moveMadeThisFrame = false;     // used to prevent multiple moves per frame (without this it's possible to turn 180 degrees and run into self by fast double click)
bool gameOver = false;              // set to true if the snake collides with itself (or walls, if worldWrapMode == false)


#ifdef WORLD_WRAP
  bool worldWrapMode = true;          // if the snake goes off the edge of the matrix, it wraps to the other side. Toroidal world instead of 2d plane.
#else
  bool worldWrapMode = false;
#endif
#ifdef HARD_MODE
  bool enableHardMode = true;         // set to true to make the snake move faster each time it eats food.
#else
  bool enableHardMode = false;
#endif
#ifdef DIAGONAL_MODE
  bool diagonalMode = true;
#else
  bool diagonalMode = false;
#endif


// these are used for changing the direction of the snake in diagonal mode
const int8_t diagonalsVectorRotationArray[8] = {0, -1, -1, -1, 0, 1, 1, 1};   // rotate through this left or right to change direction of snake in diagonal mode
uint8_t diagonalsVectorIterator = 2;                                          // starts with leftward motion as default. 
constexpr uint8_t diagonalsVectorIteratorYOffset = 6;                         // used to define Y vectors as an offset of the rotation array


// create the binary number that represents the current game mode. Used for high score tracking.
// The LSB indicates if world wrap is enabled, the second bit indicates hard mode, and the third bit indicates diagonal mode.
// This lets us store game mode as a single number, which we'll use as the index for the array that stores the high scores
// for each game mode in EEPROM. This way we can pull out the high score just by looking it up in the array using currentGameMode as the index.
uint8_t currentGameMode = diagonalMode << 2 | enableHardMode << 1 | worldWrapMode;
constexpr uint8_t gameModeAddress = 0;
constexpr uint8_t highScoreArrayAddress = gameModeAddress + sizeof(currentGameMode);  // make sure we move over to a new location in EEPROM to store that
uint8_t storedHighScore = 0;        // later on we'll read this from EEPROM


// useful function prototypes
void checkBumpers();  // function prototype. This will allow us to call one function to check both front bumper buttons.
int convertHeadingToForwardBiasSpeed(int heading, bool forRightMotor = true);
int convertHeadingToServoAngle(int heading);                        
int convertServoAngleToHeading(int servoAngle);
int runServoAtSpeed(Servo &controlledServo, float speedVector);     // runs the servo CW or CCW at specified speed in deg/s, up to the limits of motion
void drawSnakeField();    // draws the snake and food objects on the LED matrix
void updateSnake();       // moves the snake according to button presses, calls collision checker, changes snake length, moves snake, etc
int modulus(int x, int y); // a replacement for the % (modulus) built in operator. this one doesn't return negative numbers.
bool collisionDetector();   // returns true if the snake collides with itself (or possibly a wall if worldWrapMode is off)
void generateNewGame();     // generate random snake and food positions, random snake vectors

elapsedMillis printTimer = 0;
elapsedSeconds stateTimerSec = 0;     // used for tracking time spent in various behavior states
elapsedSeconds faceTimerSec = 0;


// variables for tracking servo position and robot heading
int heading = 0;
int currentServoPos = 90;                                       // used for tracking the current servo position



//********************************************************************************************************
// setup
//********************************************************************************************************




void setup() {
  Serial.begin(115200);  
  Serial.print("Game mode: "); Serial.println(currentGameMode, BIN);
  // create a random seed by reading a floating analog pin
  randomSeed(analogRead(A0));

  // if we're using jumper wires to set game mode, change the flags and rebuild the currentGameMode number
  #ifdef JUMPER_SET
    pinMode(LEFT_LIGHT_SENSOR_PIN, INPUT_PULLUP);
    pinMode(RIGHT_LIGHT_SENSOR_PIN, INPUT_PULLUP);                   
    worldWrapMode = digitalRead(LEFT_LIGHT_SENSOR_PIN);
    diagonalMode = !digitalRead(RIGHT_LIGHT_SENSOR_PIN);        // default to not using diagonal mode (pull L2 to GND to active diagonal mode)
    currentGameMode = diagonalMode << 2 | enableHardMode << 1 | worldWrapMode;
  #endif

  leftBumper.setup(L_SWITCH, INPUT_PULLUP, true);     // use internal pull up resistor, active low switch
  rightBumper.setup(R_SWITCH, INPUT_PULLUP, true);
  leftBumper.setDebounceMs(20);
  rightBumper.setDebounceMs(20);

  // set up what happens with single clicks are detected
  leftBumper.attachClick([]() { leftBumperActived = true; }); // lambda function just sets the flag variable to true when single click detected
  rightBumper.attachClick([]() { rightBumperActived = true; }); // lambda/anonymous functions are nice because I don't need a named function I can reuse elsewhere

  // set up long clicks on the bumpers so I can start and stop motion
  leftBumper.attachLongPressStart([]() { leftBumperLongPress = true; });
  rightBumper.attachLongPressStart([]() { rightBumperLongPress = true; });

  headServo.attach(SERVO_PIN);
  headServo.write(convertHeadingToServoAngle(0));

  matrix.begin();
  matrix.setRotation(0);
  face.storeImagesInFrames();
  face.setFaceState(FaceStates::EYES_CONFUSED);
  face.updateFace();

  // generate a new game - randomize snake and food position and snake vectors
  generateNewGame();

  delay(500);
  
  // Write the name of the game we'll be playing on the LED matrix
  face.writeText("snake");
  while (face.isTextBeingWritten()) {
    face.updateFace();
  }
  delay(500);

  // If you need to reverse one of the motors, set the corresponding value to true
  tank.rightMotorReversed = false;                   
  tank.leftMotorReversed = false;
  
  tankState.behaviorState = BehaviorModes::PARK;  // starts in PARK mode just to ensure that motors are stopped, then hands off to SNAKE mode
}






//********************************************************************************************************
// loop
//********************************************************************************************************




void loop() {
  static bool snakeGameInProgress = true;       // if this is false we can display a face to show the win condition or something
  static bool debugPrint = false;

  // first, check the bumpers to see if one has been pressed
  checkBumpers();


  // update the LED matrix face if we're not playing snake
  if (!snakeGameInProgress) face.updateFace();

  // now we determine behavior based on the behavior state of the robot
  switch (tankState.behaviorState) {

    // Snake mode - where the logic of the game goes
    case BehaviorModes::SNAKE:
      if (gameOver) {
        tankState.behaviorState = BehaviorModes::PLAYER_LOSES;
        break;
      }
      updateSnake();
      drawSnakeField();
      // while (true);
      break;

    // park mode - don't drive, but look at light with head
    case BehaviorModes::PARK:
      // stop moving the tank platform. Motion of the head is still allowed but will be controlled by a different system. 
      tank.stop();
      if (snakeGameInProgress) tankState.behaviorState = BehaviorModes::SNAKE;
      break;


    // game over case
    case BehaviorModes::PLAYER_LOSES:
      delay(1000);
      matrix.setFrame(7);
      matrix.clear();
      face.writeText("game over");
      while (face.isTextBeingWritten()) {
        face.updateFace();
      }
      delay(500);
      // display the player's score
      matrix.clear();
      matrix.setCursor(3, 1);
      matrix.setTextSize(1);
      matrix.setTextColor(40);
      matrix.setTextWrap(false);
      matrix.print(snakeLength);
      matrix.displayFrame(7);
      delay(1000);

      // display the high score retrieved from EEPROM
      storedHighScore = EEPROM.read(highScoreArrayAddress + currentGameMode);
      if (storedHighScore == 255) storedHighScore = 0;     // bytes in EEPROM that have never been written to seem to be initialized to 255, so set this to 0 if that's the case
      if (storedHighScore < snakeLength) storedHighScore = snakeLength;     // if the player beat the high score, change the high score
      EEPROM.update(highScoreArrayAddress + currentGameMode, storedHighScore);    // this overwrites the stored high score if the value has changed
      matrix.setFrame(7);
      matrix.clear();
      face.writeText("high score: ");
      while (face.isTextBeingWritten()) {
        face.updateFace();
      }
      delay(500);
      matrix.clear();
      matrix.setCursor(3, 1);
      matrix.setTextSize(1);
      matrix.setTextColor(40);
      matrix.setTextWrap(false);
      matrix.print(storedHighScore);
      matrix.displayFrame(7);
      delay(1000);
      //
      matrix.clear();
      face.writeText("you: ");
      while (face.isTextBeingWritten()) {
        face.updateFace();
      }
      delay(1000);
      // go back to player's score
      matrix.clear();
      matrix.setCursor(3, 1);
      matrix.setTextSize(1);
      matrix.setTextColor(40);
      matrix.setTextWrap(false);
      matrix.print(snakeLength);
      matrix.displayFrame(7);
      delay(1000);
      debugPrint = true;

      // move into the reset state to be able to start new game
      tankState.behaviorState = BehaviorModes::RESET;
      break;

    
    // do a head shake and made face when the human player wins
    case BehaviorModes::RESET:
      // reset the important flags and variables
      leftBumperActived = false;
      rightBumperActived = false;
      leftBumperLongPress = false;
      rightBumperLongPress = false;
      checkBumpers();
      // hold indefinitely until reset. long press either bumper to start a new game.
      while(!leftBumperLongPress && !rightBumperLongPress) {    
        checkBumpers();
      }

      snakeLength = 2;
      gameOver = false;
      moveMadeThisFrame = false;
      snakeVectors.xDir = 0;
      snakeVectors.yDir = 0;

      matrix.setFrame(7);
      matrix.clear();
      matrix.displayFrame(7);
      generateNewGame();
      // transition to PARK state to make sure robot isn't moving. I might add dancing later so want to make sure that stops.
      tankState.behaviorState = BehaviorModes::PARK;
      break;
    


    // all other unhandled cases resolve here
    default:
      break;

  }
}







//********************************************************************************************************
// function definitions
//********************************************************************************************************




void updateSnake() {
  // check for collisions
  gameOver = collisionDetector();
  // gameOver = false;
  if (gameOver) return;   // break out of the updateSnake function because the game is lost

  // check to see if food has been eaten
  if ((snakeBody[0].x == snakeFood.x) && (snakeBody[0].y == snakeFood.y)) {
    // increase the length of the snake. this is all we have to do to properly grow the snake from the tail.
    snakeLength++;
    // generate new food location. This might be under the snake's body somewhere.
    snakeFood.x = random(16);
    snakeFood.y = random(8);
    // decrease the time between snake moves if hard mode is enabled
    if (enableHardMode) {
      snakeMoveInterval = max(MIN_MOVE_INTERVAL, snakeMoveInterval - snakeMoveIntSubtractor);  // constrains to a minimum of 50ms between moves
    }
  }

  // check the bumpers again just to have that fresh data
  checkBumpers();

  // change the direction of the snake based on button presses
  if (!diagonalMode) {                                            // standard snake mode, no diagonal moves
    if (rightBumperActived) {
      rightBumperActived = false;                                 // reset the button press flag
      if (!moveMadeThisFrame) {                                   // check to see if a move was already made this frame
        if (snakeVectors.xDir == 0) {                             // if snake currently moving up or down
            snakeVectors.xDir = (snakeVectors.yDir > 0) ? 1 : -1; // set xDir based on yDir
            snakeVectors.yDir = 0;                                // no longer moving vertically
        } else {                                                  // snake currently moving left or right
            snakeVectors.yDir = (snakeVectors.xDir > 0) ? -1 : 1; // set yDir based on xDir
            snakeVectors.xDir = 0;                                // no longer moving in horizontally
        }
        moveMadeThisFrame = true;                                 // set the flag to prevent a second move with a quick double tap during this frame
      }
    }
    if (leftBumperActived) {
      leftBumperActived = false;                                  // reset the button press flag
      if (!moveMadeThisFrame) {
        if (snakeVectors.xDir == 0) {                             // if snake currently moving up or down
            snakeVectors.xDir = (snakeVectors.yDir > 0) ? -1 : 1; // set xDir based on yDir
            snakeVectors.yDir = 0;                                // no longer moving vertically
        } else {                                                  // snake currently moving left or right
            snakeVectors.yDir = (snakeVectors.xDir > 0) ? 1 : -1; // set yDir based on xDir
            snakeVectors.xDir = 0;                                // no longer moving horizontally
        }
        moveMadeThisFrame = true;
      }
    }
  } else {        // diagonal snake mode enabled
    if (leftBumperActived) {
      leftBumperActived = false;
      if (!moveMadeThisFrame) {
        diagonalsVectorIterator = modulus(++diagonalsVectorIterator, 8);
        moveMadeThisFrame = true;
      }
    }
    if (rightBumperActived) {
      rightBumperActived = false;
      if (!moveMadeThisFrame) {
        diagonalsVectorIterator = modulus(--diagonalsVectorIterator, 8);
        moveMadeThisFrame = true;
      }
    }
  }

  // move the snake based on the vectors with the correct timing
  static uint16_t lastSnakeMoveTime = millis();
  uint16_t now = millis();
  if (now - lastSnakeMoveTime >= snakeMoveInterval) {
    // shift each body cell other than head over one space in the array
    for (uint8_t i = snakeLength; i >= 1; i--) {
      snakeBody[i].x = snakeBody[i - 1].x;
      snakeBody[i].y = snakeBody[i - 1].y;
    }
    // move the head based on the vectors
    if (!diagonalMode) {
      snakeBody[0].x = snakeBody[1].x + snakeVectors.xDir;
      snakeBody[0].y = snakeBody[1].y + snakeVectors.yDir;
    } else {
      snakeBody[0].x = snakeBody[1].x + diagonalsVectorRotationArray[diagonalsVectorIterator];
      snakeBody[0].y = snakeBody[1].y + diagonalsVectorRotationArray[modulus(diagonalsVectorIterator + diagonalsVectorIteratorYOffset, 8)];
    }
    // wrap around if we're playing on a toroidal world
    if (worldWrapMode) {    
      snakeBody[0].x = modulus(snakeBody[0].x, MATRIX_WIDTH);
      snakeBody[0].y = modulus(snakeBody[0].y, MATRIX_HEIGHT);
    }
    lastSnakeMoveTime = now;
    moveMadeThisFrame = false;  // reset this flag so that we can make a new move next frame
  }
}








// returns true if two cells of the snake are found to occupy the same coordinates
bool collisionDetector() {
  bool collisionFound = false;
  for (int i = 1; i < snakeLength; i++) {
    if ((snakeBody[0].x == snakeBody[i].x) && (snakeBody[0].y == snakeBody[i].y)) {
      collisionFound = true;
      break;
    }
  }
  if (!worldWrapMode) {   // walls are edges we can collide with
    collisionFound = (snakeBody[0].x < 0) || (snakeBody[0].x >= MATRIX_WIDTH);
    collisionFound |= (snakeBody[0].y < 0) || (snakeBody[0].y >= MATRIX_HEIGHT);
  }
  return collisionFound;
}


void drawSnakeField() {
  // be sure to use frame 7 as the scratch pad, the other frames hold bitmap faces
  matrix.setFrame(7);
  matrix.clear();
  uint8_t brightnessSubtractor = 60 / snakeLength;
  uint8_t brightness;
  for (uint8_t i = 0; i < snakeLength; i++) {
    if (i == 0) {
      brightness = 255;
    } else if (i == 1) {
      brightness = 64;
    } else {
      brightness -= brightnessSubtractor;
    }
    matrix.drawPixel(snakeBody[i].x, snakeBody[i].y, brightness);
  }

  // draw the food
  matrix.drawPixel(snakeFood.x, snakeFood.y, 48);

  matrix.displayFrame(7);
}


void generateNewGame() {
  // randomize the snake vectors to be -1, 0, or 1
  if (!diagonalMode) {
    snakeVectors.xDir = random(-1, 2);
    snakeVectors.yDir = random(-1, 2);
    bool invalidVectorCombo = !((snakeVectors.xDir == 0 && snakeVectors.yDir != 0) || (snakeVectors.xDir != 0 && snakeVectors.yDir == 0));
    if (invalidVectorCombo) {
      snakeVectors.xDir = 0;
      while (snakeVectors.yDir == 0) {
        snakeVectors.yDir = random(-1, 2);
      }
    }
    // randomize the starting position of the snake body. start in the middle rather than along edges
    snakeBody[0].x = random(2, MATRIX_WIDTH - 2);   // random x coord of snake head
    snakeBody[0].y = random(2, MATRIX_HEIGHT - 2);    // random y coord of snake head

    // now fill out the rest of the active snake body pixels relative to head and according to motion vectors
    for (uint8_t i = 1; i < snakeLength; i++) {
      if (snakeVectors.xDir == 0) {
        snakeBody[i].y = snakeBody[i - 1].y - snakeVectors.yDir;
        snakeBody[i].x = snakeBody[0].x;
      } else {
        snakeBody[i].x = snakeBody[i - 1].x - snakeVectors.xDir;
        snakeBody[i].y = snakeBody[0].y;
      }
    }
  } else {
    snakeVectors.xDir = diagonalsVectorRotationArray[diagonalsVectorIterator];
    snakeVectors.yDir = diagonalsVectorRotationArray[diagonalsVectorIterator + diagonalsVectorIteratorYOffset];
    // doing a fixed starting point for testing diagonal mode
    for (int i = 0; i < snakeLength; i++) {
      snakeBody[i].x = 6 + i;
      snakeBody[i].y = 4;
    }
  }

  
  // fill out the rest of the snake body array with unassigned pixel values
  for (uint8_t i = snakeLength; i < MAX_SNAKE_LEN; i++) {
    snakeBody[i].x = 127; // 127 indicates unassigned body pixel
    snakeBody[i].y = 127;
  }

  // randomize the position of the snake food:
  snakeFood.x = modulus((3 * random(-1, 2)) + snakeBody[0].x, MATRIX_WIDTH);
  snakeFood.y = modulus((3 * random(-1, 2)) + snakeBody[0].y, MATRIX_HEIGHT);

  // reset to the starting snakeMoveInterval
  snakeMoveInterval = DEFAULT_MOVE_INTERVAL;
}




/**
 * @brief Computes the modulus of two integers, ensuring the result is non-negative.
 *
 * This function is a replacement for the % operator that prevents negative results by wrapping 
 * negative values around to the positive range. It is mainly used for handling angular values 
 * when the gantry wraps from 360 degrees to 0 degrees.
 *
 * @param x The dividend.
 * @param y The divisor.
 * 
 * @return int The modulus result, always non-negative.
 */
int modulus(int x, int y) {
  return x < 0 ? ((x + 1) % y) + y - 1 : x % y;
}



/**
 * @brief A small wrapper function that just makes it easier to update the switch debouncers for both the left and right bumpers.
*/
void checkBumpers() {
  leftBumper.tick();
  rightBumper.tick();
}


/**
 * @brief Converts a heading from -90 to 90 to the proper values to drive the servo from 0 to 180 degrees.
 * Has to invert the range of values because the gears reverse the direction of torque from servo.
*/
int convertHeadingToServoAngle(int heading) {
  return map(heading + 90, 0, 180, 180, 0);         // using map to invert the range because the gears on servo and pot reverse the rotation direction
}


/**
 * @brief Converts a servo angle from 0 to 180 degrees to an range of -90 to 90 for headings.
 * Really just for clarity and convenience in code.
*/
int convertServoAngleToHeading(int servoAngle) {
  return map(servoAngle - 90, -90, 90, 90, -90);       
}





/**
 * @brief Moves the servo at the specified speed until the end of its range of motion is reached.
 * 
 * This function controls the rotational speed of a servo motor based on the given speed vector. 
 * The servo will continue to move at the specified speed until it reaches the end of its allowable range 
 * (0 to 180 degrees). The function internally manages the timing of the movement to achieve the desired speed.
 * 
 * @param controlledServo A reference to the Servo object that will be controlled.
 * @param speedVector A float specifying the rotational velocity in degrees per second. 
 *        The sign of the vector determines the direction of rotation: positive for clockwise, negative for counterclockwise.
 *        The speed is constrained to a maximum absolute value defined by `maxAllowedSpeed`.
 * @return int The current position of the servo in degrees (ranging from 0 to 180 degrees, obtained using `Servo::read()`).
 */
int runServoAtSpeed(Servo &controlledServo, float speedVector) {
  constexpr int maxAllowedSpeed = 90;               // speed limit for rotation in degrees per second
  constexpr int minServoPos = 0;
  constexpr int maxServoPos = 180;                  // change these limits if you're using a different range of motion servo (e.g., 270)
  static unsigned long lastMoveTime = 0;
  static float lastSpeedVector = 0.0;
  static unsigned long moveInterval = 0;
  int currentServoPos = controlledServo.read();           // store the current position of the servo
  
  if (speedVector != lastSpeedVector) {             // doing this so that we only perform this calculation when the speedVector changes
    speedVector = constrain(speedVector, -1 * maxAllowedSpeed, maxAllowedSpeed);    // constrain the speed to our defined maximum
    moveInterval = (long)(1000.0 / abs(speedVector));                                // convert to a time between each servo move. Servo moves in 1 degree steps
    lastSpeedVector = speedVector;                                                  // reset this value so we can track whether or not it changes
  }

  if (millis() - lastMoveTime >= moveInterval) {
    if (speedVector != 0) {
      int directionStep = (speedVector > 0) ? -1 : 1;     // sets direction to negative if speed vector is greater than 0, 1 if less than 0 (takes care of inversion caused by gears in robot)
      currentServoPos = constrain(currentServoPos + directionStep, minServoPos, maxServoPos);  // moves servo by 1 degree in the appropriate direction, up to the limits of the range of motion
      controlledServo.write(currentServoPos);   // move the servo
    } else {
      controlledServo.write(currentServoPos);             // rewrite the last position as a hold function
    }
    lastMoveTime = millis();
  }
  return currentServoPos;                           // return the current position of the servo
}




/**
 * @brief Takes in the heading of the robot and returns the speed the specified motor should be running at.
 * 
 * This is used for light seeking behavior. The heading is the direction the head is pointing, and the speed
 * of the motor is defined as a piecewise linear function. The function is designed to bias toward driving
 * forward as much as possible, with only heading values that are close to the end of the range causing the
 * motor to drive full speed or reverse directions. Change these functions to change the way the robot 
 * drives in response to its heading. This uses the multiMap library to make the piecewise functions and get
 * the proper return values for each motor speed.
 * 
 * @param heading An integer between [-90, 90] (full left to full right, 0 straight ahead) that indicates direction the head is looking.
 * @param forRightMotor Set to true to calculate speed for right motor, false for left motor
*/
int convertHeadingToForwardBiasSpeed(int heading, bool forRightMotor = true) {
  int outputSpeed = 0;
  int headingsIn[5] = {-90, -45, 0, 45, 90};        // independent variable for the piecewise functions
  int leftOutMap[5] = {-255, 0, 150, 192, 255};     // dependent variable for piecewise linear function for left motor (ex: when heading is 45, lMotor speed is 192)
  int rightOutMap[5] = {255, 192, 150, 0, -255};    // dependent variable for piecewise linear function for right motor speed (ex: when heading is 45, rMotor speed is 0)

  if (forRightMotor) {
    outputSpeed = multiMap<int>(heading, headingsIn, rightOutMap, 5);
  } else {
    outputSpeed = multiMap<int>(heading, headingsIn, leftOutMap, 5);
  }
  return outputSpeed;
}

