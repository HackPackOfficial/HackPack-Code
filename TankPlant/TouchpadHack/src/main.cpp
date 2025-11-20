/**
 * HACK: Pettable Plant.
 * 
 * This hack adds another capacitive sensor to the robot. Instead of measuring soil moisture, it is a capacitive touch sensor.
 * This lets you do things like pet the plant pot and have it react to your touch!
 * 
 * When the robot is driving around or parked, you can touch the capacitive touch pad on the side, and it will respond to your touch
 * by changing its face, turning its head to look at you, smiling, wiggling around a bit, and then settling down into a special FREEZE mode. 
 * Unlike the stock code, the robot will not wake itself up out of FREEZE mode after a set amount of time. Instead, you will have to
 * touch the pad again to wake the robot up out of FREEZE mode. If it parks itself normally after spinning in circles, then it will 
 * wake itself up after a set amount of time.
 * 
 * To make this hack work, you need to add a small amount of hardware. You need to add a patch of metal to the side of the plant pot.
 * Tape a square of aluminum foil onto the side of the plant pot, or adhere a patch of metal tape to the pot. Then, tape the exposed
 * copper of a single wire to this metal patch on the pot, so that the wire and the metal patch are electrically connected. Plug
 * the other end of the wire into pin A7 of the breadboard. This uses the same modified ADCTouch library that the stock code uses to
 * do capacitive sensing, so now this new button is connected to analog pin A7.
 * 
 * You may need to adjust the threshold value (TOUCH_THRESHOLD, defined in Configuration.h) that determines whether a measured capacitance 
 * value on the new touch pad counts as a touch or not. The length of your wires and the size of the metal patch can change the capacitance 
 * that you will measure. The code does an initial measurement and generates an offset, so it partially handles these variations on its own, 
 * but it's not perfect.
 * 
 * There are still two analog pins left open on the microcontroller, assuming that you are starting from the stock robot build when
 * creating this hack. Those are pins A0 and A6. Add more touch sensors if you want! The modified CL_ADCTouch library works with multiple 
 * sensors. Be aware that the original ADCTouch library does not, however. 
*/


#include <Arduino.h>

// Many of the basic behaviors of the robot can be changed with the parameters stored in this header file.
// Open it up to see what you can change!
#include <Configuration.h>

#include <CL_DRV8835.h>           // Provides an interface for using the DRV8835 motor driver to manuever the tank chassis
#include <OneButton.h>            // Handles button debouncing and interpretation
#include <elapsedMillis.h>        // A neat little wrapper library for timing functions
#include <SimpleMovingAverage.h>  // Creates moving average filter objects, used for filtering sensor readings and related tasks.
#include <Servo.h>                // Servo controller library
#include <MultiMap.h>             // Peforms piecewise linear mapping functions (courtesy of Rob Tillaart, the GOAT of Arduino libraries: https://github.com/RobTillaart/MultiMap)
#include <CL_ADCTouch.h>          // Performs capacitive sensing using a single ADC pin
#include <Adafruit_IS31FL3731.h>  // Provides the interface for controlling the LED matrix
#include <MatrixFace.h>           // Extra functions for drawing faces on the LED matrix


// The pin definitions are all in a separate file for neatness and clarity. If you want to change
// how the sensors and actuators are connected to the microcontroller, you can change that mapping
// in this file. 
#include <PinDefinitions.h>

#pragma endregion LIBRARIES AND CONFIGURATION


#pragma region Global Variables

/**
 * Hack element: add a struct for storing the values of the touch pads that get added to the pot. By default, there are three
 * analog pins left over on the microcontroller for adding touch pads, but you could add even more by using an external ADC
 * to read the light sensors, which would free up two more analog pins for this hack. Or you could completely change the way
 * this hack works and use an external capacitive touch sensor module that communicates over I2C. 
 * 
 * Note that the RIGHT_TOUCH_PAD_PIN and the TOUCH_THRESHOLD are defined in Configuration.h
*/

ADCTouchClass touchSensor(RIGHT_TOUCH_PAD_PIN);   // defined in Configuration.h

struct TouchPads {
  int16_t rightPadVal;      // stored and measured value
  int16_t rightPadOffset;   // offset reading used to determine when a pad is touched
  bool rightPadTouched;     // flag to set when touch detected

  // add more pads as you see fit and have analog pins available for. Note that when I tried to have two touch pads, they were 
  // capacitively coupled and interfered with each other's readings. Adding more may or may not work for you without tweaking
  // the hardware.

  // initialize values to zero and false. in setup(), we'll initialize the offset readings.
  TouchPads()
    : rightPadVal(0),
      rightPadOffset(0),
      rightPadTouched(false)
  {}
};

TouchPads touchPadStates;

void doTheDance();      // The function that sequences the dance the robot performs when you touch the pad


/////////////////////////////////////////


// First, create objects and variables related to the motion of the robot
CL_DRV8835 tank(LEFT_SPEED_PIN, LEFT_DIR_PIN, RIGHT_SPEED_PIN, RIGHT_DIR_PIN); // Instance of the CL_DRV8835 class that will drive the tank
Servo headServo;                                                                  // Create an instance of Servo that controls the head servo motor
OneButton leftBumper;                                                             // debouncing object for left bumper switch
OneButton rightBumper;                                                            // debouncing object for right bumper switch
int heading = 0;                                                                  // used to track the heading of the robot (the direction the face is pointed). units degrees, range -90 to 90.
int currentServoPos = 90;                                                         // used for tracking the current servo position. units in degrees, range 0 to 180.
int16_t leftMotorSpeed = 0, rightMotorSpeed = 0;                                  // signed integers that will take the range between -256 to 255 (negative values reverse motor direction)
uint16_t parkTime = 30;                                                           // seconds, will be randomized on new entry into the PARK state
uint16_t spinTime = 0, driveTime = 0, randomizeStep = 0;                          // durations for different actions in the RANDOMIZE_POSITION behavior
elapsedMillis randomizeSpinTimer = 0, randomizeDriveTimer = 0;                    // timers for the RANDOMIZE_POSITION actions

// flags related to the front bumpers
bool leftBumperActived = false, rightBumperActived = false;                       // single clicks for object detection
bool checkForCornerTraps = true;                                                  // used for triggering detectCornerTraps() when bumper is pressed
bool cornerTrapDetected = false;                                                  // gets set to true if corner trap is detected, causing behavior state change

// Next, create the objects that will run the LED matrix face
Adafruit_IS31FL3731 matrix = Adafruit_IS31FL3731();   // set up the LED matrix
Face face(matrix);    // create an instance of the class that draws faces on the LED matrix

// create a timer for tracking time spent in a behavioral state
elapsedSeconds stateTimerSec = 0; // units are seconds

// create variables related to the light sensors
bool lightSensorsCalibrated = false;
int averageLeftLightVal = 0, averageRightLightVal = 0;     // used during initial calibration of light sensors (MODE::CALIBRATE)
int leftLightSensorOffset = 0, rightLightSensorOffset = 0; // used for balancing readings between left/right light sensors

// create the object for managing the moisture sensor
ADCTouchClass moistureSensor(MOISTURE_SENSOR_PIN);


// setting up some structures (structs) and scoped enumerations (enum classes) to track the state of the tank platform.
// See [[Footnotes#Footnote 8: Structs, enums, and enum classes]]

// first, this enum class will be used as a way to indicate how we want the tank to be moving.
// This is building up toward non-blocking (no delay() calls) state machine control of the motion platform.
enum class TankMoveTypes {
  STRAIGHT, // drive straight
  STOP,     // stop motors
  ROTATE,   // used to indicate rotate mode of turning (motors moving opposite directions)
  CURVE,    // used to indicate curve mode of turning (motors moving same direction different speeds)
  LEFT,
  RIGHT
};

/**
 * HACK NOTE: Adding new behavior states here to engage in when one of the capacitive touch pads is touched
*/
// used for staging the main behavioral modes of the robot in the main loop.
// if you want to add a new behavior mode, name it here so that you can test for it in the main loop state machine
enum class BehaviorModes {
  PARK,              // used to disable driving and keep the tank in one place.
  SEEK,              // used to make the robot drive toward light sources
  CALIBRATE,         // currently unused
  TEST,              // currently unused
  RANDOMIZE_POSITION, // used for driving to a new randomized position in the room
  RIGHT_PAD_TOUCHED_FREEZE,   // transition to this when right touch pad is touched and you want the robot to stop driving
  RIGHT_PAD_TOUCHED_MOVE   // transition to this when right pad is touched and you want the robot to move again
};

// This struct stores and organizes the state information of the tank chassis.
struct RobotStateContainer {
  BehaviorModes behaviorState;            // current state
  BehaviorModes lastBehaviorState;        // last behavior state
  unsigned long retreatInterval;
  unsigned long retreatRotateInterval;
  elapsedMillis retreatTimer;
  elapsedMillis retreatRotateTimer;
  bool retreatInitiated;
  bool retreatRotateInitiated;

  // Constructor to initialize members. when new instance is created, it initializes to these values.
  RobotStateContainer()
      : behaviorState(BehaviorModes::CALIBRATE),             // the behavior mode the robot will be in when it powers on
        lastBehaviorState(BehaviorModes::CALIBRATE),
        retreatInterval(TANK_RETREAT_INTERVAL),              // how long to retreat. Configuration.h
        retreatRotateInterval(TANK_RETREAT_ROTATE_INTERVAL), // if doing STRAIGHT retreat, how long to rotate away from object. Configuration.h
        retreatTimer(0),                                     // for tracking retreat moves- might remove
        retreatRotateTimer(0),                               // for tracking retreat moves - might remove
        retreatInitiated(false),                             // for tracking retreat moves
        retreatRotateInitiated(false)                        // for tracking retreat moves
  {}
};

// Create an instance of the struct
RobotStateContainer robotState;             // initialized to the above values


// plant moisture and happiness states
enum class PlantStates {
  PARCHED,                                // beyond thirsty, fully dried out
  THIRSTY,                                // could use some water, not fully dried out
  SATISFIED,                              // enough water/light in here, not too much
  DROWNING,                               // way too much water!
  INSOLATE_ME,                            // give me sunlight (currently unused)
  SCORCHED                                // well that's too much sun (currently unused)
};

// where we'll keep track of the needs and happiness states of the plant itself
struct PlantStateContainer {
  PlantStates waterSatisfaction;          // is the plant parched, thirsty, sated, or drowning?
  PlantStates lightSatisfaction;          // currently unused
  int moistureLevel;

  // constructor to initialize the members. I think later I want to store and set these in EEPROM for persistence across resets
  PlantStateContainer()
    : waterSatisfaction(PlantStates::SATISFIED),
      lightSatisfaction(PlantStates::SATISFIED),
      moistureLevel(20)                          
  {}
};

// create an instance of the struct
PlantStateContainer plantState;



// More touch pad hack things

// This will be used to orchestrate the random dance that happens when the pad is touched
struct TouchPadBehaviorParams {
  TankMoveTypes currentMovement;
  elapsedMillis moveTimer;
  uint16_t moveDuration;

  TouchPadBehaviorParams() 
    : currentMovement(TankMoveTypes::ROTATE),
      moveTimer(0),
      moveDuration(random(0, 3000))
  {}
};

TouchPadBehaviorParams touchPadDanceParams;




#pragma endregion Global Variables

#pragma region Function Prototypes
// function prototypes
void checkBumpers();                                                                                             // Call to check the states of both bumpers
void readLightSensors(int *left, int *right);                                                                    // get the lastest filtered values of both light sensors
bool calibrateLightSensorsLR(int *leftAverage, int *rightAverage);               // calibrate both light sensors
int aimHeadAtLight();                                                                                            // point the head at the brightest light source
int convertHeadingToForwardBiasSpeed(int heading, bool forRightMotor = true);                                    // converts direction face is pointing into motor speeds to steer tank
int convertHeadingToServoAngle(int heading);                                                                     // convert headings [-90 to 90 degrees] to corresponding servo angle [0 to 180 degrees]
int convertServoAngleToHeading(int servoAngle);                                                                  // convert servo angle [0 to 180 degrees] to corresponding headings [-90 to 90 degrees]
int runServoAtSpeed(Servo &controlledServo, float speedVector);                                                  // runs the servo CW or CCW at specified speed in deg/s, up to the limits of motion
bool retreat(TankMoveTypes fromSide = TankMoveTypes::LEFT, TankMoveTypes retreatType = TankMoveTypes::STRAIGHT); // Used to retreat from objects impacting bumpers
bool spinningInCircles(int currentHeading);                                                                      // used to detect if the robot is spinning in circles in order to transition to PARK state
bool detectCornerTrap(bool *bumperTriggered);                                                                    // call every time a bumper is pressed to detect corner trap situation
int getMoisture();                                                                                               // returns the moisture sensor reading with offset compensation
void updatePlantState(PlantStateContainer *plantStatePtr, BehaviorModes currentTankBehavior, BehaviorModes lastTankBehavior);      // updates the plant state based on moisture and light levels
const char *getPlantStateString(PlantStates state);                                                              // converts plant states to strings for printing to the LED matrix
void printStateInformation();                                                                                    // Helper function to print which state the machine is in on serial monitor
// The following functions are the behaviors that get called by the state machine in loop().
// They don't need to be separate functions and could instead be inlined in the state machine, 
// but breaking them out helps with legibility and clearly understanding the state machine.
void seekBehavior();
void randomizePositionBehavior();
void parkBehavior();

#pragma endregion Function Prototypes

#pragma region SETUP
//********************************************************************************************************
// setup
//********************************************************************************************************


void setup() {
  // start the serial monitor, if we're using serial. 
  // Enable or disable Serial monitoring and printing in Configuration.h
  SERIAL_BEGIN(115200);


  // initialize pseudorandom number generator with a floating analog pin voltage reading
  randomSeed(analogRead(A7));  // See [[Footnotes# Footnote 9: Pseudorandom Number Generators and `randomSeed()`]]


  // initialize the bumper objects
  leftBumper.setup(L_SWITCH, INPUT_PULLUP, true);     // use internal pull up resistor, active low switch
  rightBumper.setup(R_SWITCH, INPUT_PULLUP, true);
  leftBumper.setDebounceMs(20);                       // change debounce interval to 20ms rather than default of 50ms
  rightBumper.setDebounceMs(20);

  // setting up the bumper pins as interrupts. For now I like this idea because I can catch switch presses without polling.
  // I'm still going to poll just in case a situation arises where an interrupt somehow doesn't trigger.
  // I may also regret this later, since interrupts can lead to weird problems, but it's easy enough to remove and just poll.
  attachInterrupt(digitalPinToInterrupt(L_SWITCH), checkBumpers, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_SWITCH), checkBumpers, CHANGE);

  // set up what happens with single clicks are detected.
  // lambda/anonymous functions are nice because I don't need a named function I can reuse elsewhere.
  // the lambda function is indicated by the []() {......} part
  leftBumper.attachPress([]() { leftBumperActived = true; checkForCornerTraps = true; }); // lambda function just sets the flag variable to true when single click detected
  rightBumper.attachPress([]() { rightBumperActived = true; checkForCornerTraps = true; }); 

  // setup the servo controller
  headServo.attach(SERVO_PIN);
  headServo.write(convertHeadingToServoAngle(0) - SERVO_TRIM);

  // set up the LED matrix and face control systems.
  // [[Footnotes#Footnote 4: How the LED matrix control system works]]
  matrix.begin();               // starts the LED matrix controller class
  matrix.setRotation(0);        // sets the rotation of the matrix
  face.storeImagesInFrames();   // See [[Footnotes#Footnote 10: The LED Matrix Controller IC]]
  face.setFaceState(FaceStates::EYES_CONFUSED);   // set which face will be drawn first
  face.updateFace();                            // draw the face


  // cheeky little delay so you see the default face for a second while it wakes up
  delay(1000);

  // If you need to reverse one of the motors, set the corresponding value to true, or change the corresponding #define in Configuration.h
  tank.rightMotorReversed = REVERSE_RIGHT_MOTOR;                   
  tank.leftMotorReversed = REVERSE_LEFT_MOTOR;

  // set up the parking brake pin. Connecting this pin (PARKING_BRAKE_PIN, in PinDefinitons.h) to ground will disable the tank tread motors.


  // calibrate the light sensors during setup. 
  // spin in a circle to get average readings of the left and right sensors
  while (!lightSensorsCalibrated) {      
    lightSensorsCalibrated = calibrateLightSensorsLR(&averageLeftLightVal, &averageRightLightVal);
  }        
  // we've got the averages, now use those to set the offsets for the left and right light sensors
  if (averageLeftLightVal > averageRightLightVal) {
    rightLightSensorOffset = averageLeftLightVal - averageRightLightVal;  // add the difference between left and right to the right sensor to balance them out
  } else {
    leftLightSensorOffset = averageRightLightVal - averageLeftLightVal;   // add the difference between left and right to the left sensor to balance them out
  }

  // Now initialize the moisture level reading. Using noInterrupts for this block because there's a chance that interrupts
  // could mess with the sequence of operations that ADCTouch is performing behind the scenes to do single pin capacitance sensing.
  noInterrupts();
  plantState.moistureLevel = moistureSensor.read() - MOISTURE_OFFSET;    // PinDefintions.h and Configuration.h

  // also initialize the touch pad offset values (we need to figure out what number they float around when nothing is touching them so we can subtract that
  // from later measurements to detect touches)
  touchPadStates.rightPadOffset = touchSensor.read();

  interrupts();

  // Finally, put the robot into the mode you want it to start out in (usually SEEK to look for light).
  robotState.behaviorState = BehaviorModes::SEEK;
}
#pragma endregion SETUP

#pragma region LOOP

//********************************************************************************************************
// loop
//********************************************************************************************************


void loop() {
  // IMPORTANT NOTE: This code relies on everything in loop() being as non-blocking as possible. That was true of the
  // stock code, but it's even more important now that the touch pad hack is being used. The use of delay() calls
  // broke the execution of my code in ways I didn't expect in my testing with this hack.

  // Now, there are a few functions that need to be called on every iteration of the loop before we move into
  // running the state machine.
  // First, check the bumpers to see if we need to perform a retreat move:
  checkBumpers();
  
  /**
   * HACK notes: update the touch pad states now
   */

  static uint16_t lastTouchTime = 0, lastPrintUpdate = 0;

  touchPadStates.rightPadVal = touchSensor.readNext() - touchPadStates.rightPadOffset;
  touchPadStates.rightPadTouched = touchPadStates.rightPadVal > TOUCH_THRESHOLD ? true : false;

  // time tracking to make sure we don't rapidly retrigger the touch pad while touching it.
  // If you don't include the timing, the button gets read as pressed a bunch of times while you are
  // touching it, making the robot switch states rapidly (it's kind of cool, it's so fast that it
  // looks like it's drawing two faces on the matrix simultaneously).
  if (touchPadStates.rightPadTouched && (millis() - lastTouchTime >= 1000)) {
    // SERIAL_TABS(3);                                    // print 3 tabs to make the following more visible
    // SERIAL_PRINTLN(touchPadStates.rightPadVal);        // useful for tuning touch threshold value
    if (robotState.behaviorState == BehaviorModes::RIGHT_PAD_TOUCHED_FREEZE) {
      robotState.behaviorState = BehaviorModes::SEEK;
    } else if (robotState.behaviorState != BehaviorModes::RIGHT_PAD_TOUCHED_MOVE) {
      robotState.behaviorState = BehaviorModes::RIGHT_PAD_TOUCHED_MOVE;
    } else {
      robotState.behaviorState = BehaviorModes::SEEK;
    }

    touchPadStates.rightPadTouched = false;
    lastTouchTime = millis();
  }


  // Next, update the plant state (in terms of water, light, etc):
  updatePlantState(&plantState, robotState.behaviorState, robotState.lastBehaviorState);
  // Finally, update the LED matrix face:
  face.updateFace();

  // This prints out the state that the state machine is in to the serial monitor. Useful for debugging. Uncomment to use.
  // if (millis() - lastPrintUpdate > 250) {
  //   printStateInformation();
  //   lastPrintUpdate = millis();
  // }
  
  #pragma region STATE MACHINE
  // Run the state machine.
  // Now we determine behavior based on the behavior state of the robot
  switch (robotState.behaviorState) {

    // This behavior is basically sleep mode. After it does a dance from the touch pad being touched, it settles into this mode
    // until the touch pad is touched again.
    case BehaviorModes::RIGHT_PAD_TOUCHED_FREEZE:
      tank.stop();
      heading = 0;      // face the head forward
      currentServoPos = convertHeadingToServoAngle(heading + SERVO_TRIM);   // Configuration.h - SERVO_TRIM helps point head straight forward
      headServo.write(currentServoPos);
      face.setFaceState(FaceStates::EYES_SQUINT);
      break;
  

    // The robot does a dance when the touch pad is touched before settling down to sleep
    case BehaviorModes::RIGHT_PAD_TOUCHED_MOVE:
      doTheDance();
      break;


    // originally this was used for calibrating the light sensors, but now that calibration happens 
    // in setup(), this case is empty and can be used for other things or even renamed or removed.
    case BehaviorModes::CALIBRATE:
      break;

      
    // test mode - convenient for testing new features
    case BehaviorModes::TEST:
      tank.stop();
      face.setFaceState(FaceStates::SMILING_FACE);
      break;


    // park mode - don't drive, but keep turning head toward light
    case BehaviorModes::PARK:
      parkBehavior();
      break;


    // light seeking mode
    case BehaviorModes::SEEK:
      seekBehavior();
      break;


    // Spins a random amount and drives a random distance in that direction. Useful for starting from a new position
    // after transitioning out of PARK state. Otherwise if the robot was parked, it will often just spin in place
    // and park in the same spot. Change relevant parameters in Configuration.h to change the way it explores.
    // Also called to get out of corner traps.
    case BehaviorModes::RANDOMIZE_POSITION:
      randomizePositionBehavior();
      break;
    

    // the default case
    default:
      break;
  }

}

#pragma endregion STATE MACHINE
#pragma endregion LOOP




#pragma region FUNCTION DEFINITIONS
//********************************************************************************************************
// function definitions
//********************************************************************************************************

/**
 * @brief A small wrapper function that just makes it easier to update the switch debouncers for both the left and right bumpers.
*/
void checkBumpers() {
  leftBumper.tick();
  rightBumper.tick();
}


/**
 * @brief This function handles making retreat moves from objects detected by the bumpers on the front.
 * 
 * Two types of retreat move are possible: straight then rotate, and curve.
 * 
 * @param fromSide which side to retreat from (LEFT or RIGHT)
 * @param retreatType the type of retreat to make (STRAIGHT or CURVE)
 * @return returns true as long as a retreat move is in progress, returns false when the move is complete (basically,
 * if retreating, the return is true)
*/
bool retreat(TankMoveTypes fromSide, TankMoveTypes retreatType) {
  // Note that these variables rely on the keyword static and are defined in the function instead of as a global.
  // See [[Footnotes#Footnote 6: The static keyword and locality of reference]]
  static int step = 0;
  static bool newRetreatMove = true;
  static unsigned long start = 0;

  if (newRetreatMove) {
    start = millis();
    newRetreatMove = false;
  }

  switch (retreatType) 
  {
  case TankMoveTypes::STRAIGHT:                      // straight back then rotate in place type retreat
    switch (step) {
      case 0:
        if (millis() - start <= robotState.retreatInterval){
          tank.direct(-200, -200);    // reverse motors
        } else {
          start = millis();
          step++;
        }
        break;
      case 1:
        if (millis() - start <= robotState.retreatRotateInterval) {
          if (fromSide == TankMoveTypes::LEFT) {                 // rotate away from left bumper impact
            tank.rotate('R', 200);     // rotate right
          } else {
            tank.rotate('L', 200);     // rotate left
          }
        } else {
          step = 0;
          newRetreatMove = true;    // retreat is finished, so reset this flag so a new retreat move will start next time the function is called
          return false;             // because the retreat move is complete, we can return false here, stopping the rest of the function.
        }
        break;

      default:
        step = 0;
        newRetreatMove = true;
        return true;
        break;
    }
    break;

  case TankMoveTypes::CURVE: 
    // deal with a curve retreat. Simpler than straight then rotate retreat because it's one step
    if (millis() - start <= robotState.retreatInterval){
      if (fromSide == TankMoveTypes::LEFT) {
        tank.direct(-80, -255);    // simultaneously reverse and turn away from whatever hit the left bumper
      } else {
        tank.direct(-255, -80);    // reverse and turn away from whatever hit the right bumper
      }
    } else {
      newRetreatMove = true;     // retreat is finished, so reset this flag so a new retreat move will start next time the function is called
      return false;              // because the retreat move is complete, we can return false here, stopping the rest of the function.
    }       
    break;

  default:
    break;
  }

  // the only way that this should be reached is if we're still performing a retreat move.
  // so basically, this function always returns true, indicating that a retreat move is in progress. 
  // This keeps the leftBumperActivated flag set to true, which should lead straight back into the retreat
  // function again. Only when the retreat move is finished do we return false, which resets the 
  // leftBumperActivated flag to false, keeping us out of the retreat block and enabling other kinds of motion.
  return true;    
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
 * @brief Reads the values from two light sensors.
 * 
 * This function reads the analog values from two light sensors connected to the 
 * Arduino's analog input pins. The readings are stored in the variables pointed 
 * to by the `left` and `right` pointers. A small delay is introduced between 
 * the two `analogRead` operations to allow the ADC to settle after switching 
 * channels, ensuring accurate readings.
 * 
 * Also runs the readings through a simple moving average filter before returning. 
 * 
 * @param left Pointer to an integer where the left light sensor value will be stored.
 * @param right Pointer to an integer where the right light sensor value will be stored.
 */
void readLightSensors(int *left, int *right) {
  static SimpleMovingAverage lFilter(15);         // change the filter sample size to change the phase delay and the degree of smoothing (larger number -> smoother but slower response)
  static SimpleMovingAverage rFilter(15);
  static int filteredL = 0, filteredR = 0;
  filteredL = lFilter.filter(constrain(analogRead(LEFT_LIGHT_SENSOR_PIN) + leftLightSensorOffset, 0, 1023));
  *left = filteredL;
  delayMicroseconds(10);    // Small delay to allow the ADC to settle
  filteredR = rFilter.filter(constrain(analogRead(RIGHT_LIGHT_SENSOR_PIN) + rightLightSensorOffset, 0, 1023));
  *right = filteredR;
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
  constexpr int maxAllowedSpeed = MAX_HEAD_SERVO_SPEED;               // speed limit for rotation in degrees per second
  constexpr int minServoPos = 0;
  constexpr int maxServoPos = 180;                  // change these limits if you're using a different range of motion servo (e.g., 270)
  static unsigned long lastMoveTime = 0;
  static float lastSpeedVector = 0.0;
  static unsigned long moveInterval = 0;
  int currentServoPos = controlledServo.read();           // store the current position of the servo
  

  // See if the speed vector has been changed. Floating point operations are slow on the ATMega328P microcontroller,
  // so we want to minimize how frequently we have to perform them. Note that speedVector and lastSpeedVector are floats, 
  // which means that you need to [Footnotes#Footnote 5: Be careful about how you compare floating point numbers] .
  if (speedVector != lastSpeedVector) {                                           // doing this so that we only perform this calculation when the speedVector changes
    speedVector = constrain(speedVector, -1 * maxAllowedSpeed, maxAllowedSpeed);  // constrain the speed to our defined maximum
    moveInterval = (long)(1000.0 / abs(speedVector));                             // convert to a time between each servo move. Servo moves in 1 degree steps
    lastSpeedVector = speedVector;                                                // reset this value so we can track whether or not it changes
  }

  // move servo to a new position if it's time to do so
  if (millis() - lastMoveTime >= moveInterval) {
    if (speedVector != 0) {
      int directionStep = (speedVector > 0) ? -1 : 1;     // sets direction to negative if speed vector is greater than 0, 1 if less than 0 (takes care of inversion caused by gears train)
      currentServoPos = constrain(currentServoPos + directionStep, minServoPos, maxServoPos);  // moves servo by 1 degree in the appropriate direction, up to the limits of the range of motion
      controlledServo.write(currentServoPos);             // move the servo
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
int convertHeadingToForwardBiasSpeed(int heading, bool forRightMotor) {
  int outputSpeed = 0;
  if (forRightMotor) {
    outputSpeed = multiMap<int16_t>(heading, HEADINGS_IN_MAP, RIGHT_SPEED_MAP, 5);      // maps can be found and changed in RobotBehaviorVariables.h
  } else {
    outputSpeed = multiMap<int16_t>(heading, HEADINGS_IN_MAP, LEFT_SPEED_MAP, 5);       // maps can be found and changed in RobotBehaviorVariables.h
  }
  return outputSpeed;
}



/**
 * @brief Spins robot in a circle and calculates the average reading from the left and right light sensors.
 * Use to calculate the offsets to balance out the light sensors. Also tries to orient roughly toward brightest light
 * after calibration. 
 * 
 * @param leftAverage A pointer to an integer where the average brightness will be stored. 
 * @param rightAverage A pointer to where the integer value of the average right brightness will be stored.
 * @return Returns true when calibration is finished
*/
bool calibrateLightSensorsLR(int *leftAverage, int *rightAverage) {
    static int filteredLeftAvg = 0, filteredRightAvg = 0, leftRaw = 0, rightRaw = 0;
    static bool newCalibration = true;
    static unsigned long startTime = 0;
    static int maxSeen = 0, minSeen = 1023;

    // Dynamically allocate the filters. This means that we can decide how long memory is allocated
    // to these filters, and can delete them when we don't need them anymore.
    static SimpleMovingAverage* leftFilter = nullptr;
    static SimpleMovingAverage* rightFilter = nullptr;

    // If we do need to set up a new calibration and use the filters, this allocates memory for the filters.
    // This is a fun new thing for Hack Pack - dynamic memory management! This probably isn't necessary in this application,
    // but I wanted to play with it. We're playing with fire here - misuse of the new and delete keywords can really
    // mess programs up. But it's good to learn sometime. 
    constexpr uint8_t filterSize = 50;
    if (newCalibration) {
        if (leftFilter == nullptr && rightFilter == nullptr) {
            leftFilter = new SimpleMovingAverage(filterSize);
            rightFilter = new SimpleMovingAverage(filterSize);
        }
        // experimental: prefill the filters before spinning
        for (int i = 0; i < filterSize; i++) {
          leftRaw = analogRead(LEFT_LIGHT_SENSOR_PIN);
          delayMicroseconds(10);                // settling time for ADC
          rightRaw = analogRead(RIGHT_LIGHT_SENSOR_PIN);
          delayMicroseconds(10);
          leftFilter->filter(leftRaw);
          rightFilter->filter(rightRaw);
        }

        startTime = millis();
        newCalibration = false;
    }

    leftRaw = analogRead(LEFT_LIGHT_SENSOR_PIN);
    delayMicroseconds(10);                // settling time for ADC
    rightRaw = analogRead(RIGHT_LIGHT_SENSOR_PIN);
    filteredLeftAvg = leftFilter->filter(leftRaw);
    filteredRightAvg = rightFilter->filter(rightRaw);

    if (millis() - startTime <= LIGHT_CAL_SPIN_TIME) {    // spin in a circle. Left for calibrating sensors.

      tank.rotate('l', LIGHT_CAL_SPIN_SPEED);                // rotate left
      

      if (leftRaw < minSeen || rightRaw < minSeen) minSeen = min(leftRaw, rightRaw);
      if (leftRaw > maxSeen || rightRaw > maxSeen) maxSeen = max(leftRaw, rightRaw);
    } else {                                // finished spinning, so reset and return true
      tank.stop();
      *leftAverage = filteredLeftAvg;
      *rightAverage = filteredRightAvg;
      // The calibration is finished, so now we can delete the filters and deallocate the memory that
      // was saved for them. This should free up a fair amount of RAM for other processes in the code.
      delete leftFilter;
      delete rightFilter;
      leftFilter = nullptr;
      rightFilter = nullptr;     
      newCalibration = true;    // we can set this to true to allow for a new calibration if needed 
      return true;              // returning true indicates calibration is finished
    }
    return false;
}




/**
 * @brief Aims the head at the brightest source of light that is in view.
 * 
 * This works by running the servo at a speed that is defined by the proportion of the difference between the left
 * and right light level sensor readings. For some reason I was having more success using a PID (actually just P)
 * controller operating on servo speed, rather than using a similar controller adjusting the servo position. 
 * 
 * @return Returns the current servo position. 
*/
int aimHeadAtLight() {
  static int tempL = 0, tempR = 0;
  static unsigned long lastServoUpdateTime = 0;
  static float servoSpeed;
  readLightSensors(&tempL, &tempR);                 // read the light sensors (pass address of variable to store it in as argument)

  if(millis() - lastServoUpdateTime >= 10) {        // update the proportional servo speed controller every 10ms
    servoSpeed = constrain((tempR - tempL) * HEAD_SERVO_KP, -MAX_HEAD_SERVO_SPEED, MAX_HEAD_SERVO_SPEED);
    lastServoUpdateTime = millis();
  }
  return runServoAtSpeed(headServo, servoSpeed);    // returns current servo position
}




/**
 * @brief Used to detect if the robot is spinning in circles to enable transition to park state.
 * The robot tends to spin in left or right circles when it's under an even source of lighting. This function
 * tracks the heading and averages it over 100 samples spread across 10 seconds. When the average heading 
 * exceeds the threshold of +- parkingTheshold, the function returns true, indicating that it's spinning 
 * in place. Basically this determines if it's spinning hard enough to the left or right over a 10 second
 * window to push it across a threshold where it should just park and stop moving for a bit. 
 * @param currentHeading the heading (range: -90 to 90) of the robot (angle of head).
 * @param reset Reset average heading to 0 (needed for state transitions).
 * @return boolean. Returns false if not spinning in circles. Returns true if threshold exceeded.
*/
bool spinningInCircles(int currentHeading) {
  constexpr int headingSampleWindow = HEADING_SAMPLE_WINDOW;                      // track heading over period of 8 seconds.   Configuration.h
  constexpr int headingFilterBufferSize = HEADING_FILTER_SIZE;                    // take 80 samples over those 8 seconds.  Configuration.h
  static SimpleMovingAverage headingFilter(headingFilterBufferSize);              // set up the filter
  static int filteredHeading = 0;
  const int headingSamplePeriod = headingSampleWindow / headingFilterBufferSize;  // sample every 10ms
  static elapsedMillis headingMonitorTimer = 0;                                   // set up the timer object to use for sampling
  constexpr int parkingThreshold = PARKING_THRESHOLD_ANGLE;   // if average heading is above this value, return true to indicate we're spinning. Configuration.h
  if (headingMonitorTimer >= headingSamplePeriod) {
    filteredHeading = headingFilter.filter(currentHeading);                       // add new value to heading filter
    headingMonitorTimer = 0;
  }
  if (filteredHeading >= parkingThreshold || filteredHeading <= -1 * parkingThreshold) {
    headingFilter.reset();                              // reset the filter variables. we detected spinning, and need the filter to reset to detect the next spin
    filteredHeading = 0;
    return true;                                        // this indicates that we're spinning in circles to a point where it's worth parking the robot
  }
  return false;                                         // we're not spinning enough to park the robot
}




/**
 * @brief A wrapper function for ADCTouch that measures capacitance as a proxy for moisture levels.
 * 
 * @paragraph ADCTouch is cool because it uses internal circuitry on the ATMega328P to do capacitive sensing on a single analog pin.
 * It does use blocking code however, and in my testing, reading 20 samples for filtering takes 4.5ms. Doubling to 40
 * samples stabilizes the output value more effectively, but takes a full 9ms. I have considered rewriting the code of the
 * library to be non-blocking, but now that I'm only doing soil moisture and not capacitive touch buttons, I think I can just
 * only occassionally measure the soil moisture meter and take the blocking delay as a tradeoff.
 * 
 * Note that the range of values here isn't great. Fully saturated LECA gets a value back of maybe 28, and air reads 0.
 * It is kind of variable too, but I think this will work well enough for indicating if the plant is wet or dry. There is also
 * settling time. This sensor can't detect immediate changes to the moisture of the LECA. It seems to take several minutes to
 * fully register the change from watering. But that's still fine for now.
 * 
 * @return Offset compensated capacitive sensor reading. Integer. Constrained to 0 to 1023-offset.
*/
int getMoisture() {
  // automatically subtracts the moistureOffset.
  // there's a chance that interrupts could interfere with the ADC reading, so they are briefly disabled
  noInterrupts();
  int reading = moistureSensor.readNext() - MOISTURE_OFFSET; // PinDefintions.h and Configuration.h
  interrupts();
  return constrain(reading, 0, 1023 - MOISTURE_OFFSET); // Configuration.h
}




/**
 * @brief updates the plant states in terms of moisture and light, and writes status updates to the LED matrix if
 * the robot is in the correct behavior state.
 * 
 * @param plantStatePtr - pointer to the PlantStateContainer struct that is storing all the state data. 
 * @param currentTankBehavior - e.g., robotState.currentBehaviorState. Used for determining if it's time to write text to LED matrix.
 * @param lastTankBehavior - the previous behavior state of the robot
 */
void updatePlantState(PlantStateContainer *plantStatePtr, BehaviorModes currentTankBehavior, BehaviorModes lastTankBehavior) {
  // [[Footnotes#Footnote 3: Notes about measure water levels]]

  // the threshold values that determine the water satisfaction level of the plant are store in Configuration.h.
  // You can change those values to change how the plant determines itself to be parched, thirsty, satisfied, or drowning.
  constexpr int printFrequency = 10; // print to matrix every 10 seconds

  static elapsedSeconds moistureSampleTimer = 0;
  static elapsedSeconds screenPrintTimer = 0;

  if (moistureSampleTimer >= MOISTURE_SAMPLING_PERIOD) {              // Configuration.h
    plantStatePtr->moistureLevel = getMoisture();
    if (plantStatePtr->moistureLevel <= PARCHED_UPPER_THRESHOLD) {    // Configuration.h
      plantStatePtr->waterSatisfaction = PlantStates::PARCHED;
    } else if (plantStatePtr->moistureLevel <= THIRSTY_UPPER_THRESHOLD && plantStatePtr->moistureLevel > PARCHED_UPPER_THRESHOLD) {     // Configuration.h
      plantStatePtr->waterSatisfaction = PlantStates::THIRSTY;
    } else if (plantStatePtr->moistureLevel > THIRSTY_UPPER_THRESHOLD && plantStatePtr->moistureLevel <= SATISFIED_UPPER_THRESHOLD) {   // Configuration.h
      plantStatePtr->waterSatisfaction = PlantStates::SATISFIED;
    } else {
      plantStatePtr->waterSatisfaction = PlantStates::DROWNING;
    }
    moistureSampleTimer = 0;
  }

  // create a flag to see if we're in the correct mode for printing to the screen.
  // basically, if the current mode is either PARK or TEST, and the previous mode was neither PARK nor TEST,
  // then we are in the proper mode for printing to the screen. This detects a new entry into the PARK (or TEST) state,
  // at which point it sets up the text to be written to the LED matrix and starts the timer that determines when
  // the text will be written. You only want this happening on a new entry to the PARK state, not every time we enter
  // the PARK state. 
  bool properMode = (currentTankBehavior == BehaviorModes::PARK || currentTankBehavior == BehaviorModes::TEST);
  bool properPreviousMode = lastTankBehavior != BehaviorModes::PARK && lastTankBehavior != BehaviorModes::TEST;
  // check to see if this is a new transition into PARK mode, in which case we can start writing to screen
  if (properMode && properPreviousMode) {
    screenPrintTimer = 0;   // start the timer so that the first print occurs after 10 seconds
  }

  if (screenPrintTimer >= printFrequency && properMode) {
    face.writeText(getPlantStateString(plantStatePtr->waterSatisfaction));
    face.setFaceState(FaceStates::TEXT);
    SERIAL_PRINT("Moisture level: ");
    SERIAL_PRINTLN(plantStatePtr->moistureLevel);
    screenPrintTimer = 0;
  }
}


/**
 * @brief Converts a PlantStates enum value to its corresponding string representation.
 *
 * This function takes a `PlantStates` enum value as input and returns the corresponding string representation 
 * for that plant state. If the state does not match any known value, it returns "UNKNOWN".
 *
 * @param state The current state of the plant, represented as a `PlantStates` enum value.
 * 
 * @return const char* A string representation of the given plant state.
 * 
 * @retval "PARCHED" The plant is parched and needs water.
 * @retval "THIRSTY" The plant is thirsty and requires watering soon.
 * @retval "SATISFIED" The plant is in a healthy, well-watered state.
 * @retval "DROWNING" The plant has too much water and is in danger of drowning.
 * @retval "INSOLATE ME" The plant needs more sunlight.
 * @retval "SCORCHED" The plant has too much sunlight and is scorched.
 * @retval "UNKNOWN" The plant state is not recognized.
 */
const char* getPlantStateString(PlantStates state) {
  switch (state) {
    case PlantStates::PARCHED:
      return "PARCHED";
    case PlantStates::THIRSTY:
      return "THIRSTY";
    case PlantStates::SATISFIED:
      return "SATISFIED";
    case PlantStates::DROWNING:
      return "DROWNING";
    case PlantStates::INSOLATE_ME:
      return "INSOLATE ME";
    case PlantStates::SCORCHED:
      return "SCORCHED";
    default:
      return "UNKNOWN";
  }
}




/**
 * @brief Detects if the robot is stuck in a corner trap based on bumper presses.
 *
 * This function tracks the number of bumper presses within a specified time window.
 * If the number of presses exceeds the threshold (4 presses) within 30 seconds, 
 * it indicates that the robot is stuck in a corner and returns true. If the time 
 * window is exceeded or there are fewer than 4 presses, the function resets and 
 * continues monitoring.
 *
 * @param bumperTriggered Pointer to a boolean that is set to true when a bumper has been pressed.
 *                        The value can be modified by this function to clear the trigger if needed.
 * @return true if the robot is detected to be stuck in a corner (4 presses in 30 seconds).
 * @return false if no corner trap is detected.
 */
bool detectCornerTrap(bool *bumperTriggered) {
    // Constants
    const unsigned long trapTimeWindow = CORNER_TRAP_TIME_WINDOW; // 45 seconds in milliseconds. Configuration.h
    const int trapThreshold = CORNER_TRAP_THRESHOLD; // Number of bumper presses to trigger detection. Configuration.h

    // Static variables to retain values between function calls
    static unsigned long firstBumperPressTime = 0;
    static int bumperPressCount = 0;

    // If a bumper has been triggered
    if (*bumperTriggered) {
        *bumperTriggered = false;              // resets the flag
        if (bumperPressCount == 0) {
            // First press, start the timer
            firstBumperPressTime = millis();
        }

        // Increment the bumper press count
        bumperPressCount++;

        // Check if the trap condition is met
        if (bumperPressCount >= trapThreshold) {
            // Check if the presses happened within the time window
            if (millis() - firstBumperPressTime <= trapTimeWindow) {
                // Reset the press count and return true to indicate corner trap
                bumperPressCount = 0;
                *bumperTriggered = false; // Optionally reset bumperTriggered
                return true;
            } else {
                // Time window exceeded, reset the count and timer
                bumperPressCount = 1; // This counts as the first press
                firstBumperPressTime = millis();
            }
        }
    }

    // If no bumper press or trap not detected, return false
    return false;
}

#pragma region Behavior Functions

/**
 * @brief the code for the SEEK behavior
 */
void seekBehavior() {
  // the robot is driving, so we have to check the bumpers to make sure we can avoid obstacles
  if (leftBumperActived) {
    // sometime the robot gets stuck bouncing back and forth between walls in a corner, so detect that situation
    cornerTrapDetected = detectCornerTrap(&checkForCornerTraps);
    face.setFaceState(FaceStates::ANGRY_FACE);    // the bumper was pressed, so look mad
    // leftBumperActivated will be set to false once the retreat function completes the retreat move and returns false.
    // retreat() returns false when the retreat move is finished, which automatically resets the bumperActivated flag correctly.
    leftBumperActived = retreat(TankMoveTypes::LEFT, TankMoveTypes::STRAIGHT);
    // if there was a corner trap detected, get out of there by randomizing position
    if (cornerTrapDetected) {
      robotState.lastBehaviorState = robotState.behaviorState;    // store the current state as the last state before setting new current state. enables detecting transitions.
      robotState.behaviorState = BehaviorModes::RANDOMIZE_POSITION;
    }
    // by this point, we always need to reset this flag to false
    cornerTrapDetected = false;
  } else if (rightBumperActived) {
    cornerTrapDetected = detectCornerTrap(&checkForCornerTraps);
    face.setFaceState(FaceStates::ANGRY_FACE);
    rightBumperActived = retreat(TankMoveTypes::RIGHT, TankMoveTypes::STRAIGHT); 
    if (cornerTrapDetected) {
      robotState.lastBehaviorState = robotState.behaviorState;    // store the current state as the last state before setting new current state. enables detecting transitions.
      robotState.behaviorState = BehaviorModes::RANDOMIZE_POSITION;
    }
    cornerTrapDetected = false;
  } else {
    // the bumpers aren't activated, so aim head at light and drive toward light
    
    currentServoPos = aimHeadAtLight();
    heading = convertServoAngleToHeading(currentServoPos);
    
    if (heading < -10) {
      face.setFaceState(FaceStates::EYES_LEFT);
    } else if (heading > 10) {
      face.setFaceState(FaceStates::EYES_RIGHT);
    } else {
      face.setFaceState(FaceStates::EYES_FORWARD);
    }

    // take the current servo position and use that to derive the tread motor speeds
    rightMotorSpeed = convertHeadingToForwardBiasSpeed(90 - currentServoPos, true);
    leftMotorSpeed = convertHeadingToForwardBiasSpeed(90 - currentServoPos, false);
    tank.direct(leftMotorSpeed, rightMotorSpeed);

    // The following function plays an essential role in determining how the robot finds and parks itself in light patches. 
    // It requires some in depth explanation, which can be found in [[Footnotes#Footnote 2: How the robot parks in sunlight]]

    // check to see if the robot is spinning in circles. This is how we determine when it's time for the robot to park to sunbathe.
    if (spinningInCircles(heading)) {
      robotState.lastBehaviorState = robotState.behaviorState;    // store the current state as the last state before setting new current state. enables detecting transitions.
      robotState.behaviorState = BehaviorModes::PARK;    // if we are spinning in circles, transition to PARK state
    }
  }
}




/**
 * @brief The code for the RANDOMIZE_POSITION behavior.
 */
void randomizePositionBehavior() {
  face.setFaceState(FaceStates::EYES_CONFUSED);
      
  if (robotState.lastBehaviorState != BehaviorModes::RANDOMIZE_POSITION) {
    robotState.lastBehaviorState = robotState.behaviorState;    // we detected the state transition, now update the last state with the current one
    spinTime = random(POSITION_RANDOMIZE_SPIN_MIN, POSITION_RANDOMIZE_SPIN_MAX);       // randomize spin time, settable in Configuration.h
    driveTime = random(POSITION_RANDOMIZE_DRIVE_MIN, POSITION_RANDOMIZE_DRIVE_MAX);     // randomize drive forward time, settable in Configuration.h
    randomizeStep = 0;
    randomizeDriveTimer = 0;
    randomizeSpinTimer = 0;

    // this while loop smoothly points the head forward, rather than slamming it forward
    while (heading > 2 || heading < -2) {
      int8_t direction = heading > 0 ? -1 : 1;                        // [[Footnotes#Footnote 1: defining a variable inside a switch-case]]
      currentServoPos = runServoAtSpeed(headServo, direction * 30);
      heading = convertServoAngleToHeading(currentServoPos);
    }
    // now that the head is close to forward, we can set it direction to a heading of 0 and then move on to randomizing position
    heading = 0;      // face the head forward
    currentServoPos = convertHeadingToServoAngle(heading + SERVO_TRIM);   // Configuration.h - SERVO_TRIM helps point head straight forward
    headServo.write(currentServoPos);
  }

  // make sure we don't bump into objects, as usual
  if (leftBumperActived) {
    leftBumperActived = retreat(TankMoveTypes::LEFT, TankMoveTypes::STRAIGHT);      // retreatFromLeft returns false when the retreat move is finished, automatically breaking out of this block and resetting the flag
  } else if (rightBumperActived) {
    rightBumperActived = retreat(TankMoveTypes::RIGHT, TankMoveTypes::STRAIGHT);    // the bumperActivated flag should only be cleared once the retreat move is finished
  } else {
    switch (randomizeStep)
    {
    case 0:
      // spin in a circle first
      if (randomizeSpinTimer <= spinTime) {
        tank.rotate('r', 160);
      } else {
        randomizeStep++;
        randomizeDriveTimer = 0;    // start the randomizeDriveTimer
        tank.stop();
      }
      break;

    case 1:
      // drive in whatever direction we're pointed
      if (randomizeDriveTimer <= driveTime) {
        tank.drive('f', 160);
      } else {
        // driving move is complete, so we can get out of the randomize state and reset relevant flags
        robotState.lastBehaviorState = robotState.behaviorState;    // store the current state as the last state before setting new current state. enables detecting transitions.
        robotState.behaviorState = BehaviorModes::SEEK;
      }
      break;
    
    default:
      break;
    }
  }
}


/**
 * @brief the code for the PARK behavior
 */
void parkBehavior() {
  // stop moving the tank platform. Motion of the head is still allowed but will be controlled by a different system. 
  tank.stop();

  face.setFaceState(FaceStates::SMILING_FACE); // smile! it's happy and parked
  currentServoPos = aimHeadAtLight();          // keep pointing the head at the brightest light it can see

  // check to see if this is a new entry into the PARK state
  if (robotState.lastBehaviorState != BehaviorModes::PARK) {
    stateTimerSec = 0;                                      // we just transitioned to PARK, so reset the state timer to track how long we're here
    parkTime = random(MIN_PARK_TIME, MAX_PARK_TIME);        // spend anywhere between 30 seconds and 10 minutes parked
    robotState.lastBehaviorState = robotState.behaviorState;  // store the current state as the last state
  }
  // if we've been parked long enough, leave the park state and reset the variables. don't leave if text is scrolling
  if (stateTimerSec >= parkTime && !face.isTextBeingWritten()) {  
    robotState.lastBehaviorState = robotState.behaviorState;        // store the current state as the last state before setting new current state. enables detecting transitions.
    robotState.behaviorState = BehaviorModes::RANDOMIZE_POSITION;  // transition out to RANDOMIZE_POSITION mode
  }
}




void printStateInformation() {
  switch (robotState.behaviorState) {
    case BehaviorModes::PARK:
      SERIAL_PRINTLN("PARK");
      break;
    case BehaviorModes::SEEK:
      SERIAL_PRINTLN("SEEK");
      break;
    case BehaviorModes::CALIBRATE:
      SERIAL_PRINTLN("CALIBRATE");
      break;
    case BehaviorModes::TEST:
      SERIAL_PRINTLN("TEST");
      break;
    case BehaviorModes::RANDOMIZE_POSITION:
      SERIAL_PRINTLN("RANDOMIZE_POSITION");
      break;
    case BehaviorModes::RIGHT_PAD_TOUCHED_FREEZE:
      SERIAL_PRINTLN("RIGHT_PAD_TOUCHED_FREEZE");
      break;
    case BehaviorModes::RIGHT_PAD_TOUCHED_MOVE:
      SERIAL_PRINTLN("RIGHT_PAD_TOUCHED_MOVE");
      break;
    default:
      SERIAL_PRINTLN("UNKNOWN_MODE");
      break;
  }
}





void doTheDance() {
  static uint8_t step = 0;
  static elapsedMillis stateTimer = 0;
  
  switch (step) {
    case 0:
      tank.stop();
      face.setFaceState(FaceStates::EYES_CONFUSED);
      step++;
      break;
      
    case 1:
      if (heading < 65) {
        currentServoPos = runServoAtSpeed(headServo, 60.0);
        heading = convertServoAngleToHeading(currentServoPos);
      } else {
        step++;
        stateTimer = 0;
      }
      break;
      
    case 2:
      if (stateTimer >= 500) {
        face.setFaceState(FaceStates::SMILING_FACE);
        step++;
        stateTimer = 0;
      }
      break;
      
    case 3:
      if (stateTimer >= 2000) {
        step++;
      }
      break;
    
    case 4:  
      if (heading > 1) {
        currentServoPos = runServoAtSpeed(headServo, -60.0);
        heading = convertServoAngleToHeading(currentServoPos);
      } else {
        step++;
        stateTimer = 0;
      }
      break;
    
    case 5:
      if (stateTimer < 500) {
        tank.drive('f', 200);
      } else {
        step++;
        stateTimer = 0;
      }
      break;

    case 6:
      if (stateTimer < 1000) {
        tank.drive('b', 200);
      } else {
        step++;
        stateTimer = 0;
      }
      break;

    case 7:
      if (stateTimer < 500) {
        tank.drive('f', 200);
      } else {
        step++;
        stateTimer = 0;
      }
      break;

    case 8:
      if (stateTimer < 1000) {
        tank.rotate('r', 150);
      } else {
        step++;
        stateTimer = 0;
      }
      break;
    
    case 9:
      if (stateTimer < 1750) {
        tank.rotate('l', 200);
      } else {
        step++;
        stateTimer = 0;
      }
      break;

    case 10:
      if (stateTimer < 1000) {
        tank.rotate('r', 200);
      } else {
        step++;
        stateTimer = 0;
        tank.stop();
      }
      break;
  
    case 11:
      if (stateTimer >= 1000) {
        face.setFaceState(FaceStates::SMILING_FACE);
        // face.updateFace();
        step++;
        stateTimer = 0;
      }
      break;
      
    case 12:
      if (stateTimer >= 1000) {
        step++;
      }
      break;

    case 13:
      robotState.behaviorState = BehaviorModes::RIGHT_PAD_TOUCHED_FREEZE;
      step = 0;  // Reset for next time
      break;

    default:
      break;
  }
}




#pragma endregion Behavior Functions