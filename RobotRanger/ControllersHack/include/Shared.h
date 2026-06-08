/*
************************************************************************************
* The following are the libraries and functions shared between all modes.
************************************************************************************
*/

#pragma once
#pragma region LIBRARIES
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <FastLED.h>
#pragma endregion

#pragma region SHARED FUNCTION PROTOTYPES

// SERVOS
void resetPositions();
int16_t conversionYawMicro(int16_t);
int16_t conversionPitchMicro(int16_t);
void servoEase(int16_t);
void servoEase(int16_t, bool, int16_t delayMs = 30); // overloading ServoEase
void launchRoutine(uint16_t);


// BUTTONS
int8_t getButton(int16_t);
void handleButtonInput();


// SETUP
void initPins();
void initHardware();
void checkEEPROM();

// FLOAT MAP
float floatMap(float, float, float, float, float);

// HACKS
void incrementController(int16_t);
void handleButtonInputControllerSelect();
void enterController(); 
void blinkOnEnter(int8_t, uint16_t); 

#pragma endregion

/*
************************************************************************************
* The following are the pin definitions and relevant constants for the robot.
************************************************************************************
*/
#pragma region PIN DEFINITIONS

// ANALOG PINS
#define LASER_PIN A3
#define POT_PIN A6
#define BUTTON_RESISTOR_LADDER A7

// DIGITAL PINS
#define BREAK_PIN 2
#define LED_DATA_PIN 3
#define LAUNCH_SERVO_PIN 5
#define PITCH_SERVO_PIN 9
#define YAW_SERVO_PIN 11

#pragma endregion

#pragma region CONSTANT DEFINITIONS

// TIME OF FLIGHT SENSOR - Reference FOOTNOTES for more information about this sensor.
#define TIMING_BUDGET 20000 // Timing budget is a tradeoff between accuracy and speed.
#define SIGNAL_STRENGTH 4.0 // Signal strength is a measure of the quality of the signal. This number acts as a filter for the cups.

// LEDS
#define NUM_LEDS 1

// MOTORS
#define SMOOTHING 10 // motor smoothing
#define startPWM 500
#define endPWM 2500
#define visualYawCenterPWM 1525 // compensates for servo alignment, can be changed if needed
#define backlashCompensation 30 // compensates for motor backlash when easing

// GEOMETRY & CONSTANTS - Reference FOOTNOTES for more information about this section.
#define MAXD 500                    // max distance 500mm or ~ 1.5 ft - derived from ToF sensor resolution reliability at distance
#define GEAR_RATIO 4                // pitch gear to launcher ratio
#define cupR 33.0                   // cup radius in mm
#define baseR 90.0                  // base radius in mm
#define launcherR 115.0             // launcher radius arm in mm
#define sensorOffset 1.0            // sensor offset from front of robot in mm
#define barrelAdjust 7.75           // barrel adjust in mm
#define g 9810.0                    // mm/s^2
#define h 70.0                      // top of cup to center of launcher in mm

#pragma endregion

/*
 ************************************************************************************
 * This is the global state of the robot, and it is a singleton class.
 *
 * Singleton means that there is only one instance of the class,
 * and it is accessed through the getInstance() function - shorthanded as "robot".
 * For example, robot.launchServo.write(90); accesses the launchServo object.
 *
 * This prevents the need to pass around the robot object to all the modes, and prevents multiple definitions.
 * Also every effector (motor, sensor, etc.) is connected to the robot object.
 ************************************************************************************
 */

#pragma region GLOBAL ROBOT STATE
class GlobalState
{
public:
  static GlobalState &getInstance()
  {
    static GlobalState instance;
    return instance;
  }

  // HARDWARE OBJECTS
  Servo yawServo;
  Servo pitchServo;
  Servo launchServo;
  CRGB leds[NUM_LEDS];

  // TIMING VARIABLES
  int32_t currTime = 0;

  // CALIBRATION VARIABLES
  bool calibrating = false;
  bool runningYawCalibration = false;
  bool runningTiltCalibration = false;

  // BUTTON HANDLING VARIABLES
  int16_t currButton = 0;
  uint32_t pressStart = 0;              // Time when button press started
  uint32_t lastRepeat = 0;              // Time of last repeat action
  bool holding = false;                      // Whether button is being held
  int16_t lastButton = 0;                        // Last button state
  const int16_t longPressThreshold = 3500;       // Threshold for long press (ms)
  const uint32_t HOLD_TIME = 500;       // Time to hold for mode change (ms)
  const uint32_t REPEAT_INTERVAL = 200; // Time between repeats while holding (ms)
  float sensAdj = 0;                         // sensor offset compensation initalization (do not change)

  /*
  *********************************************************************************************
  * BELOW ARE TUNABLE VARIABLES IN THE WORST CASE SCENARIO YOU NEED TO ADJUST YOUR OWN ROBOT VIA CODE
  *********************************************************************************************
  */

  // LAUNCHING VARIABLES
  float v0 = 2200;        // launch velocity in mm/s
  int16_t sensAdjMin = -5;    // sensor offset compensation min (see calcOffset function for usage)
  int16_t sensAdjMax = 20;    // sensor offset compensation max (see calcOffset function for usage)
  uint16_t waitTime = 500;    // wait time in ms to let ball "settle" before launch
  uint16_t debounceTime = 175; // debounce time in ms to prevent launcher from stalling

  // PITCH & YAW OFFSETS - Tunable in function below if needed.
  float TILT_ADJUST;  // pitch tilt adjust (tunable below)
  int16_t YAW_OFFSET_MIN; // yaw offset min (tunable below)
  int16_t YAW_OFFSET_MAX; // yaw offset max (tunable below)
  int16_t YAW_EXTRA = 5;  // tunable extra offset for yaw calibration

  // HACK TRACKING VARIABLES
  bool selectController = false; 
  static const uint8_t numControllers = 4;
  CRGB controllerColors[numControllers] = {CRGB(0, 0, 255), CRGB(250, 175, 0), CRGB(0, 255, 0), CRGB(255, 0, 0)};
  int8_t controllerIndex = 0;


  // The following are basically the median values of the error deviation in both pitch and yaw.
  void defaultOffsets()
  {
    TILT_ADJUST = 18.0;
    YAW_OFFSET_MIN = 15;
    YAW_OFFSET_MAX = YAW_OFFSET_MIN + YAW_EXTRA; // offset needs to be more aggressive at the end (backlash accumalates)
  }

private:
  GlobalState() = default;
  GlobalState(const GlobalState &) = delete;            // Prevents copying (ensures one copy)
  GlobalState &operator=(const GlobalState &) = delete; // Prevents reassignment
};

/*
*********************************************************************************************
* For convenience, creates a global reference to the state under the name "robot"
*********************************************************************************************
*/
#define robot GlobalState::getInstance()
#pragma endregion

#pragma region SERIAL TOGGLE

/*
*********************************************************************************************
* The following is used to toggle the serial output. This is useful for debugging.
* It is set to 0 by default, which means no serial output will be printed - this is to save memory on the micontroller.
*********************************************************************************************
*/
#define USE_SERIAL 1 // 0 is disabled, 1 is enabled

#if USE_SERIAL
#define SERIAL_PRINT(x) Serial.print(x)
#define SERIAL_PRINTLN(x) Serial.println(x)
#define SERIAL_BEGIN(baud) Serial.begin(baud)
#define SERIAL_TAB Serial.print("\t")
#define SERIAL_TABS(x)            \
  for (uint8_t i = 0; i < x; i++) \
  {                               \
    Serial.print("\t");           \
  }
#else
#define SERIAL_PRINT(x) \
  do                    \
  {                     \
  } while (0)
#define SERIAL_PRINTLN(x) \
  do                      \
  {                       \
  } while (0)
#define SERIAL_BEGIN(baud) \
  do                       \
  {                        \
  } while (0)
#define SERIAL_TAB \
  do               \
  {                \
  } while (0)
#define SERIAL_TABS \
  do                \
  {                 \
  } while (0)
#endif

#pragma endregion SERIAL TOGGLE
