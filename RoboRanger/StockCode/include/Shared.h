#pragma once

#pragma region LIBRARIES
#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <Servo.h>
#include <FastLED.h>
#include <EEPROM.h>
#pragma endregion

#pragma region SHARED FUNCTION PROTOTYPES

//SERVOS
void resetPositions(); 
int conversionMicro(int);
void servoEase(int targetAngle);
void servoEase(int targetAngle, bool isYaw, int delayMs = 30); //overloading ServoEase
void launchRoutine(); 
void launchRoutine(long pullDelayMs, long holdDelayMs); //overloading launchRoutine

//LAUNCHING MATH
float calculateRoot(float);
int calculateLaunchAngle(float);
float calcOffset(float distance);

//BUTTONS
int getButton(int v);
void handleButtonInput();
void waitForButtonPress();

//SETUP
void initPins();
void initHardware(); 
void checkEEPROM();
void printRoots();
void defaultOffsets();

//CALIBRATION
void handleCalibration();

#pragma endregion

#pragma region PIN DEFINITIONS

//ANALOG PINS
#define LASER_PIN A3
#define POT_PIN A6
#define BUTTON_RESISTOR_LADDER A7

//DIGITAL PINS
#define BREAK_PIN 2 
#define LED_DATA_PIN 3
#define LAUNCH_SERVO_PIN 5
#define PITCH_SERVO_PIN 9
#define YAW_SERVO_PIN 11

#pragma endregion

#pragma region CONSTANT DEFINITIONS

//TIME OF FLIGHT SENSOR
#define TIMING_BUDGET 20000
#define SIGNAL_STRENGTH  4.0

//LEDS
#define NUM_LEDS 1

//MOTORS
#define SMOOTHING 10 // motor smoothing
#define startPWM 500
#define endPWM 2500
#define visualYawCenterPWM 1525 //compensates for servo alignment
#define backlashCompensation 30 //compensates for motor backlash when easing

// LAUNCHING 
#define MAXD 500      // max distance ~ 500mm or 1.5 ft
#define GEAR_RATIO 4 // pitch gear to launcher ratio
#define cupR 33.0
#define baseR 90.0
#define launcherR 115.0
#define sensorOffset 1.0    
#define barrelAdjust 7.75
#define magazineCollisionAngle 2350 //any steeper than this and loading magazine potentially jams
#define g 9810.0  // mm/s^2
#define h 70.0    // top of cup to center of launcher

#pragma endregion

#pragma region SERIAL TOGGLE
#define USE_SERIAL 0

#if USE_SERIAL
  #define SERIAL_PRINT(x)     Serial.print(x)
  #define SERIAL_PRINTLN(x)   Serial.println(x)
  #define SERIAL_BEGIN(baud)  Serial.begin(baud)
  #define SERIAL_TAB          Serial.print("\t")
  #define SERIAL_TABS(x)      for (uint8_t i = 0; i < x; i++) {Serial.print("\t");}
#else
  #define SERIAL_PRINT(x)     do {} while (0)
  #define SERIAL_PRINTLN(x)   do {} while (0)
  #define SERIAL_BEGIN(baud)  do {} while (0)
  #define SERIAL_TAB          do {} while (0)
  #define SERIAL_TABS         do {} while (0)
#endif

#pragma endregion SERIAL TOGGLE

/*
  ************************************************************************************
  * This is the global state of the robot, and it is a singleton class. 
  * Singleton means that there is only one instance of the class, 
  * and it is accessed through the getInstance() function - shorthanded as "robot". 
  * For example, robot.launchServo.write(90); accesses the launchServo object.  
  * This prevents the need to pass around the robot object to all the modes, and prevents multiple definitions. 
  * Also every effector (motor, sensor, etc.) is connected to the robot object.
  ************************************************************************************
*/

#pragma region GLOBAL ROBOT STATE
class GlobalState {
public:
    static GlobalState& getInstance() {
        static GlobalState instance;
        return instance;
    }

    // HARDWARE OBJECTS
    VL53L1X ranger;
    VL53L1X::RangingData data;
    Servo yawServo;
    Servo pitchServo;
    Servo launchServo;
    CRGB leds[NUM_LEDS];


    // TIMING VARIABLES
    long currTime = 0;

    //CALIBRATION VARIABLES
    bool calibrating = false;
    bool runningYawCalibration = false;
    
    //LAUNCHING VARIABLES
    float v0 = 2500;
    long waitTime = 150;
    float sensAdj = 0;
    float TILT_ADJUST = 0;
    int OFFSET_MIN = 0;
    int OFFSET_MAX = 0;

    // Button handling variables
    int currButton = 0; 
    unsigned long pressStart = 0;           // Time when button press started
    unsigned long lastRepeat = 0;           // Time of last repeat action
    bool holding = false;                   // Whether button is being held
    int lastButton = 0;                     // Last button state
    const int longPressThreshold = 3500;    // Threshold for long press (ms)
    const unsigned long HOLD_TIME = 500;    // Time to hold for mode change (ms)
    const unsigned long REPEAT_INTERVAL = 200; // Time between repeats while holding (ms)

private:
    GlobalState() = default;
    GlobalState(const GlobalState&) = delete; // Prevent copying
    GlobalState& operator=(const GlobalState&) = delete; // Prevent assignment
};

// For convenience, create a global reference to the state
#define robot GlobalState::getInstance()

#pragma endregion