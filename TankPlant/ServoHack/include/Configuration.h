#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

/**
 * This is a macro for easily enabling or disabling the Serial monitor print statements. Using the Serial monitor can be extremely
 * helpful for debugging or examining the behavior of your program, but it also takes up room in memory and processing power, so
 * it is helpful to be able to stop using Serial statements easily. This acts like a switch, and lets you deactivate all Serial
 * monitor use just by changing a single value. 
 * 
 * You can enable serial print debugging by setting 
 *    #define USE_SERIAL 1
 * 
 * Or, you can disable serial print debugging by setting
 *    #define USE_SERIAL 0
 * 
 * In your main code, rather than using Serial.print() or Serial.println(), use their aliases defined below (e.g., SERIAL_PRINTLN()):
 *  */ 
#define USE_SERIAL 0

#if USE_SERIAL
  #define SERIAL_PRINT(x) Serial.print(x)
  #define SERIAL_PRINTLN(x) Serial.println(x)
  #define SERIAL_BEGIN(baud) Serial.begin(baud)
#else
  #define SERIAL_PRINT(x) do {} while (0)
  #define SERIAL_PRINTLN(x) do {} while (0)
  #define SERIAL_BEGIN(baud) do {} while (0)
#endif


/**
 * When this is defined, every time a call is made to one of the functions in CL_DRV8835 that would spin the drive motors,
 * the program first sets pin D10 to be INPUT_PULLUP and then checks to see if that pin is LOW. If it is, it overrides
 * all motor speed commands and sets them to 0. This lets you prevent the robot from driving around by connecting a wire
 * from D10 to ground. As soon as you unplug the wire, the robot can move. I've found this parking brake to be useful for
 * testing other features of the robot without it driving around (like seeing what new faces look like).
 */
#define PARKING_BRAKE


// If you find that one or both of the motors that drive the tank treads are running in the opposite direction that they should,
// you can fix that here. To reverse one of the motors, change the value after the appropriate define to "true" without quotes.
#define REVERSE_RIGHT_MOTOR false
#define REVERSE_LEFT_MOTOR false


/**
 * Proportional constant for head servo proportional controller.
 * 
 * so far a kp of 1.0 has worked really well. Originally I was using an instance of FastPID to control the head servo,
 * but after testing I wound just using {kp, ki, kd} = {1.0, 0, 0}, so I rewrote a few lines of code to not rely on FastPID at all.
 * Adjust the value of this constant to change how responsive the head is to differences in light levels between the two sensors.
 * Also, you can make this value negative to make the robot turn its head away from light!
 */
constexpr float HEAD_SERVO_KP = 1.0;


// Maximum angular speed that the head servo will be allowed to spin in degrees per second
constexpr float MAX_HEAD_SERVO_SPEED = 90.0; 


/**
 * Head servo center point trim.
 * 
 * This is a rough fix to make the head point straight forward, if it doesn't already. If you find that the head looks a bit
 * too the left or right when the robot first turns on and points its head forward, you can adjust that by changing the value
 * of SERVO_TRIM. Units are degrees, positive values turn head to the right.
 * 
 * The best way to fix this is mechanically. Turn the robot on and then back off when the servo moves to center position. 
 * Then remove the gear from the shaft of the servo and rotate it slightly to occupy a new angle on the servo shaft. 
 */
constexpr int8_t SERVO_TRIM = 2; // For my testing, the robot needed +8 deg of trim 


// light sensor calibration related values
constexpr uint16_t LIGHT_CAL_SPIN_TIME = 9000;  // spin in circle for 9 seconds during calibration mode
constexpr uint8_t LIGHT_CAL_SPIN_SPEED = 140;   // spin at speed of 140 during calibration mode


// constants for changing when the robot notices that it's spinning in circles and parks.
constexpr uint16_t MIN_PARK_TIME = 10;
constexpr uint16_t MAX_PARK_TIME = 20; // minimum and maximum park times in seconds
constexpr uint8_t PARKING_THRESHOLD_ANGLE = 65;                  // heading angle used as threshold in the spinningInCircles() function
constexpr uint8_t MOISTURE_SAMPLING_PERIOD = 10;           // sample the moisture every 10 seconds. also prints state to LED matrix at this interval in PARK
constexpr uint16_t HEADING_SAMPLE_WINDOW = 8000;           // track the heading over a window of 8 seconds
constexpr uint8_t HEADING_FILTER_SIZE = 80;                // how many samples of the heading will be averaged


// tank movement values
constexpr uint16_t TANK_RETREAT_INTERVAL = 1000;         // Time in milliseconds to reverse away from object
constexpr uint16_t TANK_RETREAT_ROTATE_INTERVAL = 1000;  // Time in milliseconds to rotate away from object after reverse
constexpr uint16_t POSITION_RANDOMIZE_SPIN_MIN = 4000;   // Minimum amount of time robot will spin in place when randomizing its position
constexpr uint16_t POSITION_RANDOMIZE_SPIN_MAX = 9000;   // Maximum amount of time robot will spin in place when randomizing its position
constexpr uint16_t POSITION_RANDOMIZE_DRIVE_MIN = 10000; // Minimum amount of time robot will drive forward after spinning during position randomization sequence
constexpr uint16_t POSITION_RANDOMIZE_DRIVE_MAX = 20000; // Maximum amount of time robot will drive forward after spinning during position randomization sequence


// corner trap detection variables
constexpr uint16_t CORNER_TRAP_TIME_WINDOW = 45000;      // Time window during which bumper presses count toward the corner trap detection threshold
constexpr uint8_t CORNER_TRAP_THRESHOLD = 4;             // How many bumper presses within the above time window indicate a corner trap


/**
 * speed maps for converting headings to motor speeds.
 * 
 * It's important that these are variables, not constants. Constants get get stored in PROGMEM,
 * and multiMap isn't written in a way that lets it read arrays from PROGMEM. We could rewrite it to use
 * PROGMEM, but reading from PROGMEM is slow compared to reading from RAM, and since these arrays are used
 * frequently, we want the speed RAM offers.
 * 
 * If you want to see what these mappings look like in graph form, find that here: 
 * https://docs.google.com/spreadsheets/d/1x5wN7Up8Ra-3hVbiYZuimurmymXRQdSPz-L6IVtE-hY/edit?usp=sharing
 */
inline int16_t HEADINGS_IN_MAP[5] = {
  -90,
  -45,
  0,
  45,
  90
};    // independent variable for the piecewise functions

inline int16_t LEFT_SPEED_MAP[5] = {
  -255,
  0,
  150,
  192,
  255
};  // dependent variable for piecewise linear function for left motor (ex: when heading is 45, lMotor speed is 192)

inline int16_t RIGHT_SPEED_MAP[5] = {
  255,
  192,
  150,
  0,
  -255
}; // dependent variable for piecewise linear function for right motor speed (ex: when heading is 45, rMotor speed is 0)

/**
 * These values relate to the moisture sensor. The offset value is the number that the sensor reports when it's in open air, while the other values
 * are upper bounds for the different moisture levels. See [[Footnotes.md#Footnote 7: Notes on calibrating the moisture sensor readings]]
*/
// water threshold values
constexpr uint16_t MOISTURE_OFFSET = 754;       // get the reading on the moisture sensor in open air and use that here
constexpr uint8_t PARCHED_UPPER_THRESHOLD = 40; // the upper bound for considering the plant to be parched
constexpr uint8_t THIRSTY_UPPER_THRESHOLD = 70; // the upper bound for considering the plant to be thirsty.
constexpr int SATISFIED_UPPER_THRESHOLD = 130;  // beyond this the plant will be considered to be drowning, with water starting to fill the pot rather than be absorbed by the LECA.


/** 
 * SERVO HACK PARAMETERS 
*/
constexpr uint8_t FLAG_SERVO_PIN = 12;      // Connected flag servo data pin to D12
constexpr uint8_t FLAG_LOWERED_POS = 20;    // servo angle to put the flag into the lowered position
constexpr uint8_t FLAG_RAISED_POS = 90;     // vertical angle (useful for tuning if servo horn is misaligned)
constexpr uint8_t FLAG_WAVE_ANGLE = 30;     // degrees to wave flag by
inline uint8_t flagServoPos = FLAG_RAISED_POS;     // servo position tracking variable, set to starting position
inline float flagWaveSpeed = 40.0;                 // wave speed in degrees per second

#endif // CONFIGURATION_H