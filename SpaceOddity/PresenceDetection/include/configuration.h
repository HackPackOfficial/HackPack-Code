#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

// =============================================================================
// ULTRASONIC PRESENCE DETECTION
// =============================================================================
constexpr uint16_t ULTRASONIC_PING_INTERVAL     = 100;    // ms between pings; keep far above the 40ms sensor timeout. Range: 50 to 500
constexpr uint16_t ULTRASONIC_PRESENCE_CM       = 150;    // inside this distance a nearby person counts as "present". Range: 100 to 400
constexpr uint16_t ULTRASONIC_ANGRY_CM          = 10;     // inside this distance the creature gets angry; keep below ULTRASONIC_FAST_CM. Range: 5 to 50
constexpr uint16_t ULTRASONIC_FAST_CM           = 100;    // inside this distance swim speed ramps up with closeness; keep between ANGRY_CM and PRESENCE_CM. Range: 20 to 200
constexpr float    ULTRASONIC_MAX_SPEED_BOOST   = 6.0f;   // fastest swim multiplier when the object is close. Range: 1.0 to 10.0
constexpr uint32_t ULTRASONIC_DANCE_TRIGGER_MS  = 60000;  // greet with a dance when it's been this long since a detection. Range: 10000 to 300000
constexpr uint32_t ULTRASONIC_DANCE_DURATION_MS = 10000;  // how long the greeting dance lasts. Range: 3000 to 60000


// =============================================================================
// COORDINATE TRIM VALUES
// =============================================================================

// These variables move the center position of the magnet relative to the jar.
// If you find it isn't properly centered, tweak these values. 
// The functions use them in calculateRotatedServoTargets.
constexpr int16_t X_COORD_TRIM = -100;  // Range: -512 to 511
constexpr int16_t Y_COORD_TRIM = -100;  // Range: -512 to 511


// =============================================================================
// SERVO CONFIGURATION
// =============================================================================

// Servo travel limits in degrees. We do not use the full 0..180 range
constexpr uint8_t SERVO_MIN           = 5;    // Range: 0 to 180
constexpr uint8_t SERVO_MAX           = 155;  // Range: 0 to 180

// Trim fixes the small error between the ordered angle and the true angle.
// Add or subtract a few degrees so each arm points where we expect.
constexpr int8_t SERVO_RIGHT_TRIM     = -5; // Range: 0 to 180
constexpr int8_t SERVO_LEFT_TRIM      = 3;  // Range: 0 to 180
constexpr uint8_t MAGNET_MIN          = 5;  // Range: 0 to 180
constexpr uint8_t MAGNET_MAX          = 80; // Range: 0 to 180

// Angle of the magnet servo when the magnet is parked (fully up).
constexpr uint8_t MAGNET_PARK_POS     = 0;  // Range: 0 to 180
constexpr uint8_t MAGNET_SERVO_TRIM   = 0;  // Range: 0 to 180

// =============================================================================
// JOYSTICK CONFIGURATION
// =============================================================================

// changed to 477 because the joystick pots are not reading perfect center
// you might need to tune this for your specific joystick
constexpr uint16_t JOY_CENTER     = 477;  // Range: 0 to 1023
constexpr uint16_t JOY_THRESHOLD  = 200;  // Range: 0 to 1023
constexpr uint16_t JOY_MAX        = 1023; // Range: 0 to 1023

// =============================================================================
// LED CONFIGURATION
// =============================================================================
constexpr uint8_t NUM_LEDS      = 8;
constexpr uint8_t LED_BRIGHTNESS = 100;   // Range: 0 to 255

// =============================================================================
// TIMING CONFIGURATION (milliseconds)
// =============================================================================
constexpr uint16_t DEBOUNCE_MS         = 10;
constexpr uint16_t PATTERN_INTERVAL_MS = 100;
constexpr uint16_t UPDATE_INTERVAL_MS  = 10;
constexpr uint16_t MENU_NAV_DELAY_MS   = 150;           
constexpr uint16_t SOFT_SHUTDOWN_TIME   = 2000;         // time before the system powers off after button held
constexpr uint32_t BLOCKING_MOVE_TIMEOUT_MS = 10000;    // failsafe timeout for blocking servo moves

// change how quickly the magnet pulses
constexpr uint16_t MAGNET_SERVO_UPDATE_INTERVAL = 120;  // Range: 0 to 10000

// You probably shouldn't push THAT button, but if you do:
constexpr uint16_t stayAngryDuration = 2000;    // drop down: 1000, 2000, 3000, 5000, 10000

// Settings for the big entry and exit moves.
// These moves run at a constant speed. We drive them through ServoWrapper so
// the PWM (pulse-width modulation) signal stays on for the whole move.
// The staggered start spreads the servos out in time. This keeps the supply
// current low so the board does not reset.
constexpr float    SLOW_MOVE_DPS         = 60.0f;   // entry/exit speed in degrees per second
constexpr uint16_t SLOW_STEP_INTERVAL_MS = 20;      // interpolation step interval (50Hz update)
constexpr uint16_t SLOW_STAGGER_MS       = 90;      // staggered start to flatten current spikes

// =============================================================================
// SWIM BEHAVIOR CONFIGURATION
// =============================================================================
constexpr uint8_t   SWIM_MARGIN          = 100;     // clearance from walls
constexpr float     SWIM_SPEED           = 1.8f;    // is it a fast buddy? Drop down: 0.8 through 3.0 in steps of 0.2
constexpr float     SWIM_STEER_STRENGTH  = 0.1f;    // how hard it turns. Range: 0.0 to 1.0

// =============================================================================
// MENU CONFIGURATION
// =============================================================================
constexpr uint8_t MENU_ITEM_COUNT = 1;
const char* const MENU_LABELS[MENU_ITEM_COUNT] = {"SWIM"};

constexpr uint8_t MENU_X_OFFSET    = 30;
constexpr uint8_t MENU_TEXT_OFFSET = 40;
constexpr uint8_t MENU_ITEM_WIDTH  = 65;
constexpr uint8_t MENU_ITEM_HEIGHT = 15;
constexpr uint8_t MENU_CORNER_RAD  = 6;
constexpr uint8_t MENU_Y_START     = 16;
constexpr uint8_t MENU_Y_SPACING   = 18;


// Just my usual wrapper around Serial that makes it easy to turn on and off.
// if USE_SERIAL is defined (uncomment the following line), then the Serial
// monitor is activated.

#define USE_SERIAL

#ifdef USE_SERIAL
  #define SERIAL_PRINT(x) Serial.print(x)
  #define SERIAL_PRINTLN(x) Serial.println(x)
  #define SERIAL_BEGIN(baud) Serial.begin(baud)
  #define SERIAL_TAB Serial.print("\t")
  #define SERIAL_TABS(x) for (uint8_t i = 0; i < x; i++) {Serial.print("\t");}
#else
  #define SERIAL_PRINT(x)       do {} while (0)
  #define SERIAL_PRINTLN(x)     do {} while (0)
  #define SERIAL_BEGIN(baud)    do {} while (0)
  #define SERIAL_TAB            do {} while (0)
  #define SERIAL_TABS(x)        do {} while (0)
#endif

#endif  // CONFIGURATION_H