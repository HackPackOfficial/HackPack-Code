#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

// =============================================================================
// WEATHER CONFIGURATION
// =============================================================================

// wttr.in is rate-limited (~1 req/min/IP), so keep this long.
constexpr uint32_t WEATHER_REFRESH_MS = 1800000;  // 30 minutes - drop down: 600000, 1800000, 3600000

// Temperature range used to map forecast temperature onto a 0..1 "warmth" scale.
constexpr int8_t WARMTH_MIN_C = -10;   // colder than this clamps to fully cold
constexpr int8_t WARMTH_MAX_C = 40;    // hotter than this clamps to fully hot

// =============================================================================
// SWIM BEHAVIOR CONFIGURATION
// =============================================================================
constexpr uint8_t   SWIM_MARGIN          = 100;     // clearance from walls

// Weather-driven swim excursion (how far the next wander target sits from the
// current position). Cold -> short excursions (drifts slowly); hot -> long
// excursions (roams widely). The per-step rate stays constant, so temperature
// changes how FAR it travels, not how fast it steps.
constexpr int16_t   SWIM_COLD_DIST       = 40;      // excursion in px when cold
constexpr int16_t   SWIM_HOT_DIST        = 600;     // excursion in px when hot
constexpr int16_t   SWIM_REACH           = 4;       // px from target counted as "arrived"

// Arm-servo tracking smoothing (ServoWrapper exponential filter). Lower = tracks
// the swim target faster (more physical motion); higher = sluggish. Scaled by
// warmth so the ferrofluid physically roams when hot and stays calm when cold.
constexpr float     SERVO_SMOOTH_COLD    = 0.999f;  // cold: barely moves  - range: 0.9999f to 0.5f
constexpr float     SERVO_SMOOTH_HOT     = 0.93f;   // hot: tracks quickly - range: 0.9999f to 0.5f

// =============================================================================
// RIPPLE DISPLAY CONFIGURATION (pond-ripple OLED effect)
// =============================================================================
constexpr uint16_t  RIPPLE_SPAWN_MS  = 6000;  // drop a new "stone" this often (ms) - drop down: 2000, 4000, 6000, 8000, 10000 
constexpr float     RIPPLE_SPEED     = 0.01f; // ring expansion speed (px/ms) - range: 0.001f to 1.0f
constexpr uint8_t   RIPPLE_SPACING   = 12;    // gap between concentric rings (px) range: 1 to 20
constexpr uint8_t   RIPPLE_MAX_RINGS = 7;     // rings drawn per stone - drop down: 3, 5, 7, 9, 11
constexpr uint8_t   RIPPLE_MAX        = 8;    // max simultaneous stones on screen - range: 1 to 20
constexpr uint8_t   RIPPLE_INTENSITY  = 80;   // ink added per ring pass (overlap -> brighter) - range 1 to 100
constexpr uint16_t  RIPPLE_FRAME_MS   = 33;   // display redraw throttle (~30fps)
constexpr uint8_t   RIPPLE_BLUR_RADIUS = 1;   // box-blur radius (1 => 3x3) for the soft look - drop down: 0, 1, 2, 3, 4, 5


// =============================================================================
// LED CONFIGURATION
// =============================================================================
constexpr uint8_t NUM_LEDS      = 8;
constexpr uint8_t LED_BRIGHTNESS = 100;   // Range: 0 to 255

// =============================================================================
// COORDINATE TRIM VALUES
// =============================================================================

// These variables move the center position of the magnet relative to the jar.
// If you find it isn't properly centered, tweak these values. 
// The functions use them in calculateRotatedServoTargets.
constexpr int16_t X_COORD_TRIM = -100;  // Range: -512 to 511
constexpr int16_t Y_COORD_TRIM = -100;  // Range: -512 to 511

// The 5R parallel linkage that positions the magnet inverts the X axis relative
// to the logical swim coordinate space. When true, the servo mapping mirrors X
// about the trimmed center so the physical ferrofluid matches the OLED eye.
constexpr bool MIRROR_MAGNET_X = true;


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
// TIMING CONFIGURATION (milliseconds)
// =============================================================================
constexpr uint16_t DEBOUNCE_MS         = 10;
constexpr uint16_t PATTERN_INTERVAL_MS = 100;
constexpr uint16_t UPDATE_INTERVAL_MS  = 10;
constexpr uint16_t SOFT_SHUTDOWN_TIME   = 2000;         // time before the system powers off after button held
constexpr uint32_t BLOCKING_MOVE_TIMEOUT_MS = 10000;    // failsafe timeout for blocking servo moves

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