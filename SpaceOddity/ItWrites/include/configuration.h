#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

// =============================================================================
// COORDINATE TRIM VALUES
// =============================================================================

// These variables move the center position of the magnet relative to the jar.
// If you find it isn't properly centered, tweak these values. 
// The functions use them in calculateRotatedServoTargets.
constexpr int16_t X_COORD_TRIM = -100;  // Range: -512 to 511
constexpr int16_t Y_COORD_TRIM = -200;  // Range: -512 to 511


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
constexpr uint16_t JOY_MAX        = 1023; // Range: 0 to 1023

// =============================================================================
// LED CONFIGURATION
// =============================================================================
constexpr uint8_t NUM_LEDS      = 8;
constexpr uint8_t LED_BRIGHTNESS = 100;   // Range: 0 to 255

// =============================================================================
// TIMING CONFIGURATION (milliseconds)
// =============================================================================
constexpr uint16_t PATTERN_INTERVAL_MS = 100;
constexpr uint16_t SOFT_SHUTDOWN_TIME   = 2000;         // time before the system powers off after button held
constexpr uint32_t BLOCKING_MOVE_TIMEOUT_MS = 10000;    // failsafe timeout for blocking servo moves

// change how quickly the magnet pulses
constexpr uint16_t MAGNET_SERVO_UPDATE_INTERVAL = 120;  // Range: 0 to 10000

// Settings for the big entry and exit moves.
// These moves run at a constant speed. We drive them through ServoWrapper so
// the PWM (pulse-width modulation) signal stays on for the whole move.
// The staggered start spreads the servos out in time. This keeps the supply
// current low so the board does not reset.
constexpr float    SLOW_MOVE_DPS         = 60.0f;   // entry/exit speed in degrees per second
constexpr uint16_t SLOW_STEP_INTERVAL_MS = 20;      // interpolation step interval (50Hz update)
constexpr uint16_t SLOW_STAGGER_MS       = 90;      // staggered start to flatten current spikes


// =============================================================================
// TEXT-WRITING CONFIGURATION
// =============================================================================
// The magnet lives in a centered coordinate space around JOY_CENTER. Because
// calculateRotatedServoTargets() constrains the rotated axes to [0, JOY_MAX]
// and maps them through SERVO_MIN..SERVO_MAX, the mechanically reachable region
// is an approximately circular area of radius ~330 around the center. We
// normalize every glyph to fill a circle of WRITE_RADIUS so each letter is as
// large as the jar allows while staying reachable.
constexpr int16_t  WRITE_RADIUS        = 300;    // glyph normalization radius (jar units)
constexpr float    WRITE_SMOOTHING     = 0.6f;   // servo easing per moveTo() call
constexpr uint16_t WRITE_POINT_INTERVAL_MS = 20; // ms between glyph points (100 pts -> 2s/letter); tunable
constexpr uint16_t WRITE_LETTER_PAUSE  = 250;    // ms pause shown after each letter
constexpr uint16_t SERIAL_MSG_MAX      = 128;    // max chars in an incoming serial message
constexpr uint8_t  WRITE_DISPLAY_SIZE  = 5;      // OLED text size for the current letter

// -----------------------------------------------------------------------------
// ON-SCREEN LETTER DISTORTION (processing layer)
// -----------------------------------------------------------------------------
// The ferrofluid mechanism draws a clean letter, but on the OLED we run the
// letter through a selectable distortion pass so it stays recognizable-ish but
// is deliberately hard to read. To try a new algorithm, add an entry to the
// GlyphFx enum + a fxGlyph*() function below, then pick it in WRITE_DISPLAY_EFFECT
// (or send "#" over serial to cycle live without recompiling).
enum class GlyphFx : uint8_t {
  NONE,          // crisp letter, no distortion
  STATIC,        // TV-snow: pixel jitter + dropout + random noise
  BLOCK_GLITCH,  // sliced horizontal bands shifted/displaced
  WAVE,          // horizontal sine displacement
  SCANLINE,      // drop / noise-out random horizontal scanlines
  ZETA,          // Riemann-zeta warp + static noise
  COUNT          // (keep last; used by the cycle command)
};
constexpr GlyphFx WRITE_DISPLAY_EFFECT = GlyphFx::ZETA;  // active effect at boot

constexpr uint8_t  WRITE_DISPLAY_STATIC_KEEP  = 70;  // % of letter pixels kept (lower = noisier)
constexpr uint16_t WRITE_DISPLAY_STATIC_NOISE = 220; // count of random snow pixels added
constexpr int8_t   WRITE_DISPLAY_STATIC_JIT   = 1;   // positional jitter radius in px
constexpr uint8_t  WRITE_DISPLAY_BANDS        = 8;   // block-glitch band count
constexpr uint8_t  WRITE_DISPLAY_SCAN_DROP    = 12;  // % of scanlines fully dropped
constexpr uint8_t  WRITE_DISPLAY_SCAN_NOISE   = 12;  // % of scanlines replaced by noise
constexpr int8_t   WRITE_DISPLAY_ZETA_AMP     = 6;   // max zeta warp displacement in px

// -----------------------------------------------------------------------------
// IDLE DISPLAY ANIMATION (CRT-style static snow)
// -----------------------------------------------------------------------------
// While not drawing a letter, the OLED shows TV-snow. No per-pixel memory is
// used: each tick we stochastically flip pixels in the existing display buffer,
// which settles at a steady "on" density. Lower *_OFF = longer-lived pixels.
constexpr uint16_t IDLE_NOISE_INTERVAL_MS = 40;   // ms between snow frames (~25 fps)
constexpr uint8_t  IDLE_NOISE_ON          = 12;  // % chance an OFF pixel flips ON
constexpr uint8_t  IDLE_NOISE_OFF         = 16;  // % chance an ON pixel flips OFF

// =============================================================================
// SERIAL DEBUG MACROS
// =============================================================================
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