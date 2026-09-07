#ifndef HARDWARE_MAP_H
#define HARDWARE_MAP_H

#include <Arduino.h>

// =============================================================================
// PIN DEFINITIONS
// =============================================================================
// Servo control pins (PWM signals). Two arms tilt the magnet; one raises it.
constexpr uint8_t PIN_SERVO_LEFT      = 4;
constexpr uint8_t PIN_SERVO_MAGNET    = 5;
constexpr uint8_t PIN_SERVO_RIGHT     = 3;
// Addressable LED (light) strip pin.
constexpr uint8_t PIN_LEDS            = 6;
// Joystick parts: a button and two analog axes (X and Y).
constexpr uint8_t PIN_JOYSTICK_BUTTON = 10;
constexpr uint8_t PIN_JOY_X           = 0;
constexpr uint8_t PIN_JOY_Y           = 1;

// =============================================================================
// DISPLAY CONFIGURATION
// =============================================================================
// Display size in pixels and its I2C bus address.
// I2C is a two-wire serial bus. OLED_RESET = -1 means the reset pin is not used.
constexpr uint8_t SCREEN_WIDTH    = 128;
constexpr uint8_t SCREEN_HEIGHT   = 64;
constexpr uint8_t SCREEN_I2C_ADDR = 0x3C;
constexpr int8_t  OLED_RESET      = -1;


#endif // HARDWARE_MAP_H