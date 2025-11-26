#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

#include <Arduino.h>

// motor control pins
constexpr int LEFT_SPEED_PIN = 6;
constexpr int LEFT_DIR_PIN = 7;
constexpr int RIGHT_SPEED_PIN = 5;
constexpr int RIGHT_DIR_PIN = 4;
constexpr int SERVO_PIN = 9;

constexpr int PARKING_BRAKE_PIN = 10;

// sensor pin definitions
// I recently learned that modern C++ style is to do const int instead of #define for pin values because it
// is better for type safety and debugging, and can allow more optimizations on the part of the compiler.
constexpr int LEFT_LIGHT_SENSOR_PIN = A3;
constexpr int MOISTURE_SENSOR_PIN = A2;
constexpr int RIGHT_LIGHT_SENSOR_PIN = A1;

// switch pins
constexpr int L_SWITCH = 3;  // left object detection switch
constexpr int R_SWITCH = 2;  // right side object detection switch

#endif // PIN_DEFINITIONS_H