#ifndef PIN_ASSIGNMENTS_H
#define PIN_ASSIGNMENTS_H
#include <Arduino.h>

constexpr uint8_t 
    LED_PIN = 11,
    POT_A_PIN    = A0,
    POT_B_PIN    = A1,
    BUTTONS_PIN  = A3,
    ARM_MOTOR_SPEED_PIN = 6,
    ARM_MOTOR_DIR_PIN   = 7,
    TABLE_MOTOR_SPEED_PIN = 5,
    TABLE_MOTOR_DIR_PIN   = 4;

#endif