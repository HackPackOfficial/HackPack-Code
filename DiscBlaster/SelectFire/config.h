#pragma once
#include <Arduino.h>

// ---------- Screen Characteristics -----------
// 1 = Calibrated triangle, 2 = Circle, 3 = Double chevron, 4 = Aircraft HUD,  5 = Biohazard
#define RETICLE_STYLE 4       // [1 - 5]
#define RETICLE_X 64          // [0 - 128] crosshair x-coords 
#define RETICLE_Y 10           // [9 - 55] y - coords
#define OLED_REFRESH_MS 50UL

#define BOOT_ANIMATION 2  
#define ANIMATION_STEPS 10   

#define RPM_METER_X 10
#define RPM_METER_Y 38

#define SHOT_COUNTER_X 10
#define SHOT_COUNTER_Y 51

// ---------- Firing controls ----------
#define MOTOR1_SPEED 150     // [140 - 220] back Flywheel speed
#define MOTOR2_SPEED 160     // [140 - 255] front Flywheel Speed

// ---------- PID flywheel control ----------
#define MOTOR1_MIN_RUNNING_PWM 140
#define MOTOR2_MIN_RUNNING_PWM 140
#define DEFAULT_MOTOR1_SPEED 150
#define DEFAULT_MOTOR2_SPEED 160
#define RPM_AT_PWM_150 5100.0f
#define RPM_AT_PWM_180 6050.0f
#define RPM_AT_PWM_220 7150.0f
#define RPM_AT_PWM_255 8350.0f
#define RPM_KP 0.0400f
#define RPM_KI 0.0060f
#define RPM_KD 0.0010f
#define RPM_CONTROL_MS 100UL
#define RPM_INTEGRAL_LIMIT 5000.0f

#define SERVO_REST_ANGLE 0
#define SERVO_FIRE_ANGLE 180
#define FULL_AUTO_PUSH_MS 125                 // [100 - 500] how long servo pushes on a shot
#define FULL_AUTO_RETURN_MS 130               // [100 - 500] how long servo retracts on a shot
#define M1_TRIGGER_BOOST_MS 500UL             // [0 - 1000] milliseconds to boost M1 speed after shot
#define M1_RECOVERY_MS_PER_PWM 0UL            // [0 - 20] milliseconds of delay added whem m1 speed is slowed by 1 unit

// ---------- Power button state control ----------
#define POWER_BUTTON_DOUBLE_TAP_MS 300UL
#define POWER_BUTTON_DEBOUNCE_MS 40UL
#define POWER_BUTTON_LONG_PRESS_MS 1000UL
#define WARNING_AUTO_ADVANCE_MS 5000UL

// ---------- RPM ----------
#define pulsesPerRev 2

// ---------- Lifetime shot counter ----------
#define SHOT_RPM_DROP_THRESHOLD 35.0f
#define SHOT_MIN_VALID_PRE_RPM 1000.0f
#define SHOT_EEPROM_MAGIC 0xB157
#define SHOT_EEPROM_SLOTS 64
#define SHOTS_PER_EEPROM_SAVE 100

struct __attribute__((packed)) ShotRecord {
  uint16_t magic;
  uint32_t sequence;
  uint32_t count;
  uint16_t checksum;
};

// ---------- Battery ----------
#define ADC_REF_BUCK 4.80f              // [0.0 - 5.0] Adjsut to measured value between 5V and GND on arduino
#define ADC_REF_USB_SAFE 4.03f          // [0.0 - 5.0] Adjsut to measured value between 5V and GND on arduino
#define BATTERY_DIVIDER_RATIO 2.491f    // from voltage divider resistor values
#define R_TOP 1000000.0f
#define R_BOTTOM 680000.0f
#define BAT_EMPTY 6.0f                  // safety shutoff at 6V or 3.0V / cell
#define BAT_FULL 8.4f
#define BATTERY_UPDATE_MS 500UL

// ---------- LED animation ----------
//#define PIXEL_0_IS_REAR true
//#define LED_UPDATE_MS 20UL
