/**
 * @file RoboServo.h
 * @brief Servo control library for ESP32 family and ESP8266
 * 
 * @author dunknowcoding
 * @version 1.1.0
 * @license MIT
 * 
 * Supported boards: ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C6, ESP32-H2, ESP32-P4, ESP8266
 * 
 * Features:
 *   - Simple Arduino Servo-style API
 *   - Multiple servo support (up to 8 on ESP32, 6 on smaller variants)
 *   - 180°, 270°, 360° and custom angle servos
 *   - Configurable PWM frequency (40-400Hz)
 *   - Grouped servo control
 *   - Thread-safe channel allocation (ESP32)
 */

#ifndef ROBOSERVO_H
#define ROBOSERVO_H

#include <Arduino.h>

// =============================================================================
// Platform Detection
// =============================================================================

#if defined(ESP8266)
    #define ROBOSERVO_PLATFORM_ESP8266
#elif defined(ESP32)
    #define ROBOSERVO_PLATFORM_ESP32
#else
    #error "RoboServo library only supports ESP32 and ESP8266 boards"
#endif

// =============================================================================
// Configuration Constants
// =============================================================================

/** Maximum servos per variant */
#if defined(ROBOSERVO_PLATFORM_ESP8266)
    #define ROBOSERVO_MAX_SERVOS 8
    #define ROBOSERVO_PWM_RESOLUTION 10  ///< ESP8266: 10-bit resolution
#elif defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32H2)
    #define ROBOSERVO_MAX_SERVOS 6
    #define ROBOSERVO_PWM_RESOLUTION 14  ///< ESP32: 14-bit resolution
#elif defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
    #define ROBOSERVO_MAX_SERVOS 8
    #define ROBOSERVO_PWM_RESOLUTION 14
#elif defined(CONFIG_IDF_TARGET_ESP32P4)
    #define ROBOSERVO_MAX_SERVOS 8
    #define ROBOSERVO_PWM_RESOLUTION 14
#elif defined(CONFIG_IDF_TARGET_ESP32)
    #define ROBOSERVO_MAX_SERVOS 8
    #define ROBOSERVO_PWM_RESOLUTION 14
#else
    #define ROBOSERVO_MAX_SERVOS 6
    #define ROBOSERVO_PWM_RESOLUTION 14
#endif

/** Return value when servo attachment fails */
#define ROBOSERVO_INVALID_SERVO 255

/** PWM frequency settings */
#define ROBOSERVO_DEFAULT_FREQUENCY 50   ///< Standard servo frequency (Hz)
#define ROBOSERVO_MIN_FREQUENCY     40   ///< Minimum allowed frequency (Hz)
#define ROBOSERVO_MAX_FREQUENCY     400  ///< Maximum allowed frequency (Hz)
#define ROBOSERVO_LEDC_TIMER        0    ///< LEDC timer number (ESP32 only)

/** Default pulse width settings (microseconds) */
#define ROBOSERVO_DEFAULT_MIN_PULSE_US    500   ///< Minimum pulse (0°)
#define ROBOSERVO_DEFAULT_MAX_PULSE_US    2500  ///< Maximum pulse (max angle)
#define ROBOSERVO_DEFAULT_CENTER_PULSE_US 1500  ///< Center pulse (90°)
#define ROBOSERVO_PERIOD_US               20000 ///< PWM period at 50Hz

// =============================================================================
// Enumerations
// =============================================================================

/**
 * @brief Servo type enumeration
 * 
 * Use these to specify the rotation range of your servo:
 * - SERVO_TYPE_180: Standard servo (0-180°)
 * - SERVO_TYPE_270: Extended range servo (0-270°)
 * - SERVO_TYPE_360: Continuous rotation servo (speed control)
 * - SERVO_TYPE_CUSTOM: Custom angle range (use setMaxAngle())
 */
enum RoboServoType {
    SERVO_TYPE_CUSTOM = 0,   ///< Custom angle range
    SERVO_TYPE_180 = 180,    ///< Standard 180° servo
    SERVO_TYPE_270 = 270,    ///< Extended 270° servo
    SERVO_TYPE_360 = 360     ///< Continuous rotation servo
};

// =============================================================================
// RoboServo Class
// =============================================================================

/**
 * @class RoboServo
 * @brief Controls a single servo motor
 * 
 * Basic usage:
 * @code
 *   RoboServo servo;
 *   servo.attach(13);           // Attach to GPIO 13
 *   servo.write(90);            // Move to 90°
 *   servo.writeMicroseconds(1500);  // Or set pulse directly
 * @endcode
 * 
 * For 360° continuous rotation servos:
 * @code
 *   servo.attach(13, 500, 2500, SERVO_TYPE_360);
 *   servo.setSpeed(50);   // 50% forward
 *   servo.setSpeed(-50);  // 50% reverse
 *   servo.stop();         // Stop
 * @endcode
 */
class RoboServo {
public:
    RoboServo();
    ~RoboServo();

    // -------------------------------------------------------------------------
    // Attachment Methods
    // -------------------------------------------------------------------------
    
    /** @brief Attach servo with default settings (500-2500μs, 180°, 50Hz) */
    uint8_t attach(int pin);
    
    /** @brief Attach servo with custom pulse width range */
    uint8_t attach(int pin, int minPulseUs, int maxPulseUs);
    
    /** @brief Attach servo with pulse range and type */
    uint8_t attach(int pin, int minPulseUs, int maxPulseUs, RoboServoType servoType);
    
    /** @brief Attach servo with custom max angle */
    uint8_t attach(int pin, int minPulseUs, int maxPulseUs, int maxAngle);
    
    /** @brief Attach servo with full configuration including frequency */
    uint8_t attach(int pin, int minPulseUs, int maxPulseUs, RoboServoType servoType, int frequency);

    /** @brief Detach servo and release PWM channel */
    void detach();
    
    /** @brief Check if servo is attached */
    bool attached() const;

    // -------------------------------------------------------------------------
    // Position Control
    // -------------------------------------------------------------------------
    
    /** @brief Set servo angle (0 to maxAngle) */
    void write(int angle);
    
    /** @brief Set servo position by pulse width in microseconds */
    void writeMicroseconds(int pulseUs);
    
    /** @brief Get current angle */
    int read() const;
    
    /** @brief Get current pulse width in microseconds */
    int readMicroseconds() const;

    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------
    
    void setServoType(RoboServoType servoType);
    RoboServoType getServoType() const;
    
    void setPulseLimits(int minPulseUs, int maxPulseUs);
    int getMinPulse() const;
    int getMaxPulse() const;
    
    void setMaxAngle(int maxAngle);
    int getMaxAngle() const;
    
    void setFrequency(int frequency);  ///< Set PWM frequency (40-400Hz)
    int getFrequency() const;
    
    int getPin() const;
    uint8_t getChannel() const;

    // -------------------------------------------------------------------------
    // 360° Continuous Rotation Servo Methods
    // -------------------------------------------------------------------------
    
    /** @brief Stop servo rotation (send center pulse) */
    void stop();
    
    /** @brief Release servo (stop sending PWM signal) */
    void release();
    
    /** @brief Set rotation speed (-100 to +100) */
    void setSpeed(int speed);

    // -------------------------------------------------------------------------
    // Static Utilities
    // -------------------------------------------------------------------------
    
    static int getAttachedCount();
    static uint32_t getDefaultFrequency() { return ROBOSERVO_DEFAULT_FREQUENCY; }
    static uint8_t getServoResolution() { return ROBOSERVO_PWM_RESOLUTION; }

private:
    int _pin;
    uint8_t _channel;
    int _minPulseUs;
    int _maxPulseUs;
    int _currentPulseUs;
    RoboServoType _servoType;
    int _maxAngle;
    int _frequency;
    bool _attached;

    uint32_t microsecondsToTicks(int us) const;
    static uint8_t allocateChannel();
    static void releaseChannel(uint8_t channel);
    static void initTimer();
    static bool isValidPwmPin(int pin);
};

// =============================================================================
// RoboServoGroup Class
// =============================================================================

/**
 * @class RoboServoGroup
 * @brief Manage multiple servos as a coordinated group
 * 
 * Usage:
 * @code
 *   RoboServoGroup group;
 *   group.addServo(13);
 *   group.addServo(14);
 *   group.addServo(15);
 *   
 *   group.writeAll(90);  // All servos to 90°
 *   
 *   int angles[] = {0, 90, 180};
 *   group.writeMultiple(angles, 3);  // Each to different angle
 * @endcode
 */
class RoboServoGroup {
public:
    RoboServoGroup();
    ~RoboServoGroup();

    // -------------------------------------------------------------------------
    // Add/Remove Servos
    // -------------------------------------------------------------------------
    
    int addServo(int pin);
    int addServo(int pin, int minPulseUs, int maxPulseUs, RoboServoType servoType = SERVO_TYPE_180);
    bool removeServo(int index);
    void removeAll();

    // -------------------------------------------------------------------------
    // Group Information
    // -------------------------------------------------------------------------
    
    int count() const;
    RoboServo* getServo(int index);

    // -------------------------------------------------------------------------
    // Group Control (all servos)
    // -------------------------------------------------------------------------
    
    void writeAll(int angle);
    void writeAllMicroseconds(int pulseUs);
    void writeMultiple(const int* angles, int count);
    void writeMultipleMicroseconds(const int* pulses, int count);
    void detachAll();
    void stopAll();

    // -------------------------------------------------------------------------
    // Individual Servo Control
    // -------------------------------------------------------------------------
    
    void write(int index, int angle);
    void writeMicroseconds(int index, int pulseUs);
    int read(int index) const;

private:
    RoboServo* _servos[ROBOSERVO_MAX_SERVOS];
    int _count;
};

// =============================================================================
// Utility Functions
// =============================================================================

/**
 * @namespace RoboServoUtils
 * @brief Internal utility functions
 */
namespace RoboServoUtils {
    int mapValue(int value, int inMin, int inMax, int outMin, int outMax);
    int constrainValue(int value, int min, int max);
}

#endif // ROBOSERVO_H
