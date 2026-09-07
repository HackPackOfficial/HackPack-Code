#ifndef SERVO_WRAPPER_H
#define SERVO_WRAPPER_H

#include <Arduino.h>
#include <RoboServo.h>

class ServoWrapper {
  private:
    const uint8_t _pin;
    const float _trim;                // trim offset in degrees
    float _currentPos{0.0f};          // filtered position in degrees (no trim applied)
    float _smoothing{0.0f};
    RoboServo _serv;

    static constexpr float MIN_PULSE_US = 544.0f;
    static constexpr float MAX_PULSE_US = 2400.0f;
    static constexpr float MIN_ANGLE = 0.0f;
    static constexpr float MAX_ANGLE = 180.0f;

    // Convert a degree value (with trim already added) to a pulse width in microseconds.
    // Clamps to [MIN_ANGLE, MAX_ANGLE] before mapping.
    uint16_t degreesToMicroseconds(float degrees) const;

  public:
    ServoWrapper(uint8_t p, float t = 0.0f)
      : _pin{p},
        _trim{t}
    {}

    // variable for storing the target position for the servo
    float targetPos{90.0f};

    // Attach the servo object to the pin specified in the constructor
    void attach();

    // Detach the servo from the pin
    void detach();

    // Get the current position of the servo in degrees (without trim)
    float getCurrentPos() const;

    // Get the smoothing coefficient used by moveTo()
    float getSmoothing() const;

    // Set the smoothing coefficient used by moveTo(). Range 0.0 to 1.0.
    // 0.0 = no smoothing (immediate move). 1.0 = fully damped (no movement).
    void setSmoothing(float val = 0.0f);

    // Immediately command the servo to a position in degrees. Applies trim internally
    // and writes microseconds to the hardware. Returns the position in degrees (without trim).
    float write(float target);

    // Command the servo toward a target position using an exponential smoothing filter.
    // Returns the current filtered position in degrees (without trim).
    float moveTo(float t);

    // Set the internal position WITHOUT driving the servo. Used to sync state after
    // the pin was driven externally (e.g. by the LEDC entry/exit driver).
    void syncCurrentPos(float pos) { _currentPos = pos; }
};

#endif // SERVO_WRAPPER_H
