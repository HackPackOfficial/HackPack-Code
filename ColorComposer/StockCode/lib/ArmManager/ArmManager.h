#ifndef ARM_MANAGER_H
#define ARM_MANAGER_H

#include <Arduino.h>
#include <Crunchlabs_DRV8835.h>
#include <AS5600.h>
#include <EventDelay.h>

/**
 * @brief Manages all state and behavior for the sensor arm.
 *
 * Encapsulates the arm's motor driver, magnetic encoder, position tracking,
 * acceleration-controlled movement, and coordinate conversion between
 * millimeter radius and encoder angle counts.
 *
 * NOTE: This class is intentionally header-only. EventDelay depends on
 * Mozzi internals (MOZZI_AUDIO_RATE, audioTicks()) that are only available
 * after <Mozzi.h> is included. Because this header is always included from
 * main.cpp after Mozzi, the context is satisfied. A separate .cpp would
 * compile in isolation and lose that context.
 *
 * Typical usage:
 *
 *   ArmManager arm;
 *
 *   // in setup():
 *   arm.begin();
 *   arm.home();
 *
 *   // in updateControl(), when the I2C timer fires for the arm encoder:
 *   arm.updatePosition();
 *
 *   // also in updateControl(), every cycle:
 *   int8_t targetRadius = arm.convertPotValToRadius(mozziAnalogRead<10>(POT_A_PIN));
 *   arm.moveToAngle(arm.radiusToAngle(targetRadius));
 */
class ArmManager
{
public:
    // -------------------------------------------------------------------------
    // Pin assignments
    // -------------------------------------------------------------------------
    /// Maximum absolute radius the arm is allowed to move to during normal operation (mm).
    static constexpr uint8_t MAX_CONSTRAINED_RADIUS = 75;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    ArmManager(uint8_t speedPin, uint8_t dirPin)
        : _motor(speedPin, dirPin, 50, true),
          _currentAngle(0),
          _currentPosition(0),
          _angleLastSpeed(0),
          _radiusLastSpeed(0)
    {
        _angleAccelTimer.set(_ANGLE_ACCEL_INTERVAL);
        _radiusAccelTimer.set(_RADIUS_ACCEL_INTERVAL);
    }

    // -------------------------------------------------------------------------
    // Lifecycle
    // -------------------------------------------------------------------------

    void begin()
    {
        _encoder.begin();
        _encoder.setHysteresis(3);
    }

    /**
     * @brief Runs the homing sequence, driving the arm to its hard stop and
     *        resetting the encoder to the calibrated zero position.
     *
     * Must be called before Mozzi starts and before PWM clock divisors are
     * changed, since it relies on standard delay().
     */
    void home()
    {
        // Intentionally start lastPos and currentPos at different values so
        // the while loop always executes at least once.
        int16_t lastPos = 0, currentPos = 100;

        _motor.setSpeed(255);
        delay(100); // brief pause to take up gear backlash before watching for motion to stop

        while (currentPos != lastPos)
        {
            lastPos    = currentPos;
            currentPos = _encoder.getCumulativePosition();
            delay(100); // short debounce window between samples
        }

        _motor.setSpeed(0);

        // Shift the cumulative position so that 0 counts = arm centered over
        // the table. 661 is the empirically measured count at that position.
        _encoder.resetCumulativePosition(661);
    }

    // -------------------------------------------------------------------------
    // Per-cycle update
    // -------------------------------------------------------------------------

    /**
     * @brief Reads the encoder and updates the internal angle and position.
     *        Call once per sensor update cycle (when the I2C timer fires).
     */
    void updatePosition()
    {
        _currentAngle    = _encoder.getCumulativePosition();
        _currentPosition = static_cast<int8_t>(angleToRadius(_currentAngle));
    }

    // -------------------------------------------------------------------------
    // Motion control — call every updateControl() cycle
    // -------------------------------------------------------------------------

    /**
     * @brief Acceleration-controlled move to a target angle in encoder counts.
     *
     * Ramps motor speed up or down each time the accel timer fires.
     * Setting a target within the deadband brings the motor to a stop.
     */
    void moveToAngle(int16_t targetAngle)
    {
        int16_t newSpeed;
        int16_t displacement = targetAngle - _currentAngle;

        if (abs(displacement) <= _ANGLE_DEADBAND)
        {
            newSpeed = 0;
        }
        else if (_angleAccelTimer.ready())
        {
            int8_t dir = (displacement > 0) ? 1 : -1;
            newSpeed   = constrain(
                (_ANGLE_ACCEL_MULT * dir) + _angleLastSpeed,
                -static_cast<int16_t>(_ANGLE_MAX_SPEED),
                 static_cast<int16_t>(_ANGLE_MAX_SPEED));
            _angleAccelTimer.start();
        }
        else
        {
            // Timer hasn't fired yet — hold the current speed.
            newSpeed = _angleLastSpeed;
        }

        _motor.setSpeed(newSpeed);
        _angleLastSpeed = newSpeed;
    }

    /**
     * @brief Acceleration-controlled move to a target radius in mm.
     *
     * Converts the radius to an encoder angle internally, then runs its own
     * acceleration loop with faster accel and a lower top speed than moveToAngle().
     *
     * TODO: add a hard cutoff when the arm reaches the end of its range of
     * motion, since exceeding the valid domain breaks the angle<->radius
     * approximations and causes erratic behavior.
     */
    void moveToRadius(int8_t targetRadius)
    {
        int16_t targetAngle  = radiusToAngle(targetRadius);
        int16_t displacement = targetAngle - _currentAngle;
        int16_t newSpeed;

        if (abs(displacement) <= _RADIUS_DEADBAND)
        {
            newSpeed = 0;
        }
        else if (_radiusAccelTimer.ready())
        {
            int8_t dir = (displacement > 0) ? 1 : -1;
            newSpeed   = constrain(
                (_RADIUS_ACCEL_MULT * dir) + _radiusLastSpeed,
                -static_cast<int16_t>(_RADIUS_MAX_SPEED),
                 static_cast<int16_t>(_RADIUS_MAX_SPEED));
            _radiusAccelTimer.start();
        }
        else
        {
            newSpeed = _radiusLastSpeed;
        }

        _motor.setSpeed(newSpeed);
        _radiusLastSpeed = newSpeed;
    }

    // -------------------------------------------------------------------------
    // Coordinate conversions
    // -------------------------------------------------------------------------

    /**
     * @brief Converts a radius in mm to the encoder angle count that places
     *        the arm at that radius.
     *
     * Uses the integer linear approximation  angle = (774 * |r|) >> 7,
     * which approximates (4096 / pi) * arcsin(r / 216). Error is at most
     * 1 encoder count across the arm's usable range of motion.
     *
     * Bit-shift is performed on an unsigned value to avoid sign-bit corruption.
     */
    inline int16_t radiusToAngle(int16_t radiusMM) const
    {
        uint16_t absRadius    = static_cast<uint16_t>(abs(radiusMM));
        uint16_t intermediate = (774 * absRadius) >> 7;
        return (radiusMM < 0) ? -static_cast<int16_t>(intermediate)
                               :  static_cast<int16_t>(intermediate);
    }

    /**
     * @brief Converts an encoder angle count to the corresponding radius in mm.
     *
     * Uses the integer linear approximation  radius = (81 * |angle|) >> 9,
     * which approximates 216 * sin(angle * pi / 4096). Worst-case positional
     * error is 2 mm across the usable range of motion.
     *
     * Bit-shift is performed on an unsigned value to avoid sign-bit corruption.
     */
    inline int16_t angleToRadius(int16_t angleCounts) const
    {
        uint16_t absAngle     = static_cast<uint16_t>(abs(angleCounts));
        uint16_t intermediate = (81 * absAngle) >> 9;
        return (angleCounts < 0) ? -static_cast<int16_t>(intermediate)
                                 :  static_cast<int16_t>(intermediate);
    }

    /**
     * @brief Maps a 10-bit ADC pot reading to a target arm radius in mm.
     *
     * Applies a center deadband to suppress jitter around the mid-point detent,
     * then maps the full pot range to [-MAX_CONSTRAINED_RADIUS, MAX_CONSTRAINED_RADIUS].
     */
    int16_t convertPotValToRadius(uint16_t potVal) const
    {
        constexpr uint8_t DEADBAND = 15;
        if (potVal < static_cast<uint16_t>(512 + DEADBAND) &&
            potVal > static_cast<uint16_t>(512 - DEADBAND))
        {
            potVal = 512;
        }
        return map(static_cast<long>(potVal), 1023, 0,
                   MAX_CONSTRAINED_RADIUS, -static_cast<int16_t>(MAX_CONSTRAINED_RADIUS));
    }

    // -------------------------------------------------------------------------
    // Getters
    // -------------------------------------------------------------------------

    /// Current arm angle in encoder counts.
    int16_t getAngle()    const { return _currentAngle;    }

    /// Current arm position in mm from the center of the table.
    int8_t  getPosition() const { return _currentPosition; }

private:
    DRV8835 _motor;
    AS5600  _encoder;

    int16_t _currentAngle;
    int8_t  _currentPosition;

    // -- moveToAngle() acceleration state ------------------------------------
    static constexpr uint8_t  _ANGLE_ACCEL_MULT     = 1;
    static constexpr uint16_t _ANGLE_ACCEL_INTERVAL  = 2;   ///< ms between speed steps
    static constexpr uint8_t  _ANGLE_MAX_SPEED       = 140;
    static constexpr uint8_t  _ANGLE_DEADBAND        = 10;  ///< encoder counts

    EventDelay _angleAccelTimer;
    int16_t    _angleLastSpeed;

    // -- moveToRadius() acceleration state -----------------------------------
    static constexpr uint8_t  _RADIUS_ACCEL_MULT     = 4;
    static constexpr uint16_t _RADIUS_ACCEL_INTERVAL  = 1;  ///< ms between speed steps
    static constexpr uint8_t  _RADIUS_MAX_SPEED      = 128;
    static constexpr uint8_t  _RADIUS_DEADBAND       = 10;  ///< encoder counts

    EventDelay _radiusAccelTimer;
    int16_t    _radiusLastSpeed;
};

#endif // ARM_MANAGER_H
