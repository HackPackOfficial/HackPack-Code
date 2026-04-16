#ifndef TABLE_MANAGER_H
#define TABLE_MANAGER_H

#include <Arduino.h>
#include <Crunchlabs_DRV8835.h>
#include <AS5600.h>

/**
 * @brief Manages all state and behavior for the turntable.
 *
 * Encapsulates the table's motor driver, magnetic encoder, angular position
 * tracking, and the stop-detection algorithm that cuts motor power when the
 * table is held in place by the user.
 *
 * The stop-detection works by watching for two consecutive encoder update
 * cycles where the table's angular displacement is at or below the noise
 * threshold. Once stopped, the motor stays off until the table moves
 * appreciably again (indicating the user has released it).
 *
 * Typical usage:
 *
 *   TableManager table;
 *
 *   // in setup():
 *   table.begin();
 *
 *   // in updateControl(), every cycle, before the I2C timer block:
 *   table.updateTargetSpeed(mozziAnalogRead<10>(POT_B_PIN));
 *
 *   // in updateControl(), when the I2C timer fires for the table encoder:
 *   table.updateAngle();
 *
 *   // in updateControl(), after the I2C timer block:
 *   table.applySpeed();
 */
class TableManager
{
public:
    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    TableManager(uint8_t speedPin, uint8_t dirPin);

    /**
     * @brief Initializes the table encoder. Call in setup().
     */
    void begin();

    // -------------------------------------------------------------------------
    // Per-cycle updates
    // -------------------------------------------------------------------------

    /**
     * @brief Updates the target motor speed from a 10-bit ADC pot reading.
 *
     * If the table has been stopped by the user, this is a no-op — the target
     * speed stays at zero until the table moves again (detected in updateAngle()).
     * Call this every updateControl() cycle.
     *
     * @param potVal  Raw 10-bit ADC reading from the speed potentiometer.
     */
    void updateTargetSpeed(int16_t potVal);

    /**
     * @brief Reads the encoder, updates the internal angle, and runs the
     *        stop-detection algorithm.
     *
     * Call this once per I2C update cycle (when the I2C timer fires and the
     * table encoder is the active sensor).
     */
    void updateAngle();

    /**
     * @brief Applies the current target speed to the motor.
     *
     * Call this after the I2C sensor update block, every updateControl() cycle.
     */
    void applySpeed();

    // -------------------------------------------------------------------------
    // Input mapping
    // -------------------------------------------------------------------------

    /**
     * @brief Maps a 10-bit ADC pot reading to a motor speed in [-255, 255].
     *
     * Applies a center deadband around the detent position to prevent the table
     * from creeping when the pot is at rest.
     *
     * @param potVal  Raw ADC reading (signed to accept mozziAnalogRead output directly).
     * @return        Signed speed value: negative = reverse, positive = forward.
     */
    int16_t convertPotValToSpeed(int16_t potVal) const;

    // -------------------------------------------------------------------------
    // Getters
    // -------------------------------------------------------------------------

    /// Current cumulative table angle in encoder counts.
    int32_t getAngle()   const { return _currentAngle;  }

    /// True if the table is currently being held (stop-detection triggered).
    bool    isStopped()  const { return _tableStopped;  }

    /// Current target motor speed (useful for read-back / debugging).
    int16_t getTargetSpeed() const { return _targetSpeed; }

private:
    DRV8835 _motor;
    AS5600L _encoder;

    int32_t _currentAngle;
    int32_t _lastAngle;
    int16_t _targetSpeed;
    bool    _tableStopped;
    uint8_t _stoppedCount;

    // Stop-detection parameters
    static constexpr uint8_t _STOP_ANGLE_THRESHOLD = 5;    ///< counts; smaller change = "not moving"
    static constexpr uint8_t _STOP_COUNT_THRESHOLD = 2;    ///< consecutive low-change reads before declaring stopped
    static constexpr uint8_t _RESUME_ANGLE_THRESHOLD = 5;  ///< counts; larger change = "user released the table"
};

#endif // TABLE_MANAGER_H
