#include <TableManager.h>

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------

TableManager::TableManager(uint8_t speedPin, uint8_t dirPin)
    : _motor(speedPin, dirPin, 50, true),
      _currentAngle(0),
      _lastAngle(0),
      _targetSpeed(0),
      _tableStopped(false),
      _stoppedCount(0)
{
}

// -----------------------------------------------------------------------------
// Lifecycle
// -----------------------------------------------------------------------------

void TableManager::begin()
{
    _encoder.begin();
    _encoder.resetCumulativePosition(); // calibrate to 0 at startup
    _encoder.setHysteresis(3);
}

// -----------------------------------------------------------------------------
// Per-cycle updates
// -----------------------------------------------------------------------------

void TableManager::updateTargetSpeed(int16_t potVal)
{
    // If the table has been stopped by the user, don't update the target speed.
    // It stays at 0 until updateAngle() detects the table moving again.
    if (!_tableStopped)
    {
        _targetSpeed = convertPotValToSpeed(potVal);
    }
}

void TableManager::updateAngle()
{
    _lastAngle    = _currentAngle;
    _currentAngle = _encoder.getCumulativePosition();

    int32_t angularDisplacement = abs(_currentAngle - _lastAngle);

    if (!_tableStopped)
    {
        // Check whether the table appears to have stopped moving.
        if (angularDisplacement <= _STOP_ANGLE_THRESHOLD)
        {
            _stoppedCount++;
        }

        // Two consecutive reads with negligible motion means we declare the
        // table stopped. The window for these two reads is ~2x the I2C update
        // interval (~180ms at the default update rate).
        if (_stoppedCount >= _STOP_COUNT_THRESHOLD)
        {
            _targetSpeed  = 0;
            _stoppedCount = 0;
            _tableStopped = true;
        }
    }
    else
    {
        // Table is stopped — watch for the user releasing it. A displacement
        // above the resume threshold means the table is moving again.
        if (angularDisplacement > _RESUME_ANGLE_THRESHOLD)
        {
            _tableStopped = false;
            // _targetSpeed will be refreshed by the next updateTargetSpeed() call.
        }
    }
}

void TableManager::applySpeed()
{
    _motor.setSpeed(_targetSpeed);
}

// -----------------------------------------------------------------------------
// Input mapping
// -----------------------------------------------------------------------------

int16_t TableManager::convertPotValToSpeed(int16_t potVal) const
{
    constexpr uint8_t DEADBAND = 40;
    if (potVal > static_cast<int16_t>(512 - DEADBAND) &&
        potVal < static_cast<int16_t>(512 + DEADBAND))
    {
        potVal = 512;
    }
    return map(static_cast<long>(potVal), 0, 1023, -255, 255);
}
