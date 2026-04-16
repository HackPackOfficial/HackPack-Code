#include <ColorSensor.h>
#include <Debug.h>

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

ColorSensor::ColorSensor()
{
    for (uint8_t i = 0; i < 5; i++)
        _channelEnabled[i] = false;
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

bool ColorSensor::begin(bool i2cFastMode, TwoWire &wirePort)
{
    return _sensor.begin(i2cFastMode, wirePort);
}

void ColorSensor::reset()
{
    _sensor.reset();
}

void ColorSensor::enable()
{
    _sensor.enable();
}

void ColorSensor::setGain(uint8_t gain, bool doubleSensorArea)
{
    _sensor.setGain(gain, doubleSensorArea);
}

void ColorSensor::setResolutionAndConversionTime(uint8_t time)
{
    _sensor.setResolutionAndConversionTime(time);
}

// ---------------------------------------------------------------------------
// Core operation
// ---------------------------------------------------------------------------

void ColorSensor::update()
{
    _sensor.readRGBWIR(_raw.red, _raw.green, _raw.blue, _raw.clear, _raw.IR);
    _applyScaling();
}

void ColorSensor::_applyScaling()
{
    // Red and blue channels are less sensitive than green on this sensor.
    // The multipliers below were derived empirically using a Spyder Checkr 24
    // color-calibration card: grey patches were used to find coefficients that
    // bring all three channels into agreement over neutral tones, so that the
    // dominant channel actually reflects the dominant color in the scene.
    //
    // All multiplications use integer arithmetic and bit shifts to avoid
    // the cost of floating-point operations on the AVR.
    //
    //   red   × 1.75  ≈ (raw × 7) >> 2
    //   green × 1.0   (reference, most sensitive channel, used as-is)
    //   blue  × 2.4   ≈ (raw × 157286) >> 16
    //   clear × 1.0   (unscaled ambient light proxy)
    //   IR    × 8     = raw << 3

    _scaled.redFixed   = UFix<16, 0>((_raw.red   * 7)              >> 2);
    _scaled.greenFixed = UFix<16, 0>(_raw.green);
    _scaled.blueFixed  = UFix<16, 0>(((uint32_t)_raw.blue * 157286UL) >> 16);
    _scaled.clearFixed = UFix<16, 0>(_raw.clear);
    _scaled.IRFixed    = UFix<16, 0>(_raw.IR << 3);
}

// ---------------------------------------------------------------------------
// Sensor configuration queries
// ---------------------------------------------------------------------------

float ColorSensor::getConversionTimeMillis() 
{
    return _sensor.getConversionTimeMillis();
}

uint16_t ColorSensor::getResolution() 
{
    return _sensor.getResolution();
}

// ---------------------------------------------------------------------------
// Debug printing
// ---------------------------------------------------------------------------

void ColorSensor::setChannelEnabled(ColorChannels ch, bool enabled)
{
    _channelEnabled[static_cast<uint8_t>(ch)] = enabled;
}

bool ColorSensor::getChannelEnabled(ColorChannels ch)
{
    return _channelEnabled[static_cast<uint8_t>(ch)];
}

void ColorSensor::printColorData() const
{
    bool first = true;

    // Lambda to print the separator between values. A local lambda here is
    // slightly tidier than repeating the conditional inline six times.
    // Note: this captures 'first' by reference so it updates across calls.
    auto printSep = [&first]()
    {
        if (!first) { SERIAL_PRINT("   "); }
        else        { first = false; }
    };

    if (_channelEnabled[static_cast<uint8_t>(ColorChannels::BLUE)])
    {
        printSep();
        SERIAL_PRINT(_raw.blue);
    }

    if (_channelEnabled[static_cast<uint8_t>(ColorChannels::CLEAR)])
    {
        printSep();
        SERIAL_PRINT(_raw.clear);
    }

    if (_channelEnabled[static_cast<uint8_t>(ColorChannels::IR)])
    {
        printSep();
        SERIAL_PRINT(_raw.IR);
    }

    if (_channelEnabled[static_cast<uint8_t>(ColorChannels::GREEN)])
    {
        printSep();
        SERIAL_PRINT(_raw.green);
    }

    if (_channelEnabled[static_cast<uint8_t>(ColorChannels::RED)])
    {
        printSep();
        SERIAL_PRINT(_raw.red);
    }

    SERIAL_PRINTLN();
}
