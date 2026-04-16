/**
 * @file ColorSensor.h
 * @brief Class wrapper around the CLS16D24 color sensor.
 *
 * Handles hardware communication, white-balance correction, and
 * conversion to the fixed-point format used by Mozzi/FixMath.
 * Raw and scaled data are stored internally and accessed via getters.
 *
 * Typical usage:
 *
 *   ColorSensor colorSensor;
 *
 *   // in setup():
 *   colorSensor.begin(false);
 *   colorSensor.reset();
 *   colorSensor.enable();
 *   colorSensor.setGain(32, false);
 *   colorSensor.setResolutionAndConversionTime(0x02);
 *   colorSensor.setChannelEnabled(ColorChannels::RED,   true);
 *   colorSensor.setChannelEnabled(ColorChannels::GREEN, true);
 *   colorSensor.setChannelEnabled(ColorChannels::BLUE,  true);
 *
 *   // in updateControl() when the I2C timer fires:
 *   colorSensor.update();
 *   mappedGreen = autoGreenMap(colorSensor.getGreenFixed().asInt());
 */

#ifndef COLORSENSOR_H
#define COLORSENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <FixMath.h>
#include <CLS16D24.h>

// ---------------------------------------------------------------------------
// Supporting types — defined at file scope so main.cpp can reference them
// without qualifying with ColorSensor::
// ---------------------------------------------------------------------------

enum class ColorChannels : uint8_t
{
    RED   = 0,
    GREEN = 1,
    BLUE  = 2,
    CLEAR = 3,
    IR    = 4
};

struct ColorValues
{
    uint16_t red   = 0;
    uint16_t green = 0;
    uint16_t blue  = 0;
    uint16_t clear = 0;
    uint16_t IR    = 0;
};

struct FixedPointColorValues
{
    UFix<16, 0> redFixed   = 0;
    UFix<16, 0> greenFixed = 0;
    UFix<16, 0> blueFixed  = 0;
    UFix<16, 0> clearFixed = 0;
    UFix<16, 0> IRFixed    = 0;
};

// ---------------------------------------------------------------------------
// ColorSensor class
// ---------------------------------------------------------------------------

class ColorSensor
{
public:
    ColorSensor();

    /**
     * Initializes the I2C bus and the underlying CLS16D24 driver.
     * Call this before reset() / enable().
     *
     * @param i2cFastMode  Use 400 kHz fast mode when true (default: false here
     *                     because the caller in main.cpp sets the clock
     *                     manually after init).
     * @param wirePort     I2C interface to use.
     * @return true on success (currently always true; kept for API parity
     *         with CLS16D24::begin()).
     */
    bool begin(bool i2cFastMode = false, TwoWire &wirePort = Wire);

    void reset();
    void enable();

    /**
     * @param gain             One of: 1, 4, 8, 32, 96.
     * @param doubleSensorArea Doubles the photodiode sensing area when true.
     */
    void setGain(uint8_t gain, bool doubleSensorArea = true);

    /**
     * Sets both resolution and conversion time from a single byte.
     * See CLS16D24.h for the full table of valid values and their effects.
     *
     * Both the total time it takes to convert a color reading into values and the resolution of that data are set
     * by the value of a single byte, which you can set with setResolutionAndConversionTime(). The resolution and
     * conversion time are interdependent, so it's difficult to know what time and resolution you are going to get
     * by the value you set the byte to. Open CLS16D24.h, and at the bottom you will find a large comment that shows
     * all possible values for conversion time and resolution.
     *
     * The default value of 0x02 yields these results:
     * COMMAND	CLS_CONV	INT_TIME	Calculated Resolution (maximum possible value)	Calculated Resolution (in bits)	    Calculated Conversion Time (in milliseconds)
     * 0x02	    0	        2	        16383	                                          14	                                36.8942
     *
     * I chose this as a good general setting because it offers a lot of color resolution (exactly 14 bits worth) and a reasonable conversion time of about 37ms.
     * It might actually make more sense thought to use 0x00, because that reduces resolution to 10 bits, but also drops conversion time to 5.89ms.
     * I'm pretty much always squashing the values to even lower resolution later in the code, so dropping to a lower resolution straight from the sensor could be
     * a good thing to try out.
     */
    void setResolutionAndConversionTime(uint8_t time);

    // -----------------------------------------------------------------------
    // Core operation
    // -----------------------------------------------------------------------

    /**
     * Reads all five channels from hardware and immediately applies the
     * white-balance correction scaling, storing both raw and scaled results
     * internally. Call this whenever the I2C update timer fires.
     */
    void update();

    // -----------------------------------------------------------------------
    // Raw data getters (uint16_t, straight from hardware)
    // -----------------------------------------------------------------------

    uint16_t getRed()   const { return _raw.red;   }
    uint16_t getGreen() const { return _raw.green; }
    uint16_t getBlue()  const { return _raw.blue;  }
    uint16_t getClear() const { return _raw.clear; }
    uint16_t getIR()    const { return _raw.IR;    }

    /**
     * Direct access to the raw struct, useful if you need all five channels
     * at once without five separate function calls.
     */
    const ColorValues &getRawData() const { return _raw; }

    // -----------------------------------------------------------------------
    // White-balance-corrected fixed-point getters (UFix<16,0>)
    //
    // Scaling applied:
    //   red   *= 1.75  (approximated as (raw * 7) >> 2)
    //   green  = raw   (reference channel; most sensitive)
    //   blue  *= 2.4   (approximated as (raw * 157286) >> 16)
    //   clear  = raw   (unscaled)
    //   IR    *= 8     (raw << 3)
    // -----------------------------------------------------------------------

    UFix<16, 0> getRedFixed()   const { return _scaled.redFixed;   }
    UFix<16, 0> getGreenFixed() const { return _scaled.greenFixed; }
    UFix<16, 0> getBlueFixed()  const { return _scaled.blueFixed;  }
    UFix<16, 0> getClearFixed() const { return _scaled.clearFixed; }
    UFix<16, 0> getIRFixed()    const { return _scaled.IRFixed;    }

    /**
     * Direct access to the scaled struct, for callers that need all channels
     * or prefer struct-style access over individual getters.
     */
    const FixedPointColorValues &getScaledData() const { return _scaled; }

    // -----------------------------------------------------------------------
    // Sensor configuration queries (delegated to CLS16D24)
    // -----------------------------------------------------------------------

    float    getConversionTimeMillis();
    uint16_t getResolution()          ;

    // -----------------------------------------------------------------------
    // Debug printing
    // -----------------------------------------------------------------------

    /**
     * Enables or disables a channel for printColorData() output.
     * All channels are disabled by default.
     */
    void setChannelEnabled(ColorChannels ch, bool enabled);

    /**
     * Returns true if specified channel is enabled.
     */
    bool getChannelEnabled(ColorChannels ch);

    /**
     * Prints the mapped (post-AutoMap) values for channels whose printing
     * has been enabled via setChannelEnabled(). Clear and IR channels print
     * their scaled fixed-point values since no mapped equivalent exists here.
     *
     * The mapped values are passed in rather than stored here because they
     * are produced by AutoMap objects that live in updateControl().
     *
     * Respects the USE_SERIAL flag in Configuration.h; compiles to nothing
     * when serial output is disabled.
     */
    void printColorData() const;

private:
    CLS16D24              _sensor;
    ColorValues           _raw;
    FixedPointColorValues _scaled;
    bool                  _channelEnabled[5];

    /**
     * Applies white-balance multipliers to _raw and writes results into
     * _scaled. Called automatically by update().
     */
    void _applyScaling();
};

#endif // COLORSENSOR_H
