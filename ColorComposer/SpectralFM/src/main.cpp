/**
 * Color Composer: Spectral FM Hack.
 * This hack uses the color data channels to drive the parameters of a frequency modulation (FM)
 * synthesis engine. FM synthesis uses oscillators and both carriers and modulators, where the
 * modulator is used to change the frequency of the carrier. This is a really powerful synthesis
 * technique that can sound incredible or terrible, depending on how you set it up. This version
 * uses a single carrier oscillator being modulated by one modulator oscillator. Be sure to give
 * the table at least one full revolution to let the autoranging filters adjust. Until that happens,
 * this might sound kind of awful.
 */

#include <Arduino.h>
#include <Crunchlabs_DRV8835.h>
#include <Wire.h>
#include <PWMFreak.h>
#include <Mozzi.h>
#include <Oscil.h>
#include <IntMap.h>
#include <EventDelay.h>
#include <mozzi_utils.h>
#include <mozzi_midi.h>
#include <AS5600.h>
#include <AutoMap.h>

// Look in these files to change software and hardware configurations
#include <Configuration.h>
#include <PinAssignments.h>


/**
 * @brief Timer for staggering I2C sensor updates.
 *
 * Sensor updates block the processor, interfering with audio generation.
 * Staggering updates across three sensors at I2C_UPDATE_INTERVAL intervals
 * reduces audio glitching while maintaining acceptable sensor responsiveness.
 */
EventDelay k_i2cUpdateDelay;


// **********************************************************************************
// Arm and Table
// **********************************************************************************

// ArmManager must be included AFTER <Mozzi.h> because it uses EventDelay,
// which depends on Mozzi internals. Header-only by design for this reason.
#include <ArmManager.h>
#include <TableManager.h>

ArmManager arm(ARM_MOTOR_SPEED_PIN, ARM_MOTOR_DIR_PIN);
TableManager table(TABLE_MOTOR_SPEED_PIN, TABLE_MOTOR_DIR_PIN);


// **********************************************************************************
// Color Sensor
// **********************************************************************************

#include <ColorSensor.h>
ColorSensor colorSensor;

// Color channel data mapped into 0-255 uint8_t control values via AutoMap.
uint8_t mappedGreen = 0, mappedBlue = 0, mappedRed = 0, mappedWhite = 0;

// Two LEDs on the color sensor PCB can illuminate the viewing area.
constexpr uint8_t NUM_BRIGHTNESS_LEVELS = 5;
uint8_t brightnessIterator = 3;

/**
 * @brief Compile-time selection between array-based and bit-manipulation brightness lookup.
 *
 * Set to 1 for the array method (easier modification, minor RAM cost from PROGMEM table).
 * Set to 0 for the bit-manipulation method (zero RAM cost, equivalent functionality).
 * See Footnotes.md section [[#how LED brightness PWM values are calculated]] for derivation.
 */
#define USE_BRIGHTNESS_ARRAY 1

#if USE_BRIGHTNESS_ARRAY
const uint8_t PROGMEM LEDBrightnessLevels[NUM_BRIGHTNESS_LEVELS] = {0, 31, 63, 191, 255};

/**
 * @brief Retrieves LED brightness PWM value from PROGMEM lookup table.
 * @param level Brightness level index (0-4). Default is 3.
 * @return PWM duty cycle (0-255).
 */
inline uint8_t getBrightness(uint8_t level = 3)
{
    return pgm_read_byte(&LEDBrightnessLevels[level]);
}
#else
constexpr uint16_t LEDBrightnessSequence = 0b1000011000100001;

/**
 * @brief Computes LED brightness PWM value via bit extraction from packed constant.
 * @param level Brightness level index (0-4). Default is 3.
 * @return PWM duty cycle: 0 for level 0, otherwise ((nibble << 5) - 1).
 */
inline uint8_t getBrightness(uint8_t level = 3)
{
    return static_cast<uint8_t>(
        (level == 0) ? 0 : ((((LEDBrightnessSequence >> (4 * (level - 1))) & 0xF) << 5) - 1)
    );
}
#endif


// **********************************************************************************
// Music Generation Control
// **********************************************************************************

#include <MusicTypes.h>

// IntMaps for quantizing a color channel value (0-255) to a scale degree index.
const IntMap colorToScaleNote7(0, 256, 0, 7);
const IntMap colorToScaleNote5(0, 256, 0, 5);

// Manages the three buttons below the potentiometers. The buttons are all connected
// to a single analog input pin, so they need some special sauce to work.
#include <AnalogButtons.h>

// 255 means no button is pressed. 0, 1, or 2 corresponds to B0, B1, B2 pressed.
uint8_t buttonPressed = 255;


// **********************************************************************************
// FM Synthesis State
// **********************************************************************************

/**
 * Peak phase deviation written by updateFM() at control rate, read by
 * updateAudio() at audio rate. On AVR, int32_t writes are not atomic, so
 * there is a theoretical race condition between the two; in practice a single
 * sample of corruption is inaudible at audio rate and the risk is accepted here
 * as it is throughout the rest of the Mozzi architecture.
 */
int32_t gDeviation = 0;

/**
 * When false (default), the C:M ratio is constrained to integers 1-8,
 * producing tonal, harmonically rich spectra. When true, fractional ratios
 * in the range 1.0-4.0 are used, placing FM sidebands at inharmonic
 * frequencies — bell-like and metallic. Toggled by button B2.
 */
bool inharmonicMode = false;

// Function prototype — definition follows loop().
void updateFM();


// **********************************************************************************
// Setup
// **********************************************************************************

/**
 * @brief Initializes hardware peripherals, sensors, and Mozzi audio engine.
 *
 * Sequence matters: I2C sensors require initialization before clock speed changes,
 * and arm homing uses delay() which becomes unreliable after PWM frequency modification.
 */
void setup()
{
  SERIAL_BEGIN(115200);
  SERIAL_PRINTLN("starting");

  Wire.begin();

  // Initialize the table encoder and arm encoder, then home the arm.
  // home() must be called before Mozzi starts and before PWM clock divisors
  // are changed, because it uses standard delay(). table.begin() resets the
  // table encoder to 0 at the current position.
  table.begin();
  arm.begin();
  arm.home();

  colorSensor.begin(false);

  // IMPORTANT: Set the I2C clock to 400kHz fast mode AFTER initializing the
  // connection to the sensors.
  Wire.setClock(400000);

  colorSensor.reset();
  colorSensor.enable();
  // Possible gain settings are 1, 4, 8, 32, 96. Setting the second parameter
  // to true doubles the diode sensing area.
  colorSensor.setGain(32, false);

  // See the end of CLS16D24.h for a complete table of possible values.
  colorSensor.setResolutionAndConversionTime(0x02);
  SERIAL_PRINT("Conversion time: ");
  SERIAL_PRINTLN(colorSensor.getConversionTimeMillis());
  SERIAL_PRINT("Resolution: ");
  SERIAL_PRINTLN(colorSensor.getResolution());

  colorSensor.setChannelEnabled(ColorChannels::RED,   true);
  colorSensor.setChannelEnabled(ColorChannels::GREEN, true);
  colorSensor.setChannelEnabled(ColorChannels::BLUE,  true);
  colorSensor.setChannelEnabled(ColorChannels::CLEAR, true);
  colorSensor.setChannelEnabled(ColorChannels::IR,    false);

  // Change the PWM frequency on the motor control pins to push it above audible
  // range. After this line, millis() and delay() are unreliable.
  if (USE_FAST_PWM)
  {
    setPwmFrequency(5, 1);
  }

  if (USE_LED_PWM)
  {
    setPwmFrequency(LED_PIN, 1);
    analogWrite(LED_PIN, getBrightness(brightnessIterator));
  }
  else
  {
    digitalWrite(LED_PIN, HIGH);
  }

  // Prime the FM oscillators. The carrier starts on the root note of the first
  // scale; the modulator starts at a 1:2 ratio (one octave up), which gives a
  // clean, recognizable starting timbre. The mod-index LFO starts slow.
  float baseHz = mtof(scaleContainer.selected().getNote(0));
  aCarrier.setFreq(baseHz);
  aModulator.setFreq(baseHz * 2.0f);
  kModIndex.setFreq(0.5f);  // 0.5 Hz; updateFM() will take over from the first cycle

  k_i2cUpdateDelay.set(I2C_UPDATE_INTERVAL);

  startMozzi(MOZZI_CONTROL_RATE);
}


// **********************************************************************************
// updateControl
// **********************************************************************************

/**
 * @brief Mozzi control-rate callback for sensor polling and state updates.
 *
 * Called at MOZZI_CONTROL_RATE Hz. Handles button debouncing, sensor updates,
 * arm/table positioning, and invokes the FM parameter update. I2C reads are
 * staggered to minimize interference with audio synthesis.
 */
void updateControl()
{
  static int8_t targetArmPos = 80;
  static bool initialize = true, nextButtonPressAllowed = true;
  static EventDelay buttonTimer;
  if (initialize)
  {
    buttonTimer.set(250);
    initialize = false;
  }

  // Auto-ranging maps for the color channels. Static so they persist across
  // calls and keep tracking the observed min/max.
  static AutoMap autoGreenToUINT8_T(0, colorSensor.getResolution(), 0, 255);
  static AutoMap autoBlueToUINT8_T(0,  colorSensor.getResolution(), 0, 255);
  static AutoMap autoRedToUINT8_T(0,   colorSensor.getResolution(), 0, 255);
  static AutoMap autoWhiteToUINT8_T(0, colorSensor.getResolution(), 0, 255);

  // Check for button presses. Returns 0, 1, 2, or 255 (no press).
  buttonPressed = getButtonPressed(mozziAnalogRead<10>(BUTTONS_PIN));
  if (nextButtonPressAllowed && buttonPressed < 255)
  {
    buttonTimer.start();
    nextButtonPressAllowed = false;
    switch (buttonPressed)
    {
    case 0: // left button — LED brightness levels
      brightnessIterator = (brightnessIterator + 1) % NUM_BRIGHTNESS_LEVELS;
      analogWrite(LED_PIN, getBrightness(brightnessIterator));
      break;

    case 1: // middle button — scale selector
      scaleContainer.nextScale();
      SERIAL_TABS(2);
      SERIAL_PRINT("scale: ");
      SERIAL_PRINTLN(scaleContainer.scaleSelector);
      break;

    case 2: // right button — toggle harmonic / inharmonic C:M ratio mode
      inharmonicMode = !inharmonicMode;
      break;

    default:
      break;
    }
  }

  if (!nextButtonPressAllowed && buttonTimer.ready())
  {
    nextButtonPressAllowed = true;
  }

  // Update the table's target speed from the pot.
  table.updateTargetSpeed(mozziAnalogRead<10>(POT_B_PIN));

  static uint8_t currentSensorToUpdate = 0;

  if (k_i2cUpdateDelay.ready())
  {
    switch (currentSensorToUpdate)
    {
    case 0: // update the arm encoder
      arm.updatePosition();
      currentSensorToUpdate++;
      break;

    case 1: // update the table encoder; stop-detection runs inside updateAngle()
      table.updateAngle();
      currentSensorToUpdate++;
      break;

    case 2: // update the color sensor
      colorSensor.update();
      colorSensor.printColorData();
      currentSensorToUpdate = 0;
      break;

    default:
      currentSensorToUpdate = 0;
      break;
    }
    k_i2cUpdateDelay.start();
  }

  // Move the arm to the position requested by the potentiometer.
  targetArmPos = arm.convertPotValToRadius(mozziAnalogRead<10>(POT_A_PIN));
  arm.moveToAngle(arm.radiusToAngle(targetArmPos));

  // Commit the table motor speed.
  table.applySpeed();

  // Map raw color sensor counts into 0-255 control values.
  mappedGreen = autoGreenToUINT8_T(colorSensor.getGreenFixed().asInt());
  mappedBlue  = autoBlueToUINT8_T(colorSensor.getBlueFixed().asInt());
  mappedRed   = autoRedToUINT8_T(colorSensor.getRedFixed().asInt());
  mappedWhite = autoWhiteToUINT8_T(colorSensor.getClearFixed().asInt());

  // Derive FM synthesis parameters from the current color readings.
  updateFM();
}


// **********************************************************************************
// updateAudio
// **********************************************************************************

/**
 * @brief Mozzi audio-rate callback — the hot path.
 *
 * Called at audio rate (~16384 Hz on the ATmega328P). Advances the modulator
 * oscillator, scales its output by the deviation computed in updateFM(), and
 * uses the result as a phase offset for the carrier via Mozzi's phMod().
 * Everything here must be integer arithmetic; no function calls with unknown cost.
 *
 * The modulation calculation mirrors the classic FM/PM synthesis formula:
 *   output = carrier( θ + β * sin(ωm * t) )
 * where β (modulation index) is encoded in gDeviation and the modulator
 * provides the sin(ωm * t) term.
 */
AudioOutput updateAudio()
{
  int32_t modulation = (gDeviation * aModulator.next()) >> 8;
  return MonoOutput::from8Bit(aCarrier.phMod(modulation));
}


// **********************************************************************************
// Loop
// **********************************************************************************

/**
 * @brief Main Arduino loop — delegates entirely to Mozzi's audio hook.
 */
void loop()
{
  audioHook();
}


// **********************************************************************************
// updateFM
// **********************************************************************************

/**
 * @brief Derives FM synthesis parameters from the current color sensor readings.
 *
 * Called once per updateControl() cycle (MOZZI_CONTROL_RATE Hz). Writes to
 * gDeviation, aCarrier, aModulator, and kModIndex. The channel-to-parameter
 * assignments are defined in Configuration.h; see the comments there for rationale.
 *
 *   Green  → carrier pitch (scale-quantized note selection)
 *   Blue   → C:M ratio (harmonic content / timbre class)
 *   Red    → modulation index (timbral brightness and complexity)
 *   White  → mod-index LFO rate (speed of timbral animation)
 *
 * Button B1 cycles the active scale; pitch stays in-key regardless of which
 * green value is incoming. Button B2 toggles harmonic vs. inharmonic C:M ratios.
 */
void updateFM()
{
  // -----------------------------------------------------------------------
  // Carrier frequency — green channel selects a scale degree.
  // -----------------------------------------------------------------------
  // The same note-selection logic used by the original ambience generator:
  // map the 0-255 channel value to an index into the current scale, then
  // convert the MIDI note number to Hz with mtof().
  uint8_t noteIndex = (scaleContainer.selected().numNotes == 7)
      ? colorToScaleNote7(FM_CARRIER_CHANNEL)
      : colorToScaleNote5(FM_CARRIER_CHANNEL);

  float carrier_hz = mtof(scaleContainer.selected().getNote(noteIndex));

  // -----------------------------------------------------------------------
  // C:M ratio — blue channel.
  // -----------------------------------------------------------------------
  // In FM synthesis, the ratio between the carrier and modulator frequencies
  // determines which partials appear in the output spectrum. Integer ratios
  // produce harmonic spectra (all partials are integer multiples of the
  // fundamental), while fractional ratios produce inharmonic spectra whose
  // partials don't align with the harmonic series — the hallmark of bell,
  // gong, and metallic timbres.
  //
  // Harmonic mode: FM_RATIO_CHANNEL >> 5 maps 0-255 to 0-7, indexing into
  // the table of integer ratios {1,2,3,4,5,6,7,8}.
  //
  // Inharmonic mode: FM_RATIO_CHANNEL / 85.0 adds 0.0-3.0 to a base ratio
  // of 1.0, giving a continuous sweep from 1.0 to ~4.0 with fine resolution.
  float cm_ratio;
  if (!inharmonicMode)
  {
    static const uint8_t harmonicRatios[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    cm_ratio = (float)harmonicRatios[FM_RATIO_CHANNEL >> 5];
  }
  else
  {
    cm_ratio = 1.0f + (FM_RATIO_CHANNEL / 85.0f);  // ~1.0 to ~4.0
  }

  float mod_hz = carrier_hz * cm_ratio;
  aCarrier.setFreq(carrier_hz);
  aModulator.setFreq(mod_hz);

  // -----------------------------------------------------------------------
  // Mod-index LFO rate — white channel.
  // -----------------------------------------------------------------------
  // FM_LFO_CHANNEL >> 3 maps 0-255 to 0-31. Scaled by 0.15 and offset by
  // 0.05 gives a range of 0.05-4.70 Hz. At the low end you get a slow
  // tidal breathing of the timbre; at the high end, metallic shimmer
  // approaching the lower boundary of audible pitch modulation.
  kModIndex.setFreq(0.05f + (FM_LFO_CHANNEL >> 3) * 0.15f);

  // -----------------------------------------------------------------------
  // Deviation (modulation depth) — red channel + LFO.
  // -----------------------------------------------------------------------
  // In phase-modulation FM synthesis, the deviation controls how far the
  // carrier phase is pushed on each sample. The relationship to the
  // classical modulation index β is approximately:
  //   β ≈ deviation / modulator_frequency
  // so scaling deviation by mod_hz keeps the timbral character consistent
  // across pitch changes — a deviation that sounds "medium bright" at A4
  // will sound equally bright at A5.
  //
  // FM_INDEX_CHANNEL >> 2 compresses 0-255 to 0-63, which becomes the
  // integer modulation index. The full range sweeps from a near-pure sine
  // (index ~0) through rich harmonics to a dense, aliasing metallic texture
  // (index ~63). Tune the >> 2 shift to taste: >> 3 halves the maximum
  // index for a subtler top end; >> 1 doubles it for more aggressive sounds.
  //
  // The LFO (kModIndex.next()) returns -128 to 127. Scaled by >> 8, it
  // applies ±50% sinusoidal variation to the base deviation, so the timbre
  // breathes at the LFO rate rather than staying static.
  uint16_t mod_hz_int   = (uint16_t)mod_hz;
  int32_t baseDeviation = (int32_t)mod_hz_int * (int32_t)(FM_INDEX_CHANNEL >> 2);
  int8_t  lfo           = kModIndex.next();         // advances the LFO; returns -128 to 127
  int32_t lfoMod        = (baseDeviation * lfo) >> 8;
  gDeviation = baseDeviation + lfoMod;
  if (gDeviation < 0) gDeviation = 0;
}