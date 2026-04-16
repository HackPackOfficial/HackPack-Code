/**
 * Color Composer: Pentatonics Hack
 * 
 * Very similar to the stock code, but with a few fun differences. First, Button 2 has a different
 * behavior now. When you press and hold button 2, the system stops updating the notes getting sent
 * to the oscillators, so as long as you're holding button 2, it will keep playing the same notes.
 * The moment you release it, it goes back to changing the notes based on color!
 * 
 * Honestly, this is barely a hack. Really, this is the same as the stock code, except
 * that it defaults to a different selection of musical scales to play with. It also removes
 * the occasional arpeggio that would get triggered when especially bright colors were detected.
 * I made this to be a bit more of an instrument. The scales that were chosen are all pentatonic,
 * which means that they basically always sound good. Pentatonic scales lack dissonant notes.
 * They offer fewer notes than other scales, but they are excellent for jamming.
 * 
 * Personally, I think this one is fun to just play with. Disengage the drive wheel for the table
 * and spin it by hand to particular colors/chords. Move the arm by hand, or gently push it down
 * or pull it up a bit to give it some vibrato. Press and hold button 2 to sustain notes while you 
 * spin the table to a new color to select your next chord.
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
#include <mozzi_rand.h>
#include <mozzi_midi.h>
#include <AS5600.h>
#include <AutoMap.h>
#include <Portamento.h>
#include <ADSR.h>

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

// where color data that has been mapped into control signals will be stored
uint8_t mappedGreen = 0, mappedBlue = 0, mappedRed = 0, mappedWhite = 0;

// Two LEDs on the color sensor PCB can illuminate the viewing area, so set those up.
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
// Mozzi Configuration
// **********************************************************************************

// Important configuration values are in Configuration.h

// **********************************************************************************
// Music Generation Control
// **********************************************************************************

// Include the MusicTypes header, which defines MIDI_NOTE and Chord.
#include <MusicTypes.h>
// Contains tools useful for converting music information into oscillator parameters and things
#include <OscillatorTools.h>

// create structs for storing parameters for each oscillator
oscillatorParams osc0Params, osc1Params, osc2Params;

// create portamento object so we can glide from one note to another
Portamento<MOZZI_CONTROL_RATE> osc2Portamento;

// create ADSR envelopes to make each note fade in and out.
ADSR<MOZZI_CONTROL_RATE, MOZZI_CONTROL_RATE> osc0AmpEnv, osc1AmpEnv, osc2AmpEnv;

// Mozzi EventDelay timers, used instead of millis() for timing
EventDelay arpNoteTimer, arpTimeout;
EventDelay osc0UtilityTimer, osc1UtilityTimer, osc2UtilityTimer;

// IntMaps for squashing scaled color data down to fit a musical scale.
const IntMap colorToScaleNote7(0, 256, 0, 7);
const IntMap colorToScaleNote5(0, 256, 0, 5);

const IntMap colorToScaleNotePentatonics3Octave(0, 256, 0, 15);

// Manages the three buttons below the potentiometers. The buttons are all connected to
// a single analog input pin, so they need some special sauce to work.
#include <AnalogButtons.h>
// Variables related to the buttons
bool button2Toggle = false, previousEnableButton2Mode = false;
uint8_t buttonPressed = 255;    // 255 means no button is pressed. A value of 0, 1, or 2 corresponds to B0, B1, or B2 pressed.

// **********************************************************************************
// Music Things
// **********************************************************************************

/**
 * Note: most of the musical choices for this program are in Configuration.h. That's where you can choose which scales get used. 
 */

// set up a Chord object with the data for the first selected chord from the container.
Chord currentScale = scaleContainer.selected();

// function prototype for the function that will actually generate sounds from color data
void ambienceGenerator();
void letsGetChordinated();

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
  randSeed(analogRead(A7));

  SERIAL_BEGIN(115200);
  SERIAL_PRINTLN("starting");

  // start I2C
  Wire.begin();

  // Initialize the table encoder and arm encoder, then home the arm.
  // home() must be called before Mozzi starts and before PWM clock divisors
  // are changed, because it uses standard delay(). table.begin() resets the
  // table encoder to 0 at the current position.
  table.begin();
  arm.begin();
  arm.home();

  // start the color sensor I2C connection
  colorSensor.begin(false);

  // IMPORTANT: Set the I2C clock to 400kHz fast mode AFTER initializing the connection to the sensors.
  // Need to use fastest I2C possible to minimize latency for Mozzi.
  Wire.setClock(400000);

  colorSensor.reset();
  colorSensor.enable();
  // Possible gain settings are 1, 4, 8, 32, 96. setting the second parameter to true doubles the diode sensing area.
  // That means that it's possible to get effective gains of 1, 2, 4, 8, 16, 32, 64, 96, and 192.
  colorSensor.setGain(32, false);

  // See the end of CLS16D24.h for a complete table of possible values here:
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

  // Change the PWM frequency on the motor control pins to push it above audible range.
  // After this line, millis() and delay() are unreliable. Use Mozzi EventDelay instead.
  if (USE_FAST_PWM)
  {
    setPwmFrequency(5, 1);
  }

  // Use PWM dimming for the LEDs on the color sensor. This PWM can also cause audible noise, so if
  // you want to use it, it needs to be set at a frequency that pushes the noise outside the audible range.
  if (USE_LED_PWM)
  {
    setPwmFrequency(LED_PIN, 1);
    analogWrite(LED_PIN, getBrightness(brightnessIterator));
  }
  else
  {
    digitalWrite(LED_PIN, HIGH); 
  }

  // set some starting frequencies for the oscillators
  osc0.setFreq(mtof(scaleContainer.selected().getNote(0)));
  osc1.setFreq(mtof(scaleContainer.selected().getNote(2)));
  osc2.setFreq(mtof(scaleContainer.selected().getNote(5)));

  // Start the timer that controls how frequently the I2C sensors get polled. The sensors are updated
  // one at a time in updateControl() — arm encoder, table encoder, then color sensor, cycling through
  // in sequence. I2C_UPDATE_INTERVAL is defined in Configuration.h, and is currently set to 15ms.
  // That means each individual sensor gets a fresh reading every 45ms (3 sensors * 15ms). Updating
  // more frequently gives you more responsive control, but I2C reads block the processor, so polling
  // too aggressively will steal time from Mozzi's audio generation and cause audible glitching.
  // After this call, the timer is restarted at the end of each sensor read in updateControl() rather
  // than running on a fixed schedule, so the interval represents minimum time between reads, not a
  // strict period.
  k_i2cUpdateDelay.set(I2C_UPDATE_INTERVAL);
  
  // set the initial attack and decay levels for the oscillators (basically fade in and fade out)
  osc0AmpEnv.setADLevels(160, 140);
  osc1AmpEnv.setADLevels(80, 60);
  osc2AmpEnv.setADLevels(160, 140);
  
  // finally, start Mozzi!
  startMozzi(MOZZI_CONTROL_RATE);
}


// **********************************************************************************
// updateControl
// **********************************************************************************

/**
 * @brief Mozzi control-rate callback for sensor polling and state updates.
 *
 * Called at MOZZI_CONTROL_RATE Hz. Handles button debouncing, sensor updates,
 * arm/table positioning, and invokes the ambience generator. I2C reads are
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
    arpTimeout.set(4000);
    arpTimeout.start();
    initialize = false;
  }

  // These are auto-ranging mappings for the color channels. They work like map() but track the
  // min/max values seen so far and update the mapping range dynamically.
  // They are static variables here (not globals) so we can use colorSensor.getResolution() to set
  // the upper input bound rather than hardcoding it.
  static AutoMap autoGreenToUINT8_T(0, colorSensor.getResolution(), 0, 255);
  static AutoMap autoBlueToUINT8_T(0,  colorSensor.getResolution(), 0, 255);
  static AutoMap autoRedToUINT8_T(0,   colorSensor.getResolution(), 0, 255);
  static AutoMap autoWhiteToUINT8_T(0, colorSensor.getResolution(), 0, 255);

  // check to see if buttons are pressed. returns 0, 1, 2, or 255 (no press)
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
      currentScale = scaleContainer.selected();
      SERIAL_TABS(2);
      SERIAL_PRINT("scale: ");
      SERIAL_PRINTLN(scaleContainer.scaleSelector);
      break;

    case 2: // right button — note hold, now handled in letsGetChordinated()
      break;

    default:
      break;
    }
  }

  if (!nextButtonPressAllowed && buttonTimer.ready())
  {
    nextButtonPressAllowed = true;
  }

  // Update the table's target speed from the pot. TableManager ignores this if
  // stop-detection has kicked in and resumes tracking the pot once the table
  // starts moving again.
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

    case 1: // update the table encoder. stop-detection runs inside updateAngle()
      table.updateAngle();
      currentSensorToUpdate++;
      break;

    case 2: // update the color sensor
      colorSensor.update();
      colorSensor.printColorData();
      currentSensorToUpdate = 0;
      break;

    default:
      currentSensorToUpdate = 0;  // safety mechanism to start over in case we get a weird index
      break;
    }
    k_i2cUpdateDelay.start();
  }

  // Move the arm to the position requested by the potentiometer.
  targetArmPos = arm.convertPotValToRadius(mozziAnalogRead<10>(POT_A_PIN));
  arm.moveToAngle(arm.radiusToAngle(targetArmPos));

  // Commit the table motor speed.
  table.applySpeed();

  // Turn the different color sensor channels into uint8_t numbers using the AutoMaps
  mappedGreen = autoGreenToUINT8_T(colorSensor.getGreenFixed().asInt());
  mappedBlue  = autoBlueToUINT8_T(colorSensor.getBlueFixed().asInt());
  mappedRed   = autoRedToUINT8_T(colorSensor.getRedFixed().asInt());
  mappedWhite = autoWhiteToUINT8_T(colorSensor.getClearFixed().asInt());

  // call the function that determines how sound is generated
  letsGetChordinated();

  previousEnableButton2Mode = button2Toggle;
}


// **********************************************************************************
// updateAudio
// **********************************************************************************

/**
 * @brief Mozzi audio-rate callback for sample generation.
 *
 * Called at audio rate (typically 16384 or 32768 Hz). Mixes the three oscillators
 * with their respective volume envelopes and returns a 17-bit scaled output.
 * @return MonoOutput containing the mixed audio sample.
 */
AudioOutput updateAudio()
{
  int32_t asig = (int32_t)
    osc0.next() * osc0Params.volume +
    osc1.next() * osc1Params.volume +
    osc2.next() * osc2Params.volume;

  return MonoOutput::fromAlmostNBit(17, asig);
}


// **********************************************************************************
// Loop
// **********************************************************************************

/**
 * @brief Main Arduino loop—delegates to Mozzi's audio hook.
 *
 * All control logic resides in updateControl(); audio synthesis occurs in
 * updateAudio(). This function merely services the Mozzi framework.
 */
void loop()
{
  audioHook();
}





void letsGetChordinated()
{
  static float frozenFreq0 = 0.0f;
  static float frozenFreq1 = 0.0f;
  static float frozenFreq2 = 0.0f;

  bool noteHeld = (buttonPressed == 2);

  if (!noteHeld)
  {
    uint8_t index;

    index = colorToScaleNotePentatonics3Octave(mappedGreen);
    osc0Params.noteMIDINumber = scaleContainer.selected().getNote(index);
    osc0Params.frequency = mtof(osc0Params.noteMIDINumber);
    osc0.setFreq(osc0Params.frequency);
    frozenFreq0 = osc0Params.frequency;

    index = colorToScaleNotePentatonics3Octave(mappedRed);
    osc1Params.noteMIDINumber = scaleContainer.selected().getNote(index);
    osc1Params.frequency = mtof(osc1Params.noteMIDINumber - 12);
    osc1.setFreq(osc1Params.frequency);
    frozenFreq1 = osc1Params.frequency;

    index = colorToScaleNotePentatonics3Octave(mappedBlue);
    osc2Params.noteMIDINumber = scaleContainer.selected().getNote(index);
    osc2Params.frequency = mtof(osc2Params.noteMIDINumber);
    osc2.setFreq(osc2Params.frequency);
    frozenFreq2 = osc2Params.frequency;
  }
  // When held, setFreq is not called — oscillators keep playing frozenFreq* unchanged.

  osc0Params.volume = 180;
  osc1Params.volume = 150;
  osc2Params.volume = 210;
}