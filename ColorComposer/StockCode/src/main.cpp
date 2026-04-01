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
EventDelay osc0ButtonMode2NoteTimer, osc1ButtonMode2NoteTimer, osc2ButtonMode2NoteTimer;

// IntMaps for squashing scaled color data down to fit a musical scale.
const IntMap colorToScaleNote7(0, 256, 0, 7);
const IntMap colorToScaleNote5(0, 256, 0, 5);

// Manages the three buttons below the potentiometers. The buttons are all connected to
// a single analog input pin, so they need some special sauce to work.
#include <AnalogButtons.h>
// Variables related to the buttons
bool enableButton2Mode = false, previousEnableButton2Mode = false;
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

    case 2: // right button — toggle button mode 2
      enableButton2Mode = !enableButton2Mode;
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
  ambienceGenerator();

  previousEnableButton2Mode = enableButton2Mode;
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


// **********************************************************************************
// ambienceGenerator
// **********************************************************************************

/**
 * @brief Generates musical output from color sensor data and mode state.
 *
 * Two primary modes of operation:
 * - Normal mode: Sustained drones with notes selected from scales based on
 *   color channel values. Occasional arpeggios triggered by white channel intensity.
 * - Button mode 2: Stochastic arpeggiation with probability of note triggers
 *   modulated by color channel values.
 *
 * ADSR envelopes are reconfigured when transitioning between modes.
 */
void ambienceGenerator()
{
  static int8_t arpIndex = 0;                   // used to select notes in a scale to play as an arpeggio
  static uint8_t numNotesLeftInArp = 0;         // arpeggios only play for a limited number of notes
  static bool initialize = true;                // used to initialize some things the first time this function is called

  static uint16_t osc2PortTime = 20;            // portamento value

  const bool USE_PORTAMENTO = true;

  static uint16_t attack = 100, decay = 500, sustain = 8000, release = 3000;

  // used to move a musical note up or down by an octave. mappedWhite is the 8 bit autoMapped value
  // of the white light color channel, and the >> 6 operator performs a right shift, where it takes
  // all 8 bits in a number and literally moves them right, filling in with 0s on the left. This moves
  // 6 places to the right, so it reduces an 8 bit number to a 2 bit number, which can represent the
  // numbers 0 through 3. It's basically a way of dividing 256 by 64, but it's a much faster operation.
  // the microcontroller has hardware purpose-built for performing bit shift operations, and it only
  // takes a single clock cycle. Division is incredibly slow on this microcontroller, and might need
  // something like 40 clock cycles.
  // mappedWhite is 8 bit, this cuts it down to 2 (range 0-3).
  int8_t octaveShifter = (int8_t)(OCTAVE_SHIFTER_CHANNEL >> 6);
  static bool arpeggiate = false, arpStarted = false, arpOnTimeOut = true;

  // button2mode is basically arpeggiating each oscillator, instead of holding sustained notes.
  // baseNoteInterval is the starting point for how much time should elapse between triggering notes
  // in this mode. This gets modified by the value of the green color channel in button2mode.
  static uint8_t baseNoteInterval = 64; // baseline amount of time that an arp note will be turned on for, gets modified by sensor readings

  if (!enableButton2Mode && previousEnableButton2Mode)
    initialize = true; // transition out of button mode 2 requires resetting ADSRs

  // on first call, we need to set some things up
  if (initialize)
  {
    osc0AmpEnv.setADLevels(160, 140);
    osc1AmpEnv.setADLevels(60, 50);
    osc2AmpEnv.setADLevels(120, 110);
    osc0AmpEnv.setTimes(attack, decay, sustain, release);
    osc1AmpEnv.setTimes(attack, decay, sustain, release);
    osc2AmpEnv.setTimes(attack, decay, sustain, release);
    initialize = false;
  }

  // button2mode is basically arpeggiating each oscillator, instead of holding sustained notes
  if (enableButton2Mode)
  {
    // if we weren't already in button2Mode, we need to set some things up first
    if (!previousEnableButton2Mode)
    {
      // change the ADSR to make the notes sound plucky
      osc0AmpEnv.setTimes(5, 50, 100, 200);
      osc1AmpEnv.setTimes(5, 50, 100, 200);
      osc1AmpEnv.setADLevels(60, 40);
      osc2AmpEnv.setTimes(5, 50, 100, 200);

      // start the timer objects that will determine how often we should play a note
      // notes are triggered more or less frequently based on the value of the green color channel.
      // so we start with that baseNoteInterval, and then change it based on the green channel.
      // the right shift operations drop the value of the green channel to a max of 4 bits, so a
      // maximum of (2^4) - 1 = 15. The max() function call is there to add a lower bound. There is a chance
      // that the bit shift would create a value of 0, and then we'd be multiplying the base interval
      // by 0, which means constantly retriggering the note (which would result in a silent note because
      // the ADSR would constantly restart). So we multiply the base interval by a number between 1 and 15,
      // so the base interval ranges from 64ms to 15*64ms = 960ms.
      osc0ButtonMode2NoteTimer.set(max(1, OSC_0_BUTTON_2_MODE_TIMER >> 4) * baseNoteInterval);
      osc0ButtonMode2NoteTimer.start();
      osc1ButtonMode2NoteTimer.set(max(1, OSC_1_BUTTON_2_MODE_TIMER >> 4) * baseNoteInterval * 2);
      osc1ButtonMode2NoteTimer.start();
      osc2ButtonMode2NoteTimer.set(max(1, OSC_2_BUTTON_2_MODE_TIMER >> 4) * baseNoteInterval);
      osc2ButtonMode2NoteTimer.start();
    }

    // the base note interval starts at 64, but now it gets modified by the value of the white color channel
    baseNoteInterval = max(1, (BASE_INTERVAL_CHANNEL >> 5)) * 16; // evaluates to a range between 16 and 7*16 = 112

    // check to see if we're ready to trigger a note for osc0
    if (osc0ButtonMode2NoteTimer.ready())
    {
      // this is a fun way to add some randomness and also modulate the effect of the randomness with a sensor reading.
      // first we generate a random number between 0 and 255. If that number is less than the current value of the red
      // color channel sensor reading, then we trigger a new note. So what this means is that the more red light the sensor
      // sees, the more frequently we'll trigger a note! If there's absolutely no red light (mappedRed == 0), then we'll
      // never trigger a note because rand(256) will never return a number less than 0. If you want to tune the behavior
      // of this, change the constraints on the random number. For example, you could change to rand(128), which should
      // make notes trigger a lot more often. Or move the range up so that only extremely bright red light values trigger
      // a note by using something like rand(192, 256).
      if (rand(RANDOMNESS_THRESHOLD + 1) < OSC_0_NOTE_SELECTOR_CHANNEL)
      {
        switch (scaleContainer.selected().numNotes)
        {
          case 7:
            osc0Params.noteMIDINumber = scaleContainer.selected().getNote(colorToScaleNote7(OSC_0_NOTE_SELECTOR_CHANNEL)) + ((int8_t)rand(-1, 2) * 12);
            break;
          case 5:
            osc0Params.noteMIDINumber = scaleContainer.selected().getNote(colorToScaleNote5(OSC_0_NOTE_SELECTOR_CHANNEL)) + ((int8_t)rand(-1, 2) * 12);
            break;
          default:
            break;
        }
      }
      // turn the note on
      osc0AmpEnv.noteOn();
      osc0Params.frequency = mtof(osc0Params.noteMIDINumber);
      osc0.setFreq(osc0Params.frequency);
      // reset the timer
      osc0ButtonMode2NoteTimer.set(max(1, OSC_0_BUTTON_2_MODE_TIMER >> 4) * baseNoteInterval);
      osc0ButtonMode2NoteTimer.start();
    }

    // now do the same thing for osc1
    if (osc1ButtonMode2NoteTimer.ready())
    {
      if (rand(RANDOMNESS_THRESHOLD + 1) < OSC_1_ARP_TRIGGER_CHANNEL)
      {
        switch (scaleContainer.selected().numNotes)
        {
          case 7:
            osc1Params.noteMIDINumber = scaleContainer.selected().getNote(colorToScaleNote7(OSC_1_NOTE_SELECTOR_CHANNEL)) + ((int8_t)rand(-1, 2) * 12);
            break;
          case 5:
            osc1Params.noteMIDINumber = scaleContainer.selected().getNote(colorToScaleNote5(OSC_1_NOTE_SELECTOR_CHANNEL)) + ((int8_t)rand(-1, 2) * 12);
            break;
          default:
            break;
        }
      }
      osc1AmpEnv.noteOn();
      osc1Params.frequency = mtof(osc1Params.noteMIDINumber);
      osc1.setFreq(osc1Params.frequency);
      osc1ButtonMode2NoteTimer.set(max(1, OSC_1_BUTTON_2_MODE_TIMER >> 4) * baseNoteInterval * 2);
      osc1ButtonMode2NoteTimer.start();
    }

    // and do the same for osc2
    if (osc2ButtonMode2NoteTimer.ready())
    {
      if (rand(RANDOMNESS_THRESHOLD + 1) < OSC_2_ARP_TRIGGER_CHANNEL)
      {
        switch (scaleContainer.selected().numNotes)
        {
          case 7:
            osc2Params.noteMIDINumber = scaleContainer.selected().getNote(colorToScaleNote7(OSC_2_NOTE_SELECTOR_CHANNEL)) + ((int8_t)rand(-1, 2) * 12);
            break;
          case 5:
            osc2Params.noteMIDINumber = scaleContainer.selected().getNote(colorToScaleNote5(OSC_2_NOTE_SELECTOR_CHANNEL)) + ((int8_t)rand(-1, 2) * 12);
            break;
          default:
            break;
        }
      }
      osc2AmpEnv.noteOn();
      osc2Params.frequency = mtof(osc2Params.noteMIDINumber);
      osc2.setFreq(osc2Params.frequency);
      osc2ButtonMode2NoteTimer.set(max(1, OSC_2_BUTTON_2_MODE_TIMER >> 4) * baseNoteInterval);
      osc2ButtonMode2NoteTimer.start();
    }

    osc0AmpEnv.update();
    osc1AmpEnv.update();
    osc2AmpEnv.update();
    osc0Params.volume = osc0AmpEnv.next();
    osc1Params.volume = osc1AmpEnv.next();
    osc2Params.volume = osc2AmpEnv.next();
  }

  else  // we're not in button 2 mode in this case
  {
    // octaveShifter is currently some value between 0 and 3. This moves and constrains it
    // to be either -1, 0, or 1.
    // this constrains the amount that the octave can be shifted up or down to either -1, 0, or +1 octaves
    octaveShifter = max(-1, octaveShifter - 2);

    // this makes it so that there is a minimum amount of time between arpeggios
    // we'll occasionally trigger an arpeggio if the white light channel is bright enough, but to prevent
    // that from happening too frequently, we use this timer to prevent a new arpeggio triggering too soon
    // after the last one.
    if (arpTimeout.ready())
    {
      arpOnTimeOut = false;
    }

    // this makes sure that arpeggios only get triggered if the octave has been shifted up by 1. I did this
    // to add some brightness to the sound. Basically the arpeggio triggers if the white light channel is
    // bright enough, and I wanted the arpeggio sound to mirror that brightness by being shifted up an octave.
    // if an arpeggio is allowed to start, use a bit of randomness to determine if one actually will start.
    // the more white light there is, the more likely the arpeggio is.
    if (octaveShifter > 0 && !arpOnTimeOut && !arpStarted)
    {
      arpeggiate = (rand(RANDOMNESS_THRESHOLD + 1) <= GLOBAL_ARP_TRIGGER) ? true : false; // add some randomness so that we don't always trigger an arp
                                                              // could use something like rand(128) to force more arpeggios
    }

    // choose which note in the scale we'll be using based on the value of the green channel
    // create an index to pick a note out of the scale
    uint8_t i = 0;
    if (scaleContainer.selected().numNotes == 7)
    {
      i = colorToScaleNote7(OSC_0_NOTE_SELECTOR_CHANNEL);   // this is an IntMap
    }
    else
    {
      i = colorToScaleNote5(OSC_0_NOTE_SELECTOR_CHANNEL);   // IntMap
    }

    // get the actual note from the scale
    osc0Params.noteMIDINumber = scaleContainer.selected().getNote(i);
    // if the note has changed, restart the note playback
    if (osc0Params.lastNoteMIDINumber != osc0Params.noteMIDINumber)
    {
      osc0AmpEnv.noteOn();
      osc0Params.lastNoteMIDINumber = osc0Params.noteMIDINumber;
    }
    // calculate the frequency of the oscillator
    osc0Params.frequency = mtof(osc0Params.noteMIDINumber);
    // set the oscillator to the calculated frequency
    osc0.setFreq((osc0Params.frequency));
    // update the amp envelope (restarts with a new noteOn() call)
    osc0AmpEnv.update();
    osc0Params.volume = osc0AmpEnv.next();

    // 
    uint8_t j = 0;
    if (scaleContainer.selected().numNotes == 7)
    {
      j = colorToScaleNote7(OSC_1_NOTE_SELECTOR_CHANNEL);
    }
    else
    {
      j = colorToScaleNote5(OSC_1_NOTE_SELECTOR_CHANNEL);
    }

    switch (scaleContainer.scaleSelector)
    {
    case 0:
      osc1Params.noteMIDINumber = scaleContainer.selected().getNote((j + 4) % scaleContainer.selected().numNotes) - 12;
      break;
    case 1:
      osc1Params.noteMIDINumber = scaleContainer.selected().getNote((i + 2) % scaleContainer.selected().numNotes) + octaveShifter * 12;
      break;
    case 2:
      osc1Params.noteMIDINumber = scaleContainer.selected().getNote((j)) - 12;
      break;
    default:
      break;
    }

    if (osc1Params.lastNoteMIDINumber != osc1Params.noteMIDINumber)
    {
      osc1AmpEnv.noteOn();
      osc1Params.lastNoteMIDINumber = osc1Params.noteMIDINumber;
    }
    osc1Params.frequency = mtof(osc1Params.noteMIDINumber);
    osc1.setFreq(osc1Params.frequency);
    osc1AmpEnv.update();
    osc1Params.volume = osc1AmpEnv.next();

    if (!arpeggiate)
    {
      osc2Params.noteMIDINumber = scaleContainer.selected().getNote((i + 3) % scaleContainer.selected().numNotes) + (octaveShifter * 12);
      if (osc2Params.lastNoteMIDINumber != osc2Params.noteMIDINumber)
      {
        osc2AmpEnv.noteOn();
        osc2Params.lastNoteMIDINumber = osc2Params.noteMIDINumber;
      }
      osc2Params.frequency = mtof(osc2Params.noteMIDINumber);
      osc2.setFreq(osc2Params.frequency);
      osc2AmpEnv.update();
      osc2Params.volume = osc2AmpEnv.next();
    }
    else
    {
      if (!arpStarted)
      {
        numNotesLeftInArp = rand(4, 17);
        arpStarted = true;
        arpNoteTimer.set(min(OSC_2_ARP_TRIGGER_CHANNEL, 192));
        arpNoteTimer.start();
        arpIndex = rand(scaleContainer.selected().numNotes);
        osc2AmpEnv.setTimes(5, 5, 100, 100);
      }

      if (arpNoteTimer.ready())
      {
        osc2Params.noteMIDINumber = scaleContainer.selected().getNote(arpIndex) + (12 * (int8_t)rand(-1, 3));
        osc2Params.frequency = mtof(osc2Params.noteMIDINumber);
        osc2Params.volume = 120;

        osc2Portamento.setTime(osc2PortTime);
        if (!USE_PORTAMENTO)
          osc2.setFreq(osc2Params.frequency);

        int8_t arpShift = rand(-5, 6);
        arpIndex += arpShift;
        arpIndex = (arpIndex < scaleContainer.selected().numNotes && arpIndex >= 0)
            ? arpIndex
            : ((arpIndex < 0)
                ? arpIndex += scaleContainer.selected().numNotes
                : arpIndex -= scaleContainer.selected().numNotes);

        arpNoteTimer.start();
        numNotesLeftInArp -= 1;
      }

      if (USE_PORTAMENTO)
      {
        osc2Portamento.start(osc2Params.noteMIDINumber);
        osc2.setFreq_Q16n16(osc2Portamento.next());
      }

      if (numNotesLeftInArp == 0)
      {
        arpeggiate = false;
        arpStarted = false;
        arpTimeout.set(mappedGreen << 4);
        arpTimeout.start();
        arpOnTimeOut = true;
        osc2AmpEnv.setTimes(attack, decay, sustain, release);
      }
    }
  }
}
