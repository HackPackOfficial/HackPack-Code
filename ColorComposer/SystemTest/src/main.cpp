/**
 * RECRD System Test / Diagnostic Program
 *
 * PURPOSE:
 *   Upload this to verify hardware and debug sensor behavior. All sensor
 *   readings are printed to the serial monitor at a controlled rate. Audio
 *   synthesis is intentionally minimal — one oscillator, green channel driving
 *   note selection within the active scale — so the processor has plenty of
 *   headroom for serial output without audio glitching.
 *
 * IMPORTANT: Serial output requires USE_SERIAL to be defined.
 *   In Debug.h, uncomment:
 *       #define USE_SERIAL
 *   and set the monitor to 115200 baud. Without this, the program still runs
 *   and exercises all hardware, but nothing will appear in the serial monitor.
 *
 * HARDWARE BEHAVIOR:
 *   K0 (Pot A, A0)  — arm radial position, same as normal operation
 *   K1 (Pot B, A1)  — table speed and direction, same as normal operation
 *   B0              — cycles LED brightness, same as normal operation
 *   B1              — cycles musical scale, same as normal operation
 *   B2              — momentary note hold (hold to freeze, release to unfreeze)
 *
 * SERIAL OUTPUT (printed every ~100ms, tab-separated):
 *   COLOR RAW    red  green  blue  clear  IR
 *   COLOR MAP    mappedRed  mappedGreen  mappedBlue  mappedWhite
 *   ENCODERS     arm_angle  table_angle
 *   POTS         K0  K1
 *   BUTTONS      button_index  (255 = none pressed)
 *   SCALE        scale_index
 *   OSC0         midi_note  frequency
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
#include <ADSR.h>

#include <Configuration.h>
#include <PinAssignments.h>

// ArmManager depends on Mozzi internals — must come after <Mozzi.h>.
#include <ArmManager.h>
#include <TableManager.h>

#include <ColorSensor.h>
#include <MusicTypes.h>
#include <OscillatorTools.h>
#include <AnalogButtons.h>

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

EventDelay k_i2cUpdateDelay;

// Prints at ~100ms — readable in serial monitor, won't flood the buffer.
constexpr uint16_t PRINT_INTERVAL_MS = 100;
EventDelay k_printDelay;

// ---------------------------------------------------------------------------
// Mechanical systems
// ---------------------------------------------------------------------------

ArmManager   arm(ARM_MOTOR_SPEED_PIN, ARM_MOTOR_DIR_PIN);
TableManager table(TABLE_MOTOR_SPEED_PIN, TABLE_MOTOR_DIR_PIN);

// ---------------------------------------------------------------------------
// Color sensor
// ---------------------------------------------------------------------------

ColorSensor colorSensor;
uint8_t mappedRed = 0, mappedGreen = 0, mappedBlue = 0, mappedWhite = 0;

// ---------------------------------------------------------------------------
// LED brightness
// ---------------------------------------------------------------------------

constexpr uint8_t NUM_BRIGHTNESS_LEVELS = 5;
uint8_t brightnessIterator = 3;

const uint8_t PROGMEM LEDBrightnessLevels[NUM_BRIGHTNESS_LEVELS] = {0, 31, 63, 191, 255};
inline uint8_t getBrightness(uint8_t level = 3)
{
    return pgm_read_byte(&LEDBrightnessLevels[level]);
}

// ---------------------------------------------------------------------------
// Audio — single oscillator, minimal synthesis
// ---------------------------------------------------------------------------

// osc0, osc1, osc2 are declared in Configuration.h along with their wavetables.
// Only osc0 is used; osc1 and osc2 stay silent (volume = 0).
oscillatorParams osc0Params, osc1Params, osc2Params;
ADSR<MOZZI_CONTROL_RATE, MOZZI_CONTROL_RATE> osc0AmpEnv;

// Maps the full 0-255 mapped color range across 3 octaves of a pentatonic scale.
// Consistent with the pentatonics hack program.
const IntMap colorToScaleNote5(0, 256, 0, 5);

// ---------------------------------------------------------------------------
// Buttons
// ---------------------------------------------------------------------------

uint8_t buttonPressed = 255;

// ---------------------------------------------------------------------------
// Pot values — stored each cycle so they can be printed at the print interval
// without re-reading the ADC a second time.
// ---------------------------------------------------------------------------

uint16_t potAVal = 0, potBVal = 0;

// ---------------------------------------------------------------------------
// Scale
// ---------------------------------------------------------------------------

Chord currentScale = scaleContainer.selected();

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void setup()
{
    randSeed(analogRead(A7));

    SERIAL_BEGIN(115200);
    SERIAL_PRINTLN("RECRD System Test starting...");

    Wire.begin();

    table.begin();
    arm.begin();
    arm.home();

    colorSensor.begin(false);
    Wire.setClock(400000);

    colorSensor.reset();
    colorSensor.enable();
    colorSensor.setGain(32, false);
    colorSensor.setResolutionAndConversionTime(0x02);

    // Enable all five channels so every channel can be printed and inspected.
    colorSensor.setChannelEnabled(ColorChannels::RED,   true);
    colorSensor.setChannelEnabled(ColorChannels::GREEN, true);
    colorSensor.setChannelEnabled(ColorChannels::BLUE,  true);
    colorSensor.setChannelEnabled(ColorChannels::CLEAR, true);
    colorSensor.setChannelEnabled(ColorChannels::IR,    true);

    SERIAL_PRINT("Color sensor conversion time (ms): ");
    SERIAL_PRINTLN(colorSensor.getConversionTimeMillis());
    SERIAL_PRINT("Color sensor resolution:           ");
    SERIAL_PRINTLN(colorSensor.getResolution());

    if (USE_FAST_PWM) { setPwmFrequency(5, 1); }
    if (USE_LED_PWM)
    {
        setPwmFrequency(LED_PIN, 1);
        analogWrite(LED_PIN, getBrightness(brightnessIterator));
    }
    else
    {
        digitalWrite(LED_PIN, HIGH);
    }

    // Start osc0 on the root note of the selected scale.
    osc0.setFreq(mtof(scaleContainer.selected().getNote(0)));
    osc0Params.noteMIDINumber = scaleContainer.selected().getNote(0);
    osc0Params.frequency = mtof(osc0Params.noteMIDINumber);

    // Long sustain so the note rings clearly while you observe sensor output.
    osc0AmpEnv.setADLevels(200, 180);
    osc0AmpEnv.setTimes(50, 200, 60000, 500);
    osc0AmpEnv.noteOn();

    k_i2cUpdateDelay.set(I2C_UPDATE_INTERVAL);
    k_printDelay.set(PRINT_INTERVAL_MS);
    k_printDelay.start();

    startMozzi(MOZZI_CONTROL_RATE);
}

// ---------------------------------------------------------------------------
// updateControl
// ---------------------------------------------------------------------------

void updateControl()
{
    static bool nextButtonPressAllowed = true;
    static bool timerInitialized = false;
    static EventDelay buttonTimer;
    if (!timerInitialized)
    {
        buttonTimer.set(250);
        timerInitialized = true;
    }

    // AutoMaps for the color channels — same adaptive ranging as the main program.
    static AutoMap autoRedToUINT8_T(0,   colorSensor.getResolution(), 0, 255);
    static AutoMap autoGreenToUINT8_T(0, colorSensor.getResolution(), 0, 255);
    static AutoMap autoBlueToUINT8_T(0,  colorSensor.getResolution(), 0, 255);
    static AutoMap autoWhiteToUINT8_T(0, colorSensor.getResolution(), 0, 255);

    // Read pots every cycle for fresh motor control values.
    potAVal = mozziAnalogRead<10>(POT_A_PIN);
    potBVal = mozziAnalogRead<10>(POT_B_PIN);

    // Button handling — same debounce and lockout scheme as the main program.
    buttonPressed = getButtonPressed(mozziAnalogRead<10>(BUTTONS_PIN));
    if (nextButtonPressAllowed && buttonPressed < 255)
    {
        buttonTimer.start();
        nextButtonPressAllowed = false;
        switch (buttonPressed)
        {
        case 0: // B0 — cycle LED brightness
            brightnessIterator = (brightnessIterator + 1) % NUM_BRIGHTNESS_LEVELS;
            analogWrite(LED_PIN, getBrightness(brightnessIterator));
            break;

        case 1: // B1 — cycle scale
            scaleContainer.nextScale();
            currentScale = scaleContainer.selected();
            break;

        case 2: // B2 — momentary note hold; handled below, nothing to do on the press event itself.
            break;

        default:
            break;
        }
    }
    if (!nextButtonPressAllowed && buttonTimer.ready())
    {
        nextButtonPressAllowed = true;
    }

    // Table speed from K1.
    table.updateTargetSpeed(potBVal);

    // Staggered I2C sensor updates — arm encoder, table encoder, color sensor, cycling.
    static uint8_t currentSensorToUpdate = 0;
    if (k_i2cUpdateDelay.ready())
    {
        switch (currentSensorToUpdate)
        {
        case 0:
            arm.updatePosition();
            currentSensorToUpdate++;
            break;
        case 1:
            table.updateAngle();
            currentSensorToUpdate++;
            break;
        case 2:
            colorSensor.update();
            currentSensorToUpdate = 0;
            break;
        default:
            currentSensorToUpdate = 0;
            break;
        }
        k_i2cUpdateDelay.start();
    }

    // Arm position from K0.
    arm.moveToAngle(arm.radiusToAngle(arm.convertPotValToRadius(potAVal)));

    table.applySpeed();

    // Map color channels to 0-255.
    mappedRed   = autoRedToUINT8_T(colorSensor.getRedFixed().asInt());
    mappedGreen = autoGreenToUINT8_T(colorSensor.getGreenFixed().asInt());
    mappedBlue  = autoBlueToUINT8_T(colorSensor.getBlueFixed().asInt());
    mappedWhite = autoWhiteToUINT8_T(colorSensor.getClearFixed().asInt());

    // ---------------------------------------------------------------------------
    // Note selection — green channel drives osc0, B2 freezes it.
    // ---------------------------------------------------------------------------

    bool noteHeld = (buttonPressed == 2);

    if (!noteHeld)
    {
        uint8_t index = colorToScaleNote5(mappedGreen);
        MIDI_NOTE newNote = scaleContainer.selected().getNote(index);

        if (newNote != osc0Params.noteMIDINumber)
        {
            osc0Params.noteMIDINumber = newNote;
            osc0Params.frequency = mtof(newNote);
            osc0.setFreq(osc0Params.frequency);
            osc0AmpEnv.noteOn();
        }
    }
    // When held, osc0 keeps whatever frequency was last set — no action needed.

    osc0AmpEnv.update();
    osc0Params.volume = osc0AmpEnv.next();

    // osc1 and osc2 are silent in this program.
    osc1Params.volume = 0;
    osc2Params.volume = 0;

    // ---------------------------------------------------------------------------
    // Serial output — rate limited to PRINT_INTERVAL_MS.
    // ---------------------------------------------------------------------------

    if (k_printDelay.ready())
    {
        SERIAL_PRINT("COLOR RAW\t");
        SERIAL_PRINT(mappedRed);
        SERIAL_PRINT("\t");
        SERIAL_PRINT(mappedGreen);
        SERIAL_PRINT("\t");
        SERIAL_PRINT(mappedBlue);
        SERIAL_PRINT("\t");
        SERIAL_PRINT(mappedWhite);
        SERIAL_PRINT("\t");
        SERIAL_PRINT(colorSensor.getIR());


        SERIAL_PRINT("\tENCODERS\t");
        SERIAL_PRINT(arm.getAngle());
        SERIAL_PRINT("\t");
        SERIAL_PRINT(table.getAngle());

        SERIAL_PRINT("\tPOTS\t");
        SERIAL_PRINT(potAVal);
        SERIAL_PRINT("\t");
        SERIAL_PRINT(potBVal);

        SERIAL_PRINT("\tBUTTONS\t");
        SERIAL_PRINT(buttonPressed);

        SERIAL_PRINT("\tSCALE\t");
        SERIAL_PRINT(scaleContainer.scaleSelector);

        SERIAL_PRINT("\tOSC0\t");
        SERIAL_PRINT(osc0Params.noteMIDINumber);
        SERIAL_PRINT("\t");
        SERIAL_PRINTLN(osc0Params.frequency);

        k_printDelay.start();
    }
}

// ---------------------------------------------------------------------------
// updateAudio
// ---------------------------------------------------------------------------

AudioOutput updateAudio()
{
    // Only osc0 contributes audio; osc1 and osc2 volumes are always 0.
    int32_t asig = (int32_t)osc0.next() * osc0Params.volume;
    return MonoOutput::fromAlmostNBit(17, asig);
}

// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------

void loop()
{
    audioHook();
}