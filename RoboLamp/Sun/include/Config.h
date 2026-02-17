#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

namespace Config
{
    // Behavior Toggles
    constexpr bool useSerial = false;     // Enable or disable serial debug output
    constexpr bool useStartupAnim = true; // Enable or disable startup animation
    constexpr bool useSmiley = true;      // Enable or disable smiley face on startup
    constexpr uint8_t startupMood = 0;    // 0=distracted, 1=happy, 2=moody, 3=focused

    constexpr bool startupUseColoredLight = true;      // true = neopixels color; false = white light
    constexpr WhiteMode startupWhiteMode = WHITE_WARM; // Used only when startupUseColoredLight = false; options = WHITE_COOL, WHITE_SOFT, or WHITE_WARM
    constexpr uint8_t startupBrightnessIndex = 4;      // 0-4

    // Startup color selection
    enum StartupHueMode : uint8_t // Different startup hue selection modes
    {
        STARTUP_HUE_FIXED = 0,
        STARTUP_HUE_RANDOM = 1,
    };

    constexpr StartupHueMode startupHueMode = STARTUP_HUE_FIXED; // Choose between fixed hue (of the startupHue value) or random hue on startup
    constexpr uint16_t startupRandomHueMin = 0;                  // Used only when startupHueMode == STARTUP_HUE_RANDOM; lowest value in range of random hue
    constexpr uint16_t startupRandomHueMax = 65535;              // Used only when startupHueMode == STARTUP_HUE_RANDOM; highest value in range of random hue

    constexpr uint16_t startupHue = 55000; // Used only when startupUseColoredLight == true. 0 - 65535

    constexpr bool startupRandomPreferPalette = true; // When using random hue, select from among palette colors at a higher rate than fully random colors
    constexpr uint8_t startupPalettePickPercent = 50; // The percent of time we'll draw the random color from the palette instead of randomly

    constexpr uint16_t startupTargetHeight = 300; // Set default target height to a middle value for safety. Max range is about 130 - 770.

    // Menu Positions
    constexpr int8_t menuYaw = 90;   // Centered
    constexpr int8_t menuPitch = 80; // Slightly down

    // PID Gains
    constexpr float Kp = 1.1f;  // Proportional gain (used for joystick manual control smoothing)
    constexpr float Ki = 0.03f; // Integral gain (reduces steady-state error; clamped to prevent wind-up)
    constexpr float Kd = 0.01f; // Derivative gain (dampens oscillation and overshoot)

    // Joystick update interval - changing this value significantly impacts lamp reaction speed!
    // Typical values: 25 ms = slow, thoughtful lamp; 10 ms = speedy, energetic lamp
    constexpr unsigned long manualControlInterval = 17; // Joystick update interval in ms

    // Range Limits
    const uint8_t servoMinYawConstraint = 5;     // Limits the servo to a maximum range just shy of its hard-limit of 0
    const uint8_t servoMaxYawConstraint = 175;   // Limits the servo to a maximum range just shy of its hard-limit of 180
    const uint8_t servoMinPitchConstraint = 10;  // Limits the servo to a maximum range just shy of its hard-limit of 0
    const uint8_t servoMaxPitchConstraint = 170; // Limits the servo to a maximum range just shy of its hard-limit of 180

    // Mood Timing
    constexpr unsigned long moodIdleDelay = 8000;             // Time of inactivity before entering mood behavior
    constexpr float moodSnapThreshold = 3.0f;                 // Distance threshold (degrees) to consider "at center" position
    constexpr unsigned long snapbackDebounceTime = 200;       // milliseconds
    constexpr unsigned long userNotControllingInterval = 500; // Time to consider user has stopped controlling

    // Distracted Mood
    constexpr unsigned long distractedMinInterval = 5000;  // Minimum time between distracted movements
    constexpr unsigned long distractedMaxInterval = 9000;  // Maximum time between distracted movements
    constexpr uint8_t wanderRange = 80;                    // Range of yaw and pitch wandering from center position
    constexpr float distractedReturnThreshold = 2.0f;      // Threshold to trigger return to center position
    const uint8_t servoMinDistractedYawConstraint = 20;    // Minimum yaw constraint for distracted mood
    const uint8_t servoMaxDistractedYawConstraint = 160;   // Maximum yaw constraint for distracted mood
    const uint8_t servoMinDistractedPitchConstraint = 50;  // Minimum pitch constraint for distracted mood
    const uint8_t servoMaxDistractedPitchConstraint = 170; // Maximum pitch constraint for distracted mood
    constexpr unsigned long idleDelayAfterStartup = 4000;  // 4 seconds delay before idle wandering animation starts

    // Excited Mood
    constexpr unsigned long excitedWanderMinInterval = 1000;    // Minimum time between excited movements
    constexpr unsigned long excitedWanderMaxInterval = 7000;    // Maximum time between excited movements
    constexpr unsigned long excitedMinInterval = 20000;         // Minimum time between excited standup actions
    constexpr unsigned long excitedMaxInterval = 30000;         // Maximum time between excited standup actions
    constexpr uint8_t excitedMinPitch = 50;                     // Minimum pitch for excited mood
    constexpr uint8_t excitedMaxPitch = 110;                    // Maximum pitch for excited mood
    constexpr uint8_t excitedMinYaw = 40;                       // Minimum yaw for excited mood
    constexpr uint8_t excitedMaxYaw = 140;                      // Maximum yaw for excited mood
    constexpr unsigned long excitedInterpolationTimeMin = 400;  // Minimum interpolation time for excited movements
    constexpr unsigned long excitedInterpolationTimeMax = 1000; // Maximum interpolation time for excited movements
    constexpr uint16_t excitedMaxHeight = 900;                  // Maximum height for excited standup (set above accessible range to cause bouncing)
    constexpr unsigned long excitedMinStandupInterval = 5000;   // Minimum time between excited standup actions
    constexpr unsigned long excitedMaxStandupInterval = 9000;   // Maximum time between excited standup actions

    // Moody Mood
    constexpr uint8_t moodyDroopTargetY = 150; // Pitch angle for moody droop position
    constexpr uint8_t moodyRiseTargetY = 100;  // Pitch angle for moody rise position
    constexpr uint16_t moodyHueBlue = 43000;   // Hue value for moody blue
    constexpr uint16_t moodyHuePurple = 50000; // Hue value for moody purple
    constexpr float moodyCenterSnapThreshold = 2.0f;
    constexpr uint16_t moodySlumpTarget = 150;              // Height for moody slump position
    constexpr unsigned long delayBeforeGettingMoody = 4000; // Time to wait before starting moody animation
    constexpr unsigned long moodyInterpTime = 4000;         // Interpolation time for moody movements
    constexpr unsigned long whiteFadeDuration = 1500;       // White light fade duration in milliseconds (used during moody mood)
    constexpr unsigned long headHangTime = 1000;            // Duration to hang head down in moody mood
    constexpr unsigned long headDescendTime = 4000;         // Duration to lower head in moody mood
    constexpr unsigned long headAscendTime = 3000;          // Duration to raise head in moody mood

    // -------------------------------------------------------------------------
    // HACK 1: Sunrise / Sunset (Grab your Ambient Light Sensor from PLANT, or get an inexpensive LDR online!)
    // -------------------------------------------------------------------------
    constexpr bool enableSunriseSunset = true;

    // Ambient sensor input pin (choose an unused analog pin).
    // NOTE: A0/A1/A2/A3/A6/A7 are already used in stock code. A4 is a good default.
    constexpr uint8_t ambientLightPin = A4;

    // If your sensor reads "higher in darkness" instead of "higher in brightness", flip this.
    constexpr bool ambientInvert = false;

    // Calibration: raw analogRead() range you consider "dark room" -> "bright sunlight".
    // Tune these by printing the raw value over Serial and adjusting.
    constexpr uint16_t ambientDarkCal = 40; // Raw value at or below this number is considered "dark" (0% brightness).
    constexpr uint16_t ambientBrightCal = 650; // Raw value at or above this number is considered "bright" (100% brightness).

    // How often to sample/update the lamp while sunrise/sunset mode is active.
    constexpr unsigned long sunUpdateIntervalMs = 150;

    // Smoothing (0.0 = no smoothing, 1.0 = never changes). Typical: 0.05–0.25
    constexpr float sunSmoothingAlpha = 0.12f;

    // Warm-white brightness mapping
    constexpr uint8_t sunMinBrightness = 0;  // Minimum brightness when ambient sensor is at or below ambientDarkCal (helps prevent flicker and gives a "glow in the dark" effect)
    constexpr uint8_t sunMaxBrightness = 255; // Maximum brightness when ambient sensor is at or above ambientBrightCal

    // Optional gamma to make the ramp feel more “sun-like” to eyes (1.0 = linear).
    constexpr float sunBrightnessGamma = 2.0f;

    // Height mapping (raw encoder values; stock comments suggest ~130–770 usable)
    constexpr uint16_t sunMinHeight = 140; // If it starts bouncing at the bottom, increase this number
    constexpr uint16_t sunMaxHeight = 820; // Starts bouncing after 770

    // Height motor behavior while in sun mode
    constexpr uint8_t sunMotorSpeed = 160;      // 0–255
    constexpr uint16_t sunHeightTolerance = 12; // "close enough" to stop motor

    // Extra smoothing applied to the *output* brightness (helps reduce visible flicker)
    constexpr float sunOutputSmoothingAlpha = 0.18f;

    // Color temperature crossfade (cool when dim -> warm when bright)
    // Below sunWarmMixStart, output is mostly cool. By sunWarmMixFull, output is mostly warm.
    constexpr float sunWarmMixStart = 0.18f;
    constexpr float sunWarmMixFull = 0.70f;

    // Head pose mapping while in sun mode
    // NOTE: higher pitch angles look more "down" in this firmware (see moodyDroopTargetY).
    constexpr uint8_t sunHeadYaw = menuYaw; // "straight ahead"
    constexpr uint8_t sunPitchDim = 140;    // dimmest -> straight ahead & down
    constexpr uint8_t sunPitchBright = 60;  // brightest -> straight ahead & up

    constexpr unsigned long sunRampInDurationMs = 1200; // 0 = disabled
    constexpr uint8_t sunRampInStartBrightness = 0;     // start from off

    constexpr uint16_t sunWarmMixStart1024 = 184; // ~0.18*1024
    constexpr uint16_t sunWarmMixFull1024 = 717;  // ~0.70*1024
}

#endif // CONFIG_H