#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

namespace Config
{
    // Behavior Toggles
    constexpr bool useSerial = false;     // Enable or disable serial debug output -- ///{"options":["true","false"]}
    constexpr bool useStartupAnim = true; // Enable or disable startup animation -- ///{"options":["true","false"]}
    constexpr bool useSmiley = true;      // Enable or disable smiley face on startup -- ///{"options":["true","false"]}
    constexpr uint8_t startupMood = 0;    // 0=distracted, 1=happy, 2=moody, 3=focused -- ///{"range":[0,3]}

    constexpr bool startupUseColoredLight = true;      // true = neopixels color; false = white light -- ///{"options":["true","false"]}
    constexpr WhiteMode startupWhiteMode = WHITE_WARM; // Used only when startupUseColoredLight = false; options = WHITE_COOL, WHITE_SOFT, or WHITE_WARM -- ///{"options":["WHITE_COOL","WHITE_SOFT","WHITE_WARM"]}
    constexpr uint8_t startupBrightnessIndex = 4;      // 0-4 -- ///{"range":[0,4]}

    // Startup color selection
    enum StartupHueMode : uint8_t // Different startup hue selection modes
    {
        STARTUP_HUE_FIXED = 0,
        STARTUP_HUE_RANDOM = 1,
    };

    constexpr StartupHueMode startupHueMode = STARTUP_HUE_FIXED; // Choose between fixed hue (of the startupHue value) or random hue on startup -- ///{"options":["STARTUP_HUE_FIXED","STARTUP_HUE_RANDOM"]}
    constexpr uint16_t startupRandomHueMin = 0;                  // Used only when startupHueMode == STARTUP_HUE_RANDOM; lowest value in range of random hue -- ///{"min":0,"max":65535}
    constexpr uint16_t startupRandomHueMax = 65535;              // Used only when startupHueMode == STARTUP_HUE_RANDOM; highest value in range of random hue -- ///{"min":0,"max":65535}

    constexpr uint16_t startupHue = 55000; // Used only when startupUseColoredLight == true. 0 - 65535 -- ///{"min":0,"max":65535}

    constexpr bool startupRandomPreferPalette = true; // When using random hue, select from among palette colors at a higher rate than fully random colors -- ///{"options":["true","false"]}
    constexpr uint8_t startupPalettePickPercent = 50; // The percent of time we'll draw the random color from the palette instead of randomly -- ///{"min":0,"max":100}

    constexpr uint16_t startupTargetHeight = 300; // Set default target height to a middle value for safety. Max range is about 130 - 770. -- ///{"min":130,"max":770}

    // Menu Positions
    constexpr int8_t menuYaw = 90;   // Centered -- ///{"min":0,"max":180}
    constexpr int8_t menuPitch = 80; // Slightly down -- ///{"min":0,"max":180}

    // PID Gains
    constexpr float Kp = 1.1f;  // Proportional gain (used for joystick manual control smoothing) -- ///{"min":0,"max":10}
    constexpr float Ki = 0.03f; // Integral gain (reduces steady-state error; clamped to prevent wind-up) -- ///{"min":0,"max":1}
    constexpr float Kd = 0.01f; // Derivative gain (dampens oscillation and overshoot) -- ///{"min":0,"max":1}

    // Joystick update interval - changing this value significantly impacts lamp reaction speed!
    // Typical values: 25 ms = slow, thoughtful lamp; 10 ms = speedy, energetic lamp
    constexpr unsigned long manualControlInterval = 17; // Joystick update interval in ms -- ///{"min":5,"max":100}

    // Range Limits
    const uint8_t servoMinYawConstraint = 5;     // Limits the servo to a maximum range just shy of its hard-limit of 0 -- ///{"min":0,"max":90}
    const uint8_t servoMaxYawConstraint = 175;   // Limits the servo to a maximum range just shy of its hard-limit of 180 -- ///{"min":90,"max":180}
    const uint8_t servoMinPitchConstraint = 10;  // Limits the servo to a maximum range just shy of its hard-limit of 0 -- ///{"min":0,"max":90}
    const uint8_t servoMaxPitchConstraint = 170; // Limits the servo to a maximum range just shy of its hard-limit of 180 -- ///{"min":90,"max":180}

    // Mood Timing
    constexpr unsigned long moodIdleDelay = 8000;             // Time of inactivity before entering mood behavior -- ///{"min":1000,"max":30000}
    constexpr float moodSnapThreshold = 3.0f;                 // Distance threshold (degrees) to consider "at center" position -- ///{"min":0.5,"max":10}
    constexpr unsigned long snapbackDebounceTime = 200;       // milliseconds -- ///{"min":50,"max":1000}
    constexpr unsigned long userNotControllingInterval = 500; // Time to consider user has stopped controlling -- ///{"min":100,"max":5000}

    // Distracted Mood
    constexpr unsigned long distractedMinInterval = 5000;  // Minimum time between distracted movements -- ///{"min":1000,"max":20000}
    constexpr unsigned long distractedMaxInterval = 9000;  // Maximum time between distracted movements -- ///{"min":1000,"max":30000}
    constexpr uint8_t wanderRange = 80;                    // Range of yaw and pitch wandering from center position -- ///{"min":10,"max":120}
    constexpr float distractedReturnThreshold = 2.0f;      // Threshold to trigger return to center position -- ///{"min":0.5,"max":10}
    const uint8_t servoMinDistractedYawConstraint = 20;    // Minimum yaw constraint for distracted mood -- ///{"min":0,"max":90}
    const uint8_t servoMaxDistractedYawConstraint = 160;   // Maximum yaw constraint for distracted mood -- ///{"min":90,"max":180}
    const uint8_t servoMinDistractedPitchConstraint = 50;  // Minimum pitch constraint for distracted mood -- ///{"min":0,"max":90}
    const uint8_t servoMaxDistractedPitchConstraint = 170; // Maximum pitch constraint for distracted mood -- ///{"min":90,"max":180}
    constexpr unsigned long idleDelayAfterStartup = 4000;  // 4 seconds delay before idle wandering animation starts -- ///{"min":0,"max":20000}

    // Excited Mood
    constexpr unsigned long excitedWanderMinInterval = 1000;    // Minimum time between excited movements -- ///{"min":500,"max":10000}
    constexpr unsigned long excitedWanderMaxInterval = 7000;    // Maximum time between excited movements -- ///{"min":1000,"max":20000}
    constexpr unsigned long excitedMinInterval = 20000;         // Minimum time between excited standup actions -- ///{"min":5000,"max":60000}
    constexpr unsigned long excitedMaxInterval = 30000;         // Maximum time between excited standup actions -- ///{"min":10000,"max":90000}
    constexpr uint8_t excitedMinPitch = 50;                     // Minimum pitch for excited mood -- ///{"min":0,"max":180}
    constexpr uint8_t excitedMaxPitch = 110;                    // Maximum pitch for excited mood -- ///{"min":0,"max":180}
    constexpr uint8_t excitedMinYaw = 40;                       // Minimum yaw for excited mood -- ///{"min":0,"max":180}
    constexpr uint8_t excitedMaxYaw = 140;                      // Maximum yaw for excited mood -- ///{"min":0,"max":180}
    constexpr unsigned long excitedInterpolationTimeMin = 400;  // Minimum interpolation time for excited movements -- ///{"min":100,"max":5000}
    constexpr unsigned long excitedInterpolationTimeMax = 1000; // Maximum interpolation time for excited movements -- ///{"min":500,"max":10000}
    constexpr uint16_t excitedMaxHeight = 900;                  // Maximum height for excited standup (set above accessible range to cause bouncing) -- ///{"min":130,"max":1000}
    constexpr unsigned long excitedMinStandupInterval = 5000;   // Minimum time between excited standup actions -- ///{"min":1000,"max":30000}
    constexpr unsigned long excitedMaxStandupInterval = 9000;   // Maximum time between excited standup actions -- ///{"min":2000,"max":60000}

    // Moody Mood
    constexpr uint8_t moodyDroopTargetY = 150; // Pitch angle for moody droop position -- ///{"min":0,"max":180}
    constexpr uint8_t moodyRiseTargetY = 100;  // Pitch angle for moody rise position -- ///{"min":0,"max":180}
    constexpr uint16_t moodyHueBlue = 43000;   // Hue value for moody blue -- ///{"min":0,"max":65535}
    constexpr uint16_t moodyHuePurple = 50000; // Hue value for moody purple -- ///{"min":0,"max":65535}
    constexpr float moodyCenterSnapThreshold = 2.0f; ///{"min":0.5,"max":10}
    constexpr uint16_t moodySlumpTarget = 150;              // Height for moody slump position -- ///{"min":130,"max":770}
    constexpr unsigned long delayBeforeGettingMoody = 4000; // Time to wait before starting moody animation -- ///{"min":0,"max":30000}
    constexpr unsigned long moodyInterpTime = 4000;         // Interpolation time for moody movements -- ///{"min":1000,"max":20000}
    constexpr unsigned long whiteFadeDuration = 1500;       // White light fade duration in milliseconds (used during moody mood) -- ///{"min":500,"max":10000}
    constexpr unsigned long headHangTime = 1000;            // Duration to hang head down in moody mood -- ///{"min":500,"max":10000}
    constexpr unsigned long headDescendTime = 4000;         // Duration to lower head in moody mood -- ///{"min":1000,"max":20000}
    constexpr unsigned long headAscendTime = 3000;          // Duration to raise head in moody mood -- ///{"min":1000,"max":20000}

    // --------------------------------------------------------------------
    // HACK 4: Pomodoro Timer
    // --------------------------------------------------------------------
    constexpr unsigned long workingTime = 25UL * 60UL * 1000UL; // 25 min working time -- ///{"col":43,"options":["15UL","20UL","25UL","30UL","45UL","60UL"]}
    constexpr unsigned long breakTime = 5UL * 60UL * 1000UL; // 5 min short break -- ///{"col":41,"options":["5UL","10UL","15UL","20UL"]}
    constexpr unsigned long longBreakTime = 10UL * 60UL * 1000UL; // 10 min long break -- ///{"col":45,"options":["10UL","15UL","20UL","30UL"]}
    constexpr uint8_t numBreaksBeforeLongBreak = 3; // 3 cycles before a long break -- ///{"min":1,"max":10}

    constexpr uint16_t pomodoroWorkHue = 10;          // red-ish -- ///{"min":0,"max":65535}
    constexpr uint16_t pomodoroShortBreakHue = 22000; // green -- ///{"min":0,"max":65535}
    constexpr uint16_t pomodoroLongBreakHue = 31000;  // cyan -- ///{"min":0,"max":65535}

    constexpr uint8_t pomodoroSat = 255; // full saturation -- ///{"min":0,"max":255}
    constexpr uint8_t pomodoroVal = 255; // max brightness -- ///{"min":0,"max":255}
 
    // Joystick double-click window (ms). Affects how “snappy” double-click feels.
    constexpr uint16_t pomodoroDoubleClickMs = 450; ///{"min":100,"max":1000}
}

#endif // CONFIG_H