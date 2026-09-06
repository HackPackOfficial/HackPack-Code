#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Optional: namespace to avoid global conflicts
namespace Config
{
    // Behavior Toggles
    constexpr bool useSerial = false; ///{"options":["true","false"]}
    constexpr bool useStartupAnim = false; ///{"options":["true","false"]}
    constexpr bool useSmiley = true; ///{"options":["true","false"]}
    constexpr uint8_t startupMood = 0;            // 0=distracted, 1=excited, 2=moody, 3=focused -- ///{"range":[0,3]}
    constexpr uint16_t startupSmileyHue = 50000;  // Default purple -- ///{"min":0,"max":65535}
    constexpr uint16_t startupTargetHeight = 300; // Set default target height to a middle value for safety -- ///{"min":130,"max":770}

    // Menu Positions
    constexpr int8_t menuYaw = 90; ///{"min":0,"max":180}
    constexpr int8_t menuPitch = 80; ///{"min":0,"max":180}

    // Range Limits
    const uint8_t servoMinYawConstraint = 5;     // Limits the servo to a maximum range just shy of its hard-limit of 0 -- ///{"min":0,"max":90}
    const uint8_t servoMaxYawConstraint = 175;   // Limits the servo to a maximum range just shy of its hard-limit of 180 -- ///{"min":90,"max":180}
    const uint8_t servoMinPitchConstraint = 10;  // Limits the servo to a maximum range just shy of its hard-limit of 0 -- ///{"min":0,"max":90}
    const uint8_t servoMaxPitchConstraint = 170; // Limits the servo to a maximum range just shy of its hard-limit of 180 -- ///{"min":90,"max":180}

    // Mood Timing
    constexpr unsigned long moodIdleDelay = 8000; ///{"min":1000,"max":30000}
    constexpr float moodSnapThreshold = 3.0f; ///{"min":0.5,"max":10}
    constexpr unsigned long snapbackDebounceTime = 200; ///{"min":50,"max":1000}

    // Distracted Mood
    constexpr unsigned long distractedMinInterval = 5000; ///{"min":1000,"max":20000}
    constexpr unsigned long distractedMaxInterval = 9000; ///{"min":1000,"max":30000}
    constexpr uint8_t wanderRange = 80; ///{"min":10,"max":120}
    constexpr float distractedReturnThreshold = 2.0f; ///{"min":0.5,"max":10}
    const uint8_t servoMinDistractedYawConstraint = 20; ///{"min":0,"max":90}
    const uint8_t servoMaxDistractedYawConstraint = 160; ///{"min":90,"max":180}
    const uint8_t servoMinDistractedPitchConstraint = 50; ///{"min":0,"max":90}
    const uint8_t servoMaxDistractedPitchConstraint = 170; ///{"min":90,"max":180}

    // Excited Mood
    constexpr unsigned long excitedWanderMinInterval = 1000; ///{"min":500,"max":10000}
    constexpr unsigned long excitedWanderMaxInterval = 7000; ///{"min":1000,"max":20000}
    constexpr unsigned long excitedMinInterval = 20000; ///{"min":5000,"max":60000}
    constexpr unsigned long excitedMaxInterval = 30000; ///{"min":10000,"max":90000}
    constexpr uint8_t excitedMinPitch = 50; ///{"min":0,"max":180}
    constexpr uint8_t excitedMaxPitch = 110; ///{"min":0,"max":180}
    constexpr uint8_t excitedMinYaw = 40; ///{"min":0,"max":180}
    constexpr uint8_t excitedMaxYaw = 140; ///{"min":0,"max":180}
    constexpr unsigned long excitedInterpolationTimeMin = 400; ///{"min":100,"max":5000}
    constexpr unsigned long excitedInterpolationTimeMax = 1000; ///{"min":500,"max":10000}
    constexpr uint16_t excitedMaxHeight = 900; ///{"min":130,"max":1000}
    constexpr unsigned long excitedMinStandupInterval = 5000; ///{"min":1000,"max":30000}
    constexpr unsigned long excitedMaxStandupInterval = 9000; ///{"min":2000,"max":60000}

    // Moody Mood
    constexpr uint8_t moodyDroopTargetY = 130; ///{"min":0,"max":180}
    constexpr uint8_t moodyRiseTargetY = 100; ///{"min":0,"max":180}
    constexpr uint16_t moodyHueBlue = 43000; ///{"min":0,"max":65535}
    constexpr uint16_t moodyHuePurple = 50000; ///{"min":0,"max":65535}
    constexpr float moodyCenterSnapThreshold = 2.0f; // degrees -- ///{"min":0.5,"max":10}
    constexpr uint16_t moodySlumpTarget = 100; ///{"min":0,"max":770}
    constexpr unsigned long delayBeforeGettingMoody = 4000; ///{"min":0,"max":30000}
    constexpr unsigned long moodyInterpTime = 4000; ///{"min":1000,"max":20000}
    constexpr unsigned long whiteFadeDuration = 1500; ///{"min":500,"max":10000}
    constexpr unsigned long headHangTime = 1000; ///{"min":500,"max":10000}
    constexpr unsigned long headDescendTime = 4000; ///{"min":1000,"max":20000}
    constexpr unsigned long headAscendTime = 3000; ///{"min":1000,"max":20000}

    // namespace Config
    constexpr unsigned long idleDelayAfterStartup = 4000; // 4 seconds delay before idle wandering animation starts -- ///{"min":0,"max":20000}
    constexpr unsigned long userNotControllingInterval = 500; ///{"min":100,"max":5000}

    // IR manual control speed toggles. Press "star" for slow, "pound" for fast.
    // These values are passed into handleManualControl() as "delta" values.
    // Larger magnitude = bigger per-frame servo step.
    constexpr int16_t irHeadDeltaSlow = 1000; // Change this to edit the responsiveness of manual control in "slow" mode (smaller = less responsive) -- ///{"min":100,"max":10000}
    constexpr int16_t irHeadDeltaFast = 4000; // Change this to edit the responsiveness of manual control in "fast" mode (smaller = less responsive) -- ///{"min":100,"max":10000}

    // IR height "pulse" duration for STAR/POUND (ms)
    constexpr uint16_t irHeightPulseMsSlow = 300; // Change this to edit the duration of the height-change pulse in "slow" mode -- ///{"min":50,"max":2000}
    constexpr uint16_t irHeightPulseMsFast = 650; // Change this to edit the duration of the height-change pulse in "fast" mode -- ///{"min":50,"max":2000}
}

#endif // CONFIG_H