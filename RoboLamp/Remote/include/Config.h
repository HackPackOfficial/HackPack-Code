#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Optional: namespace to avoid global conflicts
namespace Config
{
    // Behavior Toggles
    constexpr bool useSerial = false;
    constexpr bool useStartupAnim = false;
    constexpr bool useSmiley = true;
    constexpr uint8_t startupMood = 0;            // 0=distracted, 1=excited, 2=moody, 3=focused
    constexpr uint16_t startupSmileyHue = 50000;  // Default purple
    constexpr uint16_t startupTargetHeight = 300; // Set default target height to a middle value for safety

    // Menu Positions
    constexpr int8_t menuYaw = 90;
    constexpr int8_t menuPitch = 80;

    // Range Limits
    const uint8_t servoMinYawConstraint = 5;     // Limits the servo to a maximum range just shy of its hard-limit of 0
    const uint8_t servoMaxYawConstraint = 175;   // Limits the servo to a maximum range just shy of its hard-limit of 180
    const uint8_t servoMinPitchConstraint = 10;  // Limits the servo to a maximum range just shy of its hard-limit of 0
    const uint8_t servoMaxPitchConstraint = 170; // Limits the servo to a maximum range just shy of its hard-limit of 180

    // Mood Timing
    constexpr unsigned long moodIdleDelay = 8000;
    constexpr float moodSnapThreshold = 3.0f;
    constexpr unsigned long snapbackDebounceTime = 200;

    // Distracted Mood
    constexpr unsigned long distractedMinInterval = 5000;
    constexpr unsigned long distractedMaxInterval = 9000;
    constexpr uint8_t wanderRange = 80;
    constexpr float distractedReturnThreshold = 2.0f;
    const uint8_t servoMinDistractedYawConstraint = 20;
    const uint8_t servoMaxDistractedYawConstraint = 160;
    const uint8_t servoMinDistractedPitchConstraint = 50;
    const uint8_t servoMaxDistractedPitchConstraint = 170;

    // Excited Mood
    constexpr unsigned long excitedWanderMinInterval = 1000;
    constexpr unsigned long excitedWanderMaxInterval = 7000;
    constexpr unsigned long excitedMinInterval = 20000;
    constexpr unsigned long excitedMaxInterval = 30000;
    constexpr uint8_t excitedMinPitch = 50;
    constexpr uint8_t excitedMaxPitch = 110;
    constexpr uint8_t excitedMinYaw = 40;
    constexpr uint8_t excitedMaxYaw = 140;
    constexpr unsigned long excitedInterpolationTimeMin = 400;
    constexpr unsigned long excitedInterpolationTimeMax = 1000;
    constexpr uint16_t excitedMaxHeight = 900;
    constexpr unsigned long excitedMinStandupInterval = 5000;
    constexpr unsigned long excitedMaxStandupInterval = 9000;

    // Moody Mood
    constexpr uint8_t moodyDroopTargetY = 130;
    constexpr uint8_t moodyRiseTargetY = 100;
    constexpr uint16_t moodyHueBlue = 43000;
    constexpr uint16_t moodyHuePurple = 50000;
    constexpr float moodyCenterSnapThreshold = 2.0f; // degrees
    constexpr uint16_t moodySlumpTarget = 100;
    constexpr unsigned long delayBeforeGettingMoody = 4000;
    constexpr unsigned long moodyInterpTime = 4000;
    constexpr unsigned long whiteFadeDuration = 1500;
    constexpr unsigned long headHangTime = 1000;
    constexpr unsigned long headDescendTime = 4000;
    constexpr unsigned long headAscendTime = 3000;

    // namespace Config
    constexpr unsigned long idleDelayAfterStartup = 4000; // 4 seconds delay before idle wandering animation starts
    constexpr unsigned long userNotControllingInterval = 500;

    // IR manual control speed toggles. Press "star" for slow, "pound" for fast.
    // These values are passed into handleManualControl() as "delta" values.
    // Larger magnitude = bigger per-frame servo step.
    constexpr int16_t irHeadDeltaSlow = 1000; // Change this to edit the responsiveness of manual control in "slow" mode (smaller = less responsive)
    constexpr int16_t irHeadDeltaFast = 4000; // Change this to edit the responsiveness of manual control in "fast" mode (smaller = less responsive)

    // IR height "pulse" duration for STAR/POUND (ms)
    constexpr uint16_t irHeightPulseMsSlow = 300; // Change this to edit the duration of the height-change pulse in "slow" mode
    constexpr uint16_t irHeightPulseMsFast = 650; // Change this to edit the duration of the height-change pulse in "fast" mode
}

#endif // CONFIG_H