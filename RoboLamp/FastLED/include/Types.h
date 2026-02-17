#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>

enum InterpolationMode
{
    INTERP_LINEAR,
    INTERP_EASE_IN,
    INTERP_EASE_OUT,
    INTERP_EASE_IN_OUT
};

enum WhiteMode : uint8_t
{
  WHITE_WARM = 1,
  WHITE_SOFT = 2,
  WHITE_COOL = 3
};

enum MoodState
{
    MOOD_DISTRACTED,
    MOOD_HAPPY,
    MOOD_MOODY,
    MOOD_FOCUSED
};

enum MoodBlinkType
{
    BLINK_COLOR,
    BLINK_WARM,
    BLINK_SOFT,
    BLINK_COOL
};

enum AnimationType
{
  ANIM_STARTUP,
  ANIM_EXCITED
};

enum SmileyEyeDirection
{
  LOOK_LEFT,
  LOOK_RIGHT
};

struct Keyframe
{
    int8_t yaw;
    int8_t pitch;
    uint16_t duration;
    uint8_t easing;
    int16_t height;
    int8_t controlYaw;
    int8_t controlPitch;
    bool useBezier;
};

#endif