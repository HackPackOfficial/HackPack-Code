#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>

enum InterpolationMode
{
    INTERP_LINEAR,
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

enum SmileyEyeDirection
{
  LOOK_LEFT,
  LOOK_RIGHT
};

#endif