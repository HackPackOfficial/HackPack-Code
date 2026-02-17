#ifndef ANIMATIONS_H
#define ANIMATIONS_H

const Keyframe startupAnimationRaw[] PROGMEM = {
  {90, 90, 400, INTERP_EASE_IN_OUT, -1, 0, 0, false}, // center
  {90, 90, 100, INTERP_EASE_IN_OUT, -1, 0, 0, false}, // wait
  {40, 90, 700, INTERP_EASE_IN_OUT, -1, 0, 0, false}, // left
  {40, 90, 150, INTERP_EASE_IN_OUT, -1, 0, 0, false}, // wait
  {110, 90, 1100, INTERP_EASE_IN_OUT, -1, 0, 0, false}, // right
  {110, 90, 500, INTERP_EASE_IN_OUT, 425, 0, 0, false}, // wait
  {90, 20, 350, INTERP_EASE_IN_OUT, 425, 0, 0, false}, // up
  {90, 90, 150, INTERP_EASE_IN_OUT, 425, 0, 0, false}, // down
  {100, 10, 250, INTERP_EASE_IN_OUT, 425, 0, 0, false}, // up
  {90, 80, 200, INTERP_EASE_IN_OUT, 425, 0, 0, false}, // down
  {80, 40, 250, INTERP_EASE_IN_OUT, 425, 0, 0, false}, // up
  {90, 80, 200, INTERP_EASE_IN_OUT, 365, 0, 0, false}, // down
  {90, 90, 400, INTERP_EASE_IN_OUT, 365, 0, 0, false}  // settle
};
const uint8_t startupAnimationRawLength = sizeof(startupAnimationRaw) / sizeof(Keyframe);

#endif // ANIMATIONS_H