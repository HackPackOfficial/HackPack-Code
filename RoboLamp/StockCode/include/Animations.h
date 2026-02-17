#ifndef ANIMATIONS_H
#define ANIMATIONS_H
/*
  Animations.h

  What this file does:
  - It stores "keyframes" that tell the lamp how to move. Each keyframe is one
    step in a short animation (like "look left", "look up", "wait").
  - The data lives in program memory (PROGMEM) so it doesn't use up RAM.

  What's inside one keyframe:
    { yaw, pitch, duration_ms, easingMode, height, controlYaw, controlPitch, useBezier }

  - yaw / pitch: where the lamp head should point (0 is one side, 180 the other).
  - duration_ms: how many milliseconds to take to move to that pose (more = slower).
  - easingMode: how the move speeds up or slows down ("smooth" vs "straight").
  - height: how tall the lamp should be (a number from the height sensor); -1s are used
    as placeholders and are replaced later with real values.
  - controlYaw / controlPitch and useBezier: used to make curved motions instead
    of straight lines. If useBezier is true, the move follows a curve using the
    control points.

  Quick tips:
  - Keep animations short so the lamp doesn't run out of memory.
  - Try short durations first until the motion looks right.
  - Use INTERP_EASE_IN_OUT for smooth starts and stops.
  - Set height to -1 if you want the lamp to keep its current height.
*/

// Animation 0 — linear motion (*This animation looks like a star pattern*)
const Keyframe animation0[] PROGMEM = {
    {118, 76, 310, 1, -1, 0, 0, false},
    {57, 105, 213, 3, -1, 0, 0, false},
    {114, 45, 321, 3, -1, 0, 0, false},
    {114, 127, 428, 3, -1, 0, 0, false},
    {64, 55, 401, 3, -1, 0, 0, false},
    {109, 70, 169, 2, -1, 0, 0, false}};
const uint8_t animation0Length = sizeof(animation0) / sizeof(Keyframe);

// Animation 1 — bezier motion (*This animation looks like a "U" pattern*)
const Keyframe animation1[] PROGMEM = {
    {68, 31, 150, 1, -1, 0, 0, false},
    {81, 109, 827, 3, -1, 45, 85, true},
    {102, 45, 713, 3, -1, 112, 113, true},
    {88, 103, 662, 3, -1, 122, 94, true},
    {71, 48, 610, 3, -1, 56, 92, true},
    {96, 72, 440, 2, -1, 67, 84, true}};
const uint8_t animation1Length = sizeof(animation1) / sizeof(Keyframe);

// Startup Animation — a series of poses to show off the lamp at startup (does not get called in excited mode)
const Keyframe startupAnimationRaw[] PROGMEM = {
    {90, 90, 400, 1, -1, 0, 0, false},   // head moves center
    {90, 90, 100, 0, -1, 0, 0, false},   // head pauses
    {40, 90, 700, 3, -1, 0, 0, false},   // head moves left
    {40, 90, 150, 0, -1, 0, 0, false},   // head pauses
    {110, 90, 1100, 3, -1, 0, 0, false}, // head moves right
    {110, 90, 500, 0, 425, 0, 0, false}, // head pauses while lamp rises
    {90, 20, 350, 3, 425, 0, 0, false},  // head moves up while lamp continues to rise
    {90, 90, 150, 3, 425, 0, 0, false},  // head moves down while lamp continues to rise
    {100, 10, 250, 3, 425, 0, 0, false}, // head moves up while lamp continues to rise
    {90, 80, 200, 3, 425, 0, 0, false},  // head moves down while lamp continues to rise
    {80, 40, 250, 3, 425, 0, 0, false},  // head moves up while lamp continues to rise
    {90, 80, 200, 3, 365, 0, 0, false},  // head moves down while lamp lowers a bit
    {90, 90, 400, 2, 365, 0, 0, false}   // head settles to center while lamp finishes lowering
};
const uint8_t startupAnimationRawLength = sizeof(startupAnimationRaw) / sizeof(Keyframe);

#endif // ANIMATIONS_H