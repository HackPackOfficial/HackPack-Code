#ifndef FACE_BITMAPS_H
#define FACE_BITMAPS_H
#include <Arduino.h>

// Declare the bitmap arrays as extern
extern const uint8_t smiling_face[];
extern const uint8_t angry_face[];
extern const uint8_t eyes_forward[];
extern const uint8_t eyes_right[];
extern const uint8_t eyes_left[];
extern const uint8_t eyes_confused[];
extern const uint8_t squint_blink[];

// Group all the bitmaps into an array of pointers
extern const uint8_t* const listOfBitmaps[8];

#endif
