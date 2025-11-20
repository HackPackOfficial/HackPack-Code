#ifndef MATRIX_FACE_H
#define MATRIX_FACE_H

#include <Adafruit_GFX.h>
#include <Adafruit_IS31FL3731.h>  // Include the IS31FL3731 library
#include "FaceBitmaps.h"

// Enum class for face states
enum class FaceStates {
  EYES_FORWARD,
  EYES_LEFT,
  EYES_RIGHT,
  EYES_CONFUSED,
  EYES_SQUINT,
  SMILING_FACE,
  ANGRY_FACE,
  PLAYER_LOSES,
  TEXT
};

// Struct to store the face's current state
struct FaceStateContainer {
  FaceStates currentFace;

  FaceStateContainer()
    : currentFace(FaceStates::PLAYER_LOSES) {}  // Initializes to PLAYER_LOSES by default
};

class Face {
public:
    Face(Adafruit_IS31FL3731 &display);  // Constructor now explicitly takes an IS31FL3731 object

    void updateFace();  // Method to update and draw the face based on current state
    void setFaceState(FaceStates newState);  // Method to set the face state
    void storeImagesInFrames();
    void writeText(const char* text);
    bool isTextBeingWritten();

private:
    Adafruit_IS31FL3731 &display;  // Reference to the IS31FL3731 object
    FaceStateContainer state;  // Stores the current face state

    // For text scrolling
    bool isScrolling = false;
    int16_t scrollPosition = 0;
    unsigned long lastScrollUpdate = 0;
    const char* scrollingText = nullptr;

    // To handle non-blocking text scrolling
    void updateTextScroll();
};

#endif
