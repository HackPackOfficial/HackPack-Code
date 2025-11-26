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
  CALIBRATE,
  TEXT
};

// Struct to store the face's current state
struct FaceStateContainer {
  FaceStates currentFace;

  FaceStateContainer()
    : currentFace(FaceStates::EYES_SQUINT) {}  // Initializes to CALIBRATE by default
};

class Face {
public:
    Face(Adafruit_IS31FL3731 &display);  // Constructor now explicitly takes an IS31FL3731 object

    /**
     * @brief causes the LED matrix to display the appropriate face, determined by the current value of
     * FaceStateContainer::currentFace. Also handles updating text scrolls.
     * 
     * This function is really just a wrapper for a switch statement that calls display.displayFrame(). 
     * That is a function within the Adafruit_IS31FL3731 library that tells the LED matrix controller to
     * switch to displaying the contents of one of its 8 available frame buffers. 
     */
    void updateFace();  // Method to update and draw the face based on current state

    /**
     * @brief set the face that should be displayed on the next call of updateFace()
     * @param newState the face to display, of type FaceStates
     */
    void setFaceState(FaceStates newState);  

    /**
     * @brief returns the face state that is currently set.
     * @return FaceStates type. 
    */
    FaceStates getFaceState();
    
    /**
     * @brief copies the contents of the listOfBitmaps[8] array from microcontroller memory
     * into 8 available frame buffers in the IS31FL3731 LED matrix controller IC. 
     */
    void storeImagesInFrames();

    /**
     * @brief sets text to be written onto the LED matrix. 
     * @param text the text to be written on the matrix. 
     */
    void writeText(const char* text);

    /**
     * @brief returns a boolean value of true if text is still being scrolled across the screen.
     * @return boolean - true if text is still being scrolled on the matrix.
     */
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
