#include "MatrixFace.h"

Face::Face(Adafruit_IS31FL3731 &disp)
    : display(disp) {
    // Constructor body
}


// Draw the appropriate face based on the current state. 
void Face::updateFace() {
    // If text is scrolling, update the text and skip the regular face display
    if (isScrolling) {
        updateTextScroll();
        return;
    }

    // Face display logic
    switch (state.currentFace) {
        case FaceStates::EYES_FORWARD:
            display.displayFrame(2);
            break;
        case FaceStates::EYES_LEFT:
            display.displayFrame(4);
            break;
        case FaceStates::EYES_RIGHT:
            display.displayFrame(3);
            break;
        case FaceStates::EYES_CONFUSED:
            display.displayFrame(5);
            break;
        case FaceStates::EYES_SQUINT:
            display.displayFrame(6);
            break;
        case FaceStates::SMILING_FACE:
            display.displayFrame(0);
            break;
        case FaceStates::ANGRY_FACE:
            display.displayFrame(1);
            break;
        case FaceStates::CALIBRATE:
            display.displayFrame(6);
            break;
        case FaceStates::TEXT:
            display.displayFrame(7);
            break;
        default:
            display.displayFrame(5);
            break;
    }
}

// Setter method for the face state
void Face::setFaceState(FaceStates newState) {
    state.currentFace = newState;
}

void Face::storeImagesInFrames() {
    for (int i = 0; i < 7; i++) {
        display.setFrame(i);
        display.drawGrayscaleBitmap(0, 0, listOfBitmaps[i], 16, 9);
    }
}

// Initialize the text scrolling state
void Face::writeText(const char* text) {
    scrollingText = text;
    scrollPosition = 0;  // for some reason only numbers <= 0 work for this, odd bug
    isScrolling = true;  // Set scrolling to true
    lastScrollUpdate = millis();  // Initialize the last update time
    display.setFrame(7);  // Use frame 7 for text
}

// Update the scroll position
void Face::updateTextScroll() {
    unsigned long currentMillis = millis();

    // Update scroll every 100 ms (adjust as needed)
    if (currentMillis - lastScrollUpdate >= 100) {
        display.clear();  // Clear the current frame
        display.setFrame(7);  // Use frame 7 for text
        display.setCursor(scrollPosition, 1);  // Set the cursor for the current scroll position
        display.setTextSize(1);
        display.setTextColor(40);
        display.setTextWrap(false);
        display.print(scrollingText);  // Print the scrolling text
        display.displayFrame(7);  // Show the text on frame 7

        // Move the text to the left
        scrollPosition--;

        // Reset the scroll position when text scrolls off the screen
        if (scrollPosition < -strlen(scrollingText) * 6) {  // Approximate pixel width per character
            isScrolling = false;  // Stop scrolling
        }

        // Update the last scroll update time
        lastScrollUpdate = currentMillis;
    }
}

bool Face::isTextBeingWritten() {
    return isScrolling;
}