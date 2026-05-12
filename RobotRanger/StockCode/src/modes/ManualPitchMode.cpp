#include "ManualPitchMode.h"

void ManualPitchMode::enter() {
    setColor(CRGB(128, 0, 128));  // Purple for Pitch
    initialPotValue = analogRead(POT_PIN);
    lastPitchAngle = startPWM;  // Start position
    robot.pitchServo.writeMicroseconds(lastPitchAngle);
    SERIAL_PRINTLN("ManualPitchMode entered");
}

void ManualPitchMode::exit() {
    // Don't reset positions when exiting, keep the last pitch position
    SERIAL_PRINTLN("ManualPitchMode exited");
}      

void ManualPitchMode::runStateMachine() {
    manualPitch();
    delay(SMOOTHING);
}

const char* ManualPitchMode::name() {
    return "Manual_Pitch_Mode";
}

void ManualPitchMode::setColor(CRGB color) {
    robot.leds[0] = color;
    FastLED.show();
}

void ManualPitchMode::manualPitch() {
    int rawValue = analogRead(POT_PIN);
    
    // Only move if potentiometer has been touched (changed from initial value)
    if (abs(rawValue - initialPotValue) > 10) {  // Small threshold to account for noise
        int angleValue = map(rawValue, 0, 1023, endPWM - 200, startPWM); // Reversed mapping for pitch
        robot.pitchServo.writeMicroseconds(angleValue);
        lastPitchAngle = angleValue; // Update lock position
    }

    if(robot.currButton == 1) {
    
            launchRoutine(500, 300);
    }
} 