#include "Arduino.h"
#include "MotionControl.h"

MotionControl::MotionControl(int leftSpeedPin, int leftDirPin, int rightSpeedPin, int rightDirPin) {
    _leftSpeedPin = leftSpeedPin;
    _leftDirPin = leftDirPin;
    _rightSpeedPin = rightSpeedPin;
    _rightDirPin = rightDirPin;

    pinMode(_leftSpeedPin, OUTPUT);
    pinMode(_leftDirPin, OUTPUT);
    pinMode(_rightSpeedPin, OUTPUT);
    pinMode(_rightDirPin, OUTPUT);
}

void MotionControl::drive(char direction, int speed) {
    bool forward = (direction == 'F' || direction == 'f');

    // Adjust direction for left motor
    int leftSpeed = speed;
    if (!forward) {
        leftSpeed = -leftSpeed;
    }
    if (leftMotorReversed) {
        leftSpeed = -leftSpeed;
    }

    // Adjust direction for right motor
    int rightSpeed = speed;
    if (!forward) {
        rightSpeed = -rightSpeed;
    }
    if (rightMotorReversed) {
        rightSpeed = -rightSpeed;
    }

    // Set motor speeds based on adjusted directions
    if (abs(speed) >= _speedThreshold) {
        digitalWrite(_leftDirPin, leftSpeed >= 0 ? LOW : HIGH);
        digitalWrite(_rightDirPin, rightSpeed >= 0 ? LOW : HIGH);
        analogWrite(_leftSpeedPin, abs(leftSpeed));
        analogWrite(_rightSpeedPin, abs(rightSpeed));
    } else {
        analogWrite(_leftSpeedPin, 0);
        analogWrite(_rightSpeedPin, 0);
    }
}

void MotionControl::rotate(char direction, int speed) {
    bool leftRotate = (direction == 'L' || direction == 'l');

    int leftSpeed = leftRotate ? -speed : speed;
    int rightSpeed = leftRotate ? speed : -speed;

    // Adjust for reverse polarity
    if (leftMotorReversed) {
        leftSpeed = -leftSpeed;
    }
    if (rightMotorReversed) {
        rightSpeed = -rightSpeed;
    }

    if (abs(speed) >= _speedThreshold) {
        digitalWrite(_leftDirPin, leftSpeed >= 0 ? LOW : HIGH);
        digitalWrite(_rightDirPin, rightSpeed >= 0 ? LOW : HIGH);
        analogWrite(_leftSpeedPin, abs(leftSpeed));
        analogWrite(_rightSpeedPin, abs(rightSpeed));
    } else {
        analogWrite(_leftSpeedPin, 0);
        analogWrite(_rightSpeedPin, 0);
    }
}


void MotionControl::curve(char direction, char toward, int lowSpeed, int highSpeed) {
    bool forward = (direction == 'F' || direction == 'f');
    bool leftTurn = (toward == 'L' || toward == 'l');

    int leftSpeed = lowSpeed;
    int rightSpeed = highSpeed;

    if (!forward) {
        leftSpeed = -leftSpeed;
        rightSpeed = -rightSpeed;
    }

    if (!leftTurn) {
        int temp = leftSpeed;
        leftSpeed = rightSpeed;
        rightSpeed = temp;
    }

    // Apply polarity reversal for each motor individually
    if (leftMotorReversed) {
        leftSpeed = -leftSpeed;
    }
    if (rightMotorReversed) {
        rightSpeed = -rightSpeed;
    }

    if (abs(leftSpeed) >= _speedThreshold || abs(rightSpeed) >= _speedThreshold) {
        analogWrite(_leftSpeedPin, abs(leftSpeed));
        analogWrite(_rightSpeedPin, abs(rightSpeed));
        digitalWrite(_leftDirPin, leftSpeed >= 0 ? LOW : HIGH);
        digitalWrite(_rightDirPin, rightSpeed >= 0 ? LOW : HIGH);
    } else {
        analogWrite(_leftSpeedPin, 0);
        analogWrite(_rightSpeedPin, 0);
    }
}

void MotionControl::direct(int leftSpeed, int rightSpeed) {
    // Apply polarity reversal for each motor individually
    if (leftMotorReversed) {
        leftSpeed = -leftSpeed;
    }
    if (rightMotorReversed) {
        rightSpeed = -rightSpeed;
    }

    if (abs(leftSpeed) >= _speedThreshold) {
        analogWrite(_leftSpeedPin, abs(leftSpeed));
    } else {
        analogWrite(_leftSpeedPin, 0);
    }

    if (abs(rightSpeed) >= _speedThreshold) {
        analogWrite(_rightSpeedPin, abs(rightSpeed));
    } else {
        analogWrite(_rightSpeedPin, 0);
    }

    digitalWrite(_leftDirPin, leftSpeed >= 0 ? LOW : HIGH);
    digitalWrite(_rightDirPin, rightSpeed >= 0 ? LOW : HIGH);
}

void MotionControl::stop() {
    analogWrite(_leftSpeedPin, 0);
    analogWrite(_rightSpeedPin, 0);
}

void MotionControl::setSpeedThreshold(int threshold) {
    _speedThreshold = threshold;
}
