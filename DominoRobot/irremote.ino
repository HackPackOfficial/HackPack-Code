/*
  ************************************************************************************
  * MIT License
  *
  * Copyright (c) 2025 Crunchlabs LLC (Domino Robot Code)

  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to deal
  * in the Software without restriction, including without limitation the rights
  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  * copies of the Software, and to permit persons to whom the Software is furnished
  * to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in all
  * copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
  * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
  * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
  * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  *
  ************************************************************************************
*/
#define IR_USE_AVR_TIMER1 // overrides conflicting pins in IRremote.hpp
#include <IRremote.hpp>
#include <SparkFun_TB6612.h>

#define IR_LEFT 0xF708FF00
#define IR_RIGHT 0xA55AFF00
#define IR_FORWARD 0xE718FF00
#define IR_HOLD 0x0
#define IR_BACKWARD 0xAD52FF00
#define IR_ONE 0xBA45FF00
#define IR_TWO 0xB946FF00

#define PWMA 11  // Adjust as necessary
#define PWMB 5  // Adjust as necessary
#define AIN1 9  // Adjust as necessary
#define BIN1 7  // Adjust as necessary
#define AIN2 10  // Adjust as necessary
#define BIN2 6  // Adjust as necessary
#define OFFSET_A 1         // Adjust if necessary
#define OFFSET_B 1         // Adjust if necessary
#define STBY 8
#define TOP_SPEED 90       // Define top speed
#define SERVO_RIGHT 20                                //PWM value to make the servo move right
#define SERVO_LEFT 180                               //PWM value to make the servo move left.
#define SERVO_PIN 13
#define SERVO_MIN_PULSE 650  // Minimum pulse width in microseconds
#define SERVO_MAX_PULSE 2400 // Maximum pulse width in microseconds
#define DISPENSE_DISTANCE 9000 //Value combinedMotorSpeed needs to reach to drop a domino. Decreasing this puts dominoes closer together.
#define PULSE_INTERVAL 20000 // Interval between pulses in milliseconds
#define IR_Input 3

bool servoRight = true;

// Initialize motors
Motor motor1 = Motor(AIN1, AIN2, PWMA, OFFSET_A, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, OFFSET_B, STBY);

int combinedmotorspeed, leftMotorSpeed, rightMotorSpeed;    //creating variables like this initializes all their values to 0.

//Servo servoMotor;                                           //Create an instance of a servo motor object

bool raceMode = false;
unsigned long speedBoost = 0;

bool holding = false; // This should be a global variable
unsigned long lastCommandTime = 0;
unsigned long lastServoTime = 0;

const unsigned long commandTimeout = 250; // Timeout in milliseconds

unsigned long distanceSinceLastDrop = DISPENSE_DISTANCE;    //Starting with this as DISPENSE_DISTANCE makes the vehicle drop a domino immediately.
bool dominoDropped = false;

const int RECV = IR_Input;
IRrecv irrecv(RECV);

void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn();
  digitalWrite(STBY, HIGH);  // Enable the motor driver
  pinMode(SERVO_PIN, OUTPUT);
  servoRight = true;
}

void loop() {
  bool commandReceived = false;
  unsigned long currentMillis = millis();
  if (raceMode) {
    speedBoost = (255 - TOP_SPEED);
  } else {
    speedBoost = 0;
  }

  if (servoRight) {
    pulseServoRight();
    // Serial.println("servoRight Right Right Right");
  } else {
    pulseServoLeft();
    // Serial.println("servoLeft Left Left Left");
  }

  if (irrecv.decode()) {
    commandReceived = true;
    unsigned long irCode = irrecv.decodedIRData.decodedRawData;
    Serial.println(irrecv.decodedIRData.decodedRawData, HEX);

    // Update the last command time whenever a command is received
    lastCommandTime = millis();


    if (!holding) {
      switch (irCode) {
        case IR_FORWARD:
          driveForward();
          break;
        case IR_LEFT:
          turnLeft();
          break;
        case IR_RIGHT:
          turnRight();
          break;
        case IR_BACKWARD:
          driveBackward();
          break;
        case IR_ONE:
          raceMode = false;
          break;
        case IR_TWO:
          raceMode = true;
          break;
      }
    } else if (holding && irCode == IR_HOLD) {
      // Holding, received IR_HOLD again
      // Serial.println("HOLDING...");
    } else {
      stopCar();
      holding = false;
      // Serial.println("HOLDING = FALSE");
    }

    irrecv.resume(); // Prepare for the next command
  }

  combinedmotorspeed = leftMotorSpeed + rightMotorSpeed;
  distanceSinceLastDrop += combinedmotorspeed;

  // Check for timeout to detect "no command received" condition
  if (holding && !commandReceived && millis() - lastCommandTime > commandTimeout) {
    // No command received within the timeout period
    stopCar();
    holding = false;
    // Serial.println("HOLDING = FALSE");
    // Serial.println("Timeout - STOP CAR");
    commandReceived = true;
  }
  
  if ((distanceSinceLastDrop >= DISPENSE_DISTANCE) && !dominoDropped && !raceMode) {              //if we've travelled far enough and a domino hasn't been dropped yet
      // dropDomino();                                  
      servoRight = true;
      distanceSinceLastDrop = 0;
      dominoDropped = true;                        //domino has been dropped
      Serial.println("Drop Domino_____________________");
      
    } else if ((distanceSinceLastDrop >= 0.7 * DISPENSE_DISTANCE) && dominoDropped && !raceMode) {  //if a domino was dropped and we're far enough away to close the domino gate without jamming
      // resetServo();
      servoRight = false;
      dominoDropped = false;                                                           //we haven't dropped the new domino, so now this is false
      Serial.println("Reset Servo_____________________");
    }
}

void driveForward() {
  motor1.drive(TOP_SPEED + speedBoost);
  motor2.drive(TOP_SPEED + speedBoost);
  leftMotorSpeed = TOP_SPEED;
  rightMotorSpeed = TOP_SPEED;
  // Serial.println("FORWARD");
  holding = true;
  // Serial.println("HOLDING = TRUE");
}

void turnLeft() {
  motor1.drive(TOP_SPEED + speedBoost);
  motor2.drive(0 - speedBoost * 1.5);
  leftMotorSpeed = 0;
  rightMotorSpeed = TOP_SPEED;
  // Serial.println("LEFT");
  holding = true;
  // Serial.println("HOLDING = TRUE");
}

void turnRight() {
  motor1.drive(0 - speedBoost * 1.5);
  motor2.drive(TOP_SPEED + speedBoost);
  leftMotorSpeed = TOP_SPEED;
  rightMotorSpeed = 0;
  // Serial.println("RIGHT");
  holding = true;
  // Serial.println("HOLDING = TRUE");
}

void stopCar() {
  motor1.drive(0);
  motor2.drive(0);
  leftMotorSpeed = 0;
  rightMotorSpeed = 0;
  // Serial.println("STOP");
  holding = true;
  // Serial.println("HOLDING = TRUE");
}

void driveBackward() {
  motor1.drive(-TOP_SPEED - speedBoost);
  motor2.drive(-TOP_SPEED - speedBoost);
  leftMotorSpeed = 0;
  rightMotorSpeed = 0;
  // Serial.println("BACKWARD");
  holding = true;
  // Serial.println("HOLDING = TRUE");
}

void pulseServoRight() {
  // Serial.println("PulseServoRight");
  unsigned long startTime;
  unsigned long pulseWidth = SERVO_MIN_PULSE; // Duration of the pulse in microseconds
  unsigned long period = PULSE_INTERVAL; // Total period duration in microseconds (20ms)

  // Start the pulse
  digitalWrite(SERVO_PIN, HIGH);
  startTime = micros(); // Record the start time

  // Wait for the pulse width duration
  while (micros() - startTime < pulseWidth) {
    // This loop waits until 2400 microseconds have passed since startTime
  }

  // End the pulse
  digitalWrite(SERVO_PIN, LOW);
  // Note: at this point, the time elapsed is roughly equal to pulseWidth

  // Wait for the rest of the period to complete the 20ms cycle
  while (micros() - startTime < period) {
    // This loop waits until 20000 microseconds have passed since startTime
  }

  // The function will return here, right after the 20ms period has finished
}

void pulseServoLeft() {
  // Serial.println("PulseServoLeft");
  unsigned long startTime;
  unsigned long pulseWidth = SERVO_MAX_PULSE; // Duration of the pulse in microseconds
  unsigned long period = PULSE_INTERVAL; // Total period duration in microseconds (20ms)

  // Start the pulse
  digitalWrite(SERVO_PIN, HIGH);
  startTime = micros(); // Record the start time

  // Wait for the pulse width duration
  while (micros() - startTime < pulseWidth) {
    // This loop waits until 2400 microseconds have passed since startTime
  }

  // End the pulse
  digitalWrite(SERVO_PIN, LOW);
  // Note: at this point, the time elapsed is roughly equal to pulseWidth

  // Wait for the rest of the period to complete the 20ms cycle
  while (micros() - startTime < period) {
    // This loop waits until 20000 microseconds have passed since startTime
  }

  // The function will return here, right after the 20ms period has finished
}