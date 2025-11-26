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
#include <SparkFun_TB6612.h>                                //motor driver library
#include <Servo.h>                                          //servo controller library
#include <OneButton.h>                                      //button handling library

#define KP                0.4                               //P term for PID controller
#define KD                0.8                               //D term for PID controller
#define AIN1              9                                 //motor driver
#define BIN1              7                                 //motor driver
#define AIN2              10                                //motor driver
#define BIN2              6                                 //motor driver
#define PWMA              11                                //motor driver
#define PWMB              5                                 //motor driver
#define STBY              8                                 //motor driver
#define OFFSET_A          1                                 //Switch to -1 if the motor is running backwards
#define OFFSET_B          1                                 //Switch to -1 if the motor is running backwards
#define LEFT_SENSOR_PIN   A1                                //Left line sensor pin
#define RIGHT_SENSOR_PIN  A0                                //Right sensor pin
#define BUTTON_PIN        12                                //The pin the switch for detecting domino outage is connected to.

Motor motor1 = Motor(AIN1, AIN2, PWMA, OFFSET_A, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, OFFSET_B, STBY);
int combinedmotorspeed, leftMotorSpeed, rightMotorSpeed;    //creating variables like this initializes all their values to 0.

// Define variables for PID control
float lastError, derivative, controlSignal;                 //these initialize to 0/
int error;
int TOP_SPEED = 0; 

//Line sensor variables
int leftSensorValue, rightSensorValue;                      //variables for storing the value of the line sensors

OneButton stopButton = OneButton(BUTTON_PIN, true, false);  //Active low, internal pullup resistor
bool stopButtonPressed = false;                             //Boolean for tracking the state of the button

void setup()
{
  Serial.begin(9600);                          // Start the serial communication for debugging
  digitalWrite(STBY, HIGH);                    // Enable the motor driver
  
  //Set up the button handler to detect long press of limit switch. 
  //Have to use longPress because switch is pressed and held, not pressed and release as a single click.
  stopButton.setDebounceMs(50);
  stopButton.setPressMs(50);                  // Sets up the button handler to wait for 200ms before reporting the limit switch is pressed.
  stopButton.attachLongPressStart([]() {       // Lambda function that sets stopButtonPressed flag to true when the switch is pressed. 
    stopButtonPressed = true;                
  });
  stopButtonPressed = false;
  TOP_SPEED = 255; 
  Serial.println("Staring...");
}

void loop() {
  stopButton.tick();                           // Update the button state

  // If the button is pressed, adjust the top speed
  if (stopButtonPressed) {
    stopButtonPressed = false;                 // Reset the button pressed flag
    TOP_SPEED += 85;                           // Increase the top speed by 85
    if (TOP_SPEED > 255) {                     // Wrap around to 0 if it exceeds 255
      TOP_SPEED = 0;
    }
    Serial.print("Button pressed, new TOP_SPEED: ");
    Serial.println(TOP_SPEED);
  }

  //Read the line sensors and calculate control signal.
  leftSensorValue = analogRead(LEFT_SENSOR_PIN);                //Read line sensor values
  rightSensorValue = analogRead(RIGHT_SENSOR_PIN);
  error = leftSensorValue - rightSensorValue;                   //Calculate the error (difference between the two sensor values)
  derivative = error - lastError;                               //Calculate the derivative term, which is how fast the error is changing
  controlSignal = (KP * error) + (KD * derivative);             //Calculate the control signal
  lastError = error;                                            //Store the current error for the next iteration

  // Calculate motor speeds based on the control signal
  leftMotorSpeed = TOP_SPEED - controlSignal;                   //If the error is positive, this slows down the motor on the left
  rightMotorSpeed = TOP_SPEED + controlSignal;                  //and speeds up the motor on the right. Vice versa if the error is negative.

  // Ensure motor speeds are within the valid range
  leftMotorSpeed = constrain(leftMotorSpeed, 0, TOP_SPEED);     //This makes the speed be somewhere between 0 and TOP_SPEED
  rightMotorSpeed = constrain(rightMotorSpeed, 0, TOP_SPEED);
  
  // Apply motor speeds
  motor1.drive(leftMotorSpeed);
  motor2.drive(rightMotorSpeed);
}