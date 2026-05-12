#pragma region LICENSE
/*
  ************************************************************************************
  * MIT License
  *
  * Copyright (c) 2026 Crunchlabs LLC (Robot Ranger: Stock Code)

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

#pragma endregion LICENSE

#pragma region README

/*
 ************************************************************************************
 * Robot Ranger Stock Code
 * xx.xx.2026
 * Version: 1.0
 * Author: Crunchlabs LLC
 *
 *   This stock code runs the Robot Ranger, a projectile motion launcher 
 *   coupled with a time of flight sensor. Time of flight is a method
 *   used to measure the distance to a target, and is the underlying 
 *   tech for radar, sonar, lidar, and more! Updates may occur as we add more 
 *   features and improve the code. Currently, it includes a variety of 
 *   modes, including auto-aiming, manual control, and a reflex game. 
 *   
 *   Each mode is a class that inherits from the "Mode" class. 
 *   This allows for modes to be added without changing the main code too much - a benefit over a 
 *   multi-nested state machine. Hack your own modes into the code! This flexibility is the basis of 
 *   polymorphism, a core concept in Object Oriented Programming.
 *
 *   To add modes, just create another pair of .h and .cpp files in the modes folder under src, 
 *   and add them to the modeList array in the setup declaration titled "ModesManager". 
 *   You can follow the structure below. 
 *
 *   For more info and to access the hacks, visit the Crunchlabs IDE. For community updates, 
 *   check out our Discord server. For help making your own hacks, ask our very own Mark Robot 
 *   on the IDE!
 ************************************************************************************
 */

#pragma endregion README


#pragma region MODES AND LIBRARIES

#include "Shared.h"
#include "ModesManager.h"
#include "CalibrationMode.h"
#include "AutoMode.h"
#include "ReflexMode.h"
#include "ManualYawMode.h"
#include "ManualPitchMode.h"
CalibrationMode calibration;
AutoMode automatic;
ReflexMode reflex;
ManualYawMode manualYaw;
ManualPitchMode manualPitch;
Mode *modeList[] = {&calibration, &automatic, &reflex, &manualYaw, &manualPitch}; // Can add new modes here after creating
ModesManager modeManager(modeList, sizeof(modeList) / sizeof(modeList[0]));

#pragma endregion

#pragma region SETUP

void setup()
{
  SERIAL_BEGIN(115200);
  SERIAL_PRINTLN("Version v.0 9/16 pre-submission");
  Wire.begin();
  Wire.setClock(400000);

  initPins();
  initHardware(); 
  defaultOffsets();
  checkEEPROM();

 pinMode(A0, INPUT_PULLUP);
 pinMode(A1, INPUT);
 pinMode(A2, INPUT);

  robot.OFFSET_MAX = robot.OFFSET_MIN + 4;
  modeManager.setMode(1);

  SERIAL_PRINTLN(robot.v0);
  SERIAL_PRINTLN(robot.TILT_ADJUST);
  SERIAL_PRINTLN(robot.OFFSET_MIN);
  SERIAL_PRINTLN(robot.OFFSET_MAX);
  SERIAL_PRINTLN(robot.calibrating);
  SERIAL_PRINTLN(robot.waitTime);
  
}

#pragma endregion

#pragma region MAIN LOOP
/*
  ************************************************************************************
  * Logs the current time, checks if any button is pressed (and handles accordingly)
  * then runs the relevant mode's state machine. Handle button input is essentially 
  * mode menu navigation. 
  ************************************************************************************
*/
void loop()
{
  robot.currTime = millis();
  robot.currButton = getButton(analogRead(BUTTON_RESISTOR_LADDER));
  handleButtonInput();           
  modeManager.runStateMachine(); 
}
#pragma endregion


#pragma region SHARED FUNCTIONS

void resetPositions()
{
  robot.yawServo.writeMicroseconds(visualYawCenterPWM);
  robot.pitchServo.writeMicroseconds(startPWM);
  robot.launchServo.write(90);
}

int conversionMicro(int angleDegrees)
{
  return max(startPWM, startPWM + angleDegrees * 10);
}

int getButton(int v)
{
  if (v > 660)
    return 1; // middle button
  if (v > 530)
    return 2; // left arrow button
  if (v > 370)
    return 3; // right arrow button
  return 0;
}

void handleButtonInput()
{
  //SPECIAL CASE FOR CALIBRATION MODE
  if(robot.calibrating) handleCalibration();
  

  // New button press
  if (robot.currButton != 0 && robot.currButton != robot.lastButton)
  {
    robot.pressStart = millis();
    robot.holding = false;
    robot.lastButton = robot.currButton;
  }

  // Handle button hold and repeat
  if (robot.currButton == robot.lastButton && robot.currButton != 0)
  {
    unsigned long now = millis();
    unsigned long heldTime = now - robot.pressStart;

    if (!robot.holding && heldTime >= robot.HOLD_TIME)
    {
      robot.holding = true;
      robot.lastRepeat = now;
      modeManager.changeMode(robot.currButton);
    }

    // Repeat action while holding
    if (robot.holding && (now - robot.lastRepeat >= robot.REPEAT_INTERVAL))
    {
      robot.lastRepeat = now;
      modeManager.changeMode(robot.currButton);
    }
  }

  // Handle short press on release
  if (robot.currButton == 0 && robot.lastButton != 0)
  {
    if (!robot.holding)
    {
      modeManager.changeMode(robot.lastButton);
    }
    robot.lastButton = 0;
    robot.holding = false;
  }
}

void waitForButtonPress()
{
  int pressed = false;
  while (!pressed)
  {
    if (!digitalRead(BUTTON_RESISTOR_LADDER))
      pressed = true;
  }
}

void handleCalibration() {
    if(modeManager.getCurrentModeIndex() == 0) {  
      modeManager.setMode(1);
    } else if(!robot.runningYawCalibration) {
      modeManager.setMode(0);
    }
}

#pragma endregion


#pragma region DISTANCE MATH
int calculateLaunchAngle(float distance)
{
  if (distance > MAXD)
    return 0;

  robot.sensAdj = calcOffset(distance);
  float adjustedDistance = distance + cupR + baseR + sensorOffset - robot.sensAdj; // adjusted to the right reference frame
  SERIAL_PRINT("Adjusted Distance: ");
  SERIAL_PRINTLN(adjustedDistance);

  // Tolerance parameters
  float initialTolerance = 0.5;   // mm
  float maxTolerance = 3.0;       // maximum tolerance
  float toleranceIncrement = 0.5; // increment for each loop

  // // Iterate through tolerances
  for (float tolerance = initialTolerance; tolerance <= maxTolerance; tolerance += toleranceIncrement)
  {
    // Iterate through angles
    for (float angle = 90.0; angle >= 45.0; angle -= 0.1)
    {
      float angleRad = radians(angle); // Convert angle to radians
      float predictedDistance = calculateRoot(angleRad);
      float perAngleCorrections = launcherR * cos(angleRad) - (barrelAdjust * sin(angleRad));

      float difference = abs(predictedDistance - (adjustedDistance - perAngleCorrections));

      // Check if the predicted distance is valid and within the current tolerance
      if (predictedDistance > 0 && difference <= tolerance)
      {
        // Serial.println("ANGLE: " + String(angle));
        // robot.goShoot = true;
        SERIAL_PRINTLN(launcherR * cos(angleRad));
        SERIAL_PRINT("Post Corrections ");
        SERIAL_PRINTLN(adjustedDistance - perAngleCorrections);
        return (int)round((angle - 45.0) * GEAR_RATIO); // Return adjusted angle
      }
    }
  }

  // Return 0 if no match is found
  return 0;
}

float calcOffset(float distance) // very magic number driven by trial and error
{
  if (distance <= 50 && distance >= 25 && (robot.data.peak_signal_count_rate_MCPS <= 30 || robot.data.peak_signal_count_rate_MCPS >= 100))
  {
    return -15; // very close to the sensor
  }
  else if (distance <= 25)
  {
    return -15; // somewhat close to the sensor
  }
  else
  {
    return map(distance, 25, MAXD, -10, 10); // regular distance
  }
}

float calculateRoot(float thetaRad)
{
  // starting height, which changes with angle
  float h0 = h + (launcherR * sin(thetaRad)) - (barrelAdjust * cos(thetaRad));

  // Coefficients for the quadratic trajectory equation
  float a = -(g / (2 * pow(robot.v0 * cos(thetaRad), 2)));
  float b = tan(thetaRad);
  float c = h0;
  float discriminant = b * b - 4 * a * c;

  // Check if there is a real solution
  if (discriminant < 0)
    return -1;

  // Calculate the positive root of x
  float xRoot = (-b - sqrt(discriminant)) / (2 * a);
  return xRoot > 0 ? xRoot : -1;
}

#pragma region LAUNCHING

void servoEase(int targetAngleMicroseconds)
{
  float servoAngle = robot.yawServo.readMicroseconds(); // use float to keep precision

  while (abs(servoAngle - targetAngleMicroseconds) > 2.0)
  { // 2us tolerance
    servoAngle = servoAngle * 0.9 + targetAngleMicroseconds * 0.1;
    robot.yawServo.writeMicroseconds((int)servoAngle);
    delay(30);
  }

  robot.yawServo.writeMicroseconds(targetAngleMicroseconds + backlashCompensation); 
  delay(30);
}

// Refactored servoEase that can control either yaw or pitch
void servoEase(int targetAngleMicroseconds, bool isYaw, int delayMs)
{
  float servoAngle;

  if (isYaw)
  {
    servoAngle = robot.yawServo.readMicroseconds();
  }
  else
  {
    servoAngle = robot.pitchServo.readMicroseconds();
  }

  while (abs(servoAngle - targetAngleMicroseconds) > 2.0)
  { // 2us tolerance
    servoAngle = servoAngle * 0.9 + targetAngleMicroseconds * 0.1;

    if (isYaw)
    {
      robot.yawServo.writeMicroseconds((int)servoAngle);
    }
    else
    {
      robot.pitchServo.writeMicroseconds((int)servoAngle);
    }
    delay(delayMs);
  }

  // Snap to target and compensate slightly
  if (isYaw)
  {
    robot.yawServo.writeMicroseconds(targetAngleMicroseconds + backlashCompensation);
  }
  else
  {
    robot.pitchServo.writeMicroseconds(targetAngleMicroseconds + backlashCompensation);
  }
  delay(30);
}

void launchRoutine()
{
  bool prevLaunchState = HIGH;
  robot.launchServo.write(0);
  while (true)
  {
    bool launchState = digitalRead(BREAK_PIN);
    if (prevLaunchState == LOW && launchState == HIGH)
    {
      robot.launchServo.write(90);
      break;
    }
    prevLaunchState = launchState;
    delay(SMOOTHING); // debouncing
  }
}

void launchRoutine(long pullDelayMs, long holdDelayMs)
{
  bool prevLaunchState = HIGH;
  robot.launchServo.write(0);
  delay(pullDelayMs);
  robot.launchServo.write(90);
  delay(holdDelayMs);
  robot.launchServo.write(0);

  while (true)
  {
    bool launchState = digitalRead(BREAK_PIN);
    if (prevLaunchState == LOW && launchState == HIGH)
    {
      robot.launchServo.write(90);
      break;
    }
    prevLaunchState = launchState;
    delay(SMOOTHING); // debouncing
  }
}

#pragma endregion

#pragma region SETUP FUNCTIONS

void initPins()
{
  pinMode(BREAK_PIN, INPUT_PULLUP);
  pinMode(LED_DATA_PIN, OUTPUT);
  pinMode(LASER_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(BUTTON_RESISTOR_LADDER, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void initHardware()
{

  // Initializing ToF sensor
  if (!robot.ranger.init())
    digitalWrite(LED_BUILTIN, HIGH);

  // ToF Sensor Settings
  robot.ranger.setDistanceMode(VL53L1X::Short);
  robot.ranger.setMeasurementTimingBudget(TIMING_BUDGET);
  robot.ranger.setROISize(4, 4);
  robot.ranger.setROICenter(199);

  // Attaching servos to pins
  robot.yawServo.attach(YAW_SERVO_PIN);
  robot.pitchServo.attach(PITCH_SERVO_PIN);
  robot.launchServo.attach(LAUNCH_SERVO_PIN);
  robot.yawServo.writeMicroseconds(visualYawCenterPWM);
  robot.pitchServo.writeMicroseconds(startPWM);

  // Constructing LED Object
  FastLED.addLeds<SK6812, LED_DATA_PIN, GRB>(robot.leds, NUM_LEDS); // Initialize FastLED
  FastLED.setBrightness(255);
  FastLED.clear();
}

void checkEEPROM() {
  bool tiltFlag = true;
  bool yawFlag = true;
  EEPROM.get(calibrateAddressTilt, tiltFlag);
  EEPROM.get(calibrateAddressYaw, yawFlag);
  if(!tiltFlag) { //EEPROM is 255 by default which is true, so actually false is the calibrated flag 
    EEPROM.get(tiltAddress, robot.TILT_ADJUST);
  }
  if(!yawFlag) {
    EEPROM.get(yawOffsetAddress, robot.OFFSET_MIN);
  }
}

void defaultOffsets() {
  robot.TILT_ADJUST = 11.0;
  robot.OFFSET_MIN = 18;
}

#pragma endregion
