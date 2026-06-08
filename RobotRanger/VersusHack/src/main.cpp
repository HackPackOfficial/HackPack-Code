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
 * Robot Ranger Versus HACK Code
 * Author: Crunchlabs LLC
 *
 *   This is revised code that runs a new play mode for the Robot Ranger. It is a simplified version of the stock code
 *   that alternates between two modes: Automatic and Manual. The robot starts in Automatic mode, and when the button is pressed
 *   it searches for the targets (cups) you lay out. It then attempts to launch the projectiles into the cups. After that it resets and
 *   goes into manual mode where you can control the yaw and pitch servos to aim yourself at the targets. You have a time limit to 
 *   "beat" the robot base on its run time in the Automatic mode. See if you can best it!
 ************************************************************************************
 */

#pragma endregion README

#pragma region MODES AND LIBRARIES

/*
 ************************************************************************************
 * The following are the modes used in the robot. To add or subtract more, just
 * append to/remove from the array and follow the inclusion formatting.
 * If removing modes, be sure to recompile before uploading to see if errors occur.
 ************************************************************************************
 */
#include "Shared.h"
#include "ModesManager.h"
#include "modes_headers/CalibrationMode.h"
#include "modes_headers/AutoMode.h"
#include "modes_headers/ManualYawMode.h"
#include "modes_headers/ManualPitchMode.h"
CalibrationMode calibration;
AutoMode automatic;
ManualYawMode manualYaw;
ManualPitchMode manualPitch;
Mode *robotModeList[] = {&calibration, &automatic};
Mode *yourModeList[] = {&manualYaw, &manualPitch};
ModesManager robotModeManager(robotModeList, sizeof(robotModeList) / sizeof(robotModeList[0])); // do not touch this, allows for dynamic mode addition/subtraction
ModesManager yourModeManager(yourModeList, sizeof(yourModeList) / sizeof(yourModeList[0]));     // do not touch this, allows for dynamic mode addition/subtraction

#pragma endregion

#pragma region SETUP

void setup()
{
  SERIAL_BEGIN(115200);
  SERIAL_PRINTLN("Versus Hack - Robot Ranger");
  Wire.begin();
  Wire.setClock(400000);

  initPins();
  initHardware();
  robot.defaultOffsets();
  checkEEPROM();
  robotModeManager.setMode(&automatic);
}

#pragma endregion

#pragma region MAIN LOOP
/*
 ************************************************************************************
 * This is the main loop. It logs the current time, checks if any button is pressed (and handles accordingly)
 * then runs the relevant mode's state machine. Handle button input is essentially the mode menu navigation.
 ************************************************************************************
 */
void loop()
{
  robot.currTime = millis();
  robot.currButton = getButton(analogRead(BUTTON_RESISTOR_LADDER));
  if (robot.robotTurn)
  {
    handleButtonInputRobot();
    robotModeManager.runStateMachine();
    if(robot.yourTurn){
      robot.robotTurn = false;
      yourModeManager.setMode(&manualYaw);
    }
  }
  else
  {
    handleButtonInputYour();
    yourModeManager.runStateMachine();
    if(millis() - robot.startingPlayTime >= robot.solveTime) {
      robot.robotTurn = true; 
      robot.yourTurn = false; 
      robot.solveTime = INT32_MAX; 
      robot.startTiming = false; 
      robotModeManager.setMode(&automatic); 
      
    }
  }
}

#pragma endregion

#pragma region SHARED FUNCTIONS

/*
 ************************************************************************************
 * Resets the servos to their default positions. Used when exiting a mode or resetting the robot.
 ************************************************************************************
 */
void resetPositions()
{
  robot.yawServo.writeMicroseconds(visualYawCenterPWM);
  robot.pitchServo.writeMicroseconds(startPWM);
  robot.launchServo.write(90);
}

/*
 ************************************************************************************
 * The following functions convert a yaw position or pitch angle to a microsecond value for the relevant servo.
 * For example, in yaw the 0 position is 500 microseconds, 200th/final position is 2500 microseconds.
 ************************************************************************************
 */
int16_t conversionYawMicro(int16_t position)
{
  return max(startPWM, startPWM + position * 10);
}

int16_t conversionPitchMicro(int16_t angle)
{
  return map(angle, 0, 180, startPWM, endPWM);
}

/*
 ************************************************************************************
 * Converts an analog value from the resistor ladder to a button number.
 * Numbers are mathematically derived from resistor values.
 ************************************************************************************
 */
int8_t getButton(int16_t v)
{
  if (v > 660)
    return 1; // middle button
  if (v > 530)
  {
    robot.calibrating = false;
    return 2; // left arrow button
  }
  if (v > 370)
  {
    robot.calibrating = false;
    return 3; // right arrow button
  }
  return 0;
}

/*
 ************************************************************************************
 * This function checks if a button is pressed, held, or released, and handles accordingly.
 *
 * See FOOTNOTES for more information about the button handling and resistor ladder.
 ************************************************************************************
 */
void handleButtonInputRobot()
{
  // SPECIAL CASE FOR CALIBRATION MODE
  if (robot.calibrating)
    handleCalibration();

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
    uint32_t now = millis();
    uint32_t heldTime = now - robot.pressStart;

    if (!robot.holding && heldTime >= robot.HOLD_TIME)
    {
      robot.holding = true;
      robot.lastRepeat = now;
      robotModeManager.changeMode(robot.currButton);
    }

    // Repeat action while holding
    if (robot.holding && (now - robot.lastRepeat >= robot.REPEAT_INTERVAL))
    {
      robot.lastRepeat = now;
      robotModeManager.changeMode(robot.currButton);
    }
  }

  // Handle short press on release
  if (robot.currButton == 0 && robot.lastButton != 0)
  {
    if (!robot.holding)
    {
        robotModeManager.changeMode(robot.lastButton);
    }
    robot.lastButton = 0;
    robot.holding = false;
  }
}

void handleButtonInputYour()
{

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
    uint32_t now = millis();
    uint32_t heldTime = now - robot.pressStart;

    if (!robot.holding && heldTime >= robot.HOLD_TIME)
    {
      robot.holding = true;
      robot.lastRepeat = now;
      yourModeManager.changeMode(robot.currButton);
    }

    // Repeat action while holding
    if (robot.holding && (now - robot.lastRepeat >= robot.REPEAT_INTERVAL))
    {
      robot.lastRepeat = now;
      yourModeManager.changeMode(robot.currButton);
    }
  }

  // Handle short press on release
  if (robot.currButton == 0 && robot.lastButton != 0)
  {
    if (!robot.holding)
    {
        yourModeManager.changeMode(robot.lastButton);
    }
    robot.lastButton = 0;
    robot.holding = false;
  }
}

void decideTurn()
{
  if (millis() - robot.startingPlayTime >= robot.solveTime)
  {
    robot.yourTurn = true;
    robot.solveTime = INT32_MAX;
  }

  if (robot.robotTurn)
  {

    robot.startingPlayTime = millis();
    robot.robotTurn = false;
  }

  if (robot.yourTurn)
  {

    robot.yourTurn = false;
  }
}

void waitForButtonPress()
{
  bool pressed = false;
  while (!pressed)
  {
    if (!digitalRead(BUTTON_RESISTOR_LADDER))
      pressed = true;
  }
}

void handleCalibration()
{
  if (robotModeManager.currentMode() != &calibration && !robot.runningYawCalibration && !robot.runningTiltCalibration)
  {
    robotModeManager.setMode(&calibration);
  }
  else if (robotModeManager.currentMode() != &automatic && (robot.runningYawCalibration || robot.runningTiltCalibration))
  {
    robotModeManager.setMode(&automatic);
  }
}

#pragma endregion

#pragma region DISTANCE MATH

/*
 ************************************************************************************
 * The following functions are the math for calculating the launch angle.
 * They basically brute force search for the launch angle that results in the closest distance to the target.
 * There is also a function to correct for the offset of the sensor, both physically and deviation based from manufacturing.
 ************************************************************************************
 */
int16_t calculateLaunchAngle(float distance)
{
  if (distance > MAXD)
    return 0;

  robot.sensAdj = calcOffset(distance);
  float adjustedDistance = distance + cupR + baseR + sensorOffset + robot.sensAdj; // adjusted to the right reference frame

  /*
   ************************************************************************************
   * Tolerance parameters for the brute force search.
   * You start by having strict criteria for "closeness" and then relax the criteria as you go
   * through the math in an attempt to find some solution, even if it's not the perfect solution.
   ************************************************************************************
   */
  float initialTolerance = 0.5;
  float maxTolerance = 3.5;
  float toleranceIncrement = 0.5;

  for (float tolerance = initialTolerance; tolerance <= maxTolerance; tolerance += toleranceIncrement)
  {
    // Iterate through all possible angles
    for (float angle = 90.0; angle >= 45.0; angle -= 0.1)
    {
      float angleRad = radians(angle); // Convert angle to radians
      float predictedDistance = calculateRoot(angleRad);
      float perAngleCorrections = launcherR * cos(angleRad) - (barrelAdjust * sin(angleRad)); // Based off the physical geometry of the robot

      float difference = abs(predictedDistance - (adjustedDistance - perAngleCorrections));

      // Check if the predicted distance is valid and within the current tolerance
      if (predictedDistance > 0 && difference <= tolerance)
      {
        return (int)round((angle - 45.0) * GEAR_RATIO); // Return adjusted angle (aka solution)
      }
    }
  }

  return 0; // Return 0 if no match is found (aka no solution)
}

// sensor offset compensation, empirically derived
float calcOffset(float distance)
{
  return floatMap(distance, 0.0, (float)MAXD, (float)robot.sensAdjMin, (float)robot.sensAdjMax);
}

/*
 ************************************************************************************
 * The following function calculates the real root of the quadratic trajectory equation.
 ************************************************************************************
 */
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

/*
 ************************************************************************************
 * The following function eases the servo to the target angle. It uses a simple smoothing algorithm.
 * It is an overloaded function such that the default is easing the yaw servo, but can also be used to ease the pitch servo.
 ************************************************************************************
 */
void servoEase(int16_t targetAngleMicroseconds)
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

void servoEase(int16_t targetAngleMicroseconds, bool isYaw, int16_t delayMs)
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

/*
 ************************************************************************************
 * The following function launches the projectile. It uses a simple, non-blocking state machine to control the launch servo.
 * It first pulls back the striker, waits for a bit to let the projectile settle, and then releases the striker.
 ************************************************************************************
 */
void launchRoutine(uint16_t waitTimeMs)
{
  bool runningRoutine = true;
  bool prevLaunchState = HIGH;
  int8_t currLaunchState = 0;
  uint32_t currTime = 0;
  uint32_t lastTime = 0;

  delay(SMOOTHING);

  while (runningRoutine)
  {
    currTime = millis();
    switch (currLaunchState)
    {
    case 0: // pull
      robot.launchServo.write(0);
      if (digitalRead(BREAK_PIN) == LOW)
      {
        lastTime = currTime;
        robot.launchServo.write(90);
        currLaunchState = 1;
      }
      break;
    case 1: // wait
      if (currTime - lastTime > waitTimeMs)
        currLaunchState = 2;
      break;
    case 2: // release
      robot.launchServo.write(0);
      if (currTime - lastTime > SMOOTHING)
      {
        bool launchState = digitalRead(BREAK_PIN);
        if (prevLaunchState == LOW && launchState == HIGH)
        {
          lastTime = currTime;
          currLaunchState = 3;
        }
        prevLaunchState = launchState;
      }
      break;
    case 3: // done + debouncing
      if (currTime - lastTime > robot.debounceTime)
      {
        robot.launchServo.write(90);
        runningRoutine = false;
      }
      break;
    default:
      break;
    }
  }
}
#pragma endregion

#pragma region SETUP FUNCTIONS

/*
 ************************************************************************************
 * The following functions are simply wrappers for initializing the pins and hardware.
 * They are called in the setup function.
 ************************************************************************************
 */

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
  // TIME OF FLIGHT SENSOR - Reference FOOTNOTES for more information about this sensor.
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

  // Initializing LED
  FastLED.addLeds<SK6812, LED_DATA_PIN, GRB>(robot.leds, NUM_LEDS);
  FastLED.setBrightness(255);
  FastLED.clear();
}

/*
 ************************************************************************************
 * The following functions are used to check the EEPROM for the calibration values.
 * If the values are not found, it maintains the default values.
 * These are to make sure calibration is not forgotten between power cycles.
 *
 * The default values are based off of the physical geometry of the robot. The tilt adjust
 * is derived from the worst case angular deviation of the pitch servo at factory assembly.
 * The yaw offset is empirically derived from servo backlash and rotational inertia.
 *
 * Reference FOOTNOTES for more information.
 ************************************************************************************
 */

void checkEEPROM()
{
  bool tiltFlag = true;
  bool yawFlag = true;
  EEPROM.get(calibrateAddressTilt, tiltFlag);
  EEPROM.get(calibrateAddressYaw, yawFlag);
  if (!tiltFlag)
  { // EEPROM is 255 by default which is true, so actually false is the calibrated flag
    EEPROM.get(tiltAddress, robot.TILT_ADJUST);
  }
  if (!yawFlag)
  {
    EEPROM.get(yawOffsetAddress, robot.YAW_OFFSET_MIN);
    robot.YAW_OFFSET_MAX = robot.YAW_OFFSET_MIN + robot.YAW_EXTRA;
  }
}

float floatMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#pragma endregion
