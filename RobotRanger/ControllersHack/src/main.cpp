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
 * Robot Ranger Controllers HACK Code
 * Author: Crunchlabs LLC
 *
 *   This is revised code that runs many control modes, each interfacing with a different controller style. These
 *   are all controllers you can grab from previous Hack Packs (IR, RF, Joystick, etc.). Read each header file to see which are the 
 *   relevant pins for the controller modes. 
 *
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
#include "modes_headers/IRMode.h"
#include "modes_headers/JoystickMode.h"
#include "modes_headers/OmnibMode.h"
#include "modes_headers/ManualYawMode.h"
#include "modes_headers/ManualPitchMode.h"
ManualYawMode manualYaw;
ManualPitchMode manualPitch;
IRMode IR;
JoystickMode Joystick;
OmnibMode Omnibot;
Mode *modeList[] = {&manualYaw, &manualPitch};
Mode *controllerList[] = {&IR, &Joystick, &Omnibot};
ModesManager modeManager(modeList, sizeof(modeList) / sizeof(modeList[0]));
ModesManager controllerManager(controllerList, sizeof(controllerList) / sizeof(controllerList[0]));

#pragma endregion

#pragma region SETUP

void setup()
{
  SERIAL_BEGIN(115200);
  SERIAL_PRINTLN("Controllers Hack - Robot Ranger");
  Wire.begin();
  Wire.setClock(400000);

  initPins();
  initHardware();
  robot.defaultOffsets();
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

  if (robot.selectController)
  {
    if (robot.controllerIndex == 0)
    {
      handleButtonInput();
      modeManager.runStateMachine();
    }
    else
    {
      controllerManager.runStateMachine();
    }
  }
  else
  {
    handleButtonInputControllerSelect();
    robot.leds[0] = robot.controllerColors[robot.controllerIndex];
    FastLED.show();
    if (robot.currButton == 1)
    {
      enterController();
      blinkOnEnter(3, 200);
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

void enterController()
{
  robot.selectController = true;
  switch (robot.controllerIndex)
  {
  case 0:
    modeManager.setMode(&manualYaw);
    break;
  case 1:
    controllerManager.setMode(&IR);
    break;
  case 2:
    controllerManager.setMode(&Joystick);
    break;
  case 3:
    controllerManager.setMode(&Omnibot);
    break;
  }
}

void blinkOnEnter(int8_t numBlinks, uint16_t blinkTime)
{
  int8_t count = 0;
  bool ledOn = false;
  uint32_t lastBlinkTime = millis();
  while (count <= numBlinks * 2)
  {
    uint32_t currentTime = millis();
    if (currentTime - lastBlinkTime >= blinkTime)
    {
      ledOn = !ledOn;
      lastBlinkTime = currentTime;
      if (ledOn)
      {
        robot.leds[0] = robot.controllerColors[robot.controllerIndex];
      }
      else
      {
        FastLED.clear();
      }
      FastLED.show();
      count++;
    }
  }
}

/*
 ************************************************************************************
 * This function checks if a button is pressed, held, or released, and handles accordingly.
 *
 * See FOOTNOTES for more information about the button handling and resistor ladder.
 ************************************************************************************
 */
void handleButtonInput()
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

void handleButtonInputControllerSelect()
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
      incrementController(robot.currButton);
    }

    // Repeat action while holding
    if (robot.holding && (now - robot.lastRepeat >= robot.REPEAT_INTERVAL))
    {
      robot.lastRepeat = now;
      incrementController(robot.currButton);
    }
  }

  // Handle short press on release
  if (robot.currButton == 0 && robot.lastButton != 0)
  {
    if (!robot.holding)
    {
      incrementController(robot.lastButton);
    }
    robot.lastButton = 0;
    robot.holding = false;
  }
}

void incrementController(int16_t direction)
{
  if (direction == 2)
  {
    robot.controllerIndex--;
    if (robot.controllerIndex < 0)
      robot.controllerIndex = robot.numControllers - 1;
  }
  else if (direction == 3)
  {
    robot.controllerIndex++;
    if (robot.controllerIndex >= robot.numControllers)
      robot.controllerIndex = 0;
  }
}

#pragma endregion

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

float floatMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#pragma endregion
