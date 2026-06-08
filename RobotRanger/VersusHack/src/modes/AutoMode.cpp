#include "modes_headers/AutoMode.h"

/*
************************************************************************************
* The following is the implementation file for the auto mode. The header file contains
* the declarations for the class and the functions as well as mode description.
************************************************************************************
*/

void AutoMode::enter()
{
  SERIAL_PRINTLN("AutoMode entered");
  resetPositions();
  if (!robot.calibrating)
  {
    setColor(CRGB(0, 255, 0)); // Green
  }
  else
  {
    servoEase(startPWM);
    autoState = STATE_SWEEP;
  }
}

void AutoMode::exit()
{
  SERIAL_PRINTLN("AutoMode exited");
  resetPositions();
}

void AutoMode::setColor(CRGB color)
{
  robot.leds[0] = color;
  FastLED.show();
}

const char *AutoMode::name()
{
  return "Auto_Mode";
}

void AutoMode::runStateMachine()
{

  switch (autoState)
  {
  case STATE_IDLE:
    if (robot.calibrating)
    {
      return;
    }
    if (robot.currButton == 1)
    {
      servoEase(startPWM);
      robot.startingSolveTime = millis(); 
      autoState = STATE_SWEEP;
    }
    break;
  case STATE_SWEEP:
    handleSweep();
    break;
  case STATE_FIND:
    handleFind();
    break;
  case STATE_LOCK:
    handleLockServo();
    break;
  case STATE_DONE:
    resetPositions();
    resetMemory();
    robot.solveTime = millis() - robot.startingSolveTime; 
    robot.yourTurn = true; 
    autoState = STATE_IDLE; 
    robot.runningYawCalibration = false;
    robot.runningTiltCalibration = false;
    break;
  default:
    break;
  }
}

void AutoMode::resetMemory()
{
  yawPosition = 0;
  hitsIndex = 0;
  lastMinima = -5;
  memset(smoothedDistances, 0, sizeof(smoothedDistances));
  memset(window, 0, sizeof(window));
  memset(hits, 0, sizeof(hits));
}

/*
************************************************************************************
* The following function is used to handle the sweep of the yaw servo.
* The yaw servo is incremented by its minimum travel step and at every step the ToF sensor is read and the distance is recorded in an array.
* Only readings of a certain light strength are considered valid (white cups have higher signal strength than other things).
* The distance is then smoothed using a moving average filter. Invalid values are also thrown out.
************************************************************************************
*/
void AutoMode::handleSweep()
{
  robot.ranger.readSingle(true);
  robot.data = robot.ranger.ranging_data;
  distance = robot.data.range_mm;

  if ((distance >= MAXD) || (distance < 0) || (robot.data.range_status != 0) || (robot.data.peak_signal_count_rate_MCPS <= SIGNAL_STRENGTH))
    distance = -1; // Use -1 for invalid readings instead of MAXD
  updateWindow();
  smoothingDistance();

  yawPosition++;
  if (yawPosition >= POSITIONS) // EXIT CONDITION
  {
    autoState = STATE_FIND;
    robot.yawServo.writeMicroseconds(endPWM);
    SERIAL_PRINTLN(" ");
    return;
  }

  robot.yawServo.writeMicroseconds(conversionYawMicro(yawPosition)); // move motor
}

/*
************************************************************************************
* The following function is used to rotate the yaw servo to the cup position then launch the projectile.
* It also skips hits that are in the same position as the last hit to avoid redundant hits.
* The red dot laser is turned on and off to indicate the hit position.
************************************************************************************
*/
void AutoMode::handleLockServo()
{
  int16_t currHit = 0;
  int16_t lastHit = 0;
  int8_t positionTolerance = 5; // skips hit if its in the same position as the last hit and within 3 steps
  for (int j : hits)
  {
    if (j != 0)
    {
      int adjustedOffset = ceil(map(j, 1.0, POSITIONS - 1, robot.YAW_OFFSET_MAX, robot.YAW_OFFSET_MIN));
      currHit = j - adjustedOffset;
      if (abs(currHit - lastHit) > positionTolerance || currHit < positionTolerance) // skips hit if its in the same position as the last hit and within 3 steps
      {
        servoEase(conversionYawMicro(currHit));
        digitalWrite(LASER_PIN, HIGH);
        int16_t hitDistance = smoothedDistances[j];
        int16_t tilt = calculateLaunchAngle(hitDistance);

        if (tilt != 0)
        {
          tilt = conversionPitchMicro(round(tilt + robot.TILT_ADJUST));
          robot.pitchServo.writeMicroseconds(tilt);
          delay(200);
          launchRoutine(robot.waitTime);
        }
        digitalWrite(LASER_PIN, LOW);
        delay(200);
        lastHit = currHit;
      }
    }
  }
  delay(200);
  autoState = STATE_DONE;
}

#pragma region MOVING AVERAGE

/*
************************************************************************************
* The following functions are used to smooth distances by using a moving average filter.
************************************************************************************
*/
void AutoMode::updateWindow()
{
  int8_t windowIndex = yawPosition % WINDOW;
  window[windowIndex] = distance;
}

void AutoMode::smoothingDistance()
{
  int16_t sum = 0;
  int16_t count = 0;

  for (int16_t i = 0; i < WINDOW; i++)
  {
    if (window[i] > 0) // Only include positive valid readings (exclude -1)
    {
      sum += window[i];
      count++;
    }
  }

  if (count > 0)
  {
    smoothedDistances[yawPosition] = sum / count;
  }
  else
  {
    smoothedDistances[yawPosition] = -1; // Indicates no valid readings in window
  }
}
#pragma endregion

#pragma region CUP DETECTION

void AutoMode::handleFind()
{
  detect();
  filterHits();
  reverseHits();
  autoState = STATE_LOCK;
}

/*
************************************************************************************
* The following function is used to detect the cups by finding the minima in the smoothed distances.
* There are two kinds of minima, sharp ones (very far away cups) and flat ones (close by cups).
************************************************************************************
*/
void AutoMode::detect()
{
  for (int16_t i = 1; i < POSITIONS - 1; i++)
  {
    if (smoothedDistances[i] > 0)
    {
      float slope1 = smoothedDistances[i] - smoothedDistances[i - 1];
      float slope2 = smoothedDistances[i + 1] - smoothedDistances[i];

      // Case 1: Sharp Minimums
      if (slope1 < 0 && slope2 > 0)
      {
        // Sharp minimum at index i
        hits[hitsIndex] = i;
        // processingHits(i);
        hitsIndex++;
      }

      // Case 2: Flat Region Minimum
      else if (slope1 < 0 && slope2 == 0)
      {
        int16_t start = i;
        while (i < POSITIONS - 1 && (smoothedDistances[i + 1] - smoothedDistances[i]) == 0)
        {
          i++; // Move through the flat region
        }
        int16_t end = i;
        if (smoothedDistances[end + 1] > smoothedDistances[end])
        {                                     // Confirm rise out of flat region
          int16_t center = (start + end) / 2; // Calculate center
          hits[hitsIndex] = center;
          // processingHits(center);
          hitsIndex++;
        }
      }
    }
  }
}

/*
************************************************************************************
* The following function is used to filter the hits.
* It is used to remove hits that are too close to each other.
************************************************************************************
*/
void AutoMode::filterHits()
{
  for (int16_t i = 1; i < OBJECTS; i++)
  {
    if (hits[i] != 0 && hits[i - 1] != 0)
    {
      int16_t stepSpace = calculateSpacer(smoothedDistances[hits[i]]);
      if (stepSpace >= (hits[i] - hits[i - 1]))
      {
        if (smoothedDistances[hits[i]] > smoothedDistances[hits[i - 1]])
        {
          hits[i] = 0;
        }
        else
        {
          hits[i - 1] = 0;
        }
      }
    }
  }
}

/*
************************************************************************************
* The following function is used to reverse the hits.
* It is used to reverse the hits so that the hits are in the correct order and the end of a sweep.
************************************************************************************
*/
void AutoMode::reverseHits()
{
  for (int16_t i = 0; i < OBJECTS / 2; i++)
  {
    int16_t temp = hits[i];
    hits[i] = hits[OBJECTS - 1 - i];
    hits[OBJECTS - 1 - i] = temp;
  }
}

/*
************************************************************************************
* The following function is used to calculate the spacer between the hits so that the hits are not too close to each other.
* This is based off of the physical geometry of the robot and the "shadow" of the cup, which changes with distance.
************************************************************************************
*/
int16_t AutoMode::calculateSpacer(int16_t distance)
{
  float centralAngle = (360 * cupR * 0.5) / (2 * PI * distance);
  return round(centralAngle / degreesPerStep);
}

#pragma endregion

#pragma region VISUALIZATIONS

/*
************************************************************************************
* The following functions are used to print the distances and smoothed distances to the serial monitor.
* You can also write a sketch in a program called "Processing" to visualize the distance in a radar style format.
************************************************************************************
*/
void AutoMode::processingDistances()
{
  SERIAL_PRINT(yawPosition);
  SERIAL_PRINT(",");
  SERIAL_PRINT(distance);
  SERIAL_PRINTLN(" ");
}

void AutoMode::processingSmoothedDistances()
{
  SERIAL_PRINT(yawPosition);
  SERIAL_PRINT(",");
  SERIAL_PRINT(smoothedDistances[yawPosition]);
  SERIAL_PRINTLN(" ");
}

void AutoMode::processingHits(int hit)
{
  SERIAL_PRINT("line");
  SERIAL_PRINT(",");
  SERIAL_PRINT(hit);
  SERIAL_PRINTLN(" ");
}
#pragma endregion
