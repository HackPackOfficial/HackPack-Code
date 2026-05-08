#include "AutoMode.h"


void AutoMode::enter() {
  SERIAL_PRINTLN("AutoMode entered");
  resetPositions();
  if(!robot.calibrating) {
    setColor(CRGB(0, 255, 0));  // Green
  }
  else {
    robot.runningYawCalibration = true;
    autoState = STATE_SWEEP; 
  }
}

void AutoMode::exit() {
    resetPositions();
}

void AutoMode::setColor(CRGB color) {
    robot.leds[0] = color;
    FastLED.show();
}

const char* AutoMode::name() {
    return "Auto_Mode";
}

void AutoMode::runStateMachine() {

  switch (autoState)
    {
    case STATE_IDLE:
    if(robot.calibrating) {
      return; 
    }
    if(robot.currButton == 1) {
      servoEase(startPWM);
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
    autoState = STATE_IDLE;
    robot.runningYawCalibration = false;
    break;
  default:
   SERIAL_PRINTLN("No state chosen");
    break;
    }
}   

void AutoMode::resetMemory() {
  yawPosition = 0;
  hitsIndex = 0;
  lastMinima = -5;
  memset(smoothedDistances, 0, sizeof(smoothedDistances));
  memset(window, 0, sizeof(window));
  memset(hits, 0, sizeof(hits));
}

void AutoMode::handleSweep()
{
  robot.ranger.readSingle(true);
  robot.data = robot.ranger.ranging_data;
  distance = robot.data.range_mm;

  if ((distance >= MAXD) || (distance < 0) || (robot.data.range_status != 0) || (robot.data.peak_signal_count_rate_MCPS <= SIGNAL_STRENGTH))
    distance = -1;  // Use -1 for invalid readings instead of MAXD
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

  robot.yawServo.writeMicroseconds(conversionMicro(yawPosition)); // move motor

}


void AutoMode::handleLockServo()
{
  for (int j : hits)
  {
    bool shoot = false;
    if (j != 0)
    {
      int adjustedOffset = ceil(map(j, 1.0, POSITIONS - 1, robot.OFFSET_MAX, robot.OFFSET_MIN));
      servoEase(conversionMicro(j - adjustedOffset));
      digitalWrite(LASER_PIN, HIGH);
      int hitDistance = smoothedDistances[j];
      SERIAL_PRINTLN(hitDistance);
        
      int tilt = calculateLaunchAngle(hitDistance);
      if (tilt != 0) shoot = true;
      if (shoot) 
      {
        if(hitDistance < closeDistance) tilt += (robot.TILT_ADJUST/2);
        int newTilt = conversionMicro(tilt + robot.TILT_ADJUST);
        if(newTilt > magazineCollisionAngle) newTilt = magazineCollisionAngle;
        robot.pitchServo.writeMicroseconds(newTilt);
        delay(200);
        if(hitDistance < closeDistance){
          launchRoutine(500, 300);
        }
        else{
          launchRoutine(400, 300);
        }
        shoot = false; 
      }
      digitalWrite(LASER_PIN, LOW);
      delay(250);
    }
  }
  delay(250);
  autoState = STATE_DONE;
}

#pragma region MOVING AVERAGE
void AutoMode::updateWindow()
{
  int windowIndex = yawPosition % WINDOW;
  window[windowIndex] = distance;
}

void AutoMode::smoothingDistance()
{
  int sum = 0;
  int count = 0;

  for (int i = 0; i < WINDOW; i++)
  {
    if (window[i] > 0)  // Only include positive valid readings (exclude -1)
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
    smoothedDistances[yawPosition] = -1;  // Indicates no valid readings in window
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

void AutoMode::detect()
{
  // float threshold = 10.0; // Minimum difference for significance
  for (int i = 1; i < POSITIONS - 1; i++)
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
        int start = i;
        while (i < POSITIONS - 1 && (smoothedDistances[i + 1] - smoothedDistances[i]) == 0)
        {
          i++; // Move through the flat region
        }
        int end = i;
        if (smoothedDistances[end + 1] > smoothedDistances[end])
        {                                 // Confirm rise out of flat region
          int center = (start + end) / 2; // Calculate center
          hits[hitsIndex] = center;
          // processingHits(center);
          hitsIndex++;
        } 
      }
    }
  }
}

void AutoMode::filterHits()
{
  for (int i = 1; i < OBJECTS; i++)
  {
    if (hits[i] != 0 && hits[i - 1] != 0)
    {
      int stepSpace = calculateSpacer(smoothedDistances[hits[i]]);
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

void AutoMode::reverseHits()
{
  for (int i = 0; i < OBJECTS / 2; i++)
  {
    int temp = hits[i];
    hits[i] = hits[OBJECTS - 1 - i];
    hits[OBJECTS - 1 - i] = temp;
  }
}
#pragma endregion

#pragma region TARGET SPACING

int AutoMode::calcThreshold(int distance)
{
  return THRESHOLD + (int)(THRESHOLD_TUNING * distance);
}

int AutoMode::calculateSpacer(int distance)
{
  float centralAngle = (360 * cupR * 0.5) / (2 * PI * distance);
  return round(centralAngle / degreesPerStep);
}
#pragma endregion


#pragma region VISUALIZATIONS 
//used in Processing
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
