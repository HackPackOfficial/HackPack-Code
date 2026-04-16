#ifndef MECHANISMS_H
#define MECHANISMS_H
#include <Arduino.h>
#include <Crunchlabs_DRV8835.h>
#include <AS5600.h>
#include <EventDelay.h>

// pin assignments
constexpr uint8_t TABLE_SPEED_PIN = 5,
                  TABLE_DIR_PIN = 4,
                  ARM_SPEED_PIN = 6,
                  ARM_DIR_PIN = 7,
                  LED_PIN = 11;


// motor objects
DRV8835 tableMotor(TABLE_SPEED_PIN, TABLE_DIR_PIN, 50, true);
DRV8835 armMotor(ARM_SPEED_PIN, ARM_DIR_PIN, 50, true);

// encoder objects
AS5600L tableEncoder;
AS5600 armEncoder;

// constants and variables related to the arm
constexpr uint8_t MAX_CONSTRAINED_RADIUS = 75; // the maximum absolute value radius the arm will be allowed to move to during normal operation
int8_t currentArmPosition = 0;                 // current arm position in millimeters from center of table
int16_t currentArmAngle = 0;

// constants and variables related to the table
int32_t currentTableAngle = 0, lastTableAngle = 0;

// function prototypes
void homeArm();
int16_t convertArmRadiusToAngle(int16_t radiusMM);    // convert arm radius in millimeters to angle in encoder counts
int16_t convertArmAngleToRadius(int16_t angleCounts); // convert arm angle in encoder counts to radius in millimeters
int16_t convertPotValToArmRadius(uint16_t potVal);
int16_t convertPotValToTableSpeed(int16_t potVal);
void moveArmToRadius(int8_t targetRadius, int16_t currentAngle);
void moveArmToAngle(int16_t targetAngle, int16_t currentAngle);


// this version of the homing sequence relies on standard timing functions. This only works because
// homeArm() is called before the PWM clock divisors get changed and before Mozzi gets started.
void homeArm()
{
  static int16_t lastPos = 0, currentPos = 100; // arbitrary values that need to start different from each other
  armMotor.setSpeed(255);
  delay(100); // delay for a brief period to take up gear backlash
  // just keep spinning the motor until the arm stops moving
  while (currentPos != lastPos)
  {
    lastPos = currentPos;
    currentPos = armEncoder.getCumulativePosition(); // read current position
    delay(100);                                      // basically a short debouncing window.
  }
  // arm is homed, stop the motor.
  armMotor.setSpeed(0);

  // shift the angle around so that 0 angle is directly over center of table
  armEncoder.resetCumulativePosition(661);
}

/**
 * @brief converts an arm radius (from center of table in millimeters) to the angle in encoder counts that the arm needs to be at to reach that radius.
 *
 * @param radiusMM the radius in millimeters
 *
 * @return the angle in encoder counts that corresponds to the input radius
 *
 * @paragraph The angle is defined from the line that connects the center of the table and the center of the arm gear. As the angle
 * gets more positive, the arm moves toward its vertical position. Negative angles move the other side of the center
 * line, bringing the sensor arm down toward the knobs and buttons.
 *
 * This function is an approximation, but a pretty good one. At worst, it will have an error of 1 degree. The actual function that
 * precisely converts radius to angle in encoder counts (4096 per revolution) is:
 *
 * angle = (4096 / pi) * arcsin(radiusMM / 216)
 *
 * This radius is actually the chord length of the arc described by the sensor arm, starting from the point where that arc is
 * coincident with the table. The above function is derived from the formula for chord length. It's nice because it's exact, but it's
 * terrible for use in this context because it has floating point numbers (pi) and trig functions (arcsine). The ATMega328P is
 * extremely slow at performing floating point operations and trigonometric functions. It only has hardware for adding, subtracting,
 * and multiplying integers, as well as hardware for performing bitwise operations directly on binary numbers, so if you need any
 * other math operation, it gets built from those basic operations repeated a bunch of times, making it really slow. In situations
 * like this, where we need as much processing power as possible for running Mozzi, it's worth trying to find good linear approximations
 * of the actual math function that only use those basic operations. The approximation I derived is:
 *
 * angle = (774 * radiusMM) >> 7;
 *
 * That's equivalent to (774 * radiusMM) / 128. The denominator of 128 is nice because that's 2^7, so instead of performing the slow
 * operation of dividing by 128, we can perform the extremely fast operation of bitshifting right by 7 places.
 *
 * One final note on this is that bit shift operations are not valid on signed integers (those that can contain negative values).
 * You can perform the operation and move the bits, but it won't yield the mathematical results you expect. That's because one of the
 * bits is the sign bits, and tells the processor if the number is negative or not. Performing a bit shift moves that sign bit and messes
 * up the math. So the function implementation takes the absolute value of the radius parameter and puts that in an unsigned integer
 * so that bit shifting will work. At some point it could be interesting to test various versions of this to see which is fastest.
 * Shifting by powers of 2 is faster than shifting by other numbers, so there's a chance that doing something like the following could be
 * faster than what I have now:
 *
 * return ((774 * absRadius) >> 8) << 1;
 *
 * Or maybe just forgo the intermediate step of using absolute value and unsigned integers, and just divide the signed radius value by 128.
 *
 * If you want you can see a spreadsheet that I made to help figure all this out. It's not particularly well organized, and it's missing
 * the original math that I did by hand, as well as the Desmos graphs I made to check my reasoning, but it should help get some of the
 * concepts clarified. https://docs.google.com/spreadsheets/d/1RaxqJCClSnBzAPjxKA0sKCvFVXpXQ3Gc1AZQWVGnURM/edit?usp=sharing
 *
 * Also, I inlined this function because it is really short and called frequently, and in cases like those, inlining a function can
 * potentially speed up code execution. I think that's because the processor doesn't have to go through executing a separate function call.
 * I'm not sure though, and don't even know if this speeds things up, but why not? It doesn't hurt, might help, and we need all the speed
 * we can get back for running Mozzi.
 */
inline int16_t convertArmRadiusToAngle(int16_t radiusMM)
{
  uint16_t absRadius = abs(radiusMM); // bit shifting operations on signed integers create weird results, so we have to use an unsigned int
  uint16_t intermediate = (774 * absRadius) >> 7;
  return (radiusMM < 0) ? -1 * static_cast<int16_t>(intermediate) : static_cast<int16_t>(intermediate);
}

/**
 * @brief converts an arm angle in encoder counts to the corresponding radius of the sensor from the center of the table in millimeters.
 *
 * @param angleCounts the angle in encoder counts (4096 per revolution).
 *
 * @return the radius of the sensor from the center of the table in millimeters.
 *
 * @paragraph This is similar to the function convertArmRadiusToAngle() in which I used a linear integer approximation of a function that
 * relies on floating point numbers and trigonometry functions. The precise function that converts an angle to a radius is:
 *
 * radius = 216 * sin(angle * pi / 4096)
 *
 * This can be pretty well approximated by the function
 *
 * radius = 81 * angle >> 9
 *
 * At worst, this approximation will produce a positional error of 2mm. This seems acceptable in this context, since backlash in the gears
 * and play in the arm height adjustment system produce more that 2mm of error, and this isn't a system where 2mm of error is going to make
 * a huge difference. The average and median error are .17mm and .42mm respectively in my sample set, which you can see in the spreadsheet
 * linked above.
 *
 * One important note about the linear approximations in both of these functions is that they only work because we're working with a restricted
 * domain. The sensor arm can only move through limited range of motion from about -44 degrees to 58 degrees. The functions that convert
 * angle to radius and radius to angle both look reasonably linear across that domain. However, if we were dealing with an unrestricted range
 * of motion (if the arm could spin in a full circle), these approximations wouldn't work at all, and we would need to look into other methods
 * of speeding up these math operations. If you're interested in this, you could look into piecewise linear functions and the closely related
 * concept of a lookup table with linear interpolation. You could also look into CORDIC (coordinate rotation digital computer) which is a common
 * efficient method for calculating things like trig functions. This kind of thing is used everywhere in software. Every video game you've ever
 * played has basically been a giant system of linear algebra and trig functions.
 */
inline int16_t convertArmAngleToRadius(int16_t angleCounts)
{
  uint16_t absAngleCounts = abs(angleCounts);
  uint16_t intermediate = (81 * absAngleCounts) >> 9;
  return (angleCounts < 0) ? -1 * static_cast<int16_t>(intermediate) : static_cast<int16_t>(intermediate);
}


int16_t convertPotValToArmRadius(uint16_t potVal)
{
  constexpr uint8_t DEADBAND = 15;
  potVal = (potVal < 512 + DEADBAND && potVal > 512 - DEADBAND) ? 512 : potVal; // add a bit of deadband
  return map(potVal, 1023, 0, MAX_CONSTRAINED_RADIUS, -MAX_CONSTRAINED_RADIUS);
}

int16_t convertPotValToTableSpeed(int16_t potVal)
{
  constexpr uint8_t tableSpeedDeadband = 40;
  // create a deadband in the potentiometer reading to account for noise around the detent
  potVal = (potVal > 512 + tableSpeedDeadband || potVal < 512 - tableSpeedDeadband) ? potVal : 512;
  int16_t speed = map(potVal, 0, 1023, -255, 255);
  return speed;
}



void moveArmToAngle(int16_t targetAngle, int16_t currentAngle)
{
  static EventDelay accelTimer;
  static bool initialize = true;
  constexpr uint8_t accelerationMultiplier = 1; // value added to last motor speed to cause acceleration
  constexpr uint16_t accelerationInterval = 2;  // milliseconds between speed updates
  // the first time this function is called, we need to initialize the timer to run at the appropriate interval
  if (initialize)
  {
    accelTimer.set(accelerationInterval);
    initialize = false;
  }

  static int16_t lastMotorSpeed = 0, targetMotorSpeed = 0;
  constexpr uint8_t DEADBAND = 10; // if the current position is within this distance of the target, we reached the target
  constexpr uint8_t maxMotorSpeed = 140;
  static int8_t directionVector = 1;

  int16_t displacement = targetAngle - currentAngle;

  if (abs(displacement) <= DEADBAND)
  {
    targetMotorSpeed = 0;
  }
  else
  {
    if (accelTimer.ready())
    {
      directionVector = displacement > 0 ? 1 : -1;
      targetMotorSpeed = constrain((accelerationMultiplier * directionVector) + lastMotorSpeed, -1 * maxMotorSpeed, maxMotorSpeed);

      accelTimer.start();
    }
  }
  armMotor.setSpeed(targetMotorSpeed);
  lastMotorSpeed = targetMotorSpeed;
}


void moveArmToRadius(int8_t targetRadius, int16_t currentAngle = 0)
{
  // I need to add a feature that cuts motor speed to 0 when end of range of arm motion is reached. I think sometimes it exceeds the
  // range that works properly for the angle to distance calculator and causes odd behavior.

  static EventDelay accelTimer;
  static bool initialize = true;
  constexpr uint8_t accelerationMultiplier = 4; // value added to last motor speed to cause acceleration
  constexpr uint16_t accelerationInterval = 1;  // milliseconds between speed updates
  // the first time this function is called, we need to initialize the timer to run at the appropriate interval
  if (initialize)
  {
    accelTimer.set(accelerationInterval);
    initialize = false;
  }

  static int16_t lastMotorSpeed = 0, targetMotorSpeed = 0;
  static int16_t targetAngle = 0;
  constexpr uint8_t DEADBAND = 10; // if the current position is within this distance of the target, we reached the target
  constexpr uint8_t maxMotorSpeed = 128;
  static int8_t directionVector = 1;

  // handle motor acceleration

  targetAngle = convertArmRadiusToAngle(targetRadius);
  int16_t displacement = targetAngle - currentAngle;

  if (abs(displacement) <= DEADBAND)
  {
    targetMotorSpeed = 0;
  }
  else
  {
    if (accelTimer.ready())
    {
      directionVector = displacement > 0 ? 1 : -1;
      targetMotorSpeed = constrain((accelerationMultiplier * directionVector) + lastMotorSpeed, -1 * maxMotorSpeed, maxMotorSpeed);
      accelTimer.start();
    }
  }
  armMotor.setSpeed(targetMotorSpeed);
  lastMotorSpeed = targetMotorSpeed;
}





#endif