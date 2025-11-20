/*
  ************************************************************************************
  *
  * Balance Bot Obstacle Avoidance Hack
  * 04.11.2025
  * 
  * This is the obstacle avoidance hack for the Hack Pack Balance Bot. This program is configured to read the MPU6050 IMU, control the motors, balance the bot using PID control, and avoid obstacles using an HCSR04 distance sensor.
  * The distance sensor will set the Balance bot to drive in reverse and turn slightly if something is detected within 30cm of the front of the bot.
  *
  * In order to stay balanced the robot has to successfully do two things: determine its lean (pitch angle), and drive its motors at the correct speed to keep the pitch angle close to the balanced angle of zero (setpoint).
  * First it uses trigonmetry, which is excellent for determining angles, to derive the angle of the robot from linear accelerometer data (and a raw angle reading of gyroscopic angle).
  * In this case the linear acceleration readings from our IMU form the sides of a triangle:
  *
  *     pitch = atan2(-accel_y, accel_z) * RAD_TO_DEG; // pitch derived via the inverse tangent of the y and z acceleration readings, converted to degrees from radians

                      ____-accel_y____                              
                      |             /                           |+                                  + undertuned PID pitch
                      |            /                            |*+                                 * tuned PID pitch
                      |           /                             |*   +                              ~ overtuned PID pitch
                      |          /                              |~*        +        ~               - setpoint
                      |         /                               |~ *              ~    ~  +
            accel_z   |        /                                |   *            ~      ~              +
                      |       /                                 | ~  *           **              *                    ++
                      |      /                                  |--~--*--------*----*-----~--*-----**----**-------------
                      |     /                                   |   ~   *    *          **
                      |    /                                    |    ~    **   ~            ~       ~
                      | * /                                     |     ~       ~              ~     ~
                      |  /                                      |      ~     ~                ~   ~
                      | /                                       |       ~   ~                   ~
                      |/   * pitch angle                        |         ~
                      *                                         |
  * 
  * Then it uses a calculus, utilizing a Proportional, Integral, Derivative equation (PID control) to adjust the motor speeds based on the difference between the desired angle and the actual angle.
  * Each part of the PID equation has "gain" number (kP, kI, kD) which is manually set to control how "strong" that part of the equation is, and tuned based on the behavior of the robot.
  * The PID equation is as follows: 
          - First PID looks at our error, which is the difference between the setpoint and the pitch angle

          error = setpoint - pitch            // imagine setpoint is 0 and pitch is 5, error would be -5 degrees

          - Then it calculates the P, I, and D terms based on our settings. Often the P term is the most important, but the I and D terms can help with long term drift and softening the response as we approach the setpoint

          where P = kP * error,               //P term, just based on the raw proportion of error, how far we are from the setpoint
                I = kI * integral of error,   //I term, which is the sum of all the error over time, to help with long term drift 
                D = kD * derivative of error, //D term, which is the rate of change of the error, to help soften the response as we approach the setpoint

            output = P + I + D,              //output is the sum of all three terms
  * 
  * The trig equation is done onboard the MPU6500 IMU sensor and the PID control is handled by a library which adds some helpful functionality, so you can't see the actual equations in the code. 
  * Although the math is getting done out of sight, fundamentally these equations *could* be calculated in our Arduino Nano's main loop, they would just be slower.
*/
#pragma endregion README
#pragma region LICENSE
/*
************************************************************************************
  * MIT License
  *
  * Copyright (c) 2025 Crunchlabs LLC (Balance Bot Code)

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

//////////////////////////////////////////////////
//  LIBRARIES AND CONFIGURATION //
//////////////////////////////////////////////////
#pragma region LIBRARIES AND CONFIG
// USEFUL TOGGLES
bool useDynamicSetpoint = true;      //enables dynamic pid_setpoint, which adjusts the pid_setpoint based on motor output, set false to use raw PID (which tends to drift off as gearbox of these motors has significant backlash)
bool useCalibration = false; //sets whether the MPU calibrates offsets on start up or not. This produces a delay on startup where you must hold the bot relatively upright
// you can also run a single calibration and set offset values using the IMUzero sketch.
bool useSerial = false; //enables serial output for debugging, set to false to disable serial output
// NOTE: Serial plotting can be very helpful for tuning PID, but it can also slow down the bot's response time, so it's best to disable it when not tuning

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// LED CONFIG
#pragma region LED config
#include <Adafruit_NeoPixel.h>

#define LED_COUNT 8  // Number of LEDs on the bar
#define LED_PIN 11   // LEDs on pin 11, can be changed to other digital pins
// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);  // Initializes LED object
unsigned long pattern_interval = 25;                                // Time between steps in the pattern (in milliseconds)
unsigned long lastUpdate = 0;                                       // Tracker for millis() when last update occurred
#pragma endregion LED config

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// HCSR04 DISTANCE SENSOR CONFIG
#pragma region Distance Sensor config
#include <NewPing.h>

#define TRIGGER_PIN 10   // Arduino pin tied to trigger pin on the ultrasonic sensor (HCSR04).
#define ECHO_PIN 9       // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 30  // Max distance in centimeters that the ultrasonic sensor will look for
// max dist can be increased but further distances can take longer to read which could disrupt balancing

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);  // NewPing setup of pins and maximum distance.

bool inRange = false;
int distance = 0;
#pragma endregion Distance Sensor config

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// STATE TRACKING
#pragma region State Machine
enum CtrlState { RIGHT,
                 LEFT,
                 CENTER };
CtrlState cmd_state = CENTER;

enum BalState { FWD,
                REV,
                HOLD,
                SLEEP };
BalState bal_state = HOLD;
BalState last_state = SLEEP;

enum LedState { RAINBOW,
                ROLL,
                BLUSH };
LedState led_state = BLUSH;

int cmd_count = 0;  // How many loops since last command. Used to track when to interpolate
int bal_count = 0;  // How many loops spent in current balance state, one way to time out of fwd / rev and return to hold
#pragma endregion State Machine

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// PID CONFIG
#pragma region PID config
// PID values control the response of the motors based on the pid_setpoint angle and current pitch angle to try to keep the pitch near the pid_setpoint
#include <PID_v1_bc.h>
double kP = 24.2; // the Proportional term of PID - directly proportional to the error (setpoint - pitch) between setpoint angle and input
double kI = 225; // the Integral term of PID - integrates the error over time to minimize accumulating error (if P is regularly falling short)
double kD = 0.76; // the Derivative term of PID - takes the derivative (or instantaneous rate of change) in order to soften the PI response as it approaches the input value

// Here are some extra PID settings to try out. Notice how the higher P and I values tend to jitter much more, because they are much more responsive to changes
// Higher PI values will work better with useDynamicSetpoint set to false, as the pid_setpoint gain is heavily influenced by the strength of P and I

// double kP = 17.9;
// double kI = 197;
// double kD = 0.67;

// double kP = 35.2;
// double kI = 500;
// double kD = 0.6;

// double kP = 44; // this one is pretty jittery - disabling dynamic setpoint should make it a bit better
// double kI = 340;
// double kD = 0.12;

// double kP = 24.9;
// double kI = 535;
// double kD = .48;

// A QUICK NOTE ON BALANCE BOT'S CONTROL SYSTEM
// More complex balancing systems will often utilize a primary PID loop which takes in target velocity as pid_setpoint and current velocity as input,
// and then output a desired angle into the secondary PID which controls motor output.
// This is called "cascade PID" however it requires a closed loop system with encoders which report back velocity.
// Balance Bot does not have encoders (meaning we don't know velocity) and it's motors have pretty significant backlash (wiggle room in the gearbox), making perfect balance difficult, so I had to get creative!

// ENTER DYNAMIC SETPOINT. Disclaimer: This is likely not the best control system, however it works well given the instability of the MPU and the backlash of the dc motor gearboxes.
// Essentially I've implemented a rudimentary version of what cascade PID does - using the output to the motors to adjust the pid_setpoint angle of the robot.
// This makes it more responsive to pushes, reduces drift, and allows for a driving control that tries to maintain forward movement without falling over.
// The downside is that it adds another number (setpoint_gain) which needs to be properly tuned for the system to respond well.
// overtuned setpoint_gain will cause the pid_setpoint to vary too much, causing the bot to oscillate more aggressively, but also make it very responsive to being pushed.
// undertuned setpoint_gain will make the performance closer to raw PID, which means the bot is more likely to drift and is not as responsive to being pushed.
// Try disabling useDynamicSetpoint and messing with different PID settings to see the difference.

float init_angle = -0.5;         // this is your "trim" to adjust the pid_setpoint (balanced) angle in case your bot is favoring forward over backward movement or vice versa. 
//Nominally init should be around 0 degrees, but may vary based on the mounting of the MPU sensor.
//negative values correspond to forward bias, positive values correspond to backward bias
float reset_angle = init_angle;  // our reset_angle is averaged over time in case the init_angle isn't accurate, making the system more responsive to changes in weight distribution
double pid_setpoint = init_angle;      //the "setpoint" of PID, in this case it's the angle we want the bot to try to be at to stay balanced or drive
double pid_input = 0;                  //the "input" of PID, which would be pitch, our actual current angle
double pid_output = 0;                 //the "output" of PID, which would be motor speed between -255 and 255 to capture any speed in either direction
PID pid(&pid_input, &pid_output, &pid_setpoint, kP, kI, kD, DIRECT);  // PID setup

double setpoint_gain = 0.000026;       //value used to dynamically adjust the pid_setpoint angle based on output, causing the bot to lean against a push, also helps with braking
//^^ try values between 0.00001 and 0.00005 and see how they affect the behavior. The higher setpoint_gain will be more responsive to being pushed, but may make driving more unpredictable
double reset_gain = 0.000005;          //value used to dynamically adjust the reset (what we snap pid_setpoint back to in case of changing states) at a much slower rate than setpoint_gain
//^^ over a long time this should average towards the actual true balanced point of the robot. Smaller values will make this take much longer, however if the init_angle is properly set it shouldn't be an issue
//^^ try values between 0.000001 and 0.000005

float pitch = 0;
float roll = 0;
float yaw = 0;
#pragma endregion PID config

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRV8835 MOTOR DRIVER CONFIG
#pragma region Motor Driver config
#define AENBL 5  //Assigns motor driver pins to the corresponding arduino pins
#define APHASE 7
#define BENBL 6   // "enable" pins are analog (0-255)for speed control
#define BPHASE 8  // "phase" pins are digital (0 or 1) for direction control
#define MODE A3   // "mode" pin can change the function of the DRV8835 and unlocks some cool features. Not used here, but check out the datasheet to learn more!

bool dir_right = 0;  //gets set 0 or 1 to tell the motor driver which way to spin
bool dir_left = 0;
int speed_left = 0;  // what actually sent to the motor for pid_output
int speed_right = 0;
int throttle_offset = 0;        // moves the "window" in which dynamic pid_setpoint kicks in to encourage driving
int TARGET_DRIVING_SPEED = 70;  // theoretically, the "right" speed to drive at. Increasing will make the driving more responsive, too much and it will likely outrun itself
float THROTTLE_ANGLE = 1.0;     // angle used to more quickly snap the pid_setpoint to drive in a different direction
float TURN_SPEED = 80;          // theoretically, the "right" amount of turning modulus to motor pid_output
float turn_value = 0;           // changing value to help control turn values
int turn_amount = 40;           // setting for how aggressively to turn when encountering an obstacle
bool isUpright = false;         // bool to track if the bot has fallen over
double current_abs_output = 0;  // stores the normalized output from PID (converted from -255-255 to 0-255 by taking the absolute value of output)
#pragma endregion Motor Driver config

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// MPU6500 CONFIG
#pragma region IMU config
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h> // although this library is for the MPU6050, that chip is out of production
#include <Wire.h> // instead we're using the MPU6500, which is a newer variant

MPU6050 mpu;
bool blinkState = false;
// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int IMUdataReady = 0;
volatile bool mpuInterrupt = false;
#pragma endregion IMU config
#pragma endregion LIBRARIES AND CONFIG

//////////////////////////////////////////////////
//  S E T U P //
//////////////////////////////////////////////////
#pragma region SETUP
void setup() {

  init_i2c();
  init_serial();
  init_imu();
  init_pins();

}
#pragma endregion SETUP

//////////////////////////////////////////////////
//  L O O P //
//////////////////////////////////////////////////
#pragma region LOOP
void loop() {

  if (IMUdataReady == 1) {           //reads data from MPU if the interrupt flag tells us there's data ready
    readAngles();                    //stores the yaw, pitch, and roll into the ypr[] array as radians
    pitch = -(ypr[1] * 180 / M_PI);  // convert radians to degrees
    roll = (ypr[2] * 180 / M_PI);
    yaw = (ypr[0] * 180 / M_PI);
  }

  if (is_upright()) { //if the bot is standing up
    if (useDynamicSetpoint){ // if we want the setpoint to change based on output
      dynamic_setpoint();
    }else{
      pid_setpoint = init_angle; //otherwise just set the setpoint equal to init
    }
    // PID control
    pid_input = pitch; // provide current pitch from readAngles as PID input
    pid.Compute(); // computes the PID equation

    // Use PID output to control motors
    activateMotors();
  } else { //if not standing up, turn motors off, and no need to do other calculations until it goes upright again
    analogWrite(AENBL, 0);
    analogWrite(BENBL, 0);
  }

  handle_sensors();

  last_state = bal_state;
  cmd_count++;
}
#pragma endregion LOOP

//////////////////////////////////////////////////
//  FUNCTIONS  //
//////////////////////////////////////////////////
#pragma region FUNCTIONS



/********** LOOP HELPERS **********/
#pragma region Loop Helpers
/**
 * @brief Helper to check if the bot is standing up
 * @return returns true when the current pitch is within our "upright" angle range
*/
bool is_upright() {
    return !(pitch > (init_angle + 40) || pitch < (init_angle - 40));
  }
  
  /**
   * @brief Handles updates to IR, Distance, LEDs, and Serial printing, with interpolation
   * 
  */
  void handle_sensors() {
  
    if ((cmd_count % 100) == 0) { // every 100 loops 
      obstacleAvoidance(); // check distance and avoid obstacles if necessary
    }
  
    if (millis() - lastUpdate > pattern_interval) {
      if (inRange){
        led_state = BLUSH;
      }else{
        led_state = RAINBOW;
      }
      updateLEDs();
    }
  
    if (cmd_count > 500) {  // a slower loop (once every 500 loops this runs) to let the MPU run faster without being delayed by other stuff
      if (useSerial) printSetpointAndPitch();
      cmd_count = 0;
    }
  }
#pragma endregion Loop helpers

/***** DRIVE FUNCTIONS *****/
#pragma region Driving
/**
 * @brief handles smoothing of values across changing balance states for forward / reverse movement, and implements a rudimentary feedback loop to reduce drift by tweaking the pid_setpoint angle
*/
void dynamic_setpoint() {
  if (abs(turn_value) > 0) {
    turn_value = turn_value * 0.99;
  }

  switch (bal_state) {
    case HOLD:
      throttle_offset = 0;
      if ((abs(reset_angle - init_angle) > 5)) reset_angle = init_angle;  //snap setpoint back to init in case we changed modes and there's a large difference
      break;

    case REV:
      throttle_offset = (throttle_offset * 0.8) + (TARGET_DRIVING_SPEED * 0.2);
      break;

    case FWD:
      throttle_offset = (throttle_offset * 0.8) + (-TARGET_DRIVING_SPEED * 0.2);
      break;
  }
  current_abs_output = abs(pid_output + throttle_offset);  // first we take the absolute value of the pid_output (normalizing it so instead of -255 to 255, it's just in terms of overall output rather than direction) + the driving offset, which is approximately the motor output speed we want to drive at

  if ((abs(pid_setpoint - reset_angle) > 5) || current_abs_output < 15 || current_abs_output > 250) pid_setpoint = reset_angle;

  if (current_abs_output > 20 && current_abs_output < 240) reset_angle += (pid_output + throttle_offset) * reset_gain;  //over time this will change the reset angle slightly so that the robot finds a happy and correct balance point in case it's slightly off

  if (current_abs_output < 220) pid_setpoint += (pid_output + throttle_offset) * setpoint_gain; // more reactive adjustment of pid_setpoint using a fraction of previous output
}

/**
 * @brief compensates for motor deadband, factors in turn value to output, and handles output of motor speeds to the motor driver with appropriate pin settings 
*/
void activateMotors() {
  if (pid_output < 15 && pid_output > -15) {  //if the pid_output is below a certain threshold (which wouldn't be strong enough to activate the motors anyway) just turn the motor off
    // creates a dead-band so that motors turn off when balanced to reduce jitter
    pid_output = 0;
  }
  // set motor speed with adjusted turn values
  speed_left = pid_output - turn_value;   //our motor speed should be the pid_output modulated by turn
  speed_right = pid_output + turn_value;  //since turn values go negative for right and positive for left, we subtract in one case and add in the other.

  //constrain pid_output within range
  speed_left = constrain(speed_left, -255, 255);  //ensure that the added turn value doesn't exceed output limits
  speed_right = constrain(speed_right, -255, 255);

  if (speed_right < 0) {  //converts the PID pid_output to 0-255 with a 0 or 1 to determine direction - just for how the DRV8835 motor driver uses pins.
    speed_right = abs(speed_right);
    dir_right = 0;
  } else {
    speed_right = abs(speed_right);
    dir_right = 1;
  }

  if (speed_left < 0) {
    speed_left = abs(speed_left);
    dir_left = 0;
  } else {
    speed_left = abs(speed_left);
    dir_left = 1;
  }
  digitalWrite(APHASE, dir_right);  //the phase pins are digital, meaning they accept 5v or 0v to determine motor direction
  digitalWrite(BPHASE, dir_left);
  analogWrite(AENBL, speed_right);  //the enable pins are analog, meaning they values between 0v and 5v to control motor speed
  analogWrite(BENBL, speed_left);
}
#pragma endregion Driving

/***** SERIAL PRINT HELPERS ****/
#pragma region Serial Print Helpers
/**
 * @brief prints or plots pid_setpoint, pitch angle, reset angle, and output to monitor the performance 
*/
void printSetpointAndPitch() {  
  Serial.print(F("pitch:"));
  Serial.print(pitch);
  Serial.print(F(","));
  Serial.print(F("pid_setpoint:"));
  Serial.print(pid_setpoint);
  Serial.print(F(","));
  Serial.print(F("rstAngle:"));
  Serial.print(reset_angle);
  Serial.print(F(","));
  Serial.print(F("Output:"));
  Serial.println((pid_output / 255) + reset_angle);
}

/**
 * @brief prints or plots PID values, pid_setpoint, pitch angle, reset angle, and output to monitor the performance 
*/
void printControlData() {
  Serial.print(F("P:"));
  Serial.print(kP / 10);
  Serial.print(F(","));
  Serial.print(F("I:"));
  Serial.print(kI / 200);
  Serial.print(F(","));
  Serial.print(F("D:"));
  Serial.println(kD * 2);
  Serial.print(F(","));
  Serial.print(F("MAX:"));
  Serial.print(init_angle + 3);
  Serial.print(F(","));
  Serial.print(F("MIN:"));
  Serial.print(init_angle - 3);
  Serial.print(F(","));
  Serial.print(F("pitch:"));
  Serial.print(pitch);
  Serial.print(F(","));
  Serial.print(F("pid_setpoint:"));
  Serial.print(pid_setpoint);
  Serial.print(F(","));
  Serial.print(F("rstAngle:"));
  Serial.print(reset_angle);
  Serial.print(F(","));
  Serial.print(F("Output:"));
  Serial.println((pid_output / 255) + reset_angle);
}
#pragma endregion Serial print helpers

/***** LED HELPERS ****/
#pragma region LEDs
/**
 * @brief updates LEDs based on current state setting
*/
void updateLEDs() {
  switch (led_state){
    case RAINBOW:
      rainbowCycle();
      break;

    case ROLL:
      rollLight();
      break;

    case BLUSH:
      blush();
      break;
  }
  lastUpdate = millis();  // time for next change to the display
}

/**
 * @brief sets the edge LEDs to pink to make the bot blush
*/
void blush() {
  wipe();
  strip.setPixelColor(0, strip.Color(255, 40, 40));
  strip.setPixelColor(7, strip.Color(255, 40, 40));
  strip.show();
}

/**
 * @brief updates LEDs based on roll and pitch values
*/
void rollLight() { //activate LEDs based on roll (side to side angle) and color based on pitch
  wipe();
  int pix = map(roll, -30, 30, 0, 7);
  int lean = map(pitch, -30, 30, 0, 255);
  int leanlite = lean / 4;
  strip.setPixelColor(pix, strip.Color(lean, 0, 255 - lean));
  strip.setPixelColor(pix - 1, strip.Color(leanlite, 0, 63 - leanlite));
  strip.setPixelColor(pix + 1, strip.Color(leanlite, 0, 63 - leanlite));
  strip.show();
}

/**
 * @brief clears LED bar
*/
void wipe() {  // clear all LEDs
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
  }
}

/**
 * @brief cycles rainbow colors across the LED bar
*/
void rainbowCycle() { 
  static uint16_t j = 0;
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
  }
  strip.show();
  j++;
  if (j >= 256 * 5) j = 0;
}

/**
 * @brief cycles colors around the color wheel
 * @return returns with RGB color used to set pixels in rainbowCycle
*/
uint32_t Wheel(byte WheelPos) {  //used to adjust values around the color wheel
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
#pragma endregion LEDs

/***** SENSOR HELPERS ****/
#pragma region Sensors
/**
 * @brief checks the distance sensor reading and toggles obstacle avoidance
*/
void obstacleAvoidance() {  //function to check distance with the ultrasonic sensor
  distance = sonar.ping_cm(); //read distance
  if (distance != 0 && distance < 20) { // check if the distance reading is within a certain range
    bal_state = REV; // set balance state to reverse
    turn_value = turn_amount; // set turn value to re-orient the bot
    inRange = true; // set inRange flag to true
  } else {
    bal_state = HOLD; //return to HOLD balance state
    inRange = false; // set inRange flag to false
  }
}
#pragma endregion Sensors

/***** IMU HELPERS ****/
#pragma region IMU
//IMU interrupt service routine
void dmpDataReady() {
  IMUdataReady = 1;
}

//
uint8_t GetCurrentFIFOPacket(uint8_t* data, uint8_t length, uint16_t max_loops) {
  mpu.resetFIFO();
  delay(1);
  //int countloop = 0;

  fifoCount = mpu.getFIFOCount();
  uint8_t GetPacketLoopCount = 0;

  while (fifoCount < packetSize && GetPacketLoopCount < max_loops) {
    GetPacketLoopCount++;
    fifoCount = mpu.getFIFOCount();
    delay(2);
  }

  if (GetPacketLoopCount >= max_loops) {
    return 0;
  }

  //if we get to here, there should be exactly one packet in the FIFO
  mpu.getFIFOBytes(data, packetSize);
  return 1;
}

//function that actually read the angle when the flag is set by the ISR
void readAngles() {

  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }

  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.resetFIFO();

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    IMUdataReady = 0;
  }
}
#pragma endregion IMU

/***** SETUP HELPERS ****/
#pragma region Setup Helpers
/**
 * @brief initializes the I2C communications which the IMU will use
*/
void init_i2c() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  delay(3);  // ms
}

/**
 * @brief initializes serial communications (and LEDs so we know it's started calibrating)
*/
void init_serial() {

  Serial.begin(38400);
  Serial.println(F(" STARTING UP "));

  strip.begin();            // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();             // Turn OFF all pixels ASAP
  strip.setBrightness(60);  // Set BRIGHTNESS to about 1/5 (max = 255)

  updateLEDs();
}

/**
 * @brief initializes pins and PID
*/
void init_pins() {
  digitalWrite(MODE, HIGH); //set the motor driver MODE pin to HIGH, the DRV8835 chip has an alternate mode which require a different pin configuration, check out the datasheet for more info

  pid.SetMode(AUTOMATIC); 
  pid.SetOutputLimits(-255, 255); //constrain PID to -255 to 255, motor speed ranges in both directions
  pid.SetSampleTime(5); 

  pinMode(AENBL, OUTPUT); // configure motor driver pins for output
  pinMode(BENBL, OUTPUT);
  pinMode(APHASE, OUTPUT);
  pinMode(BPHASE, OUTPUT);
}

/**
 * @brief initializes the IMU and handles calibration
*/
void init_imu() {
  delay(150);  // 150 millisecond delay helps the IMU initialize properly
  // initialize device
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  if (useCalibration){
    mpu.CalibrateAccel(10); // auto-calibration to supply offsets for the IMU, commenting these out will have the robot skip calibration and use factory default offsets (more prone to error but works ok).
    mpu.CalibrateGyro(10); // you can also skip calibration by properly calibrating your IMU with the IMUZero hack - this supplies offset numbers which will be most accurate to your specific IMU
  }
    // IF you want to permanently set offsets, use the IMUzero sketch to properly calibrate your MPU, then paste the offsets below

  // make sure init worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);  //enables "digital motion processing", this is the MPU calculating the angles onboard, rather than doing it here.

    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(3), dmpDataReady, RISING); //rising pin change senses when digital pin 3 goes to HIGH (5V) and triggers the interrupt
    mpuIntStatus = mpu.getIntStatus();

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}
#pragma endregion Setup Helpers
#pragma endregion FUNCTIONS
//////////////////////////////////////////////////
//  END CODE  //
//////////////////////////////////////////////////