/*
Huge thanks to those who have derived the kinematic conrrols for omni-directional robots. This hackpack comes with an optional educational reading that I highly reccomend: (Siradjuddin, Indrazno. "Kinematics and control a three wheeled omnidirectional mobile robot." Int. J. Electr. Electron. Eng 6.12 (2019): 1-6.) [https://www.academia.edu/download/90057161/IJEEE-V6I12P101.pdf] 

Amazingly the relationship of the wheel speeds of the robot to its overall speed vector is algebraically linear.  The formula also works for any relative wheel angle and can easily be expanded to include more wheels. This gives a mathematically deterministic way to get the robot from A to B defined by two translation variables and one rotation variable, a powerful tool. 

Combining this with a gyroscope allows for a true field oriented drive-- where  joystick commands always will move the robot forwards relative to you, despite which way the robot is facing. For those of you willing to take on the challenge, this is a great hack to try!
*/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define S1_V 19
#define S1_G 18
#define S2_V 17
#define S2_G 16
#define S3_V 15
#define S3_G 14
#define S4_V 12
#define S4_G 11

#define M1_DIR 4
#define M1_PWM 5
#define M2_DIR 7
#define M2_PWM 6
#define M3_DIR 8
#define M3_PWM 9

#define LIFT_DIR 2
#define LIFT_PWM 3

Adafruit_MPU6050 mpu;

//>>>>>>>>>>>>>>>>>>>>>>>>>> KEY ROBOT VARIABLES <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//RF Commands ----------------------------------------------------
// remote command: drive vector (in velocities) {v_x, v_y, omega}
int vSpeed[3];
// remote command: for forklift, -1 for lower, 0 for stay, 1 for move up
int moveFork = 0;
// stores the result of reciever message. 14 is default- dont move the motors.
int lastCommand[4] = {14, 14, 14, 14};
// robot will strafe when left/right is pressed and forward/back is held.
bool strafeMode = false;


//Robot Physical Parameters -------------------------------------
double angleWheel1 = 90.0;
double angleWheel2 = 210.0;
double angleWheel3 = 330.0;

// Equation puts wheel 1 facing forwards. This rotates the robot's "front" ccw.
double localAngle = 60.0;
// Gyro measurement result
double gyroAngle = 0.0;
// Radius of wheel center to robot center
double botRadius = 100.0;
// Radius of omniwheel
double wheelRadius = 35.0;

// Maximum motor duty cycle [0 - 255]
uint16_t maxSpeed = 255;

// Last MPU measuement time
unsigned long lastGyroTime;
// Drift calibration variable for gyro, found in setup
float gyroCal;
// Keeps track of cumulative movements 
float radZ;

// stores each calculated wheel speed for a given commanded vector {w1, w2, w3}.
int wheelSpeeds[3];



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> SETUP <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void setup() {
  Serial.begin(115200);
  Serial.println("OMNIB_GYRO_HACK V0.9 ..... (01/16/2025)");
  Wire.begin();


  pinMode(17, INPUT_PULLUP);
  pinMode(16, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);

  // Wait for MPU comms
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  // wait a bit before calibration
  delay(100);

  // take average of 500 measurements to estimate drift. Do NOT touch robot during calibration !!!
  for(int i = 0; i < 500; i++){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroCal += g.gyro.z;
  }
  gyroCal = gyroCal / 500.0;
}



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> LOOP <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void loop() {
  getDirection();
  getGyroValues();
  moveBot();
}



// >>>>>>>>>>>>>>>>>>>>> RADIO COMMAND PROCESSING <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// Read reciever input pins to decide what robot is going to do
void getDirection(){
  int command;

  // inputs stores meaured RF controller states, inputArr stores processed data 
  int inputArr[8];
  int inputs[8] = {1, 1, 1, 1, 1, 1, 1, 1};

  // create time variable
  unsigned long t = millis();

  // sample for 10ms fo any pins going LOW. Deals with RF signal dropoouts.
  while(millis() < (t + 15)){
    if(digitalRead(12) == 0){
      inputs[0] = 0;
    }
    if(digitalRead(13) == 0){
      inputs[1] = 0;
    }
    if(digitalRead(16) == 0){
      inputs[2] = 0;
    }
    if(digitalRead(17) == 0){
      inputs[3] = 0;
    }
    if(digitalRead(14) == 0){
      inputs[4] = 0;
    }
    if(digitalRead(15) == 0){
      inputs[5] = 0;
    }
    if(digitalRead(10) == 0){
      inputs[6] = 0;
    }
    if(digitalRead(11) == 0){
      inputs[7] = 0;
    }
  }

  inputArr[0] = inputs[0];
  inputArr[1] = inputs[1];
  inputArr[2] = inputs[2];
  inputArr[3] = inputs[3];
  inputArr[4] = inputs[4];
  inputArr[5] = inputs[5];
  inputArr[6] = inputs[6];
  inputArr[7] = inputs[7];

 
  /*for(int i = 0; i < 8; i++){
    Serial.print(inputArr[i]);
    Serial.print(", ");
  }
  Serial.println();*/
  
  
  // These sequence of inputs are read when button/ button combos are pressed
  int F[8] = {1, 1, 0, 1, 1, 0, 1, 1};      // 1. forward
  int B[8] = {1, 1, 1, 0, 0, 1, 1, 1};      // 2. back
  int L[8] = {0, 1, 0, 1, 0, 1, 1, 1};      // 3. right
  int R[8] = {1, 0, 1, 0, 1, 0, 1, 1};      // 4. left
  int U[8] = {1, 1, 1, 1, 1, 1, 1, 0};      // 5. fork up
  int D[8] = {1, 1, 1, 1, 1, 1, 0, 1};      // 6. fork down
  int FL[8] = {0, 1, 0, 1, 1, 0, 1, 1};     // 7. forward + left
  int FR[8] = {1, 0, 0, 1, 1, 0, 1, 1};     // 8. forward + right
  int BL[8] = {0, 1, 1, 0, 0, 1, 1, 1};     // 9. back + left
  int BR[8] = {1, 0, 1, 0, 0, 1, 1, 1};     // 10. back + right
  int FU[8] = {1, 1, 0, 1, 1, 0, 1, 0};     // 11. forward + fork up
  int FD[8] = {1, 1, 0, 1, 1, 0, 0, 1};     // 12. forward + fork down
  int BU[8] = {1, 1, 1, 0, 0, 1, 1, 0};     // 13. back + fork up
  int BD[8] = {1, 1, 1, 0, 0, 1, 0, 1};     // 14. back + fork down
  int NONE[8] = {1, 1, 1, 1, 1, 1, 1, 1};   // 15. default, do nothing.



  //create array that enumerates every command 
  int cmdArr[15] = {0};

  // We loop through all pin values and compares number of matches for each command. One of the commands will match all 8 pin outputs and that is what will get chosen as the transmitter's result. 

  // For example pressing the backwards button will generate a cmdArr =  {F = 0, B = 8, L = 4, R = 4, U = 4, D = 4, FL = 2, FR = 2, BL = 6, BR = 6, FU = 2, FD = 2, BU = 6, BD = 6, N = 4}. Thus backwards with a "match value" of 8 will be chosen.
  for(int i = 0; i < 8; i++){
    if(inputArr[i] == F[i]) cmdArr[0] += 1;
    if(inputArr[i] == B[i]) cmdArr[1] += 1;
    if(inputArr[i] == L[i]) cmdArr[2] += 1;
    if(inputArr[i] == R[i]) cmdArr[3] += 1;
    if(inputArr[i] == U[i]) cmdArr[4] += 1;
    if(inputArr[i] == D[i]) cmdArr[5] += 1;
    if(inputArr[i] == FL[i]) cmdArr[6] += 1;
    if(inputArr[i] == FR[i]) cmdArr[7] += 1;
    if(inputArr[i] == BL[i]) cmdArr[8] += 1;
    if(inputArr[i] == BR[i]) cmdArr[9] += 1;
    if(inputArr[i] == FU[i]) cmdArr[10] += 1;
    if(inputArr[i] == FD[i]) cmdArr[11] += 1;
    if(inputArr[i] == BU[i]) cmdArr[12] += 1;
    if(inputArr[i] == BD[i]) cmdArr[13] += 1;
    if(inputArr[i] == NONE[i]) cmdArr[14] += 1;
  }

  for(int i = 0; i < 15; i++){
    if(cmdArr[i] == 8){
      command = i;
      break;
    }
  }
  
  // Assign speed vector vlaues based on chosen command
  switch(command){
    case 0: //Forward
      vSpeed[0] = 0;
      vSpeed[1] = -1;
      vSpeed[2] = 0;
      moveFork = 0;
      break;
    case 1: //Back
      vSpeed[0] = 0;
      vSpeed[1] = 1;
      vSpeed[2] = 0;
      moveFork = 0;
      break;
    case 2: //Rotate Left (CCW)
      vSpeed[0] = 0;
      vSpeed[1] = 0;
      vSpeed[2] = -1;
      moveFork = 0;
      break;
    case 3: //Rotate Right (CW)
      vSpeed[0] = 0;
      vSpeed[1] = 0;
      vSpeed[2] = 1;
      moveFork = 0;
      break;
    case 4: //Fork Up
      vSpeed[0] = 0;
      vSpeed[1] = 0;
      vSpeed[2] = 0;
      moveFork = 1;
      break;
    case 5: //Fork Down
      vSpeed[0] = 0;
      vSpeed[1] = 0;
      vSpeed[2] = 0;
      moveFork = -1;
      break;
    case 6: //Forward + Left
      vSpeed[0] = 1;
      vSpeed[1] = -1;
      vSpeed[2] = 0;
      moveFork = 0;
      break;
    case 7: //Forward + Right
      vSpeed[0] = -1;
      vSpeed[1] = -1;
      vSpeed[2] = 0;
      moveFork = 0;
      break;
    case 8: //Back + Left
      vSpeed[0] = 1;
      vSpeed[1] = 1;
      vSpeed[2] = 0;
      moveFork = 0;
      break;
    case 9: //Back + Right
      vSpeed[0] = -1;
      vSpeed[1] = 1;
      vSpeed[2] = 0;
      moveFork = 0;
      break;
    case 10: //Forward + Fork Up
      vSpeed[0] = 0;
      vSpeed[1] = -1;
      vSpeed[2] = 0;
      moveFork = 1;
      break;
    case 11:  //Forward + Fork Down
      vSpeed[0] = 0;
      vSpeed[1] = -1;
      vSpeed[2] = 0;
      moveFork = -1;
      break;
    case 12: //Back + Fork Up
      vSpeed[0] = 0;
      vSpeed[1] = 1;
      vSpeed[2] = 0;
      moveFork = 1;
      break;
    case 13: //Back + Fork Down
      vSpeed[0] = 0;
      vSpeed[1] = 1;
      vSpeed[2] = 0;
      moveFork = -1;
      break;
    case 14: //None
      vSpeed[0] = 0;
      vSpeed[1] = 0;
      vSpeed[2] = 0;
      moveFork = 0;
      break;
    default:
      break;
  }

  //Serial.println(command);

  // Add strafing / slide drive logic. Robot will strafe sideways (crabwalk) if left or right is pressed while the forward/back button is being held down. Users can the let go of forward/back after strafing has started and the slide drive behavior will persist.

  // first lets check that our command history contains initalizing strafe.
  bool initStrafe = false;

  for(int i = 0; i < 4; i++){
    // if command is 6, 7, 8, or 9
    if(lastCommand[i] > 5 && lastCommand[i] < 10){
      initStrafe = true;
      break;
    }
  }

  // If strafe is initalized, user can let go of forward/back
  if(initStrafe == true || strafeMode == true){
    if(command == 2){ // Left
      strafeMode = true;
      vSpeed[0] = -1;
      vSpeed[1] = 0;
      vSpeed[2] = 0;
      moveFork = 0;
    } else if (command == 3){ // Right
      strafeMode = true;
      vSpeed[0] = 1;
      vSpeed[1] = 0;
      vSpeed[2] = 0;
      moveFork = 0;
    } else {  // Other commands not related to strafe
      strafeMode = false;
    }
  }
  
  // if strafemode is active, the left and right button will slide the robot instead of rotating it.
  if(strafeMode){
    if(command == 2){ // Left
      vSpeed[0] = 1;
      vSpeed[1] = 0;
      vSpeed[2] = 0;
      moveFork = 0;
    } else if (command == 3){ // Right
      vSpeed[0] = -1;
      vSpeed[1] = 0;
      vSpeed[2] = 0;
      moveFork = 0;
    }
  }
  
  /*Serial.print("X: ");
  Serial.print(vSpeed[0]);
  Serial.print(", Y: ");
  Serial.print(vSpeed[1]);
  Serial.print(", Omega: ");
  Serial.print(vSpeed[2]);
  Serial.print(". Fork Movement: ");
  Serial.println(moveFork);*/
  
  // shift the command history array down an element... 
  lastCommand[3] = lastCommand[2];
  lastCommand[2] = lastCommand[1];
  lastCommand[1] = lastCommand[0];
  // and save this excecuted command for next iteration
  lastCommand[0] = command;
}

// >>>>>>>>>>>>>>>>>>>>>>>> GYRO ANGLE MEASUREMENT <<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void getGyroValues(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float z = g.gyro.z - gyroCal;

  radZ += z * (micros() - lastGyroTime) / 1000000.0;
  lastGyroTime = micros();
  gyroAngle = -radZ * 57.2958;

  while(gyroAngle > 360.0){
    gyroAngle -= 360;
  }
  while(gyroAngle < 0.0){
    gyroAngle += 360;
  }

  // gyro seems to flip robot behavior when passin specific values. Not sure why but this fixes it at the moment.
  
  if(gyroAngle >= 150.0 && gyroAngle <= 330.0){
    vSpeed[0] = - vSpeed[0];
    vSpeed[1] = - vSpeed[1];
  }
  
}

// >>>>>>>>>>>>>>>>>>>>>>>> ROBOT ACTION FUNCTIONS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// prepare motors to coorinate bot movements
void moveBot(){
  float wheelSpeeds[3];
  getWheelSpeeds(wheelSpeeds);
  driveWheels(wheelSpeeds, maxSpeed);
  driveLift();
}

// Addresses motor controllers to drive wheels at caclulated speeds
void driveWheels(float* speeds, float maxSpeed){  
  //drive all motors at calculated speeds
  float topWheelSpeed = max(max(abs(speeds[0]), abs(speeds[1])), abs(speeds[2]));
  wheelSpeeds[0] = speeds[0] / topWheelSpeed * maxSpeed;
  wheelSpeeds[1] = speeds[1] / topWheelSpeed * maxSpeed; 
  wheelSpeeds[2] = speeds[2] / topWheelSpeed * maxSpeed; 

  if(wheelSpeeds[0] < 0){
    digitalWrite(M1_DIR, LOW);
  } else {
    digitalWrite(M1_DIR, HIGH);
  }
  analogWrite(M1_PWM, int(abs(wheelSpeeds[0])));

  if(wheelSpeeds[1] < 0){
    digitalWrite(M2_DIR, LOW);
  } else {
    digitalWrite(M2_DIR, HIGH);
  }
  analogWrite(M2_PWM, int(abs(wheelSpeeds[1])));

  if(wheelSpeeds[2] < 0){
    digitalWrite(M3_DIR, LOW);
  } else {
    digitalWrite(M3_DIR, HIGH);
  }
  analogWrite(M3_PWM, int(abs(wheelSpeeds[2])));
}

// Moves lift motor. full forward, full back, or stopped.
void driveLift(){
  if(moveFork > 0){
    digitalWrite(LIFT_DIR, LOW);
    analogWrite(LIFT_PWM, 255);
  } else if(moveFork < 0){
    digitalWrite(LIFT_DIR, HIGH);
    analogWrite(LIFT_PWM, 255);
  } else{
    digitalWrite(LIFT_DIR, HIGH);
    analogWrite(LIFT_PWM, 0);
  }
}



// >>>>>>>>>>>>>>>>>>>>>>>>>> INVERSE KINEMATICS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// calculates speed of each wheel based of commanded speed vector. Forumlas are in the reading!
void getWheelSpeeds(float* wheelList) {
  double botAngle = localAngle - gyroAngle;

  // 0.0174533 converts deg to rad 
  wheelList[0] = (-sin((botAngle + angleWheel1) * 0.0174533) 
                * cos(botAngle * 0.0174533) * vSpeed[0] + cos((botAngle + angleWheel1) * 0.0174533) * cos(botAngle * 0.0174533) * vSpeed[1] + botRadius * vSpeed[2]) / wheelRadius;
  
  wheelList[1] = (-sin((botAngle + angleWheel2) * 0.0174533) 
                * cos(botAngle * 0.0174533) * vSpeed[0] + cos((botAngle + angleWheel2) * 0.0174533) * cos(botAngle * 0.0174533) * vSpeed[1] + botRadius * vSpeed[2])/ wheelRadius;
  wheelList[2] = (-sin((botAngle + angleWheel3) * 0.0174533) 
                * cos(botAngle * 0.0174533) * vSpeed[0] + cos((botAngle + angleWheel3) * 0.0174533) * cos(botAngle * 0.0174533) * vSpeed[1] + botRadius * vSpeed[2])/ wheelRadius;
}