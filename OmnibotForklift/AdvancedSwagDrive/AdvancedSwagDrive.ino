#include "config.h"

#define RF_S1_V 19        // rf receiver signal 1 voltage
#define RF_S1_G 18        // rf receiver signal 1 ground
#define RF_S2_V 17
#define RF_S2_G 16
#define RF_S3_V 15
#define RF_S3_G 14
#define RF_S4_V 12
#define RF_S4_G 11

#define M1_DIR 4
#define M1_PWM 5
#define M2_DIR 7
#define M2_PWM 6
#define M3_DIR 8
#define M3_PWM 9

#define LIFT_DIR 2
#define LIFT_PWM 3

//>>>>>>>>>>>>>>>>>>>>>>>>>> KEY ROBOT VARIABLES <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//RF Commands ----------------------------------------------------
bool enableSwag = false;
bool alreadySwaggin = false;
long swagTimer;
int swagActive;

int swagSpeeds[3];


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> SETUP <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void setup() {
  Serial.begin(115200);
  Serial.println("OMNIB_Hack_SwagDrive V0.1.0 ..... (03/11/2025)");
  pinMode(RF_S1_V, INPUT_PULLUP);
  pinMode(RF_S1_G, INPUT_PULLUP);
  pinMode(RF_S2_V, INPUT_PULLUP);
  pinMode(RF_S2_G, INPUT_PULLUP);
  pinMode(RF_S3_V, INPUT_PULLUP);
  pinMode(RF_S3_G, INPUT_PULLUP);
  pinMode(RF_S4_V, INPUT_PULLUP);
  pinMode(RF_S4_G, INPUT_PULLUP);
}



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> LOOP <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void loop() {
  getSwag();
  swagTime();
}



// >>>>>>>>>>>>>>>>>>>>> RADIO COMMAND PROCESSING <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// Read reciever input pins to decide what robot is going to do
void getSwag(){
  // inputs stores meaured RF controller states, inputArr stores processed data 
  int inputArr[8];
  int inputs[8] = {1, 1, 1, 1, 1, 1, 1, 1};

  // create time variable
  unsigned long t = millis();

  // sample for 5ms fo any pins going LOW. Deals with RF signal dropoouts.
  while(millis() < (t + 1)){
    if(digitalRead(RF_S1_G) == 0){
      inputs[0] = 0;
    }
    if(digitalRead(RF_S1_V) == 0){
      inputs[1] = 0;
    }
    if(digitalRead(RF_S2_G) == 0){
      inputs[2] = 0;
    }
    if(digitalRead(RF_S2_V) == 0){
      inputs[3] = 0;
    }
    if(digitalRead(RF_S3_G) == 0){
      inputs[4] = 0;
    }
    if(digitalRead(RF_S3_V) == 0){
      inputs[5] = 0;
    }
    if(digitalRead(RF_S4_G) == 0){
      inputs[6] = 0;
    }
    if(digitalRead(RF_S4_V) == 0){
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

   
  for(int i = 0; i < 8; i++){
    Serial.print(inputArr[i]);
    Serial.print(", ");
  }
  Serial.println();

  for(int i = 0; i < 8; i++){
    if(inputArr[i] == 0){
      if(!alreadySwaggin){
        enableSwag = true;
        swagTimer = millis() + random(20, 800);
        break;
      }
    }
  }

  if(millis() < swagTimer){
    if(!alreadySwaggin){
      alreadySwaggin = true;
      swagSpeeds[0] = random(75, 255);
      swagSpeeds[1] = random(75, 255);
      swagSpeeds[2] = random(75, 255);

      if(random(2) == 0){
        swagSpeeds[0] = -swagSpeeds[0];
      } 
      if(random(2) == 0){
        swagSpeeds[1] = -swagSpeeds[1];
      }
       if(random(2) == 0){
        swagSpeeds[2] = -swagSpeeds[2];
      }
    } 
  } else{
    swagSpeeds[0] = 0;
    swagSpeeds[1] = 0;
    swagSpeeds[2] = 0;
    alreadySwaggin = false;
    enableSwag = false;
  }
}

// >>>>>>>>>>>>>>>>>>>>>>>> ROBOT ACTION FUNCTIONS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// prepare motors to coorinate bot movements
void swagTime(){
  if(swagSpeeds[0] < 0){
    digitalWrite(M1_DIR, LOW);
  } else {
    digitalWrite(M1_DIR, HIGH);
  }
  analogWrite(M1_PWM, int(abs(swagSpeeds[0])));

  if(swagSpeeds[1] < 0){
    digitalWrite(M2_DIR, LOW);
  } else {
    digitalWrite(M2_DIR, HIGH);
  }
  analogWrite(M2_PWM, int(abs(swagSpeeds[1])));

  if(swagSpeeds[2] < 0){
    digitalWrite(M3_DIR, LOW);
  } else {
    digitalWrite(M3_DIR, HIGH);
  }
  analogWrite(M3_PWM, int(abs(swagSpeeds[2])));
}