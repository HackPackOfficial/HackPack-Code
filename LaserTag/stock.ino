#pragma region LICENSE
/*
  ************************************************************************************
  * MIT License
  *
  * Copyright (c) 2025 Crunchlabs LLC (Laser Tag: Stock Code)

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

#pragma region PIN_DEFINITIONS
// >>>>>>>>>>>>>>>>>>>>>>>>>>>> PIN DEFINITIONS <<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define IR_SEND_PIN         3
#define IR_RECEIVE_PIN      5 
#define _IR_TIMING_TEST_PIN 7

#define SERVO_PIN       9
#define TRIGGER_PIN     12

#define TEAM1_PIN       15      // A1 pin 
#define TEAM2_PIN       16      // A2 pin
#define TEAM3_PIN       17      // A3 pin

#pragma endregion PIN_DEFINITIONS

//////////////////////////////////////////////////
//  LIBRARIES  //
//////////////////////////////////////////////////
#pragma region LIBRARIES
#define DECODE_NEC          // defines IR Protocol (Apple and Onkyo)
 
#include <Arduino.h>
#include <Servo.h>
#include <IRremote.hpp>

Servo myservo;

#pragma endregion LIBRARIES

#pragma region GAME_PARAMETERS
// >>>>>>>>>>>>>>>>>>>>>>>>>>> GAME PARAMETERS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define DEBOUNCE_DELAY 20

#define SERVO_INITIAL_POS 150     // how agressively to undarken goggles 
#define SERVO_READY_POS 120       // reduce aggresiveness near end of action
#define SERVO_HIT_POS 50

#define TRIGGER_COOLDOWN 500      // milliseconds  
#define HIT_TIMEOUT 10000         // milliseconds      

int team = 1;     // default 

// >>>>>>>>>>>>>>>>>>>>>>>>>>> GAME VARIABLES <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

int lastTriggerVal = 1;                     // trigger debounce variable
unsigned long lastTDebounceTime = 0;        // trigger button debounce time
int triggerState;                           // trigger debounce result
bool buttonWasReleased = true;              // release check, no "full auto"
unsigned long previousTriggerMillis = 0;    // cooldown timestamp between shots

// Initialize game timeout variable
unsigned long timeoutStartTime = - HIT_TIMEOUT - 1000;

// IR pulse, tracks team distinction
uint8_t sCommand;                            // IR command being sent
uint8_t rcvCommand1;                         // IR command being recieved
uint8_t rcvCommand2;                         // IR command being recieved
#pragma endregion GAME_PARAMETERS

//////////////////////////////////////////////////
//  S E T U P //
//////////////////////////////////////////////////
#pragma region SETUP
void setup() {

  // Move Goggles to start config
  myservo.attach(SERVO_PIN);
  myservo.write(SERVO_INITIAL_POS);
  delay(500);
  myservo.write(SERVO_READY_POS);
  delay(500);
  myservo.detach();

  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TEAM1_PIN, INPUT_PULLUP);
  pinMode(TEAM2_PIN, INPUT_PULLUP);
  pinMode(TEAM3_PIN, INPUT_PULLUP);

  if (digitalRead(TEAM1_PIN) == LOW) {
    team = 1;
  } else if (digitalRead(TEAM2_PIN) == LOW) {
    team = 2;
  } else if (digitalRead(TEAM3_PIN) == LOW) {
    team = 3;
  }

  if (team == 1) {
    sCommand = 0x34;
    rcvCommand1 = 0x35;
    rcvCommand2 = 0x36;
  } else if (team == 2) {
    sCommand = 0x35;
    rcvCommand1 = 0x34;
    rcvCommand2 = 0x36;
  } else {
    sCommand = 0x36;
    rcvCommand1 = 0x34;
    rcvCommand2 = 0x35;
  }

  Serial.begin(115200);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
}
#pragma endregion SETUP

//////////////////////////////////////////////////
//  L O O P //
//////////////////////////////////////////////////
#pragma region LOOP
void loop() {
  unsigned long currentMillis = millis();

  handleTrigger(currentMillis);
  handleIRReception();
}
#pragma endregion LOOP

//////////////////////////////////////////////////
//  GAMEPLAY FUNCTIONS  //
//////////////////////////////////////////////////
#pragma region GAMEPLAY_FUNCTIONS

// Read Trigger ----------
void handleTrigger(unsigned long currentMillis) {
  if (ReadTriggerButton() && buttonWasReleased && currentMillis - previousTriggerMillis >= TRIGGER_COOLDOWN) {
    previousTriggerMillis = currentMillis;
    buttonWasReleased = false;

    sendIR_Pulse();
  } else if (!ReadTriggerButton()) {
    buttonWasReleased = true;
  } else {
  }
}

// Fire "Shot" ----------
void sendIR_Pulse() {
  Serial.flush();
  IrSender.sendNEC(0x00, sCommand, 3);
  delay(10);
}

// Read incoming message ----------
void handleIRReception() {
  if (IrReceiver.decode()) {
    checkPlayerHit();
    IrReceiver.resume(); // Ensure IR receiver is reset
  }
}

// Check if message is a "shot" from an enemy team ----------
void checkPlayerHit() {
  if (IrReceiver.decodedIRData.command == rcvCommand1 || IrReceiver.decodedIRData.command == rcvCommand2) {
    if (millis() - timeoutStartTime > HIT_TIMEOUT + 1000) {
      markHit();
    }
  }
}

// Move goggles if hit ----------
void markHit() {
  // get current time
  timeoutStartTime = millis();

  // move goggles to darken
  myservo.attach(SERVO_PIN);
  myservo.write(SERVO_HIT_POS);

  while (millis() - timeoutStartTime < HIT_TIMEOUT) {

    // In last 20% of timeout, begin to move servo towards starting position
    int timeVal = (millis() - timeoutStartTime) / 100;
    if (millis() > timeoutStartTime + (HIT_TIMEOUT * (4.0 / 5.0))) {
      myservo.write(SERVO_INITIAL_POS);
    }
  }
  myservo.detach();
}
#pragma endregion GAMEPLAY_FUNCTIONS

#pragma region BUTTON_DEBOUNCING
bool ReadTriggerButton() {
  int triggerVal = digitalRead(TRIGGER_PIN);
  if (triggerVal != lastTriggerVal) {
    lastTDebounceTime = millis();
  }
  if ((millis() - lastTDebounceTime) > DEBOUNCE_DELAY) {
    if (triggerVal != triggerState) {
      triggerState = triggerVal;
    }
  }
  lastTriggerVal = triggerVal;
  return triggerState == LOW;
}
#pragma endregion BUTTON_DEBOUNCING

