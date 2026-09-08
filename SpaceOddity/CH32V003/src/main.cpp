// main.cpp — CH32V003 sender (simplified UART port)
// Replaces the BUTTON_REPEATER shared-bus scheme with UART.
//
// Pin assignments:
//   PD0  — power button (active low, keep as PD0)
//   PC5  — high-side switch output (keep as PC5)
//   PC1  — front panel button 0 (was BUTTON_REPEATER in old design; now repurposed)
//   PC3  — front panel button 1
//   PD5  — UART1 TX (Serial)
//   PD6  — UART1 RX (Serial)
//
// NOTE: PD6 was the startup-flash LED in the previous version. Serial.begin() is
// called only after the power button is pressed inside loop(), so the LED flash
// in setup() completes before PD6 is reconfigured as UART RX. No conflict.

#include <Arduino.h>
#include <ch32v00x.h>
#include <ch32v00x_pwr.h>

// use this to set the appropriate power enable active state. for the actual hardware,
// uncomment the next line.

#define ACTIVE_LOW
#ifdef  ACTIVE_LOW
  #define POWER_ON  LOW
  #define POWER_OFF HIGH
#else
  #define POWER_ON  HIGH
  #define POWER_OFF LOW
#endif  // ACTIVE_LOW

// ── Pin assignments ───────────────────────────────────────────────────────────
constexpr uint8_t POWER_BUTTON_PIN = PD0;
constexpr uint8_t POWER_ENABLE_PIN = PC5;
constexpr uint8_t BUTTON_0_PIN     = PD4;
constexpr uint8_t BUTTON_1_PIN     = PD3;
constexpr uint8_t BUTTON_2_PIN     = PD2;
constexpr uint8_t LED_PIN          = PC1;
constexpr uint8_t PROGRAM_MODE_PIN = PC3;   // Pull to ground to put ESP32 into program downloading mode


// ── Protocol ─────────────────────────────────────────────────────────────────
constexpr uint8_t MSG_INIT = 0xAE,                // ESP32 -> CH32: command to begin or restart UART communications
                  MSG_READY = 0xAB,               // CH32 -> ESP32: response to ESP32's MSG_INIT command to acknowledge communications started
                  MSG_CLOSE = 0xB7,               // ESP32 -> CH32: command to close the UART connection to make the pins availabe for something else
                  MSG_SET_HARD_OFF_TIME = 0xC3,   // ESP32 -> CH32: set the time in milliseconds that the power button needs to be held to trigger hard shutdown
                  MSG_SHUTDOWN = 0x9D,            // ESP32 -> CH32: command to make the CH32 cut the power to the ESP32
                  MSG_RESET = 0x9A;               // ESP32 -> CH32: cut power to ESP32 and then turn power back on

// ── Timing ───────────────────────────────────────────────────────────────────
constexpr uint16_t POWER_OFF_HOLD_TIME = 5000;  // hard power-off threshold (ms)
uint16_t hardOffTime = POWER_OFF_HOLD_TIME;     // settable
constexpr uint16_t SLEEP_TIMEOUT = 60000;


// ── State ─────────────────────────────────────────────────────────────────────
volatile bool buttonFell     = false;
bool   outputIsOn     = false;
uint32_t pressStartTime = 0;
uint8_t lastState = 0, currentState = 0;


void enterStandby();  // forward declaration required by WCH toolchain
void buttonISR();
uint8_t buildState();
void waitForHandshake();
void watchForHardPowerOffReq();
void handleMessages();

bool systemWasInStandby = false;
bool initialized        = false;
bool wokeFromStandby    = false;
bool sleepTimerRequired = true;
uint32_t sleepTimer = 0;

// ── ISR ───────────────────────────────────────────────────────────────────────
void buttonISR() {
  buttonFell = true;
  if (pressStartTime == 0) pressStartTime = millis();
}

// ── Helpers ───────────────────────────────────────────────────────────────────
uint8_t buildState() {
  uint8_t state = 0;
  if (digitalRead(POWER_BUTTON_PIN) == LOW) state |= (0x01 << 0);
  if (digitalRead(BUTTON_0_PIN)     == LOW) state |= (0x01 << 1);
  if (digitalRead(BUTTON_1_PIN)     == LOW) state |= (0x01 << 2);
  if (digitalRead(BUTTON_2_PIN)     == LOW) state |= (0x01 << 3);
  return state;
}

// Block until ESP32 sends MSG_INIT, then respond with READY.
void waitForHandshake() {
  static uint32_t toggler = 0;
  static uint8_t led = 0;
  while (true) {
    // this is a blocking loop, so continue monitoring for hard power off requests
    watchForHardPowerOffReq();
    if (!outputIsOn) return;  // the power was turned off but I was stuck in this while loop
    if (millis() - toggler >= 100)
    {
      digitalWrite(LED_PIN, led);
      led ^= 1;
      toggler = millis();
    }
    if (Serial.available() && (uint8_t)Serial.read() == MSG_INIT) {
      Serial.write(MSG_READY);
      return;
    }
  }
}

// Monitor the power button to see if a hard power off is requested.
void watchForHardPowerOffReq()
{
  static uint32_t pwrBtnPressTime  = 0;
  static bool     buttonPressed    = false;
  static bool     lastPwrBtnState  = HIGH;

  bool currentPwrBtnState = digitalRead(POWER_BUTTON_PIN);
  bool fell = (lastPwrBtnState == HIGH && currentPwrBtnState == LOW);
  bool rose = (lastPwrBtnState == LOW  && currentPwrBtnState == HIGH);
  lastPwrBtnState = currentPwrBtnState;

  if (fell) buttonPressed = true;
  if (rose) { buttonPressed = false; pwrBtnPressTime = 0; }

  if (buttonPressed)
  {
    if (pwrBtnPressTime == 0) pwrBtnPressTime = millis();
    if (millis() - pwrBtnPressTime >= hardOffTime)
    {
      pwrBtnPressTime = 0;
      buttonPressed   = false;
      enterStandby();
      return;
    }
  }
}


// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
  // LED flash on PD6 for signs of life. Must happen before any Serial.begin()
  // call, since PD6 doubles as UART1 RX once Serial is initialized.
  pinMode(POWER_ENABLE_PIN, OUTPUT);
  digitalWrite(POWER_ENABLE_PIN, POWER_OFF);  // high-side off

  pinMode(PROGRAM_MODE_PIN, INPUT_PULLUP);    // pullup to high to make the ESP32 boot into SPI boot mode by default

  pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUTTON_0_PIN,     INPUT_PULLUP);
  pinMode(BUTTON_1_PIN,     INPUT_PULLUP);
  pinMode(BUTTON_2_PIN,     INPUT_PULLUP);

  pinMode(PD5, INPUT_PULLUP);
  pinMode(PD6, INPUT_PULLUP);

  // WCH-specific interrupt attach signature
  attachInterrupt(POWER_BUTTON_PIN, GPIO_Mode_IPU, buttonISR,
                  EXTI_Mode_Interrupt, EXTI_Trigger_Falling);

  if (wokeFromStandby) initialized = false;
}


// ── loop ──────────────────────────────────────────────────────────────────────
void loop()
{
  if (sleepTimer == 0 && !outputIsOn && sleepTimerRequired) sleepTimer = millis();

  if (outputIsOn)
  {
    sleepTimer = 0;
    sleepTimerRequired = false;
  }

  // this puts the CH32 to sleep if the power output is off and enough time has
  // elapsed. 
  if (!outputIsOn && ((sleepTimer > 0) && (millis() - sleepTimer > SLEEP_TIMEOUT)))
  {
    sleepTimer     = 0;
    pressStartTime = 0;
    enterStandby();
  }

  // this call also performs edge detection on the power button
  watchForHardPowerOffReq();

  if (!outputIsOn && buttonFell)
  {
    if (digitalRead(POWER_BUTTON_PIN) == LOW)
    {
      digitalWrite(POWER_ENABLE_PIN, POWER_ON);
      outputIsOn         = true;
      sleepTimerRequired = false;
      buttonFell         = false;
    }
  }

  // exit early if output power hasn't been turned on yet
  if (!outputIsOn) return;

  if (!initialized)
  {
    Serial.begin(9600);
    waitForHandshake();
    initialized = true;
  } else
  {
    // handle incoming UART messages
    handleMessages();

    currentState = buildState();
    
    // check to see if the power button and one of the front panel buttons are pressed simultaneously.
    // this is how the user can have the CH32 reboot the ESP32 into programming mode.

    // this should evaluate to true if the power button is pressed and at least one front panel button is pressed
    if ((currentState & 0b00000001) && (currentState & 0b00001110)) 
    {
      initialized = false;
      // sleepTimerRequired = true;  // restart the sleep timer. don't know if i want to include this or not
      digitalWrite(POWER_ENABLE_PIN, POWER_OFF);
      delay(200);
      pinMode(PROGRAM_MODE_PIN, OUTPUT);    // pull down to ground to put ESP32 into program download mode
      digitalWrite(PROGRAM_MODE_PIN, LOW);
      digitalWrite(POWER_ENABLE_PIN, POWER_ON);
      delay(200);                                   // probably long enough to allow the ESP32 to boot up
      pinMode(PROGRAM_MODE_PIN, INPUT_PULLUP);      // reset this state to default
      return;
      // I'm hoping that this is all I need to do. the ESP32 should reboot into program downloading mode and wait
      // for the computer to upload code and trigger a reset. meanwhile, the CH32 should just go back into the
      // initialization phase where it waits for a handshake again, which the ESP32 should do once it restarts
      // and begins the new program.
    }
    
    if (currentState != lastState) Serial.write(currentState);
    lastState = currentState;
  }
}


void handleMessages()
{
  uint8_t msg = 0;
  uint8_t buf[2];
  uint16_t t = 0;
  if (Serial.available()) msg = (uint8_t)Serial.read(); else return;
  
  switch (msg)
  {
    case MSG_INIT:
      initialized = false;
      break;
    
    case MSG_CLOSE:
      // Shut down UART and return PD5/PD6 to inputs so they don't drive anything. Should
      // allow these pins to be used for other things, but I really don't know why this
      // would be desired or even when it would be used.
      Serial.end();
      pinMode(PD5, INPUT);
      pinMode(PD6, INPUT);
      break;
    
    case MSG_RESET:
      initialized = false;
      digitalWrite(POWER_ENABLE_PIN, POWER_OFF);
      delay(500);
      digitalWrite(POWER_ENABLE_PIN, POWER_ON);
      break;
    
    case MSG_SET_HARD_OFF_TIME:
      // this might be a blocking function, so be aware of that.
      Serial.readBytes(buf, 2);
      // low byte gets sent first, then high byte, so I'm assuming buf[2] == {low, high}
      t = ((uint16_t)(buf[1] << 8) | buf[0]);   // i think this should properly reconstruct the time
      if (t > 500) hardOffTime = t; else hardOffTime = POWER_OFF_HOLD_TIME;
      break;
    
    case MSG_SHUTDOWN:
      enterStandby();
      // for (uint8_t i = 0; i < 5; i++)
      // {
      //   digitalWrite(LED_PIN, HIGH);
      //   delay(1000);
      //   digitalWrite(LED_PIN, LOW);
      //   delay(1000);
      // }
      break;
    
    default:
      break;
  }
}


// ── enterStandby ──────────────────────────────────────────────────────────────
void enterStandby() {
  outputIsOn         = false;
  pressStartTime     = 0;
  buttonFell         = false;
  systemWasInStandby = true;
  sleepTimerRequired = true;
  initialized        = false;
  // Shut down UART and return PD5/PD6 to inputs so they don't drive anything
  // while the ESP32 is unpowered.
  Serial.end();
  pinMode(PD5, INPUT);
  pinMode(PD6, INPUT);

  // High-side off
  digitalWrite(POWER_ENABLE_PIN, POWER_OFF);

  EXTI->INTFR = 0xFFFFFFFF;
  PWR_EnterSTANDBYMode(PWR_STANDBYEntry_WFI);
  // execution resumes here on wake
  // we have to restart the external clock for the dev board. This might cause problems
  // when I switch to the internal oscillator on the actual hardware, so be aware of this.
  // I think it will probably be fine, but be sure to test.
  SystemInit();  // reconfigure HSE + PLL, restore full clock speed
}