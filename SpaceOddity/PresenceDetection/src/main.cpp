// =============================================================================
// Hack Pack: Space Oddity
// Hack: Presence Detection
// =============================================================================
// 
// In this version, the creature's behavior is driven by presence detection: 
// an HC-SR04 ultrasonic sensor (trigger on IO9, echo on IO7) watches for a person
// nearby.
//
// The robot boots straight into SWIM behavior, roaming the jar on its own.
// When an object is within sensing range it reacts to proximity:
//   - As a nearby object gets closer, the creature swims faster, ramping up
//     to its fastest pace as it approaches.
//   - Inside ~10cm it switches to a GET_ANGRY response - faster darting,
//     harsher servo motion, and a fire-colored LED display - until the hand
//     pulls back out of close range.
//   - If it hasn't seen anyone for a while (or right after power-on), the
//     first approach in range triggers a fixed-duration DANCE greeting before
//     settling back into its normal swim.
//   - When nothing is present it simply keeps swimming at its idle pace.
//
// Tuning in configuration.h: the presence detection section at the top exposes
// the ultrasonic ping interval, the presence/angry/fast distance thresholds in
// cm, the maximum swim-speed multiplier, and the dance trigger and duration
// timings.
//
// =============================================================================

#include <Arduino.h>
#include <configuration.h>      // behavior configuration variables
#include <hardware_map.h>       // hardware variables

#include <SPI.h>                
#include <Wire.h>               
#include <Adafruit_GFX.h>       // graphics library
#include <Adafruit_SSD1306.h>   // display management library
#include <ezButton.h>           // button debouncing
#include <FastLED.h>  // pinned at version 3.7.8; newer versions don't work well with ESP32-C3 toolchain yet
#include <ServoWrapper.h>       // some useful functions for controlling servos
#include <AgileStateMachine.h>  // a very nice state machine library
#include <PowerLink.h>          // interface with the power and front panel buttons via the coprocessor
#include <TwinkleFox.h>         // lighting effect from FastLED
#include <CardioidArray.h>      // heart-like pattern
#include <bootloader_random.h>  // for hardware RNG support
#include <Ultrasonic.h>         // HC-SR04 distance sensing

#define TRIG_PIN 9
#define ECHO_PIN 7

// HC-SR04 ultrasonic sensor (trig on IO9, echo on IO7).
// The extra constructor argument is the no-object timeout in microseconds.
Ultrasonic ultrasonic(TRIG_PIN, ECHO_PIN, 40000UL);


// =============================================================================
// POWER AND FRONT BUTTONS CONFIGURATION
// =============================================================================
// The main PCB has a CH32V003 microcontroller on board that acts as a coprocessor.
// This chip manages the front buttons, the power button, and power management hardware.
// It communicates with the ESP32-C3 over UART. This object sets up that link.
PowerLink carrierPCB;   


// =============================================================================
// PAGE/MODE DEFINITIONS
// =============================================================================

// MenuItem shows the item that the user selects in the main menu.
// The menu is separate from Mode. The menu selection starts a mode.
enum class MenuItem
{
  SWIM
};

// Mode shows the active behavior of the robot.
// The finite state machine (FSM) sets this value when the mode changes.
enum class Mode
{
  IDLE,
  SWIM,
  PARK,
  PITCH_DROP,
  CARDIOID,
  DANCE
};

// =============================================================================
// GLOBAL OBJECTS
// =============================================================================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

CRGB leds[NUM_LEDS];
TwinkleFox twinkle(leds, NUM_LEDS);

ezButton joystickButton(PIN_JOYSTICK_BUTTON);


// =============================================================================
// STATE VARIABLES
// These variables track the robot state. Some may move into the state machine later.
// =============================================================================

// Joystick state
int16_t joyXval, joyYval;

int8_t joyXAxis = 0, joyYAxis = 0;

// Selected menu item on main menu
MenuItem currentMenuSelection = MenuItem::SWIM, previousMenuSelection = MenuItem::SWIM;
int8_t currentMenuSelectionIndex = 0;   // used for changing selected items

// Mode states
Mode currentMode = Mode::SWIM;
Mode returnMode = Mode::IDLE;     // stores the mode to resume after pitchDrop completes


// State machine stuff
bool newStateEntry = false;
bool performNewEntryTimedPhase = false;

// Swim algorithm state
int16_t swimX = JOY_CENTER;
int16_t swimY = JOY_CENTER;
float swimHeading;
bool getAngry = false;

// Multiplier applied to swim speed while a hand is in range (closer = faster)
float presenceSpeedFactor = 1.0f;

// Timing
uint32_t lastPatternUpdate = 0;
uint32_t lastServoUpdate = 0;
uint32_t lastUltrasonicPing = 0;
// Ultrasonic sensor tracking
uint32_t lastObjectDetected = 0;
enum DanceState { DANCE_IDLE, DANCE_RUNNING } danceState = DANCE_IDLE;

// =============================================================================
// FUNCTION PROTOTYPES
// =============================================================================

// Input handling
void updateJoystick();

// Page/mode management
void handleMainMenuPage();
void handleSwimPage();
void handleDancePage();
void leaveDance();
void pitchDrop();
void cardioidTrace();

// Servo control
void calculateRotatedServoTargets(int16_t px, int16_t py);
void updateMagnetPulse();

// Shared servo helpers (defined near steppedMove, below)
void attachArms(uint16_t staggerMs);
void attachAll(uint16_t staggerMs);
void blockingMoveArms(float closeEnough, bool allowShutdown);
void blockingMoveServo(ServoWrapper &s, float closeEnough, bool allowShutdown);
void resumeReturnMode(int16_t swimOffsetX, int16_t swimOffsetY);

// Display functions
void drawStartupAnimation();
void drawMainMenu(MenuItem selected = MenuItem::SWIM, bool outputDisplay = true);
void animateMainMenu(MenuItem selected);

// LED functions
void updateRainbowCycle();
CRGB colorWheel(uint8_t position);
void Fire2012();

// create the servo objects
ServoWrapper leftServo(PIN_SERVO_LEFT, SERVO_LEFT_TRIM), rightServo(PIN_SERVO_RIGHT, SERVO_RIGHT_TRIM), magnetServo(PIN_SERVO_MAGNET, MAGNET_SERVO_TRIM);

// Utility functions
int32_t randomInRange(int32_t min = INT32_MIN, int32_t max = INT32_MAX);

// map floating point numbers into a new range
inline float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// =============================================================================
// STATE MACHINE STUFF
// =============================================================================

// Set up state machine
bool magnetParked = false;
void pickUpFerrofluid();    // function to grab ferrofluid from bottom of jar
void putItInPark();         // function to safely park magnet
// add an action to the state machine that toggles a parked/notparked variable

// these functions check to see if the current mode matches the desired mode, 
// and returns that as a boolean.
// These are used to trigger transitions in the state machine
bool wantSwim()      { return currentMode == Mode::SWIM; }
bool wantIdle()      { return currentMode == Mode::IDLE; }
bool wantPitchDrop() { return currentMode == Mode::PITCH_DROP; }
bool wantCardioid() { return currentMode == Mode::CARDIOID; }
bool wantDance()     { return currentMode == Mode::DANCE; }

// The finite state machine (FSM) controls the robot behavior.
// Each state runs one mode. The FSM changes state when the user selects a
// new mode or presses a button.
StateMachine fsm;

void setupStateMachine() {
  // State* name = fsm.addState("text name", min duration, max duration, onEntry, onExit, onRun);
  State* st_Idle  = fsm.addState("Idle", 0, 0, putItInPark, nullptr, handleMainMenuPage);
  State* st_Swim  = fsm.addState("Swim", 0, 0, pickUpFerrofluid, nullptr, handleSwimPage);
  State* st_Park  = fsm.addState("Park", 0, 0, putItInPark, nullptr, nullptr);               // no min or max duration,  put it in park on entry
  State* st_PitchDrop = fsm.addState("PitchDrop", 0, 0, pitchDrop, nullptr, nullptr);
  State* st_Cardioid = fsm.addState("Cardioid", 0, 0, cardioidTrace, nullptr, nullptr);
  State* st_Dance = fsm.addState("Dance", 0, 0, pickUpFerrofluid, leaveDance, handleDancePage);

  
  // add transitions between states
  st_Idle->addTransition(st_Swim, wantSwim);    // if we're in the idle state, but the currentMode gets changed to Mode::SWIM, transition to st_Swim
  
  st_Swim->addTransition(st_Idle, wantIdle);
  st_Swim->addTransition(st_PitchDrop, wantPitchDrop);
  st_Swim->addTransition(st_Cardioid,  wantCardioid);
  st_Swim->addTransition(st_Dance, wantDance);
  
  st_PitchDrop->addTransition(st_Swim,  wantSwim);
  st_PitchDrop->addTransition(st_Cardioid, wantCardioid);
  
  st_Cardioid->addTransition(st_Swim,  wantSwim);
  st_Cardioid->addTransition(st_PitchDrop, wantPitchDrop);

  st_Dance->addTransition(st_Swim, ULTRASONIC_DANCE_DURATION_MS);   // timed: the dance always runs its full duration before returning to swim
  
  
  // add actions to states
  st_Idle->addAction(Action::Type::S, magnetParked);
  st_Idle->addAction(Action::Type::RE, newStateEntry);
  
  st_Swim->addAction(Action::Type::R, magnetParked);      // this will reset magnetParked to false when the state gets activated
  st_Swim->addAction(Action::Type::RE, newStateEntry);     // this will set the newSwimEntry variable to true when the state is activated, but not on subsequent calls
  st_Swim->addAction(Action::Type::L, performNewEntryTimedPhase, 500);
  
  st_Park->addAction(Action::Type::S, magnetParked);      // set magnetParked to true when the state is activated
  st_Park->addAction(Action::Type::RE, newStateEntry);
  
  st_PitchDrop->addAction(Action::Type::RE, newStateEntry);
  
  st_Cardioid->addAction(Action::Type::RE, newStateEntry);

  st_Dance->addAction(Action::Type::RE, newStateEntry);

  fsm.setInitialState(st_Swim);
  fsm.start();
}




// =============================================================================
// SETUP
// =============================================================================

void setup() {
  // hardware random number generator (RNG) initialization. 
  // NOTE: THIS HAS TO BE DISABLED IF YOU WANT TO USE WIFI OR BLUETOOTH
  bootloader_random_enable();

  SERIAL_BEGIN(115200);   // start serial communications with computer
  delay(100);
  SERIAL_PRINTLN("begin");
  carrierPCB.begin();   // start serial communications with the CH32V003 on the carrier PCB
  SERIAL_PRINTLN("UART established");

  // start the I2C connection to the display
  Wire.setPins(21, 20);
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_I2C_ADDR);
  
  display.clearDisplay();
  drawStartupAnimation();
  delay(500);
  
  FastLED.addLeds<WS2812B, PIN_LEDS, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);
  twinkle.setTwinkleSpeed(8);
  twinkle.setTwinkleDensity(5);
  twinkle.setSecondsPerPalette(8);
  twinkle.setCoolLikeIncandescent(true);
  
  // assume the arms are physically at the park pose on power-up, so attaching
  // holds that pose and causes no movement (staggered to flatten inrush current)
  leftServo.syncCurrentPos(180.0);
  rightServo.syncCurrentPos(0.0);
  magnetServo.syncCurrentPos(MAGNET_PARK_POS);
  leftServo.attach();
  delay(250);
  rightServo.attach();
  delay(250);
  magnetServo.attach();
  delay(100);

  leftServo.targetPos = 180.0;
  rightServo.targetPos = 0.0;
  leftServo.write(leftServo.targetPos);
  rightServo.write(rightServo.targetPos);

  joystickButton.setDebounceTime(DEBOUNCE_MS);
  randomSeed(micros());    // use the amount of time the system has been on as the seed for the PRNG.
  swimHeading = randomInRange(0, 360) * DEG_TO_RAD;

  // initialize the indices to the correct numbers
  currentMenuSelectionIndex = static_cast<int8_t>(currentMenuSelection);  
  
  // start the state machine 
  setupStateMachine();
  
  delay(500);
  SERIAL_PRINTLN("ἧἐἶἶὃῥἶϿ");
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
  static uint32_t powerButtonPressedTime = 0, stayAngryTimer = 0;
  
  carrierPCB.update();      // gets the updated power and front panel button states from the main PCB

  // if the power button is newly pressed start the timer
  if (carrierPCB.stateChanged())
  {
    if (carrierPCB.buttonPressed(PowerLink::BTN_POWER)) powerButtonPressedTime = millis();
    if (carrierPCB.buttonReleased(PowerLink::BTN_POWER)) powerButtonPressedTime = 0;     // if the power button is released, reset the timer
  }
  

  // Ultrasonic sensor handling at 10Hz
  if (millis() - lastUltrasonicPing > ULTRASONIC_PING_INTERVAL)
  {
    lastUltrasonicPing = millis();
    int16_t distance = ultrasonic.read();

    // A valid reading is a real, nearby presence (not distant background like
    // walls or furniture). Only these count toward "an object was detected".
    if (distance >= 2 && distance <= ULTRASONIC_PRESENCE_CM)
    {
      bool firstDetection = (lastObjectDetected == 0);
      // time since the previous presence (large on the first detection after
      // power-on, or after a long stretch with nobody around)
      uint32_t detectionGap = millis() - lastObjectDetected;

      lastObjectDetected = millis();

      // Within 10cm = GET_ANGRY range
      if (distance <= ULTRASONIC_ANGRY_CM)
      {
        getAngry = true;
        stayAngryTimer = millis();
      }

      // As the object gets closer, swim speed increases
      presenceSpeedFactor = constrain(
          fmap(distance, ULTRASONIC_ANGRY_CM, ULTRASONIC_FAST_CM,
               ULTRASONIC_MAX_SPEED_BOOST, 1.0f),
          1.0f, ULTRASONIC_MAX_SPEED_BOOST);

      SERIAL_PRINT("dist(cm)\t");
      SERIAL_PRINT(distance);
      SERIAL_PRINT("\tspeed\t");
      SERIAL_PRINT(presenceSpeedFactor);
      SERIAL_PRINT("\treducer\t");
      SERIAL_PRINT(getAngry ? 70.0f
               : (performNewEntryTimedPhase ? 0.001f : presenceSpeedFactor));
      SERIAL_PRINT("\tmode\t");
      SERIAL_PRINTLN((int)currentMode);

      // First detection after power-on, or a minute without anyone around,
      // gets a greeting dance
      if (danceState == DANCE_IDLE && currentMode == Mode::SWIM &&
          (firstDetection || detectionGap >= ULTRASONIC_DANCE_TRIGGER_MS))
      {
        danceState = DANCE_RUNNING;
        currentMode = Mode::DANCE;
        SERIAL_PRINTLN("DANCE: greeting");
      }
    }
    else
    {
      // No meaningful presence: keep swimming normally. The time since the
      // last presence keeps growing so a return after a minute greets the
      // creature with a dance. The dance itself is never interrupted here;
      // it always runs its full duration via the FSM's timed transition.
      presenceSpeedFactor = 1.0f;
      SERIAL_PRINT("dist(cm)\t");
      SERIAL_PRINT(distance);
      SERIAL_PRINT("\tspeed\t1.0\treducer\t");
      SERIAL_PRINT(getAngry ? 70.0f
               : (performNewEntryTimedPhase ? 0.001f : presenceSpeedFactor));
      SERIAL_PRINT("\tmode\t");
      SERIAL_PRINTLN((int)currentMode);
    }
  }

  /// if the power button has been held long enough, park the magnet and shut down the power  /// if the power button has been held long enough, park the magnet and shut down the power
  if (powerButtonPressedTime != 0 && millis() - powerButtonPressedTime >= SOFT_SHUTDOWN_TIME)
  {
    if (!magnetParked) putItInPark();
    delay(500);
    carrierPCB.shutdown();
  }

  // deal with the other buttons now
  if (carrierPCB.stateChanged())
  {
    // Button 0: Lambda ( λ )
    if (carrierPCB.buttonPressed(PowerLink::BTN_0))
    {
      SERIAL_PRINTLN("λ");
      getAngry = true;
      stayAngryTimer = millis();
    }
    // Button 1: Phi ( φ )
    if (carrierPCB.buttonPressed(PowerLink::BTN_1))
    {
      SERIAL_PRINTLN("\tφ");
      if (currentMode != Mode::IDLE)
        {
            returnMode = currentMode;
            currentMode = Mode::CARDIOID;
        }
    }
    // Button 2: Delta ( δ )
    if (carrierPCB.buttonPressed(PowerLink::BTN_2))
    {
        SERIAL_PRINTLN("\t\tδ");
        if (currentMode != Mode::IDLE)
        {
            returnMode = currentMode;
            currentMode = Mode::PITCH_DROP;
        }
    }
  }

  if (millis() - lastPatternUpdate > PATTERN_INTERVAL_MS)
  {
    FastLED.setBrightness(LED_BRIGHTNESS);
    if (getAngry && !magnetParked && millis() - stayAngryTimer <= stayAngryDuration)
    {
      Fire2012();
    } else if (currentMode == Mode::DANCE)
    {
      FastLED.setBrightness(constrain(LED_BRIGHTNESS * 2, 0, 255));
      twinkle.run();
    } else 
    {
      updateRainbowCycle();
      getAngry = false;
    }
    lastPatternUpdate = millis();
  }

  updateJoystick();
  fsm.execute();
}

// =============================================================================
// INPUT HANDLING
// =============================================================================
// Read the joystick and the joystick button.
// Store the raw values and the up/down/left/right direction.

void updateJoystick() {
  joystickButton.loop();
  joyXval = (4096 - analogRead(PIN_JOY_X)) >> 2;    // we don't need to use map() for this, we can just reverse the range with subtraction
  joyYval = (4096 - analogRead(PIN_JOY_Y)) >> 2;    // and then bit shift to droop down to 10 bit. This is also just a hack. Could rewrite for 12 bit.

  joyXAxis = joyXval < static_cast<int16_t>(JOY_CENTER - JOY_THRESHOLD) ? -1 : (joyXval > static_cast<int16_t>(JOY_CENTER + JOY_THRESHOLD) ? 1 : 0);
  joyYAxis = joyYval < static_cast<int16_t>(JOY_CENTER - JOY_THRESHOLD) ? -1 : (joyYval > static_cast<int16_t>(JOY_CENTER + JOY_THRESHOLD) ? 1 : 0); 
}

// =============================================================================
// PAGE/MODE MANAGEMENT
// =============================================================================

// Show the main menu and let the user pick a mode.
// Move the joystick up or down to change the selection.
// Press the joystick button to start the selected mode.

void handleMainMenuPage() { 
  // Navigation
  // get the index of the currently selected menu item by converting the MenuItem into int8_t
  currentMenuSelectionIndex = static_cast<int8_t>(currentMenuSelection);
  // add the increment or decrement from the joystick, and handle bidirectional wrapping
  currentMenuSelectionIndex = (currentMenuSelectionIndex + joyYAxis + MENU_ITEM_COUNT) % MENU_ITEM_COUNT;
  // convert the index back into a MenuItem
  currentMenuSelection = static_cast<MenuItem>(currentMenuSelectionIndex);

  // if the selection has changed, redraw the menu
  if ((currentMenuSelection != previousMenuSelection) || (newStateEntry))
  {
    drawMainMenu(currentMenuSelection, true);
    delay(MENU_NAV_DELAY_MS);
  }

  // Selection
  if (joystickButton.isPressed()) {
    // no servo attach needed here: pickUpFerrofluid() takes over the pins via LEDC,
    // and each mode's onRun reattaches RoboServo afterwards as needed
    currentMode = Mode::SWIM;

    display.clearDisplay();  
  }
  previousMenuSelection = currentMenuSelection;
}

// =============================================================================
// SWIM BEHAVIOR
// =============================================================================

// SWIM mode: the robot moves the magnet through the fluid on its own.
// The magnet follows a heading and turns toward the fluid. The user can
// press the joystick button to go back to the menu.
void handleSwimPage() 
{
  animateMainMenu(currentMenuSelection);
  
  if (joystickButton.isPressed()) {
    currentMode = Mode::IDLE;
    return;
  }

  updateMagnetPulse();

  if (getAngry)
  {
    leftServo.setSmoothing(0.8); rightServo.setSmoothing(0.8);
  } else
  {
    // Less damping as the object gets closer, so the physical arm actually
    // accelerates along with the faster target. Smoothing 0.999 (lazily
    // drifting) at factor 1.0 down to 0.8 (lively and quick) at max factor.
    float presenceSmoothing = fmap(presenceSpeedFactor, 1.0f, ULTRASONIC_MAX_SPEED_BOOST,
                                   0.999f, 0.8f);
    presenceSmoothing = constrain(presenceSmoothing, 0.8f, 0.999f);
    leftServo.setSmoothing(presenceSmoothing);
    rightServo.setSmoothing(presenceSmoothing);
  }

  static bool reattachServos = false;
  if (!reattachServos && newStateEntry) reattachServos = true;

  if (millis() - lastServoUpdate >= UPDATE_INTERVAL_MS)
  {
    float deltaHeading = randomInRange(-10, 10) / 100.0f;

    // Edge avoidance: steer toward center when near any boundary
    bool nearEdge = (swimX < SWIM_MARGIN) || 
                    (swimX > JOY_MAX - SWIM_MARGIN) ||
                    (swimY < SWIM_MARGIN) || 
                    (swimY > JOY_MAX - SWIM_MARGIN);

    if (nearEdge) {
      float toCenter = atan2f(JOY_CENTER - swimY, JOY_CENTER - swimX);
      float diff = toCenter - swimHeading;
      // Normalize to [-PI, PI]
      while (diff > PI)  diff -= TWO_PI;
      while (diff < -PI) diff += TWO_PI;
      deltaHeading += SWIM_STEER_STRENGTH * diff;
    }


    swimHeading += deltaHeading;
    swimHeading = fmod(swimHeading, TWO_PI);

    float speedReducer = performNewEntryTimedPhase ? 0.001 : presenceSpeedFactor; // sleepy on entry, faster when something is close
    if (getAngry) speedReducer = 70.0;   // move way too fast if angry

    swimX += lroundf(SWIM_SPEED * speedReducer * cos(swimHeading));
    swimY += lroundf(SWIM_SPEED * speedReducer * sin(swimHeading));

    swimX = constrain(swimX, 0, JOY_MAX);
    swimY = constrain(swimY, 0, JOY_MAX);


    // Bounce off edges
    bool hitX = (swimX == 0 || swimX == JOY_MAX);
    bool hitY = (swimY == 0 || swimY == JOY_MAX);
    if (hitX) swimHeading = PI - swimHeading;
    if (hitY) swimHeading = -swimHeading;

    if (reattachServos)
    {
      calculateRotatedServoTargets(JOY_CENTER, JOY_CENTER + 500);
      attachArms(100);
      leftServo.write(leftServo.targetPos);   // force physical + internal state to match
      rightServo.write(rightServo.targetPos);
      reattachServos = false;
    } else
    {
      calculateRotatedServoTargets(swimX, swimY);
    }

    leftServo.moveTo(leftServo.targetPos);
    rightServo.moveTo(rightServo.targetPos);

    lastServoUpdate = millis();
  }
}


// =============================================================================
// SERVO CONTROL
// =============================================================================
// Find the servo angles that place the magnet at the point (px, py).
// The robot is a 5R parallel arm. We turn the input by 45 degrees to map
// the X/Y point to each arm. This is an approximation of inverse kinematics
// (IK). IK finds the servo angles that put the magnet at a point.
// X_COORD_TRIM and Y_COORD_TRIM shift the point back to the true center.
void calculateRotatedServoTargets(int16_t px, int16_t py) {
  // 45-degree rotation matrix to approximate the proper inverse kinematics for a 5R parallel robot.
  // Converts XY coordinates to angles for each servo. 
  constexpr float COS_45 = 0.70710678f;

  int16_t x = px - (JOY_CENTER + X_COORD_TRIM);
  int16_t y = py - (JOY_CENTER + Y_COORD_TRIM);

  float rotatedX = static_cast<float>(x - y) * COS_45;
  float rotatedY = static_cast<float>(x + y) * -COS_45;

  int16_t servo1Value = constrain(static_cast<float>(rotatedX + JOY_CENTER), 0, JOY_MAX);
  int16_t servo2Value = constrain(static_cast<float>(rotatedY + JOY_CENTER), 0, JOY_MAX);

  leftServo.targetPos = fmap(servo1Value, 0, JOY_MAX, SERVO_MIN, SERVO_MAX);
  rightServo.targetPos = fmap(servo2Value, 0, JOY_MAX, SERVO_MAX, SERVO_MIN);
}


// Move the magnet up and down a small amount when the robot is idle.
// This makes the fluid "breathe". The move stops when a mode takes control.
void updateMagnetPulse() {
  static uint32_t lastUpdate = 0;
  static int8_t magnetStep = 3;
  if (millis() - lastUpdate < MAGNET_SERVO_UPDATE_INTERVAL) return;
  if (magnetServo.targetPos >= MAGNET_MAX) {
    magnetStep = -3;
  } else if (magnetServo.targetPos <= MAGNET_MIN) {
    magnetStep = 3;
  }
  magnetServo.targetPos = constrain(magnetServo.targetPos + (float)magnetStep, MAGNET_MIN, MAGNET_MAX);
  magnetServo.setSmoothing(0.9);
  magnetServo.moveTo(magnetServo.targetPos);
  lastUpdate = millis();
}


// =============================================================================
// DISPLAY FUNCTIONS
// =============================================================================
void drawStartupAnimation() {
  display.clearDisplay();
  int maxRadius = max(display.width(), display.height()) / 2;
  for (int16_t r = maxRadius; r > 0; r -= 3) {
    display.fillCircle(display.width() / 2, display.height() / 2, r, SSD1306_INVERSE);
    display.display();
  }
}

void drawMainMenu(MenuItem selected, bool outputDisplay) {
  display.clearDisplay();
  display.setTextSize(1);

  for (uint8_t i = 0; i < MENU_ITEM_COUNT; i++) {
    uint8_t yPos = MENU_Y_START + (i * MENU_Y_SPACING);
    bool isSelected = (i == static_cast<uint8_t>(selected));

    if (isSelected) {
      display.fillRoundRect(MENU_X_OFFSET, yPos - 4, MENU_ITEM_WIDTH, MENU_ITEM_HEIGHT, MENU_CORNER_RAD, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.drawRoundRect(MENU_X_OFFSET, yPos - 4, MENU_ITEM_WIDTH, MENU_ITEM_HEIGHT, MENU_CORNER_RAD, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(MENU_TEXT_OFFSET, yPos);
    display.print(MENU_LABELS[i]);
  }

  // used to control whether or not the display actually gets updated immediately
  if (outputDisplay) display.display();
}

void animateMainMenu(MenuItem selected)
{
  constexpr uint8_t MAX_R = (MENU_ITEM_HEIGHT - 2) / 2, MIN_R = 2, R_STEP = 1;
  static MenuItem lastSelected = selected;
  static uint8_t radius = MAX_R;
  static int8_t stepDir = 1;
  static uint32_t lastUpdateTime = 0;
  static bool firstEntry = true;
  constexpr uint32_t INTERVAL = 800;
  static uint8_t bwSelector = SSD1306_BLACK;

  
  if (selected != lastSelected || firstEntry)
  {
    radius = MAX_R;
    stepDir = 1;
    firstEntry = false;
    lastUpdateTime = 0;
  }
  if (millis() - lastUpdateTime >= INTERVAL)
  {
    drawMainMenu(selected, false);    // draw to the buffer, don't actually push the contents to the display yet
    uint8_t yPos = MENU_Y_START + (currentMenuSelectionIndex * MENU_Y_SPACING);
    
    if (radius == MAX_R || radius == MIN_R) stepDir *= -1;
    if (radius == MAX_R) bwSelector = SSD1306_BLACK;

    display.fillCircle
    (
      ((SCREEN_WIDTH - (SCREEN_WIDTH - MENU_ITEM_WIDTH)) >> 1) + MENU_ITEM_WIDTH - MENU_ITEM_HEIGHT + 5, 
      yPos - 4 + (MENU_ITEM_HEIGHT >> 1), 
      radius, 
      bwSelector
    );

    radius += R_STEP * stepDir;

    display.display();
    lastUpdateTime = millis();
  }
  lastSelected = selected;
}

// =============================================================================
// LED FUNCTIONS
// =============================================================================

void updateRainbowCycle() {
  static uint16_t wheelOffset = 0;

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = colorWheel(((i * 256 / 32) + wheelOffset) & 255);
  }
  FastLED.show();

  wheelOffset++;
  if (wheelOffset >= 256 * 5) wheelOffset = 0;

  lastPatternUpdate = millis();
}


CRGB colorWheel(uint8_t position) {
  position = 255 - position;
  if (position < 85) {
    return CRGB(255 - position * 3, 0, position * 3);
  }
  if (position < 170) {
    position -= 85;
    return CRGB(0, position * 3, 255 - position * 3);
  }
  position -= 170;
  return CRGB(position * 3, 255 - position * 3, 0);
}

// keeps the UART link alive and (optionally) the power button responsive
// during long blocking moves. Pass allowShutdown = false when a shutdown
// is already in progress (e.g. from putItInPark).
// Check the power button on the carrier board.
// If the user holds the button long enough, shut the robot down.
// Call this often so the button stays responsive during long moves.
void servicePowerButton(bool allowShutdown = true)
{
  static uint32_t pressedSince = 0;
  carrierPCB.update();
  if (carrierPCB.isPressed(PowerLink::BTN_POWER))
  {
    if (pressedSince == 0) pressedSince = millis();
    else if (allowShutdown && millis() - pressedSince >= SOFT_SHUTDOWN_TIME) carrierPCB.shutdown();
  }
  else
  {
    pressedSince = 0;
  }
}

// =============================================================================
// CONSTANT-SPEED STEPPED MOVE (entry/exit moves only)
// =============================================================================
// Same motion profile as the servo_slow_move test sketch (constant deg/s, 20ms
// cadence, staggered start), but driven through ServoWrapper so the PWM signal
// stays alive for the whole move — no detach/reattach, no torque interruption,
// no supply transient at the mode boundary.
struct StepMove {
  ServoWrapper *servo;
  float        targetAngle;
  uint32_t     startAt;       // staggered-start timestamp
};

static void steppedMove(StepMove moves[], uint8_t count, bool allowShutdown, float degPerSec = SLOW_MOVE_DPS)
{
  uint32_t moveStart = millis();
  uint32_t lastStep = millis();
  bool moving = true;
  while (moving && millis() - moveStart < BLOCKING_MOVE_TIMEOUT_MS)
  {
    servicePowerButton(allowShutdown);
    uint32_t now = millis();
    if (now - lastStep < SLOW_STEP_INTERVAL_MS) { delay(1); continue; }   // yield to idle task (WDT)
    lastStep = now;

    float stepDeg = degPerSec * SLOW_STEP_INTERVAL_MS / 1000.0f;
    if (stepDeg < 0.05f) stepDeg = 0.05f;

    moving = false;
    for (uint8_t i = 0; i < count; i++)
    {
      StepMove &m = moves[i];
      float cur = m.servo->getCurrentPos();
      float diff = m.targetAngle - cur;
      if (fabsf(diff) <= 0.01f) continue;
      moving = true;
      if ((int32_t)(now - m.startAt) < 0) continue;   // still waiting for staggered start
      m.servo->write((fabsf(diff) <= stepDeg) ? m.targetAngle
                                              : cur + ((diff > 0) ? stepDeg : -stepDeg));
    }
  }
}

// =============================================================================
// SHARED SERVO HELPERS
// =============================================================================

// Attach the two arm servos. Wait a short time between them so the inrush
// current is spread out. This stops the supply from dropping. attachAll()
// does the same and also attaches the magnet servo.
void attachArms(uint16_t staggerMs = 100)
{
  leftServo.attach();  delay(staggerMs);
  rightServo.attach(); delay(staggerMs);
}
void attachAll(uint16_t staggerMs = 100)
{
  leftServo.attach();  delay(staggerMs);
  rightServo.attach(); delay(staggerMs);
  magnetServo.attach(); delay(staggerMs);
}

// Wait until both arm servos reach their target (within closeEnough degrees).
// Stop if the move takes too long. Check the power button each loop so the
// user can still shut down during a long move. Used by pitchDrop and cardioidTrace.
void blockingMoveArms(float closeEnough, bool allowShutdown = true)
{
  uint32_t moveStart = millis();
  bool moving = true;
  while (moving && millis() - moveStart < BLOCKING_MOVE_TIMEOUT_MS)
  {
    servicePowerButton(allowShutdown);
    leftServo.moveTo(leftServo.targetPos);
    rightServo.moveTo(rightServo.targetPos);
    moving = (fabsf(leftServo.getCurrentPos() - leftServo.targetPos) <= closeEnough &&
              fabsf(rightServo.getCurrentPos() - rightServo.targetPos) <= closeEnough);
  }
}

// Same as blockingMoveArms but for a single servo (used for the magnet in pitchDrop).
void blockingMoveServo(ServoWrapper &s, float closeEnough, bool allowShutdown = true)
{
  uint32_t moveStart = millis();
  bool moving = true;
  while (moving && millis() - moveStart < BLOCKING_MOVE_TIMEOUT_MS)
  {
    servicePowerButton(allowShutdown);
    s.moveTo(s.targetPos);
    moving = fabsf(s.getCurrentPos() - s.targetPos) <= closeEnough;
  }
}

// Return to the mode that started the choreography (pitchDrop or cardioidTrace).
// If that mode is SWIM, reset the swim start point from the given offset. This
// stops the fluid from jumping on the first move.
void resumeReturnMode(int16_t swimOffsetX, int16_t swimOffsetY)
{
  if (returnMode == Mode::SWIM)
  {
    swimX = JOY_CENTER + swimOffsetX;
    swimY = JOY_CENTER + swimOffsetY;
    swimHeading = atan2f(JOY_CENTER - swimY, JOY_CENTER - swimX);
  }
  currentMode = returnMode;
}

// It's important to park the magnet away from the ferrofluid before powering 
// off. If the ferrofluid is left attached to the magnet, the magnetic particles
// can be pulled out of solution.

void putItInPark()
{
  SERIAL_PRINTLN("parking");

  // attach first (attach holds the current position, so this never jumps). The PWM
  // signal stays alive for the whole park move — no torque interruption, no supply transient.
  attachAll();

  //Move the magnet to the bottom of the jar
  StepMove phase1[] = {
    { &leftServo,  0.0f,   0 },
    { &rightServo, 180.0f, millis() + SLOW_STAGGER_MS },
  };
  steppedMove(phase1, 2, false, SLOW_MOVE_DPS * 2.0f);
  delay(100);

  // Phase 2: retract the magnet.
  StepMove phase2[] = {
    { &magnetServo, (float)MAGNET_PARK_POS, 0 },
  };
  steppedMove(phase2, 1, false);
  delay(20);

  // Phase 3: Snap the arms upward to put the magnet at the top of the jar. This rapid
  // upward swing is what peels the ferrofluid off the magnet — the fluid's drag in the
  // surrounding water holds it at the bottom of the jar while the magnet yanks away.
  leftServo.write(180.0f);
  delay(100);   // this delay helps peel the ferrofluid off the magnet
  rightServo.write(0.0f);
  delay(500);

  // detach the servos
  leftServo.detach(); rightServo.detach(); magnetServo.detach();
  leftServo.targetPos = leftServo.getCurrentPos();
  rightServo.targetPos = rightServo.getCurrentPos();
  magnetServo.targetPos = magnetServo.getCurrentPos();

  magnetParked = true;
  currentMode = Mode::IDLE;       // once the system is parked, transitiion to the IDLE state
}




// Move to the bottom center of the jar and lower the magnet.
// This grabs the ferrofluid before a mode starts to move it.
// SWIM, DRAW, and DANCE call this on entry.
void pickUpFerrofluid()
{
  magnetServo.targetPos = MAGNET_MAX;
  calculateRotatedServoTargets(JOY_CENTER, JOY_CENTER + 500); // go to the bottom center

  // attach first (attach holds the current position — no jump), staggered to
  // flatten the inrush current
  attachAll(150);

  // constant-speed move to the pickup pose, staggered start
  StepMove moves[] = {
    { &leftServo,   leftServo.targetPos,  0 },
    { &rightServo,  rightServo.targetPos, millis() + SLOW_STAGGER_MS },
    { &magnetServo, (float)MAGNET_MAX,    millis() + 2 * SLOW_STAGGER_MS },
  };
  steppedMove(moves, 3, true);
  delay(250);

  // servos stay attached so the active mode's moveTo() drives them seamlessly
  leftServo.targetPos = leftServo.getCurrentPos();
  rightServo.targetPos = rightServo.getCurrentPos();
  magnetServo.targetPos = magnetServo.getCurrentPos();

  swimX = JOY_CENTER;
  swimY = JOY_CENTER + 500;
  swimHeading = atan2f(JOY_CENTER - swimY, JOY_CENTER - swimX);
}


// Fire2012 by Mark Kriegsman, July 2012
// as part of "Five Elements" shown here: http://youtu.be/knWiGsmgycY
//// 
// This basic one-dimensional 'fire' simulation works roughly as follows:
// There's a underlying array of 'heat' cells, that model the temperature
// at each point along the line.  Every cycle through the simulation, 
// four steps are performed:
//  1) All cells cool down a little bit, losing heat to the air
//  2) The heat from each cell drifts 'up' and diffuses a little
//  3) Sometimes randomly new 'sparks' of heat are added at the bottom
//  4) The heat from each cell is rendered as a color into the leds array
//     The heat-to-color mapping uses a black-body radiation approximation.
//
// Temperature is in arbitrary units from 0 (cold black) to 255 (white hot).
//
// This simulation scales it self a bit depending on NUM_LEDS; it should look
// "OK" on anywhere from 20 to 100 LEDs without too much tweaking. 
//
// I recommend running this simulation at anywhere from 30-100 frames per second,
// meaning an interframe delay of about 10-35 milliseconds.
//
// Looks best on a high-density LED setup (60+ pixels/meter).
//
//
// There are two main parameters you can play with to control the look and
// feel of your fire: COOLING (used in step 1 above), and SPARKING (used
// in step 3 above).
//

// NOTE: This is modified from the original Fire2012 function. It's all internally scoped,
// and I added a blurring effect to smooth things out a little bit.

void Fire2012()
{
  // Array of temperature readings at each simulation cell
  static uint8_t heat[NUM_LEDS];
  static bool gReverseDirection = false;
  constexpr uint32_t FRAMES_PER_SECOND = 8;

  static uint32_t fireTime = 0, blurTime = 0;

  // COOLING: How much does the air cool as it rises?
  // Less cooling = taller flames.  More cooling = shorter flames.
  // Default 50, suggested range 20-100 
  constexpr uint8_t COOLING = 20;

  // SPARKING: What chance (out of 255) is there that a new spark will be lit?
  // Higher chance = more roaring fire.  Lower chance = more flickery fire.
  // Default 120, suggested range 50-200.
  constexpr uint8_t SPARKING = 50;

  if (millis() - fireTime >= 1000 / FRAMES_PER_SECOND) 
  { 
    // Step 1.  Cool down every cell a little
    for( int i = 0; i < NUM_LEDS; i++) {
      heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / NUM_LEDS) + 2));
    }

    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    for( int k= NUM_LEDS - 1; k >= 2; k--) {
      heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
    }
    
    // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
    if( random8() < SPARKING ) {
      int y = random8(NUM_LEDS);
      heat[y] = qadd8( heat[y], random8(160,255) );
    }

    // Step 4.  Map from heat cells to LED colors
    for( int j = 0; j < NUM_LEDS; j++) {
      CRGB color = HeatColor( heat[j]);
      int pixelnumber;
      if( gReverseDirection ) {
        pixelnumber = (NUM_LEDS-1) - j;
      } else {
        pixelnumber = j;
      }
      leds[pixelnumber] = color;
    } 

    FastLED.show(); 
    fireTime = millis(); 
  }

  if (millis() - blurTime >= 100 / (FRAMES_PER_SECOND)) { blur1d(leds, NUM_LEDS, 120); FastLED.show(); blurTime = millis(); }
}


// Called when the FSM leaves the Dance state (after the dance's full duration).
// Restores the mode so the swim behavior resumes and allows a future greeting.
void leaveDance()
{
  danceState = DANCE_IDLE;
  currentMode = Mode::SWIM;
  SERIAL_PRINTLN("DANCE: back to swim");
}

// DANCE mode
void handleDancePage()
{
  constexpr float VERTICAL   = 0.5 * PI,
                  MIN_ANGLE  = (3 * PI) / 8,
                  MAX_ANGLE  = (5 * PI) / 8,
                  RESOLUTION = 0.01,       // Adjustment step for angleParameter and jumpParameter values.
                  FLOAT_ROUND_THRESHOLD = RESOLUTION / 2,
                  MIN_HEIGHT_SCALAR = 0.75,
                  MAX_HEIGHT_SCALAR = 1.5;
  
  constexpr uint32_t JUMP_PERIOD          = 500,                              // total time for a single jump to take
                     JUMP_UPDATE_INTERVAL = JUMP_PERIOD * RESOLUTION,         // milliseconds between jump position updates
                     ANGLE_CYCLE_PERIOD   = 5000,                             // total time for the angle to move through a full cycle
                     ANGLE_CYCLE_INTERVAL = ANGLE_CYCLE_PERIOD * RESOLUTION,  // update interval
                     HEIGHT_PERIOD        = 5100,                            // vary the jump height over time
                     HEIGHT_INTERVAL      = HEIGHT_PERIOD * RESOLUTION;       // update interval
  
  static uint32_t lastParamUpdate = 0, lastAngleUpdate = 0, lastHeightUpdate = 0;

  static float angleParameter = VERTICAL, // range [-PI, 0.0]. determines angle vector for jumping along. desmos t parameter
               jumpParameter = 0.0,       // range [0, 2PI]. determines height jumped to, allows for LERPing. desmos v parameter
               t = 0.0,                   // range [0, 1]. used to make driving jumpParameter easier.
               heightChangeScalar = 1.0;  // range [0, 1]. used to modify the jump height over time.

  static int8_t jumpDir = -1, angleDir = -1, jumpHeightChangeDir = 1;

  animateMainMenu(currentMenuSelection);
  updateMagnetPulse();

  if (newStateEntry)
  {
    angleParameter = VERTICAL;
    jumpParameter = 0.0;
    t = 0.0;
    jumpDir = -1;
    angleDir = -1;
  }

  if (millis() - lastHeightUpdate >= HEIGHT_INTERVAL)
  {
    if (heightChangeScalar <= MIN_HEIGHT_SCALAR + FLOAT_ROUND_THRESHOLD || heightChangeScalar >= MAX_HEIGHT_SCALAR - FLOAT_ROUND_THRESHOLD)   // floating point comparisons
    {
      jumpHeightChangeDir *= -1;
    }
    heightChangeScalar += RESOLUTION * jumpHeightChangeDir;
    lastHeightUpdate = millis();
    SERIAL_TABS(5);
    SERIAL_PRINTLN(heightChangeScalar);
  }

  if (millis() - lastAngleUpdate >= ANGLE_CYCLE_INTERVAL)
  {
    if (angleParameter <= MIN_ANGLE || angleParameter >= MAX_ANGLE) angleDir *= -1;
    angleParameter += RESOLUTION * angleDir;
    lastAngleUpdate = millis();
  }
      
  if (millis() - lastParamUpdate >= JUMP_UPDATE_INTERVAL)
  {
    if (t <= FLOAT_ROUND_THRESHOLD || t >= (1.0 - FLOAT_ROUND_THRESHOLD)) jumpDir *= -1;
    t += RESOLUTION * jumpDir;
    jumpParameter = 2 * PI * t;
    lastParamUpdate = millis();
  }
  // calculates distance traversed along vector. driving jumpParameter with a linear triangle 
  // wave will make the jumpHeight follow a sine way ranging between the desired bottom and top heights.
  // desmos u parameter.
  float jumpHeight = heightChangeScalar * (0.5 * sin(jumpParameter - (PI * 0.5)) + 0.5);

  // the radii of inner and outer circles from the center that bound the jump height (desmos R and R2)
  static int16_t outerRadius = 400, innerRadius = 100;

  // the X and Y coordinates of the endpoints of the line segment along which the jump point moves.
  float x1 = JOY_CENTER + outerRadius * cos(angleParameter), 
        y1 = JOY_CENTER + outerRadius * sin(angleParameter),
        x2 = JOY_CENTER + innerRadius * cos(angleParameter),
        y2 = JOY_CENTER + innerRadius * sin(angleParameter);
  
  // calculate slope of jump vector
  float slope = (y1 - y2) / (x1 - x2);

  // calculate final points
  int16_t xFinal = lroundf(((1 - jumpHeight) * x1) + (jumpHeight * x2)),
          yFinal = lroundf(((1 - jumpHeight) * y1) + (jumpHeight * y2));
  
  calculateRotatedServoTargets(xFinal, yFinal);

  leftServo.setSmoothing(0.99);
  rightServo.setSmoothing(0.99);

  if (newStateEntry)
  {
    attachArms(100);
  }

  leftServo.moveTo(leftServo.targetPos);
  rightServo.moveTo(rightServo.targetPos);

  if (joystickButton.isPressed()) {
    currentMode = Mode::IDLE;
  }
}

// PITCH_DROP choreography: spin the magnet in a circle, then drop it.
// Phase 1 moves to the circle start. Phase 2 traces the circle.
// Phase 3 moves up. Phase 4 retracts the magnet and holds.
// It then returns to the mode that started it.
void pitchDrop()
{
  constexpr int16_t ORBIT_RADIUS = 100;
  constexpr uint8_t CIRCLE_STEPS = 48; // ~7.5 degrees per step
  constexpr uint32_t MS_PER_STEP = 40; // 40ms × 48 steps ≈ 1.9s for full circle
  constexpr float CLOSE_ENOUGH = 0.5f;
  constexpr float APPROACH_SMOOTHING = 0.9999f;
  constexpr float CIRCLE_SMOOTHING = 0.6f;

  attachAll(100);   // attach arms + magnet; Phase 4 drives the magnet

  // Phase 1: move to circle entry point (angle 0, rightward from center)
  calculateRotatedServoTargets(JOY_CENTER + ORBIT_RADIUS, JOY_CENTER);
  leftServo.setSmoothing(APPROACH_SMOOTHING);
  rightServo.setSmoothing(APPROACH_SMOOTHING);
  blockingMoveArms(CLOSE_ENOUGH);

  // Phase 2: full circle at ORBIT_RADIUS, timed steps so the pacing is predictable
  leftServo.setSmoothing(CIRCLE_SMOOTHING);
  rightServo.setSmoothing(CIRCLE_SMOOTHING);
  for (uint8_t i = 0; i <= CIRCLE_STEPS; i++)
  {
    float angle = TWO_PI * i / CIRCLE_STEPS;
    calculateRotatedServoTargets(
        JOY_CENTER + lroundf(ORBIT_RADIUS * cosf(angle)),
        JOY_CENTER + lroundf(ORBIT_RADIUS * sinf(angle)));
    uint32_t stepStart = millis();
    while (millis() - stepStart < MS_PER_STEP)
    {
      servicePowerButton();
      leftServo.moveTo(leftServo.targetPos);
      rightServo.moveTo(rightServo.targetPos);
    }
  }

  // Phase 3: move to top center
  calculateRotatedServoTargets(JOY_CENTER, JOY_CENTER - 500);
  leftServo.setSmoothing(APPROACH_SMOOTHING);
  rightServo.setSmoothing(APPROACH_SMOOTHING);
  blockingMoveArms(CLOSE_ENOUGH);

  // Phase 4: retract magnet fully, then hold pose for 3 seconds
  magnetServo.targetPos = 0;
  magnetServo.setSmoothing(0.5f);
  blockingMoveServo(magnetServo, CLOSE_ENOUGH);
  delay(3000);

  // restore the current position fields so the resuming behavior
  // doesn't get a stale target on its first moveTo call
  leftServo.targetPos = leftServo.getCurrentPos();
  rightServo.targetPos = rightServo.getCurrentPos();

  resumeReturnMode(0, -500);
}


// CARDIOID choreography: trace a heart shape with the magnet.
// Phase 1 moves to the first point. Phase 2 traces the shape.
// At the end, the magnet parks. It then returns to the mode that started it.
void cardioidTrace()
{
  constexpr uint16_t NUM_CARDIOID_POINTS = sizeof(cardioidPoints) / sizeof(cardioidPoints[0]);
  constexpr uint32_t MS_PER_STEP = 25; // 25ms × 150 points ≈ 3.75s total
  constexpr float APPROACH_SMOOTHING = 0.9998f;
  constexpr float TRACE_SMOOTHING = 0.7f;
  constexpr float CLOSE_ENOUGH = 0.5f;

  delay (1000);

  attachAll(100);   // attach arms + magnet; the end of the routine parks the magnet

  // Phase 1: move smoothly to the first point before starting the trace
  calculateRotatedServoTargets(
      JOY_CENTER + cardioidPoints[0][0],
      JOY_CENTER + cardioidPoints[0][1]);
  leftServo.setSmoothing(APPROACH_SMOOTHING);
  rightServo.setSmoothing(APPROACH_SMOOTHING);
  blockingMoveArms(CLOSE_ENOUGH);
  delay(1000);

  // Phase 2: trace all points at a fixed pace
  leftServo.setSmoothing(TRACE_SMOOTHING);
  rightServo.setSmoothing(TRACE_SMOOTHING);
  for (uint16_t i = 0; i < NUM_CARDIOID_POINTS; i++)
  {
    calculateRotatedServoTargets(
        JOY_CENTER + cardioidPoints[i][0],
        JOY_CENTER + cardioidPoints[i][1]);
    uint32_t stepStart = millis();
    while (millis() - stepStart < MS_PER_STEP)
    {
      servicePowerButton();
      leftServo.moveTo(leftServo.targetPos);
      rightServo.moveTo(rightServo.targetPos);
    }
  }

  // restore target state so the resuming behavior doesn't jump on its first moveTo
  leftServo.targetPos = leftServo.getCurrentPos();
  rightServo.targetPos = rightServo.getCurrentPos();

  resumeReturnMode(cardioidPoints[NUM_CARDIOID_POINTS - 1][0],
                   cardioidPoints[NUM_CARDIOID_POINTS - 1][1]);

  delay(1000);

  magnetServo.targetPos = 0;
  magnetServo.write(magnetServo.targetPos);   // magnet is attached (via attachAll), so this parks it

  delay (1000);
}

// generate random number in range using hardware RNG
int32_t randomInRange(int32_t min, int32_t max) {
  return min + (esp_random() % (max - min + 1));
}