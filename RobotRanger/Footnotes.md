# Footnote 1: Time of Flight Sensor

The **VL53L1X** is a sensor developed by ST Microelectronics that uses the principle of time of flight. This is a way to **measure distance by sending out a wave** — like radio, sound, or light — then timing how long it takes for the wave to bounce back after hitting a surface. By knowing the wave’s speed and the travel time, you can figure out how far away something is. In the sensor's datasheet, you can see all the various settings used to tune it, as well as its performance at various light and distance levels. 

In the hardware initialization, we have chosen default "settings" for the sensor that have proven the best performance of it. It usually senses in a cone, so we have narrowed the cone to improve its distinction between different points. We have also traded accuracy for speed just a little bit, increasing the rate at which it sends pulses of light. 

```cpp
robot.ranger.setDistanceMode(VL53L1X::Short);
robot.ranger.setMeasurementTimingBudget(TIMING_BUDGET);
robot.ranger.setROISize(4, 4);
robot.ranger.setROICenter(199);
```


ToF sensors are hidden everywhere, from the autofocus of cameras to the object detection on cleaning robots. If any product needs to know how far something is to it, there is almost certainly a time of flight sensor helping with that. 

# Footnote 2: Button Handling & Resistor Ladder

When you're building a control panel, you often run into a frustrating limit: not enough input pins for all your buttons. A **resistor ladder** solves this by chaining resistors together and tapping each button off at a different point, so every button produces a unique voltage when pressed. Think of it like stacking voltage dividers. The microcontroller reads that single wire, checks the voltage level, and figures out which button it was. Thus, you have **multiple inputs for one single analog output**. 

```cpp
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
```

The tradeoff is that it only works cleanly for one button at a time — two buttons pressed simultaneously produce an ambiguous voltage that's hard to interpret. But for menu navigation, mode switching, or any interface where you're pressing one thing at a time, it's a elegant way to do more with less, freeing up your pins for other actuators and sensors. And in Hack Pack, pins are a scarce resource!

Real-world examples of resistor ladders include vehicle steering wheel controls, elevator panels, TV remotes, and other consumer electronics with low part count requirements. There may be additional parts or software logic to handle misuse or multiple presses, but the underlying mechanism is fundamentally the same. 

# Footnote 3: EEPROM & Calibration

EEPROM is **non-volatile memory**, meaning it survives power cycling (on and off). While other variables are stored in RAM, typically created and written to during a program's run time, EEPROM variables are specially stored, and have their own functions to read and write them. Think of it as the microcontroller's long-term notepad, while regular variables are temporary post-it notes. 

Because this memory is specially reserved, there is not a lot of it (~ 1KB on the HP microcontroller) and it only has about 100,000 write cycles before the memory degrades. That sounds like a lot, but a microcontroller running at 16MHz could burn through that in under a second if you wrote EEPROM in a typical loop.

Since you will only be calibrating the robot values a few times (hopefully) over the life span of the toy, we are using EEPROM to store these important values, being careful to write only once per relevant user action. The non-volatility helps each robot retain its unique calibration profile, as it may vary between units. 

```cpp
void checkEEPROM()
{
  bool tiltFlag = true;
  bool yawFlag = true;
  EEPROM.get(calibrateAddressTilt, tiltFlag);
  EEPROM.get(calibrateAddressYaw, yawFlag);
  if (!tiltFlag)
  { // EEPROM is 255 by default which is true, so actually false is the calibrated flag
    EEPROM.get(tiltAddress, robot.TILT_ADJUST);
  }
  if (!yawFlag)
  {
    EEPROM.get(yawOffsetAddress, robot.YAW_OFFSET_MIN);
    robot.YAW_OFFSET_MAX = robot.YAW_OFFSET_MIN + robot.YAW_EXTRA;
  }
}
```


# Footnote 4: Robot Geometry Constants

We measured the robot's physical dimensions (cross-checking CAD and real-world measurements) to accurately model projectile motion. Key variables include the launch height (which changes with angle) and barrel length, alongside constants like gravity and cup size — all used to improve trajectory predictions. If any one of these is slightly off, it can have a drastic effect on where the robot thinks it is shooting vs where it actually is. 

```cpp
#define MAXD 500          // max distance 500mm or ~ 1.5 ft - derived from ToF sensor resolution reliability at distance
#define GEAR_RATIO 4      // pitch gear to launcher ratio
#define cupR 35.0         // cup radius in mm
#define baseR 90.0        // base radius in mm
#define launcherR 115.0   // launcher radius arm in mm
#define sensorOffset 1.0  // sensor offset from front of robot in mm
#define barrelAdjust 7.75 // barrel adjust in mm
#define g 9810.0          // mm/s^2
#define h 70.0            // top of cup to center of launcher in mm
```

# Footnote 5: Objects, Mode Class, Virtual Functions, Polymorphism

**[DISCLAIMER]** This section is only scratching the very surface of the topic: **Object Oriented Programming**

This methodology is a way to organize code around objects — self-contained units that know what they are and what they can do. Rather than one long messy script, you combine reusable pieces that interact with each other. The key in OOP is how adaptable it is, using foundational building blocks to create complex systems. Objects are often used in conjunction with state machines, which you've learned about previously. There is no one perfect code methodology and often these structures work in tandem with each other.

To understand objects, we can use an analogy of a vending machine. Think of a vending machine as an object. Regardless of what it sells, these work the same way from the outside — insert money, make a selection, get your item. But a snack machine and a soda machine handle those steps differently on the inside. You don't need to know the internals to use either one. OOP is the same, where you know each "object" (vending machine) can be interfaced with in similar ways, but the content can vary based on specific needs.

**Mode Class:** The different robot/game "modes" are a form of something called a "class" which is a blueprint for an object. Think of this as a template that defines what every game mode must be able to do (initialize, run, launch, end, etc.). You can't use it directly as it just sets the rules that every specific mode has to follow.

**Virtual Functions:** These are the spots in the blueprint where each mode gets to fill in its own behavior. The word "virtual" is basically the code saying "don't assume, wait and see which mode is actually running before deciding what to do here." Each game mode overrides these with its own specific logic. For example, back to our robot, the class has the function "runStateMachine()" but each mode (an object) has a different behavior, whether it is autoaiming or manual control. Every object can call "runStateMachine()" by name, and thus the robot adapts that function based on the current mode we're in. In our earlier vending machine analogy, you could think of "dispenseItem()" as the virtual function, with each type of vending machine (an object) calling dispenseItem() to give out the relevant item (soda, snacks, toys). 

```cpp
class Mode
{
public:
    virtual void enter() = 0;
    virtual void exit() = 0;
    virtual void runStateMachine() = 0;
    virtual const char *name() = 0;
    virtual ~Mode() {}

protected:
    virtual void setColor(CRGB color) = 0;
};
```

**Polymorphism:** This is the concept that brings it all together. Since every mode follows the same blueprint, your robot's main loop can treat all modes identically — it just says "run the current mode" without needing to know whether that's autonomous mode, manual mode, or anything else. Swap the mode out, and the same code works automatically. It's what keeps your main logic clean and lets you add new modes later without breaking anything. To illustrate this imagine your main code is just object.runAction() and something that allows you to switch between objects. Now instead of having one large code file, you've broken up the implementation across each object file, with the main code being really clean and traceable. runAction() is always called, but behaves differently based on the object you're pointing to. The additional benefit is you don't need to restructure the main code when adding more objects. In double or triple nested state machines, you would need a lot of code refactoring to append or remove a new state. 

# Footnote 6: Mode Manager Class & Pointers

From Footnote 5, we learned that each mode knows what to do, but something needs to decide which mode is active and switch between them. That's the mode manager's job, and it utilizes something called a **pointer.** Think of a pointer like a sign that says "this way." It doesn't contain anything itself, it just points to where the thing is. You can pick up the sign and point it somewhere else at any time without touching what it was pointing to before. This keeps the destination (object) safe from accidental changes, which could happen if instead of pointing to a place we actually stepped foot in said place. The mode manager class holds a pointer to the current mode. When it's time to switch — say, from auto to manual mode — the manager calls exit() on the current mode, then points to the new one and calls enter(). The rest of the robot's code never changes; it just keeps talking to whatever the pointer is aimed at. This is what makes the polymorphism from before so useful in practice: a pointer is the mechanism that lets you swap objects while the main loop stays efficient. 

```cpp
  void runStateMachine()
    {
        if (currentMode() != nullptr)
        {
            currentMode()->runStateMachine(); //Points to the current mode's state machine.
        }
    }
```

**Overall, object oriented programming is a powerful way to organize code to be flexible and manageable while maintaining all the desired complexities and features.**