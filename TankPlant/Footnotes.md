Footnotes.md

# Footnote 1: defining a variable inside a switch-case
Normally it's bad practice to define a variable inside a switch case. For example, the following code block is not the way this should normally be done:

```c
int variable = 0;
// variable gets changed throughout course of program, then we encounter switch
switch (variable) {
    case 0:
        // do something 
        break;

    case 1: 
        // let's say we want to track how many times we have encountered case 1
        static int case1Count = 0;
        case1Count++;
        break;

    default:
        break;
}
```

The above block defined case1Count inside case 1 of the switch statement. This is considered bad practice because the lifetime of that variable gets
scoped to the entire switch statement, not just case 1. However, that variable never shows up in the other cases, and this can lead to the code behaving
strangely. The thing that is annoying is that this will compile, and often it will run and even work like you intend. You will probably get a warning from the 
compiler though, and it will probably say something like "jump to case label permissive." If you see this warning, check to see if you defined a variable
inside a case block. 

Again, sometimes you can do this and your code will run fine, but other times it will make your switch statements or other bits of 
code behave strangely. There are a few ways you can solve this problem. The first is just to move that variable outside the switch statement:

```c
int variable = 0;
int case1Count = 0;         // move this outside the scope of the switch statement
// variable gets changed throughout course of program, then we encounter switch
switch (variable) {
    case 0:
        // do something 
        break;

    case 1: 
        // let's say we want to track how many times we have encountered case 1
        case1Count++;
        break;

    default:
        break;
}
```

You can also solve this by creating a new scope inside the case where you want to define that variable. Every time you use curly braces {},
you create a new scope. Any variables defined inside that scope will only exist within that scope, and this will resolve the "jump to case permissive"
warning the compiler would otherwise throw:

```c
int variable = 0;
int case1Count = 0;         // move this outside the scope of the switch statement
// variable gets changed throughout course of program, then we encounter switch
switch (variable) {
    case 0:
        // do something 
        break;

    case 1: {       // NOTICE THE CURLY BRACES CREATING A NEW SCOPE HERE
        // let's say we want to track how many times we have encountered case 1
        case1Count++;
        break;
    }

    default:
        break;
}
```

This last option is why it is ok to define a new variable (int8_t direction) inside the case block that made me want to write this footnote. 
The variable "direction" is a while loop within the case block. The while loop uses curly braces, which creates a new scope and makes it 
ok to create a new variable:

```c
case BehaviorModes::RANDOMIZE_POSITION:
    static uint16_t spinTime = 0, driveTime = 0, randomizeStep = 0;
    static elapsedMillis randomizeSpinTimer = 0, randomizeDriveTimer = 0;

    face.setFaceState(FaceStates::EYES_CONFUSED);

    if (tankState.lastBehaviorState != BehaviorModes::RANDOMIZE_POSITION) {
        tankState.lastBehaviorState = tankState.behaviorState;    // we detected the state transition, now update the last state with the current one
        spinTime = random(POSITION_RANDOMIZE_SPIN_MIN, POSITION_RANDOMIZE_SPIN_MAX);       // randomize spin time, settable in RobotBehaviorParameters.h
        driveTime = random(POSITION_RANDOMIZE_DRIVE_MIN, POSITION_RANDOMIZE_DRIVE_MAX);     // randomize drive forward time, settable in RobotBehaviorParameters.h
        randomizeStep = 0;
        randomizeDriveTimer = 0;
        randomizeSpinTimer = 0;

        // this while loop creates a new scope. This makes it ok to define a new variable, even though it's inside a case block within a switch statement.
        while (heading > 2 || heading < -2) {
            int8_t direction = heading > 0 ? -1 : 1;      // Defining this variable is fine now.
            currentServoPos = runServoAtSpeed(headServo, direction * 30);
            heading = convertServoAngleToHeading(currentServoPos);
        }
        // now that the head is close to forward, we can set it direction to a heading of 0 and then move on to randomizing position
        heading = 0;      // face the head forward
        currentServoPos = convertHeadingToServoAngle(heading);
        headServo.write(currentServoPos);
    }
    //////////// case code continues after this...
```


# Footnote 2: How the robot parks in sunlight
This part is a bit hard to explain, but it's where the magic happens. It might seem like the plant pot head turns left and right to try to point straight
at the brightest light source, but that's not quite what it does. Instead, it just tries to balance out the light level readings between the left and 
right eyes. If the right eye seems more light, the head turns toward the right until the right and left eye see roughly the same level of light. The system
isn't programmed to turn the head a certain number of degrees to the right though. Instead, it tells the servo to move at a certain speed without paying
attention to its exact angle. If the right light sensors sees just a little more light than the left, the speed the servo spins at is really slow. If the
right eye sees a lot more light than the left, then the speed the servo spins at is set to be much faster. This turns the head more aggressively. This is
called a proportional controller: the speed that the head turns is proportional to the difference between the light level readings of the two eyes.

The angle that the head is pointing directly controls the way the tank tread motors drive the robot, so if the head is looking a little bit to the right, 
the left motor spins a bit faster than the right, and this drives the robot forward and to the right. This points the whole robot in the direction of 
brighter light. If the head is pointing really far to the right, the robot actually spins in place to try to point itself at the light source quickly, and
this is where the dynamics of the robot come into play. When the robot finds a bright patch of light, it will often start turning the head more and more to
the right or the left. If this is a patch of sunlight on the floor, there is often also a patch of shadow that falls on one of the eyes that exacerbates this
effect. Eventually, the robot spins in place trying to quickly bring the whole robot into alignment with the head, so the head will point forward. 
This usually happens under brighter sources of light, and this also means that the robot is being strongly illuminated from above, so in that case, both
light sensors will usually read similar light readings. The proportional controller and the servo motor can't move the head quickly in response to subtle 
changes in light levels, which means that the head isn't being driven to turn back to center, and it stays turned to either side. This serves
as a proxy measurement that indicates that the robot has found a nice bright patch of light to sunbathe in! All we have to do now is watch to see if the robot
is spinning in circles, and if it has been doing that for longer than a threshold amount of time, transition into the PARK state to sunbathe.

# Footnote 3: Notes about measure water levels
In one test where I started with totally dry LECA and added water to it, the plant quickly rose across 
the 35 threshold for DROWNING, and then stayed in the DROWNING condition for nearly 18 minutes while the
water diffused through the LECA. After that, it had dropped below the threshold and was fine.

The next test to do is to completely fill the plant pot with water while it has the LECA and plant in
the pot. That's an actual drowning condition for the plant. I put 100mL of water in the pot, which pretty
completely fills it. I made sure that the leads of the JST connector are not in the water. I very quickly
achieved sensor readings that stabilized around 205. So I'll treat 200 as my "we're literally drowning"
condition.

Next, I drained all the water to see what fully saturated LECA reads. It's stabilized around 63, so I 
will set 65 as a threshold for the upper bound of fully saturated now.

I think the next series of tests would be to go past fully saturated. Add like 25mL at a time back up
to the full 100mL and see what each of those reads. Maybe I can distinguish a bit of excess water sitting
in the pot from fully swimming in it.

Saturated + 25mL: somewhere around 130. Stabilized after hours around 120.

Saturated + 50mL:

Saturated + 75mL:

Saturated + 100mL:

# Footnote 4: How the LED matrix control system works
There are several systems in code that interact with each other to draw faces onto the LED matrix. First, at the lowest level, we have the
Adafruit_IS31FL3731 library that handles the basic functions of LED matrix. This provides methods for doing things like lighting up individual
pixels, drawing lines and other basic shapes, writing text, and drawing a bitmap on the matrix, which use a lot. 

Next, the Face class from the MatrixFace.h library handles the higher level functions of drawing faces and moving face bitmaps from microcontroller 
memory into the 8 frame buffers of the IS31FL3731, which is the controller IC on the PCB that actually drives the matrix. The IS31FL3731 actually
has onboard memory that can store 8 complete frames of pixel information, and we use this to store the bitmap images for each face and speed up
processing. By storing the face images in the frame buffers, when we want to change the face that is drawn on the matrix, we can just send a single
command to the controller that switches the frame it's currently displaying, rather than having to send 144 bytes of raw pixel brightness data 
over the I2C bus. This saves a lot of processing time and frees up the main microcontroller for other tasks. The Face class provides methods like

```cpp
face.storeImagesInFrames();   // copies the face bitmaps from microcontroller memory into the IS31FL3731 LED matrix controller chip (8 available frames for storage in IS31FL3731)
face.setFaceState(FaceStates::SMILING_FACE);   // set which face will be drawn first
face.updateFace();                            // draw the face
```

for performing these tasks. Finally, the actual face bitmaps are declared as arrays in FaceBitmaps.h, and they are defined in FaceBitmaps.cpp. In general,
these are the files you need to change if you want to modify the face designs that get displayed on the matrix. Each face pattern is a uint8_t array
that is 144 elements long, so each LED in the matrix is represented by a single byte that defines the brightness of that LED. These are listed in order
from left to right and top to bottom. There is a tool in the right sidebar of the IDE that lets you draw face bitmaps and convert them to C arrays in the
proper format, so you can design a new face and place it in the FaceBitmaps files to make it available for drawing on the matrix!

***I should probably add a bit more detail about how to build out the array of bitmaps that will get stored in the framebuffer, and also talk about how
you don't have to use the framebuffers and can also just write directly over the i2c bus.***

# Footnote 5: Be careful about how you compare floating point numbers
Note that speedVector and lastSpeedVector are floats, and normally you can't do this type of direct equality (or inequality) 
comparison between two different floating point numbers because floating point rounding error will almost always make 
something like (1.0 == 0.5 + 0.5) resolve to false, even though mathematically, it should be true. However, in this case, 
we're comparing speedVector to a direct copy of a previous version of itself, so we can use a direct comparison. There are a 
lot of good resources to get into the weeds on floating point error, and this is one such:
https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/

# Footnote 6: The static keyword and locality of scope

## locality of scope. 
Dave's Garage has [a good video on common C mistakes](https://www.youtube.com/watch?v=LLykqRNuEwA&t=448s) that gets into this.
The first mistake shown is the the one I'm talking about. Somewhere around 7:00 in the video.

In general, you want to keep variables as tightly scoped as possible. In the Dave's garage example, the iteration variable
`i` only needed to exist inside the for loop, and declaring it outside that allowed for mistakes later on when the value was
referenced. Keeping the `i` variable scoped to the for loop solves this problem. This is why it's generally a good idea to avoid
global variables when possible. Not something I'm perfect at, and for relatively short programs like the ones we create for Hack
Pack, you can get away with using a lot of global variables because it's not too onerous of a task to find all the places they
are referenced in the code. But for larger projects, this could cause hard to detect problems, and it's good practice to minimize
the scope of your variables whenever possible.

## static keyword
The static keyword ensures that a variable is only defined and initialized once. We're declaring some variables inside of
loop(), which runs forever, like a while() loop. As an example:

```c
while (1) {
    int num = 0;
    Serial.println(num);
    num++;
}
```

The above block of code will always print 0 to the Serial monitor. That's because each time the while loop starts, it redefines
the variable num as 0. Then it prints the current value of num to the Serial monitor, which is 0. The third line increments
num by 1, so at the end of the while loop, num is 1. But when the while loop starts over, num will be redefined to 0.

If we just add the static keyword, we can ensure that num will only be defined and initialized the first time that line of code
gets executed, but the variable will remain in memory for all subsequent executions.

```c
while (1) {
    static int num = 0;
    Serial.println(num);
    num++;
}
```

This block of code will print out the natural numbers indefinitely, starting from 0 on the first iteration of the while loop, 
then printing 1 in the second iteration of the loop, then 2, and so on.


# Footnote 7: Notes on calibrating the moisture sensor readings
The threshold moisture values that can be set in Configuration.h were set after a fair amount of experimentation, but may need to be changed
depending on the type of plant you want to grow. I performed tests by soaking LECA in water for a while, then draining it, putting it into the 
pot, and recording the moisture reading from the sensor. Note that the sensor several minutes to stabilize after sudden changes in water levels.
This reading (plus a bit extra) was used as the SATISFIED upper bound value since this represents the most water that the LECA can hold without
extra water pooling in the bottom of the pot. The values for PARCHED and THIRSTY were a bit more of a vibes thing, and based on letting the 
LECA dry out for various amounts of time and getting a sense of what was partially saturated or fully dried out. 

If you want to do this more scientifically, dry the LECA in a dehydrator or oven on the lowest temperature setting for several hours, and then 
weigh the LECA with a scale. This is the dry weight of the LECA. Then soak the LECA in water for a while and then drain it fully, immediately 
weighing the fully saturated LECA. Subtract the dry weight of the LECA, and you'll know how much water was absorbed by weight. Then let it dry 
out in open air for a day or so, measuring the weight of the LECA and the moisture reading from the sensor every few hours. Plot this is in a 
spreadsheet, and you'll have a correlation between sensor reading and actual amount of water stored by the LECA. 

If you want to grow a real plant in this, you can! People grow plants directly in LECA as a soilless growth medium. This requires a bit of extra
care and liquid fertilizer, but this is a way that people grow house plants. You can also change out the LECA for a different growth medium,
like soil or coconut coir. Just note that the moisture sensor readings will probably need to be adapted to the new medium. 

# Footnote 8: Structs, enums, and enum classes

We’ve used `struct`s and `enum`s in several past projects—like **Label Maker**, **Sand Garden**, and **Card Dealing Robot**—to help organize 
related data in a clean and understandable way. In this project, we’re introducing a new concept: `enum class`. Understanding the differences 
between these types can help you write code that’s easier to read, maintain, and debug.

## `struct`s: Grouping Related Data

A `struct` (short for “structure”) lets you combine multiple variables into one. You can think of it like a container for related pieces of data 
that belong together. In this project, we use the following `struct` to organize all of the information related to the robot's current state:

```cpp
// This struct stores and organizes the state information of the tank chassis.
struct RobotStateContainer {
  BehaviorModes behaviorState;            // current state
  BehaviorModes lastBehaviorState;        // last behavior state
  unsigned long retreatInterval;
  unsigned long retreatRotateInterval;
  elapsedMillis retreatTimer;
  elapsedMillis retreatRotateTimer;
  bool retreatInitiated;
  bool retreatRotateInitiated;

  // Constructor to initialize members. When a new instance is created, it initializes to these values.
  RobotStateContainer()
      : behaviorState(BehaviorModes::CALIBRATE),             // the behavior mode the robot will be in when it powers on
        lastBehaviorState(BehaviorModes::CALIBRATE),
        retreatInterval(TANK_RETREAT_INTERVAL),              // how long to retreat. Configuration.h
        retreatRotateInterval(TANK_RETREAT_ROTATE_INTERVAL), // if doing STRAIGHT retreat, how long to rotate away from object. Configuration.h
        retreatTimer(0),                                     // for tracking retreat moves - might remove
        retreatRotateTimer(0),                               // for tracking retreat moves - might remove
        retreatInitiated(false),                             // for tracking retreat moves
        retreatRotateInitiated(false)                        // for tracking retreat moves
  {}
};

// Create an instance of the struct
RobotStateContainer robotState; // initialized to the above values
```

This `struct` encapsulates many different aspects of the robot's behavior state and timers, making it easier to manage in a centralized and 
coherent way. It also simplifies function interfaces because you can pass or return the entire state container rather than dealing with each member 
individually.

## `enum`: Creating Named Constant Values

An `enum` (short for “enumeration”) is a way to define a set of named constant values. These are often used when a variable should only be allowed 
to take on a small number of specific, predefined values. For example:

```cpp
enum BehaviorModes {
    PARK,
    SEEK,
    CALIBRATE,
    TEST,
    RANDOMIZE_POSITION
};
```

Each name in the `enum` corresponds to a number (starting at 0 by default), so `PARK` is 0, `SEEK` is 1, and so on. This works well when you need 
simple labels for different modes or states.

However, one downside of traditional `enum`s is that **they share the global namespace and implicitly convert to integers**. A *namespace* is like 
a named container that keeps identifiers (like variable names, function names, or enum values) separate from each other. When something is in the 
global namespace, it means it's available everywhere, which can lead to name conflicts if two enums (or other variables) define the same name. 
That means:

* You can accidentally mix up values from different enums if they happen to have the same name or value.
* You can accidentally compare or assign arbitrary integers to an `enum`-typed variable, which might compile but produce incorrect behavior.

## `enum class`: Scoped and Safer Enums

`enum class`—also called a **scoped enum**—solves these problems by keeping the enumeration values inside a specific scope. That way, you must specify 
which enum a value belongs to when using it, which reduces accidental mistakes and improves code clarity.

Here’s how we define a scoped enum:

```cpp
enum class BehaviorModes {
    PARK,              // disables movement
    SEEK,              // drives toward light
    CALIBRATE,         // currently unused
    TEST,              // currently unused
    RANDOMIZE_POSITION // drives to a new random location
};
```

And here’s how we use it in a `switch` statement:

```cpp
switch (robotState.behaviorState) {
    case BehaviorModes::CALIBRATE:
        // do something
        break;
}
```

Note the use of `BehaviorModes::CALIBRATE`. This is required because `CALIBRATE` is scoped within the `BehaviorModes` enum class. This makes the source 
of the value explicit and helps avoid name conflicts.

## Why Use `enum class` Instead of `enum`?

Scoped enums (`enum class`) are considered **better practice** than plain enums for several reasons:

* **Strong typing**: You can’t accidentally assign an unrelated integer value to a variable of an `enum class` type.

  ```cpp
  BehaviorModes mode = BehaviorModes::CALIBRATE; // OK
  mode = 2; // ❌ Error: can’t assign int to BehaviorModes
  ```

* **Clearer syntax**: You always know where a value came from because of the `EnumName::VALUE` syntax.

* **Avoids name collisions**: With plain enums, all values go into the global scope. With `enum class`, the values are scoped, so you can have 
multiple enums with overlapping names:

  ```cpp
  enum class Direction { LEFT, RIGHT };
  enum class Turn { LEFT, RIGHT }; // ✅ OK: no collision
  ```

* **Safer comparisons**: The compiler won’t let you compare values from two different `enum class` types, which helps catch logic bugs.

In contrast, plain enums can be used in situations where you need to interoperate with C code, or if you need the enum to implicitly act as an 
integer—for example, as a bitmask or flag. But in modern C++, using `enum class` is usually the right choice.

## Summary

* Use a **`struct`** to group related variables together—ideal for representing things like a robot’s state.
* Use an **`enum`** (or preferably an **`enum class`**) to define a set of constant values that represent distinct options or modes.
* Prefer **`enum class`** when you want safer, clearer, and more maintainable code—especially as your projects get bigger and more complex.

You’ll continue to see (and use) the `::` syntax in many places, such as when accessing static variables, enum values, or class methods from another 
file. It’s a sign of well-organized and scoped code.




# Footnote 9: Pseudorandom Number Generators and `randomSeed()`

When we use the `random()` function in Arduino code, we're not actually generating truly random numbers—we're generating **pseudorandom numbers**. 
A pseudorandom number generator (PRNG) is an algorithm that produces a sequence of numbers that *appear* random but are actually determined by an 
initial input called a **seed**. Basically, a call to `random()` takes the seed number and performs a series of math operations on it to produce
an output number that looks random, but which is actually entirely predictable. For example, try running the example code below:
```cpp
void setup() {
  Serial.begin(115200);
  randomSeed(0);
  for (uint8_t i = 0; i < 10; i++) {
    Serial.println(random(0, 100));
  }
}

void loop() {
}
```

You'll always get the exact same sequence of 10 integers. The numbers you get are hard to predict, but they're always the same order. If you change
the random seed, then you'll get a different sequence. So on microcontrollers that don't have a hardware true random number generator, like the
ATMega328 microcontroller, if you never call `randomSeed()`, the PRNG will always start with the same default seed, so `random()` will generate the 
same sequence of numbers every time the program runs. That might be useful for debugging, but it's not desirable if you want unpredictable behavior, 
like varying a robot's movement path each time it turns on.

To introduce some real-world unpredictability, we often seed the generator with a number that is hard to predict. This line of code does exactly that:

```cpp
randomSeed(analogRead(A7));
```

Here, `analogRead(A7)` reads the voltage from analog pin A7, which is intentionally left **floating**—meaning it is not connected to anything. A floating analog input will usually return a somewhat random value based on electrical noise and interference, making it a decent entropy source for seeding the PRNG.

Once seeded, every call to `random()` will generate a new value in the sequence, giving the appearance of randomness. If you power cycle the Arduino and `randomSeed()` uses a new noisy input value, the sequence will change—making the robot behavior less predictable and more interesting.


# Footnote 10: The LED Matrix Controller IC

The LED matrix is controlled by the IS31FL3731 IC that is on the PCB. This allows us to connect to the LED matrix and draw on it using commands sent over I2C. This basic functionality is handled by the Adafruit_IS31FL3731 library. This provides functions that let you illuminate individual pixels, draw lines, or even copy entire 8 bit grayscale bitmap images onto the matrix.

The LED matrix itself is 16 LEDs wide and 9 pixels tall for 144 total LEDs. They are connected in what is known as a charlieplex array, which is a way of controlling a lot of LEDs with just a few IO pins from a controller. It’s a slightly complex control scheme, but fortunately, you don’t have to worry about handling that, since all of that is managed by the controller IC.  

We still need to be able to draw faces on the matrix, so I created the MatrixFace library to handle this. 
The code creates an instance of the Face class, which takes care of drawing the faces on the LED matrix. The Face class is derived from the MatrixFace.h header file, which we included at the top of main.cpp. If you open MatrixFace.h, you’ll see that it includes a few of its own files as well. The most important of these is FaceBitmaps.h, which is where the designs for each face are declared as arrays. You can find the definition for each face array in FaceBitmaps.cpp. This is where you can change the shape of each face that gets drawn on the LED matrix.

The arrays that define the face bitmaps are just brightness values for each pixel on the LED matrix. Each array in FaceBitmaps.cpp has 144 elements, with each value defining a brightness level for one pixel. They’ve been formatted to be in a 16x9 grid to roughly show the shape of the face. A bitmap is just a direct mapping of the value of each pixel in an array. Each pixel can have a brightness value between 0 and 255, which fits within the uint8_t data type. You can create new images to draw on the matrix by directly editing the values in an array,
by using the drawing commands made available by the Adafruit_IS31FL3731 library, or by drawing new designs in the bitmap editor tool in the sidebar of the IDE and copying the data into the an array in FaceBitmaps.cpp.

One important thing to know is that the integrated circuit that controls the LED matrix (the IS31FL3731) has onboard memory for storing 8 frames of animation. This is really helpful for reducing the processing load on the ATMega328 microcontroller. In setup(), the face.storeImagesInFrames() command looks in FaceBitmaps.cpp and stores each face bitmap array that is defined there in one of the frames on the LED matrix controller. This means that later in the code, when we want to change what face the LED matrix is displaying, we don’t have to spend processing time writing the value of each pixel to the LED matrix. Instead we just send a single command to the LED matrix controller to tell it which frame to display, and it takes care of the rest. This actually saves a substantial amount of time and eliminates a bunch of lag, since otherwise we would have to send 144 bytes of pixel brightness data over the I2C bus to the LED matrix, and this actually takes a pretty long time. This has another benefit, which is that we don't have to store the face bitmaps in RAM. Instead they are stored in the flash program memory (PROGMEM) where your program code is stored. This frees up valuable and limited RAM for other uses.