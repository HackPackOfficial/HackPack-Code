//Key UI variables------------------------------------------
uint8_t bootSize = 0;      // [0 - 2] 0 - small, 1 - medium, 2 - large
uint8_t initCursorPos = 6; // [0 - 7] Boot onto image selction

uint8_t penStyle = 3;     // [0 - 3] Default pen style: dist, time, dotted, chance

float manualXOffset = 0;  // [0 - 100] Manual image placement adjustment. 
float manualYOffset = 0;  // [0 - 100] Positive Y moves image down. Units in mm

//Artboard & image params-------------------------------------
double timeStep = 0.05;   // [0.01 - 0.1] Step size of parametric variable.

//Drawing Related Variables-----------------------------------
float penSmoothFactor = 0.05;  // [0 - 0.5], complementary filter for servo pos

float epsilon = 0.3;      // [0.1 - 1] tolerated error (mm) when moving to pos

uint8_t penUpPos = 70;    // [60 - 90] pen position presets
uint8_t penDownPos = 0;   // [0 - 10]
uint8_t penStartPos = 30; // [20 - 40]

float penUnit = 0.3;           // minimum movement needed to update pen position
float dps = 3.5;               // [0 - 5] dots per second (max) of pen
int penRes = 500;              // [100 - 750] time (ms) of each pen "pixel".
int dotTime = 150;             // [50 - 300] time it takes for pen to place dot
uint8_t penHold = 35;          // [0 - 100] how long pendown persits after a dot

//Motor Variables---------------------------------------------
uint8_t sampleTime = 5;         // [5 - 200] how often (ms) encoders are updated

uint8_t drawSpeed = 190;        // [0 - 255]
uint8_t motorTravelSpeed = 255; // [0 - 255]
uint8_t motorCoastDist = 3;     // [0 - 5] within dist mm we reduce motor speed
float epsilonScaling = 0.4;     // [0.1 - 0.9] scale motor speed near target

bool flipRightMotor = false;  
bool flipLeftMotor = false;

bool flipRightEncoder = false;  // CCW is positive
bool flipLeftEncoder = true;