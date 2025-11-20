//Key UI variables------------------------------------------
uint16_t boardSize = 300;     // [250 - 350] size of tic tac toe square

//Artboard & image params-------------------------------------
double timeStep = 0.1;        // [0.05 - 0.5] Step size of parametric variable.

//Drawing Related Variables-----------------------------------
float epsilon = 0.5;          // [0 - 1] tolerated error (mm) when moving to pos

uint8_t penUpPos = 70;        // [50 - 90] pen position presets
uint8_t penDownPos = 0;       // [0 - 10]
uint8_t penStartPos = 30;     // [10 - 40]

int penRes = 500;             // [100 - 1000] time (ms) held for each pen pixel.
int dotTime = 150;            // [0 - 500] time (ms) it takes for pen to place a dot
//Motor Variables---------------------------------------------
uint8_t sampleTime = 5;        // [5 - 200] how often (ms) encoders are updated
float mScale = 0.6;            // [0.1 - 0.9] 
uint8_t drawSpeed = 190;        // [0 - 255]
uint8_t motorTravelSpeed = 255; // [0 - 255]

bool flipRightMotor = false;
bool flipLeftMotor = false;

bool flipRightEncoder = false;  // CCW is positive
bool flipLeftEncoder = true;