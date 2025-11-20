//Key UI variables------------------------------------------
uint16_t boardSize = 300;   // [0 - 350] size of canvass

//Drawing Related Variables-----------------------------------
float epsilon = 0.5;        // [0.1 - 3] tolerated error (mm) when moving to pos

uint8_t penUpPos = 70;      // [60 - 90] pen position presets
uint8_t penDownPos = 0;     // [0 - 10]
uint8_t penStartPos = 30;   // [20 - 40]

//Motor Variables---------------------------------------------
uint8_t sampleTime = 5;         // [5 - 200] how often (ms) encoders are updated
float mScale = 0.6;             // [0.1 - 0.9] motor smoothing near target
uint8_t drawSpeed = 190;        // [0 - 255]
uint8_t motorTravelSpeed = 255; // [0 - 255]

bool flipRightMotor = false;
bool flipLeftMotor = false;

bool flipRightEncoder = false;  // CCW is positive
bool flipLeftEncoder = true;