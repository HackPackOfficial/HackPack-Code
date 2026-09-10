//Key UI variables------------------------------------------
uint8_t bootSize = 0;      // [0 - 2] 0 - small, 1 - medium, 2 - large -- ///{"range":[0,2]}
uint8_t initCursorPos = 6; // [0 - 7] Boot onto image selction -- ///{"range":[0,7]}

uint8_t penStyle = 3;     // [0 - 3] Default pen style: dist, time, dotted, chance -- ///{"range":[0,3]}

float manualXOffset = 0;  // [0 - 100] Manual image placement adjustment. -- ///{"min":0,"max":100}
float manualYOffset = 0;  // [0 - 100] Positive Y moves image down. Units in mm -- ///{"min":0,"max":100}

//Artboard & image params-------------------------------------
double timeStep = 0.05;   // [0.01 - 0.1] Step size of parametric variable. -- ///{"min":0.01,"max":0.1}

//Drawing Related Variables-----------------------------------
float penSmoothFactor = 0.05;  // [0 - 0.5], complementary filter for servo pos -- ///{"min":0,"max":0.5}

float epsilon = 0.3;      // [0.1 - 1] tolerated error (mm) when moving to pos -- ///{"min":0.1,"max":1}

uint8_t penUpPos = 70;    // [60 - 90] pen position presets -- ///{"min":60,"max":90}
uint8_t penDownPos = 0;   // [0 - 10] -- ///{"min":0,"max":10}
uint8_t penStartPos = 30; // [20 - 40] -- ///{"min":20,"max":40}

float penUnit = 0.3;           // minimum movement needed to update pen position
float dps = 3.5;               // [0 - 5] dots per second (max) of pen -- ///{"min":0,"max":5}
int penRes = 500;              // [100 - 750] time (ms) of each pen "pixel". -- ///{"min":100,"max":750}
int dotTime = 150;             // [50 - 300] time it takes for pen to place dot -- ///{"min":50,"max":300}
uint8_t penHold = 35;          // [0 - 100] how long pendown persits after a dot -- ///{"min":0,"max":100}

//Motor Variables---------------------------------------------
uint8_t sampleTime = 5;         // [5 - 200] how often (ms) encoders are updated -- ///{"min":5,"max":500}

uint8_t drawSpeed = 190;        // [0 - 255] -- ///{"min":0,"max":255}
uint8_t motorTravelSpeed = 255; // [0 - 255] -- ///{"min":0,"max":255}
uint8_t motorCoastDist = 3;     // [0 - 5] within dist mm we reduce motor speed -- ///{"min":0,"max":5}
float epsilonScaling = 0.4;     // [0.1 - 0.9] scale motor speed near target -- ///{"min":0.1,"max":0.9}

bool flipRightMotor = false; ///{"options":["false","true"]}
bool flipLeftMotor = false; ///{"options":["false","true"]}

bool flipRightEncoder = false;  // CCW is positive -- ///{"options":["false","true"]}
bool flipLeftEncoder = true; ///{"options":["false","true"]}