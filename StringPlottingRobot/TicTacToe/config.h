//Key UI variables------------------------------------------
uint16_t boardSize = 300;     // [250 - 350] size of tic tac toe square -- ///{"min":0,"max":350}

//Artboard & image params-------------------------------------
double timeStep = 0.1;        // [0.05 - 0.5] Step size of parametric variable. -- ///{"min":0.05,"max":0.5}

//Drawing Related Variables-----------------------------------
float epsilon = 0.5;          // [0 - 1] tolerated error (mm) when moving to pos -- ///{"min":0.1,"max":3}

uint8_t penUpPos = 70;        // [50 - 90] pen position presets -- ///{"min":60,"max":90}
uint8_t penDownPos = 0;       // [0 - 10] -- ///{"min":0,"max":10}
uint8_t penStartPos = 30;     // [10 - 40] -- ///{"min":20,"max":40}

int penRes = 500;             // [100 - 1000] time (ms) held for each pen pixel. -- ///{"min":100,"max":1000}
int dotTime = 150;            // [0 - 500] time (ms) it takes for pen to place a dot -- ///{"min":0,"max":500}
//Motor Variables---------------------------------------------
uint8_t sampleTime = 5;        // [5 - 200] how often (ms) encoders are updated -- ///{"min":5,"max":200}
float mScale = 0.6;            // [0.1 - 0.9] -- ///{"min":0.1,"max":0.9}
uint8_t drawSpeed = 190;        // [0 - 255] -- ///{"min":0,"max":255}
uint8_t motorTravelSpeed = 255; // [0 - 255] -- ///{"min":0,"max":255}

bool flipRightMotor = false; ///{"options":["false","true"]}
bool flipLeftMotor = false; ///{"options":["false","true"]}

bool flipRightEncoder = false;  // CCW is positive -- ///{"options":["false","true"]}
bool flipLeftEncoder = true; ///{"options":["false","true"]}