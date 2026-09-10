//Key UI variables------------------------------------------
uint16_t boardSize = 300;   // [0 - 350] size of canvass -- ///{"min":0,"max":350}

//Drawing Related Variables-----------------------------------
float epsilon = 0.5;        // [0.1 - 3] tolerated error (mm) when moving to pos -- ///{"min":0.1,"max":3}

uint8_t penUpPos = 70;      // [60 - 90] pen position presets -- ///{"min":60,"max":90}
uint8_t penDownPos = 0;     // [0 - 10] -- ///{"min":0,"max":10}
uint8_t penStartPos = 30;   // [20 - 40] -- ///{"min":0,"max":40}

//Motor Variables---------------------------------------------
uint8_t sampleTime = 5;         // [5 - 200] how often (ms) encoders are updated -- ///{"min":5,"max":200}
float mScale = 0.6;             // [0.1 - 0.9] motor smoothing near target -- ///{"min":0.1,"max":0.9}
uint8_t drawSpeed = 190;        // [0 - 255] -- ///{"min":0,"max":255}
uint8_t motorTravelSpeed = 255; // [0 - 255] -- ///{"min":0,"max":255}

bool flipRightMotor = false; ///{"options":["false","true"]}
bool flipLeftMotor = false; ///{"options":["false","true"]}

bool flipRightEncoder = false;  // CCW is positive -- ///{"options":["false","true"]}
bool flipLeftEncoder = true; ///{"options":["false","true"]}