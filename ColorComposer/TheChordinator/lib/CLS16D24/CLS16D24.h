/**
 * @file CLS16D24.h
 * @brief Header file for the CLS16D24 color sensor library.
 *
 * This library provides functions to interface with the CLS16D24 color sensor over I2C.
 * It includes functions for setting gain, resolution, and conversion time, as well as reading color data.
 * 
 * To do: 
 * - There the sensor has methods for setting interrupts based on high and low thresholds on the color channels. 
 *   The library currently doesn't have any functions for setting those interrupts, but they could be cool to 
 *   add for future-proofing purposes. 
 */

 #ifndef CLS16D24_H
 #define CLS16D24_H
 
 #include <Arduino.h>
 #include <Wire.h>
 
 #define CLS16D24_ADDRESS 0x38  // Default I2C address
 
 // Register addresses
 #define SYSM_CTRL 0x00
 #define CLS_GAIN 0x04
 #define CLS_TIME 0x05
 #define RCH_DATA_L 0x1C
 #define RCH_DATA_H 0x1D
 #define GCH_DATA_L 0x1E
 #define GCH_DATA_H 0x1F
 #define BCH_DATA_L 0x20
 #define BCH_DATA_H 0x21
 #define WCH_DATA_L 0x22
 #define WCH_DATA_H 0x23
 #define IRCH_DATA_L 0x24
 #define IRCH_DATA_H 0x25
 
 /**
  * @class CLS16D24
  * @brief Class for interfacing with the CLS16D24 color sensor.
  */
 class CLS16D24 {
 public:
     /**
      * @brief Constructor for CLS16D24 class.
      */
     CLS16D24();
 
     /**
      * @brief Initializes the sensor and sets up the I2C communication.
      * @param wirePort Reference to the I2C port (default: Wire).
      * @return True if initialization was successful.
      */
     bool begin(bool i2cFastMode = true, TwoWire &wirePort = Wire) {
         wire = &wirePort;
         wire->begin();
         if (i2cFastMode) wire->setClock(400000); // Set I2C to Fast Mode (400kHz)
         return true;
     }
 
     /**
      * @brief Resets the sensor.
      */
     void reset();
 
     /**
      * @brief Enables the sensor.
      */
     void enable();
 
     /**
      * @brief Sets the gain of the sensor.
      * @param gain Gain factor (1, 4, 8, 32, or 96).
      * @param doubleSensorArea Whether to enable double sensor area (default: true).
      */
     void setGain(uint8_t gain, bool doubleSensorArea = true);
 
     /**
      * @brief Sets the resolution and conversion time for the sensor.
      * @param time Resolution and conversion time setting.
      */
     void setResolutionAndConversionTime(uint8_t time);
 
     /**
      * @brief Reads RGBWIR color data from the sensor.
      * @param r Reference to store red channel value.
      * @param g Reference to store green channel value.
      * @param b Reference to store blue channel value.
      * @param w Reference to store white channel value.
      * @param ir Reference to store infrared channel value.
      */
     void readRGBWIR(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &w, uint16_t &ir);
 
     /**
      * @brief Returns the conversion time in milliseconds.
      * @return Conversion time in milliseconds.
      */
     float getConversionTimeMillis();
 
     /**
      * @brief Returns the resolution set for the sensor.
      * @return Resolution value.
      */
     uint16_t getResolution();
 
     /**
      * @brief Returns the current gain setting of the sensor.
      * @return Gain factor (1, 4, 8, 32, or 96).
      */
     uint8_t getGain();
 
 private:
     uint8_t GAIN_BYTE = 0x10; ///< Stores the gain setting.
     uint8_t CONV_TIME = 0x10; ///< Stores the conversion time setting.
     uint8_t CLS_CONV = CONV_TIME >> 4; ///< Stores CLS_CONV value.
     uint8_t INT_TIME = intPow(4, (CONV_TIME & 0x03)); ///< Stores INT_TIME value.
 
     /**
      * @brief Fast integer exponentiation.
      * @param base Base value.
      * @param exp Exponent value.
      * @return Computed power value.
      */
     inline uint16_t intPow(uint16_t base, uint16_t exp);
 
     /**
      * @brief Writes a value to a register.
      * @param reg Register address.
      * @param value Value to write.
      */
     void writeRegister(uint8_t reg, uint8_t value);
 
     /**
      * @brief Reads a 16-bit value from a register.
      * @param reg Register address.
      * @return 16-bit register value.
      */
     uint16_t readRegister16(uint8_t reg);
 
     TwoWire *wire; ///< Pointer to I2C interface.
 };
 
 #endif  // CLS16D24_H
 
 
 
 /**
  * use the setIntegrationTime(COMMAND) function to set both the sensor resolution and integration time. Basically, if you send one of the values
  * specified under the COMMAND column, that will set the corresponding resolution and conversion time specified in the row of that command value.
  * 
  * You can find the datasheet here: https://download.mikroe.com/documents/datasheets/CLS-16D24-44-DF8_TR8%20Datasheet.pdf
  * 
  * Alternatively, you can use the spreadsheet I made to calculate the resolution and conversion times that get set by the command sent. You can
  * find that spreadsheet here: https://docs.google.com/spreadsheets/d/1sWL521Zyir4Itsj1mEqNmA60axIWLgaT-tv8dGDWh7w/edit?usp=sharing
  * 
  * Note that you need to make a copy of the spreadsheet before being able to use the dropdown menus for the calculator.
  * 
  * What follows is a chart of all possible command values for setIntegrationTime() and their corresponding data resolutions and conversion times.
  * Finally, note that the max resolution is not always exactly a power of 2. E.g., 0x51 gives a max resolution value of 24575. I tested it and
  * this is accurate. That number fits in 15 bits, but it's not 2^15, it's actually less than that. The calculated resolution column tells you the
  * smallest number of bits required to represent the maximum resolution.
  * 
  * COMMAND	CLS_CONV	INT_TIME	Calculated Resolution (maximum possible value)	Calculated Resolution (in bits)	    Calculated Conversion Time (in milliseconds)	Max conversions per second:
  * 0x00	    0	        0	        1023	                                        10	                                5.8937	                                        10.18036208
  * 0x01	    0	        1	        4095	                                        12	                                12.0938	                                        4.961219799
  * 0x02	    0	        2	        16383	                                        14	                                36.8942	                                        1.62627188
  * 0x03	    0	        3	        65535	                                        16	                                136.0958                                        0.4408659194
  * 0x10	    1	        0	        2047	                                        11	                                7.9604	                                        7.537309683
  * 0x11	    1	        1	        8191	                                        13	                                20.3606	                                        2.94686797
  * 0x12	    1	        2	        32767	                                        15	                                69.9614	                                        0.857615771
  * 0x13	    1	        3	        65535	                                        16	                                268.3646	                                    0.223576433
  * 0x20	    2	        0	        3071	                                        12	                                10.0271	                                        5.983783946
  * 0x21	    2	        1	        12287	                                        14	                                28.6274	                                        2.095894143
  * 0x22	    2	        2	        49151	                                        16	                                103.0286	                                    0.5823625673
  * 0x23	    2	        3	        65535	                                        16	                                400.6334	                                    0.1497628505
  * 0x30	    3	        0	        4095	                                        12	                                12.0938	                                        4.961219799
  * 0x31	    3	        1	        16383	                                        14	                                36.8942	                                        1.62627188
  * 0x32	    3	        2	        65535	                                        16	                                136.0958	                                    0.4408659194
  * 0x33	    3	        3	        65535	                                        16	                                532.9022	                                    0.1125910158
  * 0x40	    4	        0	        5119	                                        13	                                14.1605	                                        4.237138519
  * 0x41	    4	        1	        20479	                                        15	                                45.161	                                        1.328579969
  * 0x42	    4	        2	        65535	                                        16	                                169.163	                                        0.3546874908
  * 0x43	    4	        3	        65535	                                        16	                                665.171	                                        0.09020236901
  * 0x50	    5	        0	        6143	                                        13	                                16.2272	                                        3.697495563
  * 0x51	    5	        1	        24575	                                        15	                                53.4278	                                        1.123010867
  * 0x52	    5	        2	        65535	                                        16	                                202.2302	                                    0.2966915921
  * 0x53	    5	        3	        65535	                                        16	                                797.4398	                                    0.07524078934
  * 0x60	    6	        0	        7167	                                        13	                                18.2939	                                        3.279781785
  * 0x61	    6	        1	        28671	                                        15	                                61.6946	                                        0.9725324421
  * 0x62	    6	        2	        65535	                                        16	                                235.2974	                                    0.2549964428
  * 0x63	    6	        3	        65535	                                        16	                                929.7086	                                    0.06453635042
  * 0x70	    7	        0	        8191	                                        13	                                20.3606	                                        2.94686797
  * 0x71	    7	        1	        32767	                                        15	                                69.9614	                                        0.857615771
  * 0x72	    7	        2	        65535	                                        16	                                268.3646	                                    0.223576433
  * 0x73	    7	        3	        65535	                                        16	                                1061.9774	                                    0.05649837746
  * 0x80	    8	        0	        9215	                                        14	                                22.4273	                                        2.675310893
  * 0x81	    8	        1	        36863	                                        16	                                78.2282	                                        0.7669868411
  * 0x82	    8	        2	        65535	                                        16	                                301.4318	                                    0.1990500007
  * 0x83	    8	        3	        65535	                                        16	                                1194.2462	                                    0.05024089673
  * 0x90	    9	        0	        10239	                                        14	                                24.494	                                        2.449579489
  * 0x91	    9	        1	        40959	                                        16	                                86.495	                                        0.6936817157
  * 0x92	    9	        2	        65535	                                        16	                                334.499	                                        0.1793727336
  * 0x93	    9	        3	        65535	                                        16	                                1326.515	                                    0.04523130157
  * 0xA0	    A	        0	        11263	                                        14	                                26.5607	                                        2.258976608
  * 0xA1	    A	        1	        45055	                                        16	                                94.7618	                                        0.6331665291
  * 0xA2	    A	        2	        65535	                                        16	                                367.5662	                                    0.1632359015
  * 0xA3	    A	        3	        65535	                                        16	                                1458.7838	                                    0.04113015239
  * 0xB0	    B	        0	        12287	                                        14	                                28.6274	                                        2.095894143
  * 0xB1	    B	        1	        49151	                                        16	                                103.0286	                                    0.5823625673
  * 0xB2	    B	        2	        65535	                                        16	                                400.6334	                                    0.1497628505
  * 0xB3	    B	        3	        65535	                                        16	                                1591.0526	                                    0.03771088398
  * 0xC0	    C	        0	        13311	                                        14	                                30.6941	                                        1.954773067
  * 0xC1	    C	        1	        53247	                                        16	                                111.2954	                                    0.5391058391
  * 0xC2	    C	        2	        65535	                                        16	                                433.7006	                                    0.1383442864
  * 0xC3	    C	        3	        65535	                                        16	                                1723.3214	                                    0.03481648867
  * 0xD0	    D	        0	        14335	                                        14	                                32.7608	                                        1.831457107
  * 0xD1	    D	        1	        57343	                                        16	                                119.5622	                                    0.5018308462
  * 0xD2	    D	        2	        65535	                                        16	                                466.7678	                                    0.1285435713
  * 0xD3	    D	        3	        65535	                                        16	                                1855.5902	                                    0.03233472563
  * 0xE0	    E	        0	        15359	                                        14	                                34.8275	                                        1.722776542
  * 0xE1	    E	        1	        61439	                                        16	                                127.829	                                        0.4693770584
  * 0xE2	    E	        2	        65535	                                        16	                                499.835	                                        0.1200396131
  * 0xE3	    E	        3	        65535	                                        16	                                1987.859	                                    0.03018322728
  * 0xF0	    F	        0	        16383	                                        14	                                36.8942	                                        1.62627188
  * 0xF1	    F	        1	        65535	                                        16	                                136.0958	                                    0.4408659194
  * 0xF2	    F	        2	        65535	                                        16	                                532.9022	                                    0.1125910158
  * 0xF3	    F	        3	        65535	                                        16	                                2120.1278	                                    0.02830018077
 
  */