/**
 * This is a macro for easily enabling or disabling the Serial monitor print statements.
 *
 * You can enable serial print debugging by setting
 *    #define USE_SERIAL 1
 *
 * Or, you can disable serial print debugging by setting
 *    #define USE_SERIAL 0
 *
 * In your main code, rather than using Serial.print() or Serial.println(), use their aliases defined below (e.g., SERIAL_PRINTLN()).
 *
 * The Serial code takes up a lot of flash and RAM, so it really is worth disabling it if you don't need it. When I disabled it for
 * this program, I save about 12% of the RAM and 6% of the flash storage space. That's already a large savings that could make
 * the difference between a synth that sounds good and one that glitches. But printing to the serial monitor is also pretty slow,
 * so disabling saves a lot of processing time too.
 * 
 * stats:
 * // using serial:
 * RAM:   [========  ]  84.5% (used 1730 bytes from 2048 bytes)
 * Flash: [=======   ]  74.9% (used 22998 bytes from 30720 bytes) 
 * 
 * // without serial:
 * RAM:   [=======   ]  72.5% (used 1485 bytes from 2048 bytes)
 * Flash: [=======   ]  68.8% (used 21120 bytes from 30720 bytes)
 *  */

// #define USE_SERIAL

#if USE_SERIAL
  #define SERIAL_PRINT(x) Serial.print(x)
  #define SERIAL_PRINTLN(x) Serial.println(x)
  #define SERIAL_BEGIN(baud) Serial.begin(baud)
  #define SERIAL_TAB Serial.print("\t")
  #define SERIAL_TABS(x) for (uint8_t i = 0; i < x; i++) {Serial.print("t");}
  #define SERIAL_BEGIN(x) Serial.begin(x)
#else
  #define SERIAL_PRINT(x)       do {} while (0)
  #define SERIAL_PRINTLN(x)     do {} while (0)
  #define SERIAL_BEGIN(baud)    do {} while (0)
  #define SERIAL_TAB            do {} while (0)
  #define SERIAL_TABS(x)        do {} while (0)
  #define SERIAL_BEGIN(x)       do {} while (0)
#endif
