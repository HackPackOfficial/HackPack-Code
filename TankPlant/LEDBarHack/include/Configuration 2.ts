export type CodeTypes = 'Stock Code';
export type CodeEditRange = {
  range: number[];
  id: string;
  values?: string[];
  validation?: (value: string) => { valid: boolean; message: string };
};
export type CodeInfo = {
  label: string;
  code: string;
  editRanges: CodeEditRange[];
};

export const CODES: Record<CodeTypes, CodeInfo> = {
    standard: {
        label: 'Stock Code',
        editRanges: [
            {
                id: 'USE_SERIAL',
                range: [20, 20, 20, 20],
                values: [
                    '0',
                    '1',
                ],
            },
            {
                id: 'REVERSE_RIGHT_MOTOR',
                range: [49, 29, 49, 33],
                values: [
                    'false',
                    'true',
                ],
            },
            {
                id: 'REVERSE_LEFT_MOTOR',
                range: [50, 28, 50, 32],
                values: [
                    'false',
                    'true',
                ],
            },
            {
                id: 'HEAD_SERVO_KP',
                range: [61, 33, 61, 39],
                validation: (value) => {
                  const num = parseFloat(value);            // note to Byron/IDE team: I don't know if this is correct, but I want this be a float, not an int
                  if (num < -100.0 || num > 100.0) {
                    return {
                      valid: false,
                      message: 'Value must be between -100.0 and 100.0',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'MAX_HEAD_SERVO_SPEED',
                range: [65, 40, 65, 45],
                validation: (value) => {
                  const num = parseFloat(value);            
                  if (num < 0 || num > 180.0) {
                    return {
                      valid: false,
                      message: 'Value must be between 0.0 and 180.0',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'SERVO_TRIM',
                range: [78, 31, 78, 33],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < -20 || num > 20) {
                    return {
                      valid: false,
                      message: 'Value must be between -20 and 20',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'LIGHT_CAL_SPIN_TIME',
                range: [82, 42, 82, 46],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < 0 || num > 9999) {
                    return {
                      valid: false,
                      message: 'Value must be between 0 and 9999',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'LIGHT_CAL_SPIN_SPEED',
                range: [83, 42, 83, 44],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < 0 || num > 255) {
                    return {
                      valid: false,
                      message: 'Value must be between 0 and 255',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'MIN_PARK_TIME',
                range: [87, 36, 87, 41],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < 0 || num > 65535) {
                    return {
                      valid: false,
                      message: 'Value must be between 0 and 65535',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'MAX_PARK_TIME',
                range: [87, 56, 87, 60],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < 0 || num > 65535) {           // note to Byron/IDE team: is there a validation to make sure that MAX_PARK_TIME is greater than MIN_PARK_TIME?
                    return {
                      valid: false,
                      message: 'Value must be between 0 and 65535, and must be equal to or larger than MIN_PARK_TIME',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'PARKING_THRESHOLD_ANGLE',
                range: [88, 45, 88, 46],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num <20 || num > 80) {           
                    return {
                      valid: false,
                      message: 'Value must be between 20 and 80',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'MOISTURE_SAMPLING_PERIOD',
                range: [89, 46, 89, 48],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < 1 || num > 60) {           
                    return {
                      valid: false,
                      message: 'Value must be between 1 and 60',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'HEADING_SAMPLE_WINDOW',
                range: [90, 44, 90, 48],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < 1000 || num > 20000) {           
                    return {
                      valid: false,
                      message: 'Value must be between 1000 and 20000',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'HEADING_FILTER_SIZE',
                range: [91, 41, 91, 43],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < 10 || num > 100) {           
                    return {
                      valid: false,
                      message: 'Value must be between 10 and 100',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'TANK_RETREAT_INTERVAL',
                range: [95, 44, 95, 49],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < 1000 || num > 10000) {           
                    return {
                      valid: false,
                      message: 'Value must be between 1000 and 10000',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'TANK_RETREAT_ROTATE_INTERVAL',
                range: [96, 51, 96, 56],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < 1000 || num > 10000) {           
                    return {
                      valid: false,
                      message: 'Value must be between 1000 and 10000',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'POSITION_RANDOMIZE_SPIN_MIN',
                range: [97, 50, 97, 55],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < 1000 || num > 20000) {           
                    return {
                      valid: false,
                      message: 'Value must be between 1000 and 20000',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'POSITION_RANDOMIZE_SPIN_MAX',
                range: [98, 50, 98, 55],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < 1000 || num > 20000) {           
                    return {
                      valid: false,
                      message: 'Value must be between 1000 and 20000',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'POSITION_RANDOMIZE_DRIVE_MIN',
                range: [99, 51, 99, 55],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < 1000 || num > 20000) {           
                    return {
                      valid: false,
                      message: 'Value must be between 1000 and 20000',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'POSITION_RANDOMIZE_DRIVE_MAX',
                range: [100, 51, 100, 55],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < 1000 || num > 20000) {           
                    return {
                      valid: false,
                      message: 'Value must be between 1000 and 20000',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'CORNER_TRAP_TIME_WINDOW',
                range: [104, 46, 104, 50],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < 10000 || num > 90000) {           
                    return {
                      valid: false,
                      message: 'Value must be between 10000 and 90000',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'CORNER_TRAP_THRESHOLD',
                range: [105, 43, 105, 44],
                values: [
                    '2',
                    '3',
                    '4',
                    '5',
                    '6',
                ],
              },
              {
                id: 'HEADINGS_IN_MAP[SPEED_MAP_INFLECTION_POINTS]',
                range: [120, 64, 120, 67],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < -90 || num > 90) {           
                    return {
                      valid: false,
                      message: 'Value must be between -90 and 90',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'HEADINGS_IN_MAP[SPEED_MAP_INFLECTION_POINTS]',
                range: [120, 69, 120, 71],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < -90 || num > 90) {           
                    return {
                      valid: false,
                      message: 'Value must be between -90 and 90',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'HEADINGS_IN_MAP[SPEED_MAP_INFLECTION_POINTS]',
                range: [120, 74, 120, 76],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < -90 || num > 90) {           
                    return {
                      valid: false,
                      message: 'Value must be between -90 and 90',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'HEADINGS_IN_MAP[SPEED_MAP_INFLECTION_POINTS]',
                range: [120, 77, 120, 79],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < -90 || num > 90) {           
                    return {
                      valid: false,
                      message: 'Value must be between -90 and 90',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'HEADINGS_IN_MAP[SPEED_MAP_INFLECTION_POINTS]',
                range: [120, 81, 120, 83],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < -90 || num > 90) {           
                    return {
                      valid: false,
                      message: 'Value must be between -90 and 90',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'LEFT_SPEED_MAP[SPEED_MAP_INFLECTION_POINTS]',
                range: [121, 63, 121, 66],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < -255 || num > 255) {           
                    return {
                      valid: false,
                      message: 'Value must be between -255 and 255',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'LEFT_SPEED_MAP[SPEED_MAP_INFLECTION_POINTS]',
                range: [121, 69, 121, 72],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < -255 || num > 255) {           
                    return {
                      valid: false,
                      message: 'Value must be between -255 and 255',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'LEFT_SPEED_MAP[SPEED_MAP_INFLECTION_POINTS]',
                range: [121, 72, 121, 75],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < -255 || num > 255) {           
                    return {
                      valid: false,
                      message: 'Value must be between -255 and 255',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'LEFT_SPEED_MAP[SPEED_MAP_INFLECTION_POINTS]',
                range: [121, 77, 121, 80],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < -255 || num > 255) {           
                    return {
                      valid: false,
                      message: 'Value must be between -255 and 255',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'LEFT_SPEED_MAP[SPEED_MAP_INFLECTION_POINTS]',
                range: [121, 82, 121, 85],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < -255 || num > 255) {           
                    return {
                      valid: false,
                      message: 'Value must be between -255 and 255',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              
              {
                id: 'RIGHT_SPEED_MAP[SPEED_MAP_INFLECTION_POINTS]',
                range: [122, 64, 122, 67],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < -255 || num > 255) {           
                    return {
                      valid: false,
                      message: 'Value must be between -255 and 255',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'RIGHT_SPEED_MAP[SPEED_MAP_INFLECTION_POINTS]',
                range: [122, 69, 122, 72],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < -255 || num > 255) {           
                    return {
                      valid: false,
                      message: 'Value must be between -255 and 255',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'RIGHT_SPEED_MAP[SPEED_MAP_INFLECTION_POINTS]',
                range: [122, 74, 122, 77],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < -255 || num > 255) {           
                    return {
                      valid: false,
                      message: 'Value must be between -255 and 255',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'RIGHT_SPEED_MAP[SPEED_MAP_INFLECTION_POINTS]',
                range: [122, 79, 122, 82],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < -255 || num > 255) {           
                    return {
                      valid: false,
                      message: 'Value must be between -255 and 255',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'RIGHT_SPEED_MAP[SPEED_MAP_INFLECTION_POINTS]',
                range: [122, 82, 122, 85],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < -255 || num > 255) {           
                    return {
                      valid: false,
                      message: 'Value must be between -255 and 255',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'MOISTURE_OFFSET',
                range: [129, 38, 129, 41],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < 0 || num > 1023) {           
                    return {
                      valid: false,
                      message: 'Value must be between 0 and 1023',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'PARCHED_UPPER_THRESHOLD',
                range: [130, 45, 130, 49],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < 0 || num > 1023) {           
                    return {
                      valid: false,
                      message: 'Value must be between 0 and 1023',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'THIRSTY_UPPER_THRESHOLD',
                range: [131, 45, 131, 49],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < 0 || num > 1023) {           
                    return {
                      valid: false,
                      message: 'Value must be between 0 and 1023',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
              {
                id: 'SATISFIED_UPPER_THRESHOLD',
                range: [132, 47, 132, 51],
                validation: (value) => {
                  const num = parseInt(value);            
                  if (num < 0 || num > 1023) {           
                    return {
                      valid: false,
                      message: 'Value must be between 0 and 1023',
                    };
                  }
                  return { valid: true, message: '' };
                },
              },
        ],
        code: `
        #ifndef CONFIGURATION_H
        #define CONFIGURATION_H

        #include <Arduino.h>

        /**
         * This is a macro for easily enabling or disabling the Serial monitor print statements. Using the Serial monitor can be extremely
         * helpful for debugging or examining the behavior of your program, but it also takes up room in memory and processing power, so
         * it is helpful to be able to stop using Serial statements easily. This acts like a switch, and lets you deactivate all Serial
         * monitor use just by changing a single value. 
         * 
         * You can enable serial print debugging by setting 
         *    #define USE_SERIAL 1
         * 
         * Or, you can disable serial print debugging by setting
         *    #define USE_SERIAL 0
         * 
         * In your main code, rather than using Serial.print() or Serial.println(), use their aliases defined below (e.g., SERIAL_PRINTLN()):
         *  */ 
        #define USE_SERIAL 1

        #if USE_SERIAL
          #define SERIAL_PRINT(x)     Serial.print(x)
          #define SERIAL_PRINTLN(x)   Serial.println(x)
          #define SERIAL_BEGIN(baud)  Serial.begin(baud)
          #define SERIAL_TAB          Serial.print("\t")
          #define SERIAL_TABS(x)      for (uint8_t i = 0; i < x; i++) {Serial.print("\t");}
        #else
          #define SERIAL_PRINT(x)     do {} while (0)
          #define SERIAL_PRINTLN(x)   do {} while (0)
          #define SERIAL_BEGIN(baud)  do {} while (0)
          #define SERIAL_TAB          do {} while (0)
          #define SERIAL_TABS         do {} while (0)
        #endif


        /**
         * When this is defined, every time a call is made to one of the functions in CL_DRV8835 that would spin the drive motors,
         * the program first sets pin D10 to be INPUT_PULLUP and then checks to see if that pin is LOW. If it is, it overrides
         * all motor speed commands and sets them to 0. This lets you prevent the robot from driving around by connecting a wire
         * from D10 to ground. As soon as you unplug the wire, the robot can move. I've found this parking brake to be useful for
         * testing other features of the robot without it driving around (like seeing what new faces look like).
         */
        #define PARKING_BRAKE


        // If you find that one or both of the motors that drive the tank treads are running in the opposite direction that they should,
        // you can fix that here. To reverse one of the motors, change the value after the appropriate define to "true" without quotes.
        #define REVERSE_RIGHT_MOTOR false
        #define REVERSE_LEFT_MOTOR false


        /**
         * Proportional constant for head servo proportional controller.
         * 
         * so far a kp of 1.0 has worked really well. Originally I was using an instance of FastPID to control the head servo,
         * but after testing I wound just using {kp, ki, kd} = {1.0, 0, 0}, so I rewrote a few lines of code to not rely on FastPID at all.
         * Adjust the value of this constant to change how responsive the head is to differences in light levels between the two sensors.
         * Also, you can make this value negative to make the robot turn its head away from light!
         */
        constexpr float HEAD_SERVO_KP = 1.0;


        // Maximum angular speed that the head servo will be allowed to spin in degrees per second
        constexpr float MAX_HEAD_SERVO_SPEED = 90.0; 


        /**
         * Head servo center point trim.
         * 
         * This is a rough fix to make the head point straight forward, if it doesn't already. If you find that the head looks a bit
         * too the left or right when the robot first turns on and points its head forward, you can adjust that by changing the value
         * of SERVO_TRIM. Units are degrees, positive values turn head to the right.
         * 
         * The best way to fix this is mechanically. Turn the robot on and then back off when the servo moves to center position. 
         * Then remove the gear from the shaft of the servo and rotate it slightly to occupy a new angle on the servo shaft. 
         */
        constexpr int8_t SERVO_TRIM = 0; 


        // light sensor calibration related values
        constexpr uint16_t LIGHT_CAL_SPIN_TIME = 9000;  // spin in circle for 9 seconds during calibration mode
        constexpr uint8_t LIGHT_CAL_SPIN_SPEED = 140;   // spin at speed of 140 during calibration mode


        // constants for changing when the robot notices that it's spinning in circles and parks.
        constexpr uint16_t MIN_PARK_TIME = 10, MAX_PARK_TIME = 20; // minimum and maximum park times in seconds. defaults to 10-20 so you can see all behavior
        constexpr uint8_t PARKING_THRESHOLD_ANGLE = 65;            // heading angle used as threshold in the spinningInCircles() function
        constexpr uint8_t MOISTURE_SAMPLING_PERIOD = 10;           // sample the moisture every 10 seconds. also prints state to LED matrix at this interval in PARK
        constexpr uint16_t HEADING_SAMPLE_WINDOW = 8000;           // track the heading over a window of 8 seconds
        constexpr uint8_t HEADING_FILTER_SIZE = 80;                // how many samples of the heading will be averaged


        // tank movement values
        constexpr uint16_t TANK_RETREAT_INTERVAL = 1000;         // Time in milliseconds to reverse away from object
        constexpr uint16_t TANK_RETREAT_ROTATE_INTERVAL = 1000;  // Time in milliseconds to rotate away from object after reverse
        constexpr uint16_t POSITION_RANDOMIZE_SPIN_MIN = 4000;   // Minimum amount of time robot will spin in place when randomizing its position
        constexpr uint16_t POSITION_RANDOMIZE_SPIN_MAX = 9000;   // Maximum amount of time robot will spin in place when randomizing its position
        constexpr uint16_t POSITION_RANDOMIZE_DRIVE_MIN = 10000; // Minimum amount of time robot will drive forward after spinning during position randomization sequence
        constexpr uint16_t POSITION_RANDOMIZE_DRIVE_MAX = 20000; // Maximum amount of time robot will drive forward after spinning during position randomization sequence


        // corner trap detection variables
        constexpr uint16_t CORNER_TRAP_TIME_WINDOW = 45000;      // Time window during which bumper presses count toward the corner trap detection threshold
        constexpr uint8_t CORNER_TRAP_THRESHOLD = 4;             // How many bumper presses within the above time window indicate a corner trap


        /**
         * speed maps for converting headings to motor speeds.
         * 
         * It's important that these are variables, not constants. Constants get get stored in PROGMEM,
         * and multiMap isn't written in a way that lets it read arrays from PROGMEM. We could rewrite it to use
         * PROGMEM, but reading from PROGMEM is slow compared to reading from RAM, and since these arrays are used
         * frequently, we want the speed RAM offers.
         * 
         * If you want to see what these mappings look like in graph form, find that here: 
         * https://docs.google.com/spreadsheets/d/1x5wN7Up8Ra-3hVbiYZuimurmymXRQdSPz-L6IVtE-hY/edit?usp=sharing
         */
        constexpr uint8_t SPEED_MAP_INFLECTION_POINTS = 5;
        inline int16_t HEADINGS_IN_MAP[SPEED_MAP_INFLECTION_POINTS] = {-90, -45, 0, 45, 90};    // independent variable for the piecewise functions
        inline int16_t LEFT_SPEED_MAP[SPEED_MAP_INFLECTION_POINTS] = {-255, 0, 150, 192, 255};  // dependent variable for piecewise linear function for left motor (ex: when heading is 45, lMotor speed is 192)
        inline int16_t RIGHT_SPEED_MAP[SPEED_MAP_INFLECTION_POINTS] = {255, 192, 150, 0, -255}; // dependent variable for piecewise linear function for right motor speed (ex: when heading is 45, rMotor speed is 0)

        /**
         * These values relate to the moisture sensor. The offset value is the number that the sensor reports when it's in open air, while the other values
         * are upper bounds for the different moisture levels. See [[Footnotes.md#Footnote 7: Notes on calibrating the moisture sensor readings]]
        */
        // water threshold values
        constexpr uint16_t MOISTURE_OFFSET = 754;       // get the reading on the moisture sensor in open air and use that here
        constexpr uint8_t PARCHED_UPPER_THRESHOLD = 40; // the upper bound for considering the plant to be parched
        constexpr uint8_t THIRSTY_UPPER_THRESHOLD = 70; // the upper bound for considering the plant to be thirsty.
        constexpr uint8_t SATISFIED_UPPER_THRESHOLD = 130;  // beyond this the plant will be considered to be drowning, with water starting to fill the pot rather than be absorbed by the LECA.


        #endif // CONFIGURATION_H
        `,
    },


};