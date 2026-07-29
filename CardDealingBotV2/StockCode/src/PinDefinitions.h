// BUTTON PIN INPUTS
#define BUTTON_PIN_4 14 // Button 4 ("Back").
#define BUTTON_PIN_3 15 // Button 3 ("Decrease").
#define BUTTON_PIN_2 16 // Button 2 ("Increase/Hit").
#define BUTTON_PIN_1 17 // Button 1 ("Advance/Pass").
#define RIG_SWITCH A6   // Switch to toggle rigged game vs non-rigged game.

// MOTOR PINS
#define MOTOR_1_PIN_1 9 // Motor 1 pins control the card-throwing flywheel direction.
#define MOTOR_1_PIN_2 10
#define MOTOR_1_PWM 11  // Motor 1 PWM controls the speed of the flywheel.
#define MOTOR_2_PWM 5   // Motor 2 PWM controls the speed of rotation for the (yaw) motor.
#define MOTOR_2_PIN_2 6 // Motor 2 pins control the yaw motor direction.
#define MOTOR_2_PIN_1 7
#define FEED_SERVO_PIN 3 // This pin controls the card feeding servo motor.

// SENSOR PINS
#define CARD_SENS 2  // IR sensor that determines whether or not a card has passed through the mouth of DEALR.
#define UV_READER A7 // Sensor for reading the reflectance of the UV ink on marked cards.

// OTHER PINS
#define STNDBY 8 // Standby needs to be pulled HIGH. This can be done with a wire to 5V as well.
