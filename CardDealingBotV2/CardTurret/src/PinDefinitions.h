// BUTTON PIN INPUTS
#define BUTTON_PIN_4 14 // Button 4 ("Back").
#define BUTTON_PIN_3 15 // Button 3 ("Decrease").
#define BUTTON_PIN_2 16 // Button 2 ("Increase/Hit").
#define BUTTON_PIN_1 17 // Button 1 ("Advance/Pass").

// MOTOR PINS
#define MOTOR_1_PIN_1 9 // Motor 1 pins control the card-throwing flywheel direction.
#define MOTOR_1_PIN_2 10
#define MOTOR_1_PWM 11  // Motor 1 PWM controls the speed of the flywheel.
#define MOTOR_2_PWM 5   // Motor 2 PWM controls the speed of rotation for the (yaw) motor.
#define MOTOR_2_PIN_2 6 // Motor 2 pins control the yaw motor direction.
#define MOTOR_2_PIN_1 7
#define FEED_SERVO_PIN 4 // This pin controls the card feeding servo motor.

// SENSOR PINS
#define CARD_SENS 2        // IR sensor that determines whether or not a card has passed through the mouth of DEALR.
#define IR_RECEIVER_PIN 3  // Pin used for the IR receiver from the Turret build.

// OTHER PINS
#define STNDBY 8 // Standby needs to be pulled HIGH. This can be done with a wire to 5V as well.
