#ifndef ANALOG_BUTTONS_H
#define ANALOG_BUTTONS_H
#include <Arduino.h>
#include <EventDelay.h>

/**
 * @brief Debounces the analog multiplexed buttons using Mozzi's EventDelay class.
 *
 * @details Uses time-based debouncing based on Mozzi's EventDelay class. I originally had a fancy timeless debouncing
 * scheme that used bit shifting values as a way to wait for the button press to stabilize after bouncing. It actually
 * worked pretty well, but I was still have problems with bounce during testing. I tried layering that system on top of
 * itself, but it was getting really arcane and hard to read and understand, so I switched to time-based debouncing. It
 * instantly worked better. We have to use Mozzi's alternatives to millis() for this because the timers in the Atmega328
 * are either claimed by Mozzi, or are running faster than normal because that removes what is otherwise an audible whine
 * from the motors. This means that all normal timing functions are entirely broken and we can't use them, but Mozzi offers
 * several alternatives in the form of ticks() and EventDelay.
 *
 * @param buttonPinVal the ADC reading of the pin that the buttons are connected to. Be sure to use mozziAnalogRead() and not
 * the stock Arduino analogRead(). analogRead() is actually quite slow.
 */
uint8_t getButtonPressed(uint16_t buttonPinVal)
{
  constexpr uint8_t DEBOUNCE_INTERVAL = 10;
  static EventDelay debounceTimer;
  static bool evaluatingPress = false;
  static uint8_t triggeredVal = 255, stableVal = 255;
  /**
   * this next part is where the magic happens. The buttons are multiplexed on a single analog pin. This works by
   * creating a resistor ladder, where the buttons connect the analog pin to a different point on the ladder.
   * This creates a unique voltage divider for each button, so whenever you press a button, the analog pin sees
   * a distinct voltage that represents which button was pressed. The downside of this is that only one button can
   * ever be pressed, and lower-numbered buttons will always have precedence. If you press and hold B1 and then press
   * B0, B0 will take over from B1. But the major upside of this is that you can put a bunch of buttons on a single
   * pin! As long as you only need to detect a single button being pressed at a time, this is a worthwhile trade.
   * This next line evaluates which button was pressed by checking the value reported by the ADC. I designed the
   * resistor ladder so that there's a roughly even spacing in ADC values between each button press (~340 counts
   * between each button). This requires using different resistor values at each point in the ladder, but makes
   * the code cleaner and more reliable. Otherwise you'd get logarithmically decreasing spacing between buttons
   * when represented as ADC counts.
   */
  uint8_t rawButton = ((buttonPinVal < 172) ? 0 : ((buttonPinVal < 510) ? 1 : ((buttonPinVal < 850 ? 2 : 255))));

  // now start the debounce logic if needed. Basically this looks for a state change and starts a timer.
  if (rawButton != triggeredVal && !evaluatingPress)
  {
    evaluatingPress = true;
    debounceTimer.set(DEBOUNCE_INTERVAL);
    debounceTimer.start();
    triggeredVal = rawButton;
  }

  // if the timer is up and we're evaluating a state change for possible button press, set the new stable state
  if (evaluatingPress && debounceTimer.ready())
  {
    // if the new reading is the same as the reading that triggered evaluation, set the stable state to the new reading.
    // otherwise, leave the stable state as is.
    stableVal = (rawButton == triggeredVal) ? triggeredVal : stableVal;
    evaluatingPress = false;
  }
  return stableVal;
}




#endif