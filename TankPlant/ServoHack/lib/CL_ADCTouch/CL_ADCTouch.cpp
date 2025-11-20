/*
  ADCTouch.h - Library for Capacitive touch sensors using only one ADC PIN
  Original library created by martin2250, April 23, 2014.
  Original library can be found at https://github.com/martin2250/ADCTouch

  Modified to include readNext() function by Evan Barnes for Crunchlabs, September 2024.
  Released into the public domain under the MIT license. 

  The original library, ADCTouch, does not include the readNext() function that is present here.

  The readNext function basically just puts one new ADC reading into the filter and then
  returns the updated filter value, rather than filling completely refilling the sample
  buffer with new readings each time it is called, which is how the read() function
  behaves. readNext() takes less time to execute than the read() function, but as a tradeoff
  it is more susceptible to noise. Both read() and readNext() are blocking functions, and in 
  some situations read() blocks for long enough that it can impact other program functions.
*/
#include "Arduino.h"
#include "CL_ADCTouch.h"

int _values[SAMPLES];
int _currentSlot = 0;
int _lastValue = 0;
int _sum = 0;


int ADCTouchClass::read(byte ADCChannel)
{
	//long _value = 0;
	short _value = 0; // , _sum = 0;
	for (int _counter = 0; _counter < SAMPLES; _counter++)
	{
		// set the analog pin as an input pin with a pullup resistor
		// this will start charging the capacitive element attached to that pin
		pinMode(ADCChannel, INPUT_PULLUP);

		// connect the ADC input and the internal sample and hold capacitor to ground to discharge it
		ADMUX |= 0b11111;
		// start the conversion
		ADCSRA |= (1 << ADSC);

		// ADSC is cleared when the conversion finishes
		while ((ADCSRA & (1 << ADSC)));

		pinMode(ADCChannel, INPUT);

		// get value
		_value = analogRead(ADCChannel);
		// save in rolling array
		_values[_counter] = _value;
		// add to running total
		_sum += _value;
	}
	// get average
	_lastValue = _sum / SAMPLES;
	// return 
	return _lastValue;
}


int ADCTouchClass::readNext(byte ADCChannel)
{
	short _value = 0; // , _sum = 0;
	// set the analog pin as an input pin with a pullup resistor
	// this will start charging the capacitive element attached to that pin
	pinMode(ADCChannel, INPUT_PULLUP);

	// connect the ADC input and the internal sample and hold capacitor to ground to discharge it
	ADMUX |= 0b11111;
	// start the conversion
	ADCSRA |= (1 << ADSC);

	// ADSC is cleared when the conversion finishes
	while ((ADCSRA & (1 << ADSC)));

	pinMode(ADCChannel, INPUT);

	// get current value
	_value = analogRead(ADCChannel);
	// adjust sum
	_sum = _sum - _values[_currentSlot] + _value;
	// place in rolling array
	_values[_currentSlot++] = _value;
	// determine next slot to fill
	_currentSlot = _currentSlot % SAMPLES;
	// get average
	_lastValue = _sum / SAMPLES; 
	// return 
	return _lastValue;
}


int ADCTouchClass::lastValue()
{
	return _lastValue;
}


ADCTouchClass ADCTouch;