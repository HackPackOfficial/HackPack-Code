/*
  ADCTouch.cpp - Library for Capacittive touch sensors using only one ADC PIN
  Created by martin2250, April 23, 2014.
  Released into the public domain.
*/
#include "Arduino.h"
#include "ADCTouch.h"

int _values[SAMPLES];
int _currentSlot = 0;
int _lastValue = 0;
int _sum = 0;

// Call once during setup()
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

// call once at top of every loop()
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
	// now get new sum
	//for (int i = 0; i < SAMPLES; i++)
	//{
	//	_sum += _values[i];
	//}
	// get average
	_lastValue = _sum / SAMPLES; // (_sum >> 4); // 
	// return 
	return _lastValue;
}

int ADCTouchClass::lastValue()
{
	return _lastValue;
}


ADCTouchClass ADCTouch;