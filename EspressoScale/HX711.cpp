/*
 * hx711.cpp
 *
 *  Created on: Oct 31, 2012
 *      Author: agu
 */

#include "hx711.h"

HX711::HX711(uint8_t pin_dout, uint8_t pin_slk) :
		_pin_dout(pin_dout), _pin_slk(pin_slk)
{
}

HX711::~HX711()
{
}

void HX711::init()
{
	pinMode(_pin_slk, OUTPUT);
	pinMode(_pin_dout, INPUT);

	digitalWrite(_pin_slk, HIGH);
	delayMicroseconds(200);
	digitalWrite(_pin_slk, LOW);

	averageValue();
	this->setOffset(averageValue());
	this->setScale(1);
}

long HX711::averageValue(byte times)
{
	long sum = 0;
	for (byte i = 0; i < times; i++)
	{
		sum += getValue();
	}

	return sum / times;
}

long HX711::getValue()
{
	byte data[3];

	while (digitalRead(_pin_dout))
		;

	delayMicroseconds(50);

	for (byte j = 0; j < 3; j++)
	{
		for (byte i = 0; i < 8; i++)
		{
			digitalWrite(_pin_slk, HIGH);
			delayMicroseconds(2);
			digitalWrite(_pin_slk, LOW);
			delayMicroseconds(50);
			bitWrite(data[2 - j], 7 - i, digitalRead(_pin_dout));
			delayMicroseconds(50);
		}
	}

	digitalWrite(_pin_slk, HIGH);
	delayMicroseconds(2);
	digitalWrite(_pin_slk, LOW);

	return ((long) data[2] << 16) | ((long) data[1] << 8) | (long) data[0];
}

void HX711::tare(byte times)
{
	setOffset(averageValue(times));
}

void HX711::tareWithValue(float val)
{
	_offset = _offset + (val * _scale);
}

void HX711::setOffset(long offset)
{
	_offset = offset;
}

void HX711::setScale(float scale)
{
	_scale = scale;
}

float HX711::getGram(byte times)
{
	long val = (averageValue(times) - _offset);
	return (float) val / _scale;
}
