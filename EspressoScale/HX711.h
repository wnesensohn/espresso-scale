#ifndef HX711_H_
#define HX711_H_

#include "Arduino.h"

class HX711
{
public:
	HX711(uint8_t pin_din, uint8_t pin_slk);
    void init();
	virtual ~HX711();
	long getValue();
	long averageValue(byte times = 10);
	void setOffset(long offset);
	void setScale(float scale = 742.f);
    void tare(byte times = 32);
	float getGram(byte times = 32);

private:
	const uint8_t _pin_dout;
	const uint8_t _pin_slk;
	long _offset;
	float _scale;
};

#endif /* HX711_H_ */
