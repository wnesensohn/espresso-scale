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

private:
	const uint8_t _pin_dout;
	const uint8_t _pin_slk;
};

#endif /* HX711_H_ */
