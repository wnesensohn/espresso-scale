#ifndef __HX711__H__
#define __HX711__H__

#include <Arduino.h>

#define HX711_SCK 32

#define HX711_DT 33

extern void Init_Hx711();
extern unsigned long HX711_Read(void);
extern float Get_Weight(int times);
extern void Get_Gross();
extern void adjust_scale(float scale);

#endif
