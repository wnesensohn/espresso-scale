// TODO: Re-write this library or use some other hx711 library, as I do not know what these
// chinese characters mean (also the code is a bit sloppy)...

#include "HX711.h"

unsigned long Weight_Gross = 0;
float hx_711_scale = 1;

//****************************************************
//初始化HX711
//****************************************************
void Init_Hx711()
{
	pinMode(HX711_SCK, OUTPUT);
	pinMode(HX711_DT, INPUT);
}

//****************************************************
//获取毛皮重量
//****************************************************
void Get_Gross()
{
	//Weight_Maopi = HX711_Read();
	Weight_Gross = HX711_Read();
}

void adjust_scale(float scale)
{
	hx_711_scale = scale;
}

//****************************************************
//称重
//****************************************************
float Get_Weight(int times)
{
	long weight_sum = 0;
	float weight_net = 0;

	for (int i = 0; i < times; i++)
	{
		weight_sum += HX711_Read() - Weight_Gross;
	}

	weight_net = ((float)(weight_sum)) / (hx_711_scale * times);
	return weight_net;
}

//****************************************************
//读取HX711
//****************************************************
unsigned long HX711_Read(void) //增益128
{
	unsigned long count;
	unsigned char i;
	bool Flag = 0;

	digitalWrite(HX711_DT, HIGH);
	delayMicroseconds(1);

	digitalWrite(HX711_SCK, LOW);
	delayMicroseconds(1);

	count = 0;
	while (digitalRead(HX711_DT))
		;
	for (i = 0; i < 24; i++)
	{
		digitalWrite(HX711_SCK, HIGH);
		delayMicroseconds(1);
		count = count << 1;
		digitalWrite(HX711_SCK, LOW);
		delayMicroseconds(1);
		if (digitalRead(HX711_DT))
			count++;
	}
	digitalWrite(HX711_SCK, HIGH);
	count ^= 0x800000;
	delayMicroseconds(1);
	digitalWrite(HX711_SCK, LOW);
	delayMicroseconds(1);

	return (count);
}
