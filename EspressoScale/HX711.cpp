#include "HX711.h"

float Weight_Gross = 0,Weight_Net = 0;

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
  Weight_Gross=Get_Weight();

} 

//****************************************************
//称重
//****************************************************
float Get_Weight()
{
	Weight_Net = HX711_Read();
  Serial.print("raw is ");
  Serial.println(Weight_Net);
	Weight_Net =(float)((Weight_Net)/122.2-Weight_Gross+0.05); 	
  Serial.print("maopi is ");
  Serial.println(Weight_Gross);
	return Weight_Net;
}

//****************************************************
//读取HX711
//****************************************************
unsigned long HX711_Read(void)	//增益128
{
	unsigned long count; 
	unsigned char i;
	bool Flag = 0;

	digitalWrite(HX711_DT, HIGH);
	delayMicroseconds(1);

	digitalWrite(HX711_SCK, LOW);
	delayMicroseconds(1);

  	count=0; 
  	while(digitalRead(HX711_DT)); 
  	for(i=0;i<24;i++)
	{ 
	  	digitalWrite(HX711_SCK, HIGH); 
		delayMicroseconds(1);
	  	count=count<<1; 
		digitalWrite(HX711_SCK, LOW); 
		delayMicroseconds(1);
	  	if(digitalRead(HX711_DT))
			count++; 
	} 
 	digitalWrite(HX711_SCK, HIGH); 
	count ^= 0x800000;
	delayMicroseconds(1);
	digitalWrite(HX711_SCK, LOW); 
	delayMicroseconds(1);
	
	return(count);
}
