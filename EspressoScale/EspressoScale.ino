#include "HX711.h"
#include <M5StickC.h>
#include "HampelFilter.h"
#include <FIR.h>

float weight = 0;

FIR<float, 13> fir_lp;

HampelFilter dataBuffer = HampelFilter(0.00, 11, 1.50);

void setup() {

  float coef_lp[13] = { 660, 470, -1980, -3830, 504, 10027, 15214,
                    10027, 504, -3830, -1980, 470, 660};
  fir_lp.setFilterCoeffs(coef_lp);
  fir_lp.getGain();
  
  M5.begin();
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.drawString("SCALE", 80, 0, 4);  
   Init_Hx711();
   Get_Gross();   //clear the weight
   M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);  
   Serial.begin(115200);
     
}

int i = 0;
 
void loop() {  
   M5.update(); 

   i++;

     float raw_weight = Get_Weight();

     dataBuffer.write(raw_weight);
     
   if(i % 10 == 0)
   {
    i = 0;

    weight = fir_lp.processReading(dataBuffer.readMedian());
    
     M5.Lcd.setCursor(40,30,4);
     M5.Lcd.fillRect(0, 30, 160, 30, TFT_BLACK);
     M5.Lcd.printf("%.2f g", weight);
     M5.Lcd.fillRect(0, 70, 160, 10, TFT_BLACK);
     M5.Lcd.fillRect(0, 70, weight*0.016, 10, TFT_YELLOW);
   }

     
     delay(10);  
}
