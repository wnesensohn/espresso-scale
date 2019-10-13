// This is code for an espresso scale. Think Acaia Lunar, only not ridiculously expensive (In the end, building this
// project will probably be more expensive than even the Lunar. But also much more fun!).

// All code under GPL v2 (see LICENSE), except where other licenses apply.

#include <M5Stack.h>
#include "HX711.h"
#include "HampelFilter.h"

HX711 scale(22, 21); // GROVE A
//HX711 scale(36, 26); // GROVE B
//HX711 scale(16, 17); // GROVE C

// TODO (general): 
// * auto-zeroing (zero large weight-changes in small time-increments)
// * zero-tracking (zero very small weight-changes happening over a long timespan)
//   to compensate drift. There's not a lot of drift, and it's temperature related, but it's enough to be annoying.
// * flow-rate measurement in ml/s
// * shot-time (if need be compensate for filter lag, but probably not necessary, after all it's just espresso ffs)
// * calibration with known weights
// * auto shut-down (not the annoying kind, something like "after 5 minutes after last weight change of more than 1g/30s")
// * see if we can get a reliable capacity reading of the LiPo somehow to display when we need more juice
// * switch to M5Stack
// * use HW I2C (low prio)
// * double-buffer for display to prevent annoying flickering

// TODO: Add a method to calibrate the scale by using a pre-defined weight
float current_scale = 890;
float auto_zero_eps = 0.005;

FastRunningMedian<31> hampel_filter;
FastRunningMedian<200> slow_filter_1;
FastRunningMedian<10> older_value;

// Kalman variables for filtering raw reading
float varRaw = 0.006833;
float varProcess = 2e-4;
float Pc = 0.0;
float G = 0.0;
float P = 1.0;
float Xp = 0.0;
float Zp = 0.0;
float Xe = 0.0;

void setup()
{
  //M5.Power.begin();
  // This code is from the M5StickC weighing example
  M5.begin();
  //M5.Lcd.setRotation(1);
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.drawString("SCALE", 80, 0, 4);

  scale.init();

  M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
  Serial.begin(115200);
}

int i = 0;

void loop()
{
  // doesn't need to be done in a loop...
  // or, at all, really.
  //M5.update();

  float raw_weight = scale.getGram(1);

  // Kalman filtering
  Pc = P + varProcess;
  G = Pc / (Pc + varRaw);
  P = (1 - G) * Pc;
  Xp = Xe;
  Zp = Xp;
  Xe = G * (raw_weight - Zp) + Xp;
  float kalman_filtered = Xe;

  scale.setScale(current_scale);

  // hampel filtering
  hampel_filter.addValue(raw_weight);

  slow_filter_1.addValue(raw_weight);

  float hampel_filtered = hampel_filter.getMedian();
  float slow_filtered_1 = slow_filter_1.getMedian();

  older_value.addValue(slow_filtered_1);

  // auto-zeroing
  float value_from_olde = older_value.getOldestValue();

  if(abs(slow_filtered_1 - value_from_olde) < auto_zero_eps)
  {
    // re-tare scale
    //adjust_tare(slow_filtered_1 - value_from_olde);
  }

  // kalman_filtered is the kalman-filtered weight, hampel_filtered is the hampel-filtered one.
  // Turns out that they both have their strengths, and also weaknesses:

  // Hampel is very good for static, settled weights. It barely fluctuates and settles reasonably
  // fast when adding a static weight to the scale. However, it doesn't track as well when
  // adding weight to the scale. But it also has the benefit of not showing intermediate values, so if
  // you add a couple of beans, the value jumps from 0 to the true value, with no (or almost no) steps
  // in-between, thus the uncertainty whether the value has settled is reduced.

  // Kalman is not so good for that static case. It fluctuates more, and when modifying the varProcess parameter,
  // it fluctuates less but then the delay is too much to bear. However, it's very good at tracking weight
  // as it is added.

  // For the purposes of this scale, it looks like the Kalman filter is best. Maybe it can be combined with a very
  // small hampel filter as the sensor readings are sometimes very far off. Looks like the result is
  // adequate to get a flow rate measurement.

  // Another approach would be to use the Kalman-filtered value for flow-rate measurement and end-of-shot measurement,
  // and use the Hampel-filtered value for displaying on the scale. This would have the benefit of being able to use
  // a large hampel filter, thus the displayed value should be very stable (good for weighing beans and final shot
  // weight), and a 2-3s delay there isn't a problem.

  // I think it will be hard to improve on that, as this is a 1d measurement and there's no underlying model.


  // For displaying a graph in the console logger
  Serial.printf("filtered=%8.2f,kalman=%8.2f,slow1=%8.3f,raw=%8.2f\n", hampel_filtered, kalman_filtered, slow_filtered_1, raw_weight);

  M5.Lcd.setCursor(40, 30, 4);
  M5.Lcd.fillRect(0, 30, 160, 30, TFT_BLACK);
  M5.Lcd.printf("%.2f g", hampel_filtered);
}
