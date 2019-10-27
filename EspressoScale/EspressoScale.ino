// This is code for an espresso scale. Think Acaia Lunar, only not ridiculously expensive (In the end, building this
// project will probably be more expensive than even the Lunar. But also much more fun!).

// All code under GPL v2 (see LICENSE), except where other licenses apply.

#include <M5Stack.h>
#include <WiFi.h>
#include "HX711.h"
#include "HampelFilter.h"

HX711 scale(22, 21); // GROVE A
//HX711 scale(36, 26); // GROVE B
//HX711 scale(16, 17); // GROVE C

// TODO (general): 
// * flow-rate measurement in g/s
// * shot-time (if need be compensate for filter lag, but probably not necessary, after all it's just espresso ffs)
//   using dynamic-filtered (kalman-filtered) value
// * double-buffer for display to prevent annoying flickering
// * auto-zeroing (zero large weight-changes in small time-increments)
// * zero-tracking (zero very small weight-changes happening over a long timespan)
//   to compensate drift. There's not a lot of drift, and it's temperature related, but it's enough to be annoying.
//   (actually, it's so low that we can do without drift-compensation. It might matter if using the scale in direct sunlight, which I don't)
// * calibration with known weights
// * auto shut-down (not the annoying kind, something like "after 5 minutes after last weight change of more than 1g/30s")
// * handle hx711 communication by using serial interface (don't know if possible yet) and DMA (low prio)
// X see if we can get a reliable capacity reading of the LiPo somehow to display when we need more juice - yes, works!
// X switch to M5Stack - done

// TODO: Add a method to calibrate the scale by using a pre-defined weight
float current_scale = 890.0/2;
float auto_zero_eps = 0.005;

// [g] - if the slow-filter-median differs from the dynamic-mode-switch-filter by that amount, we can
// safely assume that the user has added some weight at once. This means that it's better to purge
// the slow filter as it will settle faster that way.
// This should only affect the static mode, and as such this threshold can be much lower than the difference
// you'd expect when pouring a shot.
float static_reset_thres = 4.0;

// [g] shot-start-threshold-min - uses dynamic, kalman-filtered value for determing whether a shot has started
// depends on the scale being zeroed
float shot_start_thres_min = 0.1;
float shot_start_thres_max = 2.0;

// [1] if that many samples lie between shot_start_thres_min and shot_start_thres_max, we assume the shot has started
unsigned int shot_start_cnt_thres = 4;
unsigned int shot_start_cnt;
uint8_t shot_running = 0;

FastRunningMedian<16> hampel_filter;
FastRunningMedian<100> static_value_filter;
FastRunningMedian<10> older_value;
FastRunningMedian<5> quick_settle_filter; // this may need to be adjusted down to 4 or 3 to improve settling time
FastRunningMedian<13> shot_end_filter; // this may need to be adjusted up to make shot-end detection more reliable

// Kalman variables for filtering raw reading
float varRaw = 0.006833;
float varProcess = 2e-4;
float Pc = 0.0;
float G = 0.0;
float P = 1.0;
float Xp = 0.0;
float Zp = 0.0;
float Xe = 0.0;

// keeps track of shot-start-time
unsigned long shot_start_millis = 0;
unsigned long shot_end_millis = 0;

// [g] - if the dynamic weight differs from the shot_end_filter median by less than this amount, count the shot as ended
float shot_end_thres = 1.0;

void setup()
{
  //M5.Power.begin();
  // This code is from the M5StickC weighing example
  M5.begin();
  Wire.begin();
  //M5.Lcd.setRotation(1);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.drawString("SCALE", 80, 0, 4);

  //scale.init();

  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  Serial.begin(115200);

  btStop();
  WiFi.mode(WIFI_OFF);
}

int i = 0;

void loop()
{
  // doesn't need to be done in a loop...
  // or, at all, really.
  //M5.update();

  //float raw_weight = scale.getGram(1);
  float raw_weight = 19;


  // Kalman filtering
  Pc = P + varProcess;
  G = Pc / (Pc + varRaw);
  P = (1 - G) * Pc;
  Xp = Xe;
  Zp = Xp;
  Xe = G * (raw_weight - Zp) + Xp;
  float kalman_filtered = Xe;

  scale.setScale(current_scale);

  hampel_filter.addValue(raw_weight);
  static_value_filter.addValue(raw_weight);
  quick_settle_filter.addValue(raw_weight);
  shot_end_filter.addValue(kalman_filtered); // we use kalman-filter for all dynamic purposes

  // check whether we have a (relatively) recent dynamic value which is in stark contrast to the slow FastRunningMedian
  float quick_settle_median = quick_settle_filter.getMedian();
  if(abs(quick_settle_median - static_value_filter.getMedian()) > static_reset_thres)
  {
    static_value_filter.clear(quick_settle_median);
    hampel_filter.clear(quick_settle_median);
  }

  float hampel_filtered = hampel_filter.getMedian();
  float slow_filtered_1 = static_value_filter.getMedian();

  older_value.addValue(slow_filtered_1);

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


  if(!shot_running)
  {
    if(kalman_filtered > shot_start_thres_min && kalman_filtered < shot_start_thres_max)
    {
      shot_start_cnt++;
    }
    else
    {
      shot_start_cnt = 0;
    }
  }

  if(shot_start_cnt >= shot_start_cnt_thres)
  {
    shot_start_cnt = 0;
    shot_running = 1;
    shot_start_millis = millis();
  }

  if(abs(shot_end_filter.getMedian() - kalman_filtered) < shot_end_thres)
  {
    shot_end_millis = millis();
    shot_running = 0;
  }

  // debugging & tweaking
  Serial.printf("filtered=%8.2f,kalman=%8.2f,slow1=%8.3f,raw=%8.2f,shot_start_cnt=%d,shot_running=%d,shot_start_millis=%ld,shot_end_median=%f,shot_end_millis=%ld\n", 
  hampel_filtered, kalman_filtered, slow_filtered_1, raw_weight, shot_start_cnt, shot_running, shot_start_millis, shot_end_filter.getMedian(), shot_end_millis);

  M5.Lcd.setCursor(40, 30, 4);
  M5.Lcd.fillRect(0, 30, 320, 30, TFT_BLACK);
  M5.Lcd.printf("%.2f g t: %d b: %d c: %d", hampel_filtered, touchRead(2), M5.Power.getBatteryLevel(), M5.Power.isCharging());
}
