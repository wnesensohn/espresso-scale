// This is code for an espresso scale. Think Acaia Lunar, only not ridiculously expensive (In the end, building this
// project will probably be more expensive than even the Lunar. But also much more fun!).

// All code under GPL v2 (see LICENSE), except where other licenses apply.

#include <M5Stack.h>
#include <WiFi.h>
#include "HX711.h"
#include "HampelFilter.h"
//#include <TFT_eSPI.h>
//#include <SPI.h>

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
float static_reset_thres = 0.4;

// [g] shot-start-threshold-min - uses dynamic, kalman-filtered value for determing whether a shot has started
// depends on the scale being zeroed
float shot_start_thres_min = 0.2;
float shot_start_thres_max = 10.0;
float shot_start_max_incr = 0.5;

// [1] if that many samples lie between shot_start_thres_min and shot_start_thres_max, we assume the shot has started
unsigned int shot_start_cnt_thres = 5;
unsigned int shot_start_cnt;
uint8_t shot_running = 0;

FastRunningMedian<5> outlier_rejection_filter;
FastRunningMedian<16> hampel_filter;
FastRunningMedian<100> static_value_filter;
FastRunningMedian<64> display_filter;
FastRunningMedian<10> older_value;
FastRunningMedian<5> quick_settle_filter_trig; // this may need to be adjusted down to 4 or 3 to improve settling time
FastRunningMedian<3> quick_settle_filter_val; // this may need to be adjusted down to 4 or 3 to improve settling time
FastRunningMedian<13> shot_start_filter; // this may need to be adjusted up to make shot-start detection more reliable
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
float shot_end_thres = 0.5;
// [s] - the shot lasts at least that long, don't end it before!
float shot_min_time = 10;

unsigned int shot_end_cnt_thres = 15;
unsigned int shot_end_cnt;

int8_t battery_level = 0;

void setup()
{
  M5.begin();
  M5.Power.begin();

  //Wire.begin();
  battery_level = M5.Power.getBatteryLevel();
  Wire.~TwoWire();

  //battery_level = 0;


  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.printf("battery: %d", battery_level);

  //M5.Lcd.drawString("SCALE", 80, 0, 4);

  scale.init();

  //M5.Lcd.printf("SCALE INITIALIZED");

  //M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  Serial.begin(115200);

  //btStop();
  //WiFi.mode(WIFI_OFF);
}

int i = 0;

float previous_raw = 0;
float display_lp = 0;

int fast_settle = 0;

void loop()
{
  M5.update();

  if(M5.BtnC.wasPressed())
  {
    scale.tare(10);
  }

  float raw_weight = scale.getGram(1);

  // crude outlier rejection - we can't measure more than 1000g anyway
  if(abs(raw_weight - previous_raw) > 1000)
  {
    raw_weight = previous_raw;
  }
  previous_raw = raw_weight;

  outlier_rejection_filter.addValue(raw_weight);
  float outlier_rejected_val = outlier_rejection_filter.getMedian();

  // make outlier-rejected val the base of everything down below, because we don't want these pesky spikes
  raw_weight = outlier_rejected_val;

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
  display_filter.addValue(raw_weight);

  quick_settle_filter_val.addValue(raw_weight);
  quick_settle_filter_trig.addValue(raw_weight);

  // check whether we have a (relatively) recent dynamic value which is in stark contrast to the slow FastRunningMedian
  float quick_settle_median = quick_settle_filter_trig.getMedian();
  if(abs(quick_settle_median - static_value_filter.getMedian()) > static_reset_thres)
  {
    float quick_settle_median_val = quick_settle_filter_val.getMedian();
    static_value_filter.clear(quick_settle_median_val);
    display_filter.clear(quick_settle_median_val);
    hampel_filter.clear(quick_settle_median_val);
    fast_settle = 23;
  }

  float display_lp_d = 0.98;
  if(fast_settle > 0)
  {
    display_lp_d = 0.98/fast_settle;

    if(display_lp_d < 0.1)
    {
      display_lp_d = 0;
    }

    fast_settle--;
  }

  display_lp = display_lp * display_lp_d + (raw_weight * (1.0 - display_lp_d));

  float hampel_filtered = hampel_filter.getMedian();
  float slow_filtered_1 = static_value_filter.getMedian();
  float display_filtered = display_filter.getMedian();

  // we use kalman-filter for all dynamic purposes, but for static ones, we use hampel_filtered
  float shot_detection_current = hampel_filtered;
  shot_start_filter.addValue(hampel_filtered);
  shot_end_filter.addValue(hampel_filtered); 

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
    float shot_start_filter_median = shot_start_filter.getMedian();
    if(
      abs(shot_start_filter_median - shot_detection_current) > shot_start_thres_min && 
      abs(shot_start_filter_median - shot_detection_current) < (shot_start_thres_max * (shot_start_cnt + 1)) &&
      shot_start_filter_median < (shot_start_max_incr * (shot_start_cnt + 1)))
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
    if(shot_end_millis == 0 || (millis() - shot_end_millis) > 10000)
    {
      shot_start_cnt = 0;
      shot_running = 1;
      shot_start_millis = millis();
    }
  }

  static unsigned int shot_end_real = 0;

  if((abs(shot_end_filter.getMedian() - shot_detection_current) < shot_end_thres) && shot_running && 
    ((millis() - shot_start_millis) > shot_min_time))
  {
    shot_end_cnt++;
  }
  else
  {
    shot_end_real = millis();
    shot_end_cnt = 0;
  }

  static unsigned long shot_time = 0;

  if(shot_end_cnt >= shot_end_cnt_thres)
  {
    shot_end_cnt = 0;
    shot_running = 0;
    shot_end_millis = shot_end_real;
    shot_time = shot_end_millis - shot_start_millis;
  }

  // debugging & tweaking
  Serial.printf("filtered=%f,kalman=%f,slow1=%f,raw=%f,shot_start_cnt=%d,shot_running=%d,shot_start_millis=%ld,shot_end_median=%f,shot_end_cnt=%d,shot_end_millis=%ld,shot_time=%ld\n", 
  hampel_filtered, kalman_filtered, slow_filtered_1, raw_weight, shot_start_cnt, shot_running, shot_start_millis, shot_end_filter.getMedian(), shot_end_cnt, shot_end_millis, shot_time);

  float filtered_display_fast = round(20.0 * hampel_filtered) / 20.0;
  float filtered_display_med = round(20.0 * display_lp) / 20.0;
  float filtered_display_slow = round(20.0 * display_filtered) / 20.0;

  M5.Lcd.setCursor(40, 30, 4);
  M5.Lcd.fillRect(0, 30, 320, 100, TFT_BLACK);
  M5.Lcd.printf("%6.2f %6.2f %6.2f", filtered_display_slow, filtered_display_med, filtered_display_fast, touchRead(2));

  M5.Lcd.setCursor(40, 60, 4);
  if(shot_running){
    M5.Lcd.printf("%5.1f", (millis() - shot_start_millis) / 1000.0);
  }
  else if(shot_time > 0){
    M5.Lcd.printf("%5.1f", (shot_time) / 1000.0);
  }

}
