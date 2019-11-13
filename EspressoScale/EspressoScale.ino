// This is code for an espresso scale. Think Acaia Lunar, only not ridiculously expensive (In the end, building this
// project will probably be more expensive than even the Lunar. But also much more fun!).

// All code under GPL v2 (see LICENSE), except where other licenses apply.

#include <M5Stack.h>
//#include <WiFi.h>
#include "HX711.h"
#include "HampelFilter.h"
#include "FlowMeter.h"
//#include <TFT_eSPI.h>
//#include <SPI.h>

//HX711 scale(22, 21); // GROVE A
HX711 scale(36, 26); // GROVE B
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
// * tare after filtering, as we have to reset filters to tare, which is sub-optimal and makes tare work not reliably enough
// * calibration with known weights
// * auto shut-down (not the annoying kind, something like "after 5 minutes after last weight change of more than 1g/30s")
// * handle hx711 communication by using serial interface (don't know if possible yet) and DMA (low prio)
// X see if we can get a reliable capacity reading of the LiPo somehow to display when we need more juice - yes, works!
// X switch to M5Stack - done

// TODO: Add a method to calibrate the scale by using a pre-defined weight
float current_scale = 890.0/2;

// [g] - if the slow-filter-median differs from the dynamic-mode-switch-filter by that amount, we can
// safely assume that the user has added some weight at once. This means that it's better to purge
// the slow filter as it will settle faster that way.
// This should only affect the static mode, and as such this threshold can be much lower than the difference
// you'd expect when pouring a shot.
float static_reset_thres = 0.7;

// [g] shot-start-threshold-min
// depends on the scale being zeroed
float shot_start_thres_min = 0.7;
float shot_start_thres_max = 2.0;
float shot_start_max_incr = 2.0;

// [1] if that many samples lie between shot_start_thres_min and shot_start_thres_max, we assume the shot has started
unsigned int shot_start_cnt_thres = 6;
unsigned int shot_start_cnt;
uint8_t shot_running = 0;

FastRunningMedian<11> outlier_rejection_filter;
FastRunningMedian<16> hampel_filter;
FastRunningMedian<100> static_value_filter; // slowest filter, therefore also most accurate, good for drift compensation
FastRunningMedian<5> quick_settle_filter_trig; // this may need to be adjusted down to 4 or 3 to improve settling time
FastRunningMedian<3> quick_settle_filter_val; // this may need to be adjusted down to 4 or 3 to improve settling time
FastRunningMedian<13> shot_start_filter; // this may need to be adjusted up to make shot-start detection more reliable
FastRunningMedian<13> shot_end_filter; // this may need to be adjusted up to make shot-end detection more reliable

// non-compensated filters
FastRunningMedian<32> auto_zero_filter_fast; // fast(er) auto-zero filter
int auto_zero_distance = 5;
float auto_zero_eps = 0.05;

FlowMeter flow_meter;

// keeps track of shot-start-time
unsigned long shot_start_millis = 0;
unsigned long shot_end_millis = 0;

// [g] - if the dynamic weight differs from the shot_end_filter median by less than this amount, count the shot as ended
float shot_end_thres = 0.2;
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
  // I2C is wired on the same port as where we get our data from hx711. That's pretty unfortunate,
  // but maybe we should just re-wire this. Anyhow, it's not a problem if we just destruct TwoWire
  // here. It delays startup by a bit though (hx711 settling is still the worst part)
  Wire.~TwoWire();

  //battery_level = 0;


  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.setTextDatum(MC_DATUM);
  //M5.Lcd.printf("battery: %d", battery_level);

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

float auto_zero_offset = 0;

void loop()
{
  M5.update();

  float raw_weight = scale.getGram(1);
  unsigned long ms = millis();
  float very_raw_weight = raw_weight;

  // crude outlier rejection - we can't measure more than 1000g anyway
  if(abs(raw_weight - previous_raw) > 1000)
  {
    raw_weight = previous_raw;
  }
  previous_raw = raw_weight;

  outlier_rejection_filter.addValue(raw_weight);
  float outlier_rejected_val = outlier_rejection_filter.getMedian();

  // make outlier-rejected val the base of everything down below, because we don't want these pesky spikes
  auto_zero_filter_fast.addValue(outlier_rejected_val);

  static int auto_zero_act = 0;
  static float auto_zero_last = 0;
  static float auto_zero_first = 0;

  static float auto_zero_flt = 0;
  auto_zero_flt = (auto_zero_flt * 0.95) + (outlier_rejected_val * 0.05);

  if(++auto_zero_act == auto_zero_distance)
  {
    auto_zero_act = 0;
    //float auto_zero_fast_median = auto_zero_filter_fast.getMedian();
    float auto_zero_fast_median = auto_zero_flt;

    float auto_zero_diff = auto_zero_fast_median - auto_zero_last;
    float auto_zero_abs = abs(auto_zero_diff);

    if(auto_zero_abs < auto_zero_eps)
    {
      Serial.printf("zeroed with %f\n",  auto_zero_diff);
      auto_zero_offset = auto_zero_fast_median - auto_zero_first;
    }
    else
    {
      auto_zero_first = auto_zero_fast_median;
      Serial.printf("not auto zeroed with %f\n",  auto_zero_diff);
    }

    auto_zero_last = auto_zero_fast_median;
  }

  raw_weight = outlier_rejected_val + auto_zero_offset;

  scale.setScale(current_scale);

  hampel_filter.addValue(raw_weight);
  static_value_filter.addValue(raw_weight);

  quick_settle_filter_val.addValue(raw_weight);
  quick_settle_filter_trig.addValue(raw_weight);

  int tareMode = 0;

  if(M5.BtnC.wasPressed())
  {
    tareMode = 1;
    scale.tareWithValue(display_lp);
  }

  float static_median = static_value_filter.getMedian();
  float hampel_median = hampel_filter.getMedian();
  // check whether we have a (relatively) recent dynamic value which is in stark contrast to the slow FastRunningMedian
  float quick_settle_median = quick_settle_filter_trig.getMedian();
  if(abs(quick_settle_median - static_median) > static_reset_thres ||
    tareMode)
  {
    float quick_settle_median_val = quick_settle_filter_val.getMedian();
    quick_settle_filter_val.clear(quick_settle_median_val);
    quick_settle_filter_trig.clear(quick_settle_median_val);
    static_value_filter.clear(quick_settle_median_val);
    hampel_filter.clear(quick_settle_median_val);

    if(!tareMode){
      fast_settle = 23;
    }
    else{
      fast_settle = 1;
    }
  }

  float display_lp_d = 0.99;
  if(fast_settle > 0)
  {
    display_lp_d = 0.99/fast_settle;

    if(display_lp_d < 0.1 || tareMode)
    {
      display_lp_d = 0;
    }

    fast_settle--;
  }

  display_lp = display_lp * display_lp_d + (raw_weight * (1.0 - display_lp_d));

  float hampel_filtered = hampel_filter.getMedian();

  // we use kalman-filter for all dynamic purposes, but for static ones, we use hampel_filtered
  float shot_detection_current = hampel_filtered;
  shot_start_filter.addValue(hampel_filtered);
  shot_end_filter.addValue(hampel_filtered); 

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
    if(shot_end_millis == 0 || (ms - shot_end_millis) > 10000)
    {
      shot_start_cnt = 0;
      shot_running = 1;
      shot_start_millis = ms;
      flow_meter.clear();
    }
  }

  static unsigned int shot_end_real = 0;

  if((abs(shot_end_filter.getMedian() - shot_detection_current) < shot_end_thres) && shot_running && 
    ((ms - shot_start_millis) > shot_min_time))
  {
    shot_end_cnt++;
  }
  else
  {
    shot_end_real = ms;
    shot_end_cnt = 0;
  }

  static unsigned long shot_time = 0;
  static float shot_end_weight = 0.0f;
  
  if(shot_end_cnt >= shot_end_cnt_thres)
  {
    shot_end_cnt = 0;
    shot_running = 0;
    shot_end_millis = shot_end_real;
    shot_time = shot_end_millis - shot_start_millis;
    shot_end_weight = raw_weight;
  }

  flow_meter.addValue(ms, raw_weight);

  // debugging & tweaking
  Serial.printf("filtered=%f,vraw=%f,raw=%f,shot_start_cnt=%d,shot_running=%d,shot_start_millis=%ld,shot_end_median=%f,shot_end_cnt=%d,shot_end_millis=%ld,shot_time=%ld,flow=%f,az=%f,az_l=%f\n", 
  display_lp, very_raw_weight, raw_weight, shot_start_cnt, shot_running, shot_start_millis, shot_end_filter.getMedian(), shot_end_cnt, shot_end_millis, shot_time, flow_meter.getCurrentFlow(),
  auto_zero_filter_fast.getMedian(),
  auto_zero_last
  );


  float filtered_display_med = round(20.0 * display_lp) / 20.0;

  M5.Lcd.setCursor(40, 30, 4);
  M5.Lcd.fillRect(0, 30, 320, 100, TFT_BLACK);
  M5.Lcd.printf("%6.2f %d", filtered_display_med, touchRead(2));

  M5.Lcd.setCursor(40, 60, 4);
  if(shot_running){
    M5.Lcd.printf("%5.1f - %5.3f g/s", (ms - shot_start_millis) / 1000.0, flow_meter.getCurrentFlow());
  }
  else if(shot_time > 0){
    M5.Lcd.printf("%5.1f - %5.3f g/s total", (shot_time) / 1000.0, shot_end_weight * 1000.0 / shot_time);
  }


}
