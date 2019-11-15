// This is code for an espresso scale. Think Acaia Lunar, only not ridiculously expensive (In the end, building this
// project will probably be more expensive than even the Lunar. But also much more fun!).

// All code under GPL v2 (see LICENSE), except where other licenses apply.

#include <M5Stack.h>
//#include <WiFi.h>
#include "HX711.h"
#include "HampelFilter.h"
#include "FlowMeter.h"
#include <EEPROM.h>
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

// [g] - if the slow-filter-median differs from the dynamic-mode-switch-filter by that amount, we can
// safely assume that the user has added some weight at once. This means that it's better to purge
// the slow filter as it will settle faster that way.
// This should only affect the static mode, and as such this threshold can be much lower than the difference
// you'd expect when pouring a shot.
float static_reset_thres = 0.9;

// [g] shot-start-threshold-min
// depends on the scale being zeroed
float shot_start_thres_min = 0.2;
float shot_start_thres_max = 2.5;
float shot_start_max_incr = 2.0;

FastRunningMedian<6> outlier_rejection_filter;
FastRunningMedian<16> hampel_filter;
FastRunningMedian<100> static_value_filter; // slowest filter, therefore also most accurate, good for drift compensation
FastRunningMedian<5> quick_settle_filter_trig; // this may need to be adjusted down to 4 or 3 to improve settling time
FastRunningMedian<3> quick_settle_filter_val; // this may need to be adjusted down to 4 or 3 to improve settling time
FastRunningMedian<17> shot_start_filter; // this may need to be adjusted up to make shot-start detection more reliable
FastRunningMedian<17> shot_end_filter; // this may need to be adjusted up to make shot-end detection more reliable

FlowMeter flow_meter;

// keeps track of shot-start-time
unsigned long shot_start_millis = 0;
unsigned long shot_end_millis = 0;

// [1] if that many samples lie between shot_start_thres_min and shot_start_thres_max, we assume the shot has started
unsigned int shot_start_cnt_thres = 8;
unsigned int shot_start_cnt;
uint8_t shot_running = 0;

// [g] - if the dynamic weight differs from the shot_end_filter median by less than this amount, count the shot as ended
float shot_end_thres = 0.5;
// [s] - the shot lasts at least that long, don't end it before!
float shot_min_time = 10;

unsigned int shot_end_cnt_thres = 20;
unsigned int shot_end_cnt;

int8_t battery_level = 0;

unsigned long startup_time;

size_t _eep_size = 10;
int _eep_addr_scale = 0;

float _scale_scale;
long _scale_offset = 0;
float _scale_tare = 0.0f;

void setup()
{
  M5.begin();
  M5.Power.begin();

  Serial.begin(115200);

  EEPROM.begin(_eep_size);
  _scale_scale = EEPROM.readFloat(_eep_addr_scale);
  if(isnan(_scale_scale) || _scale_scale < 800 || _scale_scale > 1000){
    _scale_scale = 890.0f;
  }
  Serial.printf("scale factor read as %f\n", _scale_scale);

  battery_level = M5.Power.getBatteryLevel();

  M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Lcd.setTextDatum(TL_DATUM);
  scale.init();

  startup_time = millis();
  //btStop();
  //WiFi.mode(WIFI_OFF);
}

float getGramUntared(long raw)
{
  return (raw - _scale_offset) / _scale_scale;
}

float getGram(long raw)
{
  return ((raw - _scale_offset) / _scale_scale) - _scale_tare;
}

float getGramDiff(long raw)
{
  return raw / _scale_scale;
}

long previous_raw = 0;
long display_lp = 0;

int fast_settle = 0;

void loop()
{
  M5.update();

  unsigned long ms = millis();
  long very_raw_weight = scale.getValue();

  // crude outlier rejection - we can't measure more than 1000g anyway
  if(abs(getGramDiff(very_raw_weight - previous_raw)) > 1000)
  {
    very_raw_weight = previous_raw;
  }
  previous_raw = very_raw_weight;

  outlier_rejection_filter.addValue(very_raw_weight);
  long outlier_rejected_val = outlier_rejection_filter.getMedian();

  hampel_filter.addValue(outlier_rejected_val);
  static_value_filter.addValue(outlier_rejected_val);

  quick_settle_filter_val.addValue(outlier_rejected_val);
  quick_settle_filter_trig.addValue(outlier_rejected_val);

  int tareMode = 20;
  static int scale_calibrated = 0;

  if(M5.BtnC.pressedFor(1500) && !scale_calibrated)
  {
    // the calibration weight is a multiple of 10g, use the next weight to 10g
    // which may cause mis-calibration, but usually works out fine
    // for this to work, the scale must be tared and the scale factor must be
    // at least somewhat similar to the real scale factor

    float current_gram = getGram(display_lp);
    int current_gram_tens = (int)((current_gram + 5.0f) / 10.0f);

    if(current_gram_tens < 1 || current_gram_tens > 10)
    {
      current_gram_tens = 2;
    }
    float scale_calib = 10.0f * current_gram_tens;

    _scale_scale = (display_lp - _scale_offset) / (scale_calib + _scale_tare);
    EEPROM.writeFloat(_eep_addr_scale, _scale_scale);
    EEPROM.commit();
    Serial.printf("scale calibrated, new scale factor: %f with calib weight %f\n", _scale_scale, scale_calib);
    Serial.printf("scale calibrated, new scale value: %f\n", getGram(display_lp));

    scale_calibrated = 1;
  }

  if(M5.BtnC.wasReleased() || (startup_time + 2000 > ms))
  {
    if(scale_calibrated)
    {
      scale_calibrated = 0;
    }
    else 
    {
      _scale_tare = getGramUntared(display_lp);
    }
  }

  long static_median = static_value_filter.getMedian();
  long hampel_median = hampel_filter.getMedian();
  // check whether we have a (relatively) recent dynamic value which is in stark contrast to the slow FastRunningMedian
  long quick_settle_median = quick_settle_filter_trig.getMedian();
  if(abs(getGramDiff(quick_settle_median - static_median)) > static_reset_thres)
  {
    long quick_settle_median_val = quick_settle_filter_val.getMedian();
    quick_settle_filter_val.clear(quick_settle_median_val);
    quick_settle_filter_trig.clear(quick_settle_median_val);
    static_value_filter.clear(quick_settle_median_val);
    hampel_filter.clear(quick_settle_median_val);

    fast_settle = 23;
  }

  double display_lp_d = 0.95;
  if(fast_settle > 0)
  {
    display_lp_d = 0.95/fast_settle;

    if(display_lp_d < 0.1)
    {
      display_lp_d = 0;
    }

    fast_settle--;
  }

  display_lp = display_lp * display_lp_d + (outlier_rejected_val * (1.0 - display_lp_d));

  long hampel_filtered = hampel_filter.getMedian();

  long shot_detection_current = hampel_filtered;
  shot_start_filter.addValue(hampel_filtered);
  shot_end_filter.addValue(hampel_filtered); 

  if(!shot_running)
  {
    long shot_start_filter_median = shot_start_filter.getMedian();
    if(
      getGramDiff(shot_detection_current - shot_start_filter_median) > shot_start_thres_min && 
      getGramDiff(shot_detection_current - shot_start_filter_median) < (shot_start_thres_max * (shot_start_cnt + 1)) &&
      getGram(shot_start_filter_median) < (shot_start_max_incr * (shot_start_cnt + 1)))
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

  if((abs(getGramDiff(shot_end_filter.getMedian() - shot_detection_current)) < shot_end_thres) && shot_running && 
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
    shot_end_weight = getGram(outlier_rejected_val);
  }

  flow_meter.addValue(ms, getGram(outlier_rejected_val));

  // debugging & tweaking
  //Serial.printf("filtered=%f,vraw=%f,raw=%f,shot_start_cnt=%d,shot_running=%d,shot_start_millis=%ld,shot_end_median=%f,shot_end_cnt=%d,shot_end_millis=%ld,shot_time=%ld,flow=%f,az=%f,az_l=%f\n", 
  //display_lp, very_raw_weight, raw_weight, shot_start_cnt, shot_running, shot_start_millis, shot_end_filter.getMedian(), shot_end_cnt, shot_end_millis, shot_time, flow_meter.getCurrentFlow(),
  //auto_zero_filter_fast.getMedian(), auto_zero_last);

  Serial.printf("%f,%f,%f,%d,%d,%d\n", 
  getGram(very_raw_weight), getGram(outlier_rejected_val), getGram(display_lp), fast_settle, shot_start_cnt, shot_end_cnt);

  float filtered_display_med = round(10.0 * getGram(display_lp)) / 10.0;
  if(filtered_display_med <= 0 && filtered_display_med > -0.1)
  {
    filtered_display_med = 0.0f;
  }

  M5.Lcd.setCursor(40, 30);
  M5.Lcd.setTextSize(5);

  M5.Lcd.fillRect(0, 0, 320, 240, TFT_BLACK);
  //M5.Lcd.printf("%6.2f %d", filtered_display_med, touchRead(2));
  M5.Lcd.printf("%6.2f", filtered_display_med);

  M5.Lcd.setCursor(40, 180);
  M5.Lcd.setTextSize(2);
  if(shot_running){
    //M5.Lcd.printf("%5.1f - %5.3f g/s", (ms - shot_start_millis) / 1000.0, flow_meter.getCurrentFlow());
    M5.Lcd.printf("%6.1f", (ms - shot_start_millis) / 1000.0);
  }
  else if(shot_time > 0){
    M5.Lcd.printf("%6.1f - %5.3f g/s total", (shot_time) / 1000.0, shot_end_weight * 1000.0 / shot_time);
  }


}
