// This is code for an espresso scale. Think Acaia Lunar, only not ridiculously expensive (In the end, building this
// project will probably be more expensive than even the Lunar. But also much more fun!).

// All code under GPL v2 (see LICENSE), except where other licenses apply.

#include <M5Stack.h>
#include <WiFi.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"
#include "HampelFilter.h"
#include "FlowMeter.h"
#include <EEPROM.h>

NAU7802 scale;

//HX711 scale(22, 21); // GROVE A
//HX711 scale(36, 26); // GROVE B
//HX711 scale(16, 17); // GROVE C

// TODO (general): 
// X flow-rate measurement in g/s
// X shot-time (if need be compensate for filter lag, but probably not necessary, after all it's just espresso ffs)
//   using dynamic-filtered (kalman-filtered) value
// X double-buffer for display to prevent annoying flickering
// X auto-zeroing (zero large weight-changes in small time-increments)
// * zero-tracking (zero very small weight-changes happening over a long timespan)
//   to compensate drift. There's not a lot of drift, and it's temperature related, but it's enough to be annoying.
//   (actually, it's so low that we can do without drift-compensation. It might matter if using the scale in direct sunlight, which I don't)
// X tare after filtering, as we have to reset filters to tare, which is sub-optimal and makes tare work not reliably enough
// X calibration with known weights
// X auto shut-down (not the annoying kind, something like "after 5 minutes after last weight change of more than 1g/30s")
// X handle hx711 communication by using serial interface (don't know if possible yet) and DMA (low prio)
// X see if we can get a reliable capacity reading of the LiPo somehow to display when we need more juice - yes, works!
// X switch to M5Stack - done

// [g] - if the slow-filter-median differs from the dynamic-mode-switch-filter by that amount, we can
// safely assume that the user has added some weight at once. This means that it's better to purge
// the slow filter as it will settle faster that way.
// This should only affect the static mode, and as such this threshold can be much lower than the difference
// you'd expect when pouring a shot.
float static_reset_thres = 0.10;

// [g] shot-start-threshold-min
// depends on the scale being zeroed
float shot_start_thres_min = 0.2;
float shot_start_thres_max = 6.0;
float shot_start_max_incr = 1.0;

FastRunningMedian<3> outlier_rejection_filter;
FastRunningMedian<100> static_value_filter; // slowest filter, therefore also most accurate, good for drift compensation
FastRunningMedian<5> quick_settle_filter_trig; // this may need to be adjusted down to 4 or 3 to improve settling time
FastRunningMedian<3> quick_settle_filter_val; // this may need to be adjusted down to 4 or 3 to improve settling time
FastRunningMedian<60> shot_end_filter; // this may need to be adjusted up to make shot-end detection more reliable

FastRunningMedian<90> pre_infusion_timer; // this may need to be adjusted up to make shot-end detection more reliable
FastRunningMedian<11> pre_infusion_flt; // this may need to be adjusted up to make shot-end detection more reliable

FastRunningMedian<80> auto_tare_flt; // this may need to be adjusted up to make auto-tare more reliable

float auto_tare_thres = 6000; // this is in 100ths of a gram
float zero_tare_thres = 10; // this is in 100ths of a gram
float auto_tare_thres_min = 3; // this is in 100ths of a gram

FlowMeter flow_meter;

// keeps track of shot-start-time
unsigned long shot_start_millis = 0;
unsigned long shot_end_millis = 0;

// [1] if that many samples lie between shot_start_thres_min and shot_start_thres_max, we assume the shot has started
unsigned int shot_start_cnt_thres = 40;
unsigned int shot_start_cnt_disp_thres = 5;
unsigned int shot_start_cnt;
static unsigned long shot_time = 0;
float shot_end_weight = 0.0f;

bool shot_running = 0;

// [g] - if the dynamic weight differs from the shot_end_filter median by less than this amount, count the shot as ended
float shot_end_thres = 0.1;
// [s] - the shot lasts at least that long, don't end it before!
long shot_min_time = 7000;
long shot_min_time_disp = 2000;

unsigned int shot_end_cnt_thres = 20;
unsigned int shot_end_cnt;

int8_t battery_level = 0;

unsigned long startup_time;

size_t _eep_size = 10;
int _eep_addr_scale = 0;

float _scale_scale;
float _scale_tare = 0.0f;

uint16_t touch_base = 0;

void setup()
{
  M5.begin();
  M5.Power.begin();
  btStop();
  WiFi.mode(WIFI_OFF);
  Serial.begin(115200);
  EEPROM.begin(_eep_size);

  _scale_scale = EEPROM.readFloat(_eep_addr_scale);
  if(isnan(_scale_scale) || _scale_scale < 800 || _scale_scale > 1000){
    _scale_scale = 890.0f;
  }
  Serial.printf("scale factor read as %f\n", _scale_scale);

  // turn the backlight on after the screen has been cleared
  digitalWrite(TFT_BL, HIGH);
  pinMode(TFT_BL, OUTPUT);


  pinMode(2, INPUT);
  pinMode(12, INPUT);

  M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Lcd.setTextDatum(TL_DATUM);

  if (scale.begin() == false)
  {
    Serial.println("Scale not detected. Please check wiring. Freezing...");
    while (1);
  }
  Serial.println("Scale detected!");

  startup_time = millis();
  
  touchSetCycles(0xF000, 0x3000);
}

float getGramUntared(long raw)
{
  return raw;
}

float getGram(long raw)
{
  return ((raw - _scale_tare) / _scale_scale);
}

float getGramDiff(long raw)
{
  return raw / _scale_scale;
}

TFT_eSprite img = TFT_eSprite(&M5.Lcd);


void loop()
{
  battery_level = M5.Power.getBatteryLevel();
  bool isCharging = M5.Power.isCharging();

  static long pre_infusion_start = 0;
  static bool pre_infusion_freeze = false;
  static long pre_infusion_time = 0;
  static long display_lp = 0;
  static int fast_settle = 0;

  unsigned long ms = millis();

  pre_infusion_flt.addValue(touchRead(12));
  long pre_infusion_cur = pre_infusion_flt.getMedian();
  pre_infusion_timer.addValue(pre_infusion_cur);
  long pre_infusion_old = pre_infusion_timer.getOldestValue();

  static bool pre_infusion_raw_running = false;

  if((pre_infusion_old - pre_infusion_cur) > 3)
  {
    pre_infusion_raw_running = true;
  }
  else if((pre_infusion_cur - pre_infusion_old) > 3)
  {
    pre_infusion_raw_running = false;
  }

  if(!pre_infusion_freeze)
  {
    if(pre_infusion_raw_running)
    {
      pre_infusion_time = ms - pre_infusion_start;
    }
    else
    {
      pre_infusion_time = 0;
      pre_infusion_start = ms;
    }
  }

  static long very_raw_weight = 0;
  while(!scale.available());
  very_raw_weight = scale.getReading();

  M5.update();

  outlier_rejection_filter.addValue(very_raw_weight);
  long outlier_rejected_val = outlier_rejection_filter.getMedian();

  static_value_filter.addValue(outlier_rejected_val);

  quick_settle_filter_val.addValue(outlier_rejected_val);
  quick_settle_filter_trig.addValue(outlier_rejected_val);

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

    _scale_scale = (display_lp - _scale_tare) / scale_calib;
    EEPROM.writeFloat(_eep_addr_scale, _scale_scale);
    EEPROM.commit();
    Serial.printf("lp: %d, calib: %f, tare: %f\n", display_lp, scale_calib, _scale_tare);
    Serial.printf("scale calibrated, new scale factor: %f with calib weight %f\n", _scale_scale, scale_calib);
    Serial.printf("scale calibrated, new scale value: %f\n", getGram(display_lp));

    scale_calibrated = 1;
  }

  auto_tare_flt.addValue((long)(100.0f * getGram(display_lp)));

  static long significant_change_detect = 0;
  static long significant_change_millis = 0;
  if(abs(getGramDiff(significant_change_detect - display_lp)) > 0.5)
  {
    significant_change_detect = display_lp;
    significant_change_millis = ms;
    Serial.printf("significant change detected\n");
  }
  else if((ms - significant_change_millis) > (15 * 60 * 1000) && !isCharging)
  {
    // 15 minutes no activity - deep sleep!
    scale.powerDown();
    M5.Power.deepSleep(0);
  }

  static bool auto_tare_active = true;

  if(M5.BtnC.wasReleased())
  {
    if(scale_calibrated)
    {
      scale_calibrated = 0;
    }
    else 
    {
      auto_tare_active = !auto_tare_active;
    }
  }

  bool auto_tare = false;

  if(
    (abs(auto_tare_flt.getMedian() - auto_tare_flt.getNewestValue())) < auto_tare_thres_min &&
    (auto_tare_flt.getOldestValue() > auto_tare_thres) &&
    (auto_tare_flt.getNewestValue() > auto_tare_thres) &&
    auto_tare_active)
  {
    auto_tare = true;
  }
  else if(
    (abs(auto_tare_flt.getMedian() - auto_tare_flt.getNewestValue())) < auto_tare_thres_min &&
    (auto_tare_flt.getNewestValue() < zero_tare_thres) &&
    (auto_tare_flt.getNewestValue() > -zero_tare_thres))
  {
    // this is zero-tare, it should always be active as it's somehow annoying if zero drifts,
    // but other than being annoying it's inconsequential.
    _scale_tare = getGramUntared(display_lp);
    auto_tare_flt.clear(0);
  }

  if(M5.BtnA.wasReleased() || (startup_time + 1500 > ms) || auto_tare)
  {
      _scale_tare = getGramUntared(display_lp);

      // reset shot thingy
      shot_running = false;
      shot_start_cnt = 0;
      shot_end_cnt = 0;
      shot_end_weight = 0;
      shot_time = 0;
      flow_meter.clear();
      shot_end_millis = 0;

      pre_infusion_freeze = false;
      auto_tare = false;
      auto_tare_flt.clear(0);
  }

  long static_median = static_value_filter.getMedian();
  // check whether we have a (relatively) recent dynamic value which is in stark contrast to the slow FastRunningMedian
  long quick_settle_median = quick_settle_filter_trig.getMedian();
  float gram_diff = abs(getGramDiff(quick_settle_median - static_median));
  if(gram_diff > static_reset_thres)
  {
    long quick_settle_median_val = quick_settle_filter_val.getMedian();
    quick_settle_filter_val.clear(quick_settle_median_val);
    quick_settle_filter_trig.clear(quick_settle_median_val);
    static_value_filter.clear(quick_settle_median_val);

    fast_settle = (int)min(gram_diff * 4 / static_reset_thres, 15.0f);
  }

  double display_lp_d = 0.985;
  if(fast_settle > 0)
  {
    display_lp_d = 0.05;
    if(fast_settle < 8)
    {
      display_lp_d = 0.7;
    }
    if(fast_settle < 4)
    {
      display_lp_d = 0.8;
    }
    if(fast_settle < 2)
    {
      display_lp_d = 0.9;
    }
    fast_settle--;
  }

  display_lp = display_lp * display_lp_d + (outlier_rejected_val * (1.0 - display_lp_d));

  float display_weight = round(20.0 * getGram(display_lp)) / 20.0;
  // prevent displaying -0 
  if(display_weight < 0.05 && display_weight > -0.05)
  {
    display_weight = +0.0f;
  }

  bool blankWeight = startup_time + 700 > ms;
  if(blankWeight)
  {
    display_weight = +0.0f;
  }

  static float bean_weight = 0.0f;
  if(M5.BtnB.wasPressed())
  {
    if(bean_weight != 0.0f)
    {
      bean_weight = 0.0f;
    }
    else
    {
      bean_weight = display_weight;
    }
  }

  long shot_detection_current = display_lp;
  shot_end_filter.addValue(shot_detection_current); 

  static long shot_start_max = 0;
  static long shot_start_last = 0;

  float shot_start_diff = getGramDiff(shot_detection_current - shot_start_last);

  if(!shot_running)
  {
    if((abs(getGramDiff(shot_end_filter.getMedian() - shot_detection_current)) < shot_end_thres / 2))
    {
      // we don't really have a running shot as we'd end it right away, but be more strict
      // in evaluating this (the / 2 part)
      shot_start_millis = ms;
      shot_start_cnt = 0;
      shot_start_max = 0;
    }
    else if(
      getGram(shot_detection_current) > shot_start_thres_min &&
      getGram(shot_detection_current) < shot_start_thres_max &&
      shot_start_diff <= shot_start_max_incr
      )
    {
      shot_start_cnt++;
    }
    else
    {
      shot_start_millis = ms;
      shot_start_cnt = 0;
      shot_start_max = 0;
    }

    shot_start_last = shot_detection_current;
  }

  if(shot_start_cnt >= shot_start_cnt_thres &&
    shot_end_millis == 0)
  {
    shot_start_cnt = 0;
    shot_running = true;
    flow_meter.clear();
    pre_infusion_freeze = true;
  }

  static unsigned int shot_end_real = 0;
  static bool shot_invalid = false;

  if((abs(getGramDiff(shot_end_filter.getMedian() - shot_detection_current)) < shot_end_thres) && 
    shot_running)
  {
    shot_end_cnt++;
    if((ms - shot_start_millis) < shot_min_time)
    {
      shot_invalid = true;
    }
  }
  else
  {
    shot_end_real = ms;
    shot_end_cnt = 0;
    shot_invalid = false;
  }

  
  if(shot_end_cnt >= shot_end_cnt_thres)
  {
    shot_end_cnt = 0;
    shot_running = false;

    if(shot_invalid)
    {
      shot_end_millis = 0;
      shot_time = 0;
      shot_end_weight = 0;
    }
    else
    {
      shot_end_millis = shot_end_real;
      shot_time = shot_end_millis - shot_start_millis;
      shot_end_weight = display_weight;
    }
  }

  flow_meter.addValue(ms, getGram(outlier_rejected_val));

  Serial.printf("%f,%f,%f,%d,%f,%d,%d,%d\n", 
  getGram(very_raw_weight), getGram(outlier_rejected_val), getGram(display_lp), fast_settle, shot_start_diff, shot_start_cnt, shot_end_cnt, pre_infusion_old);

  float ratio = 0.0f;
  if(bean_weight != 0.0f)
  {
    if(shot_end_weight != 0.0f)
    {
      ratio = shot_end_weight / bean_weight;
    }
    else
    {
      ratio = getGram(display_lp) / bean_weight;
    }
    ratio = round(100.0f * ratio) / 100.0f;
    if(ratio <= 0)
    {
      ratio = +0;
    }
  }

  img.setColorDepth(1);
  img.createSprite(320, 240);
  img.fillSprite(TFT_BLACK);

  img.setTextColor(TFT_CYAN);

  img.setTextSize(4);

  img.setCursor(10, 40);
  img.printf("%7.2fg", display_weight);

  if(bean_weight != 0.0f)
  {
    img.setCursor(10, 90);
    img.printf("%7.2fg/g", ratio);
  }

  if(pre_infusion_time != 0 && !pre_infusion_freeze)
  {
    img.setCursor(10, 140);
    img.printf("%6.1fs PI", pre_infusion_time / 1000.0f);
  }

  img.setTextSize(2);

  if(bean_weight != 0.0f)
  {
    img.setCursor(160, 5);
    img.printf("%7.2fg dry", bean_weight);
  }

  if(shot_running)
  {
    if(pre_infusion_time != 0)
    {
      img.setCursor(40, 150);
      img.printf("%5.1fs PI", pre_infusion_time / 1000.0f);
    }
    
    if((ms - shot_start_millis) >= shot_min_time_disp)
    {
      img.setCursor(40, 180);
      img.printf("%5.1fs", (ms - shot_start_millis) / 1000.0f);
    }
  }
  else if(shot_time > 0)
  {
    if(pre_infusion_time != 0)
    {
      img.setCursor(40, 150);
      img.printf("%5.1fs PI %3.1fg", pre_infusion_time / 1000.0f, shot_end_weight);
    }

    img.setCursor(40, 180);
    img.printf("%5.1fs %3.1fg/s", shot_time / 1000.0f, shot_end_weight * 1000.0f / shot_time);
  }

  img.setTextSize(1);
  img.setCursor(29, 227);
  img.printf("Tare & Reset");
  img.setCursor(139, 227);
  img.printf("Set Dry");

  img.setCursor(212, 227);
  if(!auto_tare_active)
  {
    img.setTextColor(TFT_BLACK, TFT_CYAN);
  }
  img.printf("AutoTare | Cal");

  img.setTextColor(TFT_CYAN);

  img.setCursor(5, 3);
  if(isCharging)
  {
    img.printf("Charging: %d%%", battery_level);
  }
  else
  {
    img.printf(" Battery: %d%%", battery_level);
  }

  img.setCursor(5, 15);
  img.printf("   Touch: %d", pre_infusion_old);

  if(shot_start_cnt > shot_start_cnt_disp_thres || shot_running)
  {
    img.setCursor(5, 27);
    img.printf("*");
  }

  img.setBitmapColor(TFT_CYAN, TFT_BLACK);
  img.pushSprite(0, 0);

  img.deleteSprite();
}
