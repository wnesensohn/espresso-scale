// This is code for an espresso scale. Think Acaia Lunar, only not ridiculously expensive (In the end, building this
// project will probably be more expensive than even the Lunar. But also much more fun!).

// All code under GPL v2 (see LICENSE), except where other licenses apply.

#include <M5Stack.h>
#include <WiFi.h>
//#include "HX711.h"
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"
#include "HampelFilter.h"
#include "FlowMeter.h"
#include <EEPROM.h>
//#include "utility/In_eSPI.h"
//#include "utility/Sprite.h"
//#include <TFT_eSPI.h>
//#include <SPI.h>

NAU7802 scale;

//HX711 scale(22, 21); // GROVE A
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
float static_reset_thres = 0.3;

// [g] shot-start-threshold-min
// depends on the scale being zeroed
float shot_start_thres_min = 0.2;
float shot_start_thres_max = 0.5;
float shot_start_max_incr = 0.1;

FastRunningMedian<3> outlier_rejection_filter;
FastRunningMedian<100> static_value_filter; // slowest filter, therefore also most accurate, good for drift compensation
FastRunningMedian<5> quick_settle_filter_trig; // this may need to be adjusted down to 4 or 3 to improve settling time
FastRunningMedian<3> quick_settle_filter_val; // this may need to be adjusted down to 4 or 3 to improve settling time
FastRunningMedian<11> shot_start_filter; // this may need to be adjusted up to make shot-start detection more reliable
FastRunningMedian<11> shot_end_filter; // this may need to be adjusted up to make shot-end detection more reliable

FlowMeter flow_meter;

// keeps track of shot-start-time
unsigned long shot_start_millis = 0;
unsigned long shot_end_millis = 0;

// [1] if that many samples lie between shot_start_thres_min and shot_start_thres_max, we assume the shot has started
unsigned int shot_start_cnt_thres = 8;
unsigned int shot_start_cnt;
unsigned long shot_time = 0;
float shot_end_weight = 0.0f;

uint8_t shot_running = 0;

// [g] - if the dynamic weight differs from the shot_end_filter median by less than this amount, count the shot as ended
float shot_end_thres = 0.3;
// [s] - the shot lasts at least that long, don't end it before!
float shot_min_time = 10;

unsigned int shot_end_cnt_thres = 30;
unsigned int shot_end_cnt;

int8_t battery_level = 0;

unsigned long startup_time;

size_t _eep_size = 10;
int _eep_addr_scale = 0;

float _scale_scale;
float _scale_tare = 0.0f;

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

  M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Lcd.setTextDatum(TL_DATUM);

  if (scale.begin() == false)
  {
    Serial.println("Scale not detected. Please check wiring. Freezing...");
    while (1);
  }
  Serial.println("Scale detected!");

  startup_time = millis();
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

long display_lp = 0;

int fast_settle = 0;

void loop()
{
  M5.update();

  battery_level = M5.Power.getBatteryLevel();
  bool isCharging = M5.Power.isCharging();

  unsigned long ms = millis();

  static long very_raw_weight = 0;
  while(scale.available() != true);
  very_raw_weight = scale.getReading();

  //outlier_rejection_filter.addValue(very_raw_weight);
  //long outlier_rejected_val = outlier_rejection_filter.getMedian();
  long outlier_rejected_val = very_raw_weight;

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

  if(M5.BtnC.wasReleased())
  {
    if(scale_calibrated)
    {
      scale_calibrated = 0;
    }
    else 
    {
      // shutdown
      M5.Power.deepSleep(60000000);
    }
  }

  if(M5.BtnA.wasReleased() || (startup_time + 1500 > ms))
  {
      _scale_tare = getGramUntared(display_lp);

      // reset shot thingy
      shot_running = 0;
      shot_start_cnt = 0;
      shot_end_cnt = 0;
      shot_end_weight = 0;
      shot_time = 0;
      flow_meter.clear();
  }

  long static_median = static_value_filter.getMedian();
  // check whether we have a (relatively) recent dynamic value which is in stark contrast to the slow FastRunningMedian
  long quick_settle_median = quick_settle_filter_trig.getMedian();
  if(abs(getGramDiff(quick_settle_median - static_median)) > static_reset_thres)
  {
    long quick_settle_median_val = quick_settle_filter_val.getMedian();
    quick_settle_filter_val.clear(quick_settle_median_val);
    quick_settle_filter_trig.clear(quick_settle_median_val);
    static_value_filter.clear(quick_settle_median_val);

    fast_settle = 20; // about 1 second
  }

  double display_lp_d = 0.97;
  if(fast_settle > 0)
  {
    display_lp_d = 0;
    if(fast_settle < 15)
    {
      display_lp_d = 0.6;
    }
    if(fast_settle < 10)
    {
      display_lp_d = 0.8;
    }
    if(fast_settle < 5)
    {
      display_lp_d = 0.9;
    }
    fast_settle--;
  }

  display_lp = display_lp * display_lp_d + (outlier_rejected_val * (1.0 - display_lp_d));

  static float bean_weight = 0.0f;
  if(M5.BtnB.wasReleased())
  {
    if(bean_weight != 0.0f)
    {
      bean_weight = 0.0f;
    }
    else
    {
      bean_weight = getGram(display_lp);
    }
  }

  long shot_detection_current = display_lp;
  shot_start_filter.addValue(shot_detection_current);
  shot_end_filter.addValue(shot_detection_current); 

  if(!shot_running)
  {
    long shot_start_filter_median = shot_start_filter.getMedian();
    if(
      getGramDiff(shot_detection_current - shot_start_filter_median) > shot_start_thres_min && 
      getGram(shot_start_filter_median) < (shot_start_thres_max + (shot_start_max_incr * (shot_start_cnt + 1))))
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

  float filtered_display_med = round(20.0 * getGram(display_lp)) / 20.0;
  // no-one is interested in these small fluctuations when we really should display 0
  // of course this is cheating, but it makes for a nicer display
  if(filtered_display_med < 0.05 && filtered_display_med > -0.05)
  {
    filtered_display_med = +0.0f;
  }

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
    ratio = round(10.0f * ratio) / 10.0f;
    if(ratio <= 0)
    {
      ratio = +0;
    }
  }

  img.setColorDepth(1);
  img.createSprite(320, 240);
  img.fillSprite(TFT_BLACK);

  img.setTextColor(TFT_CYAN);

  img.setTextSize(5);
  img.setCursor(10, 50);
  img.printf("%7.2fg", filtered_display_med);

  //M5.Lcd.printf("%6.2f %d", filtered_display_med, touchRead(2));
  if(bean_weight != 0.0f)
  {
    img.setCursor(10, 120);
    img.printf("%6.1f/1", ratio);
  }

  img.setTextSize(2);

  if(bean_weight != 0.0f)
  {
    img.setCursor(160, 5);
    img.printf("%7.2fg dry", bean_weight);
  }

  img.setCursor(40, 180);
  if(shot_running){
    //M5.Lcd.printf("%5.1f - %5.3f g/s", (ms - shot_start_millis) / 1000.0, flow_meter.getCurrentFlow());
    img.printf("%5.1fs", (ms - shot_start_millis) / 1000.0);
  }
  else if(shot_time > 0){
    img.printf("%5.1fs %3.1fg/s", (shot_time) / 1000.0, shot_end_weight * 1000.0 / shot_time);
  }

  img.setTextSize(1);
  img.setCursor(30, 230);
  img.printf("Tare & Reset");
  img.setCursor(140, 230);
  img.printf("Set Dry");
  img.setCursor(215, 230);
  img.printf("Shutdown / Cal");

  img.setCursor(5, 5);
  if(isCharging)
  {
    img.printf("Charging: %d%", battery_level);
  }
  else
  {
    img.printf(" Battery: %d%", battery_level);
  }

  img.setBitmapColor(TFT_CYAN, TFT_BLACK);
  img.pushSprite(0, 0);

  img.deleteSprite();
}
