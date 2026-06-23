// Automatic plant growth monitoring station (ESP32-CAM)
#define sensor_t camera_sensor_t
#include "esp_camera.h"
#undef sensor_t
#include "SD_MMC.h"
#include "FS.h"
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include "esp_sleep.h"

// ---------- User configuration ----------
#define DEBUG_MODE 0      // 0 = release build (no Serial output), 1 = verbose logging
#define START_SERIES 0    // first series number after flashing; check the last series already on the SD card

#define START_YEAR   2026
#define START_MONTH  6
#define START_DAY    23
#define START_HOUR   15
#define START_MINUTE 22
#define START_SECOND 0

#define INTERVAL_MIN     10   // minutes between photo series
#define PHOTO_COUNT       3   // photos per series
#define PHOTO_DELAY_SEC   5   // seconds between photos within a series

// ---------- Hardware configuration ----------
#define CAMERA_RESOLUTION  FRAMESIZE_UXGA
#define JPEG_QUALITY 5            // lower = better quality, larger file (0-63)
#define CAMERA_FLUSH_FRAMES 2     // frames discarded after wake-up to let exposure/AWB settle

#define CSV_FILE           "/data.csv"
#define LOG_FILE           "/log.txt"
#define CSV_SEPARATOR      ";"
#define SD_RETRY_COUNT     3
#define SD_RETRY_MS        500
#define CMOS_WARMUP_MS     1500

#if DEBUG_MODE
  #define DBG_BEGIN(b)     Serial.begin(b)
  #define DBG_PRINTLN(...) Serial.println(__VA_ARGS__)
  #define DBG_PRINTF(...)  Serial.printf(__VA_ARGS__)
  #define DBG_FLUSH()      Serial.flush()
#else
  #define DBG_BEGIN(b)
  #define DBG_PRINTLN(...)
  #define DBG_PRINTF(...)
  #define DBG_FLUSH()
#endif

// RTC RAM survives deep sleep, cleared only on power loss or hard reset
RTC_DATA_ATTR uint32_t bootCount = 0;
RTC_DATA_ATTR uint32_t sessionCount = 0;
const uint64_t SLEEP_US = (uint64_t)INTERVAL_MIN * 60UL * 1000000ULL;

// I2C bus pins
#define I2C_SDA        1
#define I2C_SCL        3
#define LED_FLASH      4

// AI-Thinker ESP32-CAM camera pins
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK     0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27
#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

TwoWire I2CBus = TwoWire(0);
Adafruit_BME280 bme;
BH1750 lightMeter(0x23);

bool camOk   = false;
bool sdOk    = false;
bool bmeOk   = false;
bool lightOk = false;

// ---------- Clock helpers ----------
bool isLeapYear(int y) {
  return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
}

int daysInMonth(int m, int y) {
  const int d[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (m == 2 && isLeapYear(y)) return 29;
  return d[m-1];
}

void calcTimestamp(uint32_t totalSec, char* buf, size_t len) {
  int sec  = (int)(totalSec % 60);
  int mn   = (int)((totalSec / 60) % 60);
  int hr   = (int)((totalSec / 3600) % 24);
  int day  = START_DAY + (int)(totalSec / 86400);
  int mon  = START_MONTH;
  int year = START_YEAR;
  while (day > daysInMonth(mon, year)) {
    day -= daysInMonth(mon, year);
    if (++mon > 12) { mon = 1; year++; }
  }
  snprintf(buf, len, "%04d%02d%02d_%02d%02d%02d", year, mon, day, hr, mn, sec);
}

uint32_t seriesBaseSec(uint32_t series) {
  return (uint32_t)(series - START_SERIES)*INTERVAL_MIN*60UL + START_HOUR*3600UL+START_MINUTE*60UL+START_SECOND;
}

// ---------- Camera ----------
bool initCamera() {
  camera_config_t cfg;

  cfg.pin_xclk = CAM_PIN_XCLK;
  cfg.pin_pclk = CAM_PIN_PCLK;
  cfg.pin_vsync = CAM_PIN_VSYNC;
  cfg.pin_href = CAM_PIN_HREF;
  cfg.pin_pwdn = CAM_PIN_PWDN;
  cfg.pin_reset = CAM_PIN_RESET;

  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer = LEDC_TIMER_0;
  cfg.xclk_freq_hz = 20000000;

  cfg.pin_d0 = CAM_PIN_D0; cfg.pin_d1 = CAM_PIN_D1;
  cfg.pin_d2 = CAM_PIN_D2; cfg.pin_d3 = CAM_PIN_D3;
  cfg.pin_d4 = CAM_PIN_D4; cfg.pin_d5 = CAM_PIN_D5;
  cfg.pin_d6 = CAM_PIN_D6; cfg.pin_d7 = CAM_PIN_D7;

  cfg.pin_sscb_sda = CAM_PIN_SIOD;
  cfg.pin_sscb_scl = CAM_PIN_SIOC;
  cfg.frame_size   = CAMERA_RESOLUTION;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.jpeg_quality = JPEG_QUALITY;
  cfg.fb_count = 2;

  if (esp_camera_init(&cfg) != ESP_OK) {
    DBG_PRINTLN("[CAMERA] Init failed");
    return false;
  }

  camera_sensor_t* s = esp_camera_sensor_get();
  if (s) {
    // NOTE: these enhancements improve image quality but can bias pixel values
    // used for plant-health image analysis. Disable if doing quantitative analysis.
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0);

    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_aec_value(s, 300);

    s->set_gain_ctrl(s, 1);
    s->set_agc_gain(s, 0);
    s->set_gainceiling(s, (gainceiling_t)2);

    s->set_bpc(s, 1);
    s->set_wpc(s, 1);

    s->set_dcw(s, 1);
    s->set_sharpness(s, 1);
    s->set_saturation(s, 1);
  }
  DBG_PRINTLN("[CAMERA] OK");
  return true;
}

void shutdownCamera() {
  esp_camera_deinit();
  pinMode(CAM_PIN_PWDN, OUTPUT);
  digitalWrite(CAM_PIN_PWDN, HIGH);
}

// ---------- SD card ----------
bool initSD() {
  for (int i = 1; i <= SD_RETRY_COUNT; i++) {
    if (SD_MMC.begin("/sdcard", true) && SD_MMC.cardType() != CARD_NONE) {
      sdOk = true;
      if (!SD_MMC.exists(CSV_FILE)) {
        File f = SD_MMC.open(CSV_FILE, FILE_WRITE);
        if (f) {
          f.println(
            "seria" CSV_SEPARATOR
            "trial" CSV_SEPARATOR
            "timestamp" CSV_SEPARATOR
            "temperature_C" CSV_SEPARATOR
            "humidity_%" CSV_SEPARATOR
            "pressure_hPa" CSV_SEPARATOR
            "lighting_lux" CSV_SEPARATOR
            "image_file"
          );
          f.flush(); f.close();
        }
      }
      return true;
    }
    SD_MMC.end();
    delay(SD_RETRY_MS);
  }
  return false;
}

void writeLog(const char* level, uint32_t series, uint32_t trial,
              const char* ts, const char* msg) {
  if (!sdOk) return;
  File f = SD_MMC.open(LOG_FILE, FILE_APPEND);
  if (!f) return;
  f.printf("[%s] S%04u;T%u;%s;%s\n", level, series, trial, ts, msg);
  f.flush(); f.close();
}

void writeCSV(uint32_t series, uint32_t trial, const char* ts, float temp, float hum, float pres, float lux, const char* filename) {
  if (!sdOk) return;
  File f = SD_MMC.open(CSV_FILE, FILE_APPEND);
  if (!f) return;
  f.printf(
    "%u" CSV_SEPARATOR
    "%u" CSV_SEPARATOR
    "%s" CSV_SEPARATOR
    "%.2f" CSV_SEPARATOR
    "%.2f" CSV_SEPARATOR
    "%.2f" CSV_SEPARATOR
    "%.2f" CSV_SEPARATOR
    "%s\n",
    series,
    trial,
    ts,
    temp,
    hum,
    pres,
    lux,
    filename);
  f.flush(); f.close();
}

// ---------- Sensors ----------
void initSensors() {
  bmeOk = bme.begin(0x76, &I2CBus) || bme.begin(0x77, &I2CBus);
  lightOk = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &I2CBus);
}

void readSensors(float &temp, float &hum, float &pres, float &lux) {
  if (bmeOk) {
    temp = bme.readTemperature();
    hum  = bme.readHumidity();
    pres = bme.readPressure() / 100.0F;
  } else { temp = hum = pres = -999.0; }
  if (lightOk) {
    delay(180);
    lux = lightMeter.readLightLevel();
  }
  else lux = -999.0;
}

// ---------- Photo capture ----------
bool takeAndSavePhoto(uint32_t series, uint32_t trial, const char* ts, char* outName, size_t outLen) {
  snprintf(outName, outLen, "/S%04u_T%u_%s.jpg", series, trial, ts);

  if (SD_MMC.exists(outName)) {
    DBG_PRINTF("[PHOTO] Already exists: %s\n", outName);
    return false;
  }

  // Discard warm-up frames before capturing the saved frame
  for (int fi = 0; fi < CAMERA_FLUSH_FRAMES; fi++) {
    camera_fb_t* tmp = esp_camera_fb_get();
    if (tmp) {
      esp_camera_fb_return(tmp);
      delay(80);
    }
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    DBG_PRINTLN("[PHOTO] Frame buffer error");
    writeLog("ERROR", series, trial, ts, "Frame_buffer_error");
    return false;
  }

  File f = SD_MMC.open(outName, FILE_WRITE);
  if (!f) {
    esp_camera_fb_return(fb);
    writeLog("ERROR", series, trial, ts, "File_open_error");
    return false;
  }

  size_t written  = f.write(fb->buf, fb->len);
  size_t expected = fb->len;
  f.flush(); f.close();
  esp_camera_fb_return(fb);

  if (written != expected) {
    SD_MMC.remove(outName);
    writeLog("ERROR", series, trial, ts, "Incomplete_jpg_write");
    return false;
  }

  DBG_PRINTF("[PHOTO] OK: %s\n", outName);
  return true;
}

// ---------- Deep sleep ----------
void goToSleep() {
  uint64_t sessionDuration = (uint64_t)millis()* 1000ULL;

  // Compensate elapsed session time so the full wake/sleep cycle matches INTERVAL_MIN
  uint64_t actualSleepUs;
  if (sessionDuration < SLEEP_US)
      actualSleepUs = SLEEP_US - sessionDuration;
  else
      actualSleepUs = 1000000ULL;

  DBG_PRINTF("[SLEEP] Session duration: %lu us. Remaining sleep: %llu us\n", sessionDuration, actualSleepUs);
  DBG_FLUSH();

  delay(200);
  shutdownCamera();
  esp_sleep_enable_timer_wakeup(actualSleepUs);
  esp_deep_sleep_start();
}

// ---------- Main logic ----------
void setup() {
  DBG_BEGIN(115200);
  delay(300);

  pinMode(LED_FLASH, OUTPUT);
  digitalWrite(LED_FLASH, LOW);

  bootCount++;
  sessionCount++;

  uint32_t currentSeries = (uint32_t)START_SERIES + sessionCount - 1;

  DBG_PRINTF("\n=== BOOT #%u | Session #%u | Series #%u | Reset: %s ===\n", bootCount, sessionCount, currentSeries,
             (esp_reset_reason() == ESP_RST_DEEPSLEEP) ? "DEEPSLEEP" : "COLD");

  if (esp_reset_reason() != ESP_RST_DEEPSLEEP) {
    DBG_PRINTLN("[START] Cold boot - waiting 5s...");
    delay(5000);
  }

  camOk = initCamera();
  if (!camOk) {
    DBG_PRINTLN("[ERROR] Camera not working");
    goToSleep();
    return;
  }

  sdOk = initSD();
  if (!sdOk) {
    DBG_PRINTLN("[ERROR] SD unavailable");
    goToSleep();
    return;
  }

  uint32_t baseSec = seriesBaseSec(currentSeries);
  char sessionTs[16];
  calcTimestamp(baseSec, sessionTs, sizeof(sessionTs));
  writeLog("INFO", currentSeries, 0, sessionTs, "Session_start");
  DBG_PRINTF("[LOG] Series %u start\n", currentSeries);

  I2CBus.begin(I2C_SDA, I2C_SCL, 100000);
  delay(CMOS_WARMUP_MS);
  initSensors();
  if (!bmeOk)   writeLog("WARN", currentSeries, 0, sessionTs, "BME280_missing");
  if (!lightOk) writeLog("WARN", currentSeries, 0, sessionTs, "BH1750_missing");

  float temp, hum, pres, lux;
  readSensors(temp, hum, pres, lux);
  DBG_PRINTF("[SENSORS] T=%.1f H=%.1f P=%.1f L=%.1f\n", temp, hum, pres, lux);

  for (uint32_t i = 1; i <= PHOTO_COUNT; i++) {
    // Scheduled (not actual) capture time, used only to keep timestamps evenly spaced
    uint32_t photoSec = baseSec + (i - 1) * (uint32_t)PHOTO_DELAY_SEC;
    char ts[16];
    calcTimestamp(photoSec, ts, sizeof(ts));

    char filename[48];
    bool saved = takeAndSavePhoto(currentSeries, i, ts, filename, sizeof(filename));
    if (saved) { writeCSV(currentSeries, i, ts, temp, hum, pres, lux, filename); }
    if (i < PHOTO_COUNT) delay((uint32_t)PHOTO_DELAY_SEC * 1000);
  }
  writeLog("INFO", currentSeries, 0, sessionTs, "Session_end_OK");
  DBG_PRINTLN("[SESSION] Finished.");
  DBG_FLUSH();
  goToSleep();
}

// setup() always ends in deep sleep, so loop() is never reached
void loop() {}
