//AUTOMATIC PLANT GROWTH MONITORING 
#define sensor_t camera_sensor_t
#include "esp_camera.h"
#undef sensor_t
#include "SD_MMC.h"
#include "FS.h"
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include "esp_sleep.h"

// ╔══════════════════════════════════════════════════════════════╗
// ║                        CONFIGURATION                         ║
// ╚══════════════════════════════════════════════════════════════╝
#define DEBUG_MODE 0

// Numer pierwszej serii po wgraniu.
// Jeśli poprzednio skończyłeś na serii 42 — wpisz 43.
#define START_SERIES  2230 //nu

// Data i godzina startu (ustaw przed wgraniem)
#define START_YEAR   2026
#define START_MONTH  6
#define START_DAY    11
#define START_HOUR   8
#define START_MINUTE 50
#define START_SECOND 0

// Harmonogram
#define INTERVAL_MIN     10   // co ile minut seria zdjęć
#define PHOTO_COUNT       3   // ile zdjęć w serii
#define PHOTO_DELAY_SEC   5   // co ile sekund zdjęcia w serii

// Jakość obrazu
// FRAMESIZE_UXGA=1600x1200 | FRAMESIZE_HD=1280x720
// FRAMESIZE_XGA=1024x768   | FRAMESIZE_SVGA=800x600
// [ZMIANA] XGA zamiast SVGA — lepszy kompromis jakość/czas zapisu dla OV2640
#define CAMERA_RESOLUTION  FRAMESIZE_XGA
// [ZMIANA] 8 zamiast 12 — niższa wartość = lepsza jakość JPEG
#define JPEG_QUALITY       8

// ╔══════════════════════════════════════════════════════════════╗
// ║       KONIEC KONFIGURACJI                                   ║
// ╚══════════════════════════════════════════════════════════════╝

#define I2C_SDA        1
#define I2C_SCL        3
#define LED_FLASH      4
#define CSV_FILE       "/data.csv"
#define LOG_FILE       "/log.txt"
#define SD_RETRY_COUNT  3
#define SD_RETRY_MS   500
#define CMOS_WARMUP_MS 1500
// [ZMIANA] Ile klatek "flush" przed właściwym zdjęciem — daje czas AEC
#define CAMERA_FLUSH_FRAMES 2

const uint64_t SLEEP_US = (uint64_t)INTERVAL_MIN * 60UL * 1000000ULL;

// RTC RAM — przeżywa deep sleep
// UWAGA: zeruje się tylko przy power-off lub twardym resecie
RTC_DATA_ATTR uint32_t bootCount    = 0;
RTC_DATA_ATTR uint32_t sessionCount = 0;

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

// Piny kamery AI-Thinker
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

TwoWire         I2CBus = TwoWire(0);
Adafruit_BME280 bme;
BH1750          lightMeter(0x23);

bool camOk   = false;
bool sdOk    = false;
bool bmeOk   = false;
bool lightOk = false;

// ============================================================
//  ZEGAR SOFTWAROWY
// ============================================================
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
  snprintf(buf, len, "%04d%02d%02d_%02d%02d%02d",
           year, mon, day, hr, mn, sec);
}

uint32_t seriesBaseSec(uint32_t series) {
  return (uint32_t)(series - START_SERIES) * INTERVAL_MIN * 60UL
       + START_HOUR * 3600UL + START_MINUTE * 60UL + START_SECOND;
}

// ============================================================
//  KAMERA
// ============================================================
bool initCamera() {
  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0; cfg.ledc_timer = LEDC_TIMER_0;
  cfg.pin_d0 = CAM_PIN_D0; cfg.pin_d1 = CAM_PIN_D1;
  cfg.pin_d2 = CAM_PIN_D2; cfg.pin_d3 = CAM_PIN_D3;
  cfg.pin_d4 = CAM_PIN_D4; cfg.pin_d5 = CAM_PIN_D5;
  cfg.pin_d6 = CAM_PIN_D6; cfg.pin_d7 = CAM_PIN_D7;
  cfg.pin_xclk = CAM_PIN_XCLK; cfg.pin_pclk = CAM_PIN_PCLK;
  cfg.pin_vsync = CAM_PIN_VSYNC; cfg.pin_href = CAM_PIN_HREF;
  cfg.pin_sscb_sda = CAM_PIN_SIOD; cfg.pin_sscb_scl = CAM_PIN_SIOC;
  cfg.pin_pwdn = CAM_PIN_PWDN; cfg.pin_reset = CAM_PIN_RESET;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.frame_size   = CAMERA_RESOLUTION;
  cfg.jpeg_quality = JPEG_QUALITY;
  // [ZMIANA] fb_count=2 — drugi bufor redukuje artefakty klatki
  cfg.fb_count     = 2;

  if (esp_camera_init(&cfg) != ESP_OK) {
    DBG_PRINTLN("[KAMERA] Blad init");
    return false;
  }

  camera_sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    // [ZMIANA] Tryb balansu bieli: 1=Sunny — stabilny dla stałego oświetlenia
    s->set_wb_mode(s, 0);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    // [ZMIANA] Dłuższa ekspozycja bazowa — lepsza jakość przy roślinach
    s->set_aec_value(s, 300);
    s->set_gain_ctrl(s, 1);
    // [ZMIANA] Gain bazowy = 0 — minimalizuje szum
    s->set_agc_gain(s, 0);
    // [ZMIANA] Ograniczenie max gain do 4× — zapobiega szumnym zdjęciom
    s->set_gainceiling(s, (gainceiling_t)2);
    s->set_bpc(s, 1);
    s->set_wpc(s, 1);
    // [ZMIANA] Downsize correction — poprawia jakość przy skalowaniu
    s->set_dcw(s, 1);
    // [ZMIANA] Lekkie wyostrzenie krawędzi
    s->set_sharpness(s, 1);
    // [ZMIANA] Lekko zwiększone nasycenie — zieleń roślin bardziej wyraźna
    s->set_saturation(s, 1);
  }

  DBG_PRINTLN("[KAMERA] OK");
  return true;
}

void shutdownCamera() {
  esp_camera_deinit();
  pinMode(CAM_PIN_PWDN, OUTPUT);
  digitalWrite(CAM_PIN_PWDN, HIGH);
}

// ============================================================
//  KARTA SD
// ============================================================
bool initSD() {
  for (int i = 1; i <= SD_RETRY_COUNT; i++) {
    if (SD_MMC.begin("/sdcard", true) && SD_MMC.cardType() != CARD_NONE) {
      sdOk = true;
      if (!SD_MMC.exists(CSV_FILE)) {
        File f = SD_MMC.open(CSV_FILE, FILE_WRITE);
        if (f) {
          f.println("seria;trial;timestamp;temperatura_C;"
                    "wilgotnosc_proc;cisnienie_hPa;oswietlenie_lux;plik");
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

// ============================================================
//  LOG
// ============================================================
void writeLog(const char* level, uint32_t series, uint32_t trial,
              const char* ts, const char* msg) {
  if (!sdOk) return;
  File f = SD_MMC.open(LOG_FILE, FILE_APPEND);
  if (!f) return;
  f.printf("[%s] S%04u;T%u;%s;%s\n", level, series, trial, ts, msg);
  f.flush(); f.close();
}

// ============================================================
//  CZUJNIKI
// ============================================================
void initSensors() {
  bmeOk   = bme.begin(0x76, &I2CBus) || bme.begin(0x77, &I2CBus);
  lightOk = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &I2CBus);
}

void readSensors(float &temp, float &hum, float &pres, float &lux) {
  if (bmeOk) {
    temp = bme.readTemperature();
    hum  = bme.readHumidity();
    pres = bme.readPressure() / 100.0F;
  } else { temp = hum = pres = -999.0; }
  if (lightOk) { delay(180); lux = lightMeter.readLightLevel(); }
  else lux = -999.0;
}

// ============================================================
//  CSV
// ============================================================
void writeCSV(uint32_t series, uint32_t trial, const char* ts,
              float temp, float hum, float pres, float lux,
              const char* filename) {
  if (!sdOk) return;
  File f = SD_MMC.open(CSV_FILE, FILE_APPEND);
  if (!f) return;
  f.printf("%u;%u;%s;%.2f;%.2f;%.2f;%.2f;%s\n",
           series, trial, ts, temp, hum, pres, lux, filename);
  f.flush(); f.close();
}

// ============================================================
//  ZDJĘCIE
//  [ZMIANA] Wielokrotny flush klatek (CAMERA_FLUSH_FRAMES) zamiast jednego
//  Jeśli plik istnieje — pomija (nie zapisuje do CSV)
// ============================================================
bool takeAndSavePhoto(uint32_t series, uint32_t trial,
                      const char* ts, char* outName, size_t outLen) {
  snprintf(outName, outLen, "/S%04u_T%u_%s.jpg", series, trial, ts);

  if (SD_MMC.exists(outName)) {
    DBG_PRINTF("[FOTO] Juz istnieje: %s\n", outName);
    return false;
  }

  // [ZMIANA] Przepłukaj CAMERA_FLUSH_FRAMES klatek — AEC zdąży się ustabilizować
  for (int fi = 0; fi < CAMERA_FLUSH_FRAMES; fi++) {
    camera_fb_t* tmp = esp_camera_fb_get();
    if (tmp) { esp_camera_fb_return(tmp); delay(80); }
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    DBG_PRINTLN("[FOTO] Blad frame buffer");
    writeLog("ERROR", series, trial, ts, "Blad_frame_buffer");
    return false;
  }

  File f = SD_MMC.open(outName, FILE_WRITE);
  if (!f) {
    esp_camera_fb_return(fb);
    writeLog("ERROR", series, trial, ts, "Blad_otwarcia_pliku");
    return false;
  }

  size_t written  = f.write(fb->buf, fb->len);
  size_t expected = fb->len;
  f.flush(); f.close();
  esp_camera_fb_return(fb);

  if (written != expected) {
    SD_MMC.remove(outName);
    writeLog("ERROR", series, trial, ts, "Niepelny_zapis_jpg");
    return false;
  }

  DBG_PRINTF("[FOTO] OK: %s\n", outName);
  return true;
}

// ============================================================
//  DEEP SLEEP
// ============================================================
void goToSleep() {
  DBG_PRINTF("[SLEEP] %d min\n", INTERVAL_MIN);
  DBG_FLUSH();
  delay(200);
  shutdownCamera();
  esp_sleep_enable_timer_wakeup(SLEEP_US);
  esp_deep_sleep_start();
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  DBG_BEGIN(115200);
  delay(300);

  // GPIO4 (LED/SD) — wyłączony
  pinMode(LED_FLASH, OUTPUT);
  digitalWrite(LED_FLASH, LOW);

  bootCount++;
  sessionCount++;

  // Numer serii = START_SERIES + ile sesji od ostatniego cold boot
  uint32_t currentSeries = (uint32_t)START_SERIES + sessionCount - 1;

  DBG_PRINTF("\n=== BOOT #%u | Session #%u | Seria #%u | Reset: %s ===\n",
             bootCount, sessionCount, currentSeries,
             (esp_reset_reason() == ESP_RST_DEEPSLEEP) ? "DEEPSLEEP" : "COLD");

  // 5 sekund tylko przy cold boot
  if (esp_reset_reason() != ESP_RST_DEEPSLEEP) {
    DBG_PRINTLN("[START] Cold boot — czekam 5s...");
    delay(5000);
  }

  // 1. Kamera — zawsze pierwsza
  camOk = initCamera();
  if (!camOk) {
    DBG_PRINTLN("[ERROR] Kamera nie dziala");
    goToSleep();
    return;
  }

  // 2. SD — zaraz po kamerze
  sdOk = initSD();
  if (!sdOk) {
    DBG_PRINTLN("[ERROR] SD niedostepne");
    goToSleep();
    return;
  }

  // Timestamp bieżącej sesji
  uint32_t baseSec = seriesBaseSec(currentSeries);
  char sessionTs[16];
  calcTimestamp(baseSec, sessionTs, sizeof(sessionTs));

  writeLog("INFO", currentSeries, 0, sessionTs, "Sesja_start");
  DBG_PRINTF("[LOG] Seria %u start\n", currentSeries);

  // 3. I2C + czujniki
  I2CBus.begin(I2C_SDA, I2C_SCL, 100000);
  delay(CMOS_WARMUP_MS);
  initSensors();

  if (!bmeOk)   writeLog("WARN", currentSeries, 0, sessionTs, "BME280_brak");
  if (!lightOk) writeLog("WARN", currentSeries, 0, sessionTs, "BH1750_brak");

  // 4. Odczyt czujników
  float temp, hum, pres, lux;
  readSensors(temp, hum, pres, lux);
  DBG_PRINTF("[CZUJNIKI] T=%.1f H=%.1f P=%.1f L=%.1f\n",
             temp, hum, pres, lux);

  // 5. Seria zdjęć — każde zdjęcie osobno, bez pętli retry
  for (uint32_t i = 1; i <= PHOTO_COUNT; i++) {
    uint32_t photoSec = baseSec + (i - 1) * (uint32_t)PHOTO_DELAY_SEC;
    char ts[16];
    calcTimestamp(photoSec, ts, sizeof(ts));

    char filename[48];
    bool saved = takeAndSavePhoto(currentSeries, i, ts,
                                  filename, sizeof(filename));
    if (saved) {
      writeCSV(currentSeries, i, ts, temp, hum, pres, lux, filename);
    }

    if (i < PHOTO_COUNT) delay((uint32_t)PHOTO_DELAY_SEC * 1000);
  }

  writeLog("INFO", currentSeries, 0, sessionTs, "Sesja_koniec_OK");
  DBG_PRINTLN("[SESJA] Zakonczona.");
  DBG_FLUSH();

  goToSleep();
}

void loop() {}
