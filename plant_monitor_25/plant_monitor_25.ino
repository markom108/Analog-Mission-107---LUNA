// ============================================================
//  AUTOMATYCZNY MONITORING WZROSTU ROŚLIN
//  ESP32-CAM (AI-Thinker) + BME280 + BH1750
//  Zasilanie: 4xAA (6V) → pin 5V
//  SDA = GPIO1 (UOT) | SCL = GPIO3 (UOR)
//  Karta SD: tryb 1-bitowy | LED: wyłączony
//
//  ARCHITEKTURA: cała logika w setup() + deep sleep
//  Po każdym przebudzeniu ESP restartuje się i wykonuje setup()
//  od nowa. loop() nigdy nie jest osiągany.
// ============================================================

// Rozwiązanie konfliktu nazwy sensor_t między esp_camera i Adafruit_BME280
#define sensor_t camera_sensor_t
#include "esp_camera.h"
#undef sensor_t
#include "SD_MMC.h"
#include "FS.h"
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include "esp_sleep.h"
#include "esp_task_wdt.h"

// ╔══════════════════════════════════════════════════════════════╗
// ║           KONFIGURACJA — ZMIEŃ TYLKO TU, NICZEGO WIĘCEJ     ║
// ╚══════════════════════════════════════════════════════════════╝

// ------------------------------------------------------------
//  1. TRYB DEBUG
//     1 = logi przez Serial (czujniki MUSZĄ być odpięte od GPIO1/GPIO3!)
//     0 = tryb misji (czujniki podpięte, cisza totalna)
// ------------------------------------------------------------
#define DEBUG_MODE 0

// ------------------------------------------------------------
//  2. DATA I GODZINA STARTU (ustaw przed wgraniem)
//     ESP nie ma zegara — liczy czas od tej chwili.
//     Po power-off timestamps zaczynają się od nowa od tej daty.
//     Numer serii (S) NIE resetuje się — chroniony przez LAST_SERIES.txt.
// ------------------------------------------------------------
#define START_YEAR   2026
#define START_MONTH  5
#define START_DAY    22
#define START_HOUR   3
#define START_MINUTE 20
#define START_SECOND 0

// ------------------------------------------------------------
//  3. HARMONOGRAM
// ------------------------------------------------------------
#define INTERVAL_MIN      10   // co ile minut seria zdjęć
#define PHOTO_COUNT        3   // ile zdjęć w serii
#define PHOTO_DELAY_SEC    5   // co ile sekund zdjęcia w serii (ms)

// ------------------------------------------------------------
//  4. JAKOŚĆ OBRAZU
//     FRAMESIZE_UXGA  — 1600x1200 (~150KB, max jakość)
//     FRAMESIZE_HD    — 1280x720  (~70KB,  zalecana)
//     FRAMESIZE_XGA   — 1024x768  (~50KB,  bezpieczna)
//     FRAMESIZE_SVGA  — 800x600   (~30KB,  awaryjny)
//  JPEG_QUALITY: 4=najlepsza, 63=najgorsza (niższa liczba = lepsza jakość)
// ------------------------------------------------------------
#define CAMERA_RESOLUTION  FRAMESIZE_SVGA
#define JPEG_QUALITY       12

// ╔══════════════════════════════════════════════════════════════╗
// ║       KONIEC KONFIGURACJI — nie zmieniaj nic poniżej        ║
// ╚══════════════════════════════════════════════════════════════╝

// --- Stałe systemowe ---
#define I2C_SDA        1    // GPIO1 (UOT)
#define I2C_SCL        3    // GPIO3 (UOR)
#define LED_FLASH      4    // GPIO4 — wyłączony
#define CSV_FILE       "/data.csv"
#define LAST_SERIES    "/LAST_SERIES.txt"
#define BATTERY_FILE   "/BATTERY.txt"
#define WDT_TIMEOUT_S  30   // watchdog: reset po 30s zawieszenia
#define SD_RETRY_COUNT  3   // liczba prób montowania SD
#define SD_RETRY_MS   500   // przerwa między próbami (ms)
#define CMOS_WARMUP_MS 1500 // stabilizacja kamery i czujników po init (ms)

// --- Deep sleep ---
const uint64_t SLEEP_US = (uint64_t)INTERVAL_MIN * 60UL * 1000000ULL;

// --- RTC RAM — przeżywa deep sleep, zeruje się przy power-off ---
RTC_DATA_ATTR uint32_t bootCount    = 0;
RTC_DATA_ATTR uint32_t sessionCount = 0;
RTC_DATA_ATTR uint32_t seriesOffset = 0; // offset z LAST_SERIES.txt — ustawiany przy cold boot

// --- Debug makra ---
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

// --- Konfiguracja kamery AI-Thinker (nie zmieniaj) ---
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

// ============================================================
//  ZMIENNE GLOBALNE
// ============================================================
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
  return d[m - 1];
}

// Oblicza timestamp → "YYYYMMDD_HHmmss"
// outBuf musi mieć co najmniej 16 bajtów
void calcTimestamp(uint32_t totalSec, char* outBuf, size_t bufLen) {
  int sec  = (int)(totalSec % 60);
  int mn   = (int)((totalSec / 60) % 60);
  int hr   = (int)((totalSec / 3600) % 24);
  int day  = START_DAY + (int)(totalSec / 86400);
  int mon  = START_MONTH;
  int year = START_YEAR;
  while (day > daysInMonth(mon, year)) {
    day -= daysInMonth(mon, year);
    mon++;
    if (mon > 12) { mon = 1; year++; }
  }
  snprintf(outBuf, bufLen, "%04d%02d%02d_%02d%02d%02d",
           year, mon, day, hr, mn, sec);
}

// Sekundy od START_DATE dla danego numeru serii
// Używamy currentSeries (unikalny, nie resetuje się) żeby timestamp
// był zawsze unikalny — nawet po power-off
uint32_t sessionBaseSec(uint32_t currentSeries) {
  return (uint32_t)(currentSeries - 1) * INTERVAL_MIN * 60UL
       + START_HOUR * 3600UL + START_MINUTE * 60UL + START_SECOND;
}

// ============================================================
//  INICJALIZACJA KAMERY
// ============================================================
bool initCamera() {
  camera_config_t config;
  config.ledc_channel  = LEDC_CHANNEL_0;
  config.ledc_timer    = LEDC_TIMER_0;
  config.pin_d0        = CAM_PIN_D0;
  config.pin_d1        = CAM_PIN_D1;
  config.pin_d2        = CAM_PIN_D2;
  config.pin_d3        = CAM_PIN_D3;
  config.pin_d4        = CAM_PIN_D4;
  config.pin_d5        = CAM_PIN_D5;
  config.pin_d6        = CAM_PIN_D6;
  config.pin_d7        = CAM_PIN_D7;
  config.pin_xclk      = CAM_PIN_XCLK;
  config.pin_pclk      = CAM_PIN_PCLK;
  config.pin_vsync     = CAM_PIN_VSYNC;
  config.pin_href      = CAM_PIN_HREF;
  config.pin_sscb_sda  = CAM_PIN_SIOD;
  config.pin_sscb_scl  = CAM_PIN_SIOC;
  config.pin_pwdn      = CAM_PIN_PWDN;
  config.pin_reset     = CAM_PIN_RESET;
  config.xclk_freq_hz  = 20000000;
  config.pixel_format  = PIXFORMAT_JPEG;
  config.frame_size    = CAMERA_RESOLUTION;
  config.jpeg_quality  = JPEG_QUALITY;
  config.fb_count      = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    DBG_PRINTF("[KAMERA] Blad init: 0x%x\n", err);
    return false;
  }

  camera_sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_bpc(s, 1);
    s->set_wpc(s, 1);
  }

  DBG_PRINTLN("[KAMERA] OK");
  return true;
}

// ============================================================
//  WYŁĄCZENIE KAMERY PRZED SNEM
//  Fizycznie odcina napięcie od matrycy OV2640
// ============================================================
void shutdownCamera() {
  esp_camera_deinit();
  pinMode(CAM_PIN_PWDN, OUTPUT);
  digitalWrite(CAM_PIN_PWDN, HIGH);
  DBG_PRINTLN("[KAMERA] Wylaczona (PWDN HIGH)");
}

// ============================================================
//  MONTOWANIE KARTY SD (z retry)
// ============================================================
bool initSD() {
  for (int attempt = 1; attempt <= SD_RETRY_COUNT; attempt++) {
    if (SD_MMC.begin("/sdcard", true)) {
      if (SD_MMC.cardType() != CARD_NONE) {
        sdOk = true;
        DBG_PRINTF("[SD] OK (proba %d)\n", attempt);

        // Utwórz nagłówek CSV tylko przy pierwszym uruchomieniu
        if (!SD_MMC.exists(CSV_FILE)) {
          File f = SD_MMC.open(CSV_FILE, FILE_WRITE);
          if (f) {
            f.println("seria;trial;timestamp;"
                      "temperatura_C;wilgotnosc_proc;"
                      "cisnienie_hPa;oswietlenie_lux;"
                      "plik_zdjecia");
            f.flush();
            f.close();
            DBG_PRINTLN("[SD] Utworzono CSV z naglowkiem");
          }
        }
        return true;
      }
    }
    DBG_PRINTF("[SD] Proba %d nieudana, czekam...\n", attempt);
    SD_MMC.end();
    delay(SD_RETRY_MS);
  }
  DBG_PRINTLN("[SD] Blad — karta niedostepna po 3 probach");
  sdOk = false;
  return false;
}

// ============================================================
//  LAST_SERIES.TXT — numer ostatniej serii
//  Dzięki temu seria nie zeruje się po wyjęciu baterii
// ============================================================
uint32_t readLastSeries() {
  if (!SD_MMC.exists(LAST_SERIES)) return 0;
  File f = SD_MMC.open(LAST_SERIES, FILE_READ);
  if (!f) return 0;
  uint32_t val = (uint32_t)f.parseInt();
  f.close();
  return val;
}

void writeLastSeries(uint32_t series) {
  File f = SD_MMC.open(LAST_SERIES, FILE_WRITE);
  if (!f) return;
  f.print(series);
  f.flush();
  f.close();
}

// ============================================================
//  BATTERY.TXT — informacje o pracy układu
//  Nadpisywany przy każdej sesji.
//  Nie ma pomiaru napięcia (bateria wpięta tylko do 5V i GND
//  bez dzielnika napięcia — ESP nie ma jak jej zmierzyć).
//  Zamiast tego zapisuje czas pracy i numer serii.
// ============================================================
void writeBatteryFile(uint32_t series, const char* timestamp) {
  if (!sdOk) return;
  File f = SD_MMC.open(BATTERY_FILE, FILE_WRITE);
  if (!f) return;
  f.printf("Ostatnia_seria;%u\n", series);
  f.printf("Ostatni_timestamp;%s\n", timestamp);
  f.printf("Boot_count;%u\n", bootCount);
  f.printf("Session_count;%u\n", sessionCount);
  f.println("---");
  f.println("Pomiar napiecia niedostepny.");
  f.println("Bateria podpieta tylko do 5V i GND.");
  f.println("Aby mierzyc napiecie dodaj dzielnik:");
  f.println("V_bat -> R1(100k) -> GPIO33 -> R2(100k) -> GND");
  f.flush();
  f.close();
  DBG_PRINTF("[BATTERY] Zapisano do BATTERY.txt (seria %u)\n", series);
}

// ============================================================
//  INICJALIZACJA CZUJNIKÓW I2C
// ============================================================
bool initSensors() {
  bmeOk = (bme.begin(0x76, &I2CBus) || bme.begin(0x77, &I2CBus));
  DBG_PRINTF("[BME280] %s\n", bmeOk ? "OK" : "Nie znaleziono");

  lightOk = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &I2CBus);
  DBG_PRINTF("[BH1750] %s\n", lightOk ? "OK" : "Nie znaleziono");

  return bmeOk || lightOk;
}

// ============================================================
//  ODCZYT CZUJNIKÓW
// ============================================================
void readSensors(float &temp, float &hum, float &pres, float &lux) {
  if (bmeOk) {
    temp = bme.readTemperature();
    hum  = bme.readHumidity();
    pres = bme.readPressure() / 100.0F;
  } else {
    temp = hum = pres = -999.0;
  }
  if (lightOk) {
    delay(180); // BH1750 potrzebuje ~180ms na pomiar w trybie ciągłym
    lux = lightMeter.readLightLevel();
  } else {
    lux = -999.0;
  }
}

// ============================================================
//  ZAPIS DO CSV (dopisuje — nie nadpisuje)
//  flush() po każdym zapisie — zabezpieczenie przed utratą danych
// ============================================================
void writeCSV(uint32_t series, uint32_t trial,
              const char* ts,
              float temp, float hum, float pres, float lux,
              const char* filename) {
  if (!sdOk) return;
  File f = SD_MMC.open(CSV_FILE, FILE_APPEND);
  if (!f) { DBG_PRINTLN("[CSV] Blad otwarcia"); return; }
  f.printf("%u;%u;%s;%.2f;%.2f;%.2f;%.2f;%s\n",
           series, trial, ts, temp, hum, pres, lux, filename);
  f.flush();
  f.close();
  DBG_PRINTF("[CSV] S=%u T=%u\n", series, trial);
}

// ============================================================
//  ZROBIENIE I ZAPISANIE ZDJĘCIA
//  Nazwa: /S0042_T1_20260601_090000.jpg
//  Jeśli plik istnieje — dodaje _V2, _V3 itd.
// ============================================================
bool takeAndSavePhoto(uint32_t series, uint32_t trial,
                      const char* ts,
                      char* outName, size_t outLen) {
  // Przepłucz pierwszą klatkę — bywa niedoświetlona po init
  camera_fb_t* flush_fb = esp_camera_fb_get();
  if (flush_fb) esp_camera_fb_return(flush_fb);
  delay(50);

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    DBG_PRINTLN("[FOTO] Blad — brak frame buffer");
    return false;
  }

  // Znajdź wolną nazwę — dodaj _V2, _V3... jeśli nazwa już istnieje
  uint32_t version = 1;
  do {
    if (version == 1)
      snprintf(outName, outLen, "/S%04u_T%u_%s.jpg", series, trial, ts);
    else
      snprintf(outName, outLen, "/S%04u_T%u_%s_V%u.jpg", series, trial, ts, version);
    version++;
  } while (SD_MMC.exists(outName) && version <= 99);

  File f = SD_MMC.open(outName, FILE_WRITE);
  if (!f) {
    DBG_PRINTF("[FOTO] Blad otwarcia: %s\n", outName);
    esp_camera_fb_return(fb);
    return false;
  }

  size_t expected = fb->len;
  size_t written  = f.write(fb->buf, fb->len);
  f.flush();
  f.close();
  esp_camera_fb_return(fb);

  if (written != expected) {
    DBG_PRINTF("[FOTO] Blad zapisu (%u z %u bajtow)\n", written, expected);
    SD_MMC.remove(outName); // usuń niekompletny plik
    return false;
  }

  DBG_PRINTF("[FOTO] OK: %s (%u KB)\n", outName, written / 1024);
  return true;
}

// ============================================================
//  DEEP SLEEP
// ============================================================
void goToSleep() {
  DBG_PRINTF("[SLEEP] Usypiam na %d minut...\n", INTERVAL_MIN);
  DBG_FLUSH();
  delay(100);
  shutdownCamera();
  esp_sleep_enable_timer_wakeup(SLEEP_US);
  esp_deep_sleep_start();
}

// ============================================================
//  SETUP — cała logika tutaj
//  Deep sleep = ESP restartuje się po każdym przebudzeniu
//  i wykonuje setup() od nowa. loop() nigdy nie jest osiągany.
// ============================================================
void setup() {
  // Watchdog: automatyczny reset jeśli program zawiesi się na >30s
  // (np. karta SD zablokuje się w połowie zapisu)
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms    = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic  = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);

  DBG_BEGIN(115200);
  delay(100);

  // GPIO4 — wyłączony (współdzielony z SD, nie dotykamy)
  pinMode(LED_FLASH, OUTPUT);
  digitalWrite(LED_FLASH, LOW);

  bootCount++;
  sessionCount++;

  DBG_PRINTF("\n\n=== BOOT #%u | Session #%u ===\n", bootCount, sessionCount);

  // Przy pierwszym uruchomieniu — 5 sekund opóźnienia
  if (bootCount == 1) {
    DBG_PRINTLN("[START] Czekam 5s...");
    for (int i = 5; i > 0; i--) {
      esp_task_wdt_reset();
      delay(1000);
    }
  }

  // ── 1. KAMERA ───────────────────────────────────────────────
  DBG_PRINTLN("[INIT] Kamera..."); DBG_FLUSH();
  camOk = initCamera();
  if (!camOk) {
    DBG_PRINTLN("[INIT] Kamera nie dziala — ide spac");
    goToSleep();
    return;
  }

  // ── 2. KARTA SD (z retry) ───────────────────────────────────
  DBG_PRINTLN("[INIT] Karta SD..."); DBG_FLUSH();
  sdOk = initSD();

  // ── 3. NUMER SERII ──────────────────────────────────────────
  // Po cold boot (wyjęcie baterii) sessionCount zeruje się.
  // Czytamy LAST_SERIES.txt i ustawiamy seriesOffset żeby
  // kontynuować numerację — seria nigdy się nie zeruje.
  if (sdOk && bootCount == 1) {
    seriesOffset = readLastSeries();
    DBG_PRINTF("[SD] Cold boot — offset: %u, nastepna seria: #%u\n",
               seriesOffset, seriesOffset + sessionCount);
  }
  uint32_t currentSeries = seriesOffset + sessionCount;

  // ── 4. I2C I CZUJNIKI ───────────────────────────────────────
  DBG_PRINTLN("[INIT] Czujniki..."); DBG_FLUSH();
  I2CBus.begin(I2C_SDA, I2C_SCL, 100000);

  // Stabilizacja: kamera i czujniki potrzebują chwili po init
  esp_task_wdt_reset();
  delay(CMOS_WARMUP_MS);

  initSensors();

  // ── 5. ODCZYT CZUJNIKÓW ─────────────────────────────────────
  float temp, hum, pres, lux;
  readSensors(temp, hum, pres, lux);
  DBG_PRINTF("[CZUJNIKI] T=%.1fC H=%.1f%% P=%.1fhPa L=%.1flux\n",
             temp, hum, pres, lux);

  // ── 6. BAZA CZASOWA ─────────────────────────────────────────
  uint32_t baseSec = sessionBaseSec(currentSeries);
  char sessionTs[16];
  calcTimestamp(baseSec, sessionTs, sizeof(sessionTs));

  // ── 7. BATTERY.TXT ──────────────────────────────────────────
  writeBatteryFile(currentSeries, sessionTs);

  // ── 8. SERIA ZDJĘĆ ──────────────────────────────────────────
  if (camOk && sdOk) {
    for (uint32_t i = 1; i <= PHOTO_COUNT; i++) {
      esp_task_wdt_reset();

      uint32_t photoSec = baseSec + (i - 1) * (uint32_t)PHOTO_DELAY_SEC;
      char ts[16];
      calcTimestamp(photoSec, ts, sizeof(ts));

      char filename[64];
      bool saved = takeAndSavePhoto(currentSeries, i, ts,
                                    filename, sizeof(filename));

      writeCSV(currentSeries, i, ts, temp, hum, pres, lux,
               saved ? filename : "BLAD_ZAPISU");

      esp_task_wdt_reset();

      if (i < PHOTO_COUNT) {
        DBG_PRINTF("[FOTO] Czekam %ds...\n", PHOTO_DELAY_SEC);
        delay((uint32_t)PHOTO_DELAY_SEC * 1000);
      }
    }
    writeLastSeries(currentSeries);

  } else if (sdOk) {
    // Kamera nie działa — zapisz przynajmniej dane czujników
    writeCSV(currentSeries, 0, sessionTs,
             temp, hum, pres, lux, "BRAK_KAMERY");
    writeLastSeries(currentSeries);
  }

  DBG_PRINTLN("[SESJA] Zakonczona.");
  esp_task_wdt_reset();

  // ── 9. DEEP SLEEP ───────────────────────────────────────────
  goToSleep();
}

void loop() {
  // Nigdy nie wykonywany — cała logika w setup() + deep sleep
}
