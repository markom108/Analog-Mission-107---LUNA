//AUTOMATIC PLANT GROWTH MONITORING (Cpp projekt)

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
// ║       PARAMETRY DO USTAWWIENIA PRZED URUCHOMIENIEM           ║
// ╚══════════════════════════════════════════════════════════════╝
#define DEBUG_MODE 0  //Włączanie trybu debugowania [0-finalny kod do wgrania, bez żadnych komunikatów=szybsze,1-program będzie wypisywał komunikaty przez Serial Monitor]
#define START_SERIES  2870  //numer pierwszej serii zdjęć po wgraniu(sprawdź jaki numer serii jest ostatni zapisany na karcie SD)

//----------DATA I GODZINA STARTU POMIARÓW(do timestampów)---------------
#define START_YEAR   2026
#define START_MONTH  6
#define START_DAY    17
#define START_HOUR   2
#define START_MINUTE 10
#define START_SECOND 0

//----------USTAWIENIA SERII-------------------
#define INTERVAL_MIN     10   // co ile minut seria zdjęć
#define PHOTO_COUNT       3   // ile zdjęć w serii
#define PHOTO_DELAY_SEC   5   // co ile sekund zdjęcia w serii


// ╔══════════════════════════════════════════════════════════════╗
// ║    Konfiguracje HARDWARU (nie trzeba zmieniać przed startem) ║
// ╚══════════════════════════════════════════════════════════════╝

//----------USTAWIENIA OBRAZU-----------------
// Dostępne: UXGA=1600x1200, HD=1280x720, XGA=1024x768, SVGA=800x600
// XGA wybrano ze względu na wystarczającą szczegółowość zdjęcia, przy jednoczesnym ograniczeniu rozmiaru plików i czasu zapisu na kartę SD
#define CAMERA_RESOLUTION  FRAMESIZE_UXGA
#define JPEG_QUALITY 5 // im  niższa wartość = lepsza jakość JPEG, ale większy rozmiar pliku i czas zapisu (od 0 do 63)
#define CAMERA_FLUSH_FRAMES 2 // ile pierwszych klatek z kamery ignorujemy zanim zapiszemy właściwe zdjęcie; kamera po uruchomieniu nie od razu daje idealny obraz, bo musi sie najpierw ustabilizować; bez flush zdjęcia słabe, nie czytelne

//-------USTAWIENIA ZAPISU DANYCH----------------
#define CSV_FILE           "/data.csv" //nazwa pliku na karcie SD gdzie zapisuję dane z czujników
#define LOG_FILE           "/log.txt" //nazwa plikó do logów systemowych (diagnostycznych)
#define csv_separator      ";"
#define SD_RETRY_COUNT     3 // ile razy próbować uruchomić kartę SD, ESP32-CAM często ma problemy: SD nie startuje za pierwszym razem, albo nie jest od razu gotowa
#define SD_RETRY_MS        500 // czas czekania między próbami SD
#define CMOS_WARMUP_MS     1500 //czas rozgrzewki kamery po starcie; Matryca OV2640:ustawia ekspozycję,ustawia balans bieli,stabilizuje obraz; bez tego pierwsze zdjęcie może być za jasne/ciemne

//-------------DEBUGOWANIE--------------------
#if DEBUG_MODE //(instrukcja preprocesora)-wykonuje sie przed kompilacją
  //DEBUG=1 => przypisz poniższym makrom instrukcje wypisywania podanych danych w SERIAL MONITORZE! (taki debugger)
  //Serial= obiekt Arduino używany do komunikacji komputera z ESP32 
  //Ta komunikacja działa za pomocą układu w płytce do kontroli dwukierunkowej komunikacji szeregowej UART (ESP32 wysyła/odbiera sygnały do Serial przez piny UOR/UOT) używany do wyświetlania komunikatów w monitorze portu szeregowego i do uploadu kodu
  #define DBG_BEGIN(b)     Serial.begin(b) //za DBG_BEGIN(b) podstaw serial.begin(b) co oznacza uruchom komunikację szeregową (ESP32 zacznij wysyłać i odbierać dane przez port szeregowy) |b=baud rate, prędkość transmisji (bit/sec)
  #define DBG_PRINTLN(...) Serial.println(__VA_ARGS__) // ... (variadic macro) oznacza przyjmij dowolną liczbę argumentów |__VA_ARGS__ =wstaw dokładnie to co dostałeś(specjalny mechanizm preprocesoraw w C/cpp) | println=wypisz i przejdź do nowej linii
  #define DBG_PRINTF(...)  Serial.printf(__VA_ARGS__) //printf= wypisz tekst z formatowaniem
  #define DBG_FLUSH()      Serial.flush() //flush=czekaj aż wszystkie dane przeznaczone do wysłania z Serial(opróżnij kolejkę, bufor transmisji) zostaną wysłane | chcę print i potem uśpić esp, jeśli nie dam flush, to esp32 może być w trakcie wypisywania gdy 
#else
  //jeśli program odpalony w trybie DEBUG=0 to przypisz poniższym makrom puste instrukcje podczas kompilacji (czyli nie wykonuj tego kodu poprostu)
  #define DBG_BEGIN(b) //za DBG_BEGIN(b) podstaw pustą instrukcję(czyli nie rób z tym nic)
  #define DBG_PRINTLN(...)
  #define DBG_PRINTF(...)
  #define DBG_FLUSH()
#endif

//-------------RTC RAM ----------------------
// RTC RAM = pamięć podtrzymywana w deep sleep (gdy ESP32CAM idzie deep sleep RAM znika, RAM RTC zostaje)
//RTC_DATA_ATTR= tą zmienną stwórz w RTC RAM, dzięki temu nie będzie kasowana przy deep sleep
//UWAGA: RTC RAM kasuje się tylko przy power-off lub twardym resecie
RTC_DATA_ATTR uint32_t bootCount = 0; //ile razy ESP się obudziło
RTC_DATA_ATTR uint32_t sessionCount = 0; //licznik sesji
const uint64_t SLEEP_US = (uint64_t)INTERVAL_MIN * 60UL * 1000000ULL;//przelicza podany przez użytkownika czas snu między sesjami z minut na mikrosekundy 

//--------------PINY-------------------
//SENSORY:
//Wykorzystane przezemnie sensory do przesyłu danych wykorzystują protokół komunikacyji I2C(Inter-Integrated Circuit) -> polega na tym że zawsze sa 2 linie sygnałowe (SDA-do przesyłu danych i SCL-zegar synchronizujący transmisje)
#define I2C_SDA        1 //NIE ZMIENIAĆ![wybrany ze względu na małą ilość wolnych pinów w ESP32CAM, ten pin sprawdzony i układ na nim działa poprawnie] numer pinu  na ESP32CAM (GPIO1) do którego jest połączony SDA(Serial Data) magistrali I2C przesyłający dane z czujników
#define I2C_SCL        3 //NIE ZMIENIAĆ! numer pinu na ESP32CAM (oznacza GPIO3) w który jest wpięty kabel z pinu SDA(Serial Data) magistrali I2C przesyłający dane z czujników
#define LED_FLASH      4 
//KAMERA
// Piny kamery AI-Thinker [ESP32-CAM ma wbudowaną kamerę OV2640 i ona jest twardo podłączona do konkretnych GPIO. Tego NIE wybierasz]
#define CAM_PIN_PWDN    32 //power down
#define CAM_PIN_RESET   -1 //reset [-1=nie używany]
#define CAM_PIN_XCLK     0 //zegar kamery
#define CAM_PIN_SIOD    26 //I2C dla kamery (wewnętrzne sterowanie OV2640)
#define CAM_PIN_SIOC    27 //I2C dla kamery (wewnętrzne sterowanie OV2640)
#define CAM_PIN_D7      35 //CAM_PIN_D0 - CAM_PIN_D7 = 8-bitowa magistrala danych obrazu (kamera wysyła: piksele -> równolegle -> ESP32)
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25 //(synchronizacja obrazu): sygnał początek klatki
#define CAM_PIN_HREF    23 //(synchronizacja obrazu): sygnał linia obrazu
#define CAM_PIN_PCLK    22 //(synchronizacja obrazu): sygnał zegar pikseli

//--------------Obiekty czujników-------------
TwoWire I2CBus = TwoWire(0); //obiekt magistrali I2C
Adafruit_BME280 bme; //obiekt czujnika BME(temp+wilgoć+ciśnienie)
BH1750 lightMeter(0x23);//tworzę obiekt czujnika BH + od razu konfiguruję(begin) podając adres czujnika na magistrali I2C

//--------Znaczniku stanu (status flags)----
bool camOk   = false;//czy kamera okej
bool sdOk    = false;//czy SD działa
bool bmeOk   = false;//czy BME280 działą
bool lightOk = false;//czy BH1750 działa


// ╔══════════════════════════════════════════════════════════════╗
// ║                     Funkcje pomocnicze                       ║
// ╚══════════════════════════════════════════════════════════════╝

//--------------------ZEGAR SOFTWAROWY------------------------
//sprawdza czy rok jest przestępny
bool isLeapYear(int y) {
  return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
}

//zwraca liczbę dni w miesiącu
int daysInMonth(int m, int y) {
  const int d[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (m == 2 && isLeapYear(y)) return 29;
  return d[m-1];
}

//Tworzenie Timestampa: ile sekund minęło od daty startu-> data i godzina
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

//Zamienia numer serii zdjęcia na liczbę sekund od startu projektu
uint32_t seriesBaseSec(uint32_t series) {
  return (uint32_t)(series - START_SERIES) * INTERVAL_MIN * 60UL
       + START_HOUR * 3600UL + START_MINUTE * 60UL + START_SECOND;
}


// ------------------------KAMERA----------------------
/*czyli jest tak, esp32cam na swojej płytce ma moduł LEDC który jest takim metronomem, teraz tak, możemy sie do pinu podłączonego z tym modułem wpiąć tyloma rzeczami ile chcemy i poprostu korzystać z tego sygnału. LEDC ma w sobie kilke (ile) niezależnych generatorów takiego syngału, czyli można z tego samego modułu mieć parę różnych, niezależnych sygnałów do których podłączamy sie na tym samym pinie, tylko w softwarze poprostu korzystamy z różnych kanałów. Czy teraz dobrze rozumiem?*/
//Konfiguracja kamery oraz ustawienie parametrów obrazu
bool initCamera() {
//-------------KONFIGURACJA PINÓW I POŁĄCZEŃ KAMERY----------------------------
  camera_config_t cfg;//obiekt do konfiguracji modułu kamery

  //1. Konfiguracja pinów ESP32CAM (podanych w makro) z kamerą
  cfg.pin_xclk = CAM_PIN_XCLK; //ten pin ESP32CAM będzie przekazywać kamerze takty zegara (do synchronizacji robienia zdjęć)
  cfg.pin_pclk = CAM_PIN_PCLK; //-||- ODBIERAĆ sygnał PCLK z kamery(sygnał PCLK[Pixel Clock] to impuls zegara generowanego przez KAMERĘ, wysyłany w momencie przesłania przez kamerę na esp32 1 porcji danych ze zdjęcia (1 piksel/fragment piksela), esp32 widzi ten impuls i wtedy odczytuje dane)
  cfg.pin_vsync = CAM_PIN_VSYNC; //-||- sygnał VSYNC z kamery (Vertical Sync (synchronizacja klatki), kamera wysyła ten sygnał, żeby przekazać że właśnie zaczyna przesyłać nowe zdjęcie-VSYNC jest podłączony do sprzętowego sterownika kamery(z biblioteki esp_camera), który mówi, że jak przyjdzie ten rodzaj sygnału to resetuj licznik klatki i zaczyna zbierać dane od nowa)
  cfg.pin_href = CAM_PIN_HREF;//-||- ODBIERAĆ sygnał HREF z kamery (Horizontal Reference (synchronizacja linii)- czyli kamera wysyła ten sygnał żeby przekazywać że wysyła jedną linię obrazu)
  cfg.pin_pwdn = CAM_PIN_PWDN; //-||- ESP32CAM  może sterować za pomocą sygnału elektrycznego(stan wysoki/niski) wejściem PWDN kamery- czyli usypiać/włączać ją 
  cfg.pin_reset = CAM_PIN_RESET; //-||- ESP32CAM  może sterować za pomocą sygnału elektrycznego(stan wysoki/niski) wejściem RESET kamery- czyli wymuszać twardy RESET na kamerze
  
  /*2. Ustawienia zegara dla kamery w celu synchronizacji (kamera potrzebuje sygnału XCLK-clock)
    ESP32CAM posiada w sobie sprzętowy moduł LEDC, który generuje sygnału PWM czyli prostokątny sygnał elektryczny: 0 1 0 1 0 1 ....
    Moduł LEDC ma w sobie 16 niezależnych kanałów (8 high speed[bardzo precyzyjny sprzetowy] + 8 low speed[wolniejszy, mniej precyzyjny]), 
    każdy może generować własny niezależny sygnał. Dzięki temu po podpięciu naszych urządzeń do konkretnych pinów na ESP32CAM każde z urządzeń może mieć własny niezależny sygnał PWM (służący np. jako zegar). 
    
    KOMUNIKACJA:
    Najpierw łączymy za pomocą kabli lub lutujemy odpowiednie piny urządzenia (np. kamera, bo używane przez nas sensory korzystają sie z I2C) z wybranymi pinami w esp32cam (lub jak kamera czy led, te piny są już polutowane na stałe),
    a następnie w softwarze ustawiamy żeby dany LEDC channel działał na konkretnym pinie(tam wysyłał sygnał eleketroniczny PWM, które przewodem odbiera sobie urządzenie).
    W ten sposób, można mieć jednocześnie np.:
      - LED controller (kontrolowanie jasności diody co takt zegara inna jasność) [Led połączony na stałę z GPIO4]
      - sterowanie buzzerem (głośniej ciszej co takt zegara) podłączonym na pinie GPIO 12 buzzerze
      - generowanie sygnału zegara do synchronizacji kamery [ GPIO0 (XCLK) ]*/
  cfg.ledc_channel = LEDC_CHANNEL_0;    // wybieramy sobie pierwszy dostępny kanał którym będzie wysyłany sygnał generowany przez ESP32 w celu synchronizacji kamery (stała z biblioteki esp32)
  cfg.ledc_timer = LEDC_TIMER_0;        //ustaw tempo zegara, mówi jak szybko ma się zmieniać wartość z 0->1, potem 1->0 tego zegara (stała z biblioteki esp32)
  cfg.xclk_freq_hz = 20000000;      //ustawia częstotliwość kamery 20MHz (kamera dostaje 20mln taktów zegara/1 sec)

  //3. Przypisanie pinów kamerze (podanych w makro) - od teraz to co my już wiedzieliśmy (czyli takie piny jak chcieliśmy), esp32 też już zna, bo piny już skonfigurowane
  //  kamera wysyła równolegle bajty obrazu, a ESP32 odczytuje je z tych pinów (8-bitowa równoległa magistrala danych obrazu)
  cfg.pin_d0 = CAM_PIN_D0; cfg.pin_d1 = CAM_PIN_D1;
  cfg.pin_d2 = CAM_PIN_D2; cfg.pin_d3 = CAM_PIN_D3;
  cfg.pin_d4 = CAM_PIN_D4; cfg.pin_d5 = CAM_PIN_D5;
  cfg.pin_d6 = CAM_PIN_D6; cfg.pin_d7 = CAM_PIN_D7;

  //4. Konfiguracja ustawień jakości obrazu podanych przez użytkownika
  cfg.pin_sscb_sda = CAM_PIN_SIOD; //-||- będzie do przesyłania z ESP32CAM informacji o tym jak skonfigurować kamerę (dane-jakie konfiguracje kamery wprowadzić)
  cfg.pin_sscb_scl = CAM_PIN_SIOC; //-||- będzie do przesyłania z ESP32CAM informacji o tym jak skonfigurować kamerę (synchronizacj przesyłu tych konfiguracji)
  cfg.frame_size   = CAMERA_RESOLUTION; //rozdzielczośc obrazu
  cfg.pixel_format = PIXFORMAT_JPEG; //format wyjściowy obrazu 
  cfg.jpeg_quality = JPEG_QUALITY;//kompresja JPEG (stopień skompresowania obrazu)
  cfg.fb_count = 2; //liczba buforów klatek (frame buffers): ile gotowych zdjęć esp32 trzyma na RAM na raz [2 bufory: jeden jest wysyłany, drugi jednocześnie zapisuje- płynniejszy, stabilniejszy stream, aktualnie robione zdjęcie nie nadpisuje tego które się właśnie wysyła], nie więcej bo esp32cam ma bardzo ograniczony RAM, a pełna klatka zdjęcia zajmuje sporo miejsca w pamięci, co może prowadzić do crashu, resetu

  //próba uruchomienia zainicjalizowania całej konfiguracji kamery z ustawieniami do tego momentu
  if (esp_camera_init(&cfg) != ESP_OK) { 
    DBG_PRINTLN("[KAMERA] Blad init");
    return false;
  }

//-------------KONFIGURACJA PRZETWARZANIA OBRAZU PRZEZ KAMERĘ-------------------------
  camera_sensor_t* s = esp_camera_sensor_get(); //wskaźnik do obiektu sterującego sensorem kamery (czyli dostęp do ustawień fizycznej kamery, do steerowania jej obrazem) - to struct z funkcjami wskaźnikowymi
  if (s) {//jeśli istnieje (bo inaczej crash)
    //UWAGA: jeśli chcesz analizować roślinę na podstawie zdjęć, usuń poniższe ulepszenia. Poprawiają one jakość zdjęcia, ale mogą wpłynąć na wyniki analizy
    //Kolejność zastosowania funkcji ma znaczenie, jest ona na stałe zaprogrmowana w samej kamerze OV2640 w ISP-image signal Processor(sterownik do niej: esp_camera tylko tym steruje)
    
    //Automatyczny balans bieli (naturalne kolory)
    s->set_whitebal(s, 1);//Autmatyczny balans bieli(ON): różne źródło światła ma różny kolor (LED-niebieski, żarówka-żółte, itp.), żeby kolory się nie rozjeżdzału, kamera sama poprawia kolory światła usuwa żółty/niebieksi filtr, tak żeby biały przedmiot był zawsze biały nie zależnie od źródła światła
    s->set_awb_gain(s, 1);//Auto White Balance (AUTO), GAIN=wzmocnienie korekcji kolorów, whitebal włącz balans bieli, a to mocniej koryguje kolory (kamera sama dostraja kolory)
    s->set_wb_mode(s, 0);//White Balance Mode(AUTO)-kamera sama decyduje czy światło jest ciepłe czy zimne i koryguje (żeby biały był biały)
    
    //Naświetlanie (stabilizacja jasności, zbiera więcej światła)
    s->set_exposure_ctrl(s, 1);//Kontrola ekspozycji(ON-auto): ekspozycja=ile światła zbiera kamera, kamera sama ustawia czas naświetlenia i jasność obrazu (jak ciemno-wydluża czas=jaśniejsze zdjęcie, jasno-skraca czas=nie prześwietla)
    s->set_aec2(s, 1);//Automatic Exposure Control, wersja 2, lepsza (ON)-polepsza działanie Exposure control, lepiej radzi sobie w trudnym świetla, stabilizuje jasność miedzy zdjeciami, mniej nagłych zmian jasności, stabilna jasność
    s->set_aec_value(s, 300);//ustawienie jasności bazowej(początkowej): ile świtała kamera celuje żeby zebrać, dzięki temu liście nie są niedoświetlone, a kamera nie zaczyna od zbyt ciemnego obrazu
    
    //Automatyczna kontrola gain (lepsze nocne zdjęcia, nie zbiera więcej światła(żeby nie zmienić za bardzo tych zdjęć od pozostałych) tylko wzmacnia to które co już ma)
    s->set_gain_ctrl(s, 1);//Automatyczna kontrola gainu: jeśli jest ciemno i AEC nie wystarcza, gain się włącza. Kamera nie zmienia już ekspozycji, zaczyna wzmacniać sygnał, rozjaśnia obraz analogowo, MINUSY: szum, możliwa ziarnistość i fałszywe detale, ale lepszy lekko zaszumiony obraz niż całkowicie ciemny
    s->set_agc_gain(s, 0);//ręczne ustawienie poziomu gain(baseline)(0-praktycznie brak wzmocnień na start)
    s->set_gainceiling(s, (gainceiling_t)2);//Ograniczenie max gain do 4×- zapobiega szumnym zdjęciom

    //Bad Pixel Correction
    s->set_bpc(s, 1);//naprawa martwych/zepsutych pikseli, kamera ma czasem piksele które: świecą zawzse na biało, są czarne, pojedyncze kropki błędów, żeby algorytmy nie potraktowały tego jako np. choroba liścia/rośliny, to BPC wykrywa je i zastępuje średią z sąsiednich pikseli, lepsze wyniki do analizy zdjęć
    s->set_wpc(s, 1);//White Pixel Correction: naprawa jasnych pikseli(hot pixels), w ciemności lub przy wysokim gain mogą się one pojawić. WPC usuwa je
    
    //Ostrość
    s->set_dcw(s, 1);//Downsize correction: poprawia jakość przy skalowaniu, gdy sensor robi zdj w dużej rozdzielczości, ale zapisujesz JPEG w mniejszym formacie, wtedy mogą pojawić sie dziwne linie, rozmycia itp. DCW poprawia zmniejszani obrazy, próbkowania pikseli, ostrzejsze krawędzie liści, mniej rozmytych konturów
    s->set_sharpness(s, 1);// wyostrzenie krawędzi: patrzy na różnice miedzy pikselami i podkreśla granice, MINUSY: może wzmocnić szum, pojawia sie "halo"[sztuczny jasny/ciemny obrys wokół krawędzi] wokół liści
    s->set_saturation(s, 1);//zwiększone nasycenie: zwiększa intensywność kolorów, zieleń roślin bardziej wyraźna, bardziej żywe kolory, obraz mniej szary, UWAGA: kolory mniej naturalne, róznice kolorów mogą być trochę przekłamane
  }
  DBG_PRINTLN("[KAMERA] OK");
  return true; //Konfiguracja przebiegła pomyślnie
}

//Uśpienie kamery
void shutdownCamera() {
  esp_camera_deinit();//[zamknij obsługę kamery] sterownik esp_camera przestaje obsługiwać kamerę, (zwalnia: bufory RAM, przechwytywanie obrazu, zasoby ESP32)
  pinMode(CAM_PIN_PWDN, OUTPUT);//[przygotuj sterowanie usypianiem kamery] na podanym w konfiguracji wyżej pinie ma działać jako WYJŚCIE (ESP32 będzie zaraz NADAWAĆ sygnał stanu wysokiego który będzie wychodzić z ESP32CAM do kamery), trzeba to ustawić bo domyślnie input jest
  digitalWrite(CAM_PIN_PWDN, HIGH);//[uśpij kamerę(nie esp32cam)] ustaw stan wysoki na pinie PWDN. kamera przechodzi w tryb power down i ogranicza pobór energii
}

//-------------------------KARTA SD-----------------------
//tryb 1-bitowy: esp32 musi przesyłać ciąg bitów, w trybie 1 biotowym zamiast przesyłać jednocześnie na 4 pinach, wysyłasz na 1 pin. Dzięki temu zajmujesz MNIEJ pinów na przesył, ale jest to wolneijsze
bool initSD() {
  for (int i = 1; i <= SD_RETRY_COUNT; i++) {//spróbuj uruchomić kartę SD podaną w konfiguracji ilość razy
    if (SD_MMC.begin("/sdcard", true) && SD_MMC.cardType() != CARD_NONE) {// próba uruchomienia sd(od teraz sd dostępna w /sdcard, użyj trybu 1-bitowego) w trybie 1-bitowym i karta faktycznie istnieje
      sdOk = true;//flaga stanu zaktualizowana
      if (!SD_MMC.exists(CSV_FILE)) { //czy nie istnieje już taki plik
        File f = SD_MMC.open(CSV_FILE, FILE_WRITE);//otwieram plik do zapisu
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
          f.flush(); f.close();//wymuś zapis na kartę i zapisz plik
        }
      }
      return true;//karta działa
    }
    //sd nie ruszyło
    SD_MMC.end();//reset sd
    delay(SD_RETRY_MS);//czekaj chwilę
  }
  return false;//wszystkie próby zawiodły
}

// -------------------------LOG-----------------------------
void writeLog(const char* level, uint32_t series, uint32_t trial,
              const char* ts, const char* msg) {
  if (!sdOk) return;//karta nie działą
  File f = SD_MMC.open(LOG_FILE, FILE_APPEND);//otwórz log file
  if (!f) return;//jeśli log sie nie otworzył
  f.printf("[%s] S%04u;T%u;%s;%s\n", level, series, trial, ts, msg);//zapisz do loga
  f.flush(); f.close();//wymuś zapis i zamknij plik
}

// --------------------------CSV-------------------------------
void writeCSV(uint32_t series, uint32_t trial, const char* ts, float temp, float hum, float pres, float lux, const char* filename) {
  if (!sdOk) return;
  File f = SD_MMC.open(CSV_FILE, FILE_APPEND);//dopisz dane
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
    ts, //timestamp
    temp,
    hum,
    pres,
    lux,
    filename);
  f.flush(); f.close();
}

// ---------------------CZUJNIKI-------------------------------

void initSensors() {
  bmeOk   = bme.begin(0x76, &I2CBus) || bme.begin(0x77, &I2CBus); //uruchom BME przez na magistrali I2Cbus (sprawdź na adresie takim lub takim) i zapisz do flagi stanu czy się udało
  lightOk = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &I2CBus);
}

void readSensors(float &temp, float &hum, float &pres, float &lux) {
  if (bmeOk) {//jeśli flaga sensora nie zgłasza błędu inicjalizacji
    temp = bme.readTemperature();//odczytaj z czujnika temp
    hum  = bme.readHumidity();//-||- wilgoć
    pres = bme.readPressure() / 100.0F; // -||- ciśnienie (bme zwraca w paskalach, a ja chcę w hPa, stąd /100)
  } else { temp = hum = pres = -999.0; }//wartości gdy nie udało się odczytać pomiaru
  if (lightOk) { 
    delay(180); //ten czujnik ustwaiony na ciągły pomiar (CONTINUOUS_HIGH_RES_MODE), daj więc mu chwilę na zrobienie nowego pomiaru
    lux = lightMeter.readLightLevel(); }//odczytaj ostatni najnowszy pomiar
  else lux = -999.0;
}

// -----------------ZDJĘCIE [flush paru klatek + jeśli plik o danej nazwie istniej to pomiń i nie zapisuj do CSV]-----------------------------------
bool takeAndSavePhoto(uint32_t series, uint32_t trial, const char* ts, char* outName, size_t outLen) {
  snprintf(outName, outLen, "/S%04u_T%u_%s.jpg", series, trial, ts);//NAZWA ZDJĘCIA[seria, trial, timestamp] (sprintf-zabezpiecza przed przepełnieniem bufora outLen)

  if (SD_MMC.exists(outName)) { //jeśli cgjęcie już istnieje to nie rób go drugi raz, wyjdź
    DBG_PRINTF("[FOTO] Juz istnieje: %s\n", outName);
    return false;
  }

  //Kamera ma ciągły strumień klatek, cały czas robi zdjęcia, ale pierwsze klatki po starcie, zmianie ekspozycji, wybudzeniu mogą być w słabej jakości.
  //dlatego robimy FLUSH KLATEK-odrzucamy kilka pierwszych klatek, zanim zapiszemy tą właściwą
  for (int fi = 0; fi < CAMERA_FLUSH_FRAMES; fi++) {//powtórz tyle razy ile ustawiono w makro na górze pliku
    camera_fb_t* tmp = esp_camera_fb_get();//daj mi aktualną klatkę z kamery (zapisz obraz do RAM, bufora i daj mi wskaźnik do tego RAMu)
    if (tmp) { //jeśli kamera zwróciła obraz
      esp_camera_fb_return(tmp); //zwalniam bufor należący do sterownika kamery, na którym jest zdjęcie
      delay(80); //czekam chwilę, żeby zdążył sie zwolnic
    }//
  }

  camera_fb_t* fb = esp_camera_fb_get(); //daj mi aktualną klatkę z kamery
  if (!fb) {//błąd: zapisz w logu
    DBG_PRINTLN("[FOTO] Blad frame buffer");
    writeLog("ERROR", series, trial, ts, "Blad_frame_buffer");
    return false;
  }

  File f = SD_MMC.open(outName, FILE_WRITE);//otwórz nowy plik na zdjęcie. Zapis od zera, to jest nowy plik jpg
  if (!f) {//nie 
    esp_camera_fb_return(fb);//zwalniam bufor należący do sterownika kamery, na którym jest zdjęcie
    writeLog("ERROR", series, trial, ts, "Blad_otwarcia_pliku");
    return false;
  }

  size_t written  = f.write(fb->buf, fb->len);//zapisuje zdjęcie, zwraca ile bajtów realnie zapisano na karcie
  size_t expected = fb->len;//ile bajtów ma zdjęcie
  f.flush(); f.close();//wymusza opróżnienie bufora i zapis na kartę
  esp_camera_fb_return(fb);//zwalnia bufor w esp32cam

  if (written != expected) {//jeśli nie zapisano na karcie tylu samo rzeczy co było oczekiwane
    SD_MMC.remove(outName);//usuń to zdjęcie z karty
    writeLog("ERROR", series, trial, ts, "Niepelny_zapis_jpg");//zgłość błą w logach
    return false;
  }

  DBG_PRINTF("[FOTO] OK: %s\n", outName);
  return true;
}

//--------------------DEEP SLEEP-----------------------
void goToSleep() {
  unsigned long sessionDuration = millis(); //ile milisekund minęło od uruchomienia ESP do teraz 
  uint64_t elapsedUs = (uint64_t)sessionDuration * 1000ULL; //przelicz na mikrosekundy
  
  // Obliczamy ile faktycznie musi spać, by cykl trwał idealnie 10 minut
  // Jeśli praca trwała dłużej niż sen (mało prawdopodobne, ale zabezpieczamy), śpimy 1 sekundę
  uint64_t actualSleepUs;
  if (elapsedUs < SLEEP_US) //jeśli czas od uruchomienia esp32 < długości trwania przerwy miedzy sesjami
      actualSleepUs = SLEEP_US - elapsedUs;
  else
      actualSleepUs = 1000000ULL;

  DBG_PRINTF("[SLEEP] Czas pracy: %lu ms. Zostalo snu: %llu us\n", sessionDuration, actualSleepUs);
  DBG_FLUSH();
  
  delay(200);
  shutdownCamera(); // TO JEST KLUCZOWE - odcina zasilanie kamery na czas snu!
  esp_sleep_enable_timer_wakeup(actualSleepUs);
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
