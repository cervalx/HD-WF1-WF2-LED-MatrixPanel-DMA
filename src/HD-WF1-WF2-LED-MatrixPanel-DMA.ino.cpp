// Custom LED Matrix Firmware (leveraging the HUB75 DMA library) for the Huidu HUB75 Series Control Cards.
// Example shop link: https://www.aliexpress.com/item/1005005038544582.html -> WF1
// Example shop link: https://www.aliexpress.com/item/1005002271988988.html -> WF2

#if defined(WF1)
  #include "hd-wf1-esp32s2-config.h"
#elif defined(WF2)
  #include "hd-wf2-esp32s3-config.h"
#else
  #error "Please define either WF1 or WF2"
#endif  


#include <esp_err.h>
#include <esp_log.h>
#include "debug.h"
#include "littlefs_core.h"
#include <ctime>
#include "driver/ledc.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>

#include <WebServer.h>
#include <ESPmDNS.h>
#include <I2C_BM8563.h>   // https://github.com/tanakamasayuki/I2C_BM8563

#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
//#include <ElegantOTA.h> // upload firmware by going to http://<ipaddress>/update

#include <ESP32Time.h>
#include <Bounce2.h>

#define fs LittleFS

#ifndef PI
#define PI 3.14159265358979323846
#endif

/*----------------------------- Wifi Configuration -------------------------------*/

const char *wifi_ssid = "Top_of_Zurich";
const char *wifi_pass = "Zurichparadise";

/*----------------------------- RTC and NTP -------------------------------*/

I2C_BM8563 rtc(I2C_BM8563_DEFAULT_ADDRESS, Wire1);
const char* ntpServer         = "time.cloudflare.com";
const char* ntpLastUpdate     = "/ntp_last_update.txt";

// POSIX timezone strings — the ESP32 applies DST rules automatically
#define TZ_ZURICH    "CET-1CEST,M3.5.0,M10.5.0/3"   // CET(UTC+1) / CEST(UTC+2)
#define TZ_NEW_YORK  "EST5EDT,M3.2.0,M11.1.0"         // EST(UTC-5) / EDT(UTC-4)

/*-------------------------- HUB75E DMA Setup -----------------------------*/
#define PANEL_RES_X 64      // Number of pixels wide of each INDIVIDUAL panel module. 
#define PANEL_RES_Y 64     // Number of pixels tall of each INDIVIDUAL panel module.
#define PANEL_CHAIN 1      // Total number of panels chained one to another


#if defined(WF1)

HUB75_I2S_CFG::i2s_pins _pins_x1 = {WF1_R1_PIN, WF1_G1_PIN, WF1_B1_PIN, WF1_R2_PIN, WF1_G2_PIN, WF1_B2_PIN, WF1_A_PIN, WF1_B_PIN, WF1_C_PIN, WF1_D_PIN, WF1_E_PIN, WF1_LAT_PIN, WF1_OE_PIN, WF1_CLK_PIN};

#else

HUB75_I2S_CFG::i2s_pins _pins_x1 = {WF2_X1_R1_PIN, WF2_X1_G1_PIN, WF2_X1_B1_PIN, WF2_X1_R2_PIN, WF2_X1_G2_PIN, WF2_X1_B2_PIN, WF2_A_PIN, WF2_B_PIN, WF2_C_PIN, WF2_D_PIN, WF2_X1_E_PIN, WF2_LAT_PIN, WF2_OE_PIN, WF2_CLK_PIN};
HUB75_I2S_CFG::i2s_pins _pins_x2 = {WF2_X2_R1_PIN, WF2_X2_G1_PIN, WF2_X2_B1_PIN, WF2_X2_R2_PIN, WF2_X2_G2_PIN, WF2_X2_B2_PIN, WF2_A_PIN, WF2_B_PIN, WF2_C_PIN, WF2_D_PIN, WF2_X2_E_PIN, WF2_LAT_PIN, WF2_OE_PIN, WF2_CLK_PIN};

#endif


/*-------------------------- Class Instances ------------------------------*/
// Routing in the root page and webcamview.html natively uses the request
// handlers of the ESP32 WebServer class, so it explicitly instantiates the
// ESP32 WebServer.
WebServer           webServer;
WiFiMulti           wifiMulti;
ESP32Time           esp32rtc;  // offset in seconds GMT+1
MatrixPanel_I2S_DMA *dma_display = nullptr;

// INSTANTIATE A Button OBJECT FROM THE Bounce2 NAMESPACE
Bounce2::Button button = Bounce2::Button();

// ROS Task management
TaskHandle_t Task1;
TaskHandle_t Task2;

#include "led_pwm_handler.h"

RTC_DATA_ATTR int bootCount = 0;

// Display modes
enum DisplayMode {
  MODE_CLOCK_WITH_ANIMATION = 0,
  MODE_CLOCK_ONLY = 1,
  MODE_BOUNCING_SQUARES = 2,
  MODE_COUNT = 3
};

DisplayMode currentDisplayMode = MODE_CLOCK_ONLY;
unsigned long buttonPressStartTime = 0;
bool buttonPressHandled = true;
volatile bool buttonPressed = false;

// Text scrolling variables
int textScrollY = 0;
int textScrollDirection = -1;  // -1 = moving up, +1 = moving down
int textScrollX = 0;
int textScrollXDirection = 1;  // +1 = moving right, -1 = moving left
unsigned long lastTextScrollUpdate = 0;

// Bouncing squares animation variables
struct BouncingSquare {
  float x, y;
  float vx, vy;
  uint16_t color;
  int size;
};

const int NUM_SQUARES = 3;
BouncingSquare squares[NUM_SQUARES];

IRAM_ATTR void toggleButtonPressed() {
  // This function will be called when the interrupt occurs on pin PUSH_BUTTON_PIN
  buttonPressed = true;
  ESP_LOGI("toggleButtonPressed", "Interrupt Triggered.");
}

// Initialize bouncing squares
void initBouncingSquares() {
  for (int i = 0; i < NUM_SQUARES; i++) {
    squares[i].x = random(0, PANEL_RES_X - 8);
    squares[i].y = random(0, PANEL_RES_Y - 8);
    squares[i].vx = random(1, 4) * (random(0, 2) ? 1 : -1);
    squares[i].vy = random(1, 4) * (random(0, 2) ? 1 : -1);
    squares[i].size = random(4, 8);
    squares[i].color = dma_display->color565(random(100, 255), random(100, 255), random(100, 255));
  }
}

// Update and draw bouncing squares
void updateBouncingSquares() {
  dma_display->clearScreen();
  
  for (int i = 0; i < NUM_SQUARES; i++) {
    // Update position
    squares[i].x += squares[i].vx * 0.5;
    squares[i].y += squares[i].vy * 0.5;
    
    // Bounce off walls
    if (squares[i].x <= 0 || squares[i].x >= PANEL_RES_X - squares[i].size) {
      squares[i].vx = -squares[i].vx;
      squares[i].x = constrain(squares[i].x, 0, PANEL_RES_X - squares[i].size);
    }
    if (squares[i].y <= 0 || squares[i].y >= PANEL_RES_Y - squares[i].size) {
      squares[i].vy = -squares[i].vy;
      squares[i].y = constrain(squares[i].y, 0, PANEL_RES_Y - squares[i].size);
    }
    
    // Draw square
    dma_display->fillRect((int)squares[i].x, (int)squares[i].y, squares[i].size, squares[i].size, squares[i].color);
  }
}



/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}


// Function that gets current epoch time
unsigned long getEpochTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}

// Check if RTC has valid date/time
bool isRTCTimeValid(const I2C_BM8563_DateTypeDef &date, const I2C_BM8563_TimeTypeDef &time) {
  // Check reasonable year range (2020-2099)
  if (date.year < 2020 || date.year > 2099) {
    return false;
  }
  // Check month range
  if (date.month < 1 || date.month > 12) {
    return false;
  }
  // Check day range
  if (date.date < 1 || date.date > 31) {
    return false;
  }
  // Check hour range
  if (time.hours > 23) {
    return false;
  }
  // Check minute/second range
  if (time.minutes > 59 || time.seconds > 59) {
    return false;
  }
  return true;
}

// Get time from external RTC as fallback
bool getRTCTime(struct tm* timeinfo) {
  I2C_BM8563_DateTypeDef rtcDate;
  I2C_BM8563_TimeTypeDef rtcTime;
  
  // Get date and time from RTC (these are void functions)
  rtc.getDate(&rtcDate);
  rtc.getTime(&rtcTime);
  
  if (!isRTCTimeValid(rtcDate, rtcTime)) {
    return false;
  }
  
  timeinfo->tm_year = rtcDate.year - 1900;
  timeinfo->tm_mon = rtcDate.month - 1;
  timeinfo->tm_mday = rtcDate.date;
  timeinfo->tm_hour = rtcTime.hours;
  timeinfo->tm_min = rtcTime.minutes;
  timeinfo->tm_sec = rtcTime.seconds;
  timeinfo->tm_wday = rtcDate.weekDay;
  timeinfo->tm_isdst = -1;
  
  return true;
}

// Enhanced time getter with RTC fallback
bool getTimeWithFallback(struct tm* timeinfo) {
  // Try ESP32 internal RTC first
  if (getLocalTime(timeinfo)) {
    return true;
  }
  
  // Fallback to external RTC
  Serial.println("ESP32 RTC failed, trying external RTC...");
  return getRTCTime(timeinfo);
}

// Function declarations
// void updateClockWithAnimation();
void updateClockOnly();
// void updateClockOverlay();
// void initBouncingSquares();
// void updateBouncingSquares();

//
// Arduino Setup Task
//
void setup() {

  // Init Serial
  // if ARDUINO_USB_CDC_ON_BOOT is defined then the debug will go out via the USB port
  Serial.begin(115200);

  /*-------------------- START THE HUB75E DISPLAY --------------------*/
    
    // Module configuration
    HUB75_I2S_CFG mxconfig(
      PANEL_RES_X,   // module width
      PANEL_RES_Y,   // module height
      PANEL_CHAIN,   // Chain length
      _pins_x1       // pin mapping for port X1
    );
    mxconfig.i2sspeed = HUB75_I2S_CFG::HZ_8M;  //ALEX: reduced from 20 
    mxconfig.latch_blanking = 4;
    //mxconfig.clkphase = false;
    //mxconfig.driver = HUB75_I2S_CFG::FM6126A;
    //mxconfig.double_buff = false;  
    //mxconfig.min_refresh_rate = 30;


    // Display Setup
    dma_display = new MatrixPanel_I2S_DMA(mxconfig);
    dma_display->begin();
    dma_display->setRotation(3);
    dma_display->setBrightness8(1); //0-255
    dma_display->clearScreen();
    dma_display->setTextColor(dma_display->color565(255, 0, 0));

    // dma_display->fillScreenRGB888(255,0,0);
    // delay(100);
    // dma_display->fillScreenRGB888(0,255,0);
    // delay(100);    
    // dma_display->fillScreenRGB888(0,0,255);
    // delay(100);       
    dma_display->clearScreen();
    dma_display->print("Connecting");     


  /*-------------------- START THE NETWORKING --------------------*/
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(wifi_ssid, wifi_pass); // configure in the *-config.h file

  // wait for WiFi connection with timeout
  Serial.print("Waiting for WiFi to connect...");
  int wifi_retry = 0;
  const int max_wifi_retries = 30; // 30 seconds timeout

  while (wifiMulti.run() != WL_CONNECTED && wifi_retry < max_wifi_retries) {
    delay(1000);
    wifi_retry++;
    Serial.print(".");

    // Update display every 5 attempts
    if (wifi_retry % 5 == 0) {
      dma_display->clearScreen();
      dma_display->setCursor(0,0);
      dma_display->print("WiFi...");
      dma_display->setCursor(3,13);
      dma_display->printf("%d/%d", wifi_retry, max_wifi_retries);
    }
  }

  if (wifiMulti.run() == WL_CONNECTED) {
    Serial.println(" connected");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    dma_display->clearScreen();
    dma_display->setCursor(0,0);
    dma_display->print("Connected!");
    delay(1000);
  } else {
    Serial.println(" FAILED!");
    Serial.printf("WiFi connection failed after %d attempts\n", wifi_retry);
    Serial.printf("SSID: %s\n", wifi_ssid);
    Serial.printf("WiFi Status: %d\n", WiFi.status());

    dma_display->clearScreen();
    dma_display->setCursor(0,0);
    dma_display->setTextColor(dma_display->color565(255, 0, 0));
    dma_display->print("WiFi");
    dma_display->setCursor(3,13);
    dma_display->print("FAILED!");
    dma_display->setCursor(3,23);
    dma_display->printf("S:%d", WiFi.status());

    delay(5000);
    // Continue anyway to allow OTA/recovery
  }
    

  /*-------------------- --------------- --------------------*/
  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();    

  if ( wakeup_reason == ESP_SLEEP_WAKEUP_EXT0)
  {
    dma_display->setCursor(3,6);
    dma_display->print("Wake up!");
    delay(1000);
  }
  else
  {
    dma_display->print("Starting.");
  }




  /*-------------------- LEDC Controller --------------------*/
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_13_BIT ,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 4000,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = RUN_LED_PIN,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));  


    // Start fading that LED
    xTaskCreatePinnedToCore(
      ledFadeTask,            /* Task function. */
      "ledFadeTask",                 /* name of task. */
      4096,                    /* Stack size of task */
      NULL,                     /* parameter of the task */
      1,                        /* priority of the task */
      &Task1,                   /* Task handle to keep track of created task */
      0);                       /* Core */   
    

  /*-------------------- INIT LITTLE FS --------------------*/
  bool littlefs_ok = LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED);
  if(!littlefs_ok){
      Serial.println("LittleFS Mount Failed - continuing without filesystem");
  } else {
      listDir(LittleFS, "/", 1);
  }    
 
  /*-------------------- --------------- --------------------*/
  // Init I2C for RTC
  Wire1.begin(BM8563_I2C_SDA, BM8563_I2C_SCL);
  rtc.begin();

  // Get RTC date and time
  I2C_BM8563_DateTypeDef rtcDate;
  I2C_BM8563_TimeTypeDef rtcTime;
  
  // Try to read RTC - these functions return void, so we can't check their return values
  rtc.getDate(&rtcDate);
  rtc.getTime(&rtcTime);
  
  // Validate RTC data
  bool rtcDataValid = isRTCTimeValid(rtcDate, rtcTime);
  
  if (rtcDataValid) {
    Serial.printf("RTC Date: %04d-%02d-%02d %02d:%02d:%02d\n", 
                  rtcDate.year, rtcDate.month, rtcDate.date,
                  rtcTime.hours, rtcTime.minutes, rtcTime.seconds);
  } else {
    Serial.println("RTC data is invalid or corrupted");
  }
  
  time_t ntp_last_update_ts = 0;
  File file = fs.open(ntpLastUpdate, FILE_READ, true);
  if(!file) {
      Serial.println("NTP last update file not found - will perform NTP sync");
  } else  {
      file.read( (uint8_t*) &ntp_last_update_ts, sizeof(ntp_last_update_ts));          
      Serial.print("NTP last update epoch: ");
      Serial.println(ntp_last_update_ts);
      file.close();      
  }

  // Current RTC time (include time components for better comparison)
  time_t curr_rtc_ts = 0;
  bool needNTPUpdate = true;
  
  if (rtcDataValid) {
    struct tm curr_rtc_tm = {};
    curr_rtc_tm.tm_year = rtcDate.year - 1900;  // years since 1900
    curr_rtc_tm.tm_mon = rtcDate.month - 1;     // months since January (0-11)
    curr_rtc_tm.tm_mday = rtcDate.date;         // day of the month (1-31)
    curr_rtc_tm.tm_hour = rtcTime.hours;        // hours since midnight (0-23)
    curr_rtc_tm.tm_min = rtcTime.minutes;       // minutes after the hour (0-59)
    curr_rtc_tm.tm_sec = rtcTime.seconds;       // seconds after the minute (0-59)
    curr_rtc_tm.tm_isdst = -1;                  // daylight saving time flag
    
    curr_rtc_ts = mktime(&curr_rtc_tm);
    
    // Check if we need NTP update (more than 7 days old, or no previous NTP sync)
    if (ntp_last_update_ts > 0) {
      long timeDiff = abs((long int)(curr_rtc_ts - ntp_last_update_ts));
      needNTPUpdate = (timeDiff > (60*60*24*7)); // 7 days instead of 30
      Serial.printf("Time difference: %ld seconds (%ld days)\n", timeDiff, timeDiff/(60*60*24));
    }
  }

  if (!rtcDataValid || needNTPUpdate)
  {
      Serial.println("Performing NTP time synchronization...");
      ESP_LOGI("ntp_update", "Updating time from NTP server");    
  
      // Sync to UTC; the TZ string handles offset + DST automatically
      configTime(0, 0, ntpServer);
      setenv("TZ", TZ_ZURICH, 1);
      tzset();

      // Wait for NTP sync with timeout
      int ntpRetries = 0;
      struct tm timeInfo;
      while (!getLocalTime(&timeInfo) && ntpRetries < 10) {
        delay(1000);
        ntpRetries++;
        Serial.print(".");
      }
      
      if (getLocalTime(&timeInfo)) {
        Serial.println("\nNTP sync successful");
        
        // Set RTC time - setTime returns void, so we can't check its return value
        I2C_BM8563_TimeTypeDef timeStruct;
        timeStruct.hours   = timeInfo.tm_hour;
        timeStruct.minutes = timeInfo.tm_min;
        timeStruct.seconds = timeInfo.tm_sec;
        
        rtc.setTime(&timeStruct);
        Serial.println("RTC time set successfully");

        // Set RTC Date - setDate returns void, so we can't check its return value
        I2C_BM8563_DateTypeDef dateStruct;
        dateStruct.weekDay = timeInfo.tm_wday;
        dateStruct.month   = timeInfo.tm_mon + 1;
        dateStruct.date    = timeInfo.tm_mday;
        dateStruct.year    = timeInfo.tm_year + 1900;
        
        rtc.setDate(&dateStruct);
        Serial.printf("RTC updated to: %04d-%02d-%02d %02d:%02d:%02d\n",
                      dateStruct.year, dateStruct.month, dateStruct.date,
                      timeStruct.hours, timeStruct.minutes, timeStruct.seconds);
        
        // Update local variables with new time
        rtcDate = dateStruct;
        rtcTime = timeStruct;
        rtcDataValid = true;
      } else {
        Serial.println("\nNTP sync failed - will use existing RTC time if valid");
      }

      // Save NTP update timestamp
      ntp_last_update_ts = getEpochTime();
      if (ntp_last_update_ts > 0) {
        File file = fs.open(ntpLastUpdate, FILE_WRITE);
        if(!file) {
            Serial.println("Failed to open NTP timestamp file for writing");
        } else  {
            file.write( (uint8_t*) &ntp_last_update_ts, sizeof(ntp_last_update_ts));          
            file.close();      
            Serial.print("Saved NTP update timestamp: "); 
            Serial.println(ntp_last_update_ts, DEC);            
        }
      }

  }
  
  // Update ESP32 internal RTC if we have valid external RTC data
  if (rtcDataValid) {
    esp32rtc.setTime(rtcTime.seconds, rtcTime.minutes, rtcTime.hours, 
                     rtcDate.date, rtcDate.month, rtcDate.year);
    Serial.println("ESP32 internal RTC updated from external RTC:");    
    Serial.println(esp32rtc.getTime("%A, %B %d %Y %H:%M:%S"));
  } else {
    Serial.println("Warning: No valid time source available!");
  }

   /*-------------------- --------------- --------------------*/

    webServer.on("/", []() {
      webServer.send(200, "text/plain", "Hi! I am here.");
    });

    //ElegantOTA.begin(&webServer);    // Start ElegantOTA
    webServer.begin();
    Serial.println("OTA HTTP server started");

    /*-------------------- --------------- --------------------*/
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());  

    delay(1000);
    
    dma_display->clearScreen();
    dma_display->setCursor(0,0);

    dma_display->print(WiFi.localIP());
    dma_display->clearScreen();
    delay(3000);

    // Initialize bouncing squares for animation mode
    // initBouncingSquares();
}

unsigned long last_update = 0;
char buffer[64];
void loop() 
{
    webServer.handleClient();
    delay(1);

    // Update display based on current mode
    switch (currentDisplayMode) {   
        case MODE_CLOCK_ONLY:
            updateClockOnly();
            break;
    }
}

// Helper function to print bold text (double-draw shifted 1px right)
void printBold(MatrixPanel_I2S_DMA* display, int x, int y, const char* text, bool enable) {
  display->setCursor(x, y);
  display->print(text);
  if (enable) {
    display->setCursor(x + 1, y);
    display->print(text);
  }
}

// Returns NYSE session color based on NY local time
// Pre-market: 4:00-9:29 (orange), Market: 9:30-16:00 (white), After-hours: 16:00-20:00 (navy)
uint16_t getNYSEColor(const struct tm* ny_time) {
  int hour = ny_time->tm_hour;
  int min  = ny_time->tm_min;

  // Pre-market: 4:00-9:29
  if (hour >= 4 && (hour < 9 || (hour == 9 && min < 30))) {
    return dma_display->color565(255, 165, 0); // orange
  }

  // Market open: 9:30-16:00
  if ((hour == 9 && min >= 30) || (hour > 9 && hour < 16) || (hour == 16 && min == 0)) {
    return dma_display->color565(255, 255, 255); // white
  }

  // After-hours: 16:00-20:00
  if (hour >= 16 && hour < 20) {
    return dma_display->color565(0, 0, 128); // navy
  }

  // Outside trading hours (20:00-3:59): navy
  return dma_display->color565(0, 0, 128); // navy
}

// Clock only mode (no animation background) - dual timezone display
void updateClockOnly() {
  if ((millis() - last_update) > 1000) {
    struct tm timeinfo;
    if (getTimeWithFallback(&timeinfo)) {
      dma_display->clearScreen();

      // Update text scroll position every minute
      unsigned long now = millis();
      if (now - lastTextScrollUpdate >= 1000 * 60) {
        lastTextScrollUpdate = now;
        textScrollY += textScrollDirection;
        textScrollX += textScrollXDirection;

        // Reverse direction when reaching 0 or max scroll (Y axis)
        if (textScrollY <= 0) {
          textScrollY = 0;
          textScrollDirection = 1;
        } else if (textScrollY >= 20) {
          textScrollY = 20;
          textScrollDirection = -1;
        }

        // Reverse direction when reaching 0 or max scroll (X axis)
        if (textScrollX <= 0) {
          textScrollX = 0;
          textScrollXDirection = 1;
          // normal limiti is 4 but if using bold text limit is 3
        } else if (textScrollX >= 3) {
          textScrollX = 3;
          textScrollXDirection = -1;
        }
      }

      // Zurich (ZH) — top half (TZ already set to TZ_ZURICH)
      memset(buffer, 0, 64);
      snprintf(buffer, 64, "ZH%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
      dma_display->setTextColor(dma_display->color565(255, 0, 0));
      printBold(dma_display, textScrollX, 2 + textScrollY, buffer, false);
      
      // NY time — switch TZ to New York, query local time, then restore Zurich TZ
      setenv("TZ", TZ_NEW_YORK, 1);
      tzset();
      struct tm ny_timeinfo;
      getLocalTime(&ny_timeinfo);
      setenv("TZ", TZ_ZURICH, 1);
      tzset();
      
      // NY time — bottom half (color based on NYSE session)
      memset(buffer, 0, 64);
      snprintf(buffer, 64, "NY%02d:%02d:%02d", ny_timeinfo.tm_hour, ny_timeinfo.tm_min, ny_timeinfo.tm_sec);
      dma_display->setTextColor(getNYSEColor(&ny_timeinfo));
      printBold(dma_display, textScrollX, 2 + 32 + textScrollY, buffer, false);

      // DST indicator: orange pixel at (0,0) when European Summer Time is active
      if (timeinfo.tm_isdst > 0) {
        dma_display->drawPixelRGB888(0, 0, 255, 165, 0); // orange
      }

      // DST indicator: orange pixel at (0,1) when US Eastern Daylight Time is active
      if (ny_timeinfo.tm_isdst > 0) {
        dma_display->drawPixelRGB888(1, 0, 255, 165, 0); // orange
      }

      Serial.println("Dual clock update (ZH + NY)");
    } else {
      Serial.println("Failed to get time from all sources.");
      dma_display->clearScreen();
      dma_display->setCursor(8, 10);
      dma_display->setTextColor(dma_display->color565(255, 0, 0));
      dma_display->print("NO TIME");
    }
    last_update = millis();
  }
}