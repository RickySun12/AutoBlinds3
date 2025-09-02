#include "AutoBlinds.hpp"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include "time.h"
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_VL53L0X.h>
#include <driver/i2s.h>
#include <math.h>

// -------------------- Constants --------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define WIFI_TIMEOUT 45

// Buttons & encoders
#define searchToggle 25
#define alarmToggle 26
#define overrideToggle 27
#define clkRotary 32
#define dtRotary 33
#define swRotary 14

// I2S Microphone
#define I2S_WS 12
#define I2S_SCK 18
#define I2S_SD 13
#define SAMPLE_RATE 16000
#define SAMPLE_BUFFER_SIZE 1024
#define CLAP_THRESHOLD 1350

// WiFi / Time
static const char* ssid     = "Telstra062913";
static const char* password = "9dvwrtbqd7";
static const char* ntpServer = "pool.ntp.org";
static const long  gmtOffset_sec = 10 * 3600;  // GMT+10
static const int   daylightOffset_sec = 3600;  // DST+1

// ESP-NOW peer 
static uint8_t broadcastAddress[] = {0x24, 0x6F, 0x28, 0x7A, 0xAE, 0x7C};

namespace {
  // Display / sensors
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
  Adafruit_VL53L0X lox;

  // Clap/mic
  int16_t i2sBuffer[SAMPLE_BUFFER_SIZE];
  unsigned long timeLastDoubleClap = 0;

  // Flash times
  int flashStart = 600;
  int flashEnd   = 1000;

  // Flags
  bool searchFlag = false;
  bool alarmFlag = false;
  bool overrideFlag = false;
  bool swRotaryFlag = false;
  volatile bool previousAstate = false;
  volatile double encoderValue = 0;

  // Alarm state
  int  alarmHr = 7;
  int  alarmMin = 0;
  int  digitSelected = 0;
  bool changeDigit = false;
  bool alarmAm = true;
  bool inBed = false;

  bool wifiOk = true;

  // Forward decls
  void espNowInit();
  void espNowSend(bool data);
  void i2s_install();
  void printTime(struct tm timeinfo);
  void search();
  void changeAlarm();
  String alarmToString();
  void readPins();
  void selectFlash(String result, double *previousEncoderValue, unsigned long *lastFlashTimePr);
  void changeAlarmHelper(int upOrDown);
  int  readMicData();
  bool determinClap();

  void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
      Serial.print("esp now fail");
    }
  }

  void i2s_install() {
    i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_I2S,
      .intr_alloc_flags = 0,
      .dma_buf_count = 8,
      .dma_buf_len = 512,
      .use_apll = false
    };
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);

    i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_SCK,
      .ws_io_num = I2S_WS,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = I2S_SD
    };
    i2s_set_pin(I2S_NUM_0, &pin_config);
  }

  void espNowInit() {
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    esp_now_register_send_cb(OnDataSent);

    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }
  }

  void espNowSend(bool data) {
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&data, sizeof(data));
    if (result == ESP_OK) {
      Serial.println("Sending confirmed");
    } else {
      Serial.println("Sending error");
    }
  }

  void printTime(struct tm timeinfo) {
    char timeStr[18];
    strftime(timeStr, sizeof(timeStr), "%I:%M:%S", &timeinfo);
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(timeStr);
    display.setTextSize(1);
    // Keep your original formatting call:
    display.println(&timeinfo, " %P");
    display.display();
    display.clearDisplay();
  }

  void search() {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {
      Serial.print("Distance: ");
      Serial.print(measure.RangeMilliMeter);
      Serial.println(" mm");
    } else {
      // Out of range
    }
  }

  String alarmToString() {
    String result;
    String hrs = String(alarmHr);
    String mins = String(alarmMin);
    if (alarmHr < 10) result = "0" + hrs + ":";
    else result = hrs + ":";

    if (alarmMin < 10) result += "0" + String(alarmMin);
    else result += String(alarmMin);

    result += (alarmAm ? "AM" : "PM");
    return result;
  }

  void changeAlarmHelper(int upOrDown) {
    Serial.print("upOrDown: ");
    Serial.println(upOrDown);

    if (digitSelected != 3 && digitalRead(swRotary) == HIGH) {
      changeDigit = !changeDigit;
      delay(200);
    }
    if (changeDigit) { flashStart = 200; flashEnd = 400; }
    else { flashStart = 600; flashEnd = 1000; }

    if (digitSelected == 0 && changeDigit) {
      if (upOrDown == 1) { alarmHr++; if (alarmHr > 12) alarmHr = 1; }
      else if (upOrDown == -1) { alarmHr--; if (alarmHr < 1) alarmHr = 12; }
      Serial.print("alarmHr: ");
      Serial.println(alarmHr);
    } else if (digitSelected == 1 && changeDigit) {
      if (upOrDown == 1) { alarmMin++; if (alarmMin > 59) alarmMin = 0; }
      else if (upOrDown == -1) { alarmMin--; if (alarmMin < 0) alarmMin = 59; }
    } else if (digitSelected == 2 && changeDigit) {
      if (upOrDown == 1 || upOrDown == -1) alarmAm = !alarmAm;
    }
  }

  void selectFlash(String result, double *previousEncoderValue, unsigned long *lastFlashTimePr) {
    delay(50);
    String setAlarm = "Set Alarm Time";
    int upOrDown = 0;

    if ((*previousEncoderValue) < encoderValue) {
      if (!changeDigit) {
        digitSelected++;
        if (digitSelected > 3) digitSelected = 0;
      }
      upOrDown = 1;
    } else if ((*previousEncoderValue) > encoderValue) {
      if (!changeDigit) {
        digitSelected--;
        if (digitSelected < 0) digitSelected = 3;
      }
      upOrDown = -1;
    }

    (*previousEncoderValue) = encoderValue;

    if (digitSelected == 0 && (*lastFlashTimePr) + flashStart < millis()) {
      result.setCharAt(0, ' ');
      result.setCharAt(1, ' ');
      if ((*lastFlashTimePr) + flashEnd < millis()) *lastFlashTimePr = millis();
    } else if (digitSelected == 1 && (*lastFlashTimePr) + flashStart < millis()) {
      result.setCharAt(3, ' ');
      result.setCharAt(4, ' ');
      if ((*lastFlashTimePr) + flashEnd < millis()) *lastFlashTimePr = millis();
    } else if (digitSelected == 2 && (*lastFlashTimePr) + flashStart < millis()) {
      result.setCharAt(5, ' ');
      result.setCharAt(6, ' ');
      if ((*lastFlashTimePr) + flashEnd < millis()) *lastFlashTimePr = millis();
    } else if (digitSelected == 3 && (*lastFlashTimePr) + flashStart < millis()) {
      setAlarm = " ";
      if ((*lastFlashTimePr) + flashEnd < millis()) *lastFlashTimePr = millis();
    }

    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(result);
    display.setCursor(0, 18);
    display.setTextSize(1);
    display.print(setAlarm);
    display.display();

    // Exit "set alarm"
    if (digitSelected == 3 && digitalRead(swRotary) == HIGH) {
      swRotaryFlag = !swRotaryFlag;
      delay(200);
    }
    changeAlarmHelper(upOrDown);
  }

  void changeAlarm() {
    if (swRotaryFlag == true) {
      double previousEncoderValue = encoderValue;
      unsigned long lastFlashTime = millis();
      while (swRotaryFlag) {
        String result = alarmToString();
        selectFlash(result, &previousEncoderValue, &lastFlashTime);
      }
    }
  }

  void readPins() {
    display.clearDisplay();

    if (digitalRead(searchToggle) == HIGH) { searchFlag = !searchFlag; delay(200); }
    if (digitalRead(alarmToggle)  == HIGH) { alarmFlag  = !alarmFlag;  delay(200); }
    if (digitalRead(overrideToggle) == HIGH) { overrideFlag = !overrideFlag; delay(200); }
    if (digitalRead(swRotary) == HIGH) { swRotaryFlag = !swRotaryFlag; delay(200); }

    if (searchFlag) {
      display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 16); display.print("Search-On");
    }
    if (alarmFlag) {
      display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
      display.setCursor(64, 16); display.print("Alarm-On");
      display.setCursor(64, 24); display.print(alarmToString());
    }
    if (overrideFlag) {
      display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 24); display.print("Override");
    }
  }

  int readMicData() {
    size_t bytes_read;
    i2s_read(I2S_NUM_0, i2sBuffer, sizeof(i2sBuffer), &bytes_read, portMAX_DELAY);
    int32_t sum = 0;
    for (int i = 0; i < (int)(bytes_read / 2); i++) {
      sum += abs(i2sBuffer[i]);
    }
    return sum / (int)(bytes_read / 2);
  }

  bool determinClap() {
    int data = readMicData();
    Serial.println(data);
    int clapCounter = 0;
    unsigned long currentTime = 0;

    if (data > CLAP_THRESHOLD) {
      clapCounter++;
      currentTime = millis();
      while (300 + currentTime > millis()) {
        if (readMicData() > CLAP_THRESHOLD) { clapCounter++; }
      }
      int clapCounterSave = clapCounter;
      currentTime = millis();
      while (200 + currentTime > millis()) {
        if (readMicData() > CLAP_THRESHOLD) { clapCounter++; }
      }
      if (clapCounterSave == clapCounter && clapCounter > 1) {
        Serial.println("trigger");
        return true;
      }
    }
    return false;
  }
} 

// -------------------- Public API --------------------
void AutoBlinds::setupAll() {
  Serial.begin(115200);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed!");
    while (true) { delay(1000); }
  }

  // WiFi connect
  WiFi.begin(ssid, password);
  int cursor = 0;
  int t = 0;
  display.clearDisplay();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.print(".");
    cursor++; t++;
    display.display();

    if (WIFI_TIMEOUT - t <= 0) {
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.println("Wifi Fail");
      display.display();
      delay(2000);
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("No Time");
      display.display();
      delay(2000);
      wifiOk = false;
      break;
    } else if (cursor >= 10) {
      cursor = 0;
      display.clearDisplay();
      display.setCursor(0, 0);
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    configTzTime("AEST-10AEDT,M10.1.0,M4.1.0/3", ntpServer);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println(ssid);
    display.print("success");
    display.display();
    delay(2000);
  }

  // ToF
  if (!lox.begin()) {
    Serial.println("Failed to initialize VL53L0X! Check connections.");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.print("TOFS ERROR");
    display.display();
    while (1) { delay(10); }
  }
  Serial.println("VL53L0X Initialized.");

  // Pins
  pinMode(searchToggle, INPUT);
  pinMode(alarmToggle, INPUT);
  pinMode(overrideToggle, INPUT);
  pinMode(clkRotary, INPUT_PULLUP);
  pinMode(dtRotary, INPUT_PULLUP);
  pinMode(swRotary, INPUT_PULLUP);

  previousAstate = digitalRead(clkRotary);
  attachInterrupt(digitalPinToInterrupt(clkRotary), AutoBlinds::readEncoderISR, CHANGE);

  // Microphone
  i2s_install();

  // ESP-NOW
  espNowInit();
  espNowSend(true);
}

void AutoBlinds::loopAll() {
  if (wifiOk) {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
      Serial.println("NTC fail");
    }
    printTime(timeinfo);
    search();
    changeAlarm();
    readPins();
    determinClap();
  }
}

void IRAM_ATTR AutoBlinds::readEncoderISR() {
  bool aState = digitalRead(clkRotary);
  bool bState = digitalRead(dtRotary);
  if (aState != previousAstate) {
    if (aState != bState) { encoderValue++; }  // Clockwise
    else { encoderValue--; }                    // Counterclockwise
  }
  previousAstate = aState;
}
