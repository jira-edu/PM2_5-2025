#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// พินเชื่อมต่อ IC CD4511BE
const uint8_t dataPins[] = {2, 3, 4, 5};
const uint8_t digitPins[] = {6, 7, 8};

// พินเซนเซอร์ GP2Y1014AU0F
const int ledPin = 12;
const int analogPin = A0;

// พินเซนเซอร์ DS18B20
const int oneWireBus = 10;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// ตัวแปรเก็บค่าแยกกัน
volatile int pm25Value = 0;
volatile int tempValue = 0;
volatile bool displayMode = true;  // true = PM2.5, false = Temperature

// ฟังก์ชัน Kalman Filter
float estimate = 0.0;
float error_estimate = 0.5;
float error_measure = 1.0;
float kalmanGain = 0.0;

float kalmanFilter(float measurement) {
  kalmanGain = error_estimate / (error_estimate + error_measure);
  estimate = estimate + kalmanGain * (measurement - estimate);
  error_estimate = (1.0 - kalmanGain) * error_estimate;
  return estimate;
}

// ฟังก์ชันส่งข้อมูลไปยัง CD4511BE
void sendToCD4511(int digit) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(dataPins[i], (digit >> i) & 0x01);
  }
}

// ฟังก์ชันแสดงตัวเลขบน 7-segment พร้อม C.
void showNumber(int number, bool showC, bool isPM25) {
  if (isPM25) {
    if (number > 999) number = 999;
    if (number < 0) number = 0;

    int hundreds = (number / 100) % 10;
    int tens = (number / 10) % 10;
    int units = number % 10;

    for (int i = 0; i < 3; i++) {
      digitalWrite(digitPins[0], i == 0 ? LOW : HIGH);
      digitalWrite(digitPins[1], i == 1 ? LOW : HIGH);
      digitalWrite(digitPins[2], i == 2 ? LOW : HIGH);

      int digit = (i == 0) ? hundreds : (i == 1) ? tens : units;
      sendToCD4511(digit);
      vTaskDelay(5 / portTICK_PERIOD_MS);
    }
  } else {
    if (number > 99) number = 99;
    if (number < 0) number = 0;

    int tens = (number / 10) % 10;
    int units = number % 10;

    for (int i = 0; i < 3; i++) {
      digitalWrite(digitPins[0], i == 0 ? LOW : HIGH);
      digitalWrite(digitPins[1], i == 1 ? LOW : HIGH);
      digitalWrite(digitPins[2], i == 2 ? LOW : HIGH);

      if (i == 2 && showC) {
        sendToCD4511(12);
      } else {
        int digit = (i == 0) ? tens : units;
        sendToCD4511(digit);
      }
      vTaskDelay(5 / portTICK_PERIOD_MS);
    }
  }
}

// ฟังก์ชันอ่านค่าฝุ่น PM2.5
int readPM25() {
  digitalWrite(ledPin, LOW);
  delayMicroseconds(280);

  int analogValue = analogRead(analogPin);
  delayMicroseconds(40);
  digitalWrite(ledPin, HIGH);
  delayMicroseconds(9680);

  float voltage = analogValue * (5.0 / 1023.0);
  float dustDensity = max(0.0, (voltage - 0.6) * 1000.0 / 0.5);
  float filteredDust = kalmanFilter(dustDensity);

  return (int)filteredDust;
}

// ฟังก์ชันอ่านค่าอุณหภูมิจาก DS18B20
int readTemperature() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  return (int)tempC;
}

// Task สำหรับแสดงผล 7-segment
void taskDisplay(void *pvParameters) {
  for (uint8_t i = 0; i < 4; i++) pinMode(dataPins[i], OUTPUT);
  for (uint8_t i = 0; i < 3; i++) pinMode(digitPins[i], OUTPUT);

  while (1) {
    int valueToDisplay = displayMode ? pm25Value : tempValue;
    showNumber(valueToDisplay, !displayMode, displayMode);
  }
}

// Task อ่านค่าฝุ่น PM2.5
void taskReadPM25(void *pvParameters) {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  while (1) {
    pm25Value = readPM25();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Task อ่านค่าอุณหภูมิ DS18B20
void taskReadTemperature(void *pvParameters) {
  sensors.begin();
  while (1) {
    tempValue = readTemperature();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Task สลับแสดงผลทุก 10 วินาที
void taskToggleDisplay(void *pvParameters) {
  while (1) {
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    displayMode = !displayMode;
  }
}

void setup() {
  xTaskCreate(taskDisplay, "DisplayTask", 128, NULL, 1, NULL);
  xTaskCreate(taskReadPM25, "ReadPM25Task", 128, NULL, 1, NULL);
  xTaskCreate(taskReadTemperature, "TempTask", 128, NULL, 1, NULL);
  xTaskCreate(taskToggleDisplay, "ToggleTask", 128, NULL, 1, NULL);
  vTaskStartScheduler();
}

void loop() {
  // ไม่ใช้ loop() เมื่อใช้ FreeRTOS
}
