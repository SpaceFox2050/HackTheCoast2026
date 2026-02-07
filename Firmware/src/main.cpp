#include <Arduino.h>
#include <Wire.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <spo2_algorithm.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SDA_PIN 21
#define SCL_PIN 22

// OLED display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// DS1302 pins (ESP32 GPIOs)
static const uint8_t kClkPin = 18;
static const uint8_t kDatPin = 19;
static const uint8_t kRstPin = 23;

// Alarm time (24-hour)
static const uint8_t kAlarmHour = 7;
static const uint8_t kAlarmMinute = 30;
static const uint8_t kAlarmSecond = 0;

ThreeWire rtcWire(kDatPin, kClkPin, kRstPin); // DAT, CLK, RST
RtcDS1302<ThreeWire> rtc(rtcWire);

RtcDateTime alarmTime;
bool alarmTriggered = false;

MAX30105 particleSensor;

// Heart rate and SpO2 calculation variables
const int buffersize = 100;
uint32_t irBuffer[buffersize];
uint32_t redBuffer[buffersize];
uint8_t bufferIndex = 0;

int32_t spo2 = 0;
int8_t validSpo2 = 0;
int32_t heartRate = 0;
int8_t validHeartRate = 0;

static const uint32_t kFingerThreshold = 15000;  // IR value threshold for finger detection



static void PrintDateTime(const RtcDateTime &dt) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%04u-%02u-%02u %02u:%02u:%02u",
           dt.Year(), dt.Month(), dt.Day(),
           dt.Hour(), dt.Minute(), dt.Second());
  Serial.println(buf);
}

static void SetAlarmForToday(const RtcDateTime &now) {
  alarmTime = RtcDateTime(now.Year(), now.Month(), now.Day(),
                          kAlarmHour, kAlarmMinute, kAlarmSecond);
  alarmTriggered = false;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
  } else {
    Serial.println("OLED display initialized!");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Initializing...");
    display.display();
  }

  //initialize max30102 heart rate sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 was not found. Please check wiring/power.");
    while (1);
  }

   Serial.println("MAX30102 Sensor Initialized Successfully!");

  // Configure sensor
  particleSensor.setup();  // Default configuration
  particleSensor.setPulseAmplitudeRed(0x0A);   // Turn on Red LED
  particleSensor.setPulseAmplitudeGreen(0);    // Turn off Green LED
  particleSensor.setPulseAmplitudeIR(0x0A);    // Turn on IR LED

  rtc.Begin();

  if (!rtc.GetIsRunning()) {
    rtc.SetIsRunning(true);
  }

  if (!rtc.IsDateTimeValid()) {
    // RTC lost confidence: set to compile time
    rtc.SetDateTime(RtcDateTime(__DATE__, __TIME__));
  }

  RtcDateTime now = rtc.GetDateTime();
  SetAlarmForToday(now);

  
  Serial.println("RTC initialized. Current time:");
  PrintDateTime(now);
}

void loop() {
  static uint32_t lastVitalsMs = 0;
  static uint32_t lastDisplayMs = 0;
  static uint32_t lastTimeMs = 0;
  const uint32_t nowMs = millis();
  
  // Read sensor data
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  // Check if finger is detected
  if (irValue > kFingerThreshold) {
    // Add to buffer
    irBuffer[bufferIndex] = irValue;
    redBuffer[bufferIndex] = redValue;
    bufferIndex++;

    // Process buffer when full
    if (bufferIndex >= buffersize) {
      bufferIndex = 0;
      // Calculate heart rate and SpO2
      maxim_heart_rate_and_oxygen_saturation(irBuffer, buffersize, redBuffer, 
                                             &spo2, &validSpo2, 
                                             &heartRate, &validHeartRate);
    }
  } else {
    // No finger detected, reset
    validHeartRate = 0;
    validSpo2 = 0;
  }

  // Print heart rate and SpO2 every second
  if (nowMs - lastVitalsMs >= 1000) {
    lastVitalsMs = nowMs;

    Serial.print("HR: ");
    if (validHeartRate) {
      Serial.print(heartRate);
      Serial.print(" bpm");
    } else {
      Serial.print("na");
    }
    
    Serial.print(" | SpO2: ");
    if (validSpo2) {
      Serial.print(spo2);
      Serial.print(" %");
    } else {
      Serial.print("na");
    }
    Serial.println();
  }
//Update OLED display every second
  if (nowMs - lastDisplayMs >= 1000) {
    lastDisplayMs = nowMs;
    RtcDateTime now = rtc.GetDateTime();

    if (rtc.IsDateTimeValid()) {
      display.clearDisplay();
      
      // Display time (large)
      display.setTextSize(2);
      display.setCursor(10, 10);
      char timeStr[10];
      snprintf(timeStr, sizeof(timeStr), "%02u:%02u:%02u", 
               now.Hour(), now.Minute(), now.Second());
      display.println(timeStr);
      
      // Display date
      display.setTextSize(1);
      display.setCursor(10, 35);
      char dateStr[12];
      snprintf(dateStr, sizeof(dateStr), "%04u-%02u-%02u", 
               now.Year(), now.Month(), now.Day());
      display.println(dateStr);
      
      // Display HR and SpO2
      display.setCursor(0, 50);
      display.print("HR:");
      if (validHeartRate) {
        display.print(heartRate);
      } else {
        display.print("--");
      }
      display.print(" SpO2:");
      if (validSpo2) {
        display.print(spo2);
      } else {
        display.print("--");
      }
      
      display.display();
    }
  }

  // 
  // Print time every 5 seconds
  if (nowMs - lastTimeMs >= 5000) {
    lastTimeMs = nowMs;
    RtcDateTime now = rtc.GetDateTime();

    if (!rtc.IsDateTimeValid()) {
      Serial.println("RTC invalid time");
      return;
    }

    PrintDateTime(now);

    // Reset daily alarm at midnight rollover
    if (now.Hour() == 0 && now.Minute() == 0 && now.Second() == 0) {
      SetAlarmForToday(now);
    }

    if (!alarmTriggered && now >= alarmTime) {
      alarmTriggered = true;
      Serial.println("ALARM: time reached");
    }
  }
}

