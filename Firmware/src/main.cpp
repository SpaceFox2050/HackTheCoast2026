#include <Arduino.h>
#include <Wire.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <spo2_algorithm.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>

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

MAX30105 maxSensor;

static const uint8_t kSampleRateHz = 100;
static const uint8_t kBufferSize = 100;
static const uint32_t  kFingerThreshold = 1000;

uint32_t irBuffer[kBufferSize];
uint32_t redBuffer[kBufferSize];
uint8_t sampleIndex = 0;

int32_t lastSpO2 = 0;
int32_t lastHeartRate = 0;
int8_t validSpO2 = 0;
int8_t validHeartRate = 0;

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

  Wire.begin();

  if (!maxSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found. Check wiring.");
  } else {
    uint8_t ledBrightness = 60; // 0..255
    uint8_t sampleAverage = 4;  // 1, 2, 4, 8, 16, 32
    uint8_t ledMode = 2;        // 1 = Red only, 2 = Red + IR
    int sampleRate = kSampleRateHz;
    int pulseWidth = 411;       // 69, 118, 215, 411
    int adcRange = 4096;        // 2048, 4096, 8192, 16384

    maxSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    maxSensor.setPulseAmplitudeRed(0x1F);
    maxSensor.setPulseAmplitudeIR(0x1F);
    maxSensor.setPulseAmplitudeGreen(0);
  }

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
  static uint32_t lastTimeMs = 0;
  const uint32_t nowMs = millis();

  if (maxSensor.available()) {
    uint32_t irValue = maxSensor.getIR();
    uint32_t redValue = maxSensor.getRed();
    maxSensor.nextSample();

    if (irValue >= kFingerThreshold) {
      irBuffer[sampleIndex] = irValue;
      redBuffer[sampleIndex] = redValue;
      sampleIndex++;

      if (sampleIndex >= kBufferSize) {
        sampleIndex = 0;
        maxim_heart_rate_and_oxygen_saturation(irBuffer, kBufferSize,
                                               redBuffer, &lastSpO2, &validSpO2,
                                               &lastHeartRate, &validHeartRate);
      }
    } else {
      validSpO2 = 0;
      validHeartRate = 0;
    }
  }

  if (nowMs - lastVitalsMs >= 1000) {
    lastVitalsMs = nowMs;

    Serial.print("HR: ");
    if (validHeartRate) {
      Serial.print(lastHeartRate);
    } else {
      Serial.print("na");
    }
    Serial.print(" bpm, SpO2: ");
    if (validSpO2) {
      Serial.print(lastSpO2);
      Serial.println(" %");
    } else {
      Serial.println("na");
    }
  }

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

