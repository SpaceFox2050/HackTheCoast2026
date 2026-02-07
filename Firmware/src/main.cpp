#include <Arduino.h>
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
  static uint32_t lastPrintMs = 0;
  const uint32_t nowMs = millis();

  if (nowMs - lastPrintMs >= 1000) {
    lastPrintMs = nowMs;
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

