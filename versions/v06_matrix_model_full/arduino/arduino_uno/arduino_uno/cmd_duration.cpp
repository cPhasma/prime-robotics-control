#include <Arduino.h>
#include <ArduinoJson.h>
#include "commands.h"

#define TRIG_PIN 12
#define ECHO_PIN 13

// Для 3 м echo длится около 17.5 мс. 22 мс достаточно и не блокирует цикл надолго.
const unsigned long HC_SR04_TIMEOUT_US = 22000UL;

void initDuration() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
}

bool readDurationData(DurationData &out) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  out.duration_us = pulseIn(ECHO_PIN, HIGH, HC_SR04_TIMEOUT_US);
  if (out.duration_us == 0) {
    out.valid = false;
    out.distance_cm = -1.0f;
    return false;
  }

  out.distance_cm = out.duration_us * 0.0343f / 2.0f;
  out.valid = true;
  return true;
}

CmdResult handleDuration(JsonObject params) {
  (void)params;

  DurationData d;
  readDurationData(d);

  StaticJsonDocument<128> doc;
  doc["valid"] = d.valid;
  doc["duration_us"] = d.duration_us;
  doc["distance_cm"] = d.distance_cm;

  String out;
  serializeJson(doc, out);

  // Даже при timeout возвращаем exitCode=0: это не авария алгоритма, а невалидное измерение.
  return CmdResult(0, out, d.valid ? "" : "Ultrasonic timeout");
}
