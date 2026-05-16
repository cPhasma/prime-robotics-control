#include <Arduino.h>
#include <ArduinoJson.h>
#include "commands.h"

#define TRIG_PIN 12
#define ECHO_PIN 13

void initDuration() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
}

CmdResult handleDuration(JsonObject params) {
  (void)params;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // 30000 мкс примерно хватает до 5 м. Если эхо пропало, не возвращаем ошибку,
  // а отдаём valid=false. Тогда серверный регулятор не падает, а берёт последнее
  // корректное расстояние и продолжает работу.
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);

  if (duration == 0) {
    return CmdResult(0, F("{\"valid\":false,\"duration_us\":0,\"distance_cm\":-1}"), "");
  }

  float distanceCm = duration * 0.0343f / 2.0f;
  String payload = String("{\"valid\":true,\"duration_us\":") + duration +
                   String(",\"distance_cm\":") + String(distanceCm, 2) +
                   String("}");
  return CmdResult(0, payload, "");
}
