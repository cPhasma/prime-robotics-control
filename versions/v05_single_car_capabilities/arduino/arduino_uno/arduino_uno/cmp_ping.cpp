#include <Arduino.h>
#include <ArduinoJson.h>
#include "commands.h"

extern int freeMemory();

CmdResult handlePing(JsonObject params) {
  String out = F("pong");
  out += F(" heap=");
  out += String(freeMemory());
  out += F(" uptime=");
  out += String(millis() / 1000);
  out += F("s");
  return CmdResult(0, out, "");
}
