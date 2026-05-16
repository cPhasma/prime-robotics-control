#ifndef ARDUINO_COMMANDS_H
#define ARDUINO_COMMANDS_H

#include <Arduino.h>
#include <ArduinoJson.h>

// ========== РЕЗУЛЬТАТ КОМАНДЫ ==========
struct CmdResult {
  int exitCode;
  String output;
  String error;
  
  CmdResult(int code, const String& out, const String& err) 
    : exitCode(code), output(out), error(err) {}
};

// ========== ТИПЫ ФУНКЦИЙ ==========
typedef CmdResult (*CmdHandler)(JsonObject params);
typedef void (*CmdInit)();

// ========== ЗАПИСЬ КОМАНДЫ ==========
struct CommandEntry {
  const char* name;
  const char* description;
  const char* params_json;
  CmdHandler handler;
  CmdInit init;
};

// ========== ОБЪЯВЛЕНИЯ ==========
extern CmdResult handlePing(JsonObject params);
extern CmdResult handleMotor(JsonObject params);
extern CmdResult handleDuration(JsonObject params);
extern CmdResult handleMpu6050(JsonObject params);
extern void initMotor();
extern void initDuration();
extern void initMpu6050();

// ========== JSON СХЕМЫ В PROGMEM ==========

// Команда ping (простая, без параметров)
const char CMD_PING_NAME[] PROGMEM = "ping";
const char CMD_PING_DESC[] PROGMEM = "Проверка связи";

// Команда Duration для ультразвука
const char CMD_DURATION_NAME[] PROGMEM = "duration";
const char CMD_DURATION_DESC[] PROGMEM = "Получить длительность возврата Ультразвукового датичка";

// Команда mpu6050
const char CMD_MPU6050_NAME[] PROGMEM = "mpu6050";
const char CMD_MPU6050_DESC[] PROGMEM = "Получить угловую скорость MPU6050 в rad/s с однократной калибровкой";

// Команда motor
const char CMD_MOTOR_NAME[] PROGMEM = "motor";
const char CMD_MOTOR_DESC[] PROGMEM = "Управление моторами";

const char CMD_MOTOR_PARAMS[] PROGMEM = R"({
  "left_pwm":{"type":"integer","min":0,"max":255},
  "right_pwm":{"type":"integer","min":0,"max":255},
  "left_dir":{"type":"string","enum":["forward","backward"]},
  "right_dir":{"type":"string","enum":["forward","backward"]}
})";

// ========== ТАБЛИЦА КОМАНД В PROGMEM ==========
const CommandEntry CMD_TABLE[] PROGMEM = {
  { CMD_PING_NAME, CMD_PING_DESC, nullptr, handlePing, nullptr },
  { CMD_MOTOR_NAME, CMD_MOTOR_DESC, CMD_MOTOR_PARAMS, handleMotor, initMotor },
  {CMD_DURATION_NAME, CMD_DURATION_DESC, nullptr, handleDuration, initDuration},
  { CMD_MPU6050_NAME, CMD_MPU6050_DESC, nullptr, handleMpu6050, initMpu6050 }
};

const int CMD_COUNT = sizeof(CMD_TABLE) / sizeof(CMD_TABLE[0]);

// ========== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ==========

static inline bool commandNameEquals(const char* input, const char* flashName) {
  return strcmp_P(input, reinterpret_cast<PGM_P>(flashName)) == 0;
}

static inline void copyFlashString(const char* flashText, char* buffer, size_t bufferSize) {
  if (!buffer || bufferSize == 0) return;
  strncpy_P(buffer, reinterpret_cast<PGM_P>(flashText), bufferSize - 1);
  buffer[bufferSize - 1] = '\0';
}

#endif
