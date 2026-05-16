#include <Arduino.h>
#include <ArduinoJson.h>
#include "commands.h"

// I2Cdevlib by Jeff Rowberg:
// Arduino IDE -> Library Manager / ZIP library: I2Cdev + MPU6050
#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include <Wire.h>
#endif

// ========== НАСТРОЙКИ ==========
// Используем диапазон +-250 deg/s, потому что у него самая высокая точность.
// Для MPU6050 чувствительность при этом равна 131 LSB / (deg/s).
static const float GYRO_LSB_PER_DPS = 131.0f;
static const float DEG_TO_RAD_F = 0.017453292519943295f;

// ========== ГЛОБАЛЬНЫЙ ОБЪЕКТ ==========
MPU6050 mpu;

static bool mpuInitialized = false;
static bool mpuConnected = false;
static bool gyroCalibrated = false;

// ========== ИНИЦИАЛИЗАЦИЯ ==========
// Здесь только запускаем I2C и проверяем датчик.
// Калибровку специально НЕ делаем здесь каждый раз при измерении.
void initMpu6050() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#endif

  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);

  mpuConnected = mpu.testConnection();
  mpuInitialized = true;
  gyroCalibrated = false;
}

static bool ensureMpuReady() {
  if (!mpuInitialized) {
    initMpu6050();
  }
  if (!mpuConnected) {
    mpuConnected = mpu.testConnection();
  }
  return mpuConnected;
}

static bool calibrateGyroOnce() {
  if (gyroCalibrated) {
    return false;
  }

  // ВАЖНО: во время этой калибровки машинка должна стоять неподвижно.
  // CalibrateGyro() из I2Cdevlib считает смещение гироскопа и записывает offset'ы.
  delay(300);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.CalibrateGyro(6);
  gyroCalibrated = true;
  return true;
}

static float rawGyroToRadSec(int16_t rawValue) {
  float degPerSec = ((float)rawValue) / GYRO_LSB_PER_DPS;
  return degPerSec * DEG_TO_RAD_F;
}

// ========== УГЛОВАЯ СКОРОСТЬ ==========
// Возвращаем уже физическую величину: rad/s, а не сырые LSB.
CmdResult handleMpu6050(JsonObject params) {
  if (!ensureMpuReady()) {
    return CmdResult(1, "", "MPU6050 not connected");
  }

  bool calibratedNow = calibrateGyroOnce();

  int16_t rawGx = 0;
  int16_t rawGy = 0;
  int16_t rawGz = 0;
  mpu.getRotation(&rawGx, &rawGy, &rawGz);

  float gyroX = rawGyroToRadSec(rawGx);
  float gyroY = rawGyroToRadSec(rawGy);
  float gyroZ = rawGyroToRadSec(rawGz);

  StaticJsonDocument<384> doc;
  doc["valid"] = true;
  doc["unit"] = "rad/s";
  doc["calibrated"] = gyroCalibrated;
  doc["calibrated_now"] = calibratedNow;

  float gxOut = round(gyroX * 1000000.0f) / 1000000.0f;
  float gyOut = round(gyroY * 1000000.0f) / 1000000.0f;
  float gzOut = round(gyroZ * 1000000.0f) / 1000000.0f;

  // Основные поля. Сервер использует gyro_z / omega_z как угловую скорость вокруг Z.
  doc["gyro_x"] = gxOut;
  doc["gyro_y"] = gyOut;
  doc["gyro_z"] = gzOut;

  // Дублируем под omega_*, чтобы было явно понятно: это угловая скорость.
  doc["omega_x"] = gxOut;
  doc["omega_y"] = gyOut;
  doc["omega_z"] = gzOut;

  String out;
  serializeJson(doc, out);
  return CmdResult(0, out, "");
}
