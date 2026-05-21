#include <Arduino.h>
#include <ArduinoJson.h>
#include "commands.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ========== ГЛОБАЛЬНЫЙ ОБЪЕКТ ==========
Adafruit_MPU6050 mpu;
bool mpuInitialized = false;

// ========== ИНИЦИАЛИЗАЦИЯ ==========
void initMpu6050() {
  Wire.begin();
  
  if (!mpu.begin()) {
    mpuInitialized = false;
    return;
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  mpuInitialized = true;
}

// ========== СЫРЫЕ ДАННЫЕ ==========
CmdResult handleMpu6050(JsonObject params) {
  if (!mpuInitialized) {
    return CmdResult(1, "", "MPU6050 not initialized");
  }
  
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  
  // Округляем (избегаем 0. вместо 0.0)
  float gx = round(gyro.gyro.x * 10000) / 10000.0;
  float gy = round(gyro.gyro.y * 10000) / 10000.0;
  float gz = round(gyro.gyro.z * 10000) / 10000.0;
  
  float ax = round(accel.acceleration.x * 10000) / 10000.0;
  float ay = round(accel.acceleration.y * 10000) / 10000.0;
  float az = round(accel.acceleration.z * 10000) / 10000.0;
  
  // Формируем плоский JSON (без вложенных объектов)
  StaticJsonDocument<256> doc;
  doc["gyro_x"] = gx;
  doc["gyro_y"] = gy;
  doc["gyro_z"] = gz;
  doc["accel_x"] = ax;
  doc["accel_y"] = ay;
  doc["accel_z"] = az;
  
  // Сериализуем в строку
  String out;
  serializeJson(doc, out);
  
  // ← ВОЗВРАЩАЕМ через CmdResult (НЕ Serial.println!)
  return CmdResult(0, out, "");
}
