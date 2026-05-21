#include <Arduino.h>
#include <ArduinoJson.h>
#include "commands.h"
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

// Используется библиотека I2Cdevlib: I2Cdev.h + MPU6050.h.
// Установить в Arduino IDE: libraries/I2Cdev и libraries/MPU6050.

MPU6050 mpu;
bool mpuInitialized = false;
bool mpuCalibrated = false;

const float DEG_TO_RAD_F = 0.01745329251994f;
const float GYRO_LSB_PER_DEG_S = 65.5f;   // диапазон +/-500 deg/s
const float ACCEL_LSB_PER_G = 8192.0f;    // диапазон +/-4g
const float G_VALUE = 9.80665f;

void initMpu6050() {
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    mpuInitialized = false;
    return;
  }

  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);

  mpuInitialized = true;
  mpuCalibrated = false;
}

bool ensureMpuCalibrated() {
  if (!mpuInitialized) {
    initMpu6050();
  }
  if (!mpuInitialized) {
    return false;
  }

  if (!mpuCalibrated) {
    // Калибровка выполняется только один раз после включения/первого обращения.
    // В этот момент машинка должна стоять неподвижно.
    delay(300);
    mpu.CalibrateGyro(6);
    mpuCalibrated = true;
  }
  return true;
}

bool readMpu6050Data(Mpu6050Data &out) {
  if (!ensureMpuCalibrated()) {
    out.valid = false;
    return false;
  }

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  out.valid = true;
  out.gyro_x = ((float)gx / GYRO_LSB_PER_DEG_S) * DEG_TO_RAD_F;
  out.gyro_y = ((float)gy / GYRO_LSB_PER_DEG_S) * DEG_TO_RAD_F;
  out.gyro_z = ((float)gz / GYRO_LSB_PER_DEG_S) * DEG_TO_RAD_F;
  out.accel_x = ((float)ax / ACCEL_LSB_PER_G) * G_VALUE;
  out.accel_y = ((float)ay / ACCEL_LSB_PER_G) * G_VALUE;
  out.accel_z = ((float)az / ACCEL_LSB_PER_G) * G_VALUE;
  return true;
}

CmdResult handleMpu6050(JsonObject params) {
  (void)params;

  Mpu6050Data data;
  bool ok = readMpu6050Data(data);

  StaticJsonDocument<256> doc;
  doc["valid"] = ok;
  doc["calibrated"] = mpuCalibrated;
  doc["unit_gyro"] = "rad/s";
  doc["unit_accel"] = "m/s2";

  if (ok) {
    doc["gyro_x"] = data.gyro_x;
    doc["gyro_y"] = data.gyro_y;
    doc["gyro_z"] = data.gyro_z;
    doc["omega_x"] = data.gyro_x;
    doc["omega_y"] = data.gyro_y;
    doc["omega_z"] = data.gyro_z;
    doc["accel_x"] = data.accel_x;
    doc["accel_y"] = data.accel_y;
    doc["accel_z"] = data.accel_z;
  }

  String out;
  serializeJson(doc, out);
  return CmdResult(ok ? 0 : 1, out, ok ? "" : "MPU6050 not initialized");
}
