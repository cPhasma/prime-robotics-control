#include <Arduino.h>
#include <ArduinoJson.h>
#include "commands.h"

CmdResult handleSensors(JsonObject params) {
  (void)params;

  DurationData d;
  Mpu6050Data imu;

  bool distanceOk = readDurationData(d);
  bool imuOk = readMpu6050Data(imu);

  StaticJsonDocument<512> doc;
  doc["valid"] = distanceOk || imuOk;
  doc["distance_valid"] = distanceOk;
  doc["mpu_valid"] = imuOk;
  doc["duration_us"] = d.duration_us;
  doc["distance_cm"] = d.distance_cm;
  doc["unit_gyro"] = "rad/s";
  doc["unit_accel"] = "m/s2";

  if (imuOk) {
    doc["gyro_x"] = imu.gyro_x;
    doc["gyro_y"] = imu.gyro_y;
    doc["gyro_z"] = imu.gyro_z;
    doc["omega_x"] = imu.gyro_x;
    doc["omega_y"] = imu.gyro_y;
    doc["omega_z"] = imu.gyro_z;
    doc["accel_x"] = imu.accel_x;
    doc["accel_y"] = imu.accel_y;
    doc["accel_z"] = imu.accel_z;
  }

  String out;
  serializeJson(doc, out);
  return CmdResult(0, out, "");
}
