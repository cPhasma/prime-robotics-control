#include <Arduino.h>
#include <ArduinoJson.h>
#include "commands.h"

#define ML_Ctrl 2
#define ML_PWM  5
#define MR_Ctrl 4
#define MR_PWM  6

void setMotor(int ctrlPin, int pwmPin, const char* direction, int pwm) {
  if (strcmp(direction, "forward") == 0) {
    digitalWrite(ctrlPin, HIGH);
    analogWrite(pwmPin, pwm);
  } else if (strcmp(direction, "backward") == 0) {
    digitalWrite(ctrlPin, LOW);
    analogWrite(pwmPin, pwm);
  } else {
    digitalWrite(ctrlPin, LOW);
    analogWrite(pwmPin, 0);
  }
}

void initMotor() {
  pinMode(ML_Ctrl, OUTPUT);
  pinMode(ML_PWM, OUTPUT);
  pinMode(MR_Ctrl, OUTPUT);
  pinMode(MR_PWM, OUTPUT);

  analogWrite(ML_PWM, 0);
  analogWrite(MR_PWM, 0);
  digitalWrite(ML_Ctrl, LOW);
  digitalWrite(MR_Ctrl, LOW);
}

CmdResult handleMotor(JsonObject params) {
  int left_pwm = params["left_pwm"] | 0;
  int right_pwm = params["right_pwm"] | 0;

  const char* left_dir = params["left_dir"] | "forward";
  const char* right_dir = params["right_dir"] | "forward";

  left_pwm = constrain(left_pwm, 0, 255);
  right_pwm = constrain(right_pwm, 0, 255);

  setMotor(ML_Ctrl, ML_PWM, left_dir, left_pwm);
  setMotor(MR_Ctrl, MR_PWM, right_dir, right_pwm);

  return CmdResult(0, F("ok"), "");
}
