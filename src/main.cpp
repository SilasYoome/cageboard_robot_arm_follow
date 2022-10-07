#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "PCA9685.h"
#include "SimpleKalmanFilter.h"
#include "Pixetto.h"

#define sevo1_pin 0
#define servo1_angle_min -90
#define servo1_angle_max 90
#define servo1_angle_init 0

#define sevo2_pin 1
#define servo2_angle_min -90
#define servo2_angle_max 90
#define servo2_angle_init 0

#define sevo3_pin 2
#define servo3_angle_min -90
#define servo3_angle_max 90
#define servo3_angle_init 0

#define sevo4_pin 3
#define servo4_angle_min -90
#define servo4_angle_max 90
#define servo4_angle_init 0

#define rxPin 11
#define txPin 9

Pixetto ss(rxPin, txPin);

PCA9685 driver;
// PCA9685 輸出 = 12 位 = 4096 步
// 20ms 的 2.5% = 0.5ms ; 20ms 的 12.5% = 2.5ms
// 4096 的 2.5% = 102 步；4096 的 12.5% = 512 步
PCA9685_ServoEval pwmServo1; // (0deg, 90deg, 180deg)
PCA9685_ServoEval pwmServo2; // (0deg, 90deg, 180deg)
PCA9685_ServoEval pwmServo3; // (0deg, 90deg, 180deg)
PCA9685_ServoEval pwmServo4; // (0deg, 90deg, 180deg)

SimpleKalmanFilter KalmanFilter_X(1, 1, 0.01);
SimpleKalmanFilter KalmanFilter_Y(1, 1, 0.01);

int servo1Pos, servo2Pos, servo3Pos; // 當前角度
int KalmanX, KalmanY;

void setup() {
  Wire.begin(); // Wire must be started first

  driver.resetDevices(); // Software resets all PCA9685 devices on Wire line
  driver.init(); // Address pins A5-A0 set to B000000
  driver.setPWMFreqServo(); // Set frequency to 50Hz

  driver.setChannelPWM(0, pwmServo1.pwmForAngle(servo1_angle_init));
  delay(10);
  driver.setChannelPWM(1, pwmServo2.pwmForAngle(servo2_angle_init));
  delay(10);
  driver.setChannelPWM(2, pwmServo3.pwmForAngle(servo3_angle_init));
  delay(10);
  driver.setChannelPWM(3, pwmServo4.pwmForAngle(servo4_angle_init));
  delay(10);

  ss.begin();
  ss.enableFunc(Pixetto::FUNC_COLOR_DETECTION);
  delay(10);
  Serial.begin(9600);
  
  servo1Pos = servo1_angle_init+90;
  servo2Pos = servo2_angle_init+90;
  servo3Pos = servo3_angle_init+90;
}

void loop() {
  if (ss.isDetected()) {
    Serial.print("x:");
    Serial.println(ss.getPosX());
    Serial.print("y:");
    Serial.println(ss.getPosY());
    Serial.print("w:");
    Serial.println(ss.getWidth());
    Serial.println("");
    KalmanX = KalmanFilter_X.updateEstimate(ss.getPosX());
    KalmanY = KalmanFilter_Y.updateEstimate(ss.getPosY());
    if (ss.getFuncID() == Pixetto::FUNC_COLOR_DETECTION) {
      if (ss.getTypeID() == Pixetto::COLOR_RED) {
        if (KalmanX >= 50) {
          servo1Pos -= 5;
          if (servo1Pos >= servo1_angle_max+90)
            servo1Pos = 180;
          driver.setChannelPWM(0, pwmServo1.pwmForAngle(servo1Pos-90));
        }
        else if (KalmanX <= 25) {
          servo1Pos += 5;
          if (servo1Pos <= servo1_angle_min+90)
            servo1Pos = 0;
          driver.setChannelPWM(0, pwmServo1.pwmForAngle(servo1Pos-90));
        }
        else if (KalmanY <= 17) {
          servo2Pos += 5;
          if (servo2Pos >= servo2_angle_max+70)
            servo2Pos = 160;
          driver.setChannelPWM(1, pwmServo2.pwmForAngle(servo2Pos-90));

        }
        else if (KalmanY >= 60) {
          servo2Pos -= 5;
          if (servo2Pos <= servo2_angle_min+90)
            servo2Pos = 0;
          driver.setChannelPWM(1, pwmServo2.pwmForAngle(servo2Pos-90));
        }

        if (servo2Pos <= 65) {
          servo3Pos = servo2Pos + 25;
          if (servo3Pos <= servo3_angle_min+90)
            servo3Pos = 0;
          driver.setChannelPWM(2, pwmServo3.pwmForAngle(servo3Pos-90));
        }
        else if (servo2Pos >= 85) {
          servo3Pos = servo2Pos + 10;
          if (servo3Pos >= servo3_angle_max+90)
            servo3Pos = 180;
          driver.setChannelPWM(2, pwmServo3.pwmForAngle(servo3Pos-90));
        }
        else {
          servo3Pos = servo3_angle_init;
          driver.setChannelPWM(2, pwmServo3.pwmForAngle(servo3Pos));
        }

      }
    }
  }
}