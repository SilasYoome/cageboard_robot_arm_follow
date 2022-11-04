#include <Arduino.h>
#include "CageBoard_Arm_Follow.h"

enum pixetto_color {
  red = 1,
  yellow = 2,
  blue = 4,
  purple = 5
};

unsigned short rx = 11;
unsigned short tx = 9;

unsigned long time = 0;
bool flag = true;

CageBoard_Arm_Follow CageBoard(rx, tx);

void setup() {
  CageBoard.init();
}

void loop() {

  if (flag == false) {
    Serial.println(flag);
    if (CageBoard.search_ball(pixetto_color::red)) {
      flag = true;
    }
  }
  else if (flag == true) {
  }
}

/*
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
          if (servo1Pos >= servo1_angle_max + 90)
            servo1Pos = 180;
          driver.setChannelPWM(0, pwmServo1.pwmForAngle(servo1Pos - 90));
        }
        else if (KalmanX <= 25) {
          servo1Pos += 5;
          if (servo1Pos <= servo1_angle_min + 90)
            servo1Pos = 0;
          driver.setChannelPWM(0, pwmServo1.pwmForAngle(servo1Pos - 90));
        }
        else if (KalmanY <= 17) {
          servo2Pos += 5;
          if (servo2Pos >= servo2_angle_max + 30)
            servo2Pos = 120;
          driver.setChannelPWM(1, pwmServo2.pwmForAngle(servo2Pos - 90));

        }
        else if (KalmanY >= 60) {
          servo2Pos -= 5;
          if (servo2Pos <= servo2_angle_min + 90)
            servo2Pos = 0;
          driver.setChannelPWM(1, pwmServo2.pwmForAngle(servo2Pos - 90));
        }

        if (servo2Pos <= 65) {
          servo3Pos = servo2Pos + 25;
          if (servo3Pos <= servo3_angle_min + 90)
            servo3Pos = 0;
          driver.setChannelPWM(2, pwmServo3.pwmForAngle(servo3Pos - 90));
        }
        else if (servo2Pos >= 85) {
          servo3Pos = servo2Pos + 10;
          if (servo3Pos >= servo3_angle_max + 90)
            servo3Pos = 180;
          driver.setChannelPWM(2, pwmServo3.pwmForAngle(servo3Pos - 90));
        }
        else {
          servo3Pos = servo3_angle_init;
          driver.setChannelPWM(2, pwmServo3.pwmForAngle(servo3Pos));
        }

      }
    }
  }
}*/
