#include <Arduino.h>
#include "Pixetto.h"
#include "CageBoard_Arm_Follow.h"

enum pixetto_color {
  red = 1,
  yellow = 2,
  blue = 4,
  purple = 5
};

unsigned short rx = 11;
unsigned short tx = 9;

unsigned short button_pin[4] = { A1,A2,A3,A4 }; // red, yellow, blue, purple
//bool button_status[4];
bool last_button_status[4];
bool current_button_status[4];
unsigned short color;
unsigned long last_debounce_time = 0;
unsigned long debounce_delay = 100;

unsigned long time = 0;
bool flag = true;

CageBoard_Arm_Follow CageBoard(rx, tx);

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 4; i++) {
    pinMode(button_pin[i], INPUT_PULLUP);
    last_button_status[i]=LOW;
  }
  CageBoard.init();
}

void loop() {
  for (int i = 0;i < 4;i++) {
    current_button_status[i] = digitalRead(button_pin[i]);
  }

  if (flag == false) {
    Serial.println(flag);
    if (CageBoard.search_ball(color)) {
      flag = true;
    }
  }
  else if (flag == true) {
    Serial.println(flag);
    if (current_button_status[0] == LOW || current_button_status[1] == LOW ||
      current_button_status[2] == LOW || current_button_status[3] == LOW)
    {
      last_debounce_time = millis();
    }

    if ((millis() - last_debounce_time) > debounce_delay) {
      for (int i = 0;i < 4;i++) {
        last_button_status[i] = digitalRead(button_pin[i]);
      }
      for (int i = 0;i < 4;i++) {
        if (current_button_status[i] != last_button_status[i]) {
          switch (i)
          {
          case 0:
            color = pixetto_color::red;
            break;
          case 1:
            color = pixetto_color::yellow;
            break;
          case 2:
            color = pixetto_color::blue;
            break;
          case 3:
            color = pixetto_color::purple;
            break;
          default:
            break;
          }
          flag = false;
          CageBoard.reset();
          break;
        }

      }
    }
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
