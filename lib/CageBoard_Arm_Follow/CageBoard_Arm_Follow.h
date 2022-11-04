#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "PCA9685.h"
#include "SimpleKalmanFilter.h"
#include "Pixetto.h"

class CageBoard_Arm_Follow
{
public:
   CageBoard_Arm_Follow(short rx,short tx);
   ~CageBoard_Arm_Follow();
   void init();
   void reset();
   bool search_ball(short color_num);
   void follow_ball_x(short limit_superior, short limit_inferior);
   void follow_ball_y(short limit_superior, short limit_inferior);
private:
   void servo_init();
   void pixetto_init();
   void follow_ball_z();
public:
   enum Servo_num{
      num0 = 0,
      num1 = 1,
      num2 = 2,
   };
private:
   short flag;
   bool get_ball_flag;
   short servo_min_angle[3];
   short servo_max_angle[3];
   short servo_init_angle[3];

   short servoPos[3];
   short KalmanX,KalmanY;
private:
   PCA9685 driver;
   PCA9685_ServoEval pwmServo[3];
   Pixetto ss;
   SimpleKalmanFilter KalmanFilter_X;
   SimpleKalmanFilter KalmanFilter_Y;
};


