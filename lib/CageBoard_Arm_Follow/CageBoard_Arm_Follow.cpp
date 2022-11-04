#include "CageBoard_Arm_Follow.h"
CageBoard_Arm_Follow::CageBoard_Arm_Follow(short rx, short tx) : ss(rx, tx), KalmanFilter_X(1, 1, 0.01), KalmanFilter_Y(1, 1, 0.01)
{
}

CageBoard_Arm_Follow::~CageBoard_Arm_Follow()
{
}

void CageBoard_Arm_Follow::init() {
   flag = 1;
   get_ball_flag = false;
   servo_min_angle[0] = -90;
   servo_min_angle[1] = -70;
   servo_min_angle[2] = -90;

   servo_max_angle[0] = 90;
   servo_max_angle[1] = 20;
   servo_max_angle[2] = 90;

   servo_init_angle[0] = servoPos[0] = -50;
   servo_init_angle[1] = servoPos[1] = -50;
   servo_init_angle[2] = servoPos[2] = 20;

   servo_init();
   pixetto_init();
}

void CageBoard_Arm_Follow::servo_init() {
   Wire.begin();
   driver.resetDevices();
   driver.init();
   driver.setPWMFreqServo();
   driver.setChannelPWM(Servo_num::num0, pwmServo[0].pwmForAngle(servo_init_angle[0]));
   driver.setChannelPWM(Servo_num::num1, pwmServo[1].pwmForAngle(servo_init_angle[1]));
   driver.setChannelPWM(Servo_num::num2, pwmServo[2].pwmForAngle(servo_init_angle[2]));
}

void CageBoard_Arm_Follow::pixetto_init() {
   ss.begin();
   ss.enableFunc(Pixetto::FUNC_COLOR_DETECTION);
}

void CageBoard_Arm_Follow::reset() {
   flag = 1;
   get_ball_flag = false;
   servoPos[0] = servo_init_angle[0];
   driver.setChannelPWM(Servo_num::num0, pwmServo[0].pwmForAngle(servoPos[0]));
}


bool CageBoard_Arm_Follow::search_ball(short color_num) {
   static long time = 0;
   static long get_ball_time = 0;
   if (get_ball_flag && millis() - get_ball_time >= 200) {
      return true;
   }
   else {
      if (ss.isDetected() && ss.getFuncID() == Pixetto::FUNC_COLOR_DETECTION) {
         if (ss.getTypeID() == color_num) {
            get_ball_time = millis();
            get_ball_flag = true;
         }
      }
   }

   if (millis() - time >= 30) {
      servoPos[0] += flag;
      driver.setChannelPWM(Servo_num::num0, pwmServo[0].pwmForAngle(servoPos[0]));
      if (servoPos[0] >= 90 || servoPos[0] <= -90) {
         flag *= -1;
      }
      time = millis();
   }
   return false;
}

void CageBoard_Arm_Follow::follow_ball_x(short limit_superior, short limit_inferior) {
   if (ss.isDetected()) {
      if (ss.getFuncID() == Pixetto::COLOR_RED)
         KalmanX = KalmanFilter_X.updateEstimate(ss.getPosX());
   }
}