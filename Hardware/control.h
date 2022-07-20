#ifndef  _CONTROL_H
#define  _CONTROL_H
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "tim.h"


int my_abs(int i);
void Load(int moto1,int moto2);
int Vertical(float Med,float Angle,float gyro_Y);
int Velocity(int target_speed,int encoder_left,int encoder_right);
int Turn(int gyro_Z,int turn_speed);
int Follow(short theta,short rho);
#endif
