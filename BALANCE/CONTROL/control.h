#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
#define PI 3.14159265
#define ZHONGZHI 3100
#define FILTERING_TIMES  4

#define POSITION_MIDDLE 7925
#define ANGLE_ORIGIN 1020
#define ANGLE_MIDDLE 3100
extern  int Velocity_Pwm;
extern	float Balance_Pwm,Position_Pwm;
extern	float x0,x1,x2,x3;
extern  float k0,k1,k2,k3;
extern  float k0_add,k1_add,k2_add,k3_add;
extern	float Position_Current,Radian_Current,Position_0;

extern float Bias;                       //倾角偏差
extern float Last_Bias,D_Bias;    //PID相关变量
extern int balance;                     //PWM返回值 

extern float Position_PWM,Last_Position,Position_Bias,Position_Differential;
extern float Position_Least;

int TIM1_UP_IRQHandler(void);
int Balance(float angle);
int Position(int Encoder);
void Set_Pwm(int moto);
void Key(void);
void Xianfu_Pwm(void);
u8 Turn_Off(int voltage);
int myabs(int a);
void Find_Zero(void);
void Auto_run(void);
int Incremental_PI (int Encoder,int Target);
void Get_D_Angle_Balance(void);
int Mean_Filter(int sensor);
int Position_PID (int Encoder,int Target);
int Pre_Position(int Encoder);
#endif
