#ifndef __RUNTIME_STATE_H
#define __RUNTIME_STATE_H

#include "sys.h"

// 小车位置;小车速度;摆杆角度;摆杆角速度
#define STATE_DIM 4

typedef struct
{
    u8 is_initialized;


    float x_init[STATE_DIM]; // 初始值
    float x_raw[STATE_DIM]; // 传感器数据，只有小车位置和摆杆角度值有效
    float x[STATE_DIM]; // 当前值
    float x_d[STATE_DIM]; // 目标值

    float y_pos; // 当前输出电压（位置环）
    float y_w; // 当前输出电压（角度环）

    float x_sim[STATE_DIM]; // 板载仿真值
} RuntimeState;

enum {
    Priority_State_Observer = 10,
    Priority_Simulation = 11,
    Priority_Controller_Pos = 20,
    Priority_Controller_W = 30,
    Priority_Scope = 99,
};

#endif
