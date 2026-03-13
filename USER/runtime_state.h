#ifndef __RUNTIME_STATE_H
#define __RUNTIME_STATE_H

#include "sys.h"

typedef struct
{
    u8 is_initialized;

    float x0_init; // 小车位置初始值
    float x2_init; // 摆杆角度初始值

    float x0_raw; // 当前小车位置（原始传感器数据）
    float x2_raw; // 当前摆杆角度（原始传感器数据）

    float x0; // 当前小车位置
    float x1; // 当前小车速度
    float x2; // 当前摆杆角度
    float x3; // 当前摆杆角速度

    float x0_target; // 目标小车位置
    float x1_target; // 目标小车速度
    float x2_target; // 目标摆杆角度
    float x3_target; // 目标摆杆角速度

    float y_pos; // 当前输出电压（角度环）
    float y_w; // 当前输出电压（位置环）

    int qp_status; // MPC/DAQP状态码：>0成功；-1xx setup失败；-2xx update失败；-3xx solve失败；-4xx 参数/状态错误
    float mpc_f0; // MPC线性项首元素
    float mpc_u0; // MPC最优控制首元素
} RuntimeState;

enum {
    Priority_State_Observer = 10,
    Priority_Controller_Pos = 20,
    Priority_Controller_W = 30,
    Priority_Scope = 99,
};

#endif
