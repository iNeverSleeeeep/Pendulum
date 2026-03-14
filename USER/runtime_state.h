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
    float mpc_primal0; // DAQP结果 primal[0]
    float mpc_x0; // DAQP工作区 x[0]
    float mpc_h00; // MPC Hessian(0,0)
    float mpc_v0; // DAQP工作区 v[0]
    float mpc_u_internal0; // DAQP工作区 u[0]
    float mpc_rinv0; // DAQP工作区 Rinv/RinvD 首元素
    float mpc_iter; // DAQP迭代次数
    float mpc_f1; // MPC线性项第二个元素
    float mpc_f2; // MPC线性项第三个元素
    float mpc_v1; // DAQP工作区 v[1]
    float mpc_v2; // DAQP工作区 v[2]
    float mpc_v_null; // v是否为空(1为空)
    float mpc_rinv_null; // Rinv是否为空(1为空)
    float mpc_rinvd_null; // RinvD是否为空(1为空)
    float mpc_quadprog0; // daqp_quadprog自测x[0]
    float mpc_quadprog_exit; // daqp_quadprog自测exitflag
    float mpc_quadprog0_b; // 第二种H格式自测x[0]
    float mpc_quadprog_exit_b; // 第二种H格式自测exitflag
    float mpc_quadprog10_0; // 10维格式A自测x[0]
    float mpc_quadprog10_exit; // 10维格式A自测exitflag
    float mpc_quadprog10_0_b; // 10维格式B自测x[0]
    float mpc_quadprog10_exit_b; // 10维格式B自测exitflag
} RuntimeState;

enum {
    Priority_State_Observer = 10,
    Priority_Controller_Pos = 20,
    Priority_Controller_W = 30,
    Priority_Scope = 99,
};

#endif
