#include "core_framework.h"
#include "matrix_lite.h"
#include "runtime_state.h"
#include <string.h>

#define HINF_STATE_DIM 5
#define HINF_INPUT_DIM STATE_DIM

typedef struct {
    // 1. 定义控制器状态
    float x_ctrl[HINF_STATE_DIM];
    float x_ctrl_next[HINF_STATE_DIM];

    // 2. 离线算好的矩阵数组 (从 MATLAB 导出)
    float Ak[HINF_STATE_DIM][HINF_STATE_DIM];
    float Bk[HINF_STATE_DIM][HINF_INPUT_DIM];
    float Ck[1][HINF_STATE_DIM];
    float Dk[1][HINF_INPUT_DIM];
} HinfStateData;

HinfStateData g_hinf_stata_data = {
    {0},
    {0},
    {    
    {1.000046, 0.005076, 0.000702, 0.000333, 0.000000, },
    {0.022679, 1.051554, 0.257196, 0.042230, 0.000063, },
    {0.000177, 0.001420, 0.995927, -0.003662, 0.000000, },
    {-0.080908, -0.178232, -0.774847, 0.798785, -0.000226, },
    {0.000104, 0.000515, 0.009381, -0.002236, 0.995013, },
    },
    {
    {0.000000, 0.000029, -0.000156, -0.000245, },
    {0.000000, -0.000075, 0.000457, 0.000726, },
    {0.000000, -0.001017, 0.005730, 0.008982, },
    {-0.000000, -0.005424, 0.030606, 0.048323, },
    {0.000000, -0.000278, 0.001558, 0.002453, },
    },
    {
    {6.922527, 28.682286, 82.119465, 12.414365, 0.019349, },
    },
    {
    {0.000000, -0.079594, 0.449261, 0.706727, },
    },
};

static void Module_HInf_Reset(void *user_ctx)
{
    HinfStateData* hinf_stata_data = (HinfStateData*)user_ctx;
    memset(hinf_stata_data->x_ctrl, 0, sizeof(hinf_stata_data->x_ctrl));
    memset(hinf_stata_data->x_ctrl_next, 0, sizeof(hinf_stata_data->x_ctrl_next));
}

static void Module_HInf_Update(float dt_s, void *user_ctx)
{
    float u = 0.0f;
    float ak_x[HINF_STATE_DIM];
    float bk_x[HINF_STATE_DIM];
    HinfStateData* hinf_stata_data = (HinfStateData*)user_ctx;
    RuntimeState *state = (RuntimeState *)Framework_DataGet("runtime_state", 0);
    (void)dt_s;

    if (state == 0) {
        return;
    }

    // 当前控制输出 u = Ck * x_ctrl + Dk * x_measure
    u = MatLite_StateSpaceOutput(&hinf_stata_data->Ck[0][0],
                                 HINF_STATE_DIM,
                                 hinf_stata_data->x_ctrl,
                                 &hinf_stata_data->Dk[0][0],
                                 HINF_INPUT_DIM,
                                 state->x);

    // 控制器内部状态更新 x_ctrl(k+1) = Ak * x_ctrl(k) + Bk * x_measure(k)
    MatLite_MatVecMul(&hinf_stata_data->Ak[0][0],
                      HINF_STATE_DIM,
                      HINF_STATE_DIM,
                      hinf_stata_data->x_ctrl,
                      ak_x);
    MatLite_MatVecMul(&hinf_stata_data->Bk[0][0],
                      HINF_STATE_DIM,
                      HINF_INPUT_DIM,
                      state->x,
                      bk_x);
    MatLite_VecAdd(ak_x, bk_x, hinf_stata_data->x_ctrl_next, HINF_STATE_DIM);
    memcpy(hinf_stata_data->x_ctrl, hinf_stata_data->x_ctrl_next, sizeof(hinf_stata_data->x_ctrl));

    state->y_angle = u;
}

static FrameworkModuleDescriptor g_module_hinf =
{
    "hinf",
    0.005f,
    Priority_Controller_Angle,
    Module_HInf_Update,
    Module_HInf_Reset,
    &g_hinf_stata_data
};

FRAMEWORK_AUTO_REGISTER_MODULE(g_module_hinf)
